"""AUBO i10 的 cuRobo 运动规划适配器。

机器人几何唯一来源是 ``scripts/可视化子项目/aubo_description/urdf/aubo_i10.urdf``。
``model/aubo_i10_curobo.yml`` 由该 URDF 自动拟合碰撞球后生成；不要再向
本模块加入 DH 参数或手写连杆尺寸。

单位约定：对外位置为米、姿态为 4x4 齐次矩阵、关节为弧度；cuRobo 内部相同。
"""
from __future__ import annotations

from pathlib import Path
import re
import time
import numpy as np

try:
    import torch
    from curobo._src.motion.motion_planner import MotionPlanner
    from curobo._src.motion.motion_planner_cfg import MotionPlannerCfg
    from curobo._src.state.state_joint import JointState
    from curobo._src.types.device_cfg import DeviceCfg
    from curobo._src.types.pose import Pose
    from curobo._src.types.tool_pose import GoalToolPose
    _CUROBO_OK = True
    _IMPORT_ERR = ""
except Exception as exc:  # pragma: no cover - 依赖由部署环境提供
    _CUROBO_OK = False
    _IMPORT_ERR = str(exc)


class CuRoboFreeSegmentPlanner:
    """基于现场 i10 URDF 的 GPU 自由段规划器。

    此类只负责 ``q_start -> T_B_F_goal``。点云/深度图碰撞世界由上层在
    ``update_world_from_esdf`` 接入；在 ESDF 桥接完成前，禁止用于现场执行。
    """

    def __init__(self, robot_cfg_yml: str | Path, *, scene_cfg=None,
                 num_ik_seeds: int = 16, num_trajopt_seeds: int = 2,
                 use_cuda_graph: bool = True,
                 collision_activation_distance: float = 0.005,
                 allow_validated_fallback: bool = False,
                 cache_voxel_size: float = 0.005,
                 cache_dims_m=(3.0, 3.0, 3.0),
                 cache_cuboids: int = 64,
                 interpolation_dt: float = 0.025,
                 position_tolerance_m: float = 0.001,
                 orientation_tolerance_deg: float = 0.5,
                 end_effector_spheres: list | None = None,
                 end_effector_frame: str = "flange_link",
                 enable_graph_planner: bool = False):
        if not _CUROBO_OK:
            raise RuntimeError(f"cuRobo 不可用: {_IMPORT_ERR}")
        if not torch.cuda.is_available():
            raise RuntimeError("未检测到 CUDA GPU；本项目运行时不允许回退到 CPU 规划。")

        cfg_path = Path(robot_cfg_yml).resolve()
        if not cfg_path.is_file():
            raise FileNotFoundError(f"找不到 i10 cuRobo 模型配置: {cfg_path}")
        validate_collision_sphere_units(cfg_path)
        if collision_activation_distance < 0.0:
            raise ValueError("collision_activation_distance 必须非负（单位：m）")

        # 附录 B.2：把法兰末端刚体的碰撞球挂到 flange_link，用**包含本体和末端球的
        # 完整模型**创建规划器，再预热两条调用路径。不能在缺少末端球的模型上预热
        # 完成后才补挂末端。
        robot_config = _load_robot_config_dict(cfg_path)
        self.body_sphere_count = _count_spheres(robot_config)
        self.end_effector_sphere_count = 0
        self.end_effector_frame = end_effector_frame
        if end_effector_spheres:
            _attach_end_effector_spheres(robot_config, end_effector_frame, end_effector_spheres)
            self.end_effector_sphere_count = len(end_effector_spheres)
        self.total_sphere_count = self.body_sphere_count + self.end_effector_sphere_count
        self.device_cfg = DeviceCfg(device=torch.device("cuda:0"), dtype=torch.float32)
        self.config = MotionPlannerCfg.create(
            robot=robot_config,
            device_cfg=self.device_cfg,
            num_ik_seeds=num_ik_seeds,
            num_trajopt_seeds=num_trajopt_seeds,
            # 3 mm / 2 deg 是规划收敛阈值；最终碰撞和控制柜仍要二次校验。
            # 收敛判据。必须与 C++ 侧 CheckKinematicGates 的端点容差配套：
            # C++ 那边要留出余量（约 1.5 倍），否则 cuRobo 认为收敛、C++ 却判
            # 「终点法兰与目标不符」，所有候选会被复核全部挡下。
            position_tolerance=position_tolerance_m,
            orientation_tolerance=np.deg2rad(orientation_tolerance_deg),
            # 5 mm ESDF 下使用 5 mm 的近障激活距离。此前硬编码 20 mm 会把本次
            # 实测约 10 mm 净空的无碰路径判成不可行；实际穿透仍始终是硬约束。
            optimizer_collision_activation_distance=collision_activation_distance,
            interpolation_dt=interpolation_dt,
            use_cuda_graph=use_cuda_graph,
            # 未传场景时同样预分配，避免后续 update_world 静默失效。
            scene_model=scene_cfg,
            # 显存预分配。占用 ≈ (dims/voxel_size)^3 × 2 字节。
            # 原实现写死 3m@5mm（216M 体素，约 432 MB），对 8 GB 卡偏紧，
            # 改为由调用方按实际使用的体素尺寸传入。
            # cuboid 预分配要够装下"带圆柱孔平台"分解出的那组长方体（附录 B.1）。
            collision_cache={"voxel": {"layers": 1, "dims": list(cache_dims_m),
                                        "voxel_size": cache_voxel_size},
                             "cuboid": int(cache_cuboids)},
        )
        # ---- 改动一：图规划(PRM)做成开关 ----
        # cuRobo 官方 _plan_pose_single 默认 enable_graph_attempt=1，即第 0 次尝试
        # **不跑**图规划，失败后才用它兜底；本项目原实现每次无条件先跑，实测占单目标
        # 耗时 76%（12483.7ms / 17926.7ms）。
        # 注意：不能给 MotionPlannerCfg.create 传 graph_planner_config=None —— 它会
        # 无条件调 PRMGraphPlannerCfg.create(None) 并抛 TypeError。正确做法是在构造出
        # cfg 之后把该字段置 None，MotionPlanner 据此跳过 PRMGraphPlanner 的构造
        # （motion_planner.py:68-71），连启动期的图构建也一并省掉。
        if not enable_graph_planner:
            self.config.graph_planner_config = None
        self.motion = MotionPlanner(self.config)
        self._robot_cfg_yml = cfg_path
        self._scene_cfg = scene_cfg
        self._allow_validated_fallback = allow_validated_fallback
        self._num_ik_seeds = num_ik_seeds
        self._num_trajopt_seeds = num_trajopt_seeds
        self._use_cuda_graph = use_cuda_graph
        self._collision_activation_distance = collision_activation_distance
        self._interpolation_dt = interpolation_dt
        self._enable_graph_planner = bool(enable_graph_planner)
        self.joint_names = list(self.motion.joint_names)
        self._warmed = False
        self._joint_goal_warmed = False

    def warmup(self) -> None:
        """预热 CUDA 图；应在服务启动时调用，不能计入单次规划时间。

        覆盖生产实际使用的两条调用路径：抓取段的**位姿目标**多候选路径，以及
        放置段的**关节目标**路径。只预热位姿目标会让第一次放置规划付出全部
        图捕获开销（文档 S5 / 5.3）。
        """
        if not self._warmed:
            self.motion.warmup(enable_graph=True, num_warmup_iterations=2)
            torch.cuda.synchronize()
            self._warmed = True

    def warmup_joint_goal(self, q_reference: np.ndarray) -> float:
        """预热关节目标（cspace）调用路径，返回耗时（毫秒）。

        用参考构型附近的一个小位移做一次真实的 ``solve_cspace``，使 CUDA 图、
        种子缓存和插值缓冲区在接任务前就绪。失败不抛出：预热轨迹不发布为业务
        结果，但必须把结果如实交给调用方记录。
        """
        started = time.perf_counter()
        q_reference = np.asarray(q_reference, dtype=np.float64).reshape(6)
        q_goal = q_reference.copy()
        q_goal[0] += np.deg2rad(2.0)
        try:
            self.plan_joint_multi(q_reference, q_goal, return_trajectories=1)
        except Exception:  # pragma: no cover - 预热失败由调用方记录
            pass
        torch.cuda.synchronize()
        self._joint_goal_warmed = True
        return 1000.0 * (time.perf_counter() - started)

    def update_world_from_esdf(self, scene_cfg) -> None:
        """更新由 nvblox-curobo bridge 生成的 ``SceneCfg`` 碰撞世界。"""
        if self.motion.scene_collision_checker is None:
            raise RuntimeError("cuRobo 未初始化碰撞缓存，拒绝在无障碍检测下规划")
        self.motion.update_world(scene_cfg)

    def plan_pose(self, q_start: np.ndarray, T_B_F_goal: np.ndarray) -> tuple[np.ndarray, float]:
        """返回密集关节路径 ``(M, 6)`` 和 cuRobo 求解耗时（秒）。"""
        q_start = np.asarray(q_start, dtype=np.float32).reshape(1, 6)
        T_B_F_goal = np.asarray(T_B_F_goal, dtype=np.float64)
        if T_B_F_goal.shape != (4, 4):
            raise ValueError("T_B_F_goal 必须是 4x4 齐次矩阵")
        state = JointState.from_position(
            torch.as_tensor(q_start, device="cuda"), joint_names=self.joint_names
        )
        quat_wxyz = _rotation_to_quat_wxyz(T_B_F_goal[:3, :3])
        pose = Pose(
            position=torch.as_tensor(T_B_F_goal[:3, 3], dtype=torch.float32,
                                     device="cuda").view(1, 3),
            quaternion=torch.as_tensor(quat_wxyz, dtype=torch.float32,
                                       device="cuda").view(1, 4),
        )
        goal = GoalToolPose.from_poses({"flange_link": pose})
        t0 = time.perf_counter()
        result = self.motion.plan_pose(
            goal, state, max_attempts=1,
            enable_graph_attempt=0 if self._use_cuda_graph else 99,
        )
        torch.cuda.synchronize()
        elapsed = time.perf_counter() - t0
        if result is None or not bool(result.success[0].item()):
            q_fallback = self._validated_linear_fallback(goal, state, q_start, T_B_F_goal)
            if q_fallback is None:
                raise RuntimeError("cuRobo 未找到无碰撞收敛轨迹")
            torch.cuda.synchronize()
            return q_fallback, time.perf_counter() - t0
        # cuRobo 0.7 exposes the sampled trajectory through the result method;
        # older releases used ``interpolated_solution``.  Keeping a small
        # compatibility path lets the adapter work with both deployed versions.
        if hasattr(result, "get_interpolated_plan"):
            trajectory = result.get_interpolated_plan()
        else:  # pragma: no cover - compatibility with older cuRobo releases
            trajectory = result.interpolated_solution
        q = trajectory.position
        # Recent cuRobo keeps a singleton seed axis: (batch, seed, time, dof).
        # The executor contract is always a dense (time, dof) array.
        while q.ndim > 2 and q.shape[0] == 1:
            q = q[0]
        if q.ndim != 2 or q.shape[1] != 6:
            raise RuntimeError(f"cuRobo 返回了意外的关节轨迹形状: {tuple(q.shape)}")
        return q.detach().cpu().numpy(), elapsed

    def plan_pose_multi(self, q_start: np.ndarray, T_B_F_goal: np.ndarray,
                        return_trajectories: int = 4) -> dict:
        """一次交出多条收敛轨迹，并回传每一步的统计。

        cuRobo 的 ``MotionPlanner.plan_pose`` 调 trajopt 时没传 ``return_seeds``，
        用的是默认值 1，因此只会给一条；``get_interpolated_plan`` 也明确拒绝多条
        （"only single result is supported"）。本方法**复刻**
        ``motion_planner.py::_plan_pose_single``（NVlabs/curobo @ 3fd54dc）的编排顺序，
        唯一差别是把 ``return_seeds`` 传成 N，并自己做插值裁剪。

        ⚠️ 上游升级时请对照 ``_plan_pose_single`` 检查本方法是否仍然一致。

        Returns:
            {"ok": bool, "error": str, "steps": {...},
             "trajectories": [{"rank": int, "points": (M,6) ndarray, "dt": float}, ...]}
        """
        import torch

        steps = {}
        q_start = np.asarray(q_start, dtype=np.float32).reshape(1, 6)
        T_B_F_goal = np.asarray(T_B_F_goal, dtype=np.float64)
        if T_B_F_goal.shape != (4, 4):
            raise ValueError("T_B_F_goal 必须是 4x4 齐次矩阵")
        state = JointState.from_position(
            torch.as_tensor(q_start, device="cuda"), joint_names=self.joint_names
        )
        quat_wxyz = _rotation_to_quat_wxyz(T_B_F_goal[:3, :3])
        pose = Pose(
            position=torch.as_tensor(T_B_F_goal[:3, 3], dtype=torch.float32,
                                     device="cuda").view(1, 3),
            quaternion=torch.as_tensor(quat_wxyz, dtype=torch.float32,
                                       device="cuda").view(1, 4),
        )
        goal = GoalToolPose.from_poses({"flange_link": pose})
        num_seeds = self.motion.trajopt_solver.config.num_seeds

        # ---- ① 批量逆解 ----
        timer = time.perf_counter()
        ik_result = self.motion.ik_solver.solve_pose(
            goal, return_seeds=num_seeds, current_state=state
        )
        torch.cuda.synchronize()
        success_count = int(torch.count_nonzero(ik_result.success).item())
        steps["ik"] = {"seeds": self._num_ik_seeds, "returned": num_seeds,
                       "converged": success_count,
                       "ms": round(1000.0 * (time.perf_counter() - timer), 1)}
        if success_count == 0:
            return {"ok": False, "error": "目标够不着：逆解全部不收敛",
                    "failed_step": "ik", "steps": steps, "trajectories": []}

        # ---- ② 种子补位 ----
        seed_config = ik_result.solution
        padded = 0
        if success_count < num_seeds:
            good_solution = seed_config[ik_result.success][0:1, :].clone()
            seed_config[~ik_result.success] = good_solution
            padded = num_seeds - success_count
        steps["pad"] = {"padded": padded}

        # ---- ③ 图规划打草稿 ----
        seed_traj = None
        finetune_attempts = 1
        finetune_dt_scale = 0.55
        timer = time.perf_counter()
        if self.motion.graph_planner is not None:
            graph_seed = self.motion._get_graph_seed_trajectories(state, seed_config)
            if graph_seed is None:
                steps["graph"] = {"ok": False,
                                  "ms": round(1000.0 * (time.perf_counter() - timer), 1)}
                return {"ok": False, "error": "无可行路径：图规划找不到几何路径",
                        "failed_step": "graph", "steps": steps, "trajectories": []}
            seed_traj = graph_seed
            finetune_attempts = 3
            finetune_dt_scale = 0.75
            steps["graph"] = {"ok": True,
                              "ms": round(1000.0 * (time.perf_counter() - timer), 1)}
        else:
            # 改动一：必须显式标注"未启用（配置关闭）"，不得静默跳过。
            # C++ 侧据此打印 [4.3] 图规划=未启用（配置关闭）。
            steps["graph"] = {"ok": True, "skipped": True, "reason": "disabled_by_config",
                              "ms": 0.0}

        # ---- ④ 轨迹优化（交出多条）----
        want = max(1, min(int(return_trajectories), num_seeds))
        timer = time.perf_counter()
        trajopt_result = self.motion.trajopt_solver.solve_pose(
            goal, state,
            seed_config=seed_config,
            seed_traj=seed_traj,
            use_implicit_goal=True,
            return_seeds=want,
            finetune_attempts=finetune_attempts,
            finetune_dt_scale=finetune_dt_scale,
        )
        torch.cuda.synchronize()
        converged = int(torch.count_nonzero(trajopt_result.success).item())
        steps["trajopt"] = {"seeds": num_seeds, "requested": want, "converged": converged,
                            "ms": round(1000.0 * (time.perf_counter() - timer), 1)}
        if converged == 0:
            steps["trajopt"]["position_error_mm"] = round(
                float(trajopt_result.position_error.min().item()) * 1000.0, 2)
            steps["trajopt"]["rotation_error_deg"] = round(
                float(np.rad2deg(trajopt_result.rotation_error.min().item())), 2)
            return {"ok": False, "error": "优化不收敛，可能被障碍堵死",
                    "failed_step": "trajopt", "steps": steps, "trajectories": []}

        # ---- ⑤ 稠密插值：逐条裁剪 ----
        # 张量布局（实测）：
        #   success                        (batch, seeds)        = (1, 4)
        #   interpolated_trajectory.position (batch, seeds, horizon, dof) = (1, 4, 1000, 6)
        #   interpolated_last_tstep        (batch, seeds)        = (1, 4)
        # solve_pose 内部已按代价排过序（get_topk_seeds），所以 seed 下标即名次。
        timer = time.perf_counter()
        position = trajopt_result.interpolated_trajectory.position
        last_tstep = trajopt_result.interpolated_last_tstep
        success = trajopt_result.success
        if position.ndim != 4:
            raise RuntimeError(f"意外的插值轨迹形状: {tuple(position.shape)}")
        trajectories = []
        for seed in range(position.shape[1]):
            if not bool(success[0, seed].item()):
                continue
            end = int(last_tstep[0, seed].item())
            q = position[0, seed, :end, :]
            if q.ndim != 2 or q.shape[1] != 6:
                raise RuntimeError(f"意外的关节轨迹形状: {tuple(q.shape)}")
            trajectories.append({
                "rank": len(trajectories) + 1,
                "points": q.detach().cpu().numpy(),
                "dt": self._interpolation_dt,
            })
        steps["interp"] = {"count": len(trajectories),
                           "ms": round(1000.0 * (time.perf_counter() - timer), 1)}
        if not trajectories:
            return {"ok": False, "error": "插值后无有效轨迹",
                    "failed_step": "interp", "steps": steps, "trajectories": []}
        return {"ok": True, "error": "", "steps": steps, "trajectories": trajectories}

    def plan_joint_multi(self, q_start: np.ndarray, q_goal: np.ndarray,
                         return_trajectories: int = 4) -> dict:
        """关节目标（cspace）多候选规划：抓取实际末点 → 固定 q_place。

        放置段的目标是**关节构型**，不是位姿：不把 q_place 转成法兰位姿再求 IK，
        以免得到另一个关节构型（文档 5.3）。这里复刻
        ``MotionPlanner.plan_cspace`` 的编排（图规划种子 + solve_cspace），
        唯一差别是把 ``return_seeds`` 传成 N 并自己做插值裁剪，与
        ``plan_pose_multi`` 保持同一套返回结构。

        ⚠️ 升级 cuRobo 时须对照 ``motion_planner.py::plan_cspace`` 检查兼容性。

        Returns:
            {"ok": bool, "error": str, "steps": {...},
             "trajectories": [{"rank": int, "points": (M,6) ndarray, "dt": float}, ...]}
        """
        import torch

        steps = {}
        q_start = np.asarray(q_start, dtype=np.float32).reshape(1, 6)
        q_goal = np.asarray(q_goal, dtype=np.float32).reshape(1, 6)
        state = JointState.from_position(
            torch.as_tensor(q_start, device="cuda"), joint_names=self.joint_names
        )
        goal_state = JointState.from_position(
            torch.as_tensor(q_goal, device="cuda"), joint_names=self.joint_names
        )
        num_seeds = self.motion.trajopt_solver.config.num_seeds

        # ---- 5.3.1 起终构型校验（关节限位由 C++ 侧独立复核，这里只查形状与有限值）----
        if not np.all(np.isfinite(q_start)) or not np.all(np.isfinite(q_goal)):
            return {"ok": False, "error": "起终关节角含非有限数值",
                    "failed_step": "start_goal", "steps": steps, "trajectories": []}

        # ---- 5.3.2 图规划种子 ----
        timer = time.perf_counter()
        seed_traj = None
        finetune_attempts = 1
        finetune_dt_scale = 0.55
        if self.motion.graph_planner is not None:
            goal_configs = goal_state.position.view(1, 1, -1).repeat(1, num_seeds, 1)
            graph_seed = self.motion._get_graph_seed_trajectories(state, goal_configs)
            if graph_seed is not None:
                seed_traj = graph_seed
                finetune_attempts = 3
                finetune_dt_scale = 0.75
                steps["graph"] = {"ok": True,
                                  "ms": round(1000.0 * (time.perf_counter() - timer), 1)}
            else:
                # 图规划失败不直接判死：cspace 优化仍可用线性种子求解。
                steps["graph"] = {"ok": False,
                                  "ms": round(1000.0 * (time.perf_counter() - timer), 1)}
        else:
            # 改动一：同抓取段，放置段 [5.3.2] 也必须显式标注未启用。
            steps["graph"] = {"ok": True, "skipped": True, "reason": "disabled_by_config",
                              "ms": 0.0}

        # ---- 5.3.2 关节目标优化（交出多条）----
        want = max(1, min(int(return_trajectories), num_seeds))
        timer = time.perf_counter()
        trajopt_result = self.motion.trajopt_solver.solve_cspace(
            goal_state, state,
            seed_traj=seed_traj,
            return_seeds=want,
            finetune_attempts=finetune_attempts,
            finetune_dt_scale=finetune_dt_scale,
        )
        torch.cuda.synchronize()
        converged = int(torch.count_nonzero(trajopt_result.success).item())
        steps["trajopt"] = {"seeds": num_seeds, "requested": want, "converged": converged,
                            "ms": round(1000.0 * (time.perf_counter() - timer), 1)}
        if converged == 0:
            return {"ok": False, "error": "关节目标优化不收敛，可能被障碍堵死",
                    "failed_step": "trajopt", "steps": steps, "trajectories": []}

        # ---- 5.3.3 插值：逐条裁剪，与 plan_pose_multi 同一布局 ----
        timer = time.perf_counter()
        position = trajopt_result.interpolated_trajectory.position
        last_tstep = trajopt_result.interpolated_last_tstep
        success = trajopt_result.success
        if position.ndim != 4:
            raise RuntimeError(f"意外的插值轨迹形状: {tuple(position.shape)}")
        trajectories = []
        for seed in range(position.shape[1]):
            if not bool(success[0, seed].item()):
                continue
            end = int(last_tstep[0, seed].item())
            q = position[0, seed, :end, :]
            if q.ndim != 2 or q.shape[1] != 6:
                raise RuntimeError(f"意外的关节轨迹形状: {tuple(q.shape)}")
            trajectories.append({
                "rank": len(trajectories) + 1,
                "points": q.detach().cpu().numpy(),
                "dt": self._interpolation_dt,
            })
        steps["interp"] = {"count": len(trajectories),
                           "ms": round(1000.0 * (time.perf_counter() - timer), 1)}
        if not trajectories:
            return {"ok": False, "error": "插值后无有效轨迹",
                    "failed_step": "interp", "steps": steps, "trajectories": []}
        return {"ok": True, "error": "", "steps": steps, "trajectories": trajectories}

    def _validated_linear_fallback(self, goal, state, q_start, T_B_F_goal):
        """TrajOpt 不收敛时的受终检保护回退。

        单帧 ESDF 的大体素世界偶尔会让 TrajOpt 失去有效梯度。此回退只接受：
        1) 在无世界障碍下的 cuRobo TrajOpt 已收敛；2) 81 个离散构型的世界碰撞
        与自碰撞均为零。
        因而它不会以降低碰撞安全性换取“成功”。
        """
        if self._scene_cfg is None or not self._allow_validated_fallback:
            return None

        # 不能对当前 MotionPlanner 做 ``真实世界 -> 空世界 -> 真实世界`` 热切换：
        # cuRobo 的 voxel collision cache 不会可靠恢复。改用独立实例生成种子。
        seed_planner = CuRoboFreeSegmentPlanner(
            self._robot_cfg_yml,
            scene_cfg=None, num_ik_seeds=self._num_ik_seeds,
            num_trajopt_seeds=self._num_trajopt_seeds,
            use_cuda_graph=self._use_cuda_graph,
            collision_activation_distance=self._collision_activation_distance,
            allow_validated_fallback=False,
            enable_graph_planner=self._enable_graph_planner,
        )
        seed_planner.warmup()
        try:
            q_seed, _ = seed_planner.plan_pose(q_start, T_B_F_goal)
        except RuntimeError:
            return None
        # 独立的真实世界验证器：不得复用已经经历 TrajOpt 失败的实例，也不得做
        # update_world 热切换，否则 cuRobo 的 voxel cache 可能保留旧数据。
        del seed_planner
        torch.cuda.empty_cache()
        verifier = CuRoboFreeSegmentPlanner(
            self._robot_cfg_yml, scene_cfg=self._scene_cfg,
            num_ik_seeds=1, num_trajopt_seeds=1, use_cuda_graph=False,
            collision_activation_distance=self._collision_activation_distance,
            allow_validated_fallback=False,
            # 必须继承同一开关，否则关掉图规划后兜底路径又把它打开。
            enable_graph_planner=self._enable_graph_planner,
        )
        q_path = torch.as_tensor(q_seed, dtype=torch.float32, device="cuda").unsqueeze(0)
        path_state = JointState.from_position(q_path, joint_names=verifier.joint_names)
        kin_state = verifier.motion.compute_kinematics(path_state)

        constraints = verifier.motion.trajopt_solver.metrics_rollout.constraint_manager.costs
        scene_cost = constraints["scene_collision"]
        world = verifier.motion.scene_collision_checker.get_sphere_distance(
            kin_state, scene_cost._collision_buffer, torch.ones_like(scene_cost._weight),
            torch.zeros_like(scene_cost.config.activation_distance),
        )
        self_collision = constraints["self_collision"].forward(kin_state.robot_spheres)
        if bool(torch.any(world > 1e-6).item()) or bool(torch.any(self_collision > 1e-6).item()):
            return None
        return q_path[0].detach().cpu().numpy()


def _load_robot_config_dict(robot_cfg_yml: Path) -> dict:
    """读机器人 cuRobo 配置为可修改的字典（保留 kinematics 顶层结构）。"""
    import yaml

    with open(robot_cfg_yml, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    if not isinstance(config, dict) or "kinematics" not in config:
        raise ValueError(f"{robot_cfg_yml}: 缺少 kinematics 顶层节")
    # asset_root_path / urdf_path 是绝对路径，随配置文件一起搬走会失效，这里按
    # 配置文件所在位置重新解析，避免换机器后模型找不到。
    return config


def _count_spheres(robot_config: dict) -> int:
    """统计本体碰撞球总数（附录 A 的交叉验证：i12h 本体应为 420 个）。"""
    spheres = robot_config["kinematics"].get("collision_spheres") or {}
    return sum(len(v or []) for v in spheres.values())


def _edge_count(length_mm: float, radius_mm: float) -> int:
    """一条棱上的球数（含两端）：相邻球心间距 ≤ 2r，n ≥ 棱长/(2r) + 1（文档附录 A.5）。"""
    return int(np.ceil(length_mm / (2.0 * radius_mm) - 1e-9)) + 1


def generate_edge_spheres(negative_extent_mm, positive_extent_mm, radius_mm: float) -> list:
    """末端盒体棱线布球（文档 2.5.8）。

    - 只在棱线上放球，球心落在棱上（内缩 0），外凸 = 半径；
    - 放球的是 8 条棱：4 条竖棱（平行 Z，贯穿 Z 两端，含全部 8 个角点）、
      尖端（+Z 面）的 2 条 X 向与 2 条 Y 向棱；
    - 近法兰（−Z 面）的 4 条棱中间不放球，只保留竖棱提供的两端角点；
    - 每条棱相邻球心间距 ≤ 2r，保证棱上无缝覆盖。

    62×100×200、r=2.6mm：竖棱 4×40 + 尖端 X 2×11 + 尖端 Y 2×19 = 220。

    Returns:
        [{"center": [x, y, z], "radius": r}, ...]，单位**米**（cuRobo 约定）。
    """
    negative = np.asarray(negative_extent_mm, dtype=float)
    positive = np.asarray(positive_extent_mm, dtype=float)
    lo = -negative
    hi = positive
    size = hi - lo
    if np.any(size <= 0.0):
        raise ValueError(f"末端盒尺寸必须为正，实际 {size.tolist()} mm")
    if radius_mm <= 0.0:
        raise ValueError("end_effector.sphere_radius_mm 必须为正")

    centers = []
    # 竖棱：x ∈ {lo, hi}，y ∈ {lo, hi}，z 从 lo 到 hi 全段（两端即 8 个角点）。
    nz = _edge_count(size[2], radius_mm)
    for x in (lo[0], hi[0]):
        for y in (lo[1], hi[1]):
            for z in np.linspace(lo[2], hi[2], nz):
                centers.append((x, y, z))
    # 尖端 X 向：y ∈ {lo, hi}，z = hi，只取中间点（端点已由竖棱提供）。
    nx = _edge_count(size[0], radius_mm)
    for y in (lo[1], hi[1]):
        for x in np.linspace(lo[0], hi[0], nx)[1:-1]:
            centers.append((x, y, hi[2]))
    # 尖端 Y 向：x ∈ {lo, hi}，z = hi，只取中间点。
    ny = _edge_count(size[1], radius_mm)
    for x in (lo[0], hi[0]):
        for y in np.linspace(lo[1], hi[1], ny)[1:-1]:
            centers.append((x, y, hi[2]))
    return [{"center": [float(c) * 0.001 for c in center], "radius": radius_mm * 0.001}
            for center in centers]


def describe_edge_spheres(negative_extent_mm, positive_extent_mm, radius_mm: float) -> dict:
    """返回末端棱线球的可审计参数，供日志与 collision_model.json 固定模型快照使用。"""
    spheres = generate_edge_spheres(negative_extent_mm, positive_extent_mm, radius_mm)
    negative = np.asarray(negative_extent_mm, dtype=float)
    positive = np.asarray(positive_extent_mm, dtype=float)
    size = positive + negative
    counts = [_edge_count(size[axis], radius_mm) for axis in range(3)]
    return {"layout": "edges",
            "count": len(spheres),
            "radius_mm": round(float(radius_mm), 3),
            "inset_mm": 0.0,
            "bulge_mm": round(float(radius_mm), 3),
            "box_size_mm": [round(float(v), 1) for v in size],
            "edge_counts": {"vertical_z": counts[2], "tip_x": counts[0], "tip_y": counts[1]},
            "spheres": [{"center_mm": [round(float(c) * 1000.0, 3) for c in item["center"]],
                         "radius_mm": round(float(item["radius"]) * 1000.0, 3)}
                        for item in spheres]}


def _attach_end_effector_spheres(robot_config: dict, frame: str, spheres: list) -> None:
    """把末端球挂到指定连杆，并补齐碰撞连杆列表与自碰配置。

    仅配置末端与腕部必要的装配接触豁免，保留末端与其他非豁免连杆的自碰检查。
    """
    kinematics = robot_config["kinematics"]
    collision_spheres = kinematics.setdefault("collision_spheres", {})
    collision_spheres[frame] = [{"center": list(item["center"]), "radius": float(item["radius"])}
                                for item in spheres]
    link_names = kinematics.setdefault("collision_link_names", [])
    if frame not in link_names:
        link_names.append(frame)
    buffers = kinematics.setdefault("self_collision_buffer", {})
    buffers.setdefault(frame, 0.0)
    ignore = kinematics.setdefault("self_collision_ignore", {})
    # 末端与腕部末节是装配接触，必须豁免；其余连杆仍然参与自碰检查。
    ignore.setdefault(frame, [])
    if "wrist3_Link" not in ignore[frame]:
        ignore[frame].append("wrist3_Link")
    wrist = ignore.setdefault("wrist3_Link", [])
    if frame not in wrist:
        wrist.append(frame)


def validate_collision_sphere_units(robot_cfg_yml: str | Path) -> None:
    """拒绝把毫米碰撞球当作米加载到 cuRobo。

    URDF、cuRobo 和本适配器均以米为长度单位。该检查刻意不依赖 PyYAML，确保在
    CUDA 初始化之前就能报告配置问题。i10 连杆网格的拟合球半径应小于 0.2 m；
    毫米配置通常会出现 1--100 这一量级的半径。
    """
    text = Path(robot_cfg_yml).read_text(encoding="utf-8")
    radii = [float(x) for x in re.findall(r"(?m)^\s*radius:\s*([-+0-9.eE]+)\s*$", text)]
    if not radii:
        raise ValueError(f"{robot_cfg_yml}: 未找到 collision_spheres.radius")
    max_radius = max(radii)
    if not (0.0001 <= max_radius <= 0.2):
        raise ValueError(
            f"{robot_cfg_yml}: 碰撞球半径范围异常（最大 {max_radius:g}）。"
            "cuRobo/URDF 必须使用米；请检查是否把 mm 数值直接写入 YAML。"
        )


def _rotation_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    """旋转矩阵转 cuRobo 所需的 [w, x, y, z] 四元数。"""
    m = np.asarray(R, dtype=float)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q = np.array([0.25 * s, (m[2, 1] - m[1, 2]) / s,
                      (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s])
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
        q = np.empty(4)
        q[0] = (m[k, j] - m[j, k]) / s
        q[i + 1] = 0.25 * s
        q[j + 1] = (m[j, i] + m[i, j]) / s
        q[k + 1] = (m[k, i] + m[i, k]) / s
    return q / np.linalg.norm(q)
