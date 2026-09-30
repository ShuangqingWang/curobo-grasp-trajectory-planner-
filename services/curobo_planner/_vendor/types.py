"""grasp_planner.types — 对外接口数据结构（对应技术文档 §2.3 输入/输出清单）

所有位姿均为 4x4 齐次矩阵，基座坐标系 {B}；关节量为弧度 np.ndarray(6,)。
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional
import numpy as np


# ----------------------------- 输入 -----------------------------

@dataclass
class GraspCandidate:
    """一个候选抓取（感知模块输出，已换算为法兰位姿）。"""
    T_B_F_grasp: np.ndarray          # 抓取时刻法兰位姿 (4x4, {B})
    score: float = 1.0               # 感知侧评分 s_i ∈ [0,1]，越大越好
    workpiece_id: int = -1           # 对应工件索引


@dataclass
class PlanRequest:
    """单个抓取周期的规划请求（技术文档 §2.3 输入表）。"""
    q_start: np.ndarray                       # ① 拍照点关节角 (6,)
    grasp_candidates: list[GraspCandidate]    # ③④ 候选抓取法兰位姿 + 评分
    cloud_B: np.ndarray                       # ⑤ 全景环境点云 (N,3) {B}，障碍唯一权威来源
    T_B_O_target: np.ndarray                  # ⑥ 目标工件 6D 位姿 (4x4)
    target_extents: np.ndarray                # ⑦ 目标工件外形（Box 半边长 (3,)，{O} 系）
    T_B_F_place: Optional[np.ndarray] = None  # ⑧ 放置时刻法兰位姿 (4x4, {B})
    other_workpiece_poses: list[np.ndarray] = field(default_factory=list)  # 其余工件位姿（仅诊断用；碰撞由点云覆盖）
    return_to_start: bool = True              # 撤离后是否规划回拍照点
    base_platform_min_B: Optional[np.ndarray] = None  # AGV 轴对齐包围盒最小角 (3,)，{B}，m
    base_platform_max_B: Optional[np.ndarray] = None  # AGV 轴对齐包围盒最大角 (3,)，{B}，m
    bin_roi_center_B: Optional[np.ndarray] = None  # 可选箱区 ROI；不传则完全按点云建障碍
    bin_roi_half_extents: Optional[np.ndarray] = None


# ----------------------------- 输出 -----------------------------

@dataclass
class JointTrajectory:
    """时间参数化后的关节轨迹（一段）。"""
    name: str                                 # 如 "photo_to_grasp" / "grasp_to_place"
    t: np.ndarray                             # (M,) 秒，段内从 0 起
    q: np.ndarray                             # (M,6)
    qd: np.ndarray                            # (M,6)
    qdd: np.ndarray                           # (M,6)
    attached_object: bool = False             # 该段末端是否附着工件（执行侧据此校验负载）


@dataclass
class PlanDiagnostics:
    """规划诊断信息（技术文档 §2.3 输出表 ③）。"""
    success: bool = False
    failure_reason: str = ""
    chosen_candidate: int = -1                # 选中的抓取候选索引 i*
    chosen_ik_branch: int = -1
    n_candidates_tried: int = 0
    fallback_level: int = 0                   # 0=线性初值直接成功
    min_clearance_m: float = np.inf           # 全程最小障碍间隙（终检口径）
    timings_ms: dict = field(default_factory=dict)   # 各阶段耗时分解
    optimizer_iters: int = 0
    total_exec_time_s: float = 0.0            # 轨迹总执行时长（评价指标①）
    joint_path_length: float = 0.0            # 关节路径长度（评价指标②）


@dataclass
class PlanResult:
    """规划结果：分段轨迹 + 诊断。segments 按执行顺序排列。"""
    segments: list[JointTrajectory] = field(default_factory=list)
    diagnostics: PlanDiagnostics = field(default_factory=PlanDiagnostics)
    # 可视化数据（技术文档 §2.3 输出表 ④）：由 viz.snapshot() 序列化


# ----------------------------- 配置 -----------------------------

@dataclass
class PlannerConfig:
    """全部可调参数（对应文档 §7.6 分区裕度、§8 优化器、§9 限幅）。生产环境从 yaml 加载。"""
    # 分区安全裕度 [m]（§7.6）
    d_safe_free: float = 0.030
    d_hard_free: float = 0.010
    d_safe_approach: float = 0.010
    d_hard_approach: float = 0.003
    d_safe_retreat: float = 0.015
    d_hard_retreat: float = 0.005
    d_safe_self: float = 0.010
    d_hard_self: float = 0.005
    # 目标工件挖除膨胀 [m]（§7.2.2）
    target_dilation: float = 0.008
    robot_filter_dilation: float = 0.020
    # ESDF（§7.2.1）
    voxel_size: float = 0.005
    # 后端：gpu 为生产默认，直接使用现有 CUDA；cpu 仅保留作回归验证。
    planner_backend: str = "gpu"
    # 接近/撤离（§4.3, §5）
    d_approach_levels: tuple = (0.06, 0.09, 0.12)
    d_retreat: float = 0.12
    retreat_dir_B: tuple = (0.0, 0.0, 1.0)    # 撤离方向（箱口法向，{B}）
    cartesian_step: float = 0.002             # 直线段插值步长 [m]
    branch_jump_limit: float = 0.35           # 直线段相邻点 ‖Δq‖∞ 上限 [rad]
    # IK 过滤（§6.3）
    joint_limit_margin: float = 0.06          # rad
    manipulability_min: float = 0.015
    # 目标评分权重（§6.4）
    w_joint_dist: float = 1.0
    w_manip: float = 0.05
    w_clearance: float = 0.3
    w_grasp_score: float = 0.5
    w_limit: float = 0.2
    # 优化器（§8）
    n_waypoints: int = 24                     # 自由段路点数 N
    max_candidates_try: int = 6               # P：逐个尝试的候选上限（CPU 参考实现）
    collision_sample_dq: float = 0.05          # 碰撞采样细分步长 [rad]（§7.5-1 防穿隧）
    penalty_init: float = 10.0
    penalty_scale: float = 10.0
    penalty_rounds: int = 4
    lbfgs_maxiter: int = 120
    gpu_maxiter: int = 48                 # GPU 批量投影梯度每轮最大迭代数
    # 时间参数化（§9.1，规划上限=额定的比例见文档）
    qd_max: np.ndarray = field(default_factory=lambda: np.full(6, 1.8))    # rad/s
    qdd_max: np.ndarray = field(default_factory=lambda: np.full(6, 4.0))   # rad/s^2
    resample_dq: float = np.deg2rad(1.0)      # 终检/参数化重采样步长（§7.5）
    # 关节限位（AUBO i 系列典型 ±175°，按机型核对）
    q_min: np.ndarray = field(default_factory=lambda: np.full(6, -3.05))
    q_max: np.ndarray = field(default_factory=lambda: np.full(6, 3.05))
