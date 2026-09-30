# grasp_planner_pkg — AUBO-i12H 实时抓取轨迹规划与仿真

本项目把“抓取点生成”和“轨迹规划/碰撞检测”合并为一个两阶段常驻服务。
它根据基座系工件位姿和主/左/右三张深度图，为现场 AUBO-i12H 六轴机械臂
生成两段经过完整碰撞终检的标准关节轨迹：

1. 拍照位 → 抓取位；
2. 抓取位 → 放置位。

轨迹碰撞模型只包含机械臂本体、配置法兰末端、三相机深度环境和 AGV
平台；即使现场第二段已经夹住工件，规划器也不建立、不附着、不检查工件
碰撞体。两段都仍执行自碰撞、关节限位、端点精度和动作质量硬门禁。

> **重构状态（2026-09-12）**：本项目已按 `docs/轨迹规划逻辑分析.md` 重构完成。
> **该文档是唯一权威规格**；本 README 的第 3 节之后仍有部分章节描述的是重构前的旧实现
> （`best_grasp.json`、`trigger_0001_` 归档目录、`/var/hybrid_log/` 路径、
> J1~J5 累计转角门禁等），与当前代码不一致，阅读时以本节和该文档为准。

## 0. 当前实现速览（重构后）

| 事项 | 当前实现 |
| --- | --- |
| 外部触发协议 | 本地/远程 TCP，24 字节固定报头 `GP02` + `request.txt` 原文 + 主/左/右三张 TIFF 与主 RGB PNG 原始字节 |
| 应答 | 每行一个 JSON：先 `accepted`／`rejected`／`busy`，任务结束后再 `completed`／`failed` |
| 并发 | 单任务、不排队；执行或归档期间的新连接返回 `busy`，不创建新代次、不改动 `output/` |
| 会话与归档根 | **`/var/hybird_log/grasp_log/`**（注意是 `hybird_log`，与旧实现的 `hybrid_log` 拼写不同） |
| 归档结构 | `<会话>/service.log`、`session.json`，每轮 `<触发>/{request.log,result.json,input/,output/}` |
| flow grasp | `load_request → load_grasp_labels → compose_grasp_poses → write_grasp_output` |
| flow trajectory | `load_grasp_candidates → fuse_base_cloud → build_collision_world → plan_full_cycle → write_trajectory_output` |
| 抓取段目标 | 位姿目标（法兰 4×4），起点固定 `q_photo` |
| 放置段目标 | **关节目标**（`q_grasp → q_place`），不把 `q_place` 转位姿再求 IK |
| 评级指标 | 路径倍率 ≤1.5/≤2.0；抬升 ≤5mm/≤10mm；**J1~J6 单轴净转角** ≤180°/≤360° |
| 端点容差 | 统一 **1.0 mm / 0.2°**，GPU 收敛判据与 CPU 复核同值；放置段另有关节端点容差 |
| 末端碰撞 | 100×70×200 mm 末端盒生成碰撞球挂到 `flange_link`，抓取与放置两段都检查沿途碰撞 |
| 基座平台 | 安装处扣除直径 300 mm、深度 50 mm 圆柱豁免区；端点预筛与 GPU 使用同一份带孔几何 |
| 可视化 | 第三流程 `visualization`：默认相对目录 `output/trajectory_planning`，500 ms 自动跟随并回放 |

### 首次部署需要创建日志根目录

会话日志根目录无法创建时服务**不会宣布就绪**，这是刻意设计。首次部署执行一次：

```bash
sudo mkdir -p /var/hybird_log/grasp_log
sudo chown -R "$(id -u):$(id -g)" /var/hybird_log
```

### 启动顺序

```bash
# 1) 编译
bash scripts/build_tar.sh

# 2) 规划服务（自动启动并预热 cuRobo GPU 子服务，全部完成后才监听 9108）
./build/OpenmindTrajectoryPlan config/grasp_planner.json

# 3) 发一次触发（读 input/ 的文字、三张深度图与主 RGB 图）
./build/OpenmindTrajectoryTrigger --input-dir input

# 4) 可视化（可选，独立启停，不影响规划与归档）
bash scripts/simulation/serve_noetic_demo.sh config/grasp_planner.json
```


正式链路使用常驻 GPU 服务，启动时一次性加载 cuRobo、nvblox、CUDA 上下文、AUBO-i12H 标定模型和碰撞球。生产侧每个抓取周期只需原子更新输入并发送轻量触发，不需要重复加载 GPU 环境。

机器人型号以控制器 `RobotConfig.getRobotType() = aubo_i12H` 为准。运动学使用这台实机控制器返回的出厂补偿 MDH，几何使用 AUBO 官方 i12H STEP；旧 i10 模型和旧输出不得用于本机执行。

规划服务只生成和验证轨迹，不控制夹爪，也不会自动连接或移动真实机械臂。

## 1. 当前项目结构

```text
grasp_planner_pkg/
├── src/                     # C++ 规划服务 + GPU worker 用的 Python 适配
├── model/                   # AUBO-i12H 标定与 cuRobo 碰撞球
├── config/grasp_planner.json
├── input/                   # request.txt + 三张深度图
├── output/
├── src/grasp_planner/       # C++ 进程内调用的 cuRobo/nvblox
├── scripts/simulation/      # 浏览器回放
├── scripts/轨迹运行/        # 真机播放
├── tests/
├── third_party/
└── build/
```

生产输入使用项目根目录 `input/`，本次最新输出仍使用项目根目录
`output/`；每次触发的历史副本另外归档在
`/var/hybrid_log/grasp_planner_pkg/`。不再保留 `runtime_io` 旧路径。

其他两个目录不是业务输入输出：

- `tests/`：自动回归和验收脚本，用于防止运动学、碰撞模型、配置及输出接口回退；
- `build/runtime/`：当前服务状态 `realtime_service_status.json` 和单实例锁；
  触发、状态查询与停止命令统一读取这里。
- `build/cache/`：服务根据模型与法兰末端配置生成的 cuRobo 临时模型，以及
  `grasp_candidate_preferences.json` 抓取候选偏好缓存。
- `build/reports/`、`build/diagnostics/`：连续回归和离线诊断产物，不会混入
  正式 `output/`。整个 `build/` 已被 Git 忽略，停止服务后可删除并自动重建。

## 2. 实时服务快速使用

进入项目目录：

```bash
cd /home/vecow/workspace/轨迹规划/grasp_planner_pkg
```

### 2.1 启动并预热 GPU 服务

先编译，再启动 C++ 常驻进程：

```bash
bash scripts/build_tar.sh
./build/OpenmindTrajectoryPlan config/grasp_planner.json
```

这是唯一的正式启动入口。进程按配置启动 cuRobo/nvblox GPU 子服务、预热后监听
`0.0.0.0:9108`。`Ctrl+C` 停止。不要同时起两份。

### 2.2 发布一次输入

正式输入位于 `input/`：

- `input/request.txt`：本次三张深度图与主 RGB 图路径、工件代号和工件位姿；
- `input/raw_depth.tiff`：主相机单通道 `float32 TIFF` 深度图，像素单位 mm；
- `input/raw_depth_left.tiff`：左相机同步深度图；
- `input/raw_depth_right.tiff`：右相机同步深度图；
- `input/raw_color.png`：主相机 RGB 图；
生产者准备好 `request.txt`、三张 TIFF 与主 RGB 图后，在终端 2 用 TCP 触发：

```bash
./build/OpenmindTrajectoryTrigger --input-dir input
```

默认读 `input/request.txt`、三张 `input/raw_depth*.tiff` 与 `input/raw_color.png`，发到 `127.0.0.1:9108`。
离线单次测试也可以：

```bash
./build/OpenmindTrajectoryPlan config/grasp_planner.json --once --input-dir input
```

### 2.3 停止服务

在运行服务的终端按 `Ctrl+C`。

## 3. 输入说明

`input/request.txt` 使用 `key = value` 格式，只保留每次采集/抓取会变化的数据。距离、平移和深度使用 mm，关节角和姿态角使用 degree；程序内部统一转换为 m 和 rad。

动态字段如下：

| 字段 | 含义 |
|---|---|
| `depth_main_path` | 主相机 TIFF，相对于 `request.txt` 的路径 |
| `depth_left_path` | 左相机同步 TIFF 路径 |
| `depth_right_path` | 右相机同步 TIFF 路径 |
| `workpiece_type` | 行为树下发的固定工件代号；必须同时存在于两份 config 的工件目录中 |
| `T_B_O_target` | 目标工件在基座坐标系下的 4×4 齐次位姿矩阵，平移列单位 mm |

外部输入不再包含 `grasp_candidate_*`。每次触发先由 `src/grasp_tool` 根据
原始工件位姿 `T_B_O` 先生成偏移前的原始抓取中心
`T_B_TCP_raw` 和对应 `T_B_F_raw`，再按
`camera_clearance_along_tcp_z_mm` 沿该TCP局部Z轴平移，得到新
`T_B_TCP` 和对应新 `T_B_F`。只有偏移后的 `T_B_TCP/T_B_F`
会写入内部 `input/.trajectory_request.txt`；轨迹规划器把新
`T_B_F` 作为抓取终点。原始两个位姿只用于输出、日志和可视化。

### 3.1 两阶段数据接口

只有 `input/request.txt` 是上层系统维护的文本输入。三张深度图文件名在同一
设备上保持不变，只原子替换图像内容；文本中逐次变化的核心值是
`T_B_O_target`，切换工件型号时同时更新 `workpiece_type`。

| 阶段 | 输入 | 输出 |
|---|---|---|
| 阶段1：抓取点生成 | 原始 `T_B_O`、`workpiece_type`、主相机深度图和两份 config | 偏移前 `T_B_TCP_raw/T_B_F_raw`、偏移后规划 `T_B_TCP/T_B_F`、夹爪开口 |
| 阶段2：轨迹规划 | 原始 `T_B_O` + 阶段1生成的 `T_B_TCP` + `T_B_F` + 三相机深度场 | 两份标准关节路径、manifest、碰撞报告、基座系点云和可视化 |

阶段1会按 NPZ 分数生成多组候选。生产默认使用
`trajectory_selection.evaluate_all_grasp_candidates: false` 的实时质量优先
模式：先用接近方向、真实数据验证优先级、同工件历史安全分支、离线分数、
接近倾角和相对拍照位姿态偏差排列候选；每个可能发布的候选再执行完整
cuRobo IK、两段路径规划和稠密
碰撞终检。取得第一条 A/B 级生产可接受完整周期立即发布；C 级才在
`max_full_cycle_evaluations` 与 `max_selection_ms` 限制为了比较更优候选而
继续花费的时间，但 C 级只保留用于诊断，绝不正式发布。尚未找到安全解时
还受 `max_no_solution_search_ms` 和 `max_no_solution_candidates` 的有界搜索
保护，避免真实无解输入卡住服务。设置为 `true` 时进入离线全量模式，忽略早停预算并穷举
所有候选/允许的 IK 分支，用于标定和回归，不用于生产节拍。

机器人本体、平台、三相机ESDF、法兰末端、限位、端点精度和接近整圈的冗余关节转动
始终是最终发布轨迹的硬门禁，不能通过评分、缓存或早停抵消。实时早停减少
的是“已经有安全合格解后继续比较其他候选”的计算，不减少所选轨迹的检查。

通过硬门禁的完整周期按以下顺序选择：

1. A/B/C 轨迹等级；A级是不额外飞高、接近直达且无大幅腕部动作的理想路线，
   B级允许必要的小范围绕障，C级仅用于离线诊断；
2. 完整周期总分：抓取质量20%、拍照→抓取35%、抓取→放置35%、两段连续性10%；
3. 最大单轴累计转角、腕部累计转角、TCP总路径长度；
4. 实时模式在昂贵规划前优先验证同工件最近一次安全候选；离线全量模式中，
   历史候选仍只用于完全并列时保持分支稳定。

每个精确抓取IK分支仍采用两段式结构化路径：不再为了提前换姿态而先到一个
会造成腕部过冲的冗余中间构型；最终接近段保持法兰姿态不变，沿TCP局部+Z轴
直线进入。任一关节累计转动达到配置的 `330°` 会作为冗余整圈转动直接拒绝。
放置路线使用严格等级：安全直达路线支配回拍照位路线，安全回拍照位路线支配
高位保底路线；同一级的多条高位路线会全部生成后统一评分。

最终实际执行的抓取结果位于 `output/grasp_generation/best_grasp.json`，
版本为 `grasp_generation_stage1/v1`。它不是固定等于离线评分第1名，而是
与两份正式轨迹中实际选中的抓取终点一致，包含以下字段：

```text
workpiece_pose_T_B_O_matrix_mm
workpiece_pose_xyzrxryrz_mm_deg
original_grasp_center_tcp_pose_T_B_TCP_matrix_mm
original_grasp_center_tcp_pose_xyzrxryrz_mm_deg
original_grasp_center_flange_pose_T_B_F_matrix_mm
original_grasp_center_flange_pose_xyzrxryrz_mm_deg
camera_clearance_along_tcp_z_mm
grasp_point_tcp_pose_T_B_TCP_matrix_mm
grasp_point_tcp_pose_xyzrxryrz_mm_deg
grasp_flange_pose_T_B_F_matrix_mm
grasp_flange_pose_xyzrxryrz_mm_deg
gripper_opening_m
selected_candidate_index
selected_candidate_rank
selection_source
```

阶段1在约百毫秒内先生成 `grasp_results.json/.csv` 和临时候选第1名；
这时只能认为“抓取候选集合就绪”。`best_grasp.json` 只有在阶段2完整门禁
选中实际执行候选后才成为稳定最终结果。执行侧应以
`trajectory_manifest.json` 的原子发布为准，同时读取最终
`best_grasp.json` 和两份轨迹，不能把阶段1临时候选当作实机目标。

服务将这三个关键位姿组合为内部 `input/.trajectory_request.txt`，并把同一
内容复制为 `output/trajectory_planning/request_with_generated_grasp.txt`。
后者是两阶段之间可供检查、归档和复现的接口快照；轨迹规划器使用其中的
全部 `grasp_candidate_0..N` 做完整安全筛选，但不会反向修改原始
`request.txt`。偏移前的 `T_B_TCP_raw/T_B_F_raw` 仅用于诊断，不会成为
任何轨迹规划目标。

正式固定配置是 `config/grasp_planner.json`，由 `config/base/` 拆开：

- `app.json`：拍照/放置工位、TCP 标定、末端矩形、基座箱体；
- `camera.json`：主/左/右内参、深度范围、`T_cam2flange`；
- `device/aubo.json`：机型运动学；
- `pipeline/flow/grasp.yaml`、`trajectory.yaml`：两步算法参数。

`request.txt` 只提供本次工件代号和 `T_B_O_target`。

坐标系约定：`{B}` 为机械臂基座、`{C_main/C_left/C_right}` 为三个深度相机、
`{O}` 为目标工件、`{TCP}` 为抓取点/夹爪TCP、`{F}` 为机器人法兰。
`T_B_TCP` 与 `T_B_F` 通过 `config/grasp_generator.yaml` 中固定的夹爪安装标定
对应；阶段2保存二者用于追溯，但机器人抓取终点以 `T_B_F` 为准。程序对每台
相机计算：

```text
T_B_C_<name> = T_B_F_photo × T_cam2flange_<name>
```

三张 TIFF 在 CPU 并行解码，分别在 GPU 上执行精确工件 OBB 挖除，然后融合到同一个 nvblox Mapper。每次场景只调用一次 `update_esdf()`、只生成一个 cuRobo VoxelGrid。

拍照关节角与拍照法兰位姿必须来自同一个实机工位；加载 config 时程序会用 i12H 标定 MDH 做 FK 交叉校验，位置误差超过 `0.1 mm` 或姿态误差超过 `0.01°` 会拒绝启动。点云外参使用这个固定法兰位姿计算。`fixed_stations.place` 为当前固定放置工位；修改任一固定工位后，必须重启服务并重新执行碰撞和可视化验证。

## 4. 标准输出

全部历史输出位于 `/var/hybrid_log/grasp_planner_pkg/`，结构固定为：

```text
/var/hybrid_log/grasp_planner_pkg/
├── current -> grasp_log_当前服务启动时间
└── grasp_log_YYYYMMDD_HHMMSS_ffffff/
    ├── service.log
    ├── service_info.json
    ├── realtime_service_status.json
    ├── latest -> trigger_最近一次序号_触发时间
    ├── trigger_0001_YYYYMMDD_HHMMSS_ffffff/
    │   ├── grasp_generation/
    │   └── trajectory_planning/
    ├── trigger_0002_YYYYMMDD_HHMMSS_ffffff/
    └── ...
```

`service_info.json` 记录服务启动时间、PID、项目/config/model/输入路径和预热
耗时；项目内唯一稳定的实时状态位于
`build/runtime/realtime_service_status.json`，服务日志批次中的同名文件是历史
状态副本。每个触发目录额外保存 `trigger.json`、`trigger_result.json`、原始
`request_snapshot.txt`、`config_snapshot.yaml` 和
`grasp_generator_config_snapshot.yaml`，因此可以准确追溯服务启动时间、
触发顺序、触发时间、输入快照、成功/失败结果和两阶段全部输出。
`trigger.json`、`trigger_result.json` 和实时状态中的 `stage_interfaces` 还会
直接保存本次 `T_B_O`、偏移前 `T_B_TCP_raw/T_B_F_raw`、偏移量和
偏移后 `T_B_TCP/T_B_F` 的矩阵与 xyzrpy 数值。

上游注册矩阵若只含文本量化误差则保持原值，以维持历史数值分支稳定；若
旋转块已轻度偏离刚体旋转但仍满足受限阈值，抓取阶段会用 SVD 投影到最近的
SO(3)，并在服务日志、`grasp_results.json` 和阶段接口中记录原始行列式、
奇异值及修正距离。镜像、奇异或严重缩放/剪切矩阵仍会拒绝，不会静默规划。

实时状态和 `trigger_result.json` 使用三个不混淆的业务计时：

- `grasp_candidates_ready_ms`：触发到抓取候选集合生成完成；
- `execution_package_ready_ms`：触发到最终抓取点、两份轨迹和 manifest
  全部就绪；这是行为树应使用的主要节拍；
- `background_outputs_complete_ms`：点云、完整报告、PNG 与全部归档完成。

旧字段 `grasp_generation_ms`、`execution_outputs_ready_ms` 和 `total_ms`
继续保留兼容，但新接入应使用上述三个名称。

项目内 `output/` 是真实本地目录。服务每次触发先更新
`output/grasp_generation/`，再更新 `output/trajectory_planning/`；行为树、
轨迹运行脚本和网页可视化都读取这些固定路径。随后将两类输出按相同子目录
复制到 `/var/hybrid_log/.../trigger_.../` 历史目录。两处职责分别是“当前
生产输出”和“不可覆盖的历史追溯”。`output/` 根目录只允许存在上述两个
文件夹；服务状态、缓存、回归报告和诊断文件一律写入 `build/`。

每次成功触发先在本地发布以下文件，再完整复制到本次触发归档目录：

| 文件 | 用途 |
|---|---|
| `grasp_generation/best_grasp.json` | 实际通过完整轨迹门禁的候选：T_B_O、偏移前/后TCP和法兰、候选序号、偏移量与夹爪开口 |
| `grasp_generation/grasp_results.json` | 全部抓取候选及输入/config 指纹 |
| `grasp_generation/grasp_results.csv` | 全部抓取候选表格 |
| `grasp_generation/grasp_visualisation_base_mm.ply` | 工件、偏移前原始抓取中心/夹爪与偏移后规划抓取点/夹爪 |
| `trajectory_planning/request_with_generated_grasp.txt` | 阶段2实际输入：原始 T_B_O + 全部生成的 T_B_TCP/T_B_F 候选 |
| `trajectory_planning/joint_trajectory_photo_to_grasp.json` | 拍照位 → 抓取位，未携物 |
| `trajectory_planning/joint_trajectory_grasp_to_place.json` | 抓取位 → 放置位，携带目标工件 |
| `trajectory_planning/trajectory_manifest.json` | 两份轨迹的共同代次、SHA-256 和 ready 信号 |
| `trajectory_planning/point_cloud_B.ply` | 深度图反投影后的机器人基座系点云，单位 mm |
| `trajectory_planning/realtime_plan_diagnostics.json` | 输入指纹、ESDF、全部候选淘汰原因、A/B/C等级、统一评分、碰撞终检和分阶段耗时 |
| `trajectory_planning/joint_trajectory_full_cycle_preview.json` | 两段轨迹合并预览，仅用于可视化 |
| `trajectory_planning/verified_trajectory_path.png` | 点云、机械臂和轨迹路径图 |
| `trigger.json` | 本次序号、request_id、触发时间和输入快照路径 |
| `trigger_result.json` | 本次成功/失败、完成时间、generation_id、耗时和文件清单 |

两份正式执行文件使用本项目版本化契约 `aubo_joint_path/v1`，包含六轴：

```text
shoulder_joint, upperArm_joint, foreArm_joint,
wrist1_joint, wrist2_joint, wrist3_joint
```

每个点只有 `positions: [J1, J2, J3, J4, J5, J6]`，单位固定为 `rad`。正式文件不包含 `velocities`、`accelerations`、`time_from_start` 或路线时长；顶层明确记录 `speed_profile_included=false` 和 `speed_control_owner=behavior_tree`。文件还记录 `request_id`、`generation_id`、`route` 和 `workpiece_type`，执行侧或可视化侧可据此拒绝混用不同任务、代次或工件类型的路径。

`joint_trajectory_full_cycle_preview.json` 仍保留内部时间参数，仅用于碰撞密集采样、限位诊断和可视化，不是行为树的正式执行输入。

执行侧只能在以下条件全部满足后读取本地 `output/trajectory_planning/` 轨迹：

1. `trajectory_manifest.json` 中 `ready=true`；
2. manifest 和两份轨迹的 `generation_id` 一致；
3. 两份轨迹的 SHA-256 与 manifest 一致；
4. manifest 与两份路径均声明 `speed_control_owner=behavior_tree`；
5. 两份路径都已通过离线格式与首尾连续性校验。

若任何终检失败，本次仍保留独立的时间目录和失败 `trigger_result.json`，但不
发布可执行成功标志；之前各次成功触发目录保持不变，不会被覆盖。

`point_cloud_B.ply` 是 binary little-endian PLY，保留三相机全部有效深度点，不真正删掉目标工件。和 ESDF 挖除盒一致的目标点以 `config.yaml` 中的红色输出，其余环境点为灰色；因此它既能检查三组外参和融合效果，也能直观检查本次碰撞豁免范围。

诊断文件的 `point_cloud_file` 记录 PLY 路径、SHA-256、总点数、红色豁免点数和本次实际使用的 `T_B_C_by_camera_m`；PLY header 也保留三个矩阵。`target_workpiece` 保存本次工件代号、输入位姿、实体碰撞盒和点云豁免盒；`scene.target_mask_method` 记录逐相机精确三维挖除算法。`end_effector` 保存法兰坐标系下的六向边界、自动推导尺寸、配置 SHA-256 和碰撞球数量。每条路线的 `endpoint_pose_check` 还会用控制器同源标定 MDH 复算最后一个关节点，保证抓取末点和放置末点分别精确到达输入法兰目标。

## 5. 全轴碰撞与安全终检

规划和报告针对 AUBO-i12H 全六轴关节轨迹，不是只检查法兰或末端直线。最终轨迹按不大于 `0.5°` 的关节步长密集插值，并检查：

- AUBO-i12H 全连杆碰撞球与深度 ESDF/固定平台的世界碰撞；
- 机器人连杆之间的自碰撞；
- 六轴关节位置、速度和加速度限制；
- 拍照位、抓取位、放置位以及所有中间轨迹点；
- 抓取位 → 放置位阶段的附着工件与环境、机器人碰撞；
- 两段轨迹的起终点和执行顺序连续性；
- 每段最后一个关节点按本机标定 MDH 复算后的法兰到位误差。

机器人几何来自 `scripts/simulation/aubo_description/urdf/aubo_i12h.urdf` 和 `meshes/aubo_i12h/`，正式碰撞球配置位于 `model/aubo_i12h_curobo.yml`，本机补偿快照位于 `model/aubo_i12h_kinematics.yaml`。420 个本体球由官方 STEP 派生 STL 生成，并通过每连杆 10 万表面采样覆盖闸门；碰撞球的 `center/radius` 单位固定为 m。

### 5.1 AGV 基座平台统一配置

AGV/小车平台只在 [config/config.yaml](config/config.yaml) 中正式定义。规划、GPU/CPU 碰撞检测、报告、路径 PNG 和浏览器仿真共同使用同一个轴对齐长方体：

```yaml
environment:
  base_platform:
    negative_extent_xyz_mm: [400.0, 400.0, 1000.0]
    positive_extent_xyz_mm: [400.0, 400.0, 0.0]
```

两个数组分别是从机械臂基座坐标系 `{B}` 原点向 `-X/-Y/-Z` 和 `+X/+Y/+Z` 的延伸距离，全部单位为 mm。当前碰撞实体范围为 `X=-400..400 mm`、`Y=-400..400 mm`、`Z=-1000..0 mm`。

如小车上固定箱体高于 `{B}` 原点，可增大 `positive_extent_xyz_mm` 的 Z 值；如平台在某一水平方向不对称，可分别调整正负方向。每个值必须非负，且每个轴的正负延伸之和必须大于 0。修改后必须重启 GPU 服务。

`base_link` 是固定安装在该平台上的不动基座，所以系统只对 `base_link ↔ 自身 AGV 平台` 这一对必要安装接触做豁免。`base_link` 对深度 ESDF 仍检测；`shoulder_Link` 到 `wrist3_Link` 和配置法兰末端对 AGV 平台均不豁免。这保证 `+Z` 可用于高出基座原点的固定箱体，同时不会把全轴碰撞检测整体关闭。

网页中的六个平台输入框允许临时改显示副本，但临时数值不会进入规划和碰撞报告；点击“重新加载启动 config 平台”可恢复本次仿真默认几何。

### 5.2 法兰末端矩形统一配置

法兰末端只在 [config/config.yaml](config/config.yaml) 中正式定义。规划、最终碰撞报告、路径 PNG 和浏览器三维页面共同读取以下字段：

```yaml
end_effector:
  enabled: true
  parent_frame: flange_link
  geometry:
    type: box
    negative_extent_xyz_mm: [50.0, 35.0, 0.0]
    positive_extent_xyz_mm: [50.0, 35.0, 230.0]
```

法兰末端只有一个坐标系 `flange_link`。它与 `wrist3_Link` 网格的 `z=0` 圆形机械安装面严格同原点、同方向，不允许额外固定平移或 RPY。两个数组分别表示从法兰原点向 `-X/-Y/-Z` 和 `+X/+Y/+Z` 的延伸距离，矩形边界严格为 `[-negative_extent_xyz_mm, +positive_extent_xyz_mm]`，并始终与法兰 XYZ 轴平行。这里没有第二个工具坐标系；程序自动推导矩形中心和完整尺寸。

当前配置已启用 `100 × 70 × 230 mm` 的末端矩形，范围为 `X=-50..50 mm`、`Y=-35..35 mm`、`Z=0..230 mm`。规划、碰撞检测、报告、路径图和网页模拟共同读取这组边界。

更换实际法兰末端时，只需更新六个真实延伸距离，确认每个轴的正负延伸之和大于 0，然后将 `enabled` 改为 `true` 并重启 GPU 服务。启用后，cuRobo 会把同一个 box 自动转换为完整覆盖矩形的保守球填充；浏览器则显示完全相同边界的精确矩形。改变 `max_sphere_radius_mm` 只影响碰撞近似精度和 GPU 开销，不改变可视化矩形尺寸。

### 5.3 固定工件目录与目标点云豁免

行为树每次只需下发两项工件数据：

```text
workpiece_type = QR0001
T_B_O_target =
  r11 r12 r13 tx_mm
  r21 r22 r23 ty_mm
  r31 r32 r33 tz_mm
  0   0   0   1
```

`{O}` 固定在工件几何中心，位姿在机械臂基座系 `{B}` 下。工件尺寸不得再放入 `request.txt`，统一在 [config/config.yaml](config/config.yaml) 的 `workpieces.types` 中维护：

```yaml
workpieces:
  frame: workpiece_geometry_center
  visualization:
    exclusion_color_rgb: [220, 38, 38]
  types:
    QR0001:
      configured: true
      collision_box:
        negative_extent_xyz_mm: [50.0, 50.0, 50.0]
        positive_extent_xyz_mm: [50.0, 50.0, 50.0]
      point_cloud_exclusion:
        negative_extent_xyz_mm: [0.0, 0.0, 0.0]
        positive_extent_xyz_mm: [0.0, 0.0, 0.0]
```

两个数组分别表示从工件几何中心沿 `-X/-Y/-Z` 和 `+X/+Y/+Z` 延伸的距离，允许不对称。两套盒在当前生产策略中的职责是：

- `collision_box`：保留给工件目录、报告和可视化使用；当前轨迹规划不把它附着到法兰，也不参与任一段碰撞门禁。
- `point_cloud_exclusion`：可选的目标点云挖除区域。六向全为 0 表示完全不挖除；本项目当前所有工件都按全 0 运行。

GPU不会再清空目标投影得到的整块二维矩形。现在每个有效深度像素都先反投影成三维点，再变换到旋转后的工件局部坐标系，只挖除落在六向豁免盒内的点。因此目标后方的箱壁、旁边工件和同一图像矩形内的其他障碍会保留在ESDF中。

当前已启用工件可保留各自的可视化实体盒，但点云豁免区按现场要求全部设为 0。未配置代号仍保持 `configured: false` 和全 0；行为树下发未配置代号时服务会直接拒绝规划，不会静默套用其他工件。

修改或新增工件后必须重启GPU服务。程序会拒绝负数或只有部分轴为零的残缺盒；全零豁免是合法且明确的“禁用挖除”。

## 6. AUBO 轨迹运行

`scripts/轨迹运行/play_full_cycle_preview.py` 保留单个 `--trajectory`
文件接口，同时增加正式 `--manifest + --route` 接口。不加
`--execute` 时只读取和校验，不连接机械臂：

```bash
cd /home/vecow/workspace/轨迹规划/grasp_planner_pkg

# 保留的直接文件接口
python3 scripts/轨迹运行/play_full_cycle_preview.py \
  --trajectory output/trajectory_planning/joint_trajectory_photo_to_grasp.json

# 推荐：校验 manifest 绑定的同一代抓取路线
python3 scripts/轨迹运行/play_full_cycle_preview.py \
  --manifest output/trajectory_planning/trajectory_manifest.json \
  --route photo_to_grasp
```

manifest 模式检查 `ready`、契约、任务、代次、工件、路线、点数、
SHA-256 和两段抓取位连续性。这些只是轨迹文件一致性约束，不会
在脚本中额外添加限速或运动策略。

`scripts/轨迹运行/run_grasp_cycle.sh` 会自动 source Hybrid 环境，并阻塞执行：

```text
夹爪打开(r=1.0) → 拍照到抓取 → 夹爪闭合(r=0.5)
→ 抓取到放置 → 夹爪打开(r=1.0)
```

```bash
bash scripts/轨迹运行/run_grasp_cycle.sh
```

它在开始动作前完整校验两份文件并锁定 `generation_id`，然后在每段
运行前要求仍是同一代，避免服务新发布的轨迹混入已开始的抓取循环。
完整参数、路径替换方式和 Python 接口见
[`scripts/轨迹运行/README.md`](scripts/轨迹运行/README.md)。

## 7. 浏览器可视化与轨迹回放

启动脚本现在接受一个 config 文件参数：

```bash
cd /home/vecow/workspace/轨迹规划/grasp_planner_pkg
bash scripts/simulation/serve_noetic_demo.sh config/grasp_planner.json
```

命令末尾的 config 是本次仿真配置输入；省略时默认仍是 `config/config.yaml`。页面会把该文件中的 `environment.base_platform` 和 `end_effector` 分别加载为基座平台和法兰末端默认模型。两部分都可在页面临时修改，但不回写 config，也不会改变正式规划结果。

启动脚本会打印普通 URL 和带 `autoload=latest` 的 URL。打开后者会直接
加载最新两阶段 output。在普通页面中点击“一键加载最新两阶段
output”也会完成同样操作：

1. 读取阶段1 `grasp_generation/best_grasp.json`，显示 `T_B_O`、`T_B_TCP` 和 `T_B_F`；
2. 加载基座系 `point_cloud_B.ply` 及红色工件豁免点；
3. 通过 manifest 加载两条正式轨迹；
4. 在三维场景画出 O/TCP/F 三组 XYZ 坐标轴；
5. 点击“执行”回放“拍照 → 抓取 → 停留 → 放置”全轴动作。

页面的两阶段 output 根目录、点云文件和两条轨迹文件均提供非空的绝对路径
输入框，初始值就是当前项目 `output` 的默认绝对路径。直接修改输入框后可从
任意一次触发归档加载；历史两阶段 output 还会使用该目录根部的
`config_snapshot.yaml` 和 `request_snapshot.txt` 恢复本次工件及场景配置。
原有文件选择加载方式继续保留。绝对路径由启动脚本内置的只读本地服务器
提供，启动命令不变。

一键加载会校验 manifest `ready`、执行契约、`request_id`、`generation_id`、
`workpiece_type`、点数、两个 SHA-256、两段衔接，以及阶段1 `T_B_O` 与当前
`input/request.txt` 是否一致。手动选择两个 JSON 文件的原接口仍然保留。

网页启动时自动读取 `input/request.txt` 的 `workpiece_type` 和 `T_B_O_target` 4×4 矩阵，再从 `config.yaml` 选择相应六向豁免盒。加载PLY/CSV/JSON点云后，盒内点强制显示为红色，盒外点保留原RGB；没有RGB时显示为灰色。页面红框、红点、GPU ESDF挖除和碰撞报告来自同一组数据。

页面的末端矩形有两种模式：

- `启动 config 默认矩形`：绿色显示，从启动命令指定的 config 读取；
- `仅本次可视化临时自定义`：橙色显示，只用于临时观察，不进入规划、碰撞报告或实机安全结论。

修改配置文件后，可以在网页分别点击“重新加载启动 config 平台/矩形”；正式规划服务仍须 `restart`。

AGV 平台默认也会从同一个 `config.yaml` 加载，显示的六向边界与本次正式规划一致。

验证可视化模型资源：

```bash
python3 scripts/simulation/validate_i10_assets.py
```

## 8. 软件架构

正式实时链路：

```text
固定 config.yaml + 动态 request.txt + 深度图
        │
        ▼
文件触发与输入指纹
        │
        ▼
抓取点生成：T_B_O + 离线标注 → 原始TCP/法兰 → Z轴偏移后规划TCP/法兰
        │
        ▼
内部派生 request（原始输入保持不变）
        │
        ▼
nvblox 三相机深度融合 / config 配置分辨率 ESDF
        │
        ▼
cuRobo 六轴种子轨迹规划
        │
        ▼
拍照构型直接到平台自适应预抓取点 → 固定姿态TCP直线接近
        │
        ▼
机器人/法兰末端世界碰撞 + 平台 + 自碰撞 + 关节限位终检
        │
        ▼
内部时间参数化/安全终检，两份无速度 AUBO 位置路径原子发布
        │
        ├── AUBO 执行侧
        └── 基座系 PLY、浏览器可视化与诊断报告
```

主要模块：

| 模块 | 作用 |
|---|---|
| `OpenmindTrajectoryPlan` | C++ 正式服务：抓取 + 轨迹 + TCP |
| `OpenmindTrajectoryTrigger` | TCP 触发客户端 |
| `OpenmindVisualizerServer` | 仿真静态页服务 |

服务内的 MotionPlanner 只负责生成无世界碰撞的平滑种子。真实深度体素世界由独立 RobotSceneCollision 终检器加载，避免在 MotionPlanner 中热切换世界导致缓存状态不一致。

## 9. 测试与回归

使用已配置的 `pick-gpu` 环境：

```bash
cd /home/vecow/workspace/轨迹规划/grasp_planner_pkg

/home/vecow/miniconda3/envs/pick-gpu/bin/python tests/test_i10_collision_config.py
/home/vecow/miniconda3/envs/pick-gpu/bin/python tests/test_end_effector_config.py
python3 tests/test_visualizer_absolute_path_loading.py
python3 tests/test_full_cycle_preview_player.py
python3 scripts/simulation/validate_i10_assets.py
```

覆盖范围：

- IK 回环、点雅可比、优化器梯度、ESDF 和时间参数化；
- 确定性障碍绕行端到端测试；
- 两组实机 q/法兰位姿对与 i12H 标定 MDH 的亚显示精度回归；
- 420 个 i12H 本体碰撞球、米制单位和固定装配 ACM 防回归；
- config 末端矩形坐标、球填充覆盖和有效 cuRobo 模型生成；
- config AGV 平台六向边界、精确长方体 SDF 和安装基座屏蔽；
- `base_link` 安装接触豁免的最小范围，确保其他连杆/末端/工件仍会报平台碰撞；
- 固定拍照/放置工位、config 与动态 request 合并、重复字段拒绝和深度反投影；
- 基座系/mm 二进制 PLY 的坐标、RGB、SHA-256 和红色工件豁免点；
- QR工件目录完整性、未配置工件拒绝、六向不对称旋转盒和精确三维点归属；
- ESDF 语义缓存键，防止注释/排版或非场景字段误触发重建；
- ESDF 首帧缓存预留和越界受控扩容，覆盖连续工件位姿改变造成的稠密网格增大；
- 项目级服务独占锁，拒绝两个热服务共同消费触发和竞争写正式输出；
- 两份实时输出的格式、点数、碰撞门禁和共同代次；
- 两阶段 T_B_O/T_B_TCP/T_B_F 多候选接口、惰性安全回退、GPU空IK的
  控制器MDH回退，以及本地/日志归档哈希一致性；
- 拍照→抓取法兰轨迹的最大抬升和路径长度硬门禁，防止零碰撞但绕到机械臂
  上方的大环被发布；
- 拍照构型直接到预抓取点，拒绝冗余姿态中间点造成的腕部过冲；预抓取距离
  按AGV平台自动缩短，最终接近段保持法兰姿态并沿TCP轴直线运动；
- 拍照到抓取整段及最终接近段分别执行J4/J5/J6累计转角门禁；
- 全部抓取候选完整评估、IK分支统一评分、A/B/C分级和330°冗余绕转硬门禁；
- i12H URDF、官方 STEP 派生视觉/碰撞网格资源完整性。

## 10. 当前性能基线

当前工作站：RTX 4060；标准输入为主/左/右三张 `1280 × 720 float32 TIFF`。测量时相机应用另占约 `1888 MiB` 显存。

| 阶段 | 当前实测 |
|---|---:|
| 服务冷启动、抓取预热、三相机 ESDF 和全部候选预计算 | QR0008 多次实测 33.089–56.078 s，仅启动时一次；最新 34.490 s |
| 阶段1：抓取点/TCP和对应法兰位姿生成 | QR0008 最新热触发 82.2 ms |
| 阶段2：全候选选优、质量/碰撞终检与两份轨迹发布 | QR0008 最新热触发 503.2 ms |
| 从触发到两份轨迹与 manifest 就绪 | QR0008 最新热触发 632.9 ms |
| 规划器点云与完整碰撞报告就绪 | QR0008 最新热触发 1487.1 ms |
| 抓取 PLY、路径 PNG 等全部扩展输出完成 | QR0008 最新热触发 9078.2 ms |
| `nvidia-smi` 服务进程常驻显存 | 本次约 1610 MiB |

当前QR0008已删除会造成腕部过冲的“拍照位置最终姿态”中间点，J4/J5/J6
在抓取段分别单调运动128.1769/167.8485/74.0888 degree，累计量由旧路线
478.9554 degree降为370.1142 degree。热缓存命中后两份执行轨迹最新为632.9 ms
就绪。最终预抓取距离由平台门禁从期望的200 mm自动缩短为80 mm；该80 mm接近段
法兰路径长度为80.0001 mm、最大横向偏差0.0037 mm、最大姿态偏差
0.000029 degree，J4/J5/J6累计转动分别为3.5522/1.3968/1.4196 degree。
两段诊断预览时长为10.7614 s和16.4187 s，完整周期27.1801 s。三张
全分辨率点云、抓取 PLY 和路径 PNG 属于扩展输出，不阻塞行为树读取轨迹。
本次8个抓取候选均已完成评估，只有候选7通过完整周期；拍照→抓取为A级，
完整周期因采用“原路退回拍照位再去放置”的必要绕障路线评为B级，总分
82.941。其法兰最大额外抬高为0 mm、路径倍率1.153，机器人/末端碰撞均为0。
上述数值是当前硬件、当前输入和热缓存命中的测量结果，不是硬实时上限；
深度图或目标位姿变化后可能重建场景/种子，必须按生产数据重新统计。

## 11. CPU 参考接口

`src/grasp_planner/` 仍保留独立 CPU 参考后端，用于数学回归、合成场景验证和算法开发，不是生产实时服务的默认入口。

```python
from grasp_planner import (
    GraspPlanner,
    PlanRequest,
    GraspCandidate,
    PlannerConfig,
    Kinematics,
    RobotCollisionModel,
)

planner = GraspPlanner(kin, robot, cfg, safety_planes=[(normal, offset)])
result = planner.plan(request)
```

参考后端回归：

```bash
python3 tests/test_core.py
python3 tests/test_e2e_obstacle.py
```

这些脚本只验证参考管线和数学实现，不代表现场标定、真实深度或生产性能。

## 12. 已知边界与现场要求

- 当前放置位虽已通过机器人、末端和点云终检，仍必须确认它就是现场期望工位；
- 当前末端矩形已启用，六向范围为 `[-50,+50] × [-35,+35] × [0,+230] mm`，更换工具后必须实测修改；
- 当前 AGV 平台范围为 `[-400,+400] × [-400,+400] × [-1000,0] mm`，必须按现场小车和固定箱体复核；
- 深度图、手眼标定、拍照关节角和法兰位姿必须属于同一拍照时刻；
- 行为树下发的工件代号或几何中心位姿错误，会直接影响抓取候选变换和最终抓取法兰端点；
- 新工件需配置抓取数据库映射并在主配置中设为 `configured: true`；当前全零点云豁免不要求填写实体盒尺寸；
- 规划内部使用速度/加速度约束做可执行性诊断，但两份正式路径只发布关节位置；现场速度、加速度、负载和急停策略由行为树/控制器统一管理；
- 旧 i10 URDF、旧 i10 输出和 CPU 简化模型不能替代正式 i12H 标定 URDF/curobo 碰撞模型；
- 浏览器仿真用于路径检查，不能替代实机低速、空载和安全联锁验证；
- 未通过 manifest、碰撞报告和执行侧首点检查时，不得向机械臂发送轨迹。
