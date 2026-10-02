# cuRobo 抓取轨迹规划器

本项目为 AUBO-i12H 六轴机械臂生成一次抓取周期的两段关节轨迹：**拍照位 → 抓取位**、**抓取位 → 放置位**。C++ 常驻服务接收工件位姿和主、左、右三张深度图；cuRobo GPU 子进程负责碰撞世界与轨迹优化。服务只规划、验证并发布轨迹，**不会直接驱动机械臂或夹爪**。

本文按当前 `src/`、`services/`、`config/` 和构建脚本编写。`docs/` 中的版本方案记录设计与分析；运行行为以源码及当前配置为准。

## 运行流程

```text
request.txt + 三张深度 TIFF + 主相机 RGB PNG
                  │
                  ▼
OpenmindTrajectoryTrigger ──GP02/TCP──▶ OpenmindTrajectoryPlan
                                      ├─ grasp：加载离线 NPZ 标签，生成抓取候选
                                      └─ trajectory：融合三路点云，构建 ESDF，
                                         规划并复核两段轨迹
                                                   │
                                                   ▼
                                      output/ + 会话归档目录
```

服务启动时加载全部已配置工件的 NPZ 标签、机器人标定与碰撞模型，启动并预热本地 cuRobo 子进程；全部初始化通过后才监听触发端口。一次只处理一个任务，不排队。

| 部分 | 当前位置 | 职责 |
| --- | --- | --- |
| 主服务与 TCP 客户端 | `src/main.cpp`、`src/trigger_tcp_main.cpp` | 启动、接收任务、发送触发 |
| 两条处理流程 | `src/pipeline/` | 抓取候选生成、碰撞建图、规划与发布 |
| GPU 规划服务 | `services/curobo_planner/curobo_plan_service.py` | cuRobo 求解、PBA 距离变换、ESDF 查询 |
| 配置 | `config/grasp_planner.json`、`config/base/` | 工位、相机、机型、流程与阈值 |
| 模型 | `model/`、`scripts/simulation/aubo_description/` | cuRobo 球模型、运动学快照、URDF 与网格 |
| 可视化 | `src/visualizer_server_main.cpp`、`scripts/simulation/` | 本地浏览器回放 |
| 第三方源码 | `third_party/` | cuRobo、nvblox、nvblox_torch 子模块 |

## 运行前准备

本仓库目前是**现场环境项目**，不能仅凭源码在任意电脑上直接运行：

- C++17、CMake ≥ 3.16，以及 glog、gflags、yaml-cpp、Eigen3、nlohmann/json、zlib、libpng 和 pthread。`CMakeLists.txt` 会优先查找 `/opt/openmind/hybrid_packages/...` 下的 glog/gflags，再尝试系统库。
- NVIDIA GPU、CUDA 和可运行 cuRobo 的 Python 环境。`config/grasp_planner.json` 的 `curobo_python` 当前写为本机的绝对路径；部署到其他机器时需改为实际 Python 路径，并确保该环境能导入 cuRobo、PyTorch、NumPy、PyYAML 等服务所需模块。
- `config/base/pipeline/flow/grasp.yaml` 的 `asset_root` 当前指向 `/opt/openmind/sps_database/grasping/objects`。启动时会加载映射中的全部工件 NPZ；缺失或无有效标签会导致初始化失败。该资产库不在仓库内。
- `model/aubo_i12h_curobo.yml` 的 `asset_root_path` 与 `urdf_path` 当前是本机绝对路径。换目录部署时要更新为新位置，并核对 `config/base/device/aubo.json` 中的实机标定及模型一致性。

克隆时获取子模块：

```bash
git clone --recurse-submodules https://github.com/ShuangqingWang/curobo-grasp-trajectory-planner-.git
```

已克隆仓库可在项目根目录执行 `git submodule update --init --recursive`。`patches/nvblox-local.patch` 保存了本地 nvblox 子模块的一处修改；子模块提交本身不包含未提交修改，部署时应核对该补丁是否仍需要并单独应用。

### 构建

在项目根目录执行：

```bash
cmake -S . -B build
cmake --build build --parallel
```

这会生成 `build/OpenmindTrajectoryPlan`、`build/OpenmindTrajectoryTrigger` 和 `build/OpenmindVisualizerServer`。`scripts/build_tar.sh` 还会执行 CPack 打包，并尝试修补本机 `/opt/openmind/hybrid/third_party` 中 ONNX Runtime 的 RPATH；仅需编译时可使用上面的 CMake 命令。

### 日志目录

默认会话日志根目录来自 `config/base/app.json`：`/var/hybird_log/grasp_log`。服务需要能创建该目录；开发机可用 `--log-root` 覆盖：

```bash
./build/OpenmindTrajectoryPlan config/grasp_planner.json --log-root ./logs
```

## 启动和触发

以下命令均在项目根目录执行。

终端 1，启动常驻服务：

```bash
./build/OpenmindTrajectoryPlan config/grasp_planner.json --log-root ./logs
```

终端 2，准备 `input/` 中本轮文件后发送一次任务：

```bash
./build/OpenmindTrajectoryTrigger --input-dir input
```

客户端默认连接 `127.0.0.1:9108`；可用 `--host`、`--port`、`--timeout-s` 覆盖。服务端监听地址与端口在 `config/base/app.json` 的 `trigger_tcp` 中配置，当前为 `0.0.0.0:9108`。停止服务使用 `Ctrl+C`。

不启动 TCP 监听也可用同一条处理链运行一次：

```bash
./build/OpenmindTrajectoryPlan config/grasp_planner.json --once --input-dir input --log-root ./logs
```

要检查 `$include` 展开后的配置而不启动 GPU 子进程：

```bash
./build/OpenmindTrajectoryPlan config/grasp_planner.json --dump-config
```

### 输入文件

`input/` 是运行时输入目录，原始采集文件默认不会提交到 Git。一次触发需要：

| 文件 | 内容 |
| --- | --- |
| `request.txt` | 工件类型、位姿、拍照法兰状态和图片路径 |
| `raw_depth.tiff` | 主相机深度图，单通道 float32，单位 mm |
| `raw_depth_left.tiff`、`raw_depth_right.tiff` | 左、右相机同步深度图，格式同上 |
| `raw_color.png` | 主相机 RGB 图，供后续彩色点云显示 |

`request.txt` 必填字段如下。图片路径可改名，但由客户端相对 `request.txt` 所在目录读取；服务端使用 TCP 报文中的图片字节。

| 字段 | 格式与用途 |
| --- | --- |
| `workpiece_type` | `grasp.yaml` 中声明的工件代号 |
| `T_B_O_target` | 目标工件在机器人基座系下的 4×4 刚体矩阵，平移为 mm |
| `T_base_flange` | 本轮拍照法兰在基座系下的 4×4 刚体矩阵，平移为 mm |
| `T_base_flange_joint_angles_deg` | 对应拍照状态的六个关节角，JSON 数组，单位 degree |
| `depth_main_path`、`depth_left_path`、`depth_right_path`、`rgb_main_path` | 图片相对路径；省略时使用上表文件名 |

两个矩阵可写成等号后的 JSON 4×4 数组，或等号后另起四行、每行四个数。解析器会校验刚体变换；`T_base_flange` 与关节角须对应同一个拍照状态。深度有效范围由 `config/base/camera.json` 当前配置为 150～2500 mm。

批量拍照数据可用 `scripts/批量拍照数据测试/prepare_dataset_inputs.py` 逐项准备；其路径配置与用法见该目录的 README。脚本只准备输入，不会替你启动服务。

### TCP 应答

客户端发送 `GP02` 24 字节报头，加 `request.txt`、三张 TIFF 和主 RGB PNG 的原始字节。服务逐行返回 JSON：

1. `accepted`、`rejected` 或 `busy`。`accepted` 仅表示已接收，且会把上一轮输出标记为不可用。
2. `completed` 或 `failed`。成功时两段轨迹及 `trajectory_manifest.json` 已发布；这时点云文件可能仍在后台写入。
3. `archived`。报告点云和会话归档的收尾结果。

`rejected` 与 `busy` 不创建新代次。任务执行及归档期间的新连接会得到 `busy`。若客户端在收到 `accepted` 后断开，应检查同代次的 manifest 和日志，不能仅凭断线判断任务未执行。

## 规划与碰撞的当前实现

`grasp` 流程从离线 NPZ 标签读取并按分数排列抓取候选；它只生成抓取位姿，不做轨迹规划。`trajectory` 流程使用请求中的拍照关节角作为起点，先规划到候选抓取法兰位姿，再从抓取实际末点规划到配置中的放置关节角。最多按配置尝试 10 个候选，当前默认在首个两段合格的完整周期处停止。

碰撞世界由三路深度图反投影到基座系并融合而成。当前 GPU 服务先按 `|X|`、`|Y| ≤ CAP`、工作台底面与法兰 Z 上限截断点云，其中 `CAP = robot_reach_mm + 末端正 Z 长度`；再用截断点云、拍照法兰和放置法兰定轴对齐网格。**当前实现不外扩网格**。工作台只把落在网格内的部分写入占据栅格，之后用 PBA 欧氏距离变换建立 fp16 ESDF。网格外不参与该距离场的障碍查询。

当前配置为 3 mm 体素、3 mm 碰撞激活距离，图规划关闭。机器人本体使用 `model/aubo_i12h_curobo.yml` 中的碰撞球；法兰末端由 `config/base/app.json` 的盒体六向尺寸生成棱线球。工作台为体素障碍，`installation_exclusion.enabled` 当前为 `false`。代码尚未把已识别工件的 CAD 网格加入碰撞世界，也没有末端盒体对原始点云的自定义 SDF 碰撞代价；相关方案见 `docs/第二版本方案/`。

两段轨迹在 cuRobo GPU 侧检查沿途环境碰撞与自碰撞；C++ 侧再做评级、各点关节限位、净转角和端点运动学复核。当前 A/B 评级阈值包括路径倍率 1.5/2.0、抓取段相对拍照点抬升 5/10 mm、六轴最大净转角 180/360°；统一端点容差为 1.0 mm / 0.2°，放置段另有 0.5° 关节端点容差。详细参数均在 `config/base/pipeline/flow/trajectory.yaml`。

## 输出、归档与使用边界

`output/` 是当前轮的发布目录；每次触发会覆盖或清理本规划器管理的旧产物。消费端应先检查 `trajectory_manifest.json` 的 `status=completed`、`ready=true`，再按同一 `request_id`、`generation_id` 验证两条轨迹，不能只看文件是否存在。

| 路径 | 内容 |
| --- | --- |
| `output/grasp_generation/grasp_result.json` | 当前轮候选抓取位姿和分数 |
| `output/trajectory_planning/trajectory_manifest.json` | 状态、代次、选中候选、评级、轨迹哈希、碰撞世界元数据 |
| `output/trajectory_planning/joint_trajectory_photo_to_grasp.json` | 第一段正式关节轨迹 |
| `output/trajectory_planning/joint_trajectory_grasp_to_place.json` | 第二段正式关节轨迹 |
| `output/trajectory_planning/point_cloud_B.ply` | 后台生成的基座系彩色点云，供可视化使用 |
| `output/trajectory_planning/collision_cloud_B.bin` | 送往 GPU 服务的基座系 float32 点云 |
| `output/visualization/collision_model.json` | 启动时导出的固定碰撞模型快照 |

正式轨迹契约为 `aubo_joint_path/v1`：每点只有 J1～J6 的 `positions`（rad），文件不带速度曲线；速度控制归现场行为树。manifest 会记录两条轨迹的 SHA-256、点数和代次。未找到合格完整周期时，manifest 为 `failed/ready=false`，不会把上轮成功轨迹当成本轮结果。

会话归档根目录为 `log_root`。每次启动建立会话目录，内含 `session.json`、`service.log`、`curobo_service.log`；每次接受的任务另建目录，保存 `request.log`、原始 `input/`、输出副本 `output/` 和 `result.json`。归档副本与下一轮的 `output/` 覆盖互不影响。

当前 `scripts/轨迹运行/run_grasp_cycle.sh` 会明确退出并提示改用现场行为树；所依赖的真机播放 Python 脚本已移除。**不要将该脚本视为可用的执行入口。**

## 浏览器可视化

先完成构建，再运行：

```bash
bash scripts/simulation/serve_noetic_demo.sh config/grasp_planner.json
```

脚本默认在 `127.0.0.1:8088` 启动 `OpenmindVisualizerServer`，并打印访问地址。可通过 `HOST`、`PORT` 环境变量改绑定地址和端口；启动脚本要求 config 位于项目目录内。浏览器回放使用已发布的轨迹、点云和模型快照，仅用于查看结果，不参与规划与实机控制。

## 排查入口

- **启动即失败**：先用 `--dump-config` 检查展开后的配置，再看会话目录下的 `service.log` 与 `curobo_service.log`。重点核对 Python 路径、外部 NPZ 库、模型绝对路径、CUDA/cuRobo 环境和 `log_root` 写入权限。
- **触发被拒绝**：检查 `request.txt` 的四个必填业务字段、矩阵格式及图片是否齐全；`busy` 表示已有任务仍在执行或归档。
- **规划无解或失败**：查看 manifest 的 `reason`、`failed_flow`、`failed_stage`、`candidate_statistics`、`collision_world`，再看该轮 `request.log` 和 GPU 子进程日志。
- **结果文件不一致**：以 manifest 的 `generation_id`、`ready`、哈希和点数为准；等待 `archived` 后再读取最终点云与归档状态。

设计与碰撞分析资料位于 [`docs/第二版本方案/`](docs/第二版本方案/)；批量输入和可视化的附加用法分别位于 [`scripts/批量拍照数据测试/README.md`](scripts/批量拍照数据测试/README.md) 与 [`scripts/simulation/README.md`](scripts/simulation/README.md)。后两份子目录文档包含历史描述，涉及具体字段时以本 README 和当前源码为准。
