# 轨迹运行

这个目录只负责“轨迹文件 → AUBO pathBuffer”接口和抓取循环的顺序调用。它不重新规划、不修改轨迹点，也不接管行为树的速度控制。

## 文件

- `play_full_cycle_preview.py`：读取一条轨迹，一次性写入 AUBO `cubic_spline pathBuffer`并等待完成。
- `run_grasp_cycle.sh`：按“打开夹爪 → 拍照到抓取 → 闭合夹爪 → 抓取到放置 → 打开夹爪”顺序同步执行。

项目正式输入是：

```text
output/trajectory_planning/trajectory_manifest.json
output/trajectory_planning/joint_trajectory_photo_to_grasp.json
output/trajectory_planning/joint_trajectory_grasp_to_place.json
```

`manifest` 模式会在连接机械臂前检查：

- `ready=true` 且契约为 `aubo_joint_path/v1`；
- 两条轨迹的 `request_id`、`generation_id`、`workpiece_type` 与 manifest 相同；
- 路线分别为 `photo_to_grasp` 和 `grasp_to_place`；
- 文件 SHA-256 和点数与 manifest 相同；
- 第一条的最后关节点与第二条的第一关节点连续。

这些是文件一致性检查，不是额外的限速或运动安全策略。

## 正式轨迹接口

```json
{
  "format": "aubo_joint_path/v1",
  "units": {"positions": "rad"},
  "joint_names": [
    "shoulder_joint",
    "upperArm_joint",
    "foreArm_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint"
  ],
  "speed_profile_included": false,
  "speed_control_owner": "behavior_tree",
  "request_id": "plan-...",
  "generation_id": "...",
  "workpiece_type": "QR0005",
  "route": "photo_to_grasp",
  "points": [
    {"positions": [0, 0, 0, 0, 0, 0]},
    {"positions": [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}
  ]
}
```

每个点只能包含 J1-J6 的 `positions(rad)`，不带速度、加速度或 `time_from_start`。脚本仍兼容诊断用 `trajectory_msgs/JointTrajectory` JSON，但它不是实时服务发布给行为树的正式文件。

## 使用

在项目根目录执行：

```bash
cd /home/vecow/workspace/轨迹规划/grasp_planner_pkg
```

直接轨迹文件接口仍然保留。不加 `--execute` 只读取检查，不连机械臂：

```bash
python3 scripts/轨迹运行/play_full_cycle_preview.py \
  --trajectory output/trajectory_planning/joint_trajectory_photo_to_grasp.json
```

使用 manifest 检查同一代次的抓取路线：

```bash
python3 scripts/轨迹运行/play_full_cycle_preview.py \
  --manifest output/trajectory_planning/trajectory_manifest.json \
  --route photo_to_grasp
```

实机运行指定路线：

```bash
python3 scripts/轨迹运行/play_full_cycle_preview.py \
  --manifest output/trajectory_planning/trajectory_manifest.json \
  --route grasp_to_place \
  --execute
```

不传 `--trajectory` 和 `--manifest` 时，默认读取 `output/trajectory_planning/joint_trajectory_photo_to_grasp.json`。

## 完整抓取循环

```bash
bash scripts/轨迹运行/run_grasp_cycle.sh
```

脚本会自动执行：

```bash
source /opt/openmind/hybrid/hybrid_tools/scripts/run/hybrid_setup.bash
```

默认阻塞顺序是：

```text
gripper_bt_trigger -r 1.0 -s 50.0 -f 50.0
photo_to_grasp
gripper_bt_trigger -r 0.5 -s 50.0 -f 50.0
grasp_to_place
gripper_bt_trigger -r 1.0 -s 50.0 -f 50.0
```

脚本开始时会校验两条轨迹并锁定 `generation_id`。每次运行路线前再校验同一代次；如果服务在中途发布了新代次，第二条轨迹不会被混入当前循环。

路径都可在脚本顶部直接改，或用环境变量替换：

```bash
TRAJECTORY_MANIFEST=/data/task/trajectory_manifest.json \
PHOTO_TO_GRASP_TRAJECTORY=/data/task/joint_trajectory_photo_to_grasp.json \
GRASP_TO_PLACE_TRAJECTORY=/data/task/joint_trajectory_grasp_to_place.json \
bash scripts/轨迹运行/run_grasp_cycle.sh
```

其他可替换参数：`HYBRID_SETUP`、`PYTHON_BIN`、`TRAJECTORY_RUNNER`、`GRIPPER_COMMAND`、`GRIPPER_SPEED`、`GRIPPER_FORCE`、`GRIPPER_OPEN_RATIO`和 `GRIPPER_CLOSE_RATIO`。

AUBO 连接地址继续由公共 RPC 模块读取：

```bash
export AUBO_IP=192.168.192.10
export AUBO_RPC_PORT=30004
```

## Python 接口

```python
from pathlib import Path
from play_full_cycle_preview import (
    build_controller_path,
    execute_trajectory,
    load_trajectory,
    load_trajectory_manifest,
)

cycle = load_trajectory_manifest(Path("trajectory_manifest.json"))
trajectory = cycle.photo_to_grasp
controller_path = build_controller_path(trajectory)
execute_trajectory(trajectory, controller_path)

# 仍可直接读取单个轨迹文件
trajectory = load_trajectory(Path("trajectory.json"))
```
