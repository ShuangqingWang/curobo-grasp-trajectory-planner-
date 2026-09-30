---
name: grasp-planner-project
description: Project workflow for the AUBO/AGV grasp trajectory planner repository. Use when working in this project on grasp_planner_pkg, aubo_grasp_trajectory_planning_technical_plan.md, visualization outputs, tests, demos, collision envelopes, trajectory planning code, or when running Python commands for this repository.
---

# Grasp Planner Project

## Environment

Use the dedicated conda environment for all project Python commands:

```bash
conda run -n pick python ...
```

Do not use the default `python3` for tests or demos; it may lack required packages such as `scipy`.

## Key Paths

- Technical plan: `aubo_grasp_trajectory_planning_technical_plan.md`
- Main package: `grasp_planner_pkg/src/grasp_planner/`
- Main API: `grasp_planner_pkg/src/grasp_planner/plan_manager.py`
- Data/config types: `grasp_planner_pkg/src/grasp_planner/types.py`
- Demo: `grasp_planner_pkg/examples/demo_plan.py`
- Core tests: `grasp_planner_pkg/tests/test_core.py`
- Obstacle e2e test: `grasp_planner_pkg/tests/test_e2e_obstacle.py`
- Visualization subproject: `grasp_planner_pkg/scripts/simulation/`
- AUBO execution tools: `grasp_planner_pkg/scripts/aubo_test/`

## Standard Commands

Run from `grasp_planner_pkg`:

```bash
conda run -n pick python tests/test_core.py
conda run -n pick python tests/test_e2e_obstacle.py
conda run -n pick python examples/demo_plan.py
```

Run the visualization HTTP service from `grasp_planner_pkg`:

```bash
bash scripts/simulation/serve_noetic_demo.sh
```

## Standard Input

项目标准文件输入在 `grasp_planner_pkg/standard_inputs/`。

- 标准请求文件：`request.txt`
- 标准点云文件：`cloud_B.ply`
- 外部距离、平移、点云坐标统一为 `mm`
- 外部关节角统一为 `degree`
- 标准位姿字段使用 `xyzrpy`：`x y z rx ry rz`
- `rx ry rz` 为固定轴 X-Y-Z 欧拉角，组合矩阵 `R = Rz @ Ry @ Rx`
- 标准输入不需要箱体 ROI 或安全平面；点云中除目标工件挖除区域外全部视为障碍
- 加载器 `src/grasp_planner/text_input.py` 会转换为内部使用的 `m` 和 `rad`
- 现场 AUBO 型号必须通过 `robot_kinematics` 明确；当前支持 `aubo_i5` 和 `aubo_i10`。
- 若原始点云在相机坐标系 `{C}` 下，必须先用 `standard_inputs/transform_camera_cloud_to_base.py`
  转换为基座坐标系 `{B}` 下的 PLY，再让 `request.txt` 的 `cloud_path` 指向转换后的点云。
  Eye-in-Hand 关系为 `T_B_C = T_B_F_photo @ T_F_C`。

文件输入 demo：

```bash
conda run -n pick python examples/demo_plan.py --input standard_inputs/demo_case_001/input/request.txt
```

## Project Model

Treat `grasp_planner_pkg` as the CPU reference implementation of the technical plan:

- `plan_manager.py`: single public orchestration path, `GraspPlanner.plan(PlanRequest) -> PlanResult`
- `segments.py`: goal expansion, Cartesian approach/retreat segments, dense final collision check, time parameterization
- `optimizer.py`: CPU free-segment optimizer using smoothness plus collision penalties with L-BFGS-B
- `kinematics.py`: DH-based FK/Jacobian plus analytic IK and branch-tracking IK
- `robot_model.py`: sphere-fill collision model and attached workpiece handling
- `esdf.py`: point-cloud filtering, target carving, voxel ESDF, safety planes
- `adapters/`: AUBO SDK and cuRobo/nvblox integration skeletons, not production-complete

## Important Boundaries

- `AUBO_I5_DH` and `DEFAULT_GEOMETRY` are placeholder/reference values. Real hardware work must replace them with measured or URDF-derived parameters and rerun the tests.
- The current time parameterization is a CPU reference approximation. Production should use `toppra + ruckig` as described in the technical plan.
- AUBO execution and cuRobo integration are adapter skeletons with TODOs.
- Prefer preserving deterministic behavior; avoid adding randomness unless it is fixed-seed and documented.
