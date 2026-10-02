# 本体碰撞球生成（FOAM 中轴球化 + 限外溢缩半径）

生成 i12H 本体 7 个连杆的碰撞球，同时写入 GPU 规划模型与 C++ 设备配置。当前采用结果：**191 个本体球**。
原理与评测见《轨迹规划逻辑分析_发布第二版优化方案》2.5.8 与附录 A.7。

## 方案

1. **FOAM 中轴球化**：球心沿连杆中轴排布，从内部把连杆罩住。各连杆目标球数见 `common.py` 的 `FOAM_BRANCH`。
2. **限外溢缩半径**：球心、球数不动，只缩半径 `r' = min(r, d + 外溢上限)`，d 为球心到连杆表面的距离。
   外溢上限取原 cuRobo 贴面球的外凸 P95（`BULGE_CAP_MM`），保证新球不比原来更“胖”；覆盖率不作硬性要求。
3. **base_link 不重新球化**：沿用已去掉探入工作台球的 17 个球（FOAM 的底座球会探入实心工作台）。

| 连杆 | 外溢上限 mm | 球数 |
| --- | ---: | ---: |
| base_link | —（沿用） | 17 |
| shoulder_Link | 11.2 | 30 |
| upperArm_Link | 11.9 | 37 |
| foreArm_Link | 10.8 | 33 |
| wrist1_Link | 8.0 | 28 |
| wrist2_Link | 7.9 | 28 |
| wrist3_Link | 7.3 | 18 |
| **合计** | | **191** |

## 文件

| 文件 | 作用 |
| --- | --- |
| `common.py` | 路径、连杆、FOAM 目标球数、外溢上限 |
| `run_foam.py` | 第 1 步：FOAM 球化 → `data/foam_half_raw.json`（需 FOAM 环境，约 20 分钟） |
| `shrink_radius.py` | 第 2 步：缩半径 → `data/body_spheres_191.json` |
| `apply_spheres.py` | 第 3 步：写入 `model/aubo_i12h_curobo.yml` 与 `config/base/device/aubo.json` 并逐球核对；`--check` 只核对 |
| `eval_spheres.py` | 评测表面覆盖、缝隙、体积覆盖、外凸 |
| `selfcoll_check.py` | 用归档里的真实轨迹构型检查自碰撞 |
| `draw_compare.py` | 出对比图到 `output/本体碰撞球对比/` |
| `data/foam_half_raw.json` | FOAM 原始输出（已固化，复现第 2、3 步无需重跑 FOAM） |
| `data/body_spheres_191.json` | 当前采用的本体球 |
| `data/body_spheres_387_curobo.json` | 原 cuRobo 贴面球，用于对比 |

## 环境

第 2～7 步用项目环境即可：`/home/vecow/miniconda3/envs/pick-gpu/bin/python`。

第 1 步需要 FOAM（不放进项目）：

```bash
git clone --recursive https://github.com/CoMMALab/foam && cd foam && git checkout 116928f
cmake -Bbuild -DCMAKE_BUILD_TYPE=Release . && cmake --build build -j8
conda create -p ../env python=3.11 && ../env/bin/pip install numpy scipy trimesh==4.5.3 fire termcolor xmltodict rtree networkx
```

## 流程

```bash
cd scripts/本体碰撞球生成
# 1. （可选）重跑 FOAM；FOAM 结果不保证逐位可复现，已固化的 data/foam_half_raw.json 为准
../../../foam_eval/env/bin/python run_foam.py --foam-dir /home/vecow/workspace/轨迹规划/foam_eval/foam
# 2. 缩半径
PY=/home/vecow/miniconda3/envs/pick-gpu/bin/python
$PY shrink_radius.py
# 3. 写入两份配置并核对
$PY apply_spheres.py
# 4. 检查自碰撞、评测、出图
$PY selfcoll_check.py /var/hybird_log/grasp_log/<会话目录>
$PY eval_spheres.py
$PY draw_compare.py
```

更换本体球后必须重启服务。启动时 C++ 与 GPU 的本体球数不一致会直接报错。
