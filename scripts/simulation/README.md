# AUBO-i12H 机械臂可视化与双轨迹回放

这个子项目使用 `aubo_description/urdf/aubo_i12h.urdf` 的本机控制器标定运动学，按 URDF 中的 link/joint 关系装配由 AUBO 官方 i12H STEP 派生的 `aubo_description/meshes/aubo_i12h/*.STL`，与正式 cuRobo 规划使用同一法兰坐标系和同一机器人几何。

## 运行

```bash
cd /home/vecow/workspace/轨迹规划/grasp_planner_pkg
bash scripts/simulation/serve_noetic_demo.sh config/grasp_planner.json
```

启动参数是本次仿真使用的项目 config。页面启动后会从它的 `environment.base_platform` 加载 AGV 基座平台六向尺寸，并从 `end_effector` 加载法兰末端矩形尺寸。省略参数时默认使用 `config/grasp_planner.json`。脚本会自动选择 `127.0.0.1:8088..8999` 中可用的端口，终端会打印带 `config` 参数的访问地址。

终端同时打印两个 URL：普通页面和带 `autoload=latest` 的最新结果页面。打开后者会自动加载当前两阶段 output；普通页面可点击“一键加载最新两阶段 output”完成同样操作。启动脚本使用本项目的只读本地服务器，使浏览器可以读取页面中明确填写的绝对文件路径；仍建议保持默认的 `127.0.0.1` 绑定。

页面里的平台六向尺寸和末端矩形尺寸仍可临时修改。临时修改只影响当前仿真，不回写 config，也不会变更规划和碰撞报告。点击“重新加载启动 config 平台/矩形”可分别恢复对应默认值。

## 第一阶段抓取候选坐标系

页面启动后会自动读取项目相对路径
`output/grasp_generation/grasp_result.json`，并每 `500 ms` 检查内容是否更新。
最多读取按 `score` 降序的10组候选，每组显示以下4个半透明XYZ坐标系：

- `grasp_flange`：轴长 65 mm；
- `grasp_tcp`：轴长 80 mm；
- `trajectory_grasp_flange`：轴长 35 mm；
- `trajectory_grasp_tcp`：轴长 50 mm。

坐标轴保持 X红、Y绿、Z蓝，透明度为35%。页面可分别开关四类坐标系，
也可一键隐藏全部抓取候选。新文件只有在 `candidates`、`score`和40组6数位姿
全部校验通过后才替换画面；如果刚好读到文件写入中间态，页面保留上一版并自动重试。

“显示 → 碰撞球（半透明）”默认关闭；打开后会从 `config/base/device/aubo.json` 加载规划器实际使用的 420 个碰撞球，覆盖显示在 STL 机械臂网格上。球心和半径均为米，随各连杆运动；该开关只用于核对可视化，不会在浏览器中增加碰撞判定。

工件 `collision_box` 和 `point_cloud_exclusion` 可以全部为 0。全零在仿真中表示该几何未启用：页面仍正常加载机械臂、点云和轨迹，但不绘制工件豁免盒，也不将点云标红。负数或“只有部分轴为零”的残缺盒仍会报错。

最新两阶段结果（推荐）：

- “两阶段 output 文件夹”输入框默认填写项目 `output` 的绝对路径；可改为任意一次触发归档的根目录，例如 `.../001_L01_G01_P00_QR00010`。
- 点击“一键加载最新两阶段 output”。
- 页面会并行读取 `grasp_generation/best_grasp.json`、`trajectory_manifest.json`、两条轨迹和 `point_cloud_B.ply`。
- 加载历史归档时还会读取根目录中的 `config_snapshot.yaml` 和 `request_snapshot.txt`，保证工件、平台和末端显示使用该次触发的快照，而不是当前输入。
- 页面显示原始工件 `T_B_O`、阶段1抓取点 `T_B_TCP` 和阶段2法兰目标 `T_B_F`，并在三维场景画出三组 XYZ 坐标轴。
- 页面校验 manifest `ready`、执行契约、`request_id`、`generation_id`、`workpiece_type`、点数、两个 SHA-256 和两段抓取位衔接。任何一项不一致都不会更新回放轨迹。

手动双轨迹模拟：

- 两个轨迹绝对路径输入框默认指向当前 `output/trajectory_planning`；修改后可点击“从绝对路径加载两条轨迹”。
- 在“1. 拍照→抓取”输入框选择 `joint_trajectory_photo_to_grasp.json`。
- 在“2. 抓取→放置”输入框选择 `joint_trajectory_grasp_to_place.json`。
- 点击“加载两条轨迹”，确认帧数和 generation 后，再点击“执行”。
- 两个文件会按“拍照→抓取→抓取位停留 1 秒→放置”连续模拟；不同generation、不同`workpiece_type`，或与当前request工件代号不一致的文件会被拒绝。
- 如果不想加载抓取位姿和点云，可点击“加载最新 manifest 轨迹”，仍会完整校验两条轨迹。

鼠标控制：

- 左键拖拽：旋转视角，左右拖动方向已按当前界面交互调整。
- 滚轮：缩放视角。
- 右键拖拽：整体平移画面内容。

姿态输入：

- 关节角区域支持滑块拖动，也支持直接输入 J1-J6 的角度值，单位 `deg`，点击“执行关节角”后更新模型。
- `flange_link` 位姿区域支持输入它在 `{B}` 基座坐标系下的 `X/Y/Z(mm)` 和 `Rx/Ry/Rz(deg)`，点击执行后通过数值 IK 反算 J1-J6 并更新模型；这里仍是同一个法兰坐标系，不是额外工具坐标系。
- 两个区域的“列表输入”默认收起；打开后可输入六个数值并执行。例如关节角 `[0, -35, 95, 0, 58, 0]`，或法兰姿态 `[55.325, -645.275, 174.801, 179.9978, 18.2600, 80.3021]`。逗号、空格或中文逗号均可作为分隔符。

加载 PLY 点云：

- 方式 1：在绝对路径输入框填写点云文件，点击“从绝对路径加载”；默认值是项目 `output/trajectory_planning/point_cloud_B.ply`。
- 方式 2：在页面的“点云 基座坐标系 / mm”区域选择 `.ply` 文件上传。
- 方式 3：把 PLY 放在工作区里，通过 URL 参数自动加载，例如：

页面同时自动读取项目 `input/request.txt` 的 `workpiece_type`、目标几何中心位姿，以及启动参数 config 中该类型的六向 `point_cloud_exclusion`。加载点云后，正式豁免盒内的点强制标红，盒外点保留原RGB或显示灰色；红色线框、GPU规划挖除和碰撞报告使用同一数据源。

```text
http://127.0.0.1:8088/scripts/simulation/?pointcloud=../../standard_inputs/demo_case_001/input/cloud_B.ply
```

大 PLY 会自动抽稀显示，默认最多显示约 `300000` 点。可以用 `maxPoints` 调整：

```text
http://127.0.0.1:8088/scripts/simulation/?pointcloud=../../standard_inputs/demo_case_001/input/cloud_B.ply&maxPoints=600000
```

## 资源来源

- URDF: `aubo_description/urdf/aubo_i12h.urdf`（GPU 规划模型 `model/aubo_i12h_curobo.yml` 的 `urdf_path` 指向同一文件）
- Mesh: `aubo_description/meshes/aubo_i12h/`
- 运动学快照: `../../../model/aubo_i12h_kinematics.yaml`

`aubo_description` 只保留 i12H 一个机型，其他机型的 URDF 与网格已移除。

## 数据接口

页面加载后会暴露 `window.AuboViz`：

```js
window.AuboViz.setCart({
  negativeX: 400,
  positiveX: 400,
  negativeY: 400,
  positiveY: 400,
  negativeZ: 1000,
  positiveZ: 0,
});

window.AuboViz.setToolExtents({
  negativeX: 70,
  positiveX: 70,
  negativeY: 40,
  positiveY: 40,
  negativeZ: 20,
  positiveZ: 140,
});

window.AuboViz.setPointCloud([
  [100, 200, 300],
  { x: 200, y: 200, z: 350 },
]);

const plyBuffer = await fetch("../../output/trajectory_planning/point_cloud_B.ply").then((res) => res.arrayBuffer());
window.AuboViz.setPointCloudFromPlyBuffer(plyBuffer);

window.AuboViz.setTrajectory({
  unit: "deg",
  points: [
    { t: 0.0, q: [0, -35, 95, 0, 58, 0] },
    { t: 1.2, q: [18, -48, 84, 10, 55, 15] },
  ],
});
window.AuboViz.playTrajectory();

// 与页面按钮相同：加载阶段1、manifest、双轨迹和点云。
await window.AuboViz.loadLatestPipelineOutput();

window.AuboViz.setFlangePose({
  x: 858.4,
  y: -251.1,
  z: 180.2,
  rx: -135.3,
  ry: 23.95,
  rz: 111.88,
});

window.AuboViz.setFlangePose([55.325, -645.275, 174.801, 179.9978, 18.2600, 80.3021]);
```

- `setCart` 中的 `negativeX/positiveX/negativeY/positiveY/negativeZ/positiveZ` 单位是毫米，表示平台从机械臂基座坐标原点沿六个轴向的延伸距离。API 修改只影响本次显示；网页默认平台自动从启动命令指定的 config 加载。
- `setToolExtents` 的六个值单位均为毫米，分别表示矩形从唯一 `flange_link` 原点沿 `-X/+X/-Y/+Y/-Z/+Z` 的延伸距离。矩形与法兰 XYZ 轴平行，不存在额外工具坐标系、中心偏移或 RPY。
- 页面显示基座、J1-J5 子连杆和唯一 `flange_link` 坐标轴；J6 输出端不再额外绘制 `wrist3_Link` 轴。`flange_link` 与 `wrist3_Link` 的机械安装面严格同原点、同方向，不存在额外固定平移或 RPY。`wrist3_Link` 仍完整保留在机器人运动、网格和碰撞模型中，新增末端矩形唯一使用 `flange_link` 的原点和 XYZ 方向。
- `flange_link` 位姿显示和 `setFlangePose` 输入均为它在 `{B}` 基座坐标系下的 `X/Y/Z(mm)` 和 `Rx/Ry/Rz(deg)`；`setFlangePose` 会通过数值 IK 写入 J1-J6。
- 点云坐标系是机械臂基座坐标系 `{B}`，单位毫米；输入 `x,y,z` 分别表示 `{B}` 下的 `X/Y/Z(mm)`。
- 点云文件支持 `.ply`、`.csv`、`.txt`、`.json`。PLY 支持 `ascii`、`binary_little_endian`、`binary_big_endian`，会读取 `vertex` 中的 `x/y/z` 属性，坐标系按机械臂基座坐标系 `{B}`，单位按毫米处理。
- 如果 PLY 顶点有 `red/green/blue`、`r/g/b`、`diffuse_red/diffuse_green/diffuse_blue` 等颜色属性，会按原始 RGB 显示；CSV/JSON 也支持 `x,y,z,r,g,b`。
- 点云颜色的唯一例外是当前目标工件六向豁免盒：盒内点始终覆盖为 `config.yaml` 定义的红色，以明确表示这些点未进入ESDF障碍；页面同时显示同一旋转盒的红色线框。
- 轨迹 `q` 为 J1-J6；`unit` 可取 `"deg"` 或 `"rad"`。正式执行文件为 `aubo_joint_path/v1`，每点只有 `positions(rad)`，不携带速度时序。页面对这类文件使用合成的显示时间做动画，该时间不会回写轨迹，也不代表实机速度。仍兼容仅供诊断/预览的 `trajectory_msgs/JointTrajectory` JSON。
