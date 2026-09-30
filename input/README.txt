实时规划输入目录
================

唯一正式路径为 grasp_planner_pkg/input/。

只放本次规划所需的统一输入文件：

  request.txt          工件代号与本次工件位姿 T_B_O_target
  raw_depth.tiff       主相机单通道 float32 TIFF，像素单位 mm
  raw_depth_left.tiff  左相机同步深度图
  raw_depth_right.tiff 右相机同步深度图
  raw_color.png        主相机 RGB 图（仅供后台彩色点云可视化）

固定工位、相机、机型、末端和基座在 config/grasp_planner.json（及 config/base/）。
触发走 TCP，不再使用 plan.trigger.json。

启动：

  ./build/OpenmindTrajectoryPlan config/grasp_planner.json

触发：

  ./build/OpenmindTrajectoryTrigger --input-dir input
