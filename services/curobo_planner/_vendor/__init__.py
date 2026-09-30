"""从 src/grasp_planner 抽取的最小依赖子集，仅供 cuRobo 规划服务使用。

这些文件取自 git HEAD 的原始实现，已做的改动仅限：
  - nvblox_curobo_bridge：相对 import 降一层（``..x`` -> ``.x``）
  - curobo_adapter：collision_cache 的体素尺寸/范围改为构造参数（原写死 3m@5mm）

不要在此处新增业务逻辑；需要改动请回到上游模块再同步过来。

注意：nvblox_curobo_bridge 依赖旧版 request.txt 格式（单路 depth_path、
内联 grasp_candidate_*），当前 C++ 流程已不产生该格式，故本服务不使用它，
碰撞世界改由 C++ 送来的融合点云构建（见 curobo_plan_service.py）。
"""
