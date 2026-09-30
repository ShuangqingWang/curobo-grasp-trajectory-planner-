/**
 * @file collision_world_build_algorithm.cpp
 * @brief 碰撞几何基础类型的实现（见头文件说明：碰撞世界与端点预筛已由改动五、十一删除）。
 */

#include "trajectory_plan/algorithm/collision_world_build_algorithm.h"

namespace openmind::trajectory_plan
{

bool AxisAlignedBox::Contains(const Eigen::Vector3d& point) const
{
    return point.x() >= -negative_extent.x() && point.x() <= positive_extent.x() &&
           point.y() >= -negative_extent.y() && point.y() <= positive_extent.y() &&
           point.z() >= -negative_extent.z() && point.z() <= positive_extent.z();
}

} // namespace openmind::trajectory_plan
