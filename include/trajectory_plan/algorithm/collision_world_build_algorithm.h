#ifndef HYBRID_TRAJECTORY_PLAN_COLLISION_WORLD_BUILD_ALGORITHM_H
#define HYBRID_TRAJECTORY_PLAN_COLLISION_WORLD_BUILD_ALGORITHM_H

/**
 * @file collision_world_build_algorithm.h
 * @brief 碰撞几何基础类型：连杆碰撞球与轴对齐箱体。
 *
 * @note 本文件原先还包含 C++ 侧的碰撞世界（占据栅格 + 带孔平台分解 + 端点预筛）。
 *       优化方案改动五与改动十一把它们整体删除：
 *       - **改动五**：GPU 完整模型已把末端碰撞球挂在 flange_link 上，抓取段与放置段
 *         沿途都在检查末端 vs 点云 + 平台，终点只是沿途的一个点，端点预筛是重复步骤。
 *         为那 0.45ms 的预筛每轮要建 9.3M 格占据栅格（约 455ms），已删除。
 *       - **改动十一**：平台不再分解为长方体下发 GPU，而是按体素并入同一张距离场，
 *         故 InstallationExclusion / WorldBox / BuildPlatformBoxes 一并作废。
 *       此处只保留仍被设备模型与固定模型快照使用的两个几何类型。
 */

#include <Eigen/Core>

#include <cstdint>

namespace openmind::trajectory_plan
{

/**
 * @brief 附着在某一连杆上的碰撞球（连杆局部坐标，mm）。
 */
struct RobotCollisionSphere
{
    int32_t link_index = 0;      ///< 连杆序号 0..6（0 = base）
    Eigen::Vector3d center = Eigen::Vector3d::Zero(); ///< 连杆局部坐标（mm）
    double radius = 0.0;         ///< 球半径（mm）
};

/**
 * @brief 轴对齐箱体（本项目法兰末端矩形与基座平台均为轴对齐六向延伸）。
 */
struct AxisAlignedBox
{
    Eigen::Vector3d negative_extent = Eigen::Vector3d::Zero(); ///< 相对原点向 -X/-Y/-Z 延伸（mm）
    Eigen::Vector3d positive_extent = Eigen::Vector3d::Zero(); ///< 相对原点向 +X/+Y/+Z 延伸（mm）

    /** @brief 某轴总尺寸（mm）。 */
    double Size(int32_t axis) const
    {
        return negative_extent[axis] + positive_extent[axis];
    }

    /** @brief 是否退化（六向全零）。 */
    bool IsDegenerate() const
    {
        return Size(0) <= 0.0 && Size(1) <= 0.0 && Size(2) <= 0.0;
    }

    /** @brief 点是否在箱内（闭区间）。 */
    bool Contains(const Eigen::Vector3d& point) const;
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_COLLISION_WORLD_BUILD_ALGORITHM_H
