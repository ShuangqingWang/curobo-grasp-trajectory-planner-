#ifndef HYBRID_TRAJECTORY_PLAN_JOINT_TRAJECTORY_H
#define HYBRID_TRAJECTORY_PLAN_JOINT_TRAJECTORY_H

/**
 * @file joint_trajectory.h
 * @brief 关节轨迹纯类型：六轴关节点序列 + 采样间隔，单位 rad / s。
 */

#include <array>
#include <cstdint>
#include <vector>

#include "trajectory_plan/algorithm/pose.h"

namespace openmind::trajectory_plan
{

/**
 * @brief 一条关节轨迹：等间隔六轴位置点序列。
 */
struct JointTrajectory
{
    std::vector<JointRadians> points; ///< 六轴关节角（rad），首点为起点、末点为终点
    double time_step_s = 0.025;       ///< 相邻点采样间隔，单位 s

    /** @brief 轨迹点数。 */
    int64_t PointCount() const
    {
        return static_cast<int64_t>(points.size());
    }

    /** @brief 是否有效（至少两个点）。 */
    bool IsValid() const
    {
        return points.size() >= 2;
    }

    /**
     * @brief 单轴「净转角」（rad）：D_j = |q_j(终点) − q_j(起点)|，j = 1…6。
     *
     * 按方向相加、只统计首末净转角，六轴全部参与，使用保留实际圈数的连续关节角，
     * 不按 360° 取模。某轴先 +100° 再 +100° 净转角为 200°；先 +100° 再 −80°
     * 净转角为 20°。本指标不统计中途运动范围或往返路程：先 +200° 再 −200°
     * 净转角为 0°，因此不能用它判断所有中途绕行或冗余往返。
     */
    JointRadians NetJointTravelRad() const;

    /**
     * @brief 六轴净转角的最大值（rad），即评级指标 D = max(D_1 … D_6)。
     */
    double MaxNetJointTravelRad() const;

    /**
     * @brief 净转角最大的关节序号（0 起；轨迹无效时返回 -1）。
     */
    int32_t MaxNetJointTravelIndex() const;

    /** @brief 单轴累计转角（rad）：对每一轴求和 |dq_i|。仅供诊断，不参与评级。 */
    JointRadians AccumulatedTravelRad() const;

    /** @brief 腕部（J4/J5/J6）三轴累计转角之和（rad）。仅供诊断。 */
    double TotalWristJointTravelRad() const;
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_JOINT_TRAJECTORY_H
