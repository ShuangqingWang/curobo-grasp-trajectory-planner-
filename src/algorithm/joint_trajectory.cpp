/**
 * @file joint_trajectory.cpp
 * @brief 关节轨迹的累计转角统计实现（评级与 D 级判定共用）。
 */

#include "trajectory_plan/algorithm/joint_trajectory.h"

#include <algorithm>
#include <cmath>

namespace openmind::trajectory_plan
{

JointRadians JointTrajectory::AccumulatedTravelRad() const
{
    JointRadians travel {};
    if (points.size() < 2)
    {
        return travel;
    }
    for (size_t index = 1; index < points.size(); ++index)
    {
        for (int64_t joint = 0; joint < 6; ++joint)
        {
            travel[static_cast<size_t>(joint)] +=
                std::abs(WrapRadians(points[index][static_cast<size_t>(joint)] -
                                     points[index - 1][static_cast<size_t>(joint)]));
        }
    }
    return travel;
}

JointRadians JointTrajectory::NetJointTravelRad() const
{
    JointRadians travel {};
    if (points.size() < 2)
    {
        return travel;
    }
    // 首末角度差的绝对值，保留实际圈数：不 wrap，不取模。
    for (int64_t joint = 0; joint < 6; ++joint)
    {
        const size_t index = static_cast<size_t>(joint);
        travel[index] = std::abs(points.back()[index] - points.front()[index]);
    }
    return travel;
}

double JointTrajectory::MaxNetJointTravelRad() const
{
    const JointRadians travel = NetJointTravelRad();
    double maximum = 0.0;
    for (int64_t joint = 0; joint < 6; ++joint)
    {
        maximum = std::max(maximum, travel[static_cast<size_t>(joint)]);
    }
    return maximum;
}

int32_t JointTrajectory::MaxNetJointTravelIndex() const
{
    if (points.size() < 2)
    {
        return -1;
    }
    const JointRadians travel = NetJointTravelRad();
    int32_t best = 0;
    for (int64_t joint = 1; joint < 6; ++joint)
    {
        if (travel[static_cast<size_t>(joint)] > travel[static_cast<size_t>(best)])
        {
            best = static_cast<int32_t>(joint);
        }
    }
    return best;
}

double JointTrajectory::TotalWristJointTravelRad() const
{
    const JointRadians travel = AccumulatedTravelRad();
    return travel[3] + travel[4] + travel[5];
}

} // namespace openmind::trajectory_plan
