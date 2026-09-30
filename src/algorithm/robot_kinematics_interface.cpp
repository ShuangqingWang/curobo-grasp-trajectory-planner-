/**
 * @file robot_kinematics_interface.cpp
 * @brief 运动学接口的公共实现（限位检查）。
 */

#include "trajectory_plan/algorithm/robot_kinematics_interface.h"

#include <cmath>

namespace openmind::trajectory_plan
{

bool RobotKinematicsInterface::WithinLimits(const JointRadians& q) const
{
    // 1e-9 rad 数值裕量：IK 收敛解常贴着限位边缘。
    constexpr double kToleranceRad = 1e-9;
    const JointRadians lower = LowerLimitsRad();
    const JointRadians upper = UpperLimitsRad();
    for (int64_t index = 0; index < 6; ++index)
    {
        const double value = q[static_cast<size_t>(index)];
        if (value < lower[static_cast<size_t>(index)] - kToleranceRad ||
            value > upper[static_cast<size_t>(index)] + kToleranceRad)
        {
            return false;
        }
    }
    return true;
}

} // namespace openmind::trajectory_plan
