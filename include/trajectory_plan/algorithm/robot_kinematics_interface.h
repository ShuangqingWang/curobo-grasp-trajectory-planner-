#ifndef HYBRID_TRAJECTORY_PLAN_ROBOT_KINEMATICS_INTERFACE_H
#define HYBRID_TRAJECTORY_PLAN_ROBOT_KINEMATICS_INTERFACE_H

/**
 * @file robot_kinematics_interface.h
 * @brief 六轴机械臂运动学纯接口（算法层叶子）。
 *
 * algorithm/ 不依赖 device/，通过本接口调用 FK / IK / 雅可比；
 * device/ 的机型模型实现本接口，由 stage 经 PipelineContext 注入。
 */

#include <Eigen/Core>

#include <string>
#include <vector>

#include "trajectory_plan/algorithm/pose.h"

namespace openmind::trajectory_plan
{

/**
 * @brief 六轴机械臂运动学抽象：正解、逆解、几何雅可比与关节限位。
 */
class RobotKinematicsInterface
{
  public:
    virtual ~RobotKinematicsInterface() = default;

    /**
     * @brief 正解：六轴关节角（rad）→ 法兰 4x4 齐次变换。
     * @param q 关节角，单位 rad
     * @return 法兰位姿；平移与模型参数同一单位（本项目为 mm）
     */
    virtual Eigen::Matrix4d ForwardKinematics(const JointRadians& q) const = 0;

    /**
     * @brief 逆解：法兰位姿 → 去重后的关节角集合（rad），为空表示无解。
     * @param target 法兰 4x4 齐次变换（mm）
     * @return 最多约 8 组互不重复的解，全部满足关节限位
     */
    virtual std::vector<JointRadians> InverseKinematics(const Eigen::Matrix4d& target) const = 0;

    /**
     * @brief 几何雅可比：6x6，[v; w] = J q_dot。
     * @param q 关节角，单位 rad
     */
    virtual Eigen::Matrix<double, 6, 6> GeometricJacobian(const JointRadians& q) const = 0;

    /**
     * @brief 关节限位下界（rad）。
     */
    virtual JointRadians LowerLimitsRad() const = 0;

    /**
     * @brief 关节限位上界（rad）。
     */
    virtual JointRadians UpperLimitsRad() const = 0;

    /**
     * @brief 检查 q 是否在限位内（rad，含数值裕量）。
     */
    bool WithinLimits(const JointRadians& q) const;

    /**
     * @brief 机型号（如 "aubo_i12H"）。
     */
    virtual const std::string& RobotType() const = 0;
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_ROBOT_KINEMATICS_INTERFACE_H
