#ifndef HYBRID_TRAJECTORY_PLAN_AUBO_I12H_MODEL_H
#define HYBRID_TRAJECTORY_PLAN_AUBO_I12H_MODEL_H

/**
 * @file aubo_i12h_model.h
 * @brief AUBO-i12H 机型模型：修改 DH 正解 / 几何雅可比 / 多种子数值逆解（去重上限）。
 *
 * 参数来自 model/aubo_i12h_kinematics.yaml 的 real 组（控制器出厂补偿值，
 * 与控制器同构：Rx(alpha) Tx(a) Rz(theta+q) Tz(d)，单位 m/rad；内部换算 mm）。
 */

#include <nlohmann/json.hpp>
#include <Eigen/Core>

#include <array>
#include <string>
#include <vector>

#include "trajectory_plan/algorithm/robot_kinematics_interface.h"

namespace openmind::trajectory_plan
{

/**
 * @brief AUBO-i12H 修改 DH 机型模型。
 */
class AuboI12hModel : public RobotKinematicsInterface
{
  public:
    AuboI12hModel() = default;
    AuboI12hModel(const AuboI12hModel&) = delete;
    AuboI12hModel& operator=(const AuboI12hModel&) = delete;

    /**
     * @brief 用 device/aubo.json 的 mdh 节初始化。
     * @param mdh 修改 DH 参数：a / alpha / d / theta（长度 m，角度 rad）
     * @param lower_deg 六轴下界（degree）
     * @param upper_deg 六轴上界（degree）
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true
     */
    bool Init(const nlohmann::json& mdh,
              const std::array<double, 6>& lower_deg,
              const std::array<double, 6>& upper_deg,
              std::string& error);

    /**
     * @brief 正解：q（rad）→ 法兰 4x4（mm）。
     */
    Eigen::Matrix4d ForwardKinematics(const JointRadians& q) const override;

    /**
     * @brief 逆解：法兰（mm）→ 去重关节角（rad），最多约 8 组。
     * @note 确定性多种子阻尼最小二乘（与现网 Python ModifiedDHKinematics 同源），
     *       解全部满足限位。
     */
    std::vector<JointRadians> InverseKinematics(const Eigen::Matrix4d& target) const override;

    /**
     * @brief 几何雅可比（6x6）。
     */
    Eigen::Matrix<double, 6, 6> GeometricJacobian(const JointRadians& q) const override;

    /**
     * @brief 全部连杆坐标系（7 个 4x4，mm）：frames_[i] = T_0(i)，frames_[0]=I。
     */
    std::vector<Eigen::Matrix4d> ForwardKinematicsFrames(const JointRadians& q) const;

    /**
     * @brief 关节限位下界（rad）。
     */
    JointRadians LowerLimitsRad() const override;

    /**
     * @brief 关节限位上界（rad）。
     */
    JointRadians UpperLimitsRad() const override;

    /**
     * @brief 机型号。
     */
    const std::string& RobotType() const override;

  private:
    /**
     * @brief 修改 DH 固定段：Rx(alpha) Tx(a) Rz(theta) Tz(d)。
     */
    Eigen::Matrix4d FixedTransform(int32_t index) const;

    /**
     * @brief 阻尼最小二乘分支保持迭代（由种子出发）。
     */
    bool IterateIk(const Eigen::Matrix4d& target, const JointRadians& seed, JointRadians& solution) const;

    std::array<double, 6> a_ {};     ///< MDH a（mm）
    std::array<double, 6> alpha_ {}; ///< MDH alpha（rad）
    std::array<double, 6> d_ {};     ///< MDH d（mm）
    std::array<double, 6> theta_ {}; ///< MDH theta 固定角（rad）
    std::array<double, 6> lower_rad_ {}; ///< 限位下界（rad）
    std::array<double, 6> upper_rad_ {}; ///< 限位上界（rad）
    std::string robot_type_ = "aubo_i12H"; ///< 机型号
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_AUBO_I12H_MODEL_H
