#ifndef HYBRID_TRAJECTORY_PLAN_POSE_H
#define HYBRID_TRAJECTORY_PLAN_POSE_H

/**
 * @file pose.h
 * @brief 位姿纯类型与 SE(3) 工具：平移 mm、角度 degree、固定轴 XYZ 欧拉。
 *
 * 全项目统一：R = Rz(rz) @ Ry(ry) @ Rx(rx)，平移单位 mm，角度单位 degree。
 * 矩阵为 4x4 齐次（Eigen::Matrix4d），最后一行固定为 [0,0,0,1]。
 */

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <string>

namespace openmind::trajectory_plan
{

/**
 * @brief xyzrxryrz 位姿：平移 mm，固定轴 XYZ 欧拉角 degree。
 */
struct Pose
{
    double x = 0.0;  ///< 基座系平移 x，单位 mm
    double y = 0.0;  ///< 基座系平移 y，单位 mm
    double z = 0.0;  ///< 基座系平移 z，单位 mm
    double rx = 0.0; ///< 绕 X 轴旋转，单位 degree
    double ry = 0.0; ///< 绕 Y 轴旋转，单位 degree
    double rz = 0.0; ///< 绕 Z 轴旋转，单位 degree
};

/** @brief 六轴关节角，单位 degree。 */
using JointDegrees = std::array<double, 6>;

/** @brief 六轴关节角，单位 rad。 */
using JointRadians = std::array<double, 6>;

/**
 * @brief 由固定轴 XYZ 欧拉角（degree）构造旋转矩阵。
 * @return R = Rz(rz) @ Ry(ry) @ Rx(rx)
 */
Eigen::Matrix3d RotationFromXyzDeg(double rx, double ry, double rz);

/**
 * @brief 由旋转矩阵反解一组主值欧拉角（degree，固定轴 XYZ）。
 * @note 接近万向节锁时 rz 固定为 0，rx/ry 取主值。
 */
Eigen::Vector3d XyzDegFromRotation(const Eigen::Matrix3d& rotation);

/**
 * @brief 由 xyzrxryrz（mm/deg）构造 4x4 齐次变换。
 */
Eigen::Matrix4d TransformFromPose(const Pose& pose);

/**
 * @brief 由 4x4 齐次变换反解 Pose（mm/deg）。
 */
Pose PoseFromTransform(const Eigen::Matrix4d& transform);

/**
 * @brief 校验 4x4 齐次变换为合法右手系刚体变换（最后一行、正交、行列式 +1）。
 * @param name 用于错误信息的字段名
 * @param error 失败原因（成功时为空）
 * @return 校验通过返回 true
 */
bool ValidateTransform(const Eigen::Matrix4d& transform, const std::string& name, std::string& error);

/**
 * @brief 把角度折算到 [-180, 180)。
 */
double WrapDegrees(double degrees);

/**
 * @brief 把弧度折算到 [-pi, pi)。
 */
double WrapRadians(double radians);

/**
 * @brief 两个 6 轴关节角（rad）逐轴 wrap 后的最大差值。
 */
double MaxWrappedJointGapRad(const JointRadians& left, const JointRadians& right);

/**
 * @brief 把 xyzrxryrz（mm/deg）格式化为固定宽度的字符串，供日志与 JSON 使用。
 */
std::string FormatPose(const Pose& pose);

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_POSE_H
