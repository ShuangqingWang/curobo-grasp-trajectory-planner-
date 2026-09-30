/**
 * @file pose.cpp
 * @brief SE(3) 工具实现：固定轴 XYZ 欧拉（mm/degree）、变换校验、角度折算。
 */

#include "trajectory_plan/algorithm/pose.h"

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>

namespace openmind::trajectory_plan
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesPerRadian = 180.0 / kPi;
constexpr double kRadiansPerDegree = kPi / 180.0;

constexpr double kRotationOrthogonalityTolerance = 2e-3;
constexpr double kRotationDeterminantTolerance = 2e-3;

} // namespace

Eigen::Matrix3d RotationFromXyzDeg(double rx, double ry, double rz)
{
    const double cx = std::cos(rx * kRadiansPerDegree);
    const double sx = std::sin(rx * kRadiansPerDegree);
    const double cy = std::cos(ry * kRadiansPerDegree);
    const double sy = std::sin(ry * kRadiansPerDegree);
    const double cz = std::cos(rz * kRadiansPerDegree);
    const double sz = std::sin(rz * kRadiansPerDegree);

    Eigen::Matrix3d rx_matrix = Eigen::Matrix3d::Identity();
    rx_matrix(1, 1) = cx;
    rx_matrix(1, 2) = -sx;
    rx_matrix(2, 1) = sx;
    rx_matrix(2, 2) = cx;

    Eigen::Matrix3d ry_matrix = Eigen::Matrix3d::Identity();
    ry_matrix(0, 0) = cy;
    ry_matrix(0, 2) = sy;
    ry_matrix(2, 0) = -sy;
    ry_matrix(2, 2) = cy;

    Eigen::Matrix3d rz_matrix = Eigen::Matrix3d::Identity();
    rz_matrix(0, 0) = cz;
    rz_matrix(0, 1) = -sz;
    rz_matrix(1, 0) = sz;
    rz_matrix(1, 1) = cz;

    // 固定轴 XYZ：R = Rz @ Ry @ Rx。
    return rz_matrix * ry_matrix * rx_matrix;
}

Eigen::Vector3d XyzDegFromRotation(const Eigen::Matrix3d& rotation)
{
    // R = Rz @ Ry @ Rx 的一个主值解；接近万向节锁时 rz 固定为 0。
    const double pitch = std::asin(std::max(-1.0, std::min(1.0, -rotation(2, 0))));
    double roll = 0.0;
    double yaw = 0.0;
    if (std::abs(std::cos(pitch)) > 1e-7)
    {
        roll = std::atan2(rotation(2, 1), rotation(2, 2));
        yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    }
    else
    {
        roll = std::atan2(-rotation(0, 1), rotation(1, 1));
        yaw = 0.0;
    }
    return Eigen::Vector3d(roll * kDegreesPerRadian, pitch * kDegreesPerRadian, yaw * kDegreesPerRadian);
}

Eigen::Matrix4d TransformFromPose(const Pose& pose)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = RotationFromXyzDeg(pose.rx, pose.ry, pose.rz);
    transform(0, 3) = pose.x;
    transform(1, 3) = pose.y;
    transform(2, 3) = pose.z;
    return transform;
}

Pose PoseFromTransform(const Eigen::Matrix4d& transform)
{
    Pose pose;
    pose.x = transform(0, 3);
    pose.y = transform(1, 3);
    pose.z = transform(2, 3);
    const Eigen::Vector3d angles = XyzDegFromRotation(transform.block<3, 3>(0, 0));
    pose.rx = angles.x();
    pose.ry = angles.y();
    pose.rz = angles.z();
    return pose;
}

bool ValidateTransform(const Eigen::Matrix4d& transform, const std::string& name, std::string& error)
{
    if (!transform.allFinite())
    {
        error = name + " 必须全部为有限数值";
        return false;
    }
    for (int64_t column = 0; column < 4; ++column)
    {
        const double expected = (column == 3) ? 1.0 : 0.0;
        if (std::abs(transform(3, static_cast<int>(column)) - expected) > 1e-7)
        {
            error = name + " 最后一行必须为 [0, 0, 0, 1]";
            return false;
        }
    }
    const Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    if ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() >
        kRotationOrthogonalityTolerance)
    {
        error = name + " 的旋转部分不是正交矩阵";
        return false;
    }
    if (std::abs(rotation.determinant() - 1.0) > kRotationDeterminantTolerance)
    {
        error = name + " 的旋转部分行列式必须为 +1";
        return false;
    }
    return true;
}

double WrapDegrees(double degrees)
{
    double wrapped = std::fmod(degrees, 360.0);
    if (wrapped >= 180.0)
    {
        wrapped -= 360.0;
    }
    else if (wrapped < -180.0)
    {
        wrapped += 360.0;
    }
    return wrapped;
}

double WrapRadians(double radians)
{
    return WrapDegrees(radians * kDegreesPerRadian) * kRadiansPerDegree;
}

double MaxWrappedJointGapRad(const JointRadians& left, const JointRadians& right)
{
    double maximum = 0.0;
    for (int64_t index = 0; index < 6; ++index)
    {
        maximum = std::max(maximum, std::abs(WrapRadians(left[static_cast<size_t>(index)] -
                                                         right[static_cast<size_t>(index)])));
    }
    return maximum;
}

std::string FormatPose(const Pose& pose)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4);
    stream << "xyz=[" << pose.x << ", " << pose.y << ", " << pose.z << "] "
           << "rxryrz=[" << pose.rx << ", " << pose.ry << ", " << pose.rz << "]";
    return stream.str();
}

} // namespace openmind::trajectory_plan
