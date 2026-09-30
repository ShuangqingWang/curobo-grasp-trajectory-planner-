/**
 * @file aubo_i12h_model.cpp
 * @brief AUBO-i12H 修改 DH 模型实现：FK / 几何雅可比 / 确定性多种子阻尼最小二乘 IK。
 *
 * 与现网 Python ModifiedDHKinematics 同源：固定段 Rx(alpha) Tx(a) Rz(theta) Tz(d)，
 * IK 用 10 组固定种子 + 阻尼最小二乘分支保持迭代 + 去重，收敛行为一致。
 */

#include "trajectory_plan/device/aubo_i12h_model.h"

#include <cmath>

#include "trajectory_plan/tools/log.h"

namespace openmind::trajectory_plan
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadiansPerDegree = kPi / 180.0;
constexpr double kMetersToMillimeters = 1000.0;

/// IK 收敛位置容差（mm；与 Python 综合容差 |e|²<1e-12 的米制效果一致）。
constexpr double kIkPositionToleranceMm = 1e-3;
/// IK 收敛姿态容差（rad）。
constexpr double kIkRotationToleranceRad = 1e-6;
/// IK 最大迭代次数。
constexpr int64_t kIkMaxIterations = 200;
/// 阻尼最小二乘的阻尼系数。
constexpr double kIkDamping = 1e-5;
/// 解去重阈值（rad）。
constexpr double kIkDedupToleranceRad = 1e-5;

/**
 * @brief SO(3) 对数映射 → 轴角向量。
 */
Eigen::Vector3d RotationLog(const Eigen::Matrix3d& rotation)
{
    const double cosine = std::max(-1.0, std::min(1.0, (rotation.trace() - 1.0) * 0.5));
    const double angle = std::acos(cosine);
    if (angle < 1e-10)
    {
        return Eigen::Vector3d::Zero();
    }
    const double scale = angle / (2.0 * std::sin(angle));
    return scale * Eigen::Vector3d(rotation(2, 1) - rotation(1, 2),
                                   rotation(0, 2) - rotation(2, 0),
                                   rotation(1, 0) - rotation(0, 1));
}

} // namespace

bool AuboI12hModel::Init(const nlohmann::json& mdh,
                         const std::array<double, 6>& lower_deg,
                         const std::array<double, 6>& upper_deg,
                         std::string& error)
{
    if (!mdh.is_object())
    {
        error = "aubo.json 缺少 mdh 节";
        return false;
    }
    for (const std::string& key : {"a", "alpha", "d", "theta"})
    {
        if (!mdh.contains(key) || !mdh[key].is_array() || mdh[key].size() != 6)
        {
            error = "aubo.json mdh." + key + " 必须是 6 个数";
            return false;
        }
    }
    for (int64_t index = 0; index < 6; ++index)
    {
        a_[static_cast<size_t>(index)] = mdh["a"][static_cast<size_t>(index)].get<double>() * kMetersToMillimeters;
        alpha_[static_cast<size_t>(index)] = mdh["alpha"][static_cast<size_t>(index)].get<double>();
        d_[static_cast<size_t>(index)] = mdh["d"][static_cast<size_t>(index)].get<double>() * kMetersToMillimeters;
        theta_[static_cast<size_t>(index)] = mdh["theta"][static_cast<size_t>(index)].get<double>();
        lower_rad_[static_cast<size_t>(index)] = lower_deg[static_cast<size_t>(index)] * kRadiansPerDegree;
        upper_rad_[static_cast<size_t>(index)] = upper_deg[static_cast<size_t>(index)] * kRadiansPerDegree;
        if (lower_rad_[static_cast<size_t>(index)] >= upper_rad_[static_cast<size_t>(index)])
        {
            error = "aubo.json 关节限位下界必须小于上界";
            return false;
        }
    }
    return true;
}

Eigen::Matrix4d AuboI12hModel::FixedTransform(int32_t index) const
{
    // 固定段：Rx(alpha) Tx(a) Rz(theta) Tz(d)，只依赖标定参数。
    const double ca = std::cos(alpha_[static_cast<size_t>(index)]);
    const double sa = std::sin(alpha_[static_cast<size_t>(index)]);
    const double ct = std::cos(theta_[static_cast<size_t>(index)]);
    const double st = std::sin(theta_[static_cast<size_t>(index)]);

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform(0, 0) = ct;
    transform(0, 1) = -st;
    transform(0, 3) = a_[static_cast<size_t>(index)];
    transform(1, 0) = st * ca;
    transform(1, 1) = ct * ca;
    transform(1, 2) = -sa;
    transform(1, 3) = -sa * d_[static_cast<size_t>(index)];
    transform(2, 0) = st * sa;
    transform(2, 1) = ct * sa;
    transform(2, 2) = ca;
    transform(2, 3) = ca * d_[static_cast<size_t>(index)];
    return transform;
}

std::vector<Eigen::Matrix4d> AuboI12hModel::ForwardKinematicsFrames(const JointRadians& q) const
{
    std::vector<Eigen::Matrix4d> frames;
    frames.reserve(7);
    frames.push_back(Eigen::Matrix4d::Identity());
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    for (int64_t index = 0; index < 6; ++index)
    {
        Eigen::Matrix4d joint_frame = transform * FixedTransform(static_cast<int32_t>(index));
        const double cosine = std::cos(q[static_cast<size_t>(index)]);
        const double sine = std::sin(q[static_cast<size_t>(index)]);
        Eigen::Matrix4d joint_rotation = Eigen::Matrix4d::Identity();
        joint_rotation(0, 0) = cosine;
        joint_rotation(0, 1) = -sine;
        joint_rotation(1, 0) = sine;
        joint_rotation(1, 1) = cosine;
        transform = joint_frame * joint_rotation;
        frames.push_back(transform);
    }
    return frames;
}

Eigen::Matrix4d AuboI12hModel::ForwardKinematics(const JointRadians& q) const
{
    return ForwardKinematicsFrames(q)[6];
}

Eigen::Matrix<double, 6, 6> AuboI12hModel::GeometricJacobian(const JointRadians& q) const
{
    // 与 Python ModifiedDHKinematics 一致：轴/原点取自 joint_frame（应用 q_i 前）。
    Eigen::Matrix<double, 6, 6> jacobian = Eigen::Matrix<double, 6, 6>::Zero();
    const std::vector<Eigen::Matrix4d> frames = ForwardKinematicsFrames(q);
    const Eigen::Vector3d end_position = frames[6].block<3, 1>(0, 3);
    for (int64_t index = 0; index < 6; ++index)
    {
        const Eigen::Matrix4d joint_frame = frames[static_cast<size_t>(index)] * FixedTransform(static_cast<int32_t>(index));
        const Eigen::Vector3d axis = joint_frame.block<3, 1>(0, 2);
        const Eigen::Vector3d origin = joint_frame.block<3, 1>(0, 3);
        jacobian.block<3, 1>(0, static_cast<int>(index)) = axis.cross(end_position - origin);
        jacobian.block<3, 1>(3, static_cast<int>(index)) = axis;
    }
    return jacobian;
}

bool AuboI12hModel::IterateIk(const Eigen::Matrix4d& target,
                              const JointRadians& seed,
                              JointRadians& solution) const
{
    JointRadians q = seed;
    Eigen::Vector3d position_error = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotation_error = Eigen::Vector3d::Zero();
    bool converged = false;
    for (int64_t iteration = 0; iteration < kIkMaxIterations; ++iteration)
    {
        const std::vector<Eigen::Matrix4d> frames = ForwardKinematicsFrames(q);
        const Eigen::Matrix4d current = frames[6];
        position_error = target.block<3, 1>(0, 3) - current.block<3, 1>(0, 3);
        const Eigen::Matrix3d rotation_difference = target.block<3, 3>(0, 0) * current.block<3, 3>(0, 0).transpose();
        rotation_error = RotationLog(rotation_difference);
        if (position_error.norm() < kIkPositionToleranceMm && rotation_error.norm() < kIkRotationToleranceRad)
        {
            converged = true;
            break;
        }
        Eigen::Matrix<double, 6, 1> error_vector;
        error_vector.block<3, 1>(0, 0) = position_error;
        error_vector.block<3, 1>(3, 0) = rotation_error;
        const Eigen::Matrix<double, 6, 6> jacobian = GeometricJacobian(q);
        const Eigen::Matrix<double, 6, 6> system =
            jacobian.transpose() * jacobian + kIkDamping * Eigen::Matrix<double, 6, 6>::Identity();
        const Eigen::Matrix<double, 6, 1> delta = system.ldlt().solve(jacobian.transpose() * error_vector);
        if (!delta.allFinite())
        {
            break;
        }
        for (int64_t index = 0; index < 6; ++index)
        {
            q[static_cast<size_t>(index)] += delta[static_cast<int>(index)];
        }
    }
    if (!converged)
    {
        return false;
    }
    for (int64_t index = 0; index < 6; ++index)
    {
        q[static_cast<size_t>(index)] = WrapRadians(q[static_cast<size_t>(index)]);
    }
    if (!WithinLimits(q))
    {
        return false;
    }
    solution = q;
    return true;
}

std::vector<JointRadians> AuboI12hModel::InverseKinematics(const Eigen::Matrix4d& target) const
{
    // 确定性种子：4 组经验构型 + 肩/肘网格 6 组，与现网 Python 同源。
    std::vector<JointRadians> seeds;
    seeds.push_back(JointRadians{});
    seeds.push_back(JointRadians{90.0 * kRadiansPerDegree, 20.0 * kRadiansPerDegree,
                                 -130.0 * kRadiansPerDegree, -80.0 * kRadiansPerDegree,
                                 -90.0 * kRadiansPerDegree, -90.0 * kRadiansPerDegree});
    seeds.push_back(JointRadians{90.0 * kRadiansPerDegree, 80.0 * kRadiansPerDegree,
                                 -90.0 * kRadiansPerDegree, -100.0 * kRadiansPerDegree,
                                 -90.0 * kRadiansPerDegree, -90.0 * kRadiansPerDegree});
    seeds.push_back(JointRadians{-90.0 * kRadiansPerDegree, 20.0 * kRadiansPerDegree,
                                 -130.0 * kRadiansPerDegree, -80.0 * kRadiansPerDegree,
                                 90.0 * kRadiansPerDegree, 90.0 * kRadiansPerDegree});
    for (double shoulder : {-kPi, 0.0, kPi})
    {
        for (double elbow : {-kPi * 0.5, kPi * 0.5})
        {
            seeds.push_back(JointRadians{shoulder, 0.0, elbow, 0.0, -kPi * 0.5, 0.0});
        }
    }

    std::vector<JointRadians> solutions;
    for (const JointRadians& seed : seeds)
    {
        JointRadians candidate {};
        if (!IterateIk(target, seed, candidate))
        {
            continue;
        }
        bool duplicate = false;
        for (const JointRadians& existing : solutions)
        {
            if (MaxWrappedJointGapRad(candidate, existing) < kIkDedupToleranceRad)
            {
                duplicate = true;
                break;
            }
        }
        if (!duplicate)
        {
            solutions.push_back(candidate);
        }
    }
    return solutions;
}

JointRadians AuboI12hModel::LowerLimitsRad() const
{
    return lower_rad_;
}

JointRadians AuboI12hModel::UpperLimitsRad() const
{
    return upper_rad_;
}

const std::string& AuboI12hModel::RobotType() const
{
    return robot_type_;
}

} // namespace openmind::trajectory_plan
