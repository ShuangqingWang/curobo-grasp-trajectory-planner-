/**
 * @file trajectory_collision_check_algorithm.cpp
 * @brief 限位、端点、冗余转角复核（环境碰撞与自碰由 cuRobo 在 GPU 侧沿途完成）。
 */

#include "trajectory_plan/algorithm/trajectory_collision_check_algorithm.h"

#include "trajectory_plan/device/aubo_i12h_model.h"

#include <algorithm>
#include <cmath>

namespace openmind::trajectory_plan
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesPerRadian = 180.0 / kPi;

} // namespace

bool TrajectoryCollisionCheckAlgorithm::Init(const RobotKinematicsInterface* kinematics,
                                             double redundant_joint_travel_deg,
                                             double endpoint_position_tolerance_mm,
                                             double endpoint_rotation_tolerance_deg,
                                             std::string& error)
{
    if (kinematics == nullptr || redundant_joint_travel_deg <= 0.0 ||
        endpoint_position_tolerance_mm <= 0.0 || endpoint_rotation_tolerance_deg <= 0.0)
    {
        error = "TrajectoryCollisionCheckAlgorithm 参数非法";
        return false;
    }
    kinematics_ = kinematics;
    redundant_travel_deg_ = redundant_joint_travel_deg;
    endpoint_position_tolerance_mm_ = endpoint_position_tolerance_mm;
    endpoint_rotation_tolerance_rad_ = endpoint_rotation_tolerance_deg / kDegreesPerRadian;
    return true;
}

bool TrajectoryCollisionCheckAlgorithm::FillCommonGates(const JointTrajectory& trajectory,
                                                        const Eigen::Matrix4d& expected_start_flange,
                                                        const Eigen::Matrix4d& expected_goal_flange,
                                                        TrajectoryCollisionReport& report) const
{
    report = TrajectoryCollisionReport();
    if (!trajectory.IsValid() || kinematics_ == nullptr)
    {
        report.detail = "轨迹无效";
        return false;
    }
    // ---- 净转角复核：与评级 5.1 同一口径（首末角度差的绝对值，J1~J6）----
    const JointRadians net = trajectory.NetJointTravelRad();
    for (size_t joint = 0; joint < 6; ++joint)
    {
        report.net_joint_travel_deg[joint] = net[joint] * kDegreesPerRadian;
    }
    report.max_net_joint_travel_deg = trajectory.MaxNetJointTravelRad() * kDegreesPerRadian;
    report.max_net_joint_index = trajectory.MaxNetJointTravelIndex();
    report.total_wrist_travel_deg = trajectory.TotalWristJointTravelRad() * kDegreesPerRadian;
    // 恰好 360° 不触发此门禁，与评级的 B 档上限一致。
    if (report.max_net_joint_travel_deg > redundant_travel_deg_ + 1e-6)
    {
        report.redundant_joint_travel = true;
        report.detail = "J" + std::to_string(report.max_net_joint_index + 1) + " 净转角 " +
                        std::to_string(report.max_net_joint_travel_deg) + "° 超过上限 " +
                        std::to_string(redundant_travel_deg_) + "°";
        return false;
    }

    // ---- 端点吻合：位置 ≤ 容差 且 姿态 ≤ 容差，两项均须通过 ----
    // 实测误差一并带出，便于定位是容差偏紧还是坐标系不一致。
    auto measure = [&](const JointRadians& q, const Eigen::Matrix4d& expected,
                       double& position_mm, double& rotation_deg) {
        const Eigen::Matrix4d actual = kinematics_->ForwardKinematics(q);
        position_mm = (actual.block<3, 1>(0, 3) - expected.block<3, 1>(0, 3)).norm();
        const Eigen::Matrix3d relative = expected.block<3, 3>(0, 0).transpose() * actual.block<3, 3>(0, 0);
        const double cosine = std::max(-1.0, std::min(1.0, (relative.trace() - 1.0) * 0.5));
        rotation_deg = std::acos(cosine) * kDegreesPerRadian;
    };
    const double rotation_tolerance_deg = endpoint_rotation_tolerance_rad_ * kDegreesPerRadian;
    auto format_failure = [&](const char* which, double position_mm, double rotation_deg) {
        return std::string(which) + "法兰与目标不符（实测 " + std::to_string(position_mm) + " mm / " +
               std::to_string(rotation_deg) + "°，容差 " +
               std::to_string(endpoint_position_tolerance_mm_) + " mm / " +
               std::to_string(rotation_tolerance_deg) + "°）";
    };

    if (!expected_start_flange.isIdentity(1e-12))
    {
        measure(trajectory.points.front(), expected_start_flange, report.start_position_error_mm,
                report.start_rotation_error_deg);
        if (report.start_position_error_mm > endpoint_position_tolerance_mm_ ||
            report.start_rotation_error_deg > rotation_tolerance_deg)
        {
            report.endpoint_mismatch = true;
            report.detail = format_failure("起点", report.start_position_error_mm,
                                           report.start_rotation_error_deg);
            return false;
        }
    }
    if (!expected_goal_flange.isIdentity(1e-12))
    {
        measure(trajectory.points.back(), expected_goal_flange, report.goal_position_error_mm,
                report.goal_rotation_error_deg);
        if (report.goal_position_error_mm > endpoint_position_tolerance_mm_ ||
            report.goal_rotation_error_deg > rotation_tolerance_deg)
        {
            report.endpoint_mismatch = true;
            report.detail = format_failure("终点", report.goal_position_error_mm,
                                           report.goal_rotation_error_deg);
            return false;
        }
    }
    return true;
}

bool TrajectoryCollisionCheckAlgorithm::CheckJointEndpoints(const JointTrajectory& trajectory,
                                                            const JointRadians& expected_start,
                                                            const JointRadians& expected_goal,
                                                            double tolerance_deg,
                                                            TrajectoryCollisionReport& report) const
{
    if (!trajectory.IsValid())
    {
        report.joint_endpoint_mismatch = true;
        report.detail = "放置段轨迹无效";
        return false;
    }
    // 关节差保留实际圈数，不 wrap：差 360° 不是同一条运动指令。
    auto max_gap_deg = [](const JointRadians& left, const JointRadians& right) {
        double maximum = 0.0;
        for (size_t joint = 0; joint < 6; ++joint)
        {
            maximum = std::max(maximum, std::abs(left[joint] - right[joint]) * kDegreesPerRadian);
        }
        return maximum;
    };
    report.start_joint_error_deg = max_gap_deg(trajectory.points.front(), expected_start);
    report.goal_joint_error_deg = max_gap_deg(trajectory.points.back(), expected_goal);
    if (report.start_joint_error_deg > tolerance_deg)
    {
        report.joint_endpoint_mismatch = true;
        report.detail = "放置首点关节角与抓取末点不一致（实测 " +
                        std::to_string(report.start_joint_error_deg) + "°，容差 " +
                        std::to_string(tolerance_deg) + "°）";
        return false;
    }
    if (report.goal_joint_error_deg > tolerance_deg)
    {
        report.joint_endpoint_mismatch = true;
        report.detail = "放置末点关节角与 q_place 不一致（实测 " +
                        std::to_string(report.goal_joint_error_deg) + "°，容差 " +
                        std::to_string(tolerance_deg) + "°）";
        return false;
    }
    return true;
}

bool TrajectoryCollisionCheckAlgorithm::CheckKinematicGates(const JointTrajectory& trajectory,
                                                            const Eigen::Matrix4d& expected_start_flange,
                                                            const Eigen::Matrix4d& expected_goal_flange,
                                                            TrajectoryCollisionReport& report) const
{
    if (!FillCommonGates(trajectory, expected_start_flange, expected_goal_flange, report))
    {
        return false;
    }
    for (const JointRadians& q : trajectory.points)
    {
        ++report.checked_waypoints;
        if (!kinematics_->WithinLimits(q))
        {
            report.joint_limit_violation = true;
            report.detail = "关节越限位";
            return false;
        }
    }
    report.pass = true;
    return true;
}

} // namespace openmind::trajectory_plan
