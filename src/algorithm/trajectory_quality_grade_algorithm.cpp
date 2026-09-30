/**
 * @file trajectory_quality_grade_algorithm.cpp
 * @brief A/B/C 与完整周期 100 分。
 */

#include "trajectory_plan/algorithm/trajectory_quality_grade_algorithm.h"

#include <cmath>

namespace openmind::trajectory_plan
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesPerRadian = 180.0 / kPi;

} // namespace

bool TrajectoryQualityGradeAlgorithm::Init(const RobotKinematicsInterface* kinematics,
                                           const GradeThresholds& thresholds,
                                           double grasp_score_weight,
                                           double photo_to_grasp_weight,
                                           double grasp_to_place_weight,
                                           double continuity_weight,
                                           std::string& error)
{
    if (kinematics == nullptr)
    {
        error = "质量评级缺少运动学";
        return false;
    }
    kinematics_ = kinematics;
    thresholds_ = thresholds;
    grasp_score_weight_ = grasp_score_weight;
    photo_to_grasp_weight_ = photo_to_grasp_weight;
    grasp_to_place_weight_ = grasp_to_place_weight;
    continuity_weight_ = continuity_weight;
    return true;
}

std::string TrajectoryQualityGradeAlgorithm::GradeName(TrajectoryGrade grade)
{
    switch (grade)
    {
        case TrajectoryGrade::A:
            return "A";
        case TrajectoryGrade::B:
            return "B";
        case TrajectoryGrade::DISCARD:
            return "丢弃";
        default:
            return "-";
    }
}

void TrajectoryQualityGradeAlgorithm::GradeSegment(const JointTrajectory& trajectory, bool check_lift,
                                                   SegmentQuality& quality) const
{
    quality = SegmentQuality();
    if (!trajectory.IsValid() || kinematics_ == nullptr)
    {
        quality.grade = TrajectoryGrade::DISCARD;
        return;
    }

    // ---- 指标三：J1~J6 单轴净转角最大值 D = max(D_1 … D_6)。----
    // D_j = |q_j(终点) − q_j(起点)|，六轴全部参与，保留实际圈数不取模。
    const JointRadians net = trajectory.NetJointTravelRad();
    for (size_t joint = 0; joint < 6; ++joint)
    {
        quality.net_joint_travel_deg[joint] = net[joint] * kDegreesPerRadian;
    }
    quality.max_net_joint_travel_deg = trajectory.MaxNetJointTravelRad() * kDegreesPerRadian;
    quality.max_net_joint_index = trajectory.MaxNetJointTravelIndex();
    if (quality.max_net_joint_travel_deg > thresholds_.net_joint_travel_b_deg + 1e-6)
    {
        quality.travel_grade = TrajectoryGrade::DISCARD;
    }
    else if (quality.max_net_joint_travel_deg > thresholds_.net_joint_travel_a_deg + 1e-6)
    {
        quality.travel_grade = TrajectoryGrade::B;
    }
    else
    {
        quality.travel_grade = TrajectoryGrade::A;
    }

    // 逐点正解出法兰位置，供指标一、二使用。
    std::vector<Eigen::Vector3d> positions;
    positions.reserve(trajectory.points.size());
    for (const JointRadians& q : trajectory.points)
    {
        positions.push_back(kinematics_->ForwardKinematics(q).block<3, 1>(0, 3));
    }

    // ---- 指标一：路径倍率 = 法兰实际路径长 / 两端直线距离 ----
    double path = 0.0;
    for (size_t index = 1; index < positions.size(); ++index)
    {
        path += (positions[index] - positions[index - 1]).norm();
    }
    const double straight = (positions.back() - positions.front()).norm();
    quality.flange_path_length_mm = path;
    quality.endpoint_distance_mm = straight;
    quality.path_length_ratio = (straight > 1e-6) ? (path / straight) : 1.0;
    if (quality.path_length_ratio > thresholds_.path_length_ratio_b + 1e-6)
    {
        quality.ratio_grade = TrajectoryGrade::DISCARD;
    }
    else if (quality.path_length_ratio > thresholds_.path_length_ratio_a + 1e-6)
    {
        quality.ratio_grade = TrajectoryGrade::B;
    }
    else
    {
        quality.ratio_grade = TrajectoryGrade::A;
    }

    // ---- 指标二：抬升。只查抓取段，其首点即拍照位，故以首点高度为基准。----
    // A ≤5mm；B ≤10mm；>10mm 丢弃。放置段不查抬升，允许向上运动。
    if (check_lift)
    {
        // 数值容差 1 微米：正解的浮点噪声在 1e-5 mm 量级，而机器人重复定位精度
        // 在 ±50 微米量级，所以 1 微米既高于噪声、又远低于任何机械意义。
        // 曾经用 1e-6 mm（纳米级）把 1.9e-5 mm 的浮点噪声判成违规，全部候选被误杀。
        const double epsilon = thresholds_.lift_numerical_epsilon_mm;
        const double baseline_z = positions.front().z();
        double highest = baseline_z;
        for (const Eigen::Vector3d& point : positions)
        {
            if (point.z() > baseline_z + thresholds_.lift_above_photo_a_mm + epsilon)
            {
                ++quality.lift_violation_points;
            }
            highest = std::max(highest, point.z());
        }
        quality.lift_above_photo_mm = std::max(0.0, highest - baseline_z);
        if (quality.lift_above_photo_mm > thresholds_.lift_above_photo_b_mm + epsilon)
        {
            quality.lift_grade = TrajectoryGrade::DISCARD;
        }
        else if (quality.lift_above_photo_mm > thresholds_.lift_above_photo_a_mm + epsilon)
        {
            quality.lift_grade = TrajectoryGrade::B;
        }
        else
        {
            quality.lift_grade = TrajectoryGrade::A;
        }
    }
    else
    {
        quality.lift_grade = TrajectoryGrade::A;
    }

    // ---- 三项取最差 ----
    quality.grade = static_cast<TrajectoryGrade>(
        std::max({static_cast<int>(quality.ratio_grade), static_cast<int>(quality.lift_grade),
                  static_cast<int>(quality.travel_grade)}));
}

void TrajectoryQualityGradeAlgorithm::GradeCycle(const JointTrajectory& photo_to_grasp,
                                                 const JointTrajectory& grasp_to_place,
                                                 double grasp_candidate_score,
                                                 double max_grasp_candidate_score,
                                                 CycleQuality& quality) const
{
    quality = CycleQuality();
    // 抓取段查抬升门禁（首点即拍照位），放置段不查。
    GradeSegment(photo_to_grasp, true, quality.photo_to_grasp);
    GradeSegment(grasp_to_place, false, quality.grasp_to_place);
    quality.grade = static_cast<TrajectoryGrade>(
        std::max(static_cast<int>(quality.photo_to_grasp.grade),
                 static_cast<int>(quality.grasp_to_place.grade)));
    const double connection = MaxWrappedJointGapRad(photo_to_grasp.points.back(), grasp_to_place.points.front());
    quality.continuity_score = (connection <= 1e-4) ? continuity_weight_ : 0.0;
    const double grasp_norm =
        (max_grasp_candidate_score > 1e-12) ? (grasp_candidate_score / max_grasp_candidate_score) : 1.0;
    quality.grasp_quality_score = grasp_score_weight_ * std::max(0.0, std::min(1.0, grasp_norm));
    auto segment_score = [](const SegmentQuality& segment) {
        if (segment.grade == TrajectoryGrade::A)
        {
            return 1.0;
        }
        if (segment.grade == TrajectoryGrade::B)
        {
            return 0.7;
        }
        return 0.0;
    };
    quality.photo_to_grasp_score = photo_to_grasp_weight_ * segment_score(quality.photo_to_grasp);
    quality.grasp_to_place_score = grasp_to_place_weight_ * segment_score(quality.grasp_to_place);
    quality.total_score = quality.grasp_quality_score + quality.photo_to_grasp_score +
                          quality.grasp_to_place_score + quality.continuity_score;
}

} // namespace openmind::trajectory_plan
