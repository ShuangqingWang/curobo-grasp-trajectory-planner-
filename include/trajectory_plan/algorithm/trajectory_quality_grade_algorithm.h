#ifndef HYBRID_TRAJECTORY_PLAN_TRAJECTORY_QUALITY_GRADE_ALGORITHM_H
#define HYBRID_TRAJECTORY_PLAN_TRAJECTORY_QUALITY_GRADE_ALGORITHM_H

/**
 * @file trajectory_quality_grade_algorithm.h
 * @brief 轨迹质量评级：A/B/C 与完整周期总分 100。
 */

#include <Eigen/Core>

#include <string>

#include "trajectory_plan/algorithm/joint_trajectory.h"
#include "trajectory_plan/algorithm/pose.h"
#include "trajectory_plan/algorithm/robot_kinematics_interface.h"
#include "trajectory_plan/algorithm/trajectory_collision_check_algorithm.h"

namespace openmind::trajectory_plan
{

/**
 * @brief 轨迹等级。数值越大越差。
 *
 * 评级由三项指标各自给出 A / B / DISCARD，整条取**最差**的那一档：
 * 三项全 A → A；无 DISCARD 且有 B → B；任一项 DISCARD → 整条丢弃。
 */
enum class TrajectoryGrade
{
    A = 0,       ///< 三项全部达到 A 档
    B = 1,       ///< 无丢弃项，但至少一项只到 B 档
    DISCARD = 2, ///< 任一项越过丢弃线，整条不可用
    NONE = 3,    ///< 未评估
};

/**
 * @brief 三项评级指标的门槛（来自 trajectory.yaml 的 grade 节）。
 *
 * | # | 指标 | A | B | 丢弃 |
 * | - | ---- | - | - | ---- |
 * | 1 | 路径倍率 = 法兰路径长 / 端点直线距离 | ≤1.5 | ≤2.0 | >2.0 |
 * | 2 | 抬升（只查抓取段）法兰最高点高出拍照点 | ≤5mm | ≤10mm | >10mm |
 * | 3 | J1~J6 单轴净转角最大值 | ≤180° | ≤360° | >360° |
 *
 * 三项各自出 A / B / 丢弃，整条取最差的那一档：
 * 三项全 A → A；无丢弃且有 B → B；任一项丢弃 → 整条丢弃。
 */
struct GradeThresholds
{
    double path_length_ratio_a = 1.5;         ///< 路径倍率 A 档上限
    double path_length_ratio_b = 2.0;         ///< 路径倍率 B 档上限，超过即丢弃
    double net_joint_travel_a_deg = 180.0;    ///< J1~J6 单轴净转角 A 档上限
    double net_joint_travel_b_deg = 360.0;    ///< J1~J6 单轴净转角 B 档上限，超过即丢弃
    double lift_above_photo_a_mm = 5.0;       ///< 抬升 A 档上限（mm）
    double lift_above_photo_b_mm = 10.0;      ///< 抬升 B 档上限（mm），超过即丢弃
    /// 抬升判据的数值容差（mm）：正解浮点噪声在 1e-5 mm 量级，取 1 微米。
    double lift_numerical_epsilon_mm = 0.001;
};

/**
 * @brief 单段轨迹的质量指标。
 */
struct SegmentQuality
{
    TrajectoryGrade grade = TrajectoryGrade::NONE; ///< 三项取最差的结果
    double path_length_ratio = 0.0;      ///< 法兰路径长 / 端点直线距离
    double lift_above_photo_mm = 0.0;    ///< 法兰最高点高出起点的量（mm）；只对抓取轨迹有意义
    double max_net_joint_travel_deg = 0.0; ///< J1~J6 单轴净转角最大值（degree）
    JointDegrees net_joint_travel_deg {};  ///< 六轴各自的净转角（degree）
    int32_t max_net_joint_index = -1;      ///< 净转角最大值所在关节（0 起）
    double flange_path_length_mm = 0.0;  ///< 法兰路径长度（mm）
    double endpoint_distance_mm = 0.0;   ///< 端点法兰直线距离（mm）
    int32_t lift_violation_points = 0;   ///< 超过 A 档阈值的轨迹点数（供日志说明丢弃原因）
    /// 三项各自的判定，供日志逐项打印。
    TrajectoryGrade ratio_grade = TrajectoryGrade::NONE;
    TrajectoryGrade lift_grade = TrajectoryGrade::NONE;
    TrajectoryGrade travel_grade = TrajectoryGrade::NONE;
};

/**
 * @brief 完整周期（两段拼接）的质量结果。
 */
struct CycleQuality
{
    TrajectoryGrade grade = TrajectoryGrade::NONE; ///< 两段里更差的那级
    double total_score = 0.0;      ///< 满分 100
    double grasp_quality_score = 0.0; ///< 抓取离线分（满分 20）
    double photo_to_grasp_score = 0.0; ///< 拍照→抓取分（满分 35）
    double grasp_to_place_score = 0.0; ///< 抓取→放置分（满分 35）
    double continuity_score = 0.0;  ///< 连续分（满分 10）
    SegmentQuality photo_to_grasp;  ///< 第一段指标
    SegmentQuality grasp_to_place;  ///< 第二段指标
};

/**
 * @brief 轨迹质量评级（纯计算；运动学模型由 stage 注入）。
 */
class TrajectoryQualityGradeAlgorithm
{
  public:
    TrajectoryQualityGradeAlgorithm() = default;
    TrajectoryQualityGradeAlgorithm(const TrajectoryQualityGradeAlgorithm&) = delete;
    TrajectoryQualityGradeAlgorithm& operator=(const TrajectoryQualityGradeAlgorithm&) = delete;

    /**
     * @brief 绑定运动学模型与门禁。
     * @param kinematics 六轴运动学（stage 注入）
     * @param thresholds A/B 阈值
     * @param grasp_score_weight 抓取离线分权重（默认 20）
     * @param photo_to_grasp_weight 第一段权重（默认 35）
     * @param grasp_to_place_weight 第二段权重（默认 35）
     * @param continuity_weight 连续分权重（默认 10）
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true
     */
    bool Init(const RobotKinematicsInterface* kinematics,
              const GradeThresholds& thresholds,
              double grasp_score_weight,
              double photo_to_grasp_weight,
              double grasp_to_place_weight,
              double continuity_weight,
              std::string& error);

    /**
     * @brief 评估单段轨迹质量（不含碰撞，调用前须已通过碰撞检查）。
     * @param trajectory 关节轨迹（rad）
     * @param check_lift 是否启用抬升门禁；只有抓取段（拍照→抓取）传 true
     * @param quality 输出：质量指标
     * @note 抓取段的首点即拍照位，抬升门禁以该点法兰高度为基准；
     *       放置段不查抬升，允许向上运动。
     */
    void GradeSegment(const JointTrajectory& trajectory, bool check_lift, SegmentQuality& quality) const;

    /**
     * @brief 评估完整周期：两段在 q_grasp 接上，等级取更差段，总分 100。
     * @param photo_to_grasp 第一段
     * @param grasp_to_place 第二段
     * @param grasp_candidate_score 抓取离线分（0~1）
     * @param max_grasp_candidate_score 本次全部候选中的最高离线分（归一化用）
     * @param quality 输出：完整周期质量
     */
    void GradeCycle(const JointTrajectory& photo_to_grasp,
                    const JointTrajectory& grasp_to_place,
                    double grasp_candidate_score,
                    double max_grasp_candidate_score,
                    CycleQuality& quality) const;

    /** @brief 等级字符串（A/B/C/-）。 */
    static std::string GradeName(TrajectoryGrade grade);

  private:
    const RobotKinematicsInterface* kinematics_ = nullptr; ///< 运动学模型
    GradeThresholds thresholds_;      ///< A/B 阈值
    double grasp_score_weight_ = 20.0;    ///< 抓取离线分权重
    double photo_to_grasp_weight_ = 35.0; ///< 第一段权重
    double grasp_to_place_weight_ = 35.0; ///< 第二段权重
    double continuity_weight_ = 10.0;     ///< 连续分权重
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_TRAJECTORY_QUALITY_GRADE_ALGORITHM_H
