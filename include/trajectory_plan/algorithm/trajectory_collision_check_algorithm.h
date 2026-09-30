#ifndef HYBRID_TRAJECTORY_PLAN_TRAJECTORY_COLLISION_CHECK_ALGORITHM_H
#define HYBRID_TRAJECTORY_PLAN_TRAJECTORY_COLLISION_CHECK_ALGORITHM_H

/**
 * @file trajectory_collision_check_algorithm.h
 * @brief 轨迹运动学复核：限位、端点、冗余转角。
 *
 * @note 环境碰撞与自碰由 cuRobo 在 GPU 上沿途检查（末端球已挂在 flange_link 上）。
 *       改动五删除了本类的 Check() / CheckConfiguration() / CheckGeometry()：
 *       它们依赖已删除的 C++ 碰撞世界，且查证确认**从未被调用过**，是旧设计遗留的
 *       死代码。保留的 CheckKinematicGates() / CheckJointEndpoints() 是 CPU 侧的
 *       独立复核，不受影响。
 */

#include <string>

#include "trajectory_plan/algorithm/joint_trajectory.h"
#include "trajectory_plan/algorithm/pose.h"
#include "trajectory_plan/algorithm/robot_kinematics_interface.h"

namespace openmind::trajectory_plan
{

/**
 * @brief 单条轨迹的碰撞检查报告。
 */
struct TrajectoryCollisionReport
{
    bool pass = false;                  ///< 整条通过
    bool self_collision = false;        ///< 是否检出机械臂自碰
    bool environment_collision = false; ///< 是否检出与点云/基座箱体碰撞
    bool joint_limit_violation = false; ///< 是否越限位
    bool endpoint_mismatch = false;     ///< 端点 FK 与目标法兰不符
    bool joint_endpoint_mismatch = false; ///< 关节端点与期望构型不符（放置段用）
    bool redundant_joint_travel = false; ///< 任一轴净转角超过 max_redundant_joint_travel_deg
    std::string detail;                 ///< 首个失败点的描述
    int64_t checked_waypoints = 0;      ///< 已检查的离散点数
    double max_net_joint_travel_deg = 0.0; ///< J1~J6 单轴净转角最大值（degree）
    JointDegrees net_joint_travel_deg {}; ///< 六轴各自的净转角（degree）
    int32_t max_net_joint_index = -1;   ///< 净转角最大值所在关节（0 起）
    double start_position_error_mm = 0.0;  ///< 起点法兰位置误差（mm）
    double start_rotation_error_deg = 0.0; ///< 起点法兰姿态误差（degree）
    double goal_position_error_mm = 0.0;   ///< 终点法兰位置误差（mm）
    double goal_rotation_error_deg = 0.0;  ///< 终点法兰姿态误差（degree）
    double start_joint_error_deg = 0.0;    ///< 起点关节最大差（degree）
    double goal_joint_error_deg = 0.0;     ///< 终点关节最大差（degree）
    double total_wrist_travel_deg = 0.0;   ///< 腕部三轴累计转角之和（degree），仅诊断
};

/**
 * @brief 轨迹碰撞检查（纯计算；运动学模型由 stage 注入）。
 */
class TrajectoryCollisionCheckAlgorithm
{
  public:
    TrajectoryCollisionCheckAlgorithm() = default;
    TrajectoryCollisionCheckAlgorithm(const TrajectoryCollisionCheckAlgorithm&) = delete;
    TrajectoryCollisionCheckAlgorithm& operator=(const TrajectoryCollisionCheckAlgorithm&) = delete;

    /**
     * @brief 绑定运动学模型与检查参数。
     * @param kinematics 六轴运动学（stage 注入）
     * @param redundant_joint_travel_deg 单轴净转角硬上限（degree，当前 360）；
     *        恰好等于该值不触发门禁，与评级 B 档上限一致
     * @param endpoint_position_tolerance_mm 端点法兰位置容差（mm）
     * @param endpoint_rotation_tolerance_deg 端点法兰姿态容差（degree）
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true
     */
    bool Init(const RobotKinematicsInterface* kinematics,
              double redundant_joint_travel_deg,
              double endpoint_position_tolerance_mm,
              double endpoint_rotation_tolerance_deg,
              std::string& error);

    /**
     * @brief 只查限位 / 端点 / 冗余转角。cuRobo 已对 ESDF 做过环境与自碰。
     */
    bool CheckKinematicGates(const JointTrajectory& trajectory,
                             const Eigen::Matrix4d& expected_start_flange,
                             const Eigen::Matrix4d& expected_goal_flange,
                             TrajectoryCollisionReport& report) const;

    /**
     * @brief 放置段关节端点复核：首点 vs 抓取实际末点、末点 vs 指定 q_place。
     *
     * 使用独立的关节容差，不能用法兰姿态 0.2° 代替；关节差保留实际圈数，
     * 不把差 360° 当作同一构型。
     * @param trajectory 放置段轨迹（rad）
     * @param expected_start 期望首点关节角（rad）
     * @param expected_goal 期望末点关节角（rad）
     * @param tolerance_deg 关节端点容差（degree）
     * @param report 输入输出：在已有报告上追加关节端点结果
     * @return 通过返回 true
     */
    bool CheckJointEndpoints(const JointTrajectory& trajectory,
                             const JointRadians& expected_start,
                             const JointRadians& expected_goal,
                             double tolerance_deg,
                             TrajectoryCollisionReport& report) const;

  private:
    const RobotKinematicsInterface* kinematics_ = nullptr; ///< 运动学模型（stage 注入）
    double redundant_travel_deg_ = 360.0; ///< 单轴净转角硬上限（degree）
    /// 统一端点标准：位置 ≤1.0mm、姿态 ≤0.2°，两项均须通过。
    /// 与 cuRobo 服务端的收敛判据取同一组数值；GPU 通过而 CPU 不通过时记录
    /// 双方误差并拒绝该条轨迹，不自动放宽阈值。
    double endpoint_position_tolerance_mm_ = 1.0;
    double endpoint_rotation_tolerance_rad_ = 0.00349;

    bool FillCommonGates(const JointTrajectory& trajectory,
                         const Eigen::Matrix4d& expected_start_flange,
                         const Eigen::Matrix4d& expected_goal_flange,
                         TrajectoryCollisionReport& report) const;
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_TRAJECTORY_COLLISION_CHECK_ALGORITHM_H
