#ifndef HYBRID_TRAJECTORY_PLAN_GRASP_POSE_COMPOSE_ALGORITHM_H
#define HYBRID_TRAJECTORY_PLAN_GRASP_POSE_COMPOSE_ALGORITHM_H

/**
 * @file grasp_pose_compose_algorithm.h
 * @brief 一条标注 × T_B_O × 夹爪标定 × 退让 → 一组抓取候选（四组位姿 + 开口 + 分数）。
 */

#include <Eigen/Core>

#include <string>

#include "trajectory_plan/algorithm/grasp_result.h"

namespace openmind::trajectory_plan
{

/**
 * @brief 抓取位姿合成（纯计算）。
 *
 * T_B_TCP = T_B_O · T_O_DB · T_DB_TCP；法兰 = TCP · T_TCP_F；
 * 再沿 TCP 局部 Z 平移 clearance（mm）得到 trajectory_grasp_*。
 */
class GraspPoseComposeAlgorithm
{
  public:
    GraspPoseComposeAlgorithm() = default;
    GraspPoseComposeAlgorithm(const GraspPoseComposeAlgorithm&) = delete;
    GraspPoseComposeAlgorithm& operator=(const GraspPoseComposeAlgorithm&) = delete;

    /**
     * @brief 初始化标定与退让参数。
     * @param t_flange_tcp TCP → 法兰 4x4 齐次（平移 m，运行时换算 mm）
     * @param t_tcp_grasp_database GripperDatabase → TCP 4x4 齐次（纯旋转）
     * @param clearance_along_tcp_z_mm 沿 TCP 局部 Z 的退让量，单位 mm
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true
     */
    bool Init(const Eigen::Matrix4d& t_flange_tcp,
              const Eigen::Matrix4d& t_tcp_grasp_database,
              double clearance_along_tcp_z_mm,
              std::string& error);

    /**
     * @brief 合成一组抓取候选。
     * @param t_b_o 工件相对基座 4x4 齐次（mm）
     * @param label 一条离线标注（{O} 系，米）
     * @return 四组位姿 + 开口 + 分数的抓取结果
     */
    GraspResult Compose(const Eigen::Matrix4d& t_b_o, const GraspLabel& label) const;

  private:
    Eigen::Matrix4d t_tcp_flange_ = Eigen::Matrix4d::Identity(); ///< TCP → 法兰（mm）
    Eigen::Matrix4d t_database_tcp_ = Eigen::Matrix4d::Identity(); ///< DB → TCP
    double clearance_mm_ = 0.0; ///< 沿 TCP 局部 Z 的退让量（mm）
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_GRASP_POSE_COMPOSE_ALGORITHM_H
