/**
 * @file grasp_pose_compose_algorithm.cpp
 * @brief T_B_O × 标注 × 夹爪标定 × 退让 → 四组位姿 + 开口 + 分数。
 */

#include "trajectory_plan/algorithm/grasp_pose_compose_algorithm.h"

#include "trajectory_plan/algorithm/pose.h"

namespace openmind::trajectory_plan
{

bool GraspPoseComposeAlgorithm::Init(const Eigen::Matrix4d& t_flange_tcp,
                                     const Eigen::Matrix4d& t_tcp_grasp_database,
                                     double clearance_along_tcp_z_mm,
                                     std::string& error)
{
    std::string transform_error;
    if (!ValidateTransform(t_flange_tcp, "t_flange_tcp", transform_error) ||
        !ValidateTransform(t_tcp_grasp_database, "t_tcp_grasp_database", transform_error))
    {
        error = transform_error;
        return false;
    }
    t_tcp_flange_ = t_flange_tcp;
    t_tcp_flange_(0, 3) *= 1000.0;
    t_tcp_flange_(1, 3) *= 1000.0;
    t_tcp_flange_(2, 3) *= 1000.0;
    t_database_tcp_ = t_tcp_grasp_database;
    t_database_tcp_(0, 3) *= 1000.0;
    t_database_tcp_(1, 3) *= 1000.0;
    t_database_tcp_(2, 3) *= 1000.0;
    clearance_mm_ = clearance_along_tcp_z_mm;
    return true;
}

GraspResult GraspPoseComposeAlgorithm::Compose(const Eigen::Matrix4d& t_b_o, const GraspLabel& label) const
{
    // 标注在工件系 {O} 下：平移由米换算为毫米，旋转直接取用。
    Eigen::Matrix4d t_object_grasp = Eigen::Matrix4d::Identity();
    t_object_grasp.block<3, 3>(0, 0) = label.rotation;
    t_object_grasp.block<3, 1>(0, 3) = label.translation * 1000.0;
    // 咬合中心 TCP：T_B_TCP = T_B_O · T_O_DB · T_DB_TCP。
    const Eigen::Matrix4d original_tcp = t_b_o * t_object_grasp * t_database_tcp_;
    // 沿 TCP 局部 Z 平移 clearance_mm_（负值 = 往后退），得到退让后的 TCP。
    Eigen::Matrix4d clearance = Eigen::Matrix4d::Identity();
    clearance(2, 3) = clearance_mm_;
    const Eigen::Matrix4d planned_tcp = original_tcp * clearance;
    // 两个 TCP 各自右乘 T_TCP_F 换算成法兰。
    const Eigen::Matrix4d original_flange = original_tcp * t_tcp_flange_;
    const Eigen::Matrix4d planned_flange = planned_tcp * t_tcp_flange_;

    GraspResult result;
    result.grasp_tcp = PoseFromTransform(original_tcp);
    result.grasp_flange = PoseFromTransform(original_flange);
    result.trajectory_grasp_tcp = PoseFromTransform(planned_tcp);
    result.trajectory_grasp_flange = PoseFromTransform(planned_flange);
    result.gripper_width = label.width;
    result.score = label.score;
    return result;
}

} // namespace openmind::trajectory_plan
