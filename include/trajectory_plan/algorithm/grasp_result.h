#ifndef HYBRID_TRAJECTORY_PLAN_GRASP_RESULT_H
#define HYBRID_TRAJECTORY_PLAN_GRASP_RESULT_H

/**
 * @file grasp_result.h
 * @brief 一组抓取候选：咬合中心与退让后的 TCP/法兰位姿 + 夹爪开口 + 分数。
 */

#include <Eigen/Core>

#include "trajectory_plan/algorithm/pose.h"

namespace openmind::trajectory_plan
{

/**
 * @brief 一条 NPZ 离线标注解析后的内存形态。
 *
 * 位置为工件系 {O} 下的米制数据（与 NPZ 一致），旋转为 3x3 正交阵，
 * width 为夹爪开口（米），score 为离线分数（越大越好）。
 */
struct GraspLabel
{
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();  ///< {O} 系咬合中心，单位 m
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity(); ///< {O} 系姿态（GripperDatabase）
    double width = 0.0;                                     ///< 夹爪开口，单位 m，仅用于加载时过滤
    double score = 0.0;                                     ///< 离线分数
};

/**
 * @brief 一组抓取候选对外输出的全部内容。
 *
 * 只保留三件事：四组位姿 + 夹爪开口 + 分数。
 * grasp_* 是咬合中心（未退让），trajectory_grasp_* 是沿 TCP 局部 Z
 * 退让 clearance（默认 -84 mm，避免相机先撞）之后的位姿。
 * 位姿一律用 Pose（xyzrxryrz，平移 mm、旋转 degree，固定轴 XYZ 即
 * R = Rz(rz)·Ry(ry)·Rx(rx)）；需要 4x4 齐次阵的地方用 TransformFromPose 现转，
 * 不再冗余保存两份。第二步轨迹规划只读 trajectory_grasp_flange。
 */
struct GraspResult
{
    Pose grasp_tcp;               ///< 咬合中心 TCP，未退让
    Pose grasp_flange;            ///< 咬合中心对应法兰，未退让
    Pose trajectory_grasp_tcp;    ///< 退让后 TCP
    Pose trajectory_grasp_flange; ///< 退让后法兰，第二步唯一使用的抓取目标
    double gripper_width = 0.0;   ///< 该组抓取的夹爪开口，单位 m
    double score = 0.0;           ///< 离线分数，用于排序与输出
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_GRASP_RESULT_H
