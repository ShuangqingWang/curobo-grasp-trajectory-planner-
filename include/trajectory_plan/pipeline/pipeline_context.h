#ifndef HYBRID_TRAJECTORY_PLAN_PIPELINE_CONTEXT_H
#define HYBRID_TRAJECTORY_PLAN_PIPELINE_CONTEXT_H

/**
 * @file pipeline_context.h
 * @brief 一次触发两段 flow 的数据总线：请求输入、本轮中间量与结果。
 *
 * 每次触发创建独立上下文；stage 实例与启动期缓存持续复用，不放在这里。
 */

#include <nlohmann/json.hpp>

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "trajectory_plan/algorithm/grasp_result.h"
#include "trajectory_plan/algorithm/joint_trajectory.h"
#include "trajectory_plan/algorithm/pose.h"
#include "trajectory_plan/algorithm/robot_kinematics_interface.h"
#include "trajectory_plan/algorithm/trajectory_quality_grade_algorithm.h"
#include "trajectory_plan/camera/camera_manager.h"
#include "trajectory_plan/device/device_manager.h"

namespace openmind::trajectory_plan
{

/**
 * @brief 本轮候选淘汰统计（文档「流程汇总」要求逐项可查）。
 */
struct CandidateStatistics
{
    int64_t grasp_targets_total = 0;    ///< 本轮可用抓取目标数
    int64_t grasp_targets_attempted = 0;///< 实际尝试的目标数
    /// 末端预筛淘汰（按目标计）。改动五取消预筛后**恒为 0**，字段保留以维持
    /// trajectory_manifest.json 的既有输出契约不变。
    int64_t prefilter_rejected = 0;
    int64_t gpu_failed = 0;             ///< GPU 规划失败（按目标计）
    int64_t grasp_trajectories_returned = 0; ///< 抓取段返回轨迹总条数
    int64_t grade_rejected = 0;         ///< 评级淘汰（按轨迹计）
    int64_t review_rejected = 0;        ///< 复核淘汰（按轨迹计）
    int64_t place_trajectories_returned = 0; ///< 放置段返回轨迹总条数
    int64_t place_grade_rejected = 0;   ///< 放置段评级淘汰
    int64_t place_review_rejected = 0;  ///< 放置段复核淘汰
    int64_t place_plan_failed = 0;      ///< 放置段规划失败次数
    std::string stop_reason;            ///< 结束原因：首个合格即停止 / 候选耗尽 / 预算耗尽
};

/**
 * @brief 一次候选尝试记录（含失败），用于诊断与统计。
 */
struct PlanAttempt
{
    int64_t candidate_index = -1; ///< 抓取候选序号（0~9）
    int64_t grasp_rank = -1;      ///< 抓取轨迹 rank（1 起）
    int64_t place_rank = -1;      ///< 放置轨迹 rank（1 起）
    JointRadians q_grasp {};      ///< 抓取轨迹实际末点（rad）
    TrajectoryGrade photo_to_grasp_grade = TrajectoryGrade::NONE; ///< 抓取段等级
    TrajectoryGrade grasp_to_place_grade = TrajectoryGrade::NONE; ///< 放置段等级
    TrajectoryGrade cycle_grade = TrajectoryGrade::NONE;          ///< 完整周期等级
    std::string failure_reason;   ///< 淘汰原因（成功时为空）
};

/**
 * @brief 一次触发的数据总线：stage 之间唯一的交接点。
 */
struct PipelineContext
{
    // ---- 触发身份 ----
    std::string request_id;     ///< 本次触发 ID（服务端分配）
    std::string generation_id;  ///< 本次生成 ID（随机 32 位 hex）
    std::string connection_id;  ///< 连接标识（接收阶段日志关联）
    std::string workpiece_type; ///< 工件种类，如 QR0006

    // ---- TCP 帧运行时输入（service 收到的内存数据，不读本地图片）----
    std::string request_text;          ///< request.txt 原文
    bool has_t_b_o = false;            ///< 是否收到合法 T_B_O
    Eigen::Matrix4d t_b_o = Eigen::Matrix4d::Identity(); ///< 工件系到基座系（mm）
    Eigen::Matrix4d t_base_flange = Eigen::Matrix4d::Identity(); ///< 本轮拍照法兰相对基座（mm）
    JointDegrees t_base_flange_joint_angles_deg {}; ///< 本轮拍照法兰对应六轴角（deg）
    std::vector<uint8_t> depth_main_bytes;  ///< 主图 TIFF 原始字节
    std::vector<uint8_t> depth_left_bytes;  ///< 左图 TIFF 原始字节
    std::vector<uint8_t> depth_right_bytes; ///< 右图 TIFF 原始字节
    std::vector<uint8_t> rgb_main_bytes;    ///< 主图 RGB PNG 原始字节（后台可视化用）

    // ---- 输出目录（由 App 注入）----
    std::string grasp_output_dir;      ///< output/grasp_generation
    std::string trajectory_output_dir; ///< output/trajectory_planning

    // ---- flow grasp 中间量 / 输出 ----
    std::vector<GraspLabel> workpiece_labels;  ///< 本轮取用的标签（最多 candidate_count 条）
    std::vector<GraspResult> grasp_candidates; ///< 本轮候选，0 号分数最高
    double clearance_mm = 0.0;   ///< 实际使用的 TCP 局部 Z 退让（mm）
    int64_t label_pool_size = 0; ///< 该工件缓存中的有效标签总数（日志用）
    std::string grasp_result_path; ///< 本轮 grasp_result.json 路径

    // ---- flow trajectory 输入 ----
    std::vector<Pose> grasp_flange_poses;       ///< 每个候选的 trajectory_grasp_flange
    std::vector<double> grasp_candidate_scores; ///< 与上对齐的离线分数
    JointRadians q_photo {};  ///< 拍照六轴（rad），抓取段起点
    JointRadians q_place {};  ///< 放置六轴（rad），放置段终点
    Eigen::Matrix4d photo_flange = Eigen::Matrix4d::Identity(); ///< 拍照法兰（mm）
    Eigen::Matrix4d place_flange = Eigen::Matrix4d::Identity(); ///< 放置法兰（mm），仅用于独立复核

    // ---- flow trajectory 中间量与结果 ----
    std::vector<Eigen::Vector3d> cloud_b;      ///< 基座系融合点云（mm）
    std::vector<uint8_t> cloud_camera_indices; ///< 与 cloud_b 对齐的来源相机（0/1/2）
    /// 与 cloud_b 对齐的来源深度图行主序像素下标；后台将主相机点精确映射到 RGB。
    std::vector<uint32_t> cloud_source_pixel_indices;
    int32_t main_depth_width = 0;  ///< 主深度图宽度，用于校验 RGB 像素一一对应
    int32_t main_depth_height = 0; ///< 主深度图高度，用于校验 RGB 像素一一对应
    /// 本轮 GPU 碰撞世界的可审计元数据（改动十）：体素尺寸、激活距离、外扩量及来源、
    /// 锚点、网格范围/尺寸/体素数、占据体素（按点云与平台拆分）。
    /// 网格每轮都变（点云每轮都变），故不能放进启动期的固定模型快照，
    /// 必须随每轮 trajectory_manifest.json 落盘，历史归档才能回放当轮网格。
    nlohmann::json collision_world_meta;
    std::vector<PlanAttempt> attempts;         ///< 全部尝试（含失败）
    CandidateStatistics statistics;            ///< 候选淘汰统计
    JointTrajectory photo_to_grasp_trajectory; ///< 抓取段（选中时有效）
    JointTrajectory grasp_to_place_trajectory; ///< 放置段（选中时有效）
    CycleQuality cycle_quality;                ///< 完整周期质量
    bool has_solution = false;                 ///< 是否选出合格完整周期
    int64_t selected_candidate_index = -1;     ///< 选中的抓取候选原编号
    int64_t selected_grasp_rank = -1;          ///< 选中的抓取 rank（1 起）
    int64_t selected_place_rank = -1;          ///< 选中的放置 rank（1 起）
    double joint_link_max_diff_deg = 0.0;      ///< 两段衔接处的最大关节差（度）
    bool joint_link_pass = false;              ///< 两段衔接检查结果
    std::string failure_reason;                ///< 无解时的原因

    // ---- 第三流程衔接（发布契约）----
    nlohmann::json grasp_preview;      ///< manifest 的 grasp_preview 节
    /// 本轮已发布的 manifest 文档（改动六）。发完 completed 之后，后台写出点云
    /// 并在这份文档上只改 point_cloud 的 status/sha256/点数后补发同代次 manifest。
    nlohmann::json manifest_snapshot;
    std::string collision_model_id;    ///< 固定模型标识
    std::string collision_model_sha256;///< 固定模型快照哈希

    // ---- 各 stage 耗时（ms）----
    std::map<std::string, double> stage_timings_ms;
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_PIPELINE_CONTEXT_H
