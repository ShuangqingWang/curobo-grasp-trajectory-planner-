#ifndef HYBRID_TRAJECTORY_PLAN_TRAJECTORY_OUTPUT_FORMAT_ALGORITHM_H
#define HYBRID_TRAJECTORY_PLAN_TRAJECTORY_OUTPUT_FORMAT_ALGORITHM_H

/**
 * @file trajectory_output_format_algorithm.h
 * @brief 轨迹输出排版：两条 aubo_joint_path/v1、manifest、PLY 文本与 SHA-256。
 */

#include <nlohmann/json.hpp>

#include <Eigen/Core>

#include <array>
#include <string>
#include <vector>

#include "trajectory_plan/algorithm/joint_trajectory.h"
#include "trajectory_plan/algorithm/trajectory_quality_grade_algorithm.h"

namespace openmind::trajectory_plan
{

/**
 * @brief 轨迹输出排版（纯计算）。
 */
class TrajectoryOutputFormatAlgorithm
{
  public:
    TrajectoryOutputFormatAlgorithm() = default;
    TrajectoryOutputFormatAlgorithm(const TrajectoryOutputFormatAlgorithm&) = delete;
    TrajectoryOutputFormatAlgorithm& operator=(const TrajectoryOutputFormatAlgorithm&) = delete;

    /**
     * @brief 生成 aubo_joint_path/v1 轨迹文档（与 scripts/轨迹运行 执行契约一致）。
     * @param trajectory 关节轨迹（rad）
     * @param route "photo_to_grasp" / "grasp_to_place"
     * @param request_id 触发 ID
     * @param generation_id 生成 ID
     * @param workpiece_type 工件种类
     * @param time_step_s 采样间隔（s）
     * @return JSON 文档
     */
    static nlohmann::json BuildJointTrajectoryDocument(const JointTrajectory& trajectory,
                                                       const std::string& route,
                                                       const std::string& request_id,
                                                       const std::string& generation_id,
                                                       const std::string& workpiece_type,
                                                       double time_step_s);

    /**
     * @brief 本轮发布的一个文件条目（路径 + SHA-256 + 规模）。
     */
    struct FileEntry
    {
        std::string path;        ///< service 侧路径
        std::string sha256;      ///< 文件内容 SHA-256
        int64_t item_count = 0;  ///< 轨迹点数或点云点数
    };

    /**
     * @brief 组装 trajectory_manifest.json 的完整发布输入。
     *
     * manifest 同时绑定两段路径、哈希、代次、等级、所选抓取 rank 与放置 rank，
     * 以及关节连接检查结果；并补充点云条目、固定模型关联与 grasp_preview，
     * 使第三流程可以只靠三份主文件加 manifest 完成校验与显示。
     */
    struct ManifestInput
    {
        std::string status = "completed";  ///< processing / completed / failed
        bool ready = false;                ///< 本轮轨迹是否可用
        std::string request_id;            ///< 触发 ID
        std::string generation_id;         ///< 生成 ID
        std::string workpiece_type;        ///< 工件种类
        std::string reason;                ///< 无解或失败原因
        int64_t selected_candidate_index = -1; ///< 选中的抓取候选原编号
        int64_t selected_grasp_rank = -1;  ///< 选中的抓取 rank（1 起）
        int64_t selected_place_rank = -1;  ///< 选中的放置 rank（1 起）
        FileEntry photo_to_grasp;          ///< 抓取段文件条目
        FileEntry grasp_to_place;          ///< 放置段文件条目
        FileEntry point_cloud;             ///< 点云条目（文件名、SHA-256 及点数）
        /// 点云状态（改动六）：轨迹优先应答时先发 "pending"，后台写完点云再补发
        /// "ready" 并填上 sha256 与点数。第三流程需容忍 pending：先显示轨迹，
        /// 点云等下一轮轮询补上。
        std::string point_cloud_status = "ready";
        /// 本轮 GPU 碰撞世界元数据（改动十）。网格每轮都变，故不进固定模型快照，
        /// 随每轮 manifest 落盘，历史归档才能回放当轮网格范围。
        nlohmann::json collision_world;
        std::string collision_model_id;    ///< 固定模型标识
        std::string collision_model_sha256;///< 固定模型快照哈希
        double joint_link_max_diff_deg = 0.0; ///< 两段衔接的最大关节差（度）
        bool joint_link_pass = false;      ///< 两段衔接检查结果
        nlohmann::json grasp_preview;      ///< 抓取夹爪预览元数据
        nlohmann::json statistics;         ///< 本轮候选淘汰统计
    };

    /**
     * @brief 生成 trajectory_manifest.json 文档。
     * @param input 发布输入
     * @param cycle 完整周期质量（等级取两段较差档）
     * @return JSON 文档
     */
    static nlohmann::json BuildManifestDocument(const ManifestInput& input, const CycleQuality& cycle);

    /**
     * @brief 生成基座系点云二进制 PLY（format binary_little_endian 1.0）。
     *
     * 改动八：原为 ASCII，17.28 万点逐点格式化产出 6.6 MB，加 SHA-256 约 510ms；
     * 改二进制后约 2 MB、约 160ms。每个顶点 15 字节：
     * float32 x/y/z（小端，mm）+ uchar red/green/blue。
     *
     * 第三流程的 `parsePlyPointCloud` 已支持二进制（`parseBinaryPlyPoints`），
     * **前端无需改动**。
     *
     * @param points 基座系点（mm）
     * @param colors 每点 RGB（与 points 对齐）
     * @param comments 头部注释（坐标系/单位等）
     * @return PLY 字节内容（头部为 ASCII，顶点区为二进制）
     */
    static std::string BuildPlyBinary(const std::vector<Eigen::Vector3d>& points,
                                      const std::vector<std::array<uint8_t, 3>>& colors,
                                      const std::vector<std::string>& comments);

    /**
     * @brief 字符串的 SHA-256（小写十六进制）。
     */
    static std::string Sha256Hex(const std::string& content);
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_TRAJECTORY_OUTPUT_FORMAT_ALGORITHM_H
