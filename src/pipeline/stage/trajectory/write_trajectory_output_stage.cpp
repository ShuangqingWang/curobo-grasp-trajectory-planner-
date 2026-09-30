/**
 * @file write_trajectory_output_stage.cpp
 * @brief flow trajectory / 五 write_trajectory_output：业务步骤 [6/6] 发布。
 *
 * 遵循「output 覆盖与一致性」：原子替换本轮文件 → 清理旧受管产物 → 最后提交 manifest。
 * 本轮开始时 pipeline 已把 manifest 标成 processing/ready=false；
 * 全部正式产物发布成功才改为 completed/ready=true。
 *
 * 正常搜索无解时**仍执行本 stage**：只写点云与带原因、ready=false 的 manifest，
 * 不保留旧成功轨迹冒充当前结果。
 */

#include "trajectory_plan/algorithm/trajectory_output_format_algorithm.h"
#include "trajectory_plan/pipeline/stage_registry.h"
#include "trajectory_plan/service/output_publisher.h"
#include "trajectory_plan/tools/json_util.h"
#include "trajectory_plan/tools/log.h"


namespace openmind::trajectory_plan
{

class WriteTrajectoryOutputStage : public StageBase
{
  public:
    bool Init(const StageConfig& config, std::string& error) override
    {
        BindConfig(config);
        output_dir_ = config.trajectory_output_dir;
        if (output_dir_.empty())
        {
            error = "trajectory 输出目录未配置";
            return false;
        }
        if (!EnsureDirectory(output_dir_, error))
        {
            return false;
        }
        LOG_INFO << log_tag_ << "[初始化] output_dir=" << output_dir_;
        return true;
    }

    bool Execute(PipelineContext& ctx, std::string& error) override
    {
        if (!EnsureDirectory(output_dir_, error))
        {
            return false;
        }
        // 本轮产生、需保留的受管文件；其余受管旧产物在清理时删除。
        std::vector<std::string> keep;

        // ---- 点云：改动六，挪到后台 ----
        // 点云只供第三流程可视化，**执行轨迹并不需要它**。第一版在这里花 510ms
        // 写 6.6MB ASCII PLY，客户端为此白等。现在先占位，manifest 里标
        // point_cloud.status="pending"，由 router 在发完 completed 之后写出并补发。
        const std::string cloud_name = "point_cloud_B.ply";
        keep.push_back(cloud_name);
        keep.push_back("collision_cloud_B.bin");

        TrajectoryOutputFormatAlgorithm::ManifestInput manifest;
        manifest.request_id = ctx.request_id;
        manifest.generation_id = ctx.generation_id;
        manifest.workpiece_type = ctx.workpiece_type;
        manifest.selected_candidate_index = ctx.selected_candidate_index;
        manifest.selected_grasp_rank = ctx.selected_grasp_rank;
        manifest.selected_place_rank = ctx.selected_place_rank;
        manifest.point_cloud.path = JoinPath(output_dir_, cloud_name);
        // sha256 与点数留空，待后台写完点云后补发（改动六）。
        manifest.point_cloud_status = "pending";
        manifest.collision_world = ctx.collision_world_meta;
        manifest.collision_model_id = ctx.collision_model_id;
        manifest.collision_model_sha256 = ctx.collision_model_sha256;
        manifest.joint_link_max_diff_deg = ctx.joint_link_max_diff_deg;
        manifest.joint_link_pass = ctx.joint_link_pass;
        manifest.statistics = BuildStatisticsJson(ctx.statistics);

        const std::string photo_name = "joint_trajectory_photo_to_grasp.json";
        const std::string place_name = "joint_trajectory_grasp_to_place.json";
        if (ctx.has_solution)
        {
            const nlohmann::json photo_doc = TrajectoryOutputFormatAlgorithm::BuildJointTrajectoryDocument(
                ctx.photo_to_grasp_trajectory, "photo_to_grasp", ctx.request_id, ctx.generation_id,
                ctx.workpiece_type, ctx.photo_to_grasp_trajectory.time_step_s);
            const nlohmann::json place_doc = TrajectoryOutputFormatAlgorithm::BuildJointTrajectoryDocument(
                ctx.grasp_to_place_trajectory, "grasp_to_place", ctx.request_id, ctx.generation_id,
                ctx.workpiece_type, ctx.grasp_to_place_trajectory.time_step_s);
            const std::string photo_text = photo_doc.dump(2) + "\n";
            const std::string place_text = place_doc.dump(2) + "\n";
            if (!WriteTextFileAtomic(JoinPath(output_dir_, photo_name), photo_text, error) ||
                !WriteTextFileAtomic(JoinPath(output_dir_, place_name), place_text, error))
            {
                // 写入失败即本轮失败；日志明确写入失败，不记录发布成功。
                LOG_ERROR << log_tag_ << "    [6/6] 轨迹写入失败: " << error;
                return false;
            }
            keep.push_back(photo_name);
            keep.push_back(place_name);
            manifest.photo_to_grasp.path = JoinPath(output_dir_, photo_name);
            manifest.photo_to_grasp.sha256 = TrajectoryOutputFormatAlgorithm::Sha256Hex(photo_text);
            manifest.photo_to_grasp.item_count = ctx.photo_to_grasp_trajectory.PointCount();
            manifest.grasp_to_place.path = JoinPath(output_dir_, place_name);
            manifest.grasp_to_place.sha256 = TrajectoryOutputFormatAlgorithm::Sha256Hex(place_text);
            manifest.grasp_to_place.item_count = ctx.grasp_to_place_trajectory.PointCount();
            manifest.status = "completed";
            manifest.ready = true;
            LOG_INFO << log_tag_ << "    [6/6] 抓取轨迹写入=成功 路径=" << manifest.photo_to_grasp.path;
            LOG_INFO << log_tag_ << "    [6/6] 放置轨迹写入=成功 路径=" << manifest.grasp_to_place.path;
        }
        else
        {
            // 无解：不发布轨迹，只留点云与失败状态；本轮候选保留供排查。
            // 不保留旧成功轨迹冒充当前结果，也不把"没找到"报成 completed。
            manifest.status = "failed";
            manifest.ready = false;
            manifest.reason = ctx.failure_reason.empty() ? "未找到 A/B 级完整周期" : ctx.failure_reason;
            LOG_INFO << log_tag_ << "    [6/6] 不发布轨迹 ready=false 原因=" << manifest.reason;
        }

        // ---- 第三流程预览元数据：带入最终选中候选，不重新计算位姿 ----
        if (!ctx.grasp_preview.is_null() && ctx.grasp_preview.is_object())
        {
            nlohmann::json preview = ctx.grasp_preview;
            preview["selected_candidate_index"] = ctx.selected_candidate_index;
            manifest.grasp_preview = preview;
            ctx.grasp_preview = preview;
        }

        // ---- 清理本轮不再产生的旧受管产物 ----
        // 本轮产生的候选文件已在 plan_full_cycle 写出；这里保留本轮代次的候选，
        // 删除上轮遗留的同类文件，避免旧 rank 文件或失败前旧结果混入。
        AppendCurrentGenerationCandidates(ctx, keep);
        std::vector<std::string> removed;
        std::string cleanup_error;
        const bool cleaned =
            OutputPublisher::CleanupManagedArtifacts(output_dir_, keep, removed, cleanup_error);
        if (!cleaned)
        {
            // 删除失败必须报出来，不能以"清理完成"掩盖。
            error = "旧产物清理失败: " + cleanup_error;
            LOG_ERROR << log_tag_ << "    [6/6] " << error;
            return false;
        }
        std::string removed_list;
        for (size_t index = 0; index < removed.size(); ++index)
        {
            removed_list += (index > 0 ? ", " : "") + removed[index];
        }
        LOG_INFO << log_tag_ << "    [6/6] 旧产物清理=完成 删除=" << removed.size()
                 << "个 受管文件=[" << removed_list << "]";

        // ---- 最后发布本轮 manifest ----
        const nlohmann::json document =
            TrajectoryOutputFormatAlgorithm::BuildManifestDocument(manifest, ctx.cycle_quality);
        if (!OutputPublisher::PublishManifest(output_dir_, document, error))
        {
            LOG_ERROR << log_tag_ << "    [6/6] manifest 发布失败: " << error;
            return false;
        }
        // 后台补发点云与归档时要复用同一份 manifest 输入，只改点云那几项。
        ctx.manifest_snapshot = document;
        LOG_INFO << log_tag_ << "    [6/6] 点云=挂起（后台写出） 产物覆盖=成功 "
                    "旧产物清理=完成 manifest发布=成功"
                 << " ready=" << (manifest.ready ? "true" : "false")
                 << " candidate=" << ctx.selected_candidate_index
                 << " grasp_rank=" << ctx.selected_grasp_rank
                 << " place_rank=" << ctx.selected_place_rank << " 等级="
                 << TrajectoryQualityGradeAlgorithm::GradeName(ctx.cycle_quality.grade);
        return true;
    }

  private:
    /**
     * @brief 把本轮实际写出的候选文件加入保留集合。
     *
     * 本轮候选按 photo_to_grasp_cand*／grasp_to_place_cand* 命名；这里按本轮
     * 尝试记录重建文件名，避免删掉刚写出的本轮候选。
     */
    static void AppendCurrentGenerationCandidates(const PipelineContext& ctx,
                                                  std::vector<std::string>& keep)
    {
        for (const PlanAttempt& attempt : ctx.attempts)
        {
            if (attempt.candidate_index < 0)
            {
                continue;
            }
            for (int64_t rank = 1; rank <= 8; ++rank)
            {
                keep.push_back("candidates/photo_to_grasp_cand" +
                               std::to_string(attempt.candidate_index) + "_rank" +
                               std::to_string(rank) + ".json");
                if (attempt.grasp_rank > 0)
                {
                    keep.push_back("candidates/grasp_to_place_cand" +
                                   std::to_string(attempt.candidate_index) + "_grasp_rank" +
                                   std::to_string(attempt.grasp_rank) + "_rank" +
                                   std::to_string(rank) + ".json");
                }
            }
        }
    }

    /** @brief 候选淘汰统计 → manifest 的 candidate_statistics 节。 */
    static nlohmann::json BuildStatisticsJson(const CandidateStatistics& statistics)
    {
        nlohmann::json node;
        node["grasp_targets_total"] = statistics.grasp_targets_total;
        node["grasp_targets_attempted"] = statistics.grasp_targets_attempted;
        node["prefilter_rejected"] = statistics.prefilter_rejected;
        node["gpu_failed"] = statistics.gpu_failed;
        node["grasp_trajectories_returned"] = statistics.grasp_trajectories_returned;
        node["grade_rejected"] = statistics.grade_rejected;
        node["review_rejected"] = statistics.review_rejected;
        node["place_trajectories_returned"] = statistics.place_trajectories_returned;
        node["place_plan_failed"] = statistics.place_plan_failed;
        node["place_grade_rejected"] = statistics.place_grade_rejected;
        node["place_review_rejected"] = statistics.place_review_rejected;
        node["stop_reason"] = statistics.stop_reason;
        return node;
    }

    std::string output_dir_; ///< output/trajectory_planning
};

REGISTER_STAGE("write_trajectory_output", WriteTrajectoryOutputStage);

} // namespace openmind::trajectory_plan
