/**
 * @file write_grasp_output_stage.cpp
 * @brief flow grasp / 四 write_grasp_output：原子发布本轮 grasp_result.json。
 *
 * 业务步骤 [4/4]。根级包含 request_id、generation_id、workpiece_type 和 candidates；
 * 原子写入成功后才宣告第一流程成功。第一流程发布成功不设置轨迹 ready=true。
 */

#include "trajectory_plan/algorithm/grasp_output_format_algorithm.h"
#include "trajectory_plan/pipeline/stage_registry.h"
#include "trajectory_plan/tools/json_util.h"
#include "trajectory_plan/tools/log.h"

namespace openmind::trajectory_plan
{

class WriteGraspOutputStage : public StageBase
{
  public:
    bool Init(const StageConfig& config, std::string& error) override
    {
        BindConfig(config);
        output_dir_ = config.grasp_output_dir;
        if (output_dir_.empty())
        {
            error = "grasp 输出目录未配置";
            return false;
        }
        // 启动即准备输出目录与原子写入模块。
        if (!EnsureDirectory(output_dir_, error))
        {
            return false;
        }
        LOG_INFO << log_tag_ << "[初始化] 目录=" << output_dir_;
        return true;
    }

    bool Execute(PipelineContext& ctx, std::string& error) override
    {
        if (ctx.grasp_candidates.empty())
        {
            error = "没有候选可发布";
            return false;
        }
        if (!EnsureDirectory(output_dir_, error))
        {
            return false;
        }
        const nlohmann::json document = GraspOutputFormatAlgorithm::BuildGraspResultDocument(
            ctx.grasp_candidates, ctx.request_id, ctx.generation_id, ctx.workpiece_type);
        const std::string path = JoinPath(output_dir_, "grasp_result.json");
        if (!WriteTextFileAtomic(path, document.dump(2) + "\n", error))
        {
            // 写入失败即第一流程失败：上轮文件即使仍在磁盘也不能当成本轮输入。
            return false;
        }
        ctx.grasp_result_path = path;
        // 第三流程的夹爪预览元数据在这里成形；选中候选由第二流程填入。
        ctx.grasp_preview =
            GraspOutputFormatAlgorithm::BuildGraspPreview(ctx.grasp_candidates, ctx.clearance_mm, -1);
        LOG_INFO << log_tag_ << "    [4/4] 发布候选 文件=" << path
                 << " request_id=" << ctx.request_id << " generation_id=" << ctx.generation_id
                 << " 候选=" << ctx.grasp_candidates.size()
                 << " 退让=" << ctx.clearance_mm << "mm 原子写入=成功";
        return true;
    }

  private:
    std::string output_dir_; ///< output/grasp_generation
};

REGISTER_STAGE("write_grasp_output", WriteGraspOutputStage);

} // namespace openmind::trajectory_plan
