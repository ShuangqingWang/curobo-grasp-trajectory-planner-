/**
 * @file load_request_stage.cpp
 * @brief flow grasp / 一 load_request：校验本轮工件类型、工件位姿与关联标识。
 *
 * 业务步骤 [1/4]。输入来自 TCP 入口创建的本轮上下文，客户端已发送完整内容，
 * 本 stage 不再读取 service 侧的旧 input 文件。
 */

#include <algorithm>

#include "trajectory_plan/pipeline/stage_registry.h"
#include "trajectory_plan/tools/json_util.h"
#include "trajectory_plan/tools/log.h"

namespace openmind::trajectory_plan
{

class LoadRequestStage : public StageBase
{
  public:
    bool Init(const StageConfig& config, std::string& error) override
    {
        BindConfig(config);
        if (config.flow_config == nullptr)
        {
            error = "缺少 grasp.yaml";
            return false;
        }
        // 启动即确定支持的工件类型；触发时只做归属判断，不再解析映射表。
        const nlohmann::json& mapping = (*config.flow_config)["grasp_label_files"];
        if (!mapping.is_object() || mapping.empty())
        {
            error = "grasp.yaml 的 grasp_label_files 为空";
            return false;
        }
        supported_workpieces_.clear();
        for (auto iterator = mapping.begin(); iterator != mapping.end(); ++iterator)
        {
            supported_workpieces_.push_back(iterator.key());
        }
        std::string list;
        for (size_t index = 0; index < supported_workpieces_.size(); ++index)
        {
            list += (index > 0 ? "," : "") + supported_workpieces_[index];
        }
        LOG_INFO << log_tag_ << "[初始化] 支持工件=[" << list << "]";
        return true;
    }

    bool Execute(PipelineContext& ctx, std::string& error) override
    {
        // [1/4] 校验请求：工件类型、工件位姿、本轮标识。
        if (ctx.request_id.empty() || ctx.generation_id.empty())
        {
            error = "本轮标识缺失（request_id / generation_id）";
            return false;
        }
        if (ctx.workpiece_type.empty())
        {
            error = "工件类型为空";
            return false;
        }
        if (std::find(supported_workpieces_.begin(), supported_workpieces_.end(), ctx.workpiece_type) ==
            supported_workpieces_.end())
        {
            error = "未知工件类型: " + ctx.workpiece_type;
            return false;
        }
        if (!ctx.has_t_b_o)
        {
            error = "本轮未提供 T_B_O_target";
            return false;
        }
        // 位姿必须是有限数值的 4×4 刚体变换：末行合法、旋转正交且为右手系。
        if (!ValidateTransform(ctx.t_b_o, "T_B_O_target", error))
        {
            return false;
        }
        LOG_INFO << log_tag_ << "    [1/4] 输入校验 工件=" << ctx.workpiece_type
                 << " T_B_O=" << FormatPose(PoseFromTransform(ctx.t_b_o)) << " 结果=通过";
        return true;
    }

  private:
    std::vector<std::string> supported_workpieces_; ///< 启动配置声明支持的工件
};

REGISTER_STAGE("load_request", LoadRequestStage);

} // namespace openmind::trajectory_plan
