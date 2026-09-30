/**
 * @file pipeline.cpp
 * @brief 组合器实现：启动构造并初始化 stage 链，触发后顺序执行并统一打日志。
 */

#include "trajectory_plan/pipeline/pipeline.h"

#include <chrono>

#include "trajectory_plan/pipeline/stage_registry.h"
#include "trajectory_plan/tools/log.h"

namespace openmind::trajectory_plan
{
namespace
{

/// stage 使用中文数字编号；每条 flow 的 stage 独立从"一"开始。
const char* const kOrdinals[] = {"一", "二", "三", "四", "五", "六", "七", "八", "九", "十"};

/**
 * @brief 第 index 个 stage 的中文编号（越界时退回阿拉伯数字）。
 */
std::string Ordinal(size_t index)
{
    if (index < sizeof(kOrdinals) / sizeof(kOrdinals[0]))
    {
        return kOrdinals[index];
    }
    return std::to_string(index + 1);
}

} // namespace

StageRegistry& StageRegistry::Instance()
{
    static StageRegistry instance;
    return instance;
}

bool StageRegistry::Register(const std::string& type, std::function<std::unique_ptr<IStage>()> factory)
{
    if (type.empty() || factories_.count(type) != 0)
    {
        return false;
    }
    factories_[type] = std::move(factory);
    return true;
}

std::unique_ptr<IStage> StageRegistry::Create(const std::string& type) const
{
    const auto iterator = factories_.find(type);
    if (iterator == factories_.end())
    {
        return nullptr;
    }
    return iterator->second();
}

std::vector<std::string> StageRegistry::RegisteredNames() const
{
    std::vector<std::string> names;
    names.reserve(factories_.size());
    for (const auto& item : factories_)
    {
        names.push_back(item.first);
    }
    return names;
}

bool Pipeline::Init(const nlohmann::json& pipeline_config,
                    const PipelineStartupResources& resources,
                    const std::vector<std::string>& wanted_flows,
                    std::string& error)
{
    if (!pipeline_config.is_object())
    {
        error = "pipeline 配置必须是对象";
        return false;
    }
    // 全量解析 flow 名字表：报错时能列全可选项。
    flow_names_.clear();
    for (auto iterator = pipeline_config.begin(); iterator != pipeline_config.end(); ++iterator)
    {
        if (!iterator.value().is_array())
        {
            error = "pipeline 的 " + iterator.key() + " 必须是 stage 名字数组";
            return false;
        }
        flow_names_.push_back(iterator.key());
    }

    chains_.clear();
    enabled_flows_ = wanted_flows;
    for (const std::string& flow_name : wanted_flows)
    {
        if (!pipeline_config.contains(flow_name))
        {
            error = "unknown flow '" + flow_name + "', available: ";
            for (size_t index = 0; index < flow_names_.size(); ++index)
            {
                error += (index > 0 ? ", " : "") + flow_names_[index];
            }
            return false;
        }
        const nlohmann::json& stage_names = pipeline_config[flow_name];
        LOG_INFO << "[flow=" << flow_name << "][初始化开始] stage数量=" << stage_names.size();
        const auto flow_started = std::chrono::steady_clock::now();

        std::vector<std::unique_ptr<IStage>> chain;
        size_t stage_index = 0;
        for (const nlohmann::json& stage_name : stage_names)
        {
            StageConfig config;
            config.id = stage_name.get<std::string>();
            config.type = config.id;
            config.flow = flow_name;
            config.ordinal = Ordinal(stage_index);
            config.params = nlohmann::json::object();
            config.config_root = resources.config_root;
            const auto flow_config = resources.flow_configs.find(flow_name);
            config.flow_config =
                (flow_config != resources.flow_configs.end()) ? flow_config->second : nullptr;
            config.kinematics = resources.kinematics;
            config.device_manager = resources.device_manager;
            config.camera_manager = resources.camera_manager;
            config.grasp_output_dir = resources.grasp_output_dir;
            config.trajectory_output_dir = resources.trajectory_output_dir;

            std::unique_ptr<IStage> stage = StageRegistry::Instance().Create(config.type);
            if (!stage)
            {
                error = "unknown stage '" + config.type + "' in flow '" + flow_name + "'";
                return false;
            }
            const auto stage_started = std::chrono::steady_clock::now();
            std::string stage_error;
            if (!stage->Init(config, stage_error))
            {
                error = "flow " + flow_name + " 的 stage " + config.type + " 初始化失败" +
                        (stage_error.empty() ? "" : ": " + stage_error);
                LOG_ERROR << "[flow=" << flow_name << "]" << config.LogTag()
                          << "[初始化失败] 原因=" << stage_error;
                return false;
            }
            const double stage_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - stage_started)
                    .count();
            LOG_INFO << "[flow=" << flow_name << "]" << config.LogTag()
                     << "[初始化完成] 耗时=" << stage_ms << "ms";
            chain.push_back(std::move(stage));
            ++stage_index;
        }
        const double flow_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - flow_started).count();
        LOG_INFO << "[flow=" << flow_name << "][初始化完成] 总耗时=" << flow_ms << "ms";
        chains_[flow_name] = std::move(chain);
    }
    return true;
}

bool Pipeline::Run(const std::string& flow_name, PipelineContext& ctx, FlowResult& result)
{
    result = FlowResult();
    const auto chain_iterator = chains_.find(flow_name);
    if (chain_iterator == chains_.end())
    {
        result.success = false;
        result.reason = "flow 未初始化: " + flow_name;
        return false;
    }

    SetLogContext("[flow=" + flow_name + "]");
    LOG_INFO << "[开始] request_id=" << ctx.request_id << " generation_id=" << ctx.generation_id;
    const auto flow_start = std::chrono::steady_clock::now();

    const auto& chain = chain_iterator->second;
    for (size_t index = 0; index < chain.size(); ++index)
    {
        const auto& stage = chain[index];
        if (!result.success)
        {
            // 失败之后的 stage 记录 skipped 及原因，不能记为通过。
            LOG_INFO << stage->GetLogTag() << "[跳过] 原因=上游 " << result.failed_stage << " 失败";
            continue;
        }
        LOG_INFO << stage->GetLogTag() << "[开始]";
        const auto stage_start = std::chrono::steady_clock::now();
        std::string stage_error;
        const bool ok = stage->Execute(ctx, stage_error);
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - stage_start).count();
        ctx.stage_timings_ms[flow_name + "/" + stage->GetName()] = elapsed_ms;
        if (!ok)
        {
            LOG_ERROR << stage->GetLogTag() << "[失败] 原因=" << stage_error
                      << " 耗时=" << elapsed_ms << "ms";
            result.success = false;
            result.failed_stage = stage->GetName();
            result.reason = stage_error;
            continue;
        }
        LOG_INFO << stage->GetLogTag() << "[完成] 耗时=" << elapsed_ms << "ms";
    }

    result.elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - flow_start).count();
    if (result.success)
    {
        LOG_INFO << "[完成] 结果=成功 总耗时=" << result.elapsed_ms << "ms";
    }
    else
    {
        LOG_ERROR << "[完成] 结果=失败 失败位置=" << result.failed_stage
                  << " 原因=" << result.reason << " 总耗时=" << result.elapsed_ms << "ms";
    }
    ClearLogContext();
    return result.success;
}

std::vector<std::string> Pipeline::FlowNames() const
{
    return flow_names_;
}

const std::vector<std::string>& Pipeline::EnabledFlows() const
{
    return enabled_flows_;
}

} // namespace openmind::trajectory_plan
