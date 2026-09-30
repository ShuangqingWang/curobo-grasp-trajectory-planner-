/**
 * @file load_grasp_labels_stage.cpp
 * @brief flow grasp / 二 load_grasp_labels：启动加载全部启用工件的 NPZ 并建立只读缓存。
 *
 * 业务步骤 [2/4]。启动时按 asset_root / object_directory / grasp_label / label_file
 * 定位文件，校验、按夹爪宽度上限过滤、合并同工件全部有效标签、按 score 降序稳定排序
 * 并缓存；触发后只按工件取缓存的前 candidate_count 条，不重复打开 NPZ。
 *
 * 启动配置声明支持的工件若标签缺失、文件损坏或过滤后为空，初始化失败并指出工件和文件，
 * 不把该工件悄悄留到业务触发时才加载。
 */

#include <algorithm>

#include "trajectory_plan/algorithm/grasp_label_load_algorithm.h"
#include "trajectory_plan/pipeline/stage_registry.h"
#include "trajectory_plan/tools/json_util.h"
#include "trajectory_plan/tools/log.h"

namespace openmind::trajectory_plan
{
namespace
{

/**
 * @brief 一个工件的标签缓存及来源统计。
 */
struct WorkpieceLabelCache
{
    std::vector<GraspLabel> labels;      ///< 过滤并按 score 降序稳定排序后的有效标签
    std::vector<std::string> source_files; ///< 来源 NPZ 文件
    int64_t raw_count = 0;               ///< 原始标签数
    int64_t width_filtered = 0;          ///< 宽度过滤掉的数量
};

} // namespace

class LoadGraspLabelsStage : public StageBase
{
  public:
    bool Init(const StageConfig& config, std::string& error) override
    {
        BindConfig(config);
        if (config.flow_config == nullptr || config.config_root == nullptr)
        {
            error = "缺少 grasp.yaml 或启动配置";
            return false;
        }
        const nlohmann::json& grasp_cfg = *config.flow_config;
        const nlohmann::json& app = (*config.config_root)["app"];
        max_width_m_ = app.value("gripper_width_max_m", 0.1);
        candidate_count_ = grasp_cfg.value("candidate_count", 10);
        const std::string asset_root = grasp_cfg.value("asset_root", "");
        const nlohmann::json& mapping = grasp_cfg["grasp_label_files"];
        if (!mapping.is_object() || mapping.empty())
        {
            error = "grasp.yaml 的 grasp_label_files 为空";
            return false;
        }

        cache_.clear();
        for (auto iterator = mapping.begin(); iterator != mapping.end(); ++iterator)
        {
            const std::string workpiece = iterator.key();
            // 同一工件允许多个 NPZ；保留工件映射，包括 QR00010 → QR0010 目录映射。
            std::string object_directory = workpiece;
            std::vector<std::string> label_files;
            const nlohmann::json& entry = iterator.value();
            if (entry.is_string())
            {
                label_files.push_back(entry.get<std::string>());
            }
            else if (entry.is_object())
            {
                object_directory = entry.value("object_directory", workpiece);
                if (entry.contains("label_file") && entry["label_file"].is_string())
                {
                    label_files.push_back(entry["label_file"].get<std::string>());
                }
                if (entry.contains("label_files") && entry["label_files"].is_array())
                {
                    for (const nlohmann::json& item : entry["label_files"])
                    {
                        label_files.push_back(item.get<std::string>());
                    }
                }
            }
            if (label_files.empty())
            {
                error = "工件 " + workpiece + " 没有配置 NPZ 文件";
                return false;
            }

            WorkpieceLabelCache cache;
            for (const std::string& file : label_files)
            {
                const std::string path =
                    JoinPath(JoinPath(JoinPath(asset_root, object_directory), "grasp_label"), file);
                GraspLabelLoadAlgorithm loader;
                std::string load_error;
                if (!loader.LoadLabels(path, max_width_m_, load_error))
                {
                    error = "工件 " + workpiece + " 的标签文件 " + path + " 加载失败: " + load_error;
                    return false;
                }
                const std::vector<GraspLabel> labels = loader.TopLabels(std::numeric_limits<int64_t>::max());
                cache.labels.insert(cache.labels.end(), labels.begin(), labels.end());
                cache.source_files.push_back(file);
                cache.raw_count += loader.RawLabelCount();
                cache.width_filtered += loader.WidthFilteredCount();
            }
            // 合并后按 score 降序稳定排序：同分保持配置文件顺序及文件内原始顺序。
            std::stable_sort(cache.labels.begin(), cache.labels.end(),
                             [](const GraspLabel& left, const GraspLabel& right) {
                                 return left.score > right.score;
                             });
            if (cache.labels.empty())
            {
                error = "工件 " + workpiece + " 过滤后没有有效标签（宽度上限 " +
                        std::to_string(max_width_m_) + " m）";
                return false;
            }
            std::string files;
            for (size_t index = 0; index < cache.source_files.size(); ++index)
            {
                files += (index > 0 ? "," : "") + cache.source_files[index];
            }
            LOG_INFO << log_tag_ << "[初始化] 工件=" << workpiece << " 文件=[" << files
                     << "] 原始标签=" << cache.raw_count << " 宽度过滤=" << cache.width_filtered
                     << " 有效标签=" << cache.labels.size();
            cache_.emplace(workpiece, std::move(cache));
        }
        LOG_INFO << log_tag_ << "[初始化] 缓存工件数=" << cache_.size()
                 << " 宽度上限=" << max_width_m_ << "m 候选上限=" << candidate_count_;
        return true;
    }

    bool Execute(PipelineContext& ctx, std::string& error) override
    {
        const auto iterator = cache_.find(ctx.workpiece_type);
        if (iterator == cache_.end())
        {
            error = "标签缓存中没有工件 " + ctx.workpiece_type;
            return false;
        }
        const WorkpieceLabelCache& cache = iterator->second;
        if (cache.labels.empty())
        {
            error = "工件 " + ctx.workpiece_type + " 没有有效标签";
            return false;
        }
        // 取前 candidate_count 条；不足则使用实际数量，不复制补足。
        const size_t take = std::min(cache.labels.size(), static_cast<size_t>(candidate_count_));
        ctx.workpiece_labels.assign(cache.labels.begin(),
                                    cache.labels.begin() + static_cast<long>(take));
        ctx.label_pool_size = static_cast<int64_t>(cache.labels.size());
        LOG_INFO << log_tag_ << "    [2/4] 标签选取 工件=" << ctx.workpiece_type << " 缓存=复用"
                 << " 有效标签=" << cache.labels.size() << " 请求候选=" << candidate_count_
                 << " 实际候选=" << ctx.workpiece_labels.size() << " 排序=score降序";
        return true;
    }

  private:
    std::map<std::string, WorkpieceLabelCache> cache_; ///< 只读标签缓存（启动建立）
    double max_width_m_ = 0.1;      ///< 夹爪宽度上限
    int64_t candidate_count_ = 10;  ///< 每轮取用候选数上限
};

REGISTER_STAGE("load_grasp_labels", LoadGraspLabelsStage);

} // namespace openmind::trajectory_plan
