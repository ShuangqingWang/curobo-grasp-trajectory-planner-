/**
 * @file output_publisher.cpp
 * @brief output 覆盖与一致性实现。
 */

#include "trajectory_plan/service/output_publisher.h"

#include <algorithm>
#include <filesystem>

#include "trajectory_plan/tools/json_util.h"
#include "trajectory_plan/tools/log.h"

namespace openmind::trajectory_plan
{
namespace
{

constexpr char kManifestName[] = "trajectory_manifest.json";

/**
 * @brief 本规划器管理的固定产物（相对 trajectory_planning 目录）。
 *
 * 前四项是当前版本的正式产物；其余是历史版本留下的同类产物，同样归本规划器管理，
 * 必须随本轮一起清掉，否则消费端会把上一代次的旧结果当成本轮结果。
 * 清理只作用于这张表和 candidates/ 下的 .json，不碰 output 下的无关文件。
 */
const char* const kManagedFiles[] = {
    "joint_trajectory_photo_to_grasp.json",
    "joint_trajectory_grasp_to_place.json",
    "point_cloud_B.ply",
    "collision_depth_main.f32",
    "collision_depth_left.f32",
    "collision_depth_right.f32",
    // ---- 历史版本产物 ----
    "collision_cloud_B.bin",
    "joint_trajectory_full_cycle_preview.json",
    "realtime_plan_diagnostics.json",
    "request_with_generated_grasp.txt",
    "verified_trajectory_path.png",
    "best_grasp.json",
};

/**
 * @brief 组装带状态的最小 manifest（processing / failed 用）。
 */
nlohmann::json BuildStatusManifest(const std::string& status,
                                   const std::string& request_id,
                                   const std::string& generation_id,
                                   const std::string& workpiece_type)
{
    nlohmann::json document;
    document["schema"] = "trajectory_manifest/v2";
    document["status"] = status;
    document["ready"] = false;
    document["request_id"] = request_id;
    document["generation_id"] = generation_id;
    document["workpiece_type"] = workpiece_type;
    return document;
}

} // namespace

bool OutputPublisher::PurgeForeignGeneration(const std::string& trajectory_output_dir,
                                             const std::string& grasp_output_dir,
                                             const std::string& generation_id,
                                             bool keep_cloud_binary,
                                             std::vector<std::string>& removed,
                                             std::string& error)
{
    removed.clear();
    error.clear();
    std::error_code code;

    auto remove_path = [&](const std::string& path, const std::string& label) {
        if (!std::filesystem::exists(path, code))
        {
            return true;
        }
        std::filesystem::remove(path, code);
        if (code)
        {
            error = "删除 " + label + " 失败: " + code.message();
            return false;
        }
        removed.push_back(label);
        return true;
    };

    // 带代次标识的文件：只删不属于本轮的。
    auto belongs_to_round = [&generation_id](const std::filesystem::path& path) {
        std::string content;
        std::string read_error;
        if (!ReadTextFile(path.string(), content, read_error))
        {
            return false;
        }
        nlohmann::json document;
        if (!ParseJson(content, document, read_error) || !document.is_object())
        {
            return false;
        }
        return document.value("generation_id", std::string()) == generation_id;
    };

    // 两条正式轨迹与点云：异常失败时输出 stage 没跑完，一定是上轮遗留。
    for (const char* stale : {"joint_trajectory_photo_to_grasp.json",
                              "joint_trajectory_grasp_to_place.json", "point_cloud_B.ply"})
    {
        if (!remove_path(JoinPath(trajectory_output_dir, stale), stale))
        {
            return false;
        }
    }
    // 碰撞世界的三路深度文件：本轮确实已下发时保留，否则一定是上轮遗留。
    if (!keep_cloud_binary)
    {
        for (const char* stale : {"collision_depth_main.f32", "collision_depth_left.f32",
                                  "collision_depth_right.f32"})
        {
            if (!remove_path(JoinPath(trajectory_output_dir, stale), stale))
            {
                return false;
            }
        }
    }
    if (!remove_path(JoinPath(trajectory_output_dir, "collision_cloud_B.bin"), "collision_cloud_B.bin"))
    {
        return false;
    }

    const std::string grasp_result = JoinPath(grasp_output_dir, "grasp_result.json");
    if (std::filesystem::exists(grasp_result, code) && !belongs_to_round(grasp_result) &&
        !remove_path(grasp_result, "grasp_generation/grasp_result.json"))
    {
        return false;
    }

    const std::string candidates_dir = JoinPath(trajectory_output_dir, "candidates");
    if (std::filesystem::exists(candidates_dir, code))
    {
        for (const auto& entry : std::filesystem::directory_iterator(candidates_dir, code))
        {
            if (!entry.is_regular_file() || entry.path().extension() != ".json")
            {
                continue;
            }
            if (belongs_to_round(entry.path()))
            {
                continue; // 本轮已产生的有效候选保留供排查
            }
            if (!remove_path(entry.path().string(),
                             "candidates/" + entry.path().filename().string()))
            {
                return false;
            }
        }
    }
    return true;
}

std::string OutputPublisher::ManifestPath(const std::string& trajectory_output_dir)
{
    return JoinPath(trajectory_output_dir, kManifestName);
}

bool OutputPublisher::PublishProcessing(const std::string& trajectory_output_dir,
                                        const std::string& request_id,
                                        const std::string& generation_id,
                                        const std::string& workpiece_type,
                                        std::string& error)
{
    if (!EnsureDirectory(trajectory_output_dir, error))
    {
        return false;
    }
    nlohmann::json document =
        BuildStatusManifest("processing", request_id, generation_id, workpiece_type);
    document["reason"] = "本轮任务处理中，上轮结果已失效";
    return WriteTextFileAtomic(ManifestPath(trajectory_output_dir), document.dump(2) + "\n", error);
}

bool OutputPublisher::PublishFailed(const std::string& trajectory_output_dir,
                                    const std::string& request_id,
                                    const std::string& generation_id,
                                    const std::string& workpiece_type,
                                    const std::string& failed_flow,
                                    const std::string& failed_stage,
                                    const std::string& reason,
                                    std::string& error)
{
    if (!EnsureDirectory(trajectory_output_dir, error))
    {
        return false;
    }
    nlohmann::json document =
        BuildStatusManifest("failed", request_id, generation_id, workpiece_type);
    if (!failed_flow.empty())
    {
        document["failed_flow"] = failed_flow;
    }
    if (!failed_stage.empty())
    {
        document["failed_stage"] = failed_stage;
    }
    document["reason"] = reason;
    return WriteTextFileAtomic(ManifestPath(trajectory_output_dir), document.dump(2) + "\n", error);
}

bool OutputPublisher::PublishManifest(const std::string& trajectory_output_dir,
                                      const nlohmann::json& manifest,
                                      std::string& error)
{
    if (!EnsureDirectory(trajectory_output_dir, error))
    {
        return false;
    }
    return WriteTextFileAtomic(ManifestPath(trajectory_output_dir), manifest.dump(2) + "\n", error);
}

bool OutputPublisher::CleanupManagedArtifacts(const std::string& trajectory_output_dir,
                                              const std::vector<std::string>& keep_relative_paths,
                                              std::vector<std::string>& removed,
                                              std::string& error)
{
    removed.clear();
    error.clear();
    auto keep = [&keep_relative_paths](const std::string& relative) {
        return std::find(keep_relative_paths.begin(), keep_relative_paths.end(), relative) !=
               keep_relative_paths.end();
    };
    auto remove_one = [&](const std::string& relative) {
        const std::string path = JoinPath(trajectory_output_dir, relative);
        std::error_code code;
        if (!std::filesystem::exists(path, code))
        {
            return true;
        }
        std::filesystem::remove(path, code);
        if (code)
        {
            // 删除失败必须报出来，不能以"清理完成"掩盖。
            error = "删除旧产物 " + relative + " 失败: " + code.message();
            return false;
        }
        removed.push_back(relative);
        return true;
    };

    for (const char* managed : kManagedFiles)
    {
        if (!keep(managed) && !remove_one(managed))
        {
            return false;
        }
    }

    // 原子写入中断留下的临时文件（.<名字>.<pid>.tmp）：既不是本轮产物，也会被归档
    // 误收，一并清掉。
    std::error_code scan_code;
    if (std::filesystem::exists(trajectory_output_dir, scan_code))
    {
        for (const auto& entry :
             std::filesystem::directory_iterator(trajectory_output_dir, scan_code))
        {
            if (!entry.is_regular_file())
            {
                continue;
            }
            const std::string name = entry.path().filename().string();
            if (name.size() > 4 && name.front() == '.' &&
                name.compare(name.size() - 4, 4, ".tmp") == 0 && !keep(name) && !remove_one(name))
            {
                return false;
            }
        }
    }

    // candidates/ 下的历史候选：本轮没再产生的一律清掉，避免上轮 rank 文件混入。
    const std::string candidates_dir = JoinPath(trajectory_output_dir, "candidates");
    std::error_code code;
    if (std::filesystem::exists(candidates_dir, code))
    {
        for (const auto& entry : std::filesystem::directory_iterator(candidates_dir, code))
        {
            if (!entry.is_regular_file())
            {
                continue;
            }
            if (entry.path().extension() != ".json")
            {
                continue;
            }
            const std::string relative = "candidates/" + entry.path().filename().string();
            if (!keep(relative) && !remove_one(relative))
            {
                return false;
            }
        }
    }
    return true;
}

} // namespace openmind::trajectory_plan
