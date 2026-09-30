/**
 * @file response_builder.cpp
 * @brief 外部 TCP 应答排版实现。
 */

#include "trajectory_plan/service/response_builder.h"

#include <nlohmann/json.hpp>

namespace openmind::trajectory_plan
{
namespace
{

/**
 * @brief 把归档结果写进应答；归档失败不改判规划结果，只增加状态字段。
 */
void AppendArchive(nlohmann::json& document, const ArchiveStatus& archive)
{
    document["archive_status"] = archive.status;
    if (!archive.path.empty())
    {
        document["archive_path"] = archive.path;
    }
    if (!archive.error.empty())
    {
        document["archive_error"] = archive.error;
    }
}

} // namespace

std::string ResponseBuilder::Accepted(const std::string& request_id, const std::string& generation_id)
{
    nlohmann::json document;
    document["status"] = "accepted";
    document["request_id"] = request_id;
    document["generation_id"] = generation_id;
    return document.dump();
}

std::string ResponseBuilder::Rejected(const std::string& request_id, const std::string& reason)
{
    nlohmann::json document;
    document["status"] = "rejected";
    document["request_id"] = request_id;
    document["reason"] = reason;
    return document.dump();
}

std::string ResponseBuilder::Busy(const std::string& reason)
{
    nlohmann::json document;
    document["status"] = "busy";
    document["reason"] = reason;
    return document.dump();
}

std::string ResponseBuilder::Completed(const std::string& request_id,
                                       const std::string& generation_id,
                                       bool ready,
                                       const std::string& manifest_path)
{
    nlohmann::json document;
    document["status"] = "completed";
    document["request_id"] = request_id;
    document["generation_id"] = generation_id;
    document["ready"] = ready;
    document["manifest_path"] = manifest_path;
    return document.dump();
}

std::string ResponseBuilder::Failed(const std::string& request_id,
                                    const std::string& generation_id,
                                    const std::string& failed_flow,
                                    const std::string& failed_stage,
                                    const std::string& reason)
{
    nlohmann::json document;
    document["status"] = "failed";
    document["request_id"] = request_id;
    document["generation_id"] = generation_id;
    document["ready"] = false;
    if (!failed_flow.empty())
    {
        document["failed_flow"] = failed_flow;
    }
    if (!failed_stage.empty())
    {
        document["failed_stage"] = failed_stage;
    }
    document["reason"] = reason;
    return document.dump();
}

std::string ResponseBuilder::Archived(const std::string& request_id,
                                      const std::string& generation_id,
                                      bool point_cloud_ready,
                                      const ArchiveStatus& archive)
{
    nlohmann::json document;
    document["status"] = "archived";
    document["request_id"] = request_id;
    document["generation_id"] = generation_id;
    document["point_cloud_ready"] = point_cloud_ready;
    AppendArchive(document, archive);
    return document.dump();
}

} // namespace openmind::trajectory_plan
