/**
 * @file session_archive.cpp
 * @brief 会话日志与逐轮归档实现。
 */

#include "trajectory_plan/service/session_archive.h"

#include <sys/stat.h>
#include <sys/types.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "trajectory_plan/tools/json_util.h"
#include "trajectory_plan/tools/log.h"

namespace openmind::trajectory_plan
{
namespace
{

/**
 * @brief 本地时间戳 YYYYMMDD_HHMMSS_mmm（会话目录与触发目录共用）。
 */
std::string TimestampWithMilliseconds()
{
    const auto now = std::chrono::system_clock::now();
    const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - seconds).count();
    const std::time_t raw = std::chrono::system_clock::to_time_t(now);
    std::tm local {};
    localtime_r(&raw, &local);
    std::ostringstream text;
    text << std::put_time(&local, "%Y%m%d_%H%M%S") << "_" << std::setfill('0') << std::setw(3)
         << milliseconds;
    return text.str();
}

/**
 * @brief 本地时区 ISO8601 时间串，写进 session.json / result.json。
 */
std::string LocalIso8601()
{
    const std::time_t raw = std::time(nullptr);
    std::tm local {};
    localtime_r(&raw, &local);
    std::ostringstream text;
    text << std::put_time(&local, "%Y-%m-%dT%H:%M:%S%z");
    return text.str();
}

/**
 * @brief 当前时区名（session.json 里统一记录）。
 */
std::string LocalTimezone()
{
    const std::time_t raw = std::time(nullptr);
    std::tm local {};
    localtime_r(&raw, &local);
    char buffer[64] {};
    std::strftime(buffer, sizeof(buffer), "%Z", &local);
    return buffer;
}

/**
 * @brief 原样写二进制文件。
 */
bool WriteBinaryFile(const std::string& path, const void* data, size_t size, std::string& error)
{
    std::ofstream stream(path, std::ios::out | std::ios::binary | std::ios::trunc);
    if (!stream.is_open())
    {
        error = "无法写入 " + path + ": " + std::strerror(errno);
        return false;
    }
    if (size > 0)
    {
        stream.write(static_cast<const char*>(data), static_cast<std::streamsize>(size));
    }
    stream.close();
    if (!stream)
    {
        error = "写入 " + path + " 失败";
        return false;
    }
    return true;
}

} // namespace

bool SessionArchive::OpenSession(const std::string& log_root, std::string& error)
{
    if (log_root.empty())
    {
        error = "log_root 为空";
        return false;
    }
    std::error_code code;
    std::filesystem::create_directories(log_root, code);
    if (code)
    {
        error = "无法创建会话日志根目录 " + log_root + ": " + code.message();
        return false;
    }

    // 目录名包含日期、时间、毫秒；重名时增加唯一后缀，不复用旧会话。
    const std::string stamp = TimestampWithMilliseconds();
    std::string candidate = JoinPath(log_root, stamp);
    int32_t suffix = 1;
    while (std::filesystem::exists(candidate))
    {
        candidate = JoinPath(log_root, stamp + "_" + std::to_string(suffix));
        ++suffix;
    }
    std::filesystem::create_directories(candidate, code);
    if (code)
    {
        error = "无法创建会话目录 " + candidate + ": " + code.message();
        return false;
    }

    session_dir_ = candidate;
    session_id_ = std::filesystem::path(candidate).filename().string();
    started_at_ = LocalIso8601();
    ready_ = true;
    trigger_sequence_ = 0;

    // service.log 覆盖整个会话：从启动、预热到退出。
    EnableServiceLogMirror(JoinPath(session_dir_, "service.log"));
    return true;
}

bool SessionArchive::WriteSessionJson(const nlohmann::json& config_snapshot,
                                      const std::string& status,
                                      const std::string& detail)
{
    if (!ready_)
    {
        return false;
    }
    nlohmann::json document;
    document["session_id"] = session_id_;
    document["session_dir"] = session_dir_;
    document["started_at"] = started_at_;
    document["updated_at"] = LocalIso8601();
    document["timezone"] = LocalTimezone();
    document["status"] = status;
    document["detail"] = detail;
    document["service_version"] = "1.0.0";
    document["config"] = config_snapshot;
    std::string error;
    if (!WriteTextFileAtomic(JoinPath(session_dir_, "session.json"), document.dump(2) + "\n", error))
    {
        LOG_ERROR << "[service][会话] session.json 写入失败: " << error;
        return false;
    }
    return true;
}

const std::string& SessionArchive::SessionDir() const
{
    return session_dir_;
}

const std::string& SessionArchive::SessionId() const
{
    return session_id_;
}

bool SessionArchive::SessionReady() const
{
    return ready_;
}

TriggerArchive SessionArchive::BeginTrigger(const std::string& connection_id)
{
    TriggerArchive archive;
    if (!ready_)
    {
        archive.open_error = "会话目录不可用";
        return archive;
    }
    archive.sequence = ++trigger_sequence_;
    std::ostringstream name;
    name << TimestampWithMilliseconds() << "_" << std::setfill('0') << std::setw(4) << archive.sequence;

    std::error_code code;
    std::string candidate = JoinPath(session_dir_, name.str());
    int32_t suffix = 1;
    while (std::filesystem::exists(candidate))
    {
        candidate = JoinPath(session_dir_, name.str() + "_" + std::to_string(suffix));
        ++suffix;
    }
    std::filesystem::create_directories(candidate, code);
    if (code)
    {
        archive.open_error = "无法创建本轮归档目录 " + candidate + ": " + code.message();
        LOG_ERROR << "[归档1/3] " << archive.open_error;
        return archive;
    }
    archive.directory = candidate;
    archive.input_dir = JoinPath(candidate, "input");
    archive.output_dir = JoinPath(candidate, "output");
    archive.request_log = JoinPath(candidate, "request.log");
    std::filesystem::create_directories(archive.input_dir, code);
    std::filesystem::create_directories(archive.output_dir, code);
    if (code)
    {
        archive.open_error = "无法创建本轮 input/output 目录: " + code.message();
        LOG_ERROR << "[归档1/3] " << archive.open_error;
        return archive;
    }
    archive.opened = true;

    // 本轮日志同时写 request.log；接收前期按连接标识记录的行补入本轮日志。
    EnableRequestLogMirror(archive.request_log);
    const int64_t replayed = ReplayConnectionLinesToRequestLog(connection_id);
    LOG_INFO << "[归档1/3] 创建本轮目录=" << archive.directory << " 补入接收日志=" << replayed << "行";
    return archive;
}

bool SessionArchive::SaveInputs(const TriggerArchive& archive,
                                const std::string& request_text,
                                const std::vector<uint8_t>& main_bytes,
                                const std::vector<uint8_t>& left_bytes,
                                const std::vector<uint8_t>& right_bytes,
                                const std::vector<uint8_t>& rgb_main_bytes,
                                std::string& error)
{
    if (!archive.opened)
    {
        error = archive.open_error.empty() ? "本轮归档目录未创建" : archive.open_error;
        return false;
    }
    const auto started = std::chrono::steady_clock::now();
    // request.txt 保留原文，不改写；三张深度图与主 RGB 图统一用标准文件名保存。
    if (!WriteBinaryFile(JoinPath(archive.input_dir, "request.txt"), request_text.data(),
                         request_text.size(), error) ||
        !WriteBinaryFile(JoinPath(archive.input_dir, "raw_depth.tiff"), main_bytes.data(),
                         main_bytes.size(), error) ||
        !WriteBinaryFile(JoinPath(archive.input_dir, "raw_depth_left.tiff"), left_bytes.data(),
                         left_bytes.size(), error) ||
        !WriteBinaryFile(JoinPath(archive.input_dir, "raw_depth_right.tiff"), right_bytes.data(),
                         right_bytes.size(), error) ||
        !WriteBinaryFile(JoinPath(archive.input_dir, "raw_color.png"), rgb_main_bytes.data(),
                         rgb_main_bytes.size(), error))
    {
        LOG_ERROR << "[归档2/3] 输入保存失败: " << error;
        return false;
    }
    const uint64_t total = request_text.size() + main_bytes.size() + left_bytes.size() + right_bytes.size() +
                           rgb_main_bytes.size();
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    LOG_INFO << "[归档2/3] 输入保存完成 文字=1 图片=4 字节数=" << total << " 耗时=" << elapsed_ms << "ms";
    return true;
}

bool SessionArchive::ArchiveOutputs(const TriggerArchive& archive,
                                    const std::string& project_output_root,
                                    std::vector<std::string>& copied_files,
                                    std::string& error)
{
    copied_files.clear();
    if (!archive.opened)
    {
        error = archive.open_error.empty() ? "本轮归档目录未创建" : archive.open_error;
        return false;
    }
    if (!std::filesystem::exists(project_output_root))
    {
        error = "项目 output 目录不存在: " + project_output_root;
        return false;
    }
    const auto started = std::chrono::steady_clock::now();
    std::error_code code;
    // 独立文件副本，不使用软链接，不随下一轮文件修改而改变。
    for (auto iterator = std::filesystem::recursive_directory_iterator(
             project_output_root, std::filesystem::directory_options::skip_permission_denied, code);
         iterator != std::filesystem::recursive_directory_iterator(); ++iterator)
    {
        const std::filesystem::path& source = iterator->path();
        const std::string relative = std::filesystem::relative(source, project_output_root, code).string();
        const std::filesystem::path target = std::filesystem::path(archive.output_dir) / relative;
        if (iterator->is_directory())
        {
            std::filesystem::create_directories(target, code);
            continue;
        }
        if (!iterator->is_regular_file())
        {
            continue;
        }
        std::filesystem::create_directories(target.parent_path(), code);
        std::error_code copy_code;
        std::filesystem::copy_file(source, target,
                                   std::filesystem::copy_options::overwrite_existing, copy_code);
        if (copy_code)
        {
            error = "复制 " + relative + " 失败: " + copy_code.message();
            LOG_ERROR << "[归档3/3] " << error;
            return false;
        }
        copied_files.push_back(relative);
    }
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    LOG_INFO << "[归档3/3] 输出保存完成 文件数=" << copied_files.size() << " 耗时=" << elapsed_ms << "ms";
    return true;
}

bool SessionArchive::WriteResult(const TriggerArchive& archive,
                                 const nlohmann::json& result,
                                 std::string& error)
{
    if (!archive.opened)
    {
        error = archive.open_error.empty() ? "本轮归档目录未创建" : archive.open_error;
        return false;
    }
    nlohmann::json document = result;
    document["written_at"] = LocalIso8601();
    return WriteTextFileAtomic(JoinPath(archive.directory, "result.json"), document.dump(2) + "\n", error);
}

void SessionArchive::EndTrigger(const TriggerArchive& archive)
{
    (void)archive;
    DisableRequestLogMirror();
}

} // namespace openmind::trajectory_plan
