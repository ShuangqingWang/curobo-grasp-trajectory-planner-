/**
 * @file log.cpp
 * @brief 日志上下文与 service.log 镜像的实现（log.h 的配套实现文件）。
 */

#include "trajectory_plan/tools/log.h"

#include <deque>
#include <fstream>
#include <mutex>
#include <sstream>

namespace openmind::trajectory_plan
{
namespace
{

/// 当前线程的日志前缀；一次触发开始时设置，结束时清除。
thread_local std::string g_thread_log_prefix;

/// service.log 镜像流；未启用时为空。
std::ofstream g_service_log_mirror;

/// 镜像流互斥锁：日志来自多线程，写同一文件必须串行。
std::mutex g_service_log_mutex;

/// 本轮 request.log 镜像流；未启用时为空。
std::ofstream g_request_log_mirror;

/// 最近日志行的环形缓存，用于把接受任务前的连接日志补进 request.log。
constexpr size_t kRecentLineCapacity = 256;
std::deque<std::string> g_recent_lines;

} // namespace

void SetLogContext(const std::string& flow_prefix)
{
    g_thread_log_prefix = flow_prefix;
}

void ClearLogContext()
{
    g_thread_log_prefix.clear();
}

const std::string& LogPrefix()
{
    return g_thread_log_prefix;
}

void EnableServiceLogMirror(const std::string& path)
{
    std::lock_guard<std::mutex> lock(g_service_log_mutex);
    if (g_service_log_mirror.is_open())
    {
        g_service_log_mirror.close();
    }
    g_service_log_mirror.open(path, std::ios::out | std::ios::app);
}

void DisableServiceLogMirror()
{
    std::lock_guard<std::mutex> lock(g_service_log_mutex);
    if (g_service_log_mirror.is_open())
    {
        g_service_log_mirror.close();
    }
}

void EnableRequestLogMirror(const std::string& path)
{
    std::lock_guard<std::mutex> lock(g_service_log_mutex);
    if (g_request_log_mirror.is_open())
    {
        g_request_log_mirror.close();
    }
    g_request_log_mirror.open(path, std::ios::out | std::ios::app);
}

void DisableRequestLogMirror()
{
    std::lock_guard<std::mutex> lock(g_service_log_mutex);
    if (g_request_log_mirror.is_open())
    {
        g_request_log_mirror.close();
    }
}

int64_t ReplayConnectionLinesToRequestLog(const std::string& connection_id)
{
    std::lock_guard<std::mutex> lock(g_service_log_mutex);
    if (!g_request_log_mirror.is_open() || connection_id.empty())
    {
        return 0;
    }
    int64_t replayed = 0;
    for (const std::string& line : g_recent_lines)
    {
        if (line.find(connection_id) != std::string::npos)
        {
            g_request_log_mirror << line << std::endl;
            ++replayed;
        }
    }
    g_request_log_mirror.flush();
    return replayed;
}

void MirrorToServiceLog(const std::string& line)
{
    std::lock_guard<std::mutex> lock(g_service_log_mutex);
    g_recent_lines.push_back(line);
    while (g_recent_lines.size() > kRecentLineCapacity)
    {
        g_recent_lines.pop_front();
    }
    if (g_service_log_mirror.is_open())
    {
        g_service_log_mirror << line << std::endl;
        g_service_log_mirror.flush();
    }
    // 本轮日志同时进 request.log，使每轮归档目录自带完整的接收/flow/发布记录。
    if (g_request_log_mirror.is_open())
    {
        g_request_log_mirror << line << std::endl;
        g_request_log_mirror.flush();
    }
}

LogLineProxy::LogLineProxy(const google::LogSeverity severity, const char* file, int line, bool mirror)
    : severity_(severity), file_(file), line_(line), mirror_(mirror)
{
}

LogLineProxy::~LogLineProxy()
{
    if (mirror_)
    {
        MirrorToServiceLog(buffer_.str());
    }
    google::LogMessage log_message(file_, line_, severity_);
    log_message.stream() << buffer_.str();
}

} // namespace openmind::trajectory_plan
