/**
 * @file logger_init.cpp
 * @brief glog 初始化实现：产出 log.INFO / log.WARNING / log.ERROR 于会话目录。
 */

#include "trajectory_plan/tools/logger_init.hpp"

#include "glog/logging.h"

namespace openmind::trajectory_plan
{

void InitGlogInSession(const std::string& session_dir, const std::string& process_name)
{
    static bool initialized = false;
    if (initialized)
    {
        return;
    }
    FLAGS_log_dir = session_dir;
    FLAGS_logtostderr = false;
    FLAGS_alsologtostderr = true;
    FLAGS_stderrthreshold = google::GLOG_INFO;
    google::InitGoogleLogging(process_name.c_str());
    initialized = true;
}

} // namespace openmind::trajectory_plan
