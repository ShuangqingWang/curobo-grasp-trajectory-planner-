#ifndef HYBRID_TRAJECTORY_PLAN_LOGGER_INIT_HPP
#define HYBRID_TRAJECTORY_PLAN_LOGGER_INIT_HPP

/**
 * @file logger_init.hpp
 * @brief glog 初始化：把 glog 的落盘目录指向已经建好的会话目录。
 *
 * 会话目录、service.log 与逐轮 request.log 由 SessionArchive 管理；
 * 本文件只负责 glog 自身的初始化，必须在会话目录创建之后调用。
 */

#include <string>

namespace openmind::trajectory_plan
{

/**
 * @brief 初始化 glog，日志写入给定会话目录。
 * @param session_dir 已创建的会话目录
 * @param process_name 进程名（glog 日志前缀）
 */
void InitGlogInSession(const std::string& session_dir, const std::string& process_name);

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_LOGGER_INIT_HPP
