#ifndef HYBRID_TRAJECTORY_PLAN_LOG_H
#define HYBRID_TRAJECTORY_PLAN_LOG_H

/**
 * @file log.h
 * @brief 统一日志宏五档（LOG_INFO / LOG_WARN / LOG_ERROR / LOG_DEBUG / LOG_TRACE），
 *        底层 glog，并镜像到 service.log。禁止业务代码直接使用 std::cout / std::cerr。
 *
 * 业务 log 前缀固定为 "[grasp]" / "[trajectory]"，由 flow 运行器按 flow 设置
 * （SetLogContext），与 output JSON 使用同一套字段名。
 */

#include <sstream>
#include <string>

#include "glog/logging.h"

namespace openmind::trajectory_plan
{

/**
 * @brief 设置当前线程的日志前缀（一次触发开始时调用一次）。
 * @param flow_prefix 业务前缀，如 "[grasp]" / "[trajectory]"
 */
void SetLogContext(const std::string& flow_prefix);

/** @brief 清除当前线程的日志前缀（一次触发结束时调用）。 */
void ClearLogContext();

/** @brief 当前线程的日志前缀；未设置时为空串。 */
const std::string& LogPrefix();

/**
 * @brief 打开 service.log 镜像（logger_init 调用；未打开时镜像关闭）。
 * @param path 控制台镜像文件路径（如会话目录下 service.log）
 */
void EnableServiceLogMirror(const std::string& path);

/** @brief 关闭 service.log 镜像。 */
void DisableServiceLogMirror();

/**
 * @brief 打开本轮 request.log 镜像（归档模块在创建本轮目录后调用）。
 *
 * 本轮日志同时写到 service.log 与 request.log；request.log 关闭时只写 service.log。
 * @param path 本轮触发目录下的 request.log 路径
 */
void EnableRequestLogMirror(const std::string& path);

/** @brief 关闭本轮 request.log 镜像。 */
void DisableRequestLogMirror();

/**
 * @brief 把最近若干条含指定连接标识的日志补写进当前 request.log。
 *
 * 接收前期尚未建本轮目录，那些行先按连接标识写 service.log；
 * 接受任务后用本函数把相关接收记录补入 request.log。
 * @param connection_id 连接标识，如 conn_1
 * @return 实际补入的行数
 */
int64_t ReplayConnectionLinesToRequestLog(const std::string& connection_id);

/**
 * @brief 把一行格式化日志镜像到 service.log（log.h 内部使用）。
 */
void MirrorToServiceLog(const std::string& line);

/**
 * @brief 内部实现：glog 一行 + service.log 镜像，析构时落盘。
 */
class LogLineProxy
{
  public:
    /**
     * @param severity glog 级别（google::INFO 等）
     * @param file 源码文件
     * @param line 源码行号
     * @param mirror 是否镜像到 service.log（info/warn/error 为 true）
     */
    LogLineProxy(const google::LogSeverity severity, const char* file, int line, bool mirror);
    LogLineProxy(const LogLineProxy&) = delete;
    LogLineProxy& operator=(const LogLineProxy&) = delete;
    ~LogLineProxy();

    /**
     * @brief 流式写入入口。
     */
    std::ostream& Stream()
    {
        return buffer_;
    }

  private:
    google::LogSeverity severity_;
    const char* file_;
    int line_;
    bool mirror_;
    std::ostringstream buffer_;
};

} // namespace openmind::trajectory_plan

/// 五档：err / warn / info 对应 glog ERROR / WARNING / INFO；debug / trace 落到 VLOG。
#define HYBRID_TP_LOG_LEVEL_error LOG(ERROR)
#define HYBRID_TP_LOG_LEVEL_warn LOG(WARNING)
#define HYBRID_TP_LOG_LEVEL_info LOG(INFO)
#define HYBRID_TP_LOG_LEVEL_debug VLOG(1)
#define HYBRID_TP_LOG_LEVEL_trace VLOG(2)

/**
 * @brief 统一日志宏：LOG_ERROR / LOG_WARN / LOG_INFO / LOG_DEBUG / LOG_TRACE。
 * @note 用法：LOG_INFO << "[grasp] candidate_count=10";
 */
#define LOG_ERROR ::openmind::trajectory_plan::LogLineProxy(google::GLOG_ERROR, __FILE__, __LINE__, true).Stream() \
    << ::openmind::trajectory_plan::LogPrefix()
#define LOG_WARN ::openmind::trajectory_plan::LogLineProxy(google::GLOG_WARNING, __FILE__, __LINE__, true).Stream() \
    << ::openmind::trajectory_plan::LogPrefix()
#define LOG_INFO ::openmind::trajectory_plan::LogLineProxy(google::GLOG_INFO, __FILE__, __LINE__, true).Stream() \
    << ::openmind::trajectory_plan::LogPrefix()
#define LOG_DEBUG VLOG(1) << ::openmind::trajectory_plan::LogPrefix()
#define LOG_TRACE VLOG(2) << ::openmind::trajectory_plan::LogPrefix()

#endif // HYBRID_TRAJECTORY_PLAN_LOG_H
