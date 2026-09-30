#ifndef HYBRID_TRAJECTORY_PLAN_SESSION_ARCHIVE_H
#define HYBRID_TRAJECTORY_PLAN_SESSION_ARCHIVE_H

/**
 * @file session_archive.h
 * @brief 会话日志与逐轮归档（文档「编译、会话日志与逐轮归档」）。
 *
 * 目录布局（注意根目录拼写是 hybird_log，与旧实现的 hybrid_log 不同）：
 *
 * ```text
 * /var/hybird_log/grasp_log/
 * ├── 20260907_103000_123/                 # 一次 service 启动，一个会话目录
 * │   ├── service.log                      # 从启动、预热到退出的完整日志
 * │   ├── session.json                     # 会话标识、时间、时区、配置、版本、退出状态
 * │   └── 20260907_103215_456_0001/        # 本会话第一次触发
 * │       ├── request.log
 * │       ├── result.json
 * │       ├── input/{request.txt,raw_depth*.tiff}
 * │       └── output/{grasp_generation,trajectory_planning}
 * ```
 *
 * 归档为独立文件副本，不使用软链接；项目 output 继续覆盖，归档目录逐轮保留。
 */

#include <nlohmann/json.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace openmind::trajectory_plan
{

/**
 * @brief 一次触发的归档上下文。
 */
struct TriggerArchive
{
    bool opened = false;        ///< 本轮目录是否创建成功
    std::string directory;      ///< 本轮触发目录绝对路径
    std::string input_dir;      ///< 本轮 input 目录
    std::string output_dir;     ///< 本轮 output 目录
    std::string request_log;    ///< 本轮 request.log 路径
    int64_t sequence = 0;       ///< 会话内递增序号
    std::string open_error;     ///< 创建失败原因
};

/**
 * @brief 会话日志与逐轮归档管理器。
 *
 * 服务启动时 OpenSession 一次；此后每轮 BeginTrigger → SaveInputs →
 * （执行 flow）→ ArchiveOutputs → WriteResult → EndTrigger。
 */
class SessionArchive
{
  public:
    SessionArchive() = default;
    SessionArchive(const SessionArchive&) = delete;
    SessionArchive& operator=(const SessionArchive&) = delete;

    /**
     * @brief 创建本次启动的会话目录并打开 service.log 镜像。
     *
     * 重名时增加唯一后缀，绝不覆盖已有目录。会话日志根目录本身无法创建时返回
     * false，调用方不得宣布服务就绪。
     * @param log_root 日志根目录（app.json 的 log_root）
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true
     */
    bool OpenSession(const std::string& log_root, std::string& error);

    /**
     * @brief 写 session.json（启动成功、初始化失败、退出时均可重复调用覆盖）。
     * @param config_snapshot 启动配置快照
     * @param status 会话状态：initializing / ready / init_failed / exited
     * @param detail 补充说明（失败模块、退出原因等）
     */
    bool WriteSessionJson(const nlohmann::json& config_snapshot,
                          const std::string& status,
                          const std::string& detail);

    /** @brief 会话目录。 */
    const std::string& SessionDir() const;

    /** @brief 会话标识（目录名）。 */
    const std::string& SessionId() const;

    /** @brief 会话目录是否可用。 */
    bool SessionReady() const;

    /**
     * @brief 创建本轮触发目录并打开 request.log。
     *
     * 目录名为 `日期_时间_毫秒_会话内序号`。创建失败时返回的 TriggerArchive
     * opened=false，调用方仍须继续执行规划，只把归档状态报成 failed。
     * @param connection_id 连接标识，用于把接收前期日志补入 request.log
     */
    TriggerArchive BeginTrigger(const std::string& connection_id);

    /**
     * @brief 保存 service 实际收到的原始文字、三张深度图与主 RGB 图。
     *
     * 保存的是收到的字节，不是任务结束后再复制的客户端文件。
     * @param archive 本轮归档上下文
     * @param request_text request.txt 原文（不改写）
     * @param main_bytes 主图原始字节
     * @param left_bytes 左图原始字节
     * @param right_bytes 右图原始字节
     * @param rgb_main_bytes 主 RGB 图原始字节
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true
     */
    static bool SaveInputs(const TriggerArchive& archive,
                           const std::string& request_text,
                           const std::vector<uint8_t>& main_bytes,
                           const std::vector<uint8_t>& left_bytes,
                           const std::vector<uint8_t>& right_bytes,
                           const std::vector<uint8_t>& rgb_main_bytes,
                           std::string& error);

    /**
     * @brief 复制本轮项目 output 到归档 output，保留目录层级。
     * @param archive 本轮归档上下文
     * @param project_output_root 项目 output 根目录
     * @param copied_files 输出：实际复制的文件相对路径
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true
     */
    static bool ArchiveOutputs(const TriggerArchive& archive,
                               const std::string& project_output_root,
                               std::vector<std::string>& copied_files,
                               std::string& error);

    /**
     * @brief 写本轮 result.json。
     */
    static bool WriteResult(const TriggerArchive& archive,
                            const nlohmann::json& result,
                            std::string& error);

    /** @brief 关闭 request.log 镜像，结束本轮归档。 */
    static void EndTrigger(const TriggerArchive& archive);

  private:
    std::string session_dir_;  ///< 会话目录
    std::string session_id_;   ///< 会话标识
    std::string started_at_;   ///< 会话开始时间（本地时区 ISO8601）
    bool ready_ = false;       ///< 会话目录是否建立成功
    int64_t trigger_sequence_ = 0; ///< 会话内触发序号
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_SESSION_ARCHIVE_H
