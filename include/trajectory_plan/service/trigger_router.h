#ifndef HYBRID_TRAJECTORY_PLAN_TRIGGER_ROUTER_H
#define HYBRID_TRAJECTORY_PLAN_TRIGGER_ROUTER_H

/**
 * @file trigger_router.h
 * @brief 触发路由：TCP 帧 → 校验 → accepted → grasp → trajectory → 归档 → 最终应答。
 *
 * 生命周期（文档「接收、执行与应答」）：
 *
 * ```text
 * 检查service就绪且空闲 → 完整接收报头、文字和三图 → 校验长度与请求字段
 *   → 创建本轮request_id、generation_id及上下文
 *   → 将本轮manifest标记为处理中、ready=false → 返回accepted
 *   → 创建本轮归档目录，保存收到的原始文字和三张图片
 *   → 执行grasp，覆盖发布本轮grasp_result.json
 *   → 执行trajectory，覆盖发布本轮轨迹结果
 *   → 归档本轮output、完成本轮日志和result.json
 *   → 返回completed／failed
 * ```
 *
 * 单任务、不排队：任务运行或归档期间新连接返回 busy，不创建新代次、不改动 output。
 */

#include "nlohmann/json.hpp"

#include <atomic>
#include <mutex>
#include <string>

#include "trajectory_plan/camera/camera_manager.h"
#include "trajectory_plan/device/device_manager.h"
#include "trajectory_plan/pipeline/pipeline.h"
#include "trajectory_plan/service/session_archive.h"
#include "trajectory_plan/service/tcp_server.h"

namespace openmind::trajectory_plan
{

/**
 * @brief 触发路由。
 */
class TriggerRouter
{
  public:
    TriggerRouter() = default;
    TriggerRouter(const TriggerRouter&) = delete;
    TriggerRouter& operator=(const TriggerRouter&) = delete;

    /**
     * @brief 绑定依赖。
     * @param pipeline 已初始化两条 flow 的 pipeline
     * @param archive 会话与逐轮归档管理器
     * @param output_root 项目 output 根目录
     * @param grasp_output_dir output/grasp_generation
     * @param trajectory_output_dir output/trajectory_planning
     * @param collision_model_id 固定模型标识（写入每轮 manifest）
     * @param collision_model_sha256 固定模型快照哈希
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true
     */
    bool Init(Pipeline* pipeline,
              SessionArchive* archive,
              const std::string& output_root,
              const std::string& grasp_output_dir,
              const std::string& trajectory_output_dir,
              const std::string& collision_model_id,
              const std::string& collision_model_sha256,
              std::string& error);

    /**
     * @brief TcpServer 的帧处理器：先 accepted／rejected／busy，完成后 completed／failed。
     * @param frame 收到的完整帧
     * @param sender 应答发送器（可多次发送，每次一行 JSON）
     */
    void HandleFrame(const TriggerFrame& frame, const ResponseSender& sender);

    /**
     * @brief 服务是否已就绪（全部初始化与预热完成才置 true）。
     */
    void SetServiceReady(bool ready);

    /**
     * @brief 直接执行一轮（--once 本地回放入口，不经 TCP）。
     * @param frame 已填好 request_text 与三图的帧
     * @param reply 输出：最终应答行
     * @return 规划成功返回 true
     */
    bool RunOnce(const TriggerFrame& frame, std::string& reply);

  private:
    /**
     * @brief 执行一轮完整任务；调用方保证已取得单任务占用。
     */
    void RunTask(const TriggerFrame& frame, const ResponseSender& sender, std::string* reply_out);

    Pipeline* pipeline_ = nullptr;        ///< pipeline（两条 flow）
    SessionArchive* archive_ = nullptr;   ///< 会话与逐轮归档
    std::string output_root_;             ///< 项目 output 根目录
    std::string grasp_output_dir_;        ///< output/grasp_generation
    std::string trajectory_output_dir_;   ///< output/trajectory_planning
    std::string collision_model_id_;      ///< 固定模型标识
    std::string collision_model_sha256_;  ///< 固定模型快照哈希
    std::mutex task_mutex_;               ///< 单任务占用（try_lock 失败即 busy）
    std::atomic<bool> service_ready_ {false}; ///< 服务是否可接任务
    std::atomic<uint64_t> trigger_sequence_ {0}; ///< 触发序号（request_id 用）
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_TRIGGER_ROUTER_H
