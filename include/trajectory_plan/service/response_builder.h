#ifndef HYBRID_TRAJECTORY_PLAN_RESPONSE_BUILDER_H
#define HYBRID_TRAJECTORY_PLAN_RESPONSE_BUILDER_H

/**
 * @file response_builder.h
 * @brief 外部 TCP 应答排版（唯一真源）：UTF-8、每行一个 JSON。
 *
 * 生命周期约定（文档「接收、执行与应答」）：
 * - accepted：任务获准执行，**不表示规划成功**；
 * - rejected：输入拒绝，未创建代次、未改动 output；
 * - busy：单任务占用中，不排队、不创建新代次；
 * - completed／failed：**两条轨迹 + manifest 发布完成即发送**，客户端到此即可驱动机械臂；
 * - archived：点云、归档、result.json 全部结束后发送，`archive_status`／`archive_path` 在此行。
 *
 * @note 改动六（轨迹优先应答）把应答由两行改为三行。与主文档的差异：
 *       主文档写「最终结果在正式产物发布**和归档尝试结束后**发送」，本版把归档结果
 *       移到第三行 `archived`，`completed` 在轨迹发布后即发，省下关键路径 520ms。
 *       客户端在第二行后断开时第三行发不出，**只记日志，不改变已发布的轨迹**；
 *       客户端也不能仅凭"没收到 archived"判断归档失败。
 *       单任务占用**持续到 archived**，后台收尾期间新连接仍返回 busy。
 */

#include <string>

namespace openmind::trajectory_plan
{

/**
 * @brief 本轮归档结果（随最终应答一并报告）。
 */
struct ArchiveStatus
{
    std::string status = "skipped"; ///< success / failed / skipped
    std::string path;               ///< 本轮归档目录（尽力给出）
    std::string error;              ///< 归档失败原因
};

/**
 * @brief 应答排版。
 */
class ResponseBuilder
{
  public:
    ResponseBuilder() = delete;

    /**
     * @brief 接受应答：仅表示任务获准执行。
     */
    static std::string Accepted(const std::string& request_id, const std::string& generation_id);

    /**
     * @brief 拒绝应答：输入非法，未开始任何 flow。
     */
    static std::string Rejected(const std::string& request_id, const std::string& reason);

    /**
     * @brief 忙应答：已有任务在执行或归档，不排队。
     */
    static std::string Busy(const std::string& reason);

    /**
     * @brief 完成应答：两条轨迹与 manifest 已发布，客户端到此即可执行。
     * @param ready manifest 的 ready 状态
     * @param manifest_path service 侧 manifest 路径
     * @note 归档结果不在本行，见 Archived()。
     */
    static std::string Completed(const std::string& request_id,
                                 const std::string& generation_id,
                                 bool ready,
                                 const std::string& manifest_path);

    /**
     * @brief 失败应答：携带本轮关联、失败位置和原因，并报告归档状态。
     * @param failed_flow 失败所在 flow（可为空，表示 pipeline 级失败）
     * @param failed_stage 失败所在 stage（可为空）
     */
    static std::string Failed(const std::string& request_id,
                              const std::string& generation_id,
                              const std::string& failed_flow,
                              const std::string& failed_stage,
                              const std::string& reason);

    /**
     * @brief 收尾应答（第三行）：点云、归档、result.json 全部结束。
     * @param point_cloud_ready 后台点云是否写出成功
     * @param archive 归档结果
     */
    static std::string Archived(const std::string& request_id,
                                const std::string& generation_id,
                                bool point_cloud_ready,
                                const ArchiveStatus& archive);
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_RESPONSE_BUILDER_H
