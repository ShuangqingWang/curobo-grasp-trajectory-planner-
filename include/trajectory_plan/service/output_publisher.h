#ifndef HYBRID_TRAJECTORY_PLAN_OUTPUT_PUBLISHER_H
#define HYBRID_TRAJECTORY_PLAN_OUTPUT_PUBLISHER_H

/**
 * @file output_publisher.h
 * @brief output 覆盖与一致性（文档「output 覆盖与一致性」）。
 *
 * `output/` 表示最近一次已接受任务的结果。规则：
 * 1. 新任务开始先原子发布本轮 manifest，`status=processing`、`ready=false`，
 *    使上轮结果失效；这一步失败则不接受任务、不开始 flow。
 * 2. 结果文件以临时文件写完再原子替换的方式覆盖。
 * 3. 清理本轮不再产生的旧受管产物（旧候选、旧轨迹），避免上轮残留混入；
 *    清理仅限明确归本规划器管理的文件，不删除 output 下无关文件。
 * 4. 全部正式产物写入成功并完成旧产物清理后，最后发布 `status=completed`、`ready=true`。
 * 5. 本轮无解或异常统一收尾为 `status=failed`、`ready=false` 并说明原因。
 */

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace openmind::trajectory_plan
{

/**
 * @brief 本规划器管理的 output 产物集合（清理只作用于这些文件）。
 */
class OutputPublisher
{
  public:
    OutputPublisher() = delete;

    /**
     * @brief 任务开始：原子发布 `status=processing` 的 manifest，使上轮结果失效。
     * @param trajectory_output_dir output/trajectory_planning
     * @param request_id 本轮请求标识
     * @param generation_id 本轮代次标识
     * @param workpiece_type 工件类型
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true；失败时调用方必须拒绝任务、不启动 flow
     */
    static bool PublishProcessing(const std::string& trajectory_output_dir,
                                  const std::string& request_id,
                                  const std::string& generation_id,
                                  const std::string& workpiece_type,
                                  std::string& error);

    /**
     * @brief 发布失败 manifest：`status=failed`、`ready=false` 并说明原因。
     *
     * 失败收尾不保留旧成功轨迹冒充当前结果；本轮已产生的有效候选或点云可保留。
     */
    static bool PublishFailed(const std::string& trajectory_output_dir,
                              const std::string& request_id,
                              const std::string& generation_id,
                              const std::string& workpiece_type,
                              const std::string& failed_flow,
                              const std::string& failed_stage,
                              const std::string& reason,
                              std::string& error);

    /**
     * @brief 发布最终 manifest（由输出 stage 在全部正式产物写成功后调用）。
     * @param manifest 完整 manifest 文档（含 status/ready 字段）
     */
    static bool PublishManifest(const std::string& trajectory_output_dir,
                                const nlohmann::json& manifest,
                                std::string& error);

    /**
     * @brief 清理本轮不再产生的旧受管产物。
     *
     * 只处理本规划器管理的文件：两条正式轨迹、点云、点云二进制、candidates 目录
     * 下的 .json。不在 keep_relative_paths 中的受管文件会被删除。
     * @param trajectory_output_dir output/trajectory_planning
     * @param keep_relative_paths 本轮产生、需保留的相对路径
     * @param removed 输出：实际删除的相对路径
     * @param error 失败原因（删除失败不得以"清理完成"掩盖）
     * @return 全部删除成功返回 true
     */
    static bool CleanupManagedArtifacts(const std::string& trajectory_output_dir,
                                        const std::vector<std::string>& keep_relative_paths,
                                        std::vector<std::string>& removed,
                                        std::string& error);

    /**
     * @brief 异常失败收尾：清掉不属于本轮的受管产物。
     *
     * 「不保留旧成功轨迹冒充当前结果」——异常失败时输出 stage 没有跑完，两条正式
     * 轨迹和点云一定是上轮遗留，必须删掉；带代次标识的候选与抓取候选结果文件按
     * generation_id 判断，只删不属于本轮的。本轮已产生的有效候选与点云保留供排查。
     *
     * @param trajectory_output_dir output/trajectory_planning
     * @param grasp_output_dir output/grasp_generation
     * @param generation_id 本轮代次标识
     * @param keep_cloud_binary 本轮是否确实下发了碰撞世界（true 时保留三路深度文件）
     * @param removed 输出：实际删除的路径
     * @param error 失败原因（成功时为空）
     * @return 全部删除成功返回 true
     */
    static bool PurgeForeignGeneration(const std::string& trajectory_output_dir,
                                       const std::string& grasp_output_dir,
                                       const std::string& generation_id,
                                       bool keep_cloud_binary,
                                       std::vector<std::string>& removed,
                                       std::string& error);

    /**
     * @brief manifest 绝对／相对路径（应答与日志用）。
     */
    static std::string ManifestPath(const std::string& trajectory_output_dir);
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_OUTPUT_PUBLISHER_H
