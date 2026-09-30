#ifndef HYBRID_TRAJECTORY_PLAN_PIPELINE_H
#define HYBRID_TRAJECTORY_PLAN_PIPELINE_H

/**
 * @file pipeline.h
 * @brief 组合器：按 flow 名构造 stage 链、启动初始化、顺序执行并统一打日志。
 *
 * pipeline 是执行核心：启动时根据配置确定启用的 flow，按顺序创建并初始化其 stage；
 * 启动完成后 flow、stage 实例、模型和求解器保持常驻，触发只创建本轮上下文。
 */

#include <nlohmann/json.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "trajectory_plan/pipeline/stage_interface.h"

namespace openmind::trajectory_plan
{

/**
 * @brief 启动时注入给全部 stage 的共享资源。
 */
struct PipelineStartupResources
{
    const nlohmann::json* config_root = nullptr;          ///< 合并配置根
    std::map<std::string, const nlohmann::json*> flow_configs; ///< flow 名 → 该 flow 的 yaml
    const RobotKinematicsInterface* kinematics = nullptr; ///< 机型运动学
    const DeviceManager* device_manager = nullptr;        ///< 碰撞模型
    const CameraManager* camera_manager = nullptr;        ///< TIFF 解码
    std::string grasp_output_dir;      ///< output/grasp_generation
    std::string trajectory_output_dir; ///< output/trajectory_planning
};

/**
 * @brief 一条 flow 的执行结果（供 pipeline 统一收尾与应答）。
 */
struct FlowResult
{
    bool success = true;        ///< 全部 stage 成功
    std::string failed_stage;   ///< 失败所在 stage（成功时为空）
    std::string reason;         ///< 失败原因
    double elapsed_ms = 0.0;    ///< flow 总耗时
};

/**
 * @brief Pipeline：读 pipeline.json 的 flow 定义，构造、初始化并执行 stage 链。
 */
class Pipeline
{
  public:
    Pipeline() = default;
    Pipeline(const Pipeline&) = delete;
    Pipeline& operator=(const Pipeline&) = delete;

    /**
     * @brief 启动初始化：构造要跑的 flow 的 stage 链并逐个 Init。
     * @param pipeline_config 合并后的 pipeline 节（含各 flow 的 stage 名数组）
     * @param resources 注入给每个 stage 的启动资源
     * @param wanted_flows 要启用的 flow 名（按顺序执行）
     * @param error 失败原因（成功时为空）
     * @return 全部 stage 初始化成功返回 true；失败时不得宣布服务就绪
     */
    bool Init(const nlohmann::json& pipeline_config,
              const PipelineStartupResources& resources,
              const std::vector<std::string>& wanted_flows,
              std::string& error);

    /**
     * @brief 顺序执行一条 flow 的全部 stage。
     * @param flow_name flow 名（"grasp" / "trajectory"）
     * @param ctx 本次触发的数据总线
     * @param result 输出：执行结果（失败 stage 与原因）
     * @return 全部 stage 成功返回 true
     * @note 某个 stage 失败后，后续 stage 记录 skipped 及原因，不记为通过。
     */
    bool Run(const std::string& flow_name, PipelineContext& ctx, FlowResult& result);

    /**
     * @brief 配置文件里声明的全部 flow 名（错误提示用）。
     */
    std::vector<std::string> FlowNames() const;

    /**
     * @brief 已启用的 flow 名（按执行顺序）。
     */
    const std::vector<std::string>& EnabledFlows() const;

  private:
    std::map<std::string, std::vector<std::unique_ptr<IStage>>> chains_; ///< flow 名 → stage 链
    std::vector<std::string> flow_names_;   ///< 配置声明顺序的 flow 名
    std::vector<std::string> enabled_flows_; ///< 本进程启用的 flow 名
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_PIPELINE_H
