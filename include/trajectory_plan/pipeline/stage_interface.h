#ifndef HYBRID_TRAJECTORY_PLAN_STAGE_INTERFACE_H
#define HYBRID_TRAJECTORY_PLAN_STAGE_INTERFACE_H

/**
 * @file stage_interface.h
 * @brief Pipeline stage 抽象接口：Init / Execute / GetName 三段式。
 *
 * 启动时 Init 完成所有不依赖本轮目标和深度图的准备（固定配置、标定、标签缓存、
 * 求解器绑定与预热）；触发后 Execute 只创建本轮上下文并执行已准备好的资源。
 * stage 只通过 StageRegistry 用字符串取用；数据经 PipelineContext 进出。
 */

#include <nlohmann/json.hpp>

#include <string>

#include "trajectory_plan/pipeline/pipeline_context.h"

namespace openmind::trajectory_plan
{

/**
 * @brief 一个 stage 的注册配置与启动资源。
 *
 * 启动资源由 App 注入，stage 在 Init 阶段一次性读取并缓存；
 * 触发后不再重新解析配置或重新打开固定资源文件。
 */
struct StageConfig
{
    std::string id;             ///< 在 flow 链里的名字
    std::string type;           ///< StageRegistry 注册名
    std::string flow;           ///< 所属 flow 名（grasp / trajectory）
    std::string ordinal;        ///< 中文数字编号（一、二、三、四、五）
    nlohmann::json params;      ///< 该 stage 的参数块（可为 null）

    // ---- 启动资源（只读指针，生命周期由 App 持有）----
    const nlohmann::json* config_root = nullptr;  ///< 合并后的启动配置根
    const nlohmann::json* flow_config = nullptr;  ///< 本 flow 的 yaml 配置
    const RobotKinematicsInterface* kinematics = nullptr; ///< 机型运动学
    const DeviceManager* device_manager = nullptr;        ///< 碰撞球与自碰表
    const CameraManager* camera_manager = nullptr;        ///< TIFF 解码
    std::string grasp_output_dir;      ///< output/grasp_generation
    std::string trajectory_output_dir; ///< output/trajectory_planning

    /**
     * @brief 读取参数，缺省或 null 时返回 fallback。
     */
    template <typename T>
    T Param(const std::string& key, const T& fallback) const
    {
        const auto iterator = params.find(key);
        if (iterator == params.end() || iterator->is_null())
        {
            return fallback;
        }
        return iterator->get<T>();
    }

    /**
     * @brief 日志用的 stage 标签，如 "[一 load_request]"。
     */
    std::string LogTag() const
    {
        return "[" + ordinal + " " + id + "]";
    }
};

/**
 * @brief Pipeline stage 抽象接口。
 */
class IStage
{
  public:
    virtual ~IStage() = default;

    /**
     * @brief 启动初始化：准备本 stage 触发后要复用的全部固定资源。
     * @param config 注册配置与启动资源
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true；失败时服务不得宣布就绪
     */
    virtual bool Init(const StageConfig& config, std::string& error) = 0;

    /**
     * @brief 执行本 stage 的职责，读写 PipelineContext。
     * @param ctx 本次触发的数据总线
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true；失败中断整条 flow
     */
    virtual bool Execute(PipelineContext& ctx, std::string& error) = 0;

    /**
     * @brief stage 名（flow 链中的名字，同时是日志 tag）。
     */
    virtual const std::string& GetName() const = 0;

    /**
     * @brief 日志用的中文编号标签，如 "[一 load_request]"。
     */
    virtual const std::string& GetLogTag() const = 0;
};

/**
 * @brief IStage 的公共实现：保存名字与日志标签，子类只实现 Init/Execute 主体。
 */
class StageBase : public IStage
{
  public:
    const std::string& GetName() const override
    {
        return name_;
    }

    const std::string& GetLogTag() const override
    {
        return log_tag_;
    }

  protected:
    /**
     * @brief 子类在 Init 开头调用，绑定名字、编号与启动资源。
     */
    void BindConfig(const StageConfig& config)
    {
        name_ = config.id;
        log_tag_ = config.LogTag();
    }

    std::string name_;    ///< stage 名
    std::string log_tag_; ///< "[编号 名字]"
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_STAGE_INTERFACE_H
