#ifndef HYBRID_TRAJECTORY_PLAN_STAGE_REGISTRY_H
#define HYBRID_TRAJECTORY_PLAN_STAGE_REGISTRY_H

/**
 * @file stage_registry.h
 * @brief Stage 字符串注册表：REGISTER_STAGE 自注册 + 按 type 构造。
 *
 * stage 实现写在各自 .cpp 的匿名命名空间中，靠静态注册对象自注册；
 * 因此 stage 目标必须是 OBJECT library 并用 $<TARGET_OBJECTS:...> 注入，
 * 否则链接器会丢弃整个 .o，运行时报"未知 stage"。
 */

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "trajectory_plan/pipeline/stage_interface.h"

namespace openmind::trajectory_plan
{

/**
 * @brief Stage 工厂注册表（Meyers 单例，线程安全，禁拷贝）。
 */
class StageRegistry
{
  public:
    StageRegistry(const StageRegistry&) = delete;
    StageRegistry& operator=(const StageRegistry&) = delete;

    /**
     * @brief 全局唯一实例。
     */
    static StageRegistry& Instance();

    /**
     * @brief 注册一个 stage 工厂。
     * @param type 注册名（pipeline.json 里引用的名字）
     * @param factory 无参工厂，每次调用返回新实例
     * @return 重名注册返回 false
     */
    bool Register(const std::string& type, std::function<std::unique_ptr<IStage>()> factory);

    /**
     * @brief 按注册名构造 stage；未知名字返回 nullptr。
     */
    std::unique_ptr<IStage> Create(const std::string& type) const;

    /**
     * @brief 已注册的名字列表（错误提示用）。
     */
    std::vector<std::string> RegisteredNames() const;

  private:
    StageRegistry() = default;

    std::map<std::string, std::function<std::unique_ptr<IStage>()>> factories_; ///< 工厂表
};

/**
 * @brief 注册辅助：一个注册器持有一个自注册动作。
 */
struct StageRegistrar
{
    StageRegistrar(const std::string& type, std::function<std::unique_ptr<IStage>()> factory)
    {
        StageRegistry::Instance().Register(type, std::move(factory));
    }
};

} // namespace openmind::trajectory_plan

/**
 * @brief 把 stage 类注册进全局注册表（写在 stage 的 .cpp 文件作用域）。
 * @param type_name 注册名字符串（不带引号写 "xxx"）
 * @param class_name stage 类名
 */
#define REGISTER_STAGE(type_name, class_name) \
    static ::openmind::trajectory_plan::StageRegistrar g_stage_registrar_##class_name( \
        type_name, []() { return std::make_unique<class_name>(); })

#endif // HYBRID_TRAJECTORY_PLAN_STAGE_REGISTRY_H
