#ifndef HYBRID_TRAJECTORY_PLAN_CONFIG_SERVICE_H
#define HYBRID_TRAJECTORY_PLAN_CONFIG_SERVICE_H

/**
 * @file config_service.h
 * @brief 配置服务：$include 递归展开 + deepMerge（空值跳过），全局一遍。
 *
 * 四键并列：app / camera / device / pipeline 各自 $include 自己的片段；
 * 合并与解析分开：合并在本服务一次做完，取值由各模块从合并结果读取。
 */

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace openmind::trajectory_plan
{

/**
 * @brief 配置服务：唯一配置入口。
 */
class ConfigService
{
  public:
    ConfigService() = default;
    ConfigService(const ConfigService&) = delete;
    ConfigService& operator=(const ConfigService&) = delete;

    /**
     * @brief 加载启动配置（含 $include 递归展开与 deepMerge）。
     * @param config_path 启动配置路径（如 config/grasp_planner.json）
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true
     */
    bool Load(const std::string& config_path, std::string& error);

    /**
     * @brief 合并后的配置根（只读）。
     */
    const nlohmann::json& Root() const;

    /**
     * @brief 按 JSON 指针取子节点；不存在返回空对象引用（is_null）。
     */
    const nlohmann::json& Get(const std::string& pointer) const;

    /**
     * @brief 启动配置文件的目录（相对 include 与 flow yaml 解析用）。
     */
    const std::string& ConfigDir() const;

    /**
     * @brief 展开并合并一份 JSON 文件（含其 $include 链）。
     * @param path 文件路径
     * @param output 输出：合并结果
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true
     * @note 静态工具，供 serve 脚本以外的代码复用；ConfigService 内部也走它。
     */
    static bool LoadWithIncludes(const std::string& path, nlohmann::json& output, std::string& error);

    /**
     * @brief deepMerge：把 override 合并进 base；空值（""/[]/{}）跳过。
     * @param base 输入输出：基线
     * @param override 覆盖层
     * @param path 当前路径（错误信息用）
     */
    static void DeepMerge(nlohmann::json& base, const nlohmann::json& override, const std::string& path = "/");

  private:
    nlohmann::json root_;        ///< 合并后的配置根
    std::string config_dir_;     ///< 启动配置所在目录
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_CONFIG_SERVICE_H
