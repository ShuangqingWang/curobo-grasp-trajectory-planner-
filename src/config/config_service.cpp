/**
 * @file config_service.cpp
 * @brief 配置服务实现：$include 递归展开 + deepMerge（空值跳过），全局一遍。
 */

#include "trajectory_plan/config/config_service.h"

#include <fstream>
#include <sstream>

#include "trajectory_plan/tools/json_util.h"
#include "trajectory_plan/tools/log.h"

namespace openmind::trajectory_plan
{
namespace
{

/**
 * @brief 是否为空值（"" / [] / {}）：空值跳过，不覆盖基线。
 */
bool IsEmptyValue(const nlohmann::json& value)
{
    return (value.is_string() && value.get<std::string>().empty()) ||
           (value.is_array() && value.empty()) ||
           (value.is_object() && value.empty());
}

/**
 * @brief 取文件所在目录（用于相对 $include 解析）。
 */
std::string DirectoryOf(const std::string& path)
{
    const size_t separator = path.find_last_of('/');
    if (separator == std::string::npos)
    {
        return ".";
    }
    return path.substr(0, separator);
}

/**
 * @brief 解析 $include：路径相对启动配置目录；允许指向仓库外的标定 JSON。
 */
bool ResolveIncludePath(const std::string& include_value,
                        const std::string& base_dir,
                        std::string& resolved)
{
    if (include_value.empty())
    {
        return false;
    }
    resolved = (include_value.front() == '/') ? include_value : base_dir + "/" + include_value;
    return true;
}

} // namespace

bool ExpandNestedIncludes(nlohmann::json& node, const std::string& base_dir, std::string& error);

void ConfigService::DeepMerge(nlohmann::json& base, const nlohmann::json& override, const std::string& path)
{
    if (!override.is_object())
    {
        base = override;
        return;
    }
    for (auto iterator = override.begin(); iterator != override.end(); ++iterator)
    {
        const std::string key = iterator.key();
        if (key == "$include")
        {
            continue;
        }
        const nlohmann::json& value = iterator.value();
        if (IsEmptyValue(value))
        {
            continue;
        }
        if (base.contains(key) && base[key].is_object() && value.is_object())
        {
            // 同名对象递归合并，使 $include 打底 + 同级覆盖的写法生效。
            DeepMerge(base[key], value, path + key + "/");
        }
        else
        {
            base[key] = value;
        }
    }
}

bool ConfigService::LoadWithIncludes(const std::string& path, nlohmann::json& output, std::string& error)
{
    std::string content;
    if (!ReadTextFile(path, content, error))
    {
        return false;
    }
    nlohmann::json document;
    if (!ParseJson(content, document, error))
    {
        error = path + ": " + error;
        return false;
    }

    const std::string base_dir = DirectoryOf(path);
    nlohmann::json merged;
    // ① 先放基线：$include 的内容（对象）或按数组顺序累加。
    if (document.is_object() && document.contains("$include"))
    {
        const nlohmann::json& include = document["$include"];
        if (include.is_string())
        {
            std::string included_path;
            if (!ResolveIncludePath(include.get<std::string>(), base_dir, included_path))
            {
                error = path + ": $include 必须是非空字符串";
                return false;
            }
            if (!LoadWithIncludes(included_path, merged, error))
            {
                error = path + ": " + error;
                return false;
            }
        }
        else if (include.is_array())
        {
            // 多个 $include 依次合并，后包含覆盖前包含。
            merged = nlohmann::json::object();
            for (const auto& item : include)
            {
                if (!item.is_string())
                {
                    error = path + ": $include 数组元素必须是字符串";
                    return false;
                }
                std::string included_path;
                if (!ResolveIncludePath(item.get<std::string>(), base_dir, included_path))
                {
                    error = path + ": $include 必须是非空字符串";
                    return false;
                }
                nlohmann::json fragment;
                if (!LoadWithIncludes(included_path, fragment, error))
                {
                    error = path + ": " + error;
                    return false;
                }
                DeepMerge(merged, fragment);
            }
        }
        else
        {
            error = path + ": $include 必须是字符串或字符串数组";
            return false;
        }
    }
    else
    {
        merged = nlohmann::json::object();
    }

    // ② 再用覆盖层（同级的其余键）盖上去；文档本身若是数组/标量则整体覆盖。
    if (document.is_object())
    {
        DeepMerge(merged, document);
    }
    else
    {
        merged = document;
    }
    if (!ExpandNestedIncludes(merged, DirectoryOf(path), error))
    {
        return false;
    }
    output = std::move(merged);
    return true;
}

bool ExpandNestedIncludes(nlohmann::json& node, const std::string& base_dir, std::string& error)
{
    if (!node.is_object())
    {
        return true;
    }
    if (node.contains("$include"))
    {
        nlohmann::json included;
        if (!ConfigService::LoadWithIncludes(
                (node["$include"].is_string()
                     ? ((node["$include"].get<std::string>().front() == '/')
                            ? node["$include"].get<std::string>()
                            : base_dir + "/" + node["$include"].get<std::string>())
                     : std::string()),
                included, error))
        {
            if (!node["$include"].is_string())
            {
                error = "嵌套 $include 必须是字符串";
            }
            return false;
        }
        ConfigService::DeepMerge(included, node);
        node = std::move(included);
        node.erase("$include");
    }
    for (auto iterator = node.begin(); iterator != node.end(); ++iterator)
    {
        if (iterator.key() == "$include")
        {
            continue;
        }
        if (!ExpandNestedIncludes(iterator.value(), base_dir, error))
        {
            return false;
        }
    }
    return true;
}

bool ConfigService::Load(const std::string& config_path, std::string& error)
{
    if (!LoadWithIncludes(config_path, root_, error))
    {
        return false;
    }
    config_dir_ = DirectoryOf(config_path);

    // 启动校验：四键并列是结构契约，缺一不可。
    for (const std::string& key : {"app", "camera", "device", "pipeline"})
    {
        if (!root_.contains(key) || !root_[key].is_object())
        {
            error = "启动配置缺少顶层键: " + key;
            return false;
        }
    }
    return true;
}

const nlohmann::json& ConfigService::Root() const
{
    return root_;
}

const nlohmann::json& ConfigService::Get(const std::string& pointer) const
{
    static const nlohmann::json kNullValue = nlohmann::json();
    try
    {
        return root_.at(nlohmann::json::json_pointer(pointer));
    }
    catch (const nlohmann::json::exception&)
    {
        return kNullValue;
    }
}

const std::string& ConfigService::ConfigDir() const
{
    return config_dir_;
}

} // namespace openmind::trajectory_plan
