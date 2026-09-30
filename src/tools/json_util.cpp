/**
 * @file json_util.cpp
 * @brief JSON/YAML 文件读写与文本工具：原子落盘、矩阵互转、时间戳与随机 ID。
 */

#include "trajectory_plan/tools/json_util.h"

#include "trajectory_plan/algorithm/pose.h"

#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <random>
#include <sstream>

#include "yaml-cpp/yaml.h"

namespace openmind::trajectory_plan
{

bool ReadTextFile(const std::string& path, std::string& content, std::string& error)
{
    std::ifstream stream(path, std::ios::in | std::ios::binary);
    if (!stream.is_open())
    {
        error = "无法打开文件: " + path;
        return false;
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    if (stream.bad())
    {
        error = "读取文件失败: " + path;
        return false;
    }
    content = buffer.str();
    return true;
}

bool WriteTextFileAtomic(const std::string& path, const std::string& content, std::string& error)
{
    const std::string directory = path.substr(0, path.find_last_of('/'));
    if (!directory.empty() && !EnsureDirectory(directory, error))
    {
        return false;
    }
    const std::string temporary = path + ".tmp." + std::to_string(static_cast<long long>(::getpid()));
    {
        std::ofstream stream(temporary, std::ios::out | std::ios::binary | std::ios::trunc);
        if (!stream.is_open())
        {
            error = "无法写临时文件: " + temporary;
            return false;
        }
        stream << content;
        stream.flush();
        if (stream.bad())
        {
            error = "写文件失败: " + temporary;
            return false;
        }
    }
    if (std::rename(temporary.c_str(), path.c_str()) != 0)
    {
        error = "rename 失败: " + temporary + " -> " + path;
        return false;
    }
    return true;
}

bool ParseJson(const std::string& content, nlohmann::json& document, std::string& error)
{
    try
    {
        document = nlohmann::json::parse(content);
        return true;
    }
    catch (const nlohmann::json::exception& exception)
    {
        error = std::string("JSON 解析失败: ") + exception.what();
        return false;
    }
}

bool ReadJsonFile(const std::string& path, nlohmann::json& document, std::string& error)
{
    std::string content;
    if (!ReadTextFile(path, content, error))
    {
        return false;
    }
    return ParseJson(content, document, error);
}

bool ReadYamlFile(const std::string& path, nlohmann::json& document, std::string& error)
{
    try
    {
        const YAML::Node node = YAML::LoadFile(path);
        if (node.IsNull())
        {
            document = nlohmann::json::object();
            return true;
        }
        std::stringstream stream;
        // yaml-cpp 没有直接转 JSON；用一次往返太绕，改为按节点递归转换。
        document = YamlToJson(node);
        return true;
    }
    catch (const YAML::Exception& exception)
    {
        error = std::string("YAML 解析失败: ") + exception.what();
        return false;
    }
}

nlohmann::json YamlToJson(const YAML::Node& node)
{
    if (node.IsScalar())
    {
        const std::string text = node.as<std::string>();
        // 布尔 / 整数 / 浮点优先；否则按字符串。
        if (text == "true" || text == "True" || text == "TRUE")
        {
            return nlohmann::json(true);
        }
        if (text == "false" || text == "False" || text == "FALSE")
        {
            return nlohmann::json(false);
        }
        // 必须整串都是数字才算数字。std::stod 只解析前缀就返回，
        // 例如 "127.0.0.1" 会被它读成 127.0 而不报错，导致 IP、版本号一类
        // 的配置被静默转成数字。故这里检查是否把整串都消费掉了。
        try
        {
            size_t consumed = 0;
            const double value = std::stod(text, &consumed);
            if (consumed == text.size())
            {
                return nlohmann::json(value);
            }
        }
        catch (const std::exception&)
        {
        }
        return nlohmann::json(text);
    }
    if (node.IsSequence())
    {
        nlohmann::json array = nlohmann::json::array();
        for (const YAML::Node& item : node)
        {
            array.push_back(YamlToJson(item));
        }
        return array;
    }
    if (node.IsMap())
    {
        nlohmann::json object = nlohmann::json::object();
        for (const auto& entry : node)
        {
            object[entry.first.as<std::string>()] = YamlToJson(entry.second);
        }
        return object;
    }
    return nlohmann::json();
}

nlohmann::json MatrixToJson(const Eigen::Matrix4d& matrix)
{
    nlohmann::json rows = nlohmann::json::array();
    for (int64_t row = 0; row < 4; ++row)
    {
        nlohmann::json values = nlohmann::json::array();
        for (int64_t column = 0; column < 4; ++column)
        {
            values.push_back(matrix(static_cast<int>(row), static_cast<int>(column)));
        }
        rows.push_back(std::move(values));
    }
    return rows;
}

bool MatrixFromJson(const nlohmann::json& value, Eigen::Matrix4d& matrix, std::string& error)
{
    if (!value.is_array() || value.size() != 4)
    {
        error = "矩阵必须是 4 行数组";
        return false;
    }
    for (int64_t row = 0; row < 4; ++row)
    {
        if (!value[static_cast<size_t>(row)].is_array() || value[static_cast<size_t>(row)].size() != 4)
        {
            error = "矩阵每行必须是 4 个数";
            return false;
        }
        for (int64_t column = 0; column < 4; ++column)
        {
            const nlohmann::json& entry = value[static_cast<size_t>(row)][static_cast<size_t>(column)];
            if (!entry.is_number())
            {
                error = "矩阵元素必须是数值";
                return false;
            }
            matrix(static_cast<int>(row), static_cast<int>(column)) = entry.get<double>();
        }
    }
    return true;
}

std::string TimestampCompact()
{
    const std::time_t now = std::time(nullptr);
    std::tm local_time {};
    localtime_r(&now, &local_time);
    char buffer[32] {};
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &local_time);
    return std::string(buffer);
}

std::string TimestampMilliseconds()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
    std::tm local_time {};
    localtime_r(&seconds, &local_time);
    char buffer[32] {};
    std::strftime(buffer, sizeof(buffer), "%Y%m%d%H%M%S", &local_time);
    char result[40] {};
    std::snprintf(result, sizeof(result), "%s%03lld", buffer, static_cast<long long>(milliseconds));
    return std::string(result);
}

std::string RandomHex32()
{
    static constexpr char kHexDigits[] = "0123456789abcdef";
    std::random_device device;
    std::mt19937_64 generator(device());
    std::uniform_int_distribution<uint64_t> distribution;
    std::string result;
    result.reserve(32);
    for (int64_t index = 0; index < 4; ++index)
    {
        const uint64_t value = distribution(generator);
        for (int64_t shift = 60; shift >= 0; shift -= 4)
        {
            result.push_back(kHexDigits[(value >> shift) & 0xF]);
        }
    }
    return result;
}

bool EnsureDirectory(const std::string& path, std::string& error)
{
    if (path.empty())
    {
        return true;
    }
    struct stat status
    {
    };
    if (stat(path.c_str(), &status) == 0)
    {
        return S_ISDIR(status.st_mode);
    }
    // 逐级创建，现场目录多为两层以内。
    const size_t separator = path.find_last_of('/');
    if (separator != std::string::npos && separator > 0)
    {
        const std::string parent = path.substr(0, separator);
        if (!EnsureDirectory(parent, error))
        {
            return false;
        }
    }
    if (mkdir(path.c_str(), 0755) != 0)
    {
        error = "创建目录失败: " + path;
        return false;
    }
    return true;
}

std::string JoinPath(const std::string& directory, const std::string& name)
{
    if (directory.empty())
    {
        return name;
    }
    if (directory.back() == '/')
    {
        return directory + name;
    }
    return directory + "/" + name;
}

bool ParseRequestText(const std::string& content,
                      std::string& workpiece_type,
                      Eigen::Matrix4d& t_b_o,
                      Eigen::Matrix4d& t_base_flange,
                      std::array<double, 6>& t_base_flange_joint_angles_deg,
                      std::string& depth_main_rel,
                      std::string& depth_left_rel,
                      std::string& depth_right_rel,
                      std::string& rgb_main_rel,
                      std::string& error)
{
    workpiece_type.clear();
    t_b_o = Eigen::Matrix4d::Identity();
    t_base_flange = Eigen::Matrix4d::Identity();
    t_base_flange_joint_angles_deg.fill(0.0);
    depth_main_rel = "raw_depth.tiff";
    depth_left_rel = "raw_depth_left.tiff";
    depth_right_rel = "raw_depth_right.tiff";
    rgb_main_rel = "raw_color.png";
    bool has_matrix = false;
    bool has_base_flange = false;
    bool has_base_flange_joint_angles = false;
    std::istringstream stream(content);
    std::string line;
    while (std::getline(stream, line))
    {
        const size_t comment = line.find('#');
        if (comment != std::string::npos)
        {
            line = line.substr(0, comment);
        }
        const size_t first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos)
        {
            continue;
        }
        line = line.substr(first);
        const size_t last = line.find_last_not_of(" \t\r");
        if (last != std::string::npos)
        {
            line = line.substr(0, last + 1);
        }
        size_t equal = line.find('=');
        if (equal == std::string::npos)
        {
            equal = line.find(':');
        }
        if (equal == std::string::npos)
        {
            continue;
        }
        std::string key = line.substr(0, equal);
        std::string value = line.substr(equal + 1);
        while (!key.empty() && (key.back() == ' ' || key.back() == '\t'))
        {
            key.pop_back();
        }
        while (!value.empty() && (value.front() == ' ' || value.front() == '\t'))
        {
            value.erase(value.begin());
        }
        if (key == "workpiece_type")
        {
            workpiece_type = value;
        }
        else if (key == "depth_main_path")
        {
            depth_main_rel = value;
        }
        else if (key == "depth_left_path")
        {
            depth_left_rel = value;
        }
        else if (key == "depth_right_path")
        {
            depth_right_rel = value;
        }
        else if (key == "rgb_main_path")
        {
            rgb_main_rel = value;
        }
        else if (key == "T_B_O_target" || key == "T_base_flange")
        {
            Eigen::Matrix4d& matrix = key == "T_B_O_target" ? t_b_o : t_base_flange;
            const std::string field_name = key;
            if (!value.empty())
            {
                nlohmann::json document;
                if (!ParseJson(value, document, error) || !MatrixFromJson(document, matrix, error))
                {
                    error = field_name + " JSON 无效";
                    return false;
                }
                if (key == "T_B_O_target")
                {
                    has_matrix = true;
                }
                else
                {
                    has_base_flange = true;
                }
                continue;
            }
            for (int32_t row = 0; row < 4; ++row)
            {
                if (!std::getline(stream, line))
                {
                    error = field_name + " 需要 4 行矩阵";
                    return false;
                }
                const size_t row_comment = line.find('#');
                if (row_comment != std::string::npos)
                {
                    line = line.substr(0, row_comment);
                }
                std::istringstream row_stream(line);
                for (int32_t column = 0; column < 4; ++column)
                {
                    if (!(row_stream >> matrix(row, column)))
                    {
                        error = field_name + " 每一行必须是 4 个数";
                        return false;
                    }
                }
            }
            if (key == "T_B_O_target")
            {
                has_matrix = true;
            }
            else
            {
                has_base_flange = true;
            }
        }
        else if (key == "T_base_flange_joint_angles_deg")
        {
            nlohmann::json document;
            if (!ParseJson(value, document, error) || !document.is_array() || document.size() != 6)
            {
                error = "T_base_flange_joint_angles_deg JSON 无效";
                return false;
            }
            for (size_t index = 0; index < t_base_flange_joint_angles_deg.size(); ++index)
            {
                if (!document[index].is_number())
                {
                    error = "T_base_flange_joint_angles_deg 必须是 6 个数";
                    return false;
                }
                t_base_flange_joint_angles_deg[index] = document[index].get<double>();
            }
            has_base_flange_joint_angles = true;
        }
    }
    if (workpiece_type.empty() || !has_matrix || !has_base_flange || !has_base_flange_joint_angles)
    {
        error = "request 必须包含 workpiece_type、T_B_O_target、T_base_flange 和 T_base_flange_joint_angles_deg";
        return false;
    }
    return ValidateTransform(t_b_o, "T_B_O_target", error) &&
           ValidateTransform(t_base_flange, "T_base_flange", error);
}

} // namespace openmind::trajectory_plan
