#ifndef HYBRID_TRAJECTORY_PLAN_JSON_UTIL_H
#define HYBRID_TRAJECTORY_PLAN_JSON_UTIL_H

/**
 * @file json_util.h
 * @brief JSON 与文本文件读写工具：原子落盘、YAML 装载、矩阵互转。
 */

#include <nlohmann/json.hpp>

#include <Eigen/Core>

#include <array>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace openmind::trajectory_plan
{

/**
 * @brief 读取文本文件全部内容；失败返回 false。
 */
bool ReadTextFile(const std::string& path, std::string& content, std::string& error);

/**
 * @brief 原子写入文本文件（临时文件 + rename，读取侧不会看到半写文件）。
 */
bool WriteTextFileAtomic(const std::string& path, const std::string& content, std::string& error);

/**
 * @brief 解析 JSON 文本；失败返回 false。
 */
bool ParseJson(const std::string& content, nlohmann::json& document, std::string& error);

/**
 * @brief 读取 JSON 文件；失败返回 false。
 */
bool ReadJsonFile(const std::string& path, nlohmann::json& document, std::string& error);

/**
 * @brief 读取 YAML 文件为 JSON；失败返回 false。
 */
bool ReadYamlFile(const std::string& path, nlohmann::json& document, std::string& error);

/**
 * @brief yaml-cpp 节点 → JSON（数字优先，识别 true/false）。
 */
nlohmann::json YamlToJson(const YAML::Node& node);

/**
 * @brief 4x4 齐次变换（mm）→ JSON 行主序数组。
 */
nlohmann::json MatrixToJson(const Eigen::Matrix4d& matrix);

/**
 * @brief JSON 行主序 4x4 数组 → 齐次变换；失败返回 false。
 */
bool MatrixFromJson(const nlohmann::json& value, Eigen::Matrix4d& matrix, std::string& error);

/**
 * @brief 当前时间戳字符串 YYYYMMDD_HHMMSS。
 */
std::string TimestampCompact();

/**
 * @brief 当前时间戳毫秒字符串（request_id 用）。
 */
std::string TimestampMilliseconds();

/**
 * @brief 32 位随机十六进制字符串（generation_id 用）。
 */
std::string RandomHex32();

/**
 * @brief 确保目录存在；失败返回 false。
 */
bool EnsureDirectory(const std::string& path, std::string& error);

/**
 * @brief 路径拼接（用 '/' 连接，不要求绝对路径）。
 */
std::string JoinPath(const std::string& directory, const std::string& name);

/**
 * @brief 解析 request.txt：工件、拍照状态、三张深度图与主 RGB 图路径。
 */
bool ParseRequestText(const std::string& content,
                      std::string& workpiece_type,
                      Eigen::Matrix4d& t_b_o,
                      Eigen::Matrix4d& t_base_flange,
                      std::array<double, 6>& t_base_flange_joint_angles_deg,
                      std::string& depth_main_rel,
                      std::string& depth_left_rel,
                      std::string& depth_right_rel,
                      std::string& rgb_main_rel,
                      std::string& error);

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_JSON_UTIL_H
