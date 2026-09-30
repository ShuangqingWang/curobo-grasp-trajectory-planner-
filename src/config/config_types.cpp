/**
 * @file config_types.cpp
 * @brief 配置强类型视图的解析实现：固定工位、相机标定、碰撞几何。
 */

#include "trajectory_plan/config/config_types.h"

#include <cstdint>

#include "trajectory_plan/tools/json_util.h"

namespace openmind::trajectory_plan
{

bool FixedStation::Parse(const nlohmann::json& node, FixedStation& station, std::string& error)
{
    if (!node.is_object())
    {
        error = "工位配置必须是对象";
        return false;
    }
    const nlohmann::json& q = node["q_deg"];
    const nlohmann::json& flange = node["flange_xyzrxryrz_mm_deg"];
    if (!q.is_array() || q.size() != 6)
    {
        error = "工位 q_deg 必须是 6 个数（degree）";
        return false;
    }
    if (!flange.is_array() || flange.size() != 6)
    {
        error = "工位 flange_xyzrxryrz_mm_deg 必须是 6 个数（mm/degree）";
        return false;
    }
    for (int64_t index = 0; index < 6; ++index)
    {
        station.q_deg[static_cast<size_t>(index)] = q[static_cast<size_t>(index)].get<double>();
    }
    station.flange_pose.x = flange[0].get<double>();
    station.flange_pose.y = flange[1].get<double>();
    station.flange_pose.z = flange[2].get<double>();
    station.flange_pose.rx = flange[3].get<double>();
    station.flange_pose.ry = flange[4].get<double>();
    station.flange_pose.rz = flange[5].get<double>();
    station.valid = true;
    return true;
}

bool CameraConfig::Parse(const nlohmann::json& node, CameraConfig& config, std::string& error)
{
    if (!node.is_object())
    {
        error = "相机配置必须是对象";
        return false;
    }
    for (const std::string& key : {"fx", "fy", "cx", "cy"})
    {
        if (!node.contains(key) || !node[key].is_number())
        {
            error = "相机配置缺少数值字段: " + key;
            return false;
        }
    }
    config.fx = node["fx"].get<double>();
    config.fy = node["fy"].get<double>();
    config.cx = node["cx"].get<double>();
    config.cy = node["cy"].get<double>();
    if (!node.contains("T_cam2flange") || !MatrixFromJson(node["T_cam2flange"], config.t_cam2flange, error))
    {
        error = "相机 T_cam2flange: " + error;
        return false;
    }
    return true;
}

bool BoxConfig::Parse(const nlohmann::json& node, BoxConfig& box, std::string& error)
{
    if (!node.is_object())
    {
        error = "箱体配置必须是对象";
        return false;
    }
    for (const std::string& key : {"negative_extent_xyz_mm", "positive_extent_xyz_mm"})
    {
        if (!node.contains(key) || !node[key].is_array() || node[key].size() != 3)
        {
            error = "箱体配置 " + key + " 必须是 3 个数（mm）";
            return false;
        }
    }
    for (int64_t index = 0; index < 3; ++index)
    {
        box.negative_extent[static_cast<int>(index)] =
            node["negative_extent_xyz_mm"][static_cast<size_t>(index)].get<double>();
        box.positive_extent[static_cast<int>(index)] =
            node["positive_extent_xyz_mm"][static_cast<size_t>(index)].get<double>();
        if (box.negative_extent[static_cast<int>(index)] < 0.0 || box.positive_extent[static_cast<int>(index)] < 0.0)
        {
            error = "箱体六向延伸必须为非负数（mm）";
            return false;
        }
    }
    if (node.contains("collision_margin_mm") && node["collision_margin_mm"].is_number())
    {
        box.collision_margin_mm = node["collision_margin_mm"].get<double>();
    }
    box.valid = true;
    return true;
}

} // namespace openmind::trajectory_plan
