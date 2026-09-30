#ifndef HYBRID_TRAJECTORY_PLAN_CONFIG_TYPES_H
#define HYBRID_TRAJECTORY_PLAN_CONFIG_TYPES_H

/**
 * @file config_types.h
 * @brief 配置的强类型视图：固定工位、相机标定、碰撞几何（解析层使用的结构）。
 */

#include <nlohmann/json.hpp>

#include <Eigen/Core>

#include <array>
#include <string>

#include "trajectory_plan/algorithm/pose.h"

namespace openmind::trajectory_plan
{

/**
 * @brief 固定工位：拍照 / 放置的关节角与法兰位姿。
 */
struct FixedStation
{
    JointDegrees q_deg {}; ///< 六轴关节角（degree）
    Pose flange_pose;      ///< 法兰 xyzrxryrz（mm/deg）
    bool valid = false;    ///< 是否从配置解析成功

    /** @brief 法兰 4x4 齐次（mm）。 */
    Eigen::Matrix4d FlangeMatrix() const
    {
        return TransformFromPose(flange_pose);
    }

    /**
     * @brief 从 app.json 的 photo/place 节解析。
     */
    static bool Parse(const nlohmann::json& node, FixedStation& station, std::string& error);
};

/**
 * @brief 单路相机标定（camera.json 的 main/left/right 节）。
 */
struct CameraConfig
{
    std::string name;      ///< main / left / right
    double fx = 0.0;       ///< 焦距 x，pixel
    double fy = 0.0;       ///< 焦距 y，pixel
    double cx = 0.0;       ///< 主点 x，pixel
    double cy = 0.0;       ///< 主点 y，pixel
    Eigen::Matrix4d t_cam2flange = Eigen::Matrix4d::Identity(); ///< 相机 → 法兰（mm）

    /**
     * @brief 从 camera.json 的对应节解析。
     */
    static bool Parse(const nlohmann::json& node, CameraConfig& config, std::string& error);
};

/**
 * @brief 六向轴对齐箱体（app.json 的 end_effector / base_platform 节）。
 */
struct BoxConfig
{
    Eigen::Vector3d negative_extent = Eigen::Vector3d::Zero(); ///< -X/-Y/-Z 延伸（mm）
    Eigen::Vector3d positive_extent = Eigen::Vector3d::Zero(); ///< +X/+Y/+Z 延伸（mm）
    double collision_margin_mm = 0.0; ///< 外附加安全裕量（mm）
    bool valid = false; ///< 是否解析成功

    /**
     * @brief 从 JSON 节解析 negative_extent_xyz_mm / positive_extent_xyz_mm / collision_margin_mm。
     */
    static bool Parse(const nlohmann::json& node, BoxConfig& box, std::string& error);
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_CONFIG_TYPES_H
