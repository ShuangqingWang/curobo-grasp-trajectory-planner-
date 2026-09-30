/**
 * @file device_manager.cpp
 * @brief 设备管理器实现：从 device 配置节构造 AUBO-i12H 模型与碰撞球集合。
 */

#include "trajectory_plan/device/device_manager.h"

#include "trajectory_plan/device/aubo_i12h_model.h"
#include "trajectory_plan/tools/json_util.h"

namespace openmind::trajectory_plan
{
namespace
{

constexpr double kMetersToMillimeters = 1000.0;

/**
 * @brief 连杆名 → 连杆序号（与 curobo collision_spheres 的键一致）。
 */
int32_t LinkIndexByName(const std::string& name)
{
    static const std::vector<std::string> kLinkNames = {
        "base_link", "shoulder_Link", "upperArm_Link", "foreArm_Link",
        "wrist1_Link", "wrist2_Link", "wrist3_Link",
    };
    for (size_t index = 0; index < kLinkNames.size(); ++index)
    {
        if (name == kLinkNames[index])
        {
            return static_cast<int32_t>(index);
        }
    }
    return -1;
}

} // namespace

bool DeviceManager::Init(const nlohmann::json& device_config, std::string& error)
{
    if (!device_config.is_object())
    {
        error = "device 配置节必须是对象";
        return false;
    }
    const nlohmann::json& limits = device_config["joint_limits_deg"];
    std::array<double, 6> lower_deg {};
    std::array<double, 6> upper_deg {};
    if (limits.is_object() && limits["lower"].is_array() && limits["lower"].size() == 6 &&
        limits["upper"].is_array() && limits["upper"].size() == 6)
    {
        for (int64_t index = 0; index < 6; ++index)
        {
            lower_deg[static_cast<size_t>(index)] = limits["lower"][static_cast<size_t>(index)].get<double>();
            upper_deg[static_cast<size_t>(index)] = limits["upper"][static_cast<size_t>(index)].get<double>();
        }
    }
    else
    {
        error = "device/joint_limits_deg 必须包含 lower/upper 各 6 个数（degree）";
        return false;
    }

    auto model = std::make_unique<AuboI12hModel>();
    if (!model->Init(device_config["mdh"], lower_deg, upper_deg, error))
    {
        return false;
    }
    kinematics_ = std::move(model);

    // 碰撞球：curobo collision_spheres（米）→ 连杆局部坐标（mm）。
    const nlohmann::json& spheres = device_config["collision_spheres"];
    if (!spheres.is_object())
    {
        error = "device/collision_spheres 必须是对象";
        return false;
    }
    for (auto iterator = spheres.begin(); iterator != spheres.end(); ++iterator)
    {
        const int32_t link_index = LinkIndexByName(iterator.key());
        if (link_index < 0 || !iterator.value().is_array())
        {
            continue;
        }
        for (const nlohmann::json& sphere : iterator.value())
        {
            if (!sphere.is_object() || !sphere["center"].is_array() || sphere["center"].size() != 3 ||
                !sphere["radius"].is_number())
            {
                continue;
            }
            RobotCollisionSphere entry;
            entry.link_index = link_index;
            entry.center = Eigen::Vector3d(sphere["center"][0].get<double>(),
                                           sphere["center"][1].get<double>(),
                                           sphere["center"][2].get<double>()) *
                           kMetersToMillimeters;
            entry.radius = sphere["radius"].get<double>() * kMetersToMillimeters;
            collision_spheres_.push_back(entry);
        }
    }
    if (collision_spheres_.empty())
    {
        error = "device/collision_spheres 没有解析到任何碰撞球";
        return false;
    }

    // 自碰忽略对（连杆名 → 连杆名列表）。
    self_collision_ignore_pairs_.clear();
    const nlohmann::json& ignore = device_config["self_collision_ignore"];
    if (ignore.is_object())
    {
        for (auto iterator = ignore.begin(); iterator != ignore.end(); ++iterator)
        {
            const int32_t left = LinkIndexByName(iterator.key());
            if (left < 0 || !iterator.value().is_array())
            {
                continue;
            }
            for (const nlohmann::json& other : iterator.value())
            {
                const int32_t right = LinkIndexByName(other.get<std::string>());
                if (right >= 0 && right != left)
                {
                    self_collision_ignore_pairs_.push_back({left, right});
                }
            }
        }
    }

    // 自碰缓冲（curobo 按连杆给的缓冲，取全局最大，mm）。
    self_collision_buffer_mm_ = 0.0;
    const nlohmann::json& buffers = device_config["self_collision_buffer"];
    if (buffers.is_object())
    {
        for (auto iterator = buffers.begin(); iterator != buffers.end(); ++iterator)
        {
            if (iterator.value().is_number())
            {
                self_collision_buffer_mm_ =
                    std::max(self_collision_buffer_mm_, iterator.value().get<double>() * kMetersToMillimeters);
            }
        }
    }
    return true;
}

const RobotKinematicsInterface* DeviceManager::Kinematics() const
{
    return kinematics_.get();
}

const std::vector<RobotCollisionSphere>& DeviceManager::CollisionSpheres() const
{
    return collision_spheres_;
}

const std::vector<std::array<int32_t, 2>>& DeviceManager::SelfCollisionIgnorePairs() const
{
    return self_collision_ignore_pairs_;
}

double DeviceManager::SelfCollisionBufferMm() const
{
    return self_collision_buffer_mm_;
}

} // namespace openmind::trajectory_plan
