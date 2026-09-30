#ifndef HYBRID_TRAJECTORY_PLAN_DEVICE_MANAGER_H
#define HYBRID_TRAJECTORY_PLAN_DEVICE_MANAGER_H

/**
 * @file device_manager.h
 * @brief 设备管理器：持有机型运动学/碰撞模型，不认识 pipeline、不控真机。
 */

#include <nlohmann/json.hpp>

#include <memory>
#include <string>
#include <vector>

#include "trajectory_plan/algorithm/collision_world_build_algorithm.h"
#include "trajectory_plan/algorithm/robot_kinematics_interface.h"

namespace openmind::trajectory_plan
{

/**
 * @brief 设备管理器：按 device 配置节构造机型模型。
 */
class DeviceManager
{
  public:
    DeviceManager() = default;
    DeviceManager(const DeviceManager&) = delete;
    DeviceManager& operator=(const DeviceManager&) = delete;

    /**
     * @brief 从合并配置的 device 节构造机型模型（aubo_i12H）。
     * @param device_config 合并后的 device 节（aubo.json）
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true
     */
    bool Init(const nlohmann::json& device_config, std::string& error);

    /**
     * @brief 机型运动学（stage 注入算法层用）。
     */
    const RobotKinematicsInterface* Kinematics() const;

    /**
     * @brief 机型碰撞球（连杆局部坐标，mm）。
     */
    const std::vector<RobotCollisionSphere>& CollisionSpheres() const;

    /**
     * @brief 自碰忽略连杆对。
     */
    const std::vector<std::array<int32_t, 2>>& SelfCollisionIgnorePairs() const;

    /**
     * @brief 自碰附加缓冲（mm）。
     */
    double SelfCollisionBufferMm() const;

  private:
    std::unique_ptr<RobotKinematicsInterface> kinematics_; ///< 机型运动学
    std::vector<RobotCollisionSphere> collision_spheres_; ///< 碰撞球（mm）
    std::vector<std::array<int32_t, 2>> self_collision_ignore_pairs_; ///< 自碰忽略对
    double self_collision_buffer_mm_ = 5.0; ///< 自碰附加缓冲（mm）
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_DEVICE_MANAGER_H
