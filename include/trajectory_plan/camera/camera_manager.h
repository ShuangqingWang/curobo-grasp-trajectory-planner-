#ifndef HYBRID_TRAJECTORY_PLAN_CAMERA_MANAGER_H
#define HYBRID_TRAJECTORY_PLAN_CAMERA_MANAGER_H

/**
 * @file camera_manager.h
 * @brief 相机管理器：只提供 TCP/落盘 TIFF → 深度缓冲的解码能力，不认识 pipeline。
 *
 * 本项目不连相机 SDK；camera/ 只解析 TCP 送来的 TIFF 字节。
 */

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "trajectory_plan/algorithm/depth_cloud_fuse_algorithm.h"

namespace openmind::trajectory_plan
{

/**
 * @brief TIFF 深度解码器接口。
 */
class DepthDecoder
{
  public:
    virtual ~DepthDecoder() = default;

    /**
     * @brief 把 TIFF 文件字节解码为深度图（mm）。
     * @param bytes TIFF 原文件字节
     * @param map 输出：深度图（mm，行主序）
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true
     */
    virtual bool Decode(const std::vector<uint8_t>& bytes, DepthMap& map, std::string& error) const = 0;

    /** @brief 解码器名字（日志用）。 */
    virtual const std::string& Name() const = 0;
};

/**
 * @brief 相机管理器：持有 TIFF 深度解码器。
 */
class CameraManager
{
  public:
    CameraManager() = default;
    CameraManager(const CameraManager&) = delete;
    CameraManager& operator=(const CameraManager&) = delete;

    /**
     * @brief 初始化（本项目固定 TIFF float32 mm 解码器）。
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true
     */
    bool Init(std::string& error);

    /**
     * @brief 解码一路 TIFF 字节。
     */
    bool DecodeDepth(const std::vector<uint8_t>& bytes, DepthMap& map, std::string& error) const;

  private:
    std::unique_ptr<DepthDecoder> decoder_; ///< TIFF 解码器
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_CAMERA_MANAGER_H
