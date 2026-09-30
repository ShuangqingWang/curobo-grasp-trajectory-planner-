#ifndef HYBRID_TRAJECTORY_PLAN_TIFF_DEPTH_CAMERA_H
#define HYBRID_TRAJECTORY_PLAN_TIFF_DEPTH_CAMERA_H

/**
 * @file tiff_depth_camera.h
 * @brief 最小 TIFF 解码器：无压缩的单通道 float32/uint16/uint8 → 深度缓冲（mm）。
 *
 * 现场 TIFF 为 little-endian、未压缩、每行一条 strip 的 float32 深度图。
 * 本解码器不依赖 libtiff，支持 strip 连续排布，压缩格式明确报错。
 */

#include <cstdint>
#include <string>
#include <vector>

#include "trajectory_plan/algorithm/depth_cloud_fuse_algorithm.h"
#include "trajectory_plan/camera/camera_manager.h"

namespace openmind::trajectory_plan
{

/**
 * @brief TIFF 深度解码器。
 */
class TiffDepthDecoder : public DepthDecoder
{
  public:
    TiffDepthDecoder() = default;
    TiffDepthDecoder(const TiffDepthDecoder&) = delete;
    TiffDepthDecoder& operator=(const TiffDepthDecoder&) = delete;

    /**
     * @brief 把 TIFF 文件字节解码为深度图（mm）。
     */
    bool Decode(const std::vector<uint8_t>& bytes, DepthMap& map, std::string& error) const override;

    /** @brief "tiff" */
    const std::string& Name() const override;

  private:
    static const std::string kName; ///< 解码器名字
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_TIFF_DEPTH_CAMERA_H
