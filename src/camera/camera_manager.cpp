/**
 * @file camera_manager.cpp
 * @brief 相机管理器实现：固定 TIFF float32 mm 解码器。
 */

#include "trajectory_plan/camera/camera_manager.h"

#include "trajectory_plan/camera/tiff_depth_camera.h"

namespace openmind::trajectory_plan
{

bool CameraManager::Init(std::string& error)
{
    decoder_ = std::make_unique<TiffDepthDecoder>();
    return true;
}

bool CameraManager::DecodeDepth(const std::vector<uint8_t>& bytes, DepthMap& map, std::string& error) const
{
    if (!decoder_)
    {
        error = "相机解码器未初始化";
        return false;
    }
    return decoder_->Decode(bytes, map, error);
}

} // namespace openmind::trajectory_plan
