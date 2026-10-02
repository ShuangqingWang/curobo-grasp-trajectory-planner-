/**
 * @file fuse_base_cloud_stage.cpp
 * @brief flow trajectory / 二 fuse_base_cloud：三路深度解码 → 基座系融合点云。
 *
 * 启动准备：相机内外参与深度处理模块（复用启动加载的标定，不每轮重新解析）。
 * 触发执行：校验并解码本轮三路深度图，过滤无效深度，反投影并融合到基座系。
 * 完整深度解码和像素检查由本 stage 执行；深度单位 mm。
 */

#include <array>
#include <chrono>

#include "trajectory_plan/algorithm/depth_cloud_fuse_algorithm.h"
#include "trajectory_plan/config/config_types.h"
#include "trajectory_plan/pipeline/stage_registry.h"
#include "trajectory_plan/tools/log.h"

namespace openmind::trajectory_plan
{

class FuseBaseCloudStage : public StageBase
{
  public:
    bool Init(const StageConfig& config, std::string& error) override
    {
        BindConfig(config);
        if (config.camera_manager == nullptr || config.config_root == nullptr)
        {
            error = "缺少相机管理器或配置";
            return false;
        }
        camera_manager_ = config.camera_manager;
        const nlohmann::json& cameras = (*config.config_root)["camera"];
        CameraConfig main_cfg;
        CameraConfig left_cfg;
        CameraConfig right_cfg;
        if (!CameraConfig::Parse(cameras["main"], main_cfg, error) ||
            !CameraConfig::Parse(cameras["left"], left_cfg, error) ||
            !CameraConfig::Parse(cameras["right"], right_cfg, error))
        {
            return false;
        }
        const double min_depth = cameras.value("depth_min_mm", 150.0);
        const double max_depth = cameras.value("depth_max_mm", 2500.0);
        auto to_calibration = [min_depth, max_depth](const CameraConfig& config) {
            DepthCameraCalibration calibration;
            calibration.fx = config.fx;
            calibration.fy = config.fy;
            calibration.cx = config.cx;
            calibration.cy = config.cy;
            calibration.t_flange_camera = config.t_cam2flange;
            calibration.min_depth_mm = min_depth;
            calibration.max_depth_mm = max_depth;
            // 点云按 stride 4 抽样，只用于可视化与归档；碰撞世界改用全分辨率深度图（文档 2.5.3）。
            calibration.stride_px = 4;
            return calibration;
        };
        main_calibration_ = to_calibration(main_cfg);
        left_calibration_ = to_calibration(left_cfg);
        right_calibration_ = to_calibration(right_cfg);
        LOG_INFO << log_tag_ << "[初始化] cameras=3 calibration=loaded 深度范围=[" << min_depth
                 << "," << max_depth << "]mm";
        return true;
    }

    bool Execute(PipelineContext& ctx, std::string& error) override
    {
        if (ctx.depth_main_bytes.empty() || ctx.depth_left_bytes.empty() ||
            ctx.depth_right_bytes.empty())
        {
            error = "本轮三路深度图不完整";
            return false;
        }
        // 融合需要本轮拍照工位法兰；由上一个 stage 准备。
        DepthCloudFuseAlgorithm fuser;
        if (!fuser.Init(ctx.photo_flange, main_calibration_, left_calibration_, right_calibration_,
                        error))
        {
            return false;
        }
        const auto decode_started = std::chrono::steady_clock::now();
        DepthMap main_map;
        DepthMap left_map;
        DepthMap right_map;
        if (!camera_manager_->DecodeDepth(ctx.depth_main_bytes, main_map, error) ||
            !camera_manager_->DecodeDepth(ctx.depth_left_bytes, left_map, error) ||
            !camera_manager_->DecodeDepth(ctx.depth_right_bytes, right_map, error))
        {
            return false;
        }
        ctx.main_depth_width = main_map.width;
        ctx.main_depth_height = main_map.height;
        const double decode_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - decode_started)
                .count();
        // 全分辨率深度交给碰撞世界：GPU 服务直接反投影全部像素（文档 2.5.3）。
        const std::array<const DepthMap*, 3> maps = {&main_map, &left_map, &right_map};
        const std::array<const DepthCameraCalibration*, 3> calibrations = {
            &main_calibration_, &left_calibration_, &right_calibration_};
        for (size_t camera = 0; camera < 3; ++camera)
        {
            const DepthMap& map = *maps[camera];
            const DepthCameraCalibration& calibration = *calibrations[camera];
            CollisionDepthImage& target = ctx.collision_depths[camera];
            target.width = map.width;
            target.height = map.height;
            target.depth_mm.assign(map.depth_mm.begin(), map.depth_mm.end());
            target.fx = calibration.fx;
            target.fy = calibration.fy;
            target.cx = calibration.cx;
            target.cy = calibration.cy;
            target.min_depth_mm = calibration.min_depth_mm;
            target.max_depth_mm = calibration.max_depth_mm;
            target.t_base_camera = ctx.photo_flange * calibration.t_flange_camera;
        }

        const auto fusion_started = std::chrono::steady_clock::now();
        if (!fuser.Fuse(main_map, left_map, right_map, ctx.cloud_b, ctx.cloud_camera_indices,
                        ctx.cloud_source_pixel_indices, error))
        {
            return false;
        }
        const double fusion_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - fusion_started)
                .count();
        if (ctx.cloud_b.empty())
        {
            error = "三路深度融合后点云为空";
            return false;
        }
        LOG_INFO << log_tag_ << "    cameras=3 points=" << ctx.cloud_b.size()
                 << " calibration=reused decode_ms=" << decode_ms << " fusion_ms=" << fusion_ms;
        return true;
    }

  private:
    const CameraManager* camera_manager_ = nullptr; ///< TIFF 解码
    DepthCameraCalibration main_calibration_;  ///< 主相机标定（启动加载）
    DepthCameraCalibration left_calibration_;  ///< 左相机标定
    DepthCameraCalibration right_calibration_; ///< 右相机标定
};

REGISTER_STAGE("fuse_base_cloud", FuseBaseCloudStage);

} // namespace openmind::trajectory_plan
