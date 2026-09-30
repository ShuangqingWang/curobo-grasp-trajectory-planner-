/**
 * @file depth_cloud_fuse_algorithm.cpp
 * @brief 三路深度图 × 内参 × T_B_C_i → 基座系一份融合点云。
 */

#include "trajectory_plan/algorithm/depth_cloud_fuse_algorithm.h"

#include "trajectory_plan/algorithm/pose.h"

#include <algorithm>
#include <cmath>
#include <thread>

namespace openmind::trajectory_plan
{

bool DepthCloudFuseAlgorithm::Init(const Eigen::Matrix4d& t_base_flange_photo,
                                   const DepthCameraCalibration& main_calib,
                                   const DepthCameraCalibration& left_calib,
                                   const DepthCameraCalibration& right_calib,
                                   std::string& error)
{
    std::string transform_error;
    if (!ValidateTransform(t_base_flange_photo, "photo flange", transform_error))
    {
        error = "photo flange: " + transform_error;
        return false;
    }
    for (const DepthCameraCalibration* calibration : {&main_calib, &left_calib, &right_calib})
    {
        if (calibration->fx <= 0.0 || calibration->fy <= 0.0)
        {
            error = "相机内参 fx/fy 必须为正";
            return false;
        }
        if (!ValidateTransform(calibration->t_flange_camera, "T_cam2flange", transform_error))
        {
            error = "T_cam2flange: " + transform_error;
            return false;
        }
    }
    t_base_flange_photo_ = t_base_flange_photo;
    main_calib_ = main_calib;
    left_calib_ = left_calib;
    right_calib_ = right_calib;
    return true;
}

bool DepthCloudFuseAlgorithm::Fuse(const DepthMap& main_map,
                                   const DepthMap& left_map,
                                   const DepthMap& right_map,
                                   std::vector<Eigen::Vector3d>& points,
                                   std::vector<uint8_t>& camera_indices,
                                   std::vector<uint32_t>& source_pixel_indices,
                                   std::string& error) const
{
    points.clear();
    camera_indices.clear();
    source_pixel_indices.clear();
    const std::array<const DepthMap*, 3> maps = {&main_map, &left_map, &right_map};
    const std::array<const DepthCameraCalibration*, 3> calibrations = {&main_calib_, &left_calib_,
                                                                      &right_calib_};

    // 先统一校验，避免线程里报错还要跨线程传递失败原因。
    for (int64_t camera = 0; camera < 3; ++camera)
    {
        const DepthMap& map = *maps[static_cast<size_t>(camera)];
        if (map.width <= 0 || map.height <= 0 ||
            map.depth_mm.size() != static_cast<size_t>(map.width) * map.height)
        {
            error = "深度图尺寸与数据量不一致（相机 " + std::to_string(camera) + "）";
            return false;
        }
    }

    // 改动七：三路之间没有数据依赖，各起一个线程并行反投影。
    // ⚠️ 拼接顺序**必须固定为 主 → 左 → 右**：主文档要求"同输入得到同顺序候选"，
    //    点云顺序一旦变化，体素化与下游全部结果都会漂移。因此每路写进自己的
    //    缓冲区，最后按固定下标顺序拼接，而不是谁先算完谁先追加。
    std::array<std::vector<Eigen::Vector3d>, 3> per_camera_points;
    std::array<std::vector<uint8_t>, 3> per_camera_indices;
    std::array<std::vector<uint32_t>, 3> per_camera_source_pixels;
    std::array<std::thread, 3> workers;

    const auto project = [&](int64_t camera) {
        const DepthMap& map = *maps[static_cast<size_t>(camera)];
        const DepthCameraCalibration& calibration = *calibrations[static_cast<size_t>(camera)];
        std::vector<Eigen::Vector3d>& out_points = per_camera_points[static_cast<size_t>(camera)];
        std::vector<uint8_t>& out_indices = per_camera_indices[static_cast<size_t>(camera)];
        std::vector<uint32_t>& out_source_pixels =
            per_camera_source_pixels[static_cast<size_t>(camera)];
        // T_B_C = T_B_F_photo · T_F_C。
        const Eigen::Matrix4d t_base_camera = t_base_flange_photo_ * calibration.t_flange_camera;
        const Eigen::Matrix3d rotation = t_base_camera.block<3, 3>(0, 0);
        const Eigen::Vector3d translation = t_base_camera.block<3, 1>(0, 3);
        const int32_t stride = std::max(1, calibration.stride_px);

        // 本路点数上限：按 stride 抽样后的像素数。一次 reserve 到位，避免反复扩容。
        const size_t upper_bound = static_cast<size_t>((map.height + stride - 1) / stride) *
                                   static_cast<size_t>((map.width + stride - 1) / stride);
        out_points.reserve(upper_bound);
        out_indices.reserve(upper_bound);
        out_source_pixels.reserve(upper_bound);

        for (int32_t row = 0; row < map.height; row += stride)
        {
            for (int32_t column = 0; column < map.width; column += stride)
            {
                const double depth = map.depth_mm[static_cast<size_t>(row) * map.width + column];
                if (!std::isfinite(depth) || depth < calibration.min_depth_mm ||
                    depth > calibration.max_depth_mm)
                {
                    continue;
                }
                // 像素反投影到相机系（mm），再转到基座系。
                Eigen::Vector3d point_camera;
                point_camera.x() = (static_cast<double>(column) - calibration.cx) * depth / calibration.fx;
                point_camera.y() = (static_cast<double>(row) - calibration.cy) * depth / calibration.fy;
                point_camera.z() = depth;
                out_points.push_back(rotation * point_camera + translation);
                out_indices.push_back(static_cast<uint8_t>(camera));
                out_source_pixels.push_back(static_cast<uint32_t>(
                    static_cast<size_t>(row) * static_cast<size_t>(map.width) + column));
            }
        }
    };

    for (int64_t camera = 0; camera < 3; ++camera)
    {
        workers[static_cast<size_t>(camera)] = std::thread(project, camera);
    }
    for (std::thread& worker : workers)
    {
        worker.join();
    }

    size_t total = 0;
    for (const std::vector<Eigen::Vector3d>& part : per_camera_points)
    {
        total += part.size();
    }
    points.reserve(total);
    camera_indices.reserve(total);
    source_pixel_indices.reserve(total);
    // 固定的 主 → 左 → 右 顺序拼接，与串行实现逐点一致。
    for (size_t camera = 0; camera < per_camera_points.size(); ++camera)
    {
        points.insert(points.end(), per_camera_points[camera].begin(),
                      per_camera_points[camera].end());
        camera_indices.insert(camera_indices.end(), per_camera_indices[camera].begin(),
                              per_camera_indices[camera].end());
        source_pixel_indices.insert(source_pixel_indices.end(), per_camera_source_pixels[camera].begin(),
                                    per_camera_source_pixels[camera].end());
    }

    if (points.empty())
    {
        error = "三路深度图融合后没有任何有效点";
        return false;
    }
    return true;
}

} // namespace openmind::trajectory_plan
