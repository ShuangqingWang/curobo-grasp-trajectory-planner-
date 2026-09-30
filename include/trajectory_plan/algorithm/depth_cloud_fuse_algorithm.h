#ifndef HYBRID_TRAJECTORY_PLAN_DEPTH_CLOUD_FUSE_ALGORITHM_H
#define HYBRID_TRAJECTORY_PLAN_DEPTH_CLOUD_FUSE_ALGORITHM_H

/**
 * @file depth_cloud_fuse_algorithm.h
 * @brief 三路深度图 × 内参 × T_B_C_i → 基座系一份融合点云（不融主图、不挖点）。
 */

#include <Eigen/Core>

#include <string>
#include <vector>

namespace openmind::trajectory_plan
{

/**
 * @brief 单路相机投影配置。
 */
struct DepthCameraCalibration
{
    double fx = 0.0; ///< 焦距 x，pixel
    double fy = 0.0; ///< 焦距 y，pixel
    double cx = 0.0; ///< 主点 x，pixel
    double cy = 0.0; ///< 主点 y，pixel
    double min_depth_mm = 0.0; ///< 有效深度下界
    double max_depth_mm = 0.0; ///< 有效深度上界
    Eigen::Matrix4d t_flange_camera = Eigen::Matrix4d::Identity(); ///< 相机 → 法兰（mm）
    int32_t stride_px = 1; ///< 像素抽稀步长（性能控制，默认全采）
};

/**
 * @brief 一路解码后的深度图（mm，行主序）。
 */
struct DepthMap
{
    int32_t width = 0;  ///< 宽，pixel
    int32_t height = 0; ///< 高，pixel
    std::vector<double> depth_mm; ///< 行主序深度，单位 mm；size = width*height
};

/**
 * @brief 深度图融合（纯计算）。
 *
 * T_B_C_i = T_B_F_photo · T_F_C_i；三路点统一表达在基座系，不挖点、不融主图。
 */
class DepthCloudFuseAlgorithm
{
  public:
    DepthCloudFuseAlgorithm() = default;
    DepthCloudFuseAlgorithm(const DepthCloudFuseAlgorithm&) = delete;
    DepthCloudFuseAlgorithm& operator=(const DepthCloudFuseAlgorithm&) = delete;

    /**
     * @brief 初始化拍照法兰与三路标定。
     * @param t_base_flange_photo 拍照法兰相对基座 4x4 齐次（mm）
     * @param main_calib 主相机标定
     * @param left_calib 左相机标定
     * @param right_calib 右相机标定
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true
     */
    bool Init(const Eigen::Matrix4d& t_base_flange_photo,
              const DepthCameraCalibration& main_calib,
              const DepthCameraCalibration& left_calib,
              const DepthCameraCalibration& right_calib,
              std::string& error);

    /**
     * @brief 融合三路深度图为基座系点云。
     * @param main_map 主相机深度图（mm）
     * @param left_map 左相机深度图（mm）
     * @param right_map 右相机深度图（mm）
     * @param points 输出：基座系点（mm）
     * @param camera_indices 输出：每个点的来源相机索引（0/1/2，与 points 对齐）
     * @param source_pixel_indices 输出：每个点在来源深度图中的行主序像素下标（与 points 对齐）
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true
     */
    bool Fuse(const DepthMap& main_map,
              const DepthMap& left_map,
              const DepthMap& right_map,
              std::vector<Eigen::Vector3d>& points,
              std::vector<uint8_t>& camera_indices,
              std::vector<uint32_t>& source_pixel_indices,
              std::string& error) const;

  private:
    Eigen::Matrix4d t_base_flange_photo_ = Eigen::Matrix4d::Identity(); ///< 拍照法兰（mm）
    DepthCameraCalibration main_calib_;  ///< 主相机标定
    DepthCameraCalibration left_calib_;  ///< 左相机标定
    DepthCameraCalibration right_calib_; ///< 右相机标定
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_DEPTH_CLOUD_FUSE_ALGORITHM_H
