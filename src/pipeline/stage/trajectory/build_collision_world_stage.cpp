/**
 * @file build_collision_world_stage.cpp
 * @brief flow trajectory / 三 build_collision_world：本轮点云 → GPU 碰撞世界（业务步骤 [1/6]）。
 *
 * 触发执行只做三件事：**探活 → 写 cloud.bin → set_world**。GPU 服务按文档 2.5
 * 截断点云、确定长方体（不外扩）、3mm 体素化（点云 + 工作台）、PBA 精确 EDT 得 fp16 ESDF。
 */

#include <chrono>

#include "trajectory_plan/algorithm/curobo_plan_client.h"
#include "trajectory_plan/tools/json_util.h"
#include "trajectory_plan/config/config_types.h"
#include "trajectory_plan/pipeline/stage_registry.h"
#include "trajectory_plan/tools/log.h"

namespace openmind::trajectory_plan
{
namespace
{

/// 下发 GPU 的三路全分辨率深度文件（主/左/右），位于 output/trajectory_planning。
constexpr const char* kDepthFileNames[3] = {"collision_depth_main.f32", "collision_depth_left.f32",
                                            "collision_depth_right.f32"};

} // namespace

class BuildCollisionWorldStage : public StageBase
{
  public:
    bool Init(const StageConfig& config, std::string& error) override
    {
        BindConfig(config);
        if (config.config_root == nullptr || config.flow_config == nullptr ||
            config.device_manager == nullptr)
        {
            error = "缺少碰撞世界依赖";
            return false;
        }
        output_dir_ = config.trajectory_output_dir;

        const nlohmann::json curobo = config.flow_config->value("curobo", nlohmann::json::object());
        world_voxel_size_mm_ = curobo.value("world_voxel_size_mm", 3.0);
        collision_activation_mm_ = curobo.value("collision_activation_mm", 3.0);
        enable_graph_planner_ = curobo.value("enable_graph_planner", false);
        platform_as_voxel_ = curobo.value("platform_as_voxel", true);
        if (!platform_as_voxel_)
        {
            error = "curobo.platform_as_voxel=false 不支持：工作台只以体素并入距离场";
            return false;
        }
        if (collision_activation_mm_ <= 0.0)
        {
            error = "curobo.collision_activation_mm 必须为正";
            return false;
        }
        if (world_voxel_size_mm_ <= 0.0)
        {
            error = "curobo.world_voxel_size_mm 必须为正";
            return false;
        }

        // CAP = 臂展 + 末端长度，仅用于日志核对；截断与定界由 GPU 服务按同一份 app.json 计算。
        const nlohmann::json& app = (*config.config_root)["app"];
        if (!app.contains("robot_reach_mm"))
        {
            error = "app.json 缺少 robot_reach_mm，无法确定点云 XY 截断上限 CAP";
            return false;
        }
        cap_mm_ = app.value("robot_reach_mm", 0.0) +
                  app["end_effector"]["positive_extent_xyz_mm"][2].get<double>();

        if (!planner_.Init(curobo.value("host", std::string("127.0.0.1")),
                           curobo.value("port", 34567),
                           curobo.value("timeout_ms", 120000), error))
        {
            return false;
        }

        LOG_INFO << log_tag_ << "[初始化] GPU 碰撞世界 体素=" << world_voxel_size_mm_
                 << "mm 激活距离=" << collision_activation_mm_ << "mm CAP=" << cap_mm_
                 << "mm 外扩=无 平台=并入体素 图规划="
                 << (enable_graph_planner_ ? "启用" : "未启用（配置关闭）");
        return true;
    }

    bool Execute(PipelineContext& ctx, std::string& error) override
    {
        // ---- 探活：连不上直接判整个 flow 失败，不在无障碍检测下规划 ----
        CuroboPingInfo ping;
        if (!planner_.Ping(ping, error))
        {
            error = "cuRobo 规划服务探活失败: " + error;
            return false;
        }
        LOG_INFO << log_tag_ << "    [1/6] 服务探活=通过 device=" << ping.device
                 << " 本体球=" << ping.body_spheres << " 末端球=" << ping.end_effector_spheres
                 << " 总球数=" << ping.total_spheres
                 << " 最大球半径=" << ping.max_sphere_radius_mm << "mm";
        if (ping.end_effector_spheres <= 0)
        {
            error = "GPU 完整模型缺少末端碰撞球，拒绝在缺失末端保护的模型上规划";
            return false;
        }

        if (!EnsureDirectory(output_dir_, error))
        {
            return false;
        }
        // 三路全分辨率深度图走文件而非 TCP 报文（每路 1280×720 float32 约 3.7MB）。
        // GPU 服务在显存里反投影全部像素；这三个文件不进归档，归档已有原始 tiff（文档 2.5.3）。
        const auto write_started = std::chrono::steady_clock::now();
        CuroboWorldRequest world;
        int64_t pixel_count = 0;
        for (size_t camera = 0; camera < ctx.collision_depths.size(); ++camera)
        {
            const CollisionDepthImage& image = ctx.collision_depths[camera];
            if (image.width <= 0 || image.height <= 0 ||
                image.depth_mm.size() != static_cast<size_t>(image.width) * image.height)
            {
                error = "碰撞世界缺少第 " + std::to_string(camera) + " 路全分辨率深度图";
                return false;
            }
            CuroboDepthImage request;
            request.path = JoinPath(output_dir_, kDepthFileNames[camera]);
            if (!CuroboPlanClient::WriteDepthBinary(image.depth_mm, request.path, error))
            {
                return false;
            }
            request.width = image.width;
            request.height = image.height;
            request.fx = image.fx;
            request.fy = image.fy;
            request.cx = image.cx;
            request.cy = image.cy;
            request.t_base_camera = image.t_base_camera;
            world.depth_images.push_back(request);
            world.depth_min_mm = image.min_depth_mm;
            world.depth_max_mm = image.max_depth_mm;
            pixel_count += static_cast<int64_t>(image.depth_mm.size());
        }
        const double write_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - write_started).count();
        world.voxel_size_mm = world_voxel_size_mm_;
        world.collision_activation_mm = collision_activation_mm_;
        world.enable_graph_planner = enable_graph_planner_;
        world.photo_flange = ctx.photo_flange;
        // world_key 相同时服务端复用已建好的 ESDF，不重建。
        world.world_key = ctx.generation_id;
        const auto started = std::chrono::steady_clock::now();
        if (!planner_.SetWorld(world, ctx.collision_world_meta, error))
        {
            return false;
        }
        const double build_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
        LOG_INFO << log_tag_ << "    [1/6] 深度文件写入=成功 路数=" << world.depth_images.size()
                 << " 像素=" << pixel_count << " 写入耗时=" << write_ms
                 << "ms world_key=" << world.world_key << " 体素=" << world_voxel_size_mm_
                 << "mm dynamic_world=updated gpu_world_update_ms=" << build_ms;
        const nlohmann::json& meta = ctx.collision_world_meta;
        if (!meta.is_null())
        {
            const nlohmann::json truncated = meta.value("truncated", nlohmann::json::object());
            LOG_INFO << log_tag_ << "    [1/6] ① 反投影+截断 有效像素 " << meta.value("cloud_points", 0)
                     << "→" << meta.value("cloud_points_kept", 0) << " CAP=" << meta.value("cap_mm", 0.0)
                     << "mm z_hi=" << meta.value("z_hi_mm", 0.0) << "mm 切掉 |x|>CAP="
                     << truncated.value("abs_x_over_cap", 0)
                     << " |y|>CAP=" << truncated.value("abs_y_over_cap", 0)
                     << " z<底面=" << truncated.value("z_below_floor", 0)
                     << " z>z_hi=" << truncated.value("z_above_z_hi", 0);
            const nlohmann::json lo = meta.value("grid_min_mm", nlohmann::json::array());
            const nlohmann::json hi = meta.value("grid_max_mm", nlohmann::json::array());
            const nlohmann::json lo_src = meta.value("grid_min_source", nlohmann::json::array());
            const nlohmann::json hi_src = meta.value("grid_max_source", nlohmann::json::array());
            LOG_INFO << log_tag_ << "    [1/6] ② 长方体 x[" << lo[0] << ", " << hi[0] << "] y["
                     << lo[1] << ", " << hi[1] << "] z[" << lo[2] << ", " << hi[2]
                     << "] 外扩=无 来源 min=" << lo_src.dump() << " max=" << hi_src.dump();
            LOG_INFO << log_tag_ << "    [1/6] ③ 体素化 网格="
                     << meta.value("grid_shape", nlohmann::json::array()).dump()
                     << " 格数=" << meta.value("voxel_count", 0)
                     << " 占据=" << meta.value("occupied_voxel_count", 0)
                     << "（点云 " << meta.value("occupied_from_cloud", 0)
                     << " + 平台 " << meta.value("occupied_from_platform", 0) << "）";
            const nlohmann::json t = meta.value("timings_ms", nlohmann::json::object());
            LOG_INFO << log_tag_ << "    [1/6] ④ ESDF 完成 耗时 反投影=" << t.value("project_ms", 0.0)
                     << "ms 截断=" << t.value("truncate_ms", 0.0)
                     << "ms 定界=" << t.value("box_ms", 0.0) << "ms 体素化="
                     << t.value("voxelize_ms", 0.0) << "ms ESDF=" << t.value("esdf_ms", 0.0)
                     << "ms 写入规划器=" << t.value("upload_ms", 0.0) << "ms";
        }
        return true;
    }

  private:
    std::string output_dir_;              ///< output/trajectory_planning
    CuroboPlanClient planner_;            ///< cuRobo 服务客户端（启动绑定）
    double world_voxel_size_mm_ = 3.0;    ///< ESDF 体素边长（文档 2.5.6）
    double collision_activation_mm_ = 3.0;///< 激活距离（文档 2.5.9）
    double cap_mm_ = 0.0;                 ///< XY 截断上限 CAP（仅日志）
    bool enable_graph_planner_ = false;   ///< 图规划开关（文档 2.8 ③）
    bool platform_as_voxel_ = true;       ///< 平台并入体素（文档 2.13）
};

REGISTER_STAGE("build_collision_world", BuildCollisionWorldStage);

} // namespace openmind::trajectory_plan
