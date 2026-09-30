/**
 * @file openmind_trajectory_plan_app.cpp
 * @brief 躯干实现：会话日志 → 配置与模型 → pipeline 初始化与预热 → 起 TCP。
 */

#include "trajectory_plan/openmind_trajectory_plan_app.h"

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

#include "trajectory_plan/algorithm/collision_world_build_algorithm.h"
#include "trajectory_plan/algorithm/curobo_plan_client.h"
#include "trajectory_plan/algorithm/trajectory_output_format_algorithm.h"
#include "trajectory_plan/config/config_types.h"
#include "trajectory_plan/tools/interrupt.h"
#include "trajectory_plan/tools/json_util.h"
#include "trajectory_plan/tools/logger_init.hpp"
#include "trajectory_plan/tools/log.h"

namespace openmind::trajectory_plan
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesPerRadian = 180.0 / kPi;

/// 工位校验容差：位置 > 0.1 mm 或姿态 > 0.01° 拒绝。
constexpr double kStationPositionToleranceMm = 0.1;
constexpr double kStationRotationToleranceDeg = 0.01;
constexpr int32_t kCuroboStartupTimeoutMs = 300000;

/** @brief 数值转命令行参数文本（不丢精度、不带多余尾零）。 */
std::string FormatNumber(double value)
{
    std::ostringstream stream;
    stream.precision(10);
    stream << value;
    return stream.str();
}

/**
 * @brief 校验 FK(q) 与配置法兰一致（位置与姿态双容差）。
 *
 * 附录 A：C++ 与 cuRobo 用的是两份独立模型，装错不会报错，只会让终点系统性偏掉。
 * 这里是启动期的第一道交叉验证。
 */
bool VerifyStation(const RobotKinematicsInterface* kinematics,
                   const JointDegrees& q_deg,
                   const Pose& flange_pose,
                   const std::string& name,
                   std::string& error)
{
    JointRadians q {};
    for (size_t index = 0; index < 6; ++index)
    {
        q[index] = q_deg[index] * kPi / 180.0;
    }
    const Eigen::Matrix4d actual = kinematics->ForwardKinematics(q);
    const Eigen::Matrix4d expected = TransformFromPose(flange_pose);
    const double position_error = (actual.block<3, 1>(0, 3) - expected.block<3, 1>(0, 3)).norm();
    const Eigen::Matrix3d relative = expected.block<3, 3>(0, 0).transpose() * actual.block<3, 3>(0, 0);
    const double cosine = std::max(-1.0, std::min(1.0, (relative.trace() - 1.0) * 0.5));
    const double rotation_error = std::acos(cosine) * kDegreesPerRadian;
    if (position_error > kStationPositionToleranceMm || rotation_error > kStationRotationToleranceDeg)
    {
        error = name + " 校验失败: FK(q) 与配置法兰不一致，位置误差 " + std::to_string(position_error) +
                " mm（>0.1），姿态误差 " + std::to_string(rotation_error) + " deg（>0.01）";
        return false;
    }
    return true;
}

} // namespace

bool OpenmindTrajectoryPlanApp::LoadConfig(const std::string& config_path, std::string& error)
{
    return config_service_.Load(config_path, error);
}

void OpenmindTrajectoryPlanApp::OverrideLogRoot(const std::string& log_root)
{
    log_root_override_ = log_root;
}

bool OpenmindTrajectoryPlanApp::Init(std::string& error)
{
    const nlohmann::json& root = config_service_.Root();
    const nlohmann::json& app = root["app"];

    // ---- S0 会话目录：启动立即创建，确保初始化失败也有日志 ----
    // 日志根目录按确认路径使用 /var/hybird_log/grasp_log（注意是 hybird_log）。
    const std::string configured_log_root = app.value("log_root", "/var/hybird_log/grasp_log");
    const std::string log_root = log_root_override_.empty() ? configured_log_root : log_root_override_;
    if (!session_archive_.OpenSession(log_root, error))
    {
        // 会话日志根目录无法创建时不宣布服务就绪，并向启动终端报告错误。
        std::cerr << "[service][启动] 会话日志目录创建失败: " << error << std::endl;
        return false;
    }
    InitGlogInSession(session_archive_.SessionDir(), "OpenmindTrajectoryPlan");
    LOG_INFO << "[service][启动] session_id=" << session_archive_.SessionId()
             << " 会话目录=" << session_archive_.SessionDir();
    if (!log_root_override_.empty() && log_root_override_ != configured_log_root)
    {
        LOG_WARN << "[service][启动] 日志根目录被 --log-root 覆盖：配置=" << configured_log_root
                 << " 实际=" << log_root_override_;
    }
    session_archive_.WriteSessionJson(root, "initializing", "配置加载完成，开始初始化");

    output_root_ = app.value("output_root", std::string("output"));
    grasp_output_dir_ = JoinPath(output_root_, "grasp_generation");
    trajectory_output_dir_ = JoinPath(output_root_, "trajectory_planning");

    auto fail = [this, &error](const std::string& stage) {
        LOG_ERROR << "[service][启动] " << stage << " 失败: " << error;
        session_archive_.WriteSessionJson(config_service_.Root(), "init_failed", stage + ": " + error);
        StopCuroboService();
        return false;
    };

    // ---- S1/S2 CPU 侧：相机、机型运动学与碰撞模型 ----
    if (!camera_manager_.Init(error))
    {
        return fail("相机管理器初始化");
    }
    if (!device_manager_.Init(root["device"], error))
    {
        return fail("设备管理器初始化");
    }

    // 仅放置工位来自固定配置；拍照工位由每轮 request.txt 提供。
    FixedStation place_station;
    if (!FixedStation::Parse(app["place"], place_station, error))
    {
        return fail("工位配置解析");
    }
    if (!VerifyStation(device_manager_.Kinematics(), place_station.q_deg, place_station.flange_pose,
                       "place", error))
    {
        return fail("工位一致性校验");
    }
    LOG_INFO << "[service][启动] place 工位校验通过，photo 工位由 request.txt 提供";

    // ---- flow 配置：grasp.yaml / trajectory.yaml ----
    const std::string flow_dir = JoinPath(config_service_.ConfigDir(), "base/pipeline/flow");
    if (!ReadYamlFile(JoinPath(flow_dir, "grasp.yaml"), grasp_flow_config_, error) ||
        !ReadYamlFile(JoinPath(flow_dir, "trajectory.yaml"), trajectory_flow_config_, error))
    {
        return fail("flow 配置加载");
    }

    if (!StartCuroboService(error))
    {
        return fail("cuRobo GPU 服务启动");
    }

    // ---- S3~S5 pipeline：按顺序创建并初始化启用 flow 的全部 stage ----
    // 标签缓存、带孔平台、GPU 求解器绑定与两条路径预热都发生在这里。
    PipelineStartupResources resources;
    resources.config_root = &config_service_.Root();
    resources.flow_configs["grasp"] = &grasp_flow_config_;
    resources.flow_configs["trajectory"] = &trajectory_flow_config_;
    resources.kinematics = device_manager_.Kinematics();
    resources.device_manager = &device_manager_;
    resources.camera_manager = &camera_manager_;
    resources.grasp_output_dir = grasp_output_dir_;
    resources.trajectory_output_dir = trajectory_output_dir_;

    const std::vector<std::string> enabled_flows = {"grasp", "trajectory"};
    LOG_INFO << "[pipeline][startup] session_id=" << session_archive_.SessionId()
             << " enabled_flows=[grasp, trajectory]";
    if (!pipeline_.Init(root["pipeline"], resources, enabled_flows, error))
    {
        return fail("pipeline 初始化");
    }

    // ---- 固定模型快照：供第三流程使用，与规划同源 ----
    if (!ExportCollisionModelSnapshot(error))
    {
        return fail("固定模型快照导出");
    }

    if (!trigger_router_.Init(&pipeline_, &session_archive_, output_root_, grasp_output_dir_,
                              trajectory_output_dir_, collision_model_id_, collision_model_sha256_,
                              error))
    {
        return fail("触发路由初始化");
    }

    initialized_ = true;
    return true;
}

bool OpenmindTrajectoryPlanApp::StartCuroboService(std::string& error)
{
    const nlohmann::json& root = config_service_.Root();
    const nlohmann::json& app = root["app"];
    const std::string python = root.value("curobo_python", std::string());
    if (python.empty())
    {
        error = "启动配置缺少 curobo_python";
        return false;
    }
    const nlohmann::json curobo =
        trajectory_flow_config_.value("curobo", nlohmann::json::object());
    const std::string host = curobo.value("host", std::string("127.0.0.1"));
    const std::string port = std::to_string(curobo.value("port", 34567));
    const std::string script = "services/curobo_planner/curobo_plan_service.py";

    // ---- 决定规划器构造的参数：启动时传入，运行期不可改（文档 0.4）----
    const double activation_mm = curobo.value("collision_activation_mm", 3.0);
    const double voxel_mm = curobo.value("world_voxel_size_mm", 3.0);
    const bool enable_graph = curobo.value("enable_graph_planner", false);
    const double z_hi_max_mm = curobo.value("world_cache_z_hi_max_mm", 600.0);
    if (!app.contains("robot_reach_mm"))
    {
        error = "app.json 缺少 robot_reach_mm，无法确定体素缓存尺寸";
        return false;
    }
    // 体素缓存按最大可能长方体预分配：X、Y 各 2×CAP，Z 从工作台底面到 z_hi 上限。
    const double cap_mm = app.value("robot_reach_mm", 0.0) +
                          app["end_effector"]["positive_extent_xyz_mm"][2].get<double>();
    const double floor_z_mm = -app["base_platform"]["negative_extent_xyz_mm"][2].get<double>();
    const double cache_dims_mm[3] = {2.0 * cap_mm, 2.0 * cap_mm, z_hi_max_mm - floor_z_mm};
    if (cache_dims_mm[2] <= 0.0)
    {
        error = "curobo.world_cache_z_hi_max_mm 必须高于工作台底面";
        return false;
    }

    std::vector<std::string> arguments = {python, script, "--host", host, "--port", port,
                                          "--activation-mm", FormatNumber(activation_mm),
                                          "--cache-voxel-size-mm", FormatNumber(voxel_mm),
                                          "--cache-dims-mm", FormatNumber(cache_dims_mm[0]),
                                          FormatNumber(cache_dims_mm[1]),
                                          FormatNumber(cache_dims_mm[2])};
    if (enable_graph)
    {
        arguments.push_back("--enable-graph-planner");
    }
    arguments.push_back("--warmup-on-start");

    // 子进程 stdout/stderr 落到会话目录（文档 0.5）。
    const std::string child_log = JoinPath(session_archive_.SessionDir(), "curobo_service.log");
    const int log_fd = open(child_log.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
    if (log_fd < 0)
    {
        error = "无法创建 " + child_log + ": " + std::strerror(errno);
        return false;
    }

    const pid_t pid = fork();
    if (pid < 0)
    {
        close(log_fd);
        error = std::string("创建 cuRobo 子进程失败: ") + std::strerror(errno);
        return false;
    }
    if (pid == 0)
    {
        dup2(log_fd, STDOUT_FILENO);
        dup2(log_fd, STDERR_FILENO);
        std::vector<char*> argv;
        for (std::string& argument : arguments)
        {
            argv.push_back(argument.data());
        }
        argv.push_back(nullptr);
        execv(python.c_str(), argv.data());
        std::cerr << "启动 cuRobo Python 失败: " << std::strerror(errno) << std::endl;
        _exit(127);
    }
    close(log_fd);
    curobo_pid_ = pid;
    LOG_INFO << "[service][启动] cuRobo 子进程已启动 pid=" << curobo_pid_
             << " endpoint=" << host << ":" << port << " 激活距离=" << activation_mm
             << "mm 体素缓存=" << voxel_mm << "mm×[" << cache_dims_mm[0] << ", " << cache_dims_mm[1]
             << ", " << cache_dims_mm[2] << "]mm 图规划=" << (enable_graph ? "启用" : "关闭")
             << " 日志=" << child_log;

    CuroboPlanClient planner;
    if (!planner.Init(host, std::stoi(port), curobo.value("timeout_ms", 120000), error))
    {
        StopCuroboService();
        return false;
    }
    const auto started = std::chrono::steady_clock::now();
    const auto deadline = started + std::chrono::milliseconds(kCuroboStartupTimeoutMs);
    while (std::chrono::steady_clock::now() < deadline)
    {
        int status = 0;
        const pid_t state = waitpid(curobo_pid_, &status, WNOHANG);
        if (state == curobo_pid_)
        {
            curobo_pid_ = -1;
            error = "cuRobo 子进程启动期间退出，状态=" + std::to_string(status) + "，详见 " +
                    child_log;
            return false;
        }
        CuroboPingInfo ping;
        std::string ping_error;
        if (planner.Ping(ping, ping_error))
        {
            // 启动后核对：子进程实际构造参数必须与 trajectory.yaml 一致（文档 0.4）。
            const nlohmann::json& applied = ping.applied;
            std::string mismatch;
            if (!applied.is_object())
            {
                mismatch = "applied 缺失";
            }
            else if (std::fabs(applied.value("collision_activation_mm", -1.0) - activation_mm) > 1e-9)
            {
                mismatch = "collision_activation_mm 期望=" + FormatNumber(activation_mm) + " 实际=" +
                           applied["collision_activation_mm"].dump();
            }
            else if (applied.value("enable_graph_planner", !enable_graph) != enable_graph)
            {
                mismatch = std::string("enable_graph_planner 期望=") + (enable_graph ? "true" : "false") +
                           " 实际=" + applied["enable_graph_planner"].dump();
            }
            else if (std::fabs(applied.value("cache_voxel_size_mm", -1.0) - voxel_mm) > 1e-9)
            {
                mismatch = "world_voxel_size_mm 期望=" + FormatNumber(voxel_mm) + " 实际=" +
                           applied["cache_voxel_size_mm"].dump();
            }
            if (!mismatch.empty())
            {
                error = "cuRobo 启动参数与配置不一致: " + mismatch;
                StopCuroboService();
                return false;
            }
            const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                          std::chrono::steady_clock::now() - started).count();
            LOG_INFO << "[service][启动] cuRobo 已就绪 gpu=" << ping.device
                     << " 本体球=" << ping.body_spheres << " 末端球=" << ping.end_effector_spheres
                     << " 总球数=" << ping.total_spheres << " 体素缓存格="
                     << applied.value("cache_shape", nlohmann::json::array()).dump()
                     << " 参数核对=一致 耗时=" << elapsed_ms << "ms";
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    error = "等待 cuRobo 服务就绪超时，详见 " + child_log;
    StopCuroboService();
    return false;
}

void OpenmindTrajectoryPlanApp::StopCuroboService()
{
    if (curobo_pid_ <= 0)
    {
        return;
    }
    LOG_INFO << "[service][退出] 停止 cuRobo 子进程 pid=" << curobo_pid_;
    if (kill(curobo_pid_, SIGTERM) != 0 && errno != ESRCH)
    {
        LOG_WARN << "[service][退出] 向 cuRobo 子进程发送 SIGTERM 失败: " << std::strerror(errno);
    }
    int status = 0;
    while (waitpid(curobo_pid_, &status, 0) < 0 && errno == EINTR)
    {
    }
    curobo_pid_ = -1;
}

bool OpenmindTrajectoryPlanApp::ExportCollisionModelSnapshot(std::string& error)
{
    const nlohmann::json& app = config_service_.Root()["app"];
    BoxConfig platform;
    BoxConfig end_effector;
    if (!BoxConfig::Parse(app["base_platform"], platform, error) ||
        !BoxConfig::Parse(app["end_effector"], end_effector, error))
    {
        return false;
    }
    nlohmann::json document;
    // 改动十：升级为 v2 —— 新增末端球的完整球心与半径、末端外凸量、激活距离，
    // 使可视化只从这一份快照读碰撞球，不再自己去读 aubo.json 另拼一套。
    document["schema"] = "collision_model/v2";
    document["units"] = {{"length", "mm"}, {"angle", "deg"}};
    document["robot_type"] = config_service_.Root()["device"].value("robot_type", std::string());
    document["base_frame"] = "base_link";

    // 实际碰撞球：连杆局部球心与半径，随所属连杆运动。
    nlohmann::json spheres = nlohmann::json::array();
    for (const RobotCollisionSphere& sphere : device_manager_.CollisionSpheres())
    {
        spheres.push_back({{"link_index", sphere.link_index},
                           {"center_mm", {sphere.center.x(), sphere.center.y(), sphere.center.z()}},
                           {"radius_mm", sphere.radius}});
    }
    document["body_spheres"] = spheres;
    document["body_sphere_count"] = spheres.size();

    // 末端球由 GPU 服务按同一份 end_effector 几何生成。改动十：这里取回**完整的
    // 球心与半径**（不再只取数量），写进快照供可视化直接绘制。
    int64_t end_effector_sphere_count = 0;
    nlohmann::json end_effector_spheres = nlohmann::json::array();
    double end_effector_bulge_mm = 0.0;
    {
        const nlohmann::json curobo =
            trajectory_flow_config_.value("curobo", nlohmann::json::object());
        CuroboPlanClient planner;
        std::string planner_error;
        if (planner.Init(curobo.value("host", std::string("127.0.0.1")),
                         curobo.value("port", 34567), curobo.value("timeout_ms", 120000),
                         planner_error))
        {
            CuroboPingInfo ping;
            if (!planner.Ping(ping, planner_error))
            {
                error = "读取 GPU 完整模型碰撞球数失败: " + planner_error;
                return false;
            }
            end_effector_sphere_count = ping.end_effector_spheres;
            if (ping.end_effector_detail.is_object())
            {
                end_effector_spheres =
                    ping.end_effector_detail.value("spheres", nlohmann::json::array());
                end_effector_bulge_mm = ping.end_effector_detail.value("bulge_mm", 0.0);
            }
            if (ping.body_spheres != static_cast<int64_t>(spheres.size()))
            {
                // 附录 A 的交叉验证：两份模型的本体球数必须一致，装错不会报错。
                LOG_WARN << "[service][启动] 本体碰撞球数不一致：C++=" << spheres.size()
                         << " GPU=" << ping.body_spheres << "，请核对是否装了同一机型";
            }
        }
    }
    document["end_effector_spheres"] = end_effector_spheres;
    document["end_effector_sphere_count"] = end_effector_sphere_count;
    document["end_effector_sphere_frame"] =
        app["end_effector"].value("parent_frame", std::string("flange_link"));
    document["end_effector_sphere_bulge_mm"] = end_effector_bulge_mm;
    document["total_sphere_count"] =
        static_cast<int64_t>(spheres.size()) + end_effector_sphere_count;
    // 改动九：激活距离是整条碰撞链上唯一的安全余量参数，写进快照使历史归档可回溯。
    document["collision_activation_mm"] =
        trajectory_flow_config_.value("curobo", nlohmann::json::object())
            .value("collision_activation_mm", 3.0);

    // 末端刚体：挂载坐标系与几何；末端球由 GPU 服务按同一几何生成。
    document["end_effector"] = {
        {"parent_frame", app["end_effector"].value("parent_frame", std::string("flange_link"))},
        {"geometry_type", "box"},
        {"negative_extent_xyz_mm",
         {end_effector.negative_extent.x(), end_effector.negative_extent.y(),
          end_effector.negative_extent.z()}},
        {"positive_extent_xyz_mm",
         {end_effector.positive_extent.x(), end_effector.positive_extent.y(),
          end_effector.positive_extent.z()}},
        // 末端棱线布球（文档 2.5.8）：球心在 8 条棱上，外凸 = 半径。
        {"sphere_layout", app["end_effector"].value("sphere_layout", std::string("edges"))},
        {"sphere_radius_mm", app["end_effector"].value("sphere_radius_mm", 0.0)},
    };

    // 固定平台：改动十一起按**实心**处理并由 GPU 服务栅格化并入体素距离场，
    // 不再分解为长方体，也不再有圆柱豁免区。
    document["base_platform"] = {
        {"negative_extent_xyz_mm",
         {platform.negative_extent.x(), platform.negative_extent.y(), platform.negative_extent.z()}},
        {"positive_extent_xyz_mm",
         {platform.positive_extent.x(), platform.positive_extent.y(), platform.positive_extent.z()}},
        {"representation", "voxel"},
    };
    // 安装处圆柱豁免区：改动十一取消的是长方体**分解**，孔本身以体素形式保留
    // （见该改动的实施更正）。可视化据此显示有限深度的圆柱孔。
    {
        const nlohmann::json node =
            app["base_platform"].value("installation_exclusion", nlohmann::json::object());
        const bool enabled = node.value("enabled", false);
        const double depth_mm = node.value("depth_mm", 0.0);
        const double top_z_mm = platform.positive_extent.z();
        document["installation_exclusion"] = {
            {"enabled", enabled},
            {"geometry_type", node.value("geometry_type", std::string("cylinder"))},
            {"center_xy_mm", node.value("center_xy_mm", std::vector<double>{0.0, 0.0})},
            {"diameter_mm", node.value("diameter_mm", 0.0)},
            {"depth_mm", depth_mm},
            {"z_range_mm", {top_z_mm - depth_mm, top_z_mm}},
        };
    }
    document["camera_clearance_along_tcp_z_mm"] = app.value("camera_clearance_along_tcp_z_mm", -84.0);
    document["photo"] = app["photo"];
    document["place"] = app["place"];

    const std::string text = document.dump(2) + "\n";
    collision_model_sha256_ = TrajectoryOutputFormatAlgorithm::Sha256Hex(text);
    // 模型标识覆盖运动学/标定与固定几何版本，不能只用机型名称判断一致。
    collision_model_id_ = document["robot_type"].get<std::string>() + "@" +
                          collision_model_sha256_.substr(0, 16);

    nlohmann::json final_document = document;
    final_document["model_id"] = collision_model_id_;
    final_document["snapshot_sha256"] = collision_model_sha256_;

    const std::string directory = JoinPath(output_root_, "visualization");
    if (!EnsureDirectory(directory, error))
    {
        return false;
    }
    // 原子导出：可视化读取时不会看到半写文件。
    if (!WriteTextFileAtomic(JoinPath(directory, "collision_model.json"),
                             final_document.dump(2) + "\n", error))
    {
        return false;
    }
    LOG_INFO << "[service][启动] 固定模型快照=已导出 model_id=" << collision_model_id_
             << " 本体球=" << spheres.size() << " 末端球=" << end_effector_sphere_count
             << " 总球数=" << (static_cast<int64_t>(spheres.size()) + end_effector_sphere_count)
             << " 末端外凸=" << end_effector_bulge_mm << "mm 平台=并入体素";
    return true;
}

bool OpenmindTrajectoryPlanApp::Run(std::string& error)
{
    if (!initialized_)
    {
        error = "尚未 Init";
        return false;
    }
    const nlohmann::json& app = config_service_.Root()["app"];
    const nlohmann::json trigger = app.value("trigger_tcp", nlohmann::json::object());
    const std::string host = trigger.value("bind_host", std::string("0.0.0.0"));
    const uint16_t port = static_cast<uint16_t>(trigger.value("port", 9108));

    TcpServerLimits limits;
    limits.protocol_magic = trigger.value("protocol_magic", std::string("GP02"));
    limits.max_request_text_bytes = trigger.value("max_request_text_bytes", 1048576ull);
    limits.max_depth_image_bytes = trigger.value("max_depth_image_bytes", 134217728ull);
    limits.max_total_frame_bytes = trigger.value("max_total_frame_bytes", 536870912ull);
    limits.receive_timeout_ms = trigger.value("receive_timeout_ms", 30000);

    if (!tcp_server_.Start(host, port, limits,
                           [this](const TriggerFrame& frame, const ResponseSender& sender) {
                               trigger_router_.HandleFrame(frame, sender);
                           },
                           error))
    {
        LOG_ERROR << "[service][启动] TCP 服务启动失败: " << error;
        session_archive_.WriteSessionJson(config_service_.Root(), "init_failed",
                                          "TCP 服务启动失败: " + error);
        return false;
    }

    // 所有必要初始化和预热成功后才宣布就绪，开始接受规划任务。
    trigger_router_.SetServiceReady(true);
    LOG_INFO << "[pipeline][startup] service_ready=true address=" << host << ":" << port;
    session_archive_.WriteSessionJson(config_service_.Root(), "ready",
                                      "监听 " + host + ":" + std::to_string(port));

    WaitForInterrupt();
    LOG_INFO << "[service][退出] 收到中断，停止服务";
    Stop();
    session_archive_.WriteSessionJson(config_service_.Root(), "exited", "收到中断信号，正常退出");
    return true;
}

void OpenmindTrajectoryPlanApp::Stop()
{
    trigger_router_.SetServiceReady(false);
    tcp_server_.Stop();
    StopCuroboService();
}

bool OpenmindTrajectoryPlanApp::RunOnce(const std::string& input_dir, std::string& error)
{
    if (!initialized_)
    {
        error = "尚未 Init";
        return false;
    }
    // 回放入口：按客户端的做法把文字和三图全部读入内存后再交给同一条处理链路。
    TriggerFrame frame;
    frame.connection_id = "local_once";
    frame.peer = input_dir;
    if (!ReadTextFile(JoinPath(input_dir, "request.txt"), frame.request_text, error))
    {
        return false;
    }
    std::string workpiece_type;
    Eigen::Matrix4d t_b_o = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d t_base_flange = Eigen::Matrix4d::Identity();
    std::array<double, 6> t_base_flange_joint_angles_deg {};
    std::string main_rel;
    std::string left_rel;
    std::string right_rel;
    std::string rgb_main_rel;
    if (!ParseRequestText(frame.request_text, workpiece_type, t_b_o, t_base_flange,
                          t_base_flange_joint_angles_deg, main_rel, left_rel, right_rel, rgb_main_rel, error))
    {
        return false;
    }
    const std::string paths[4] = {JoinPath(input_dir, main_rel), JoinPath(input_dir, left_rel),
                                  JoinPath(input_dir, right_rel), JoinPath(input_dir, rgb_main_rel)};
    std::vector<uint8_t>* buffers[4] = {&frame.depth_main, &frame.depth_left, &frame.depth_right,
                                        &frame.rgb_main};
    for (int32_t index = 0; index < 4; ++index)
    {
        std::string bytes;
        if (!ReadTextFile(paths[index], bytes, error))
        {
            return false;
        }
        buffers[index]->assign(bytes.begin(), bytes.end());
    }
    trigger_router_.SetServiceReady(true);
    LOG_INFO << "[service][--once] workpiece_type=" << workpiece_type << " input=" << input_dir;
    std::string reply;
    const bool ok = trigger_router_.RunOnce(frame, reply);
    LOG_INFO << "[service][--once] reply=" << reply;
    std::cout << reply << std::endl;
    if (!ok)
    {
        error = "本轮任务未成功完成: " + reply;
    }
    return ok;
}

void OpenmindTrajectoryPlanApp::DumpConfig() const
{
    // --dump-config：合并后的完整配置树，供现场核对与脚本读取（CLI 工具通道，非业务日志）。
    std::cout << config_service_.Root().dump(2) << std::endl;
}

} // namespace openmind::trajectory_plan
