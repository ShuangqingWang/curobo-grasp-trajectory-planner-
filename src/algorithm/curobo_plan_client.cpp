/**
 * @file curobo_plan_client.cpp
 * @brief cuRobo 规划服务客户端：本地 TCP + 按行分隔 JSON。
 */

#include "trajectory_plan/algorithm/curobo_plan_client.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>

#include <nlohmann/json.hpp>

namespace openmind::trajectory_plan
{
namespace
{

/** @brief RAII 套接字，确保任何返回路径都关闭 fd。 */
class ScopedSocket
{
  public:
    explicit ScopedSocket(int fd) : fd_(fd) {}
    ~ScopedSocket()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
        }
    }
    ScopedSocket(const ScopedSocket&) = delete;
    ScopedSocket& operator=(const ScopedSocket&) = delete;
    int Get() const { return fd_; }

  private:
    int fd_ = -1;
};

} // namespace

bool CuroboPlanClient::Init(const std::string& host, int32_t port, int32_t timeout_ms, std::string& error)
{
    if (host.empty() || port <= 0 || port > 65535 || timeout_ms <= 0)
    {
        error = "CuroboPlanClient 参数非法";
        return false;
    }
    host_ = host;
    port_ = port;
    timeout_ms_ = timeout_ms;
    return true;
}

bool CuroboPlanClient::Call(const std::string& request_line, std::string& response_line, std::string& error) const
{
    ScopedSocket socket_guard(::socket(AF_INET, SOCK_STREAM, 0));
    const int fd = socket_guard.Get();
    if (fd < 0)
    {
        error = std::string("创建套接字失败: ") + std::strerror(errno);
        return false;
    }
    timeval timeout {};
    timeout.tv_sec = timeout_ms_ / 1000;
    timeout.tv_usec = (timeout_ms_ % 1000) * 1000;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    const int nodelay = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    sockaddr_in address {};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port_));
    if (::inet_pton(AF_INET, host_.c_str(), &address.sin_addr) != 1)
    {
        error = "非法的服务地址: " + host_;
        return false;
    }
    if (::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
    {
        error = "连接 cuRobo 规划服务失败(" + host_ + ":" + std::to_string(port_) + "): " + std::strerror(errno);
        return false;
    }

    std::string payload = request_line;
    payload.push_back('\n');
    size_t sent = 0;
    while (sent < payload.size())
    {
        const ssize_t written = ::send(fd, payload.data() + sent, payload.size() - sent, MSG_NOSIGNAL);
        if (written <= 0)
        {
            error = std::string("发送请求失败: ") + std::strerror(errno);
            return false;
        }
        sent += static_cast<size_t>(written);
    }

    // 应答是一整行 JSON，轨迹点多时可达数百 KB，必须读到换行为止。
    response_line.clear();
    std::vector<char> buffer(65536);
    while (true)
    {
        const ssize_t received = ::recv(fd, buffer.data(), buffer.size(), 0);
        if (received < 0)
        {
            error = std::string("接收应答失败: ") + std::strerror(errno);
            return false;
        }
        if (received == 0)
        {
            error = "cuRobo 规划服务提前关闭连接";
            return false;
        }
        response_line.append(buffer.data(), static_cast<size_t>(received));
        const size_t newline = response_line.find('\n');
        if (newline != std::string::npos)
        {
            response_line.resize(newline);
            return true;
        }
    }
}

bool CuroboPlanClient::Ping(CuroboPingInfo& info, std::string& error) const
{
    info = CuroboPingInfo{};
    nlohmann::json request;
    request["cmd"] = "ping";
    std::string response_line;
    if (!Call(request.dump(), response_line, error))
    {
        return false;
    }
    const nlohmann::json response = nlohmann::json::parse(response_line, nullptr, false);
    if (response.is_discarded() || !response.value("ok", false))
    {
        error = "ping 失败: " + response_line;
        return false;
    }
    info.pid = response.value("pid", static_cast<int64_t>(0));
    info.device = response.value("device", std::string());
    info.body_spheres = response.value("body_spheres", 0);
    info.end_effector_spheres = response.value("end_effector_spheres", 0);
    info.total_spheres = response.value("total_spheres",
                                        info.body_spheres + info.end_effector_spheres);
    info.max_sphere_radius_mm = response.value("max_sphere_radius_mm", 0.0);
    info.end_effector_detail = response.value("end_effector_sphere_detail", nlohmann::json());
    info.applied = response.value("applied", nlohmann::json());
    return true;
}

bool CuroboPlanClient::SetWorld(const CuroboWorldRequest& request, nlohmann::json& meta,
                                std::string& error) const
{
    meta = nlohmann::json();
    nlohmann::json document;
    document["cmd"] = "set_world";
    nlohmann::json depth_images = nlohmann::json::array();
    for (const CuroboDepthImage& image : request.depth_images)
    {
        nlohmann::json pose = nlohmann::json::array();
        for (int32_t row = 0; row < 4; ++row)
        {
            nlohmann::json values = nlohmann::json::array();
            for (int32_t column = 0; column < 4; ++column)
            {
                values.push_back(image.t_base_camera(row, column));
            }
            pose.push_back(values);
        }
        depth_images.push_back({{"path", image.path},
                                {"width", image.width},
                                {"height", image.height},
                                {"fx", image.fx},
                                {"fy", image.fy},
                                {"cx", image.cx},
                                {"cy", image.cy},
                                {"T_base_camera", pose}});
    }
    document["depth_images"] = depth_images;
    document["depth_min_mm"] = request.depth_min_mm;
    document["depth_max_mm"] = request.depth_max_mm;
    document["voxel_size_mm"] = request.voxel_size_mm;
    document["collision_activation_mm"] = request.collision_activation_mm;
    document["enable_graph_planner"] = request.enable_graph_planner;
    document["world_key"] = request.world_key;
    nlohmann::json photo_flange = nlohmann::json::array();
    for (int32_t row = 0; row < 4; ++row)
    {
        nlohmann::json values = nlohmann::json::array();
        for (int32_t column = 0; column < 4; ++column)
        {
            values.push_back(request.photo_flange(row, column));
        }
        photo_flange.push_back(values);
    }
    document["T_base_flange"] = photo_flange;
    std::string response_line;
    if (!Call(document.dump(), response_line, error))
    {
        return false;
    }
    const nlohmann::json response = nlohmann::json::parse(response_line, nullptr, false);
    if (response.is_discarded() || !response.value("ok", false))
    {
        error = "下发碰撞世界失败: " +
                (response.is_discarded() ? response_line : response.value("error", response_line));
        return false;
    }
    // 复用已建好的 ESDF 时服务端不回 collision_world，保持 meta 为 null，
    // 由调用方沿用上一轮（同一 generation_id 本就是同一份世界）。
    if (response.contains("collision_world"))
    {
        meta = response["collision_world"];
    }
    return true;
}

namespace
{

/** @brief 从服务端 steps 节填充统计结构；字段缺失时保持默认值。 */
void FillSteps(const nlohmann::json& node, CuroboPlanSteps& steps)
{
    if (!node.is_object())
    {
        return;
    }
    if (node.contains("ik"))
    {
        const nlohmann::json& ik = node["ik"];
        steps.ik_seeds = ik.value("seeds", 0);
        steps.ik_returned = ik.value("returned", 0);
        steps.ik_converged = ik.value("converged", 0);
        steps.ik_ms = ik.value("ms", 0.0);
    }
    if (node.contains("pad"))
    {
        steps.padded = node["pad"].value("padded", 0);
    }
    if (node.contains("graph"))
    {
        const nlohmann::json& graph = node["graph"];
        steps.graph_ok = graph.value("ok", false);
        steps.graph_skipped = graph.value("skipped", false);
        steps.graph_ms = graph.value("ms", 0.0);
    }
    if (node.contains("trajopt"))
    {
        const nlohmann::json& trajopt = node["trajopt"];
        steps.trajopt_requested = trajopt.value("requested", 0);
        steps.trajopt_converged = trajopt.value("converged", 0);
        steps.trajopt_ms = trajopt.value("ms", 0.0);
        steps.position_error_mm = trajopt.value("position_error_mm", 0.0);
        steps.rotation_error_deg = trajopt.value("rotation_error_deg", 0.0);
    }
    if (node.contains("interp"))
    {
        steps.interp_count = node["interp"].value("count", 0);
    }
}

/**
 * @brief 解析应答的 trajectories 数组；位姿目标与关节目标共用同一布局。
 */
bool ParseTrajectories(const nlohmann::json& response,
                       std::vector<JointTrajectory>& trajectories,
                       std::string& error)
{
    if (!response.contains("trajectories") || !response["trajectories"].is_array() ||
        response["trajectories"].empty())
    {
        error = "服务返回了空的候选轨迹列表";
        return false;
    }
    const nlohmann::json& list = response["trajectories"];
    trajectories.reserve(list.size());
    for (const nlohmann::json& item : list)
    {
        const nlohmann::json& points = item["points"];
        if (!points.is_array() || points.empty())
        {
            error = "候选轨迹为空";
            return false;
        }
        JointTrajectory trajectory;
        trajectory.time_step_s = item.value("dt", 0.0);  // 缺字段时为 0，由调用方按配置核对后拒收
        trajectory.points.reserve(points.size());
        for (const nlohmann::json& row : points)
        {
            if (!row.is_array() || row.size() != 6)
            {
                error = "轨迹点不是 6 个关节角";
                return false;
            }
            JointRadians q {};
            for (int64_t index = 0; index < 6; ++index)
            {
                q[static_cast<size_t>(index)] = row[static_cast<size_t>(index)].get<double>();
            }
            trajectory.points.push_back(q);
        }
        trajectories.push_back(std::move(trajectory));
    }
    return true;
}

} // namespace

bool CuroboPlanClient::Plan(const JointRadians& q_start,
                            const Eigen::Matrix4d& goal_flange,
                            int64_t return_trajectories,
                            std::vector<JointTrajectory>& trajectories,
                            CuroboPlanSteps& steps,
                            std::string& error) const
{
    trajectories.clear();
    steps = CuroboPlanSteps();

    nlohmann::json document;
    document["cmd"] = "plan";
    nlohmann::json start = nlohmann::json::array();
    for (int64_t index = 0; index < 6; ++index)
    {
        start.push_back(q_start[static_cast<size_t>(index)]);
    }
    document["q_start_rad"] = start;
    nlohmann::json goal = nlohmann::json::array();
    for (int32_t row = 0; row < 4; ++row)
    {
        goal.push_back({goal_flange(row, 0), goal_flange(row, 1), goal_flange(row, 2), goal_flange(row, 3)});
    }
    document["T_goal_mm"] = goal;
    document["return_trajectories"] = return_trajectories;

    std::string response_line;
    if (!Call(document.dump(), response_line, error))
    {
        return false;
    }
    const nlohmann::json response = nlohmann::json::parse(response_line, nullptr, false);
    if (response.is_discarded())
    {
        error = "应答不是合法 JSON";
        return false;
    }
    FillSteps(response.value("steps", nlohmann::json::object()), steps);
    steps.total_ms = response.value("total_ms", 0.0);
    if (!response.value("ok", false))
    {
        steps.failed_step = response.value("failed_step", std::string());
        error = response.value("error", std::string("规划失败"));
        return false;
    }
    return ParseTrajectories(response, trajectories, error);
}

bool CuroboPlanClient::PlanJoint(const JointRadians& q_start,
                                 const JointRadians& q_goal,
                                 int64_t return_trajectories,
                                 std::vector<JointTrajectory>& trajectories,
                                 CuroboPlanSteps& steps,
                                 std::string& error) const
{
    trajectories.clear();
    steps = CuroboPlanSteps();

    nlohmann::json document;
    document["cmd"] = "plan_joint";
    nlohmann::json start = nlohmann::json::array();
    nlohmann::json goal = nlohmann::json::array();
    for (int64_t index = 0; index < 6; ++index)
    {
        start.push_back(q_start[static_cast<size_t>(index)]);
        goal.push_back(q_goal[static_cast<size_t>(index)]);
    }
    document["q_start_rad"] = start;
    document["q_goal_rad"] = goal;
    document["return_trajectories"] = return_trajectories;

    std::string response_line;
    if (!Call(document.dump(), response_line, error))
    {
        return false;
    }
    const nlohmann::json response = nlohmann::json::parse(response_line, nullptr, false);
    if (response.is_discarded())
    {
        error = "应答不是合法 JSON";
        return false;
    }
    FillSteps(response.value("steps", nlohmann::json::object()), steps);
    steps.total_ms = response.value("total_ms", 0.0);
    if (!response.value("ok", false))
    {
        steps.failed_step = response.value("failed_step", std::string());
        error = response.value("error", std::string("关节目标规划失败"));
        return false;
    }
    return ParseTrajectories(response, trajectories, error);
}

bool CuroboPlanClient::WriteDepthBinary(const std::vector<float>& depth_mm, const std::string& path,
                                        std::string& error)
{
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (!stream.is_open())
    {
        error = "无法写入深度文件: " + path;
        return false;
    }
    stream.write(reinterpret_cast<const char*>(depth_mm.data()),
                 static_cast<std::streamsize>(depth_mm.size() * sizeof(float)));
    if (!stream.good())
    {
        error = "深度文件写入失败: " + path;
        return false;
    }
    return true;
}

bool CuroboPlanClient::WriteCloudBinary(const std::vector<Eigen::Vector3d>& cloud,
                                        const std::string& path,
                                        std::string& error)
{
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (!stream.is_open())
    {
        error = "无法写入点云文件: " + path;
        return false;
    }
    std::vector<float> flat;
    flat.reserve(cloud.size() * 3);
    for (const Eigen::Vector3d& point : cloud)
    {
        flat.push_back(static_cast<float>(point.x()));
        flat.push_back(static_cast<float>(point.y()));
        flat.push_back(static_cast<float>(point.z()));
    }
    stream.write(reinterpret_cast<const char*>(flat.data()),
                 static_cast<std::streamsize>(flat.size() * sizeof(float)));
    if (!stream.good())
    {
        error = "点云文件写入失败: " + path;
        return false;
    }
    return true;
}

} // namespace openmind::trajectory_plan
