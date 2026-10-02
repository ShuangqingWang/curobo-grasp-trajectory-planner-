#ifndef HYBRID_TRAJECTORY_PLAN_CUROBO_PLAN_CLIENT_H
#define HYBRID_TRAJECTORY_PLAN_CUROBO_PLAN_CLIENT_H

/**
 * @file curobo_plan_client.h
 * @brief cuRobo GPU 规划服务的客户端：给起点关节角和目标法兰位姿，取回一条轨迹。
 *
 * 服务端见 services/curobo_planner/curobo_plan_service.py。协议是本地 TCP +
 * 按行分隔的 JSON。本类只负责通信与单位转换，不做任何规划决策：
 * 抓取候选排序、A/B/C 评级、发布策略仍然留在 stage 里。
 *
 * 单位：关节角 rad，平移 mm（与工程其余部分一致，服务端内部转成米）。
 */

#include <Eigen/Core>
#include <nlohmann/json.hpp>

#include <cstdint>
#include <string>
#include <vector>

#include "trajectory_plan/algorithm/joint_trajectory.h"
#include "trajectory_plan/algorithm/robot_kinematics_interface.h"

namespace openmind::trajectory_plan
{

/**
 * @brief 碰撞世界下发参数。
 */
/**
 * @brief 一路全分辨率深度图的下发描述（文件 + 相机参数）。
 */
struct CuroboDepthImage
{
    std::string path;            ///< 深度文件路径（float32，行主序，mm）
    int32_t width = 0;           ///< 宽，pixel
    int32_t height = 0;          ///< 高，pixel
    double fx = 0.0;             ///< 焦距 x，pixel
    double fy = 0.0;             ///< 焦距 y，pixel
    double cx = 0.0;             ///< 主点 x，pixel
    double cy = 0.0;             ///< 主点 y，pixel
    Eigen::Matrix4d t_base_camera = Eigen::Matrix4d::Identity(); ///< 相机 → 基座（mm）
};

struct CuroboWorldRequest
{
    /// 三路全分辨率深度图（主/左/右）。GPU 服务在显存里反投影全部像素（文档 2.5.3）。
    std::vector<CuroboDepthImage> depth_images;
    double depth_min_mm = 150.0;  ///< 有效深度下界
    double depth_max_mm = 2500.0; ///< 有效深度上界
    double voxel_size_mm = 3.0;  ///< ESDF 体素边长（mm，文档 2.5.6）
    /// 激活距离（mm，文档 2.5.9）。须与 GPU 子进程启动参数一致，服务端核对。
    double collision_activation_mm = 3.0;
    /// 图规划开关（文档 2.8 ③）。须与 GPU 子进程启动参数一致，服务端核对。
    bool enable_graph_planner = false;
    std::string world_key;          ///< 世界指纹；与上次相同则服务端复用，不重建
    Eigen::Matrix4d photo_flange = Eigen::Matrix4d::Identity(); ///< 本轮拍照法兰相对基座（mm）
};

/**
 * @brief 探活应答：GPU 名称、完整模型的碰撞球构成与规划器实际构造参数。
 */
struct CuroboPingInfo
{
    int64_t pid = 0;                    ///< 应答的 GPU 服务进程 pid（核对是否为本进程拉起的子进程）
    std::string device;                 ///< GPU 名称
    int64_t body_spheres = 0;           ///< 本体碰撞球数
    int64_t end_effector_spheres = 0;   ///< 末端碰撞球数
    int64_t total_spheres = 0;          ///< 总球数
    double max_sphere_radius_mm = 0.0;  ///< 模型中最大球半径
    /// 末端球的完整球心与半径及布球参数。C++ 据此写固定模型快照。服务端未提供时为 null。
    nlohmann::json end_effector_detail;
    /// 规划器实际构造参数（激活距离、图规划、体素缓存），启动时与配置核对（文档 0.4）。
    nlohmann::json applied;
};

/**
 * @brief 一次规划中各步骤的统计，用于日志。字段与服务端 steps 一一对应。
 */
struct CuroboPlanSteps
{
    int64_t ik_seeds = 0;        ///< ① 撒了多少个逆解种子
    int64_t ik_returned = 0;     ///< ① 交出几个（= trajopt 种子数）
    int64_t ik_converged = 0;    ///< ① 其中收敛几个
    double ik_ms = 0.0;
    int64_t padded = 0;          ///< ② 补位了几个
    bool graph_ok = false;       ///< ③ 图规划是否找到几何路径
    bool graph_skipped = false;  ///< ③ 是否没有图规划器
    double graph_ms = 0.0;
    int64_t trajopt_requested = 0; ///< ④ 请求交出几条
    int64_t trajopt_converged = 0; ///< ④ 实际收敛几条
    double trajopt_ms = 0.0;
    double position_error_mm = 0.0;  ///< ④ 不收敛时的最小位置误差
    double rotation_error_deg = 0.0; ///< ④ 不收敛时的最小姿态误差
    int64_t interp_count = 0;    ///< ⑤ 插值出几条
    double total_ms = 0.0;       ///< 服务端总耗时
    std::string failed_step;     ///< 失败发生在哪一步（ik/graph/trajopt/interp）
};

/**
 * @brief cuRobo 规划服务客户端（阻塞式，短连接）。
 */
class CuroboPlanClient
{
  public:
    CuroboPlanClient() = default;
    CuroboPlanClient(const CuroboPlanClient&) = delete;
    CuroboPlanClient& operator=(const CuroboPlanClient&) = delete;

    /**
     * @brief 绑定服务地址。
     * @param host 服务主机（本机 127.0.0.1）
     * @param port 端口
     * @param timeout_ms 单次请求的收发超时（ms）
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true
     */
    bool Init(const std::string& host, int32_t port, int32_t timeout_ms, std::string& error);

    /**
     * @brief 探活；顺带取回 GPU 名称与完整模型的碰撞球构成。
     * @param info 输出：探活信息
     * @param error 失败原因（成功时为空）
     */
    bool Ping(CuroboPingInfo& info, std::string& error) const;

    /**
     * @brief 下发碰撞世界。同一 world_key 服务端会复用。
     * @param request 下发参数
     * @param meta 输出：本轮网格的可审计元数据（范围、尺寸、体素数、占据来源拆分），
     *        由调用方写进 trajectory_manifest.json 的 collision_world 节（改动十）
     * @param error 失败原因（成功时为空）
     */
    bool SetWorld(const CuroboWorldRequest& request, nlohmann::json& meta, std::string& error) const;

    /**
     * @brief 规划轨迹，一次交出多条候选。
     * @param q_start 起点六轴（rad）
     * @param goal_flange 目标法兰位姿 4x4（平移 mm）
     * @param return_trajectories 期望交出的候选条数上限
     * @param trajectories 输出：按 cuRobo 代价排序的候选轨迹（下标 0 = rank1）
     * @param steps 输出：各步骤统计，供日志
     * @param error 失败原因（成功时为空；规划不出解也算失败）
     * @return 至少拿到一条返回 true
     *
     * @warning 排序是 cuRobo 自己的代价（精确到位 + 平滑省时），**与工程的
     *          A/B/C 评级无关**。实测它会把绕行 8.6 倍、高出拍照点 1.28 m 的
     *          轨迹排在第 2 名。调用方必须自己逐条评级，不能直接取 rank1。
     */
    bool Plan(const JointRadians& q_start,
              const Eigen::Matrix4d& goal_flange,
              int64_t return_trajectories,
              std::vector<JointTrajectory>& trajectories,
              CuroboPlanSteps& steps,
              std::string& error) const;

    /**
     * @brief 关节目标规划：抓取实际末点 → 固定 q_place（放置段）。
     *
     * 目标是**关节构型**本身，不把 q_place 转成法兰位姿再求 IK，以免得到另一个
     * 关节构型（文档 5.3）。一次交出多条候选，按 cuRobo 代价排序。
     * @param q_start 起点六轴（rad），必须是抓取轨迹优化、插值后的实际末点
     * @param q_goal 目标六轴（rad），固定 q_place
     * @param return_trajectories 期望交出的候选条数上限
     * @param trajectories 输出：候选轨迹（下标 0 = rank1）
     * @param steps 输出：各步骤统计
     * @param error 失败原因（成功时为空）
     * @return 至少拿到一条返回 true
     */
    bool PlanJoint(const JointRadians& q_start,
                   const JointRadians& q_goal,
                   int64_t return_trajectories,
                   std::vector<JointTrajectory>& trajectories,
                   CuroboPlanSteps& steps,
                   std::string& error) const;

    /**
     * @brief 把一路深度图写成服务端要的二进制格式（float32 行主序，mm）。
     */
    static bool WriteDepthBinary(const std::vector<float>& depth_mm, const std::string& path,
                                 std::string& error);

    /**
     * @brief 把基座系点云写成二进制格式（float32 xyz，mm）。
     */
    static bool WriteCloudBinary(const std::vector<Eigen::Vector3d>& cloud,
                                 const std::string& path,
                                 std::string& error);

  private:
    std::string host_ = "127.0.0.1";
    int32_t port_ = 34567;
    int32_t timeout_ms_ = 60000;

    /** @brief 建连、发一行 JSON、收一行 JSON。 */
    bool Call(const std::string& request_line, std::string& response_line, std::string& error) const;
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_CUROBO_PLAN_CLIENT_H
