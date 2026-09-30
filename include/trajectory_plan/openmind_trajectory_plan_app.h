#ifndef HYBRID_TRAJECTORY_PLAN_OPENMIND_TRAJECTORY_PLAN_APP_H
#define HYBRID_TRAJECTORY_PLAN_OPENMIND_TRAJECTORY_PLAN_APP_H

/**
 * @file openmind_trajectory_plan_app.h
 * @brief 常驻规划服务躯干：会话日志 → 配置与模型 → pipeline 初始化与预热 → 起 TCP。
 *
 * 启动顺序（文档 S1~S6）：
 * - S0 建会话目录并打开 service.log，确保初始化失败也有日志；
 * - S1 配置与流水线组装：机型、关节限位、拍照/放置工位、相机标定、平台与末端几何；
 * - S2 CPU 初始化：运动学、深度融合、评级、复核、输出模块与抓取标签缓存；
 * - S3/S4 固定世界与 GPU 求解器：带孔平台、末端碰撞球、IK/图规划/优化器；
 * - S5 预热：抓取位姿目标与放置关节目标两条调用路径；
 * - S6 全部成功后 service_ready=true，才开始接受规划任务。
 */

#include "nlohmann/json.hpp"

#include <sys/types.h>

#include <string>

#include "trajectory_plan/camera/camera_manager.h"
#include "trajectory_plan/config/config_service.h"
#include "trajectory_plan/device/device_manager.h"
#include "trajectory_plan/pipeline/pipeline.h"
#include "trajectory_plan/service/session_archive.h"
#include "trajectory_plan/service/tcp_server.h"
#include "trajectory_plan/service/trigger_router.h"

namespace openmind::trajectory_plan
{

/**
 * @brief 常驻规划服务躯干：OpenmindTrajectoryPlan。
 */
class OpenmindTrajectoryPlanApp
{
  public:
    OpenmindTrajectoryPlanApp() = default;
    OpenmindTrajectoryPlanApp(const OpenmindTrajectoryPlanApp&) = delete;
    OpenmindTrajectoryPlanApp& operator=(const OpenmindTrajectoryPlanApp&) = delete;

    /**
     * @brief 读取合并后的配置（argv[1]）。
     */
    bool LoadConfig(const std::string& config_path, std::string& error);

    /**
     * @brief 覆盖 app.json 的 log_root（--log-root）。
     *
     * 生产按配置使用 /var/hybird_log/grasp_log；该覆盖用于 /var 不可写的开发机，
     * 实际生效路径会写进启动日志和 session.json，不会悄悄换地方。
     */
    void OverrideLogRoot(const std::string& log_root);

    /**
     * @brief 初始化：会话日志、相机、机型、pipeline（含 GPU 预热）、固定模型快照。
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true；失败时不宣布服务就绪
     */
    bool Init(std::string& error);

    /**
     * @brief 起 TCP 服务并等待中断；收到信号后优雅退出。
     */
    bool Run(std::string& error);

    /**
     * @brief 用本地目录跑一轮后退出（回放测试入口，不听 TCP）。
     * @param input_dir 含 request.txt 与三张 TIFF 的目录
     */
    bool RunOnce(const std::string& input_dir, std::string& error);

    /** @brief 停止服务。 */
    void Stop();

    /** @brief 打印合并后的配置（--dump-config 用）。 */
    void DumpConfig() const;

  private:
    /**
     * @brief 导出第三流程使用的固定模型快照 output/visualization/collision_model.json。
     *
     * 快照包含实际碰撞球、固定平台、圆柱豁免区，以及模型标识、挂载坐标系和单位。
     * 可视化启动时读取一次，不在浏览器中另生成一套碰撞球。
     */
    bool ExportCollisionModelSnapshot(std::string& error);

    /** @brief 启动并等待本进程托管的 cuRobo GPU 服务就绪。 */
    bool StartCuroboService(std::string& error);

    /** @brief 停止并回收本进程启动的 cuRobo 子进程。 */
    void StopCuroboService();

    ConfigService config_service_;          ///< 配置服务
    CameraManager camera_manager_;          ///< 相机（TIFF 解码）
    DeviceManager device_manager_;          ///< 机型模型
    SessionArchive session_archive_;        ///< 会话日志与逐轮归档
    Pipeline pipeline_;                     ///< 两条 flow
    TriggerRouter trigger_router_;          ///< 触发路由
    TcpServer tcp_server_;                  ///< 外部触发 TCP 服务
    nlohmann::json grasp_flow_config_;      ///< grasp.yaml
    nlohmann::json trajectory_flow_config_; ///< trajectory.yaml
    std::string output_root_;               ///< 项目 output 根目录
    std::string grasp_output_dir_;          ///< output/grasp_generation
    std::string trajectory_output_dir_;     ///< output/trajectory_planning
    std::string log_root_override_;         ///< --log-root 覆盖值（空表示用配置）
    std::string collision_model_id_;        ///< 固定模型标识
    std::string collision_model_sha256_;    ///< 固定模型快照哈希
    pid_t curobo_pid_ = -1;                 ///< 本进程托管的 cuRobo Python 子进程
    bool initialized_ = false;              ///< 是否已 Init
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_OPENMIND_TRAJECTORY_PLAN_APP_H
