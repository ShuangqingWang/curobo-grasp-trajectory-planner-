/**
 * @file compose_grasp_poses_stage.cpp
 * @brief flow grasp / 三 compose_grasp_poses：工件系标签 → 基座系四组位姿。
 *
 * 业务步骤 [3/4]，内部子步骤 [3.1]~[3.4]。
 *
 * ```text
 * T_B_TCP_original = T_B_O · T_O_DB · T_DB_TCP
 * T_B_TCP_planned  = T_B_TCP_original · TransZ(clearance)
 * T_B_F_original   = T_B_TCP_original · T_TCP_F
 * T_B_F_planned    = T_B_TCP_planned  · T_TCP_F
 * ```
 *
 * 退让是右乘 TCP 局部 Z 方向平移，保持姿态不变，不是沿基座系 Z 轴移动。
 * 标定方向在启动时核对并记录，不凭字段名猜测，也不未经核对直接求逆。
 */

#include <cmath>

#include "trajectory_plan/algorithm/grasp_pose_compose_algorithm.h"
#include "trajectory_plan/pipeline/stage_registry.h"
#include "trajectory_plan/tools/json_util.h"
#include "trajectory_plan/tools/log.h"

namespace openmind::trajectory_plan
{

class ComposeGraspPosesStage : public StageBase
{
  public:
    bool Init(const StageConfig& config, std::string& error) override
    {
        BindConfig(config);
        if (config.config_root == nullptr)
        {
            error = "缺少启动配置";
            return false;
        }
        const nlohmann::json& app = (*config.config_root)["app"];
        Eigen::Matrix4d t_tcp_flange = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d t_db_tcp = Eigen::Matrix4d::Identity();
        if (!MatrixFromJson(app["t_flange_tcp"], t_tcp_flange, error) ||
            !MatrixFromJson(app["t_tcp_grasp_database"], t_db_tcp, error))
        {
            return false;
        }
        clearance_mm_ = app.value("camera_clearance_along_tcp_z_mm", -84.0);
        if (!std::isfinite(clearance_mm_))
        {
            error = "camera_clearance_along_tcp_z_mm 非有限数值";
            return false;
        }

        // ---- 标定方向核对（文档「步骤 3」要求启动时明确，不能仅凭字段名判断）----
        // 合成需要 T_DB_TCP 与 T_TCP_F。
        // app.json 的 t_flange_tcp 数值为 [0,0,-0.2115]：TCP 的 +Z 指向工件，
        // 法兰位于 TCP 后方 211.5mm，因此该矩阵实际就是 T_TCP_F，直接右乘，不求逆。
        const double tcp_to_flange_z_mm = t_tcp_flange(2, 3) * 1000.0;
        if (tcp_to_flange_z_mm >= 0.0)
        {
            error = "t_flange_tcp 的 Z 平移应为负（法兰在 TCP 后方），实际 " +
                    std::to_string(tcp_to_flange_z_mm) + " mm；标定方向需重新核对";
            return false;
        }
        // t_tcp_grasp_database 为纯旋转对合阵（自身即自身的逆），作为 T_DB_TCP 使用。
        const Eigen::Matrix3d db_rotation = t_db_tcp.block<3, 3>(0, 0);
        const bool involutory = (db_rotation * db_rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-9);
        if (!involutory)
        {
            LOG_WARN << log_tag_ << "[初始化] t_tcp_grasp_database 非对合阵，"
                     << "方向敏感，请按标定定义确认是否需要求逆";
        }

        if (!composer_.Init(t_tcp_flange, t_db_tcp, clearance_mm_, error))
        {
            return false;
        }
        LOG_INFO << log_tag_ << "[初始化] 标定方向=T_DB_TCP/T_TCP_F 平移单位=mm"
                 << " T_TCP_F_z=" << tcp_to_flange_z_mm << "mm 退让=" << clearance_mm_ << "mm"
                 << " 对合校验=" << (involutory ? "通过" : "需人工确认");
        return true;
    }

    bool Execute(PipelineContext& ctx, std::string& error) override
    {
        if (ctx.workpiece_labels.empty())
        {
            error = "本轮没有可用标签";
            return false;
        }
        ctx.clearance_mm = clearance_mm_;
        ctx.grasp_candidates.clear();
        LOG_INFO << log_tag_ << "    [3/4] 位姿合成 候选=" << ctx.workpiece_labels.size()
                 << " 标定=复用";

        // [3.1] 工件系标签转换到基座系 TCP；[3.2] 沿 TCP 局部 Z 退让；
        // [3.3] 原始与退让 TCP 分别换算法兰。三步在 Compose 内一次完成。
        for (const GraspLabel& label : ctx.workpiece_labels)
        {
            ctx.grasp_candidates.push_back(composer_.Compose(ctx.t_b_o, label));
        }
        LOG_INFO << log_tag_ << "      [3.1] 工件系标签转换到基座系TCP 完成";
        LOG_INFO << log_tag_ << "      [3.2] 沿TCP局部Z退让 " << clearance_mm_ << "mm 完成";
        LOG_INFO << log_tag_ << "      [3.3] 原始与退让TCP分别换算法兰 完成";

        // [3.4] 输出位姿校验：每个生成位姿数值必须有效。
        for (size_t index = 0; index < ctx.grasp_candidates.size(); ++index)
        {
            const GraspResult& candidate = ctx.grasp_candidates[index];
            if (!PoseFinite(candidate.grasp_tcp) || !PoseFinite(candidate.grasp_flange) ||
                !PoseFinite(candidate.trajectory_grasp_tcp) ||
                !PoseFinite(candidate.trajectory_grasp_flange))
            {
                error = "候选 " + std::to_string(index) + " 的位姿含非有限数值";
                return false;
            }
        }
        if (ctx.grasp_candidates.empty())
        {
            error = "没有合成出任何抓取候选";
            return false;
        }
        LOG_INFO << log_tag_ << "      [3.4] 输出位姿校验 通过 数量=" << ctx.grasp_candidates.size();
        for (size_t index = 0; index < ctx.grasp_candidates.size(); ++index)
        {
            const GraspResult& candidate = ctx.grasp_candidates[index];
            LOG_INFO << log_tag_ << "          candidate=" << index << " score=" << candidate.score
                     << " gripper_width_m=" << candidate.gripper_width
                     << " grasp_tcp_mm_deg=[" << FormatPose(candidate.grasp_tcp) << "]"
                     << " grasp_flange_mm_deg=[" << FormatPose(candidate.grasp_flange) << "]"
                     << " trajectory_grasp_tcp_mm_deg=[" << FormatPose(candidate.trajectory_grasp_tcp) << "]"
                     << " trajectory_grasp_flange_mm_deg=[" << FormatPose(candidate.trajectory_grasp_flange)
                     << "]";
        }
        return true;
    }

  private:
    /** @brief 六个分量都必须是有限数值。 */
    static bool PoseFinite(const Pose& pose)
    {
        return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.z) &&
               std::isfinite(pose.rx) && std::isfinite(pose.ry) && std::isfinite(pose.rz);
    }

    GraspPoseComposeAlgorithm composer_; ///< 启动初始化，触发后复用
    double clearance_mm_ = -84.0;        ///< 沿 TCP 局部 Z 的退让量（mm）
};

REGISTER_STAGE("compose_grasp_poses", ComposeGraspPosesStage);

} // namespace openmind::trajectory_plan
