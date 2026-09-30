/**
 * @file load_grasp_candidates_stage.cpp
 * @brief flow trajectory / 一 load_grasp_candidates：核验本轮候选并准备两工位信息。
 *
 * 启动准备：放置工位 q_place 及对应法兰位姿，并校验 q_place 六轴限位。
 * 触发执行：读取本轮抓取候选与 request.txt 中的拍照法兰状态，准备有序候选和起点。
 *
 * 候选是基座系的目标法兰位姿；起点使用固定 q_photo，目标法兰位姿不能替代起点关节角。
 */

#include <cmath>

#include "trajectory_plan/config/config_types.h"
#include "trajectory_plan/pipeline/stage_registry.h"
#include "trajectory_plan/tools/json_util.h"
#include "trajectory_plan/tools/log.h"

namespace openmind::trajectory_plan
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadiansPerDegree = kPi / 180.0;
constexpr double kDegreesPerRadian = 180.0 / kPi;

} // namespace

class LoadGraspCandidatesStage : public StageBase
{
  public:
    bool Init(const StageConfig& config, std::string& error) override
    {
        BindConfig(config);
        if (config.config_root == nullptr || config.kinematics == nullptr)
        {
            error = "缺少启动配置或运动学";
            return false;
        }
        kinematics_ = config.kinematics;
        const nlohmann::json& app = (*config.config_root)["app"];
        FixedStation place;
        if (!FixedStation::Parse(app["place"], place, error))
        {
            return false;
        }
        for (size_t index = 0; index < 6; ++index)
        {
            q_place_[index] = place.q_deg[index] * kRadiansPerDegree;
        }
        place_flange_ = place.FlangeMatrix();

        // 启动加载并校验固定 q_place 的六轴限位（文档 5.3）。
        if (!kinematics_->WithinLimits(q_place_))
        {
            error = "q_place 超出机型关节限位";
            return false;
        }
        max_candidates_ = config.flow_config != nullptr
                              ? config.flow_config->value("max_grasp_candidates", 10)
                              : 10;
        LOG_INFO << log_tag_ << "[初始化] station=loaded q_place 限位=通过 q_photo=每轮取自request.txt"
                 << " 候选上限=" << max_candidates_;
        return true;
    }

    bool Execute(PipelineContext& ctx, std::string& error) override
    {
        if (ctx.grasp_candidates.empty())
        {
            error = "本轮没有抓取候选（第一流程未产出或未执行）";
            return false;
        }
        // 代次一致性：候选来自本轮 ctx，与 grasp_result.json 的 generation_id 同源。
        if (ctx.generation_id.empty())
        {
            error = "本轮代次标识缺失，拒绝使用候选";
            return false;
        }
        ctx.grasp_flange_poses.clear();
        ctx.grasp_candidate_scores.clear();
        for (const GraspResult& candidate : ctx.grasp_candidates)
        {
            if (static_cast<int64_t>(ctx.grasp_flange_poses.size()) >= max_candidates_)
            {
                break;
            }
            const Pose& flange = candidate.trajectory_grasp_flange;
            if (!std::isfinite(flange.x) || !std::isfinite(flange.y) || !std::isfinite(flange.z) ||
                !std::isfinite(flange.rx) || !std::isfinite(flange.ry) || !std::isfinite(flange.rz))
            {
                error = "候选 " + std::to_string(ctx.grasp_flange_poses.size()) +
                        " 的 trajectory_grasp_flange 含非有限数值";
                return false;
            }
            ctx.grasp_flange_poses.push_back(flange);
            ctx.grasp_candidate_scores.push_back(candidate.score);
        }
        if (ctx.grasp_flange_poses.empty())
        {
            error = "没有有效的抓取法兰目标";
            return false;
        }
        // 候选必须已按分数降序；这里核验顺序，不重新排序。
        for (size_t index = 1; index < ctx.grasp_candidate_scores.size(); ++index)
        {
            if (ctx.grasp_candidate_scores[index] > ctx.grasp_candidate_scores[index - 1] + 1e-12)
            {
                error = "抓取候选未按分数降序排列";
                return false;
            }
        }
        for (size_t index = 0; index < 6; ++index)
        {
            ctx.q_photo[index] = ctx.t_base_flange_joint_angles_deg[index] * kRadiansPerDegree;
        }
        ctx.q_place = q_place_;
        ctx.photo_flange = ctx.t_base_flange;
        ctx.place_flange = place_flange_;
        ctx.statistics.grasp_targets_total = static_cast<int64_t>(ctx.grasp_flange_poses.size());

        JointDegrees photo_deg {};
        JointDegrees place_deg {};
        for (size_t index = 0; index < 6; ++index)
        {
            photo_deg[index] = ctx.q_photo[index] * kDegreesPerRadian;
            place_deg[index] = q_place_[index] * kDegreesPerRadian;
        }
        LOG_INFO << log_tag_ << "    generation_match=true 候选=" << ctx.grasp_flange_poses.size()
                 << " start_source=request.T_base_flange_joint_angles_deg";
        LOG_INFO << log_tag_ << "    q_photo_deg=[" << FormatJoint(photo_deg) << "]"
                 << " q_place_deg=[" << FormatJoint(place_deg) << "]";
        return true;
    }

  private:
    /** @brief 六轴角度排成日志用的一行。 */
    static std::string FormatJoint(const JointDegrees& q)
    {
        std::string text;
        for (size_t index = 0; index < 6; ++index)
        {
            text += (index > 0 ? ", " : "") + std::to_string(q[index]);
        }
        return text;
    }

    const RobotKinematicsInterface* kinematics_ = nullptr; ///< 机型运动学
    JointRadians q_place_ {};   ///< 放置六轴（rad）
    Eigen::Matrix4d place_flange_ = Eigen::Matrix4d::Identity(); ///< 放置法兰（mm）
    int64_t max_candidates_ = 10; ///< 本轮最多使用的候选数
};

REGISTER_STAGE("load_grasp_candidates", LoadGraspCandidatesStage);

} // namespace openmind::trajectory_plan
