/**
 * @file plan_full_cycle_stage.cpp
 * @brief flow trajectory / 四 plan_full_cycle：业务步骤 [2/6]~[5/6]。
 *
 * 对最多 10 个抓取候选按分数从高到低逐个试：
 *   [2/6] 取一个候选法兰位姿
 *   [3/6] 末端刚体碰撞预筛（低成本端点预筛，不替代 GPU 的沿途末端碰撞保护）
 *   [4/6] 请 cuRobo 交出多条候选轨迹（内部 ①IK ②补位 ③图规划 ④优化 ⑤插值）
 *   [5/6] 按 rank 逐条筛选：5.1 评级 → 5.2 复核 → 5.3 放置段关节目标规划及复核
 *
 * 取一个、筛一个、规划一个：步骤 2~5 保持在同一个候选循环内，不提前预筛全部目标。
 * 抓取与放置两段均通过才采用；首个两段合格的完整周期结束搜索。
 */

#include <algorithm>
#include <chrono>
#include <cmath>

#include "trajectory_plan/algorithm/curobo_plan_client.h"
#include "trajectory_plan/algorithm/pose.h"
#include "trajectory_plan/algorithm/trajectory_collision_check_algorithm.h"
#include "trajectory_plan/algorithm/trajectory_output_format_algorithm.h"
#include "trajectory_plan/algorithm/trajectory_quality_grade_algorithm.h"
#include "trajectory_plan/pipeline/stage_registry.h"
#include "trajectory_plan/tools/json_util.h"
#include "trajectory_plan/tools/log.h"

namespace openmind::trajectory_plan
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesPerRadian = 180.0 / kPi;

/**
 * @brief 六轴净转角排成日志用的一行。
 */
std::string FormatJointDegrees(const JointDegrees& values)
{
    std::string text;
    for (size_t index = 0; index < 6; ++index)
    {
        text += (index > 0 ? ", " : "") + std::to_string(values[index]);
    }
    return text;
}

} // namespace

class PlanFullCycleStage : public StageBase
{
  public:
    bool Init(const StageConfig& config, std::string& error) override
    {
        BindConfig(config);
        if (config.kinematics == nullptr || config.flow_config == nullptr)
        {
            error = "缺少运动学或 trajectory.yaml";
            return false;
        }
        const nlohmann::json& cfg = *config.flow_config;
        output_dir_ = config.trajectory_output_dir;

        evaluate_all_ = cfg.value("evaluate_all_grasp_candidates", false);
        stop_on_first_cycle_ = cfg.value("stop_on_first_complete_cycle", true) && !evaluate_all_;
        max_selection_ms_ = cfg.value("max_selection_ms", 15000.0);
        no_solution_ms_ = cfg.value("max_no_solution_search_ms", 30000.0);
        max_no_solution_candidates_ = cfg.value("max_no_solution_candidates", 12);
        endpoint_joint_tolerance_deg_ = cfg.value("endpoint_joint_tolerance_deg", 0.5);

        GradeThresholds thresholds;
        if (cfg.contains("grade") && cfg["grade"].is_object())
        {
            const nlohmann::json& grade = cfg["grade"];
            thresholds.path_length_ratio_a = grade.value("path_length_ratio_a", 1.5);
            thresholds.path_length_ratio_b = grade.value("path_length_ratio_b", 2.0);
            thresholds.net_joint_travel_a_deg = grade.value("net_joint_travel_a_deg", 180.0);
            thresholds.net_joint_travel_b_deg = grade.value("net_joint_travel_b_deg", 360.0);
            thresholds.lift_above_photo_a_mm = grade.value("lift_above_photo_a_mm", 5.0);
            thresholds.lift_above_photo_b_mm = grade.value("lift_above_photo_b_mm", 10.0);
        }
        thresholds.lift_numerical_epsilon_mm = cfg.value("lift_numerical_epsilon_mm", 0.001);
        thresholds_ = thresholds;

        endpoint_position_tolerance_mm_ = cfg.value("endpoint_position_tolerance_mm", 1.0);
        endpoint_rotation_tolerance_deg_ = cfg.value("endpoint_rotation_tolerance_deg", 0.2);
        const double redundant = cfg.value("max_redundant_joint_travel_deg", 360.0);
        const nlohmann::json& weights = cfg.value("score_weights", nlohmann::json::object());

        if (!checker_.Init(config.kinematics, redundant,
                           endpoint_position_tolerance_mm_, endpoint_rotation_tolerance_deg_, error) ||
            !grader_.Init(config.kinematics, thresholds_, weights.value("grasp_quality", 20.0),
                          weights.value("photo_to_grasp", 35.0), weights.value("grasp_to_place", 35.0),
                          weights.value("continuity", 10.0), error))
        {
            return false;
        }

        const nlohmann::json curobo = cfg.value("curobo", nlohmann::json::object());
        return_trajectories_ = curobo.value("return_trajectories", 4);
        place_return_trajectories_ = curobo.value("place_return_trajectories", 4);
        if (!planner_.Init(curobo.value("host", std::string("127.0.0.1")),
                           curobo.value("port", 34567),
                           curobo.value("timeout_ms", 120000), error))
        {
            return false;
        }

        // 启动即确认 GPU 求解器就绪且**完整模型**（本体球 + 末端球）已挂好，
        // 并确认两条调用路径已预热；不能在缺少末端球的模型上完成预热后才补挂末端。
        CuroboPingInfo ping;
        if (!planner_.Ping(ping, error))
        {
            error = "cuRobo 规划服务不可用，服务不得宣布就绪: " + error;
            return false;
        }
        if (ping.end_effector_spheres <= 0)
        {
            error = "GPU 完整模型缺少末端碰撞球（附录 B.2 要求抓取、放置两段均检查末端沿途碰撞）";
            return false;
        }
        LOG_INFO << log_tag_ << "[初始化] gpu=" << ping.device
                 << " body_spheres=" << ping.body_spheres
                 << " end_effector_spheres=" << ping.end_effector_spheres
                 << " total_spheres=" << ping.total_spheres
                 << " photo_to_grasp_end_effector_collision=enabled"
                 << " grasp_to_place_end_effector_collision=enabled";
        LOG_INFO << log_tag_ << "[初始化] 抓取候选轨迹=" << return_trajectories_
                 << " 放置候选轨迹=" << place_return_trajectories_
                 << " gpu_tolerance=" << endpoint_position_tolerance_mm_ << "mm/"
                 << endpoint_rotation_tolerance_deg_ << "° cpu_endpoint_tolerance="
                 << endpoint_position_tolerance_mm_ << "mm/" << endpoint_rotation_tolerance_deg_
                 << "° endpoint_joint_tolerance=" << endpoint_joint_tolerance_deg_ << "°";
        return true;
    }

    bool Execute(PipelineContext& ctx, std::string& error) override
    {
        (void)error;
        ctx.attempts.clear();
        ctx.has_solution = false;
        CandidateStatistics& statistics = ctx.statistics;
        statistics.grasp_targets_total = static_cast<int64_t>(ctx.grasp_flange_poses.size());

        const auto started = std::chrono::steady_clock::now();
        auto elapsed_ms = [&started]() {
            return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
                .count();
        };

        LOG_INFO << log_tag_ << "[开始] solvers=reused warmed=true 抓取候选="
                 << ctx.grasp_flange_poses.size() << " 评级阈值=倍率" << thresholds_.path_length_ratio_a
                 << "/" << thresholds_.path_length_ratio_b << " 抬升" << thresholds_.lift_above_photo_a_mm
                 << "/" << thresholds_.lift_above_photo_b_mm << "mm 净转角"
                 << thresholds_.net_joint_travel_a_deg << "/" << thresholds_.net_joint_travel_b_deg
                 << "° 复核容差=" << endpoint_position_tolerance_mm_ << "mm/"
                 << endpoint_rotation_tolerance_deg_ << "° 搜索预算=" << no_solution_ms_ << "ms/"
                 << max_no_solution_candidates_ << "候选";

        std::vector<std::string> produced_files;
        for (size_t candidate_index = 0; candidate_index < ctx.grasp_flange_poses.size();
             ++candidate_index)
        {
            if (!evaluate_all_ && !ctx.has_solution &&
                (statistics.grasp_targets_attempted >= max_no_solution_candidates_ ||
                 elapsed_ms() >= no_solution_ms_))
            {
                statistics.stop_reason = "预算耗尽";
                LOG_INFO << log_tag_ << "[停止] 原因=预算耗尽 未尝试目标="
                         << (ctx.grasp_flange_poses.size() - candidate_index);
                break;
            }
            if (!evaluate_all_ && ctx.has_solution && elapsed_ms() >= max_selection_ms_)
            {
                statistics.stop_reason = "选优预算耗尽";
                break;
            }

            // ---- [2/6] 取一个抓取候选的法兰位姿 ----
            const Pose& goal_pose = ctx.grasp_flange_poses[candidate_index];
            const Eigen::Matrix4d goal_flange = TransformFromPose(goal_pose);
            LOG_INFO << log_tag_ << "  [2/6] 选择候选 candidate=" << candidate_index << " 进度="
                     << (candidate_index + 1) << "/" << ctx.grasp_flange_poses.size()
                     << " 抓取分数=" << ctx.grasp_candidate_scores[candidate_index]
                     << " goal_flange=[" << FormatPose(goal_pose) << "]";

            // ---- [3/6] 末端预筛：改动五已取消 ----
            // GPU 完整模型把末端碰撞球挂在 flange_link 上，抓取段与放置段**沿途**
            // 都在查末端 vs 点云 + 平台，终点只是沿途的一个点，预筛是重复步骤。
            // 业务编号 [3/6] 保留不重排，后续 [4/6]~[6/6] 编号不动。
            LOG_INFO << log_tag_ << "  [3/6] 末端预筛 已取消（由 GPU 完整模型的沿途末端碰撞覆盖）";
            ++statistics.grasp_targets_attempted;

            // ---- [4/6] 请 cuRobo 交出多条候选轨迹 ----
            std::vector<JointTrajectory> grasp_trajectories;
            CuroboPlanSteps steps;
            std::string plan_error;
            const bool planned = planner_.Plan(ctx.q_photo, goal_flange, return_trajectories_,
                                               grasp_trajectories, steps, plan_error);
            LogPlanSteps(steps, candidate_index, planned);
            if (!planned)
            {
                ++statistics.gpu_failed;
                PlanAttempt attempt;
                attempt.candidate_index = static_cast<int64_t>(candidate_index);
                attempt.failure_reason = "GPU 规划失败(" + steps.failed_step + "): " + plan_error;
                ctx.attempts.push_back(attempt);
                LOG_INFO << log_tag_ << "  [4/6] GPU规划 失败 原因=" << plan_error
                         << " 卡在=" << steps.failed_step << "，尝试下一抓取候选";
                LOG_INFO << log_tag_ << "  [5/6] 未执行 原因=无返回轨迹";
                continue;
            }
            statistics.grasp_trajectories_returned += static_cast<int64_t>(grasp_trajectories.size());

            // 全部候选落盘：未经评级筛选的原始候选，只供事后排查，不是发布产物。
            for (size_t rank = 0; rank < grasp_trajectories.size(); ++rank)
            {
                const std::string name = "photo_to_grasp_cand" + std::to_string(candidate_index) +
                                         "_rank" + std::to_string(rank + 1) + ".json";
                std::string write_error;
                if (WriteCandidate(ctx, name, grasp_trajectories[rank], "photo_to_grasp", write_error))
                {
                    produced_files.push_back("candidates/" + name);
                }
                else
                {
                    LOG_WARN << log_tag_ << "        候选轨迹落盘失败 " << name << ": " << write_error;
                }
            }

            // ---- [5/6] 按 rank 逐条筛选 ----
            bool adopted = false;
            for (size_t rank = 0; rank < grasp_trajectories.size() && !adopted; ++rank)
            {
                if (TryGraspRank(ctx, statistics, candidate_index, rank, goal_flange,
                                 grasp_trajectories[rank], produced_files))
                {
                    adopted = true;
                }
            }
            if (adopted)
            {
                LOG_INFO << log_tag_ << "  [5/6] 筛选 完成 结果=采用完整周期 candidate="
                         << candidate_index << " 抓取等级="
                         << TrajectoryQualityGradeAlgorithm::GradeName(ctx.cycle_quality.photo_to_grasp.grade)
                         << " 放置等级="
                         << TrajectoryQualityGradeAlgorithm::GradeName(ctx.cycle_quality.grasp_to_place.grade)
                         << " 综合等级="
                         << TrajectoryQualityGradeAlgorithm::GradeName(ctx.cycle_quality.grade);
                statistics.stop_reason = "首个合格完整周期即停止";
                if (stop_on_first_cycle_)
                {
                    break;
                }
            }
            else
            {
                LOG_INFO << log_tag_ << "  [5/6] 筛选 完成 结果=未采用 candidate="
                         << candidate_index << " 的 " << grasp_trajectories.size()
                         << " 条候选全部不可用，尝试下一抓取候选";
            }
        }

        if (!ctx.has_solution)
        {
            if (statistics.stop_reason.empty())
            {
                statistics.stop_reason = "候选耗尽";
            }
            ctx.failure_reason = "未找到 A/B 级完整周期（" + statistics.stop_reason + "）";
            LOG_INFO << log_tag_ << "[结束] 结果=无解 原因=" << ctx.failure_reason;
        }
        // 清理本轮不再产生的旧受管候选由输出 stage 统一执行。
        ctx.stage_timings_ms["trajectory/plan_full_cycle_search"] = elapsed_ms();
        LOG_INFO << log_tag_ << "[结束] 尝试目标=" << statistics.grasp_targets_attempted << "/"
                 << statistics.grasp_targets_total << " 预筛淘汰=" << statistics.prefilter_rejected
                 << "（预筛已取消，恒为 0）"
                 << " GPU失败=" << statistics.gpu_failed
                 << " 返回轨迹=" << statistics.grasp_trajectories_returned
                 << " 评级淘汰=" << statistics.grade_rejected
                 << " 复核淘汰=" << statistics.review_rejected
                 << " 放置规划失败=" << statistics.place_plan_failed
                 << " 放置评级淘汰=" << statistics.place_grade_rejected
                 << " 放置复核淘汰=" << statistics.place_review_rejected
                 << " 结束原因=" << statistics.stop_reason << " 总耗时=" << elapsed_ms() << "ms";
        // 正常搜索无解不是 stage 异常：仍然进入输出 stage 写 ready=false 与原因。
        return true;
    }

  private:
    /**
     * @brief 对一条抓取 rank 做 5.1 评级 → 5.2 复核 → 5.3 放置段；两段均通过才采用。
     */
    bool TryGraspRank(PipelineContext& ctx,
                      CandidateStatistics& statistics,
                      size_t candidate_index,
                      size_t rank,
                      const Eigen::Matrix4d& goal_flange,
                      const JointTrajectory& grasp_trajectory,
                      std::vector<std::string>& produced_files)
    {
        PlanAttempt attempt;
        attempt.candidate_index = static_cast<int64_t>(candidate_index);
        attempt.grasp_rank = static_cast<int64_t>(rank + 1);

        // ---- 5.1 抓取段三项评级 ----
        SegmentQuality quality;
        grader_.GradeSegment(grasp_trajectory, true, quality);
        LOG_INFO << log_tag_ << "  [5.1] candidate=" << candidate_index << " rank=" << (rank + 1)
                 << " 路径倍率=" << quality.path_length_ratio << " 等级="
                 << TrajectoryQualityGradeAlgorithm::GradeName(quality.ratio_grade)
                 << " | 抬升=" << quality.lift_above_photo_mm << "mm 违规点="
                 << quality.lift_violation_points << " 等级="
                 << TrajectoryQualityGradeAlgorithm::GradeName(quality.lift_grade)
                 << " | J1~J6净转角_deg=[" << FormatJointDegrees(quality.net_joint_travel_deg)
                 << "] 最大净转角=" << quality.max_net_joint_travel_deg << "° 关节=J"
                 << (quality.max_net_joint_index + 1) << " 等级="
                 << TrajectoryQualityGradeAlgorithm::GradeName(quality.travel_grade)
                 << " → 综合评级=" << TrajectoryQualityGradeAlgorithm::GradeName(quality.grade);
        attempt.photo_to_grasp_grade = quality.grade;
        if (quality.grade == TrajectoryGrade::DISCARD)
        {
            ++statistics.grade_rejected;
            attempt.failure_reason = "抓取段评级丢弃";
            ctx.attempts.push_back(attempt);
            LOG_INFO << log_tag_ << "  [5.2] 未执行 原因=评级丢弃，尝试下一条轨迹";
            return false;
        }

        // ---- 5.2 抓取段运动学复核 ----
        TrajectoryCollisionReport report;
        if (!checker_.CheckKinematicGates(grasp_trajectory, ctx.photo_flange, goal_flange, report))
        {
            ++statistics.review_rejected;
            attempt.failure_reason = "抓取段复核不过: " + report.detail;
            ctx.attempts.push_back(attempt);
            LOG_INFO << log_tag_ << "  [5.2] 复核失败 " << report.detail
                     << " 起点误差=" << report.start_position_error_mm << "mm/"
                     << report.start_rotation_error_deg << "° 终点误差="
                     << report.goal_position_error_mm << "mm/" << report.goal_rotation_error_deg
                     << "° 容差=" << endpoint_position_tolerance_mm_ << "mm/"
                     << endpoint_rotation_tolerance_deg_ << "°，尝试下一条轨迹";
            return false;
        }
        LOG_INFO << log_tag_ << "  [5.2] 六轴限位=通过 净转角复核=通过 起点误差="
                 << report.start_position_error_mm << "mm/" << report.start_rotation_error_deg
                 << "° 终点误差=" << report.goal_position_error_mm << "mm/"
                 << report.goal_rotation_error_deg << "° 容差=" << endpoint_position_tolerance_mm_
                 << "mm/" << endpoint_rotation_tolerance_deg_ << "° 复核=通过";

        // ---- 5.3 放置段：抓取实际末点 → 固定 q_place（关节目标）----
        // q_grasp 必须取最终优化、插值后的抓取轨迹末点，不能取优化前 IK 种子。
        const JointRadians q_grasp = grasp_trajectory.points.back();
        attempt.q_grasp = q_grasp;
        LOG_INFO << log_tag_ << "  [5.3][segment=grasp_to_place] candidate=" << candidate_index
                 << " grasp_rank=" << (rank + 1);
        LOG_INFO << log_tag_ << "    [5.3.1] 起终构型 q_start=抓取实际末点 q_goal=q_place 校验=通过";

        std::vector<JointTrajectory> place_trajectories;
        CuroboPlanSteps place_steps;
        std::string place_error;
        if (!planner_.PlanJoint(q_grasp, ctx.q_place, place_return_trajectories_, place_trajectories,
                                place_steps, place_error))
        {
            ++statistics.place_plan_failed;
            attempt.failure_reason = "放置段关节目标规划失败: " + place_error;
            ctx.attempts.push_back(attempt);
            LOG_INFO << log_tag_ << "    [5.3.2] 关节目标规划=失败 卡在=" << place_steps.failed_step
                     << " 原因=" << place_error << " 耗时=" << place_steps.total_ms
                     << "ms，尝试下一抓取rank";
            return false;
        }
        statistics.place_trajectories_returned += static_cast<int64_t>(place_trajectories.size());
        LOG_INFO << log_tag_ << "    [5.3.2] 关节目标规划=成功 求解器=复用 世界=复用 图规划="
                 << (place_steps.graph_skipped ? "未启用（配置关闭）" : (place_steps.graph_ok ? "通过" : "未找到"))
                 << " 优化收敛=" << place_steps.trajopt_converged << " 耗时=" << place_steps.total_ms
                 << "ms";
        LOG_INFO << log_tag_ << "    [5.3.3] 插值 返回候选=" << place_trajectories.size()
                 << " dt=" << (place_trajectories.empty() ? 0.025 : place_trajectories[0].time_step_s)
                 << "s";

        // 放置候选落盘，文件名包含父抓取轨迹。
        for (size_t place_rank = 0; place_rank < place_trajectories.size(); ++place_rank)
        {
            const std::string name = "grasp_to_place_cand" + std::to_string(candidate_index) +
                                     "_grasp_rank" + std::to_string(rank + 1) + "_rank" +
                                     std::to_string(place_rank + 1) + ".json";
            std::string write_error;
            if (WriteCandidate(ctx, name, place_trajectories[place_rank], "grasp_to_place", write_error))
            {
                produced_files.push_back("candidates/" + name);
            }
            else
            {
                LOG_WARN << log_tag_ << "        放置候选落盘失败 " << name << ": " << write_error;
            }
        }

        for (size_t place_rank = 0; place_rank < place_trajectories.size(); ++place_rank)
        {
            const JointTrajectory& place_trajectory = place_trajectories[place_rank];
            // 放置段评级：不查抬升，允许向上运动。
            SegmentQuality place_quality;
            grader_.GradeSegment(place_trajectory, false, place_quality);
            LOG_INFO << log_tag_ << "    [5.3.4] place_rank=" << (place_rank + 1) << " 路径倍率="
                     << place_quality.path_length_ratio << " 六轴净转角=["
                     << FormatJointDegrees(place_quality.net_joint_travel_deg) << "] 最大="
                     << place_quality.max_net_joint_travel_deg << "° 等级="
                     << TrajectoryQualityGradeAlgorithm::GradeName(place_quality.grade)
                     << " 抬升检查=不适用";
            if (place_quality.grade == TrajectoryGrade::DISCARD)
            {
                ++statistics.place_grade_rejected;
                LOG_INFO << log_tag_ << "    [5.3.4] 放置评级丢弃，尝试下一 place_rank";
                continue;
            }

            // 放置段复核：法兰端点 + 关节端点 + 限位 + 净转角。
            TrajectoryCollisionReport place_report;
            if (!checker_.CheckKinematicGates(place_trajectory, goal_flange, ctx.place_flange,
                                              place_report))
            {
                ++statistics.place_review_rejected;
                LOG_INFO << log_tag_ << "    [5.3.5] 放置法兰端点复核失败 " << place_report.detail
                         << "，尝试下一 place_rank";
                continue;
            }
            if (!checker_.CheckJointEndpoints(place_trajectory, q_grasp, ctx.q_place,
                                              endpoint_joint_tolerance_deg_, place_report))
            {
                ++statistics.place_review_rejected;
                LOG_INFO << log_tag_ << "    [5.3.5] 放置关节端点复核失败 " << place_report.detail
                         << "，尝试下一 place_rank";
                continue;
            }
            LOG_INFO << log_tag_ << "    [5.3.5] 首点关节差=" << place_report.start_joint_error_deg
                     << "° 末点关节差=" << place_report.goal_joint_error_deg << "° 关节容差="
                     << endpoint_joint_tolerance_deg_ << "° 法兰位置误差="
                     << place_report.goal_position_error_mm << "mm 姿态误差="
                     << place_report.goal_rotation_error_deg << "° 限位=通过 连接检查=通过";

            // ---- 两段均通过：采用完整周期 ----
            CycleQuality cycle;
            cycle.photo_to_grasp = quality;
            cycle.grasp_to_place = place_quality;
            // 完整周期等级取两段较差档：A+A 为 A，A+B 或 B+B 为 B。
            cycle.grade = static_cast<TrajectoryGrade>(
                std::max(static_cast<int>(quality.grade), static_cast<int>(place_quality.grade)));
            grader_.GradeCycle(grasp_trajectory, place_trajectory,
                               ctx.grasp_candidate_scores[candidate_index],
                               ctx.grasp_candidate_scores.empty()
                                   ? 1.0
                                   : *std::max_element(ctx.grasp_candidate_scores.begin(),
                                                       ctx.grasp_candidate_scores.end()),
                               cycle);

            attempt.place_rank = static_cast<int64_t>(place_rank + 1);
            attempt.grasp_to_place_grade = place_quality.grade;
            attempt.cycle_grade = cycle.grade;
            ctx.attempts.push_back(attempt);

            ctx.has_solution = true;
            ctx.selected_candidate_index = static_cast<int64_t>(candidate_index);
            ctx.selected_grasp_rank = static_cast<int64_t>(rank + 1);
            ctx.selected_place_rank = static_cast<int64_t>(place_rank + 1);
            ctx.photo_to_grasp_trajectory = grasp_trajectory;
            ctx.grasp_to_place_trajectory = place_trajectory;
            ctx.cycle_quality = cycle;
            ctx.joint_link_max_diff_deg = place_report.start_joint_error_deg;
            ctx.joint_link_pass = true;
            LOG_INFO << log_tag_ << "  [完整周期] candidate=" << candidate_index << " 抓取rank="
                     << (rank + 1) << " 放置rank=" << (place_rank + 1) << " 抓取等级="
                     << TrajectoryQualityGradeAlgorithm::GradeName(quality.grade) << " 放置等级="
                     << TrajectoryQualityGradeAlgorithm::GradeName(place_quality.grade)
                     << " 综合等级=" << TrajectoryQualityGradeAlgorithm::GradeName(cycle.grade);
            return true;
        }

        // 放置候选全部失败 → 下一抓取 rank。
        attempt.failure_reason = "放置候选全部未通过";
        ctx.attempts.push_back(attempt);
        LOG_INFO << log_tag_ << "    [5.3] 放置候选全部失败，尝试下一抓取rank";
        return false;
    }

    /**
     * @brief 把一条候选轨迹写到 output/trajectory_planning/candidates/。
     */
    bool WriteCandidate(const PipelineContext& ctx,
                        const std::string& name,
                        const JointTrajectory& trajectory,
                        const std::string& segment,
                        std::string& error) const
    {
        const std::string directory = JoinPath(output_dir_, "candidates");
        if (!EnsureDirectory(directory, error))
        {
            return false;
        }
        const nlohmann::json document = TrajectoryOutputFormatAlgorithm::BuildJointTrajectoryDocument(
            trajectory, segment, ctx.request_id, ctx.generation_id, ctx.workpiece_type,
            trajectory.time_step_s);
        return WriteTextFileAtomic(JoinPath(directory, name), document.dump(2) + "\n", error);
    }

    /**
     * @brief 打印 GPU 内部五步统计 [4.1]~[4.5]。
     */
    void LogPlanSteps(const CuroboPlanSteps& steps, size_t candidate_index, bool planned) const
    {
        LOG_INFO << log_tag_ << "  [4/6] GPU规划 candidate=" << candidate_index << " 请求轨迹="
                 << return_trajectories_ << " solvers=reused startup_warmup=completed";
        LOG_INFO << log_tag_ << "    [4.1] IK 种子=" << steps.ik_seeds << " 请求解="
                 << steps.ik_returned << " 收敛解=" << steps.ik_converged << " 耗时=" << steps.ik_ms
                 << "ms";
        LOG_INFO << log_tag_ << "    [4.2] 补位 复制=" << steps.padded;
        LOG_INFO << log_tag_ << "    [4.3] 图规划 结果="
                 << (steps.graph_skipped ? "未启用（配置关闭）" : (steps.graph_ok ? "通过" : "未找到"))
                 << " 耗时=" << steps.graph_ms << "ms";
        LOG_INFO << log_tag_ << "    [4.4] 优化 请求=" << steps.trajopt_requested << " 收敛="
                 << steps.trajopt_converged << " 耗时=" << steps.trajopt_ms << "ms";
        if (!planned && steps.failed_step == "trajopt" && steps.trajopt_converged == 0)
        {
            LOG_INFO << log_tag_ << "    [4.4] 最小位置误差=" << steps.position_error_mm
                     << "mm 姿态误差=" << steps.rotation_error_deg << "°";
        }
        LOG_INFO << log_tag_ << "    [4.5] 插值 返回=" << steps.interp_count << " 总耗时="
                 << steps.total_ms << "ms";
    }

    TrajectoryCollisionCheckAlgorithm checker_; ///< CPU 独立复核
    TrajectoryQualityGradeAlgorithm grader_;    ///< 三项评级
    CuroboPlanClient planner_;                  ///< cuRobo 服务客户端（启动绑定）
    GradeThresholds thresholds_;                ///< 评级阈值
    std::string output_dir_;                    ///< output/trajectory_planning
    bool evaluate_all_ = false;                 ///< 穷举模式（仅离线诊断）
    bool stop_on_first_cycle_ = true;           ///< 首个合格完整周期即停止
    double max_selection_ms_ = 15000.0;         ///< 已有解后的选优预算
    double no_solution_ms_ = 30000.0;           ///< 无解时的搜索时间预算
    int64_t max_no_solution_candidates_ = 12;   ///< 无解时的候选数预算
    int64_t return_trajectories_ = 4;           ///< 抓取段候选条数
    int64_t place_return_trajectories_ = 4;     ///< 放置段候选条数
    double endpoint_position_tolerance_mm_ = 1.0;  ///< 法兰端点位置容差
    double endpoint_rotation_tolerance_deg_ = 0.2; ///< 法兰端点姿态容差
    double endpoint_joint_tolerance_deg_ = 0.5;    ///< 放置段关节端点容差
};

REGISTER_STAGE("plan_full_cycle", PlanFullCycleStage);

} // namespace openmind::trajectory_plan
