/**
 * @file grasp_output_format_algorithm.cpp
 * @brief 抓取结果 JSON 与日志行。
 */

#include "trajectory_plan/algorithm/grasp_output_format_algorithm.h"

#include <iomanip>
#include <sstream>

#include "trajectory_plan/tools/json_util.h"

namespace openmind::trajectory_plan
{
namespace
{

/**
 * @brief 一组位姿排成 6 个数的扁平数组：[x, y, z, rx, ry, rz]。
 *
 * 平移单位 mm，旋转单位 degree，固定轴 XYZ 欧拉角，即
 * R = Rz(rz) · Ry(ry) · Rx(rx)。需要 4x4 的下游自行用该约定还原。
 */
nlohmann::json PoseJson(const Pose& pose)
{
    return nlohmann::json::array({pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz});
}

} // namespace

nlohmann::json GraspOutputFormatAlgorithm::BuildGraspResultDocument(const std::vector<GraspResult>& candidates,
                                                                    const std::string& request_id,
                                                                    const std::string& generation_id,
                                                                    const std::string& workpiece_type)
{
    nlohmann::json list = nlohmann::json::array();
    for (const GraspResult& candidate : candidates)
    {
        nlohmann::json item;
        // 分数：来自 NPZ 离线标注，越大越好；candidates 已按它降序排好。
        item["score"] = candidate.score;
        // 夹爪开口，单位 m：抓这一组时夹爪需要张开的宽度。
        item["gripper_width"] = candidate.gripper_width;
        // 咬合中心（未退让）。
        item["grasp_tcp"] = PoseJson(candidate.grasp_tcp);
        item["grasp_flange"] = PoseJson(candidate.grasp_flange);
        // 沿 TCP 局部 Z 退让 84 mm 之后（第二步轨迹规划用 trajectory_grasp_flange）。
        item["trajectory_grasp_tcp"] = PoseJson(candidate.trajectory_grasp_tcp);
        item["trajectory_grasp_flange"] = PoseJson(candidate.trajectory_grasp_flange);
        list.push_back(item);
    }
    nlohmann::json document;
    // 根级带本轮关联：第二流程据此校验请求与代次一致，不凭文件存在判断本轮成功。
    document["request_id"] = request_id;
    document["generation_id"] = generation_id;
    document["workpiece_type"] = workpiece_type;
    document["candidates"] = list;
    return document;
}

nlohmann::json GraspOutputFormatAlgorithm::BuildGraspPreview(const std::vector<GraspResult>& candidates,
                                                             double clearance_along_tcp_z_mm,
                                                             int64_t selected_candidate_index)
{
    nlohmann::json list = nlohmann::json::array();
    for (size_t index = 0; index < candidates.size(); ++index)
    {
        const GraspResult& candidate = candidates[index];
        nlohmann::json item;
        // 保留原编号：显示排序不得重新编号，也不默认把第 0 个当成最终选中候选。
        item["candidate_index"] = static_cast<int64_t>(index);
        item["score"] = candidate.score;
        item["gripper_width"] = candidate.gripper_width;
        item["grasp_tcp"] = PoseJson(candidate.grasp_tcp);
        item["grasp_flange"] = PoseJson(candidate.grasp_flange);
        item["trajectory_grasp_tcp"] = PoseJson(candidate.trajectory_grasp_tcp);
        item["trajectory_grasp_flange"] = PoseJson(candidate.trajectory_grasp_flange);
        list.push_back(item);
    }
    nlohmann::json preview;
    preview["camera_clearance_along_tcp_z_mm"] = clearance_along_tcp_z_mm;
    preview["selected_candidate_index"] = selected_candidate_index;
    preview["candidates"] = list;
    return preview;
}

std::string GraspOutputFormatAlgorithm::BuildCandidateLogLines(const std::vector<GraspResult>& candidates)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6);
    for (size_t index = 0; index < candidates.size(); ++index)
    {
        const GraspResult& candidate = candidates[index];
        stream << "candidate " << index << "/" << candidates.size()
               << " score = " << candidate.score << "\n";
        stream << "  grasp_tcp xyzrxryrz_mm_deg = " << FormatPose(candidate.grasp_tcp) << "\n";
        stream << "  grasp_flange xyzrxryrz_mm_deg = " << FormatPose(candidate.grasp_flange) << "\n";
        stream << "  trajectory_grasp_tcp xyzrxryrz_mm_deg = "
               << FormatPose(candidate.trajectory_grasp_tcp) << "\n";
        stream << "  trajectory_grasp_flange xyzrxryrz_mm_deg = "
               << FormatPose(candidate.trajectory_grasp_flange) << "\n";
        stream << "  gripper_width = " << candidate.gripper_width << "\n";
    }
    return stream.str();
}

} // namespace openmind::trajectory_plan
