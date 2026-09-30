#ifndef HYBRID_TRAJECTORY_PLAN_GRASP_OUTPUT_FORMAT_ALGORITHM_H
#define HYBRID_TRAJECTORY_PLAN_GRASP_OUTPUT_FORMAT_ALGORITHM_H

/**
 * @file grasp_output_format_algorithm.h
 * @brief 抓取候选 → grasp_result.json 文档与日志行（纯排版，不落盘）。
 */

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

#include "trajectory_plan/algorithm/grasp_result.h"

namespace openmind::trajectory_plan
{

/**
 * @brief 抓取输出排版（纯计算）。
 */
class GraspOutputFormatAlgorithm
{
  public:
    GraspOutputFormatAlgorithm() = default;
    GraspOutputFormatAlgorithm(const GraspOutputFormatAlgorithm&) = delete;
    GraspOutputFormatAlgorithm& operator=(const GraspOutputFormatAlgorithm&) = delete;

    /**
     * @brief 生成 grasp_result.json 文档。
     *
     * 根级包含 request_id、generation_id、workpiece_type 和 candidates；
     * candidates 按分数从高到低排列（candidates[0] 最高），最多 10 个。
     * 每个元素只有六项：score、gripper_width 与四组位姿。每组位姿是
     * [x, y, z, rx, ry, rz] 六个数（mm / degree，固定轴 XYZ，
     * R = Rz(rz)·Ry(ry)·Rx(rx)），不再附带等价的 4x4 齐次阵。
     * JSON 中只存结构化结果，不混入日志文本。
     *
     * @param candidates 最多 10 组候选（已按分数降序）
     * @param request_id 本轮请求标识
     * @param generation_id 本轮代次标识
     * @param workpiece_type 工件类型
     * @return JSON 文档
     */
    static nlohmann::json BuildGraspResultDocument(const std::vector<GraspResult>& candidates,
                                                   const std::string& request_id,
                                                   const std::string& generation_id,
                                                   const std::string& workpiece_type);

    /**
     * @brief 生成 manifest 的 grasp_preview 节（第三流程夹爪预览元数据）。
     *
     * 候选保留原编号，不因显示排序重新编号；选中候选由第二流程填入。
     * @param candidates 本轮候选（原顺序）
     * @param clearance_along_tcp_z_mm 本轮实际采用的退让量（mm，带符号）
     * @param selected_candidate_index 最终选中的候选原编号；未选中传 -1
     */
    static nlohmann::json BuildGraspPreview(const std::vector<GraspResult>& candidates,
                                            double clearance_along_tcp_z_mm,
                                            int64_t selected_candidate_index);

    /**
     * @brief 生成候选日志行（key=value，字段名与 JSON 一致）。
     * @param candidates 最多 10 组候选
     * @return 多行文本（每候选一组）
     */
    static std::string BuildCandidateLogLines(const std::vector<GraspResult>& candidates);
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_GRASP_OUTPUT_FORMAT_ALGORITHM_H
