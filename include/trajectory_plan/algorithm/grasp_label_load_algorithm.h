#ifndef HYBRID_TRAJECTORY_PLAN_GRASP_LABEL_LOAD_ALGORITHM_H
#define HYBRID_TRAJECTORY_PLAN_GRASP_LABEL_LOAD_ALGORITHM_H

/**
 * @file grasp_label_load_algorithm.h
 * @brief 读取工件外部离线 NPZ 标注：开口上限过滤 + 分数降序 + 截断最多 10 条。
 */

#include <cstdint>
#include <string>
#include <vector>

#include "trajectory_plan/algorithm/grasp_result.h"

namespace openmind::trajectory_plan
{

/**
 * @brief 离线抓取标注加载（纯计算，不关心基座位姿与退让）。
 *
 * NPZ 字段：translations / rotations / widths / scores，工件系 {O}，单位米。
 */
class GraspLabelLoadAlgorithm
{
  public:
    GraspLabelLoadAlgorithm() = default;
    GraspLabelLoadAlgorithm(const GraspLabelLoadAlgorithm&) = delete;
    GraspLabelLoadAlgorithm& operator=(const GraspLabelLoadAlgorithm&) = delete;

    /**
     * @brief 设置一份 NPZ 的来源并读取标注。
     * @param npz_path NPZ 文件路径
     * @param max_width_m 夹爪开口上限（米）；无上限传 0 或负值
     * @param error 失败原因（成功时为空）
     * @return 成功返回 true
     * @note 读取后内部保留按分数降序、宽度过滤后的标注列表。
     */
    bool LoadLabels(const std::string& npz_path, double max_width_m, std::string& error);

    /**
     * @brief 当前已加载且过滤后的标注（按分数降序，最多 candidate_count 条）。
     * @param candidate_count 截断上限（超出丢弃，不足全给）
     */
    std::vector<GraspLabel> TopLabels(int64_t candidate_count) const;

    /** @brief 已加载文件的原始合格标注数量（截断前）。 */
    int64_t EligibleCount() const;

    /** @brief NPZ 中的原始标注总数（过滤前）。 */
    int64_t RawLabelCount() const;

    /** @brief 因开口 <=0 或超过上限被过滤掉的标注数量。 */
    int64_t WidthFilteredCount() const;

    /** @brief 已加载文件路径。 */
    const std::string& NpzPath() const;

  private:
    std::vector<GraspLabel> labels_; ///< 按分数降序、开口过滤后的全部标注
    std::string npz_path_;           ///< 已加载文件路径
    int64_t raw_label_count_ = 0;    ///< NPZ 原始标注总数
    int64_t width_filtered_count_ = 0; ///< 宽度过滤掉的标注数
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_GRASP_LABEL_LOAD_ALGORITHM_H
