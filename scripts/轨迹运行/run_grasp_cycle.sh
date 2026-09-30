#!/usr/bin/env bash
# 顺序执行一次完整抓取循环。实际执行固定为以下 8 步：
#
#   1. 打开夹爪
#      输入：GRIPPER_OPEN_RATIO、GRIPPER_SPEED、GRIPPER_FORCE（环境变量/本脚本默认值）。
#   2. 执行拍照位到抓取位的关节轨迹
#      输入：TRAJECTORY_MANIFEST、PHOTO_TO_GRASP_TRAJECTORY。
#      轨迹终点即 best_grasp.json 的 grasp_flange_pose…（抓取法兰位）。
#   3. 从抓取法兰位回退到原始抓取中心法兰位
#      输入：GRASP_RESULT_FILE（默认 output/grasp_generation/best_grasp.json）中的
#      original_grasp_center_flange_pose_xyzrxryrz_mm_deg。
#   4. 闭合夹爪
#      输入：GRIPPER_CLOSE_RATIO、GRIPPER_SPEED、GRIPPER_FORCE（环境变量/本脚本默认值）。
#   5. 从原始抓取中心前进到抓取法兰位
#      输入：GRASP_RESULT_FILE 中的 grasp_flange_pose_xyzrxryrz_mm_deg。
#      该目标位姿同时是第 2 步轨迹终点和第 6 步轨迹起点。
#   6. 执行抓取位到放置位的关节轨迹
#      输入：TRAJECTORY_MANIFEST、GRASP_TO_PLACE_TRAJECTORY。
#   7. 打开夹爪
#      输入：GRIPPER_OPEN_RATIO、GRIPPER_SPEED、GRIPPER_FORCE（环境变量/本脚本默认值）。
#   8. 回到拍照位
#      输入：PLANNER_CONFIG_FILE（默认 config/grasp_planner.json）中的
#      app.photo.q_deg。该字段是 AUBO 六轴关节角（度），通过
#      move_flange_pose_from_grasp_json.py 的 moveJoint 执行。
#
# 预检（不计入以上步骤）：先校验 manifest 的轨迹代次/SHA-256，以及第 3、5 步的
# GRASP_RESULT_FILE 和第 8 步的 PLANNER_CONFIG_FILE 字段格式；任何预检失败都不会驱动夹爪或机械臂。
#
# 每条命令都是阻塞执行的。只有上一条命令成功返回后，脚本才会执行下一步；
# 任意一步返回非 0，整个脚本立即停止。

set -Eeuo pipefail

# Hybrid环境提供 gripper_bt_trigger。允许用 HYBRID_SETUP 环境变量替换路径。
HYBRID_SETUP="${HYBRID_SETUP:-/opt/openmind/hybrid/hybrid_tools/scripts/run/hybrid_setup.bash}"
if [[ ! -f "${HYBRID_SETUP}" ]]; then
    echo "[错误] Hybrid环境脚本不存在: ${HYBRID_SETUP}" >&2
    exit 1
fi

# 某些ROS/Hybrid setup脚本会读取尚未定义的环境变量；source期间临时关闭
# nounset，完成后立即恢复本脚本的严格模式。
set +u
# shellcheck source=/opt/openmind/hybrid/hybrid_tools/scripts/run/hybrid_setup.bash
source "${HYBRID_SETUP}"
set -u

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

echo "[错误] 真机播放的 Python 脚本已移除，请用现场行为树直接读 output/trajectory_planning/" >&2
exit 2

# ======================== 可替换轨迹文件 ========================
# 五个输入路径集中在这里，也可以在启动脚本前设置同名环境变量。
# manifest 用来绑定两条轨迹的共同代次和 SHA-256，不提供限速或安全策略。
TRAJECTORY_MANIFEST="${TRAJECTORY_MANIFEST:-${PROJECT_ROOT}/output/trajectory_planning/trajectory_manifest.json}"
PHOTO_TO_GRASP_TRAJECTORY="${PHOTO_TO_GRASP_TRAJECTORY:-${PROJECT_ROOT}/output/trajectory_planning/joint_trajectory_photo_to_grasp.json}"
GRASP_TO_PLACE_TRAJECTORY="${GRASP_TO_PLACE_TRAJECTORY:-${PROJECT_ROOT}/output/trajectory_planning/joint_trajectory_grasp_to_place.json}"
GRASP_RESULT_FILE="${GRASP_RESULT_FILE:-${PROJECT_ROOT}/output/grasp_generation/best_grasp.json}"
PLANNER_CONFIG_FILE="${PLANNER_CONFIG_FILE:-${PROJECT_ROOT}/config/grasp_planner.json}"
# ===============================================================

PYTHON_BIN="${PYTHON_BIN:-python3}"
TRAJECTORY_RUNNER="${TRAJECTORY_RUNNER:-${SCRIPT_DIR}/play_full_cycle_preview.py}"
FLANGE_POSE_RUNNER="${FLANGE_POSE_RUNNER:-${SCRIPT_DIR}/move_flange_pose_from_grasp_json.py}"
GRIPPER_COMMAND="${GRIPPER_COMMAND:-gripper_bt_trigger}"

if ! command -v "${GRIPPER_COMMAND}" >/dev/null 2>&1; then
    echo "[错误] source Hybrid环境后仍未找到命令: ${GRIPPER_COMMAND}" >&2
    exit 1
fi

# 夹爪命令参数。
GRIPPER_SPEED="${GRIPPER_SPEED:-50.0}"
GRIPPER_FORCE="${GRIPPER_FORCE:-50.0}"
GRIPPER_OPEN_RATIO="${GRIPPER_OPEN_RATIO:-1.0}"
GRIPPER_CLOSE_RATIO="${GRIPPER_CLOSE_RATIO:-0.0}"

run_gripper() {
    local ratio="$1"
    "${GRIPPER_COMMAND}" \
        -r "${ratio}" \
        -s "${GRIPPER_SPEED}" \
        -f "${GRIPPER_FORCE}"
}

run_trajectory() {
    local route="$1"
    "${PYTHON_BIN}" "${TRAJECTORY_RUNNER}" \
        --manifest "${TRAJECTORY_MANIFEST}" \
        --photo-trajectory "${PHOTO_TO_GRASP_TRAJECTORY}" \
        --place-trajectory "${GRASP_TO_PLACE_TRAJECTORY}" \
        --route "${route}" \
        --expected-generation "${GENERATION_ID}" \
        --execute
}

run_flange_pose() {
    local field="$1"
    "${PYTHON_BIN}" "${FLANGE_POSE_RUNNER}" \
        --grasp-file "${GRASP_RESULT_FILE}" \
        --field "${field}" \
        --acc "${FLANGE_MOVE_ACC}" \
        --vel "${FLANGE_MOVE_VEL}" \
        --wait "${FLANGE_MOVE_WAIT_S}"
}

validate_flange_pose() {
    local field="$1"
    "${PYTHON_BIN}" "${FLANGE_POSE_RUNNER}" \
        --grasp-file "${GRASP_RESULT_FILE}" \
        --field "${field}" \
        --validate-only
}

run_photo_joint_position() {
    "${PYTHON_BIN}" "${FLANGE_POSE_RUNNER}" \
        --photo-config "${PLANNER_CONFIG_FILE}" \
        --joint-acc "${PHOTO_JOINT_MOVE_ACC}" \
        --joint-vel "${PHOTO_JOINT_MOVE_VEL}" \
        --wait "${PHOTO_JOINT_MOVE_WAIT_S}"
}

validate_photo_joint_position() {
    "${PYTHON_BIN}" "${FLANGE_POSE_RUNNER}" \
        --photo-config "${PLANNER_CONFIG_FILE}" \
        --validate-only
}

# 两段法兰点到点运动的 moveLine 参数；默认沿用 aubo_rpc.py 的默认值。
FLANGE_MOVE_ACC="${FLANGE_MOVE_ACC:-${AUBO_LINE_ACC:-1.2}}"
FLANGE_MOVE_VEL="${FLANGE_MOVE_VEL:-${AUBO_LINE_VEL:-0.25}}"
FLANGE_MOVE_WAIT_S="${FLANGE_MOVE_WAIT_S:-${AUBO_MOVE_WAIT:-3.0}}"

# 第 8 步回拍照位的 moveJoint 参数（q_deg 是关节角，不能使用 moveLine）。
PHOTO_JOINT_MOVE_ACC="${PHOTO_JOINT_MOVE_ACC:-${AUBO_JOINT_ACC:-1.0}}"
PHOTO_JOINT_MOVE_VEL="${PHOTO_JOINT_MOVE_VEL:-${AUBO_JOINT_VEL:-0.2}}"
PHOTO_JOINT_MOVE_WAIT_S="${PHOTO_JOINT_MOVE_WAIT_S:-${AUBO_MOVE_WAIT:-3.0}}"

# 打开夹爪之前先完整校验 manifest 和两条轨迹，并锁定本轮代次。
GENERATION_ID="$(
    "${PYTHON_BIN}" "${TRAJECTORY_RUNNER}" \
        --manifest "${TRAJECTORY_MANIFEST}" \
        --photo-trajectory "${PHOTO_TO_GRASP_TRAJECTORY}" \
        --place-trajectory "${GRASP_TO_PLACE_TRAJECTORY}" \
        --print-generation
)"
echo "[轨迹代次] ${GENERATION_ID}"

# 在任何实际运动前确认两个新输入位姿均存在且格式正确。
validate_flange_pose "original_grasp_center_flange_pose_xyzrxryrz_mm_deg"
validate_flange_pose "grasp_flange_pose_xyzrxryrz_mm_deg"
validate_photo_joint_position

echo "[1/8] 打开夹爪（ratio=${GRIPPER_OPEN_RATIO}, speed=${GRIPPER_SPEED}, force=${GRIPPER_FORCE}）"
run_gripper "${GRIPPER_OPEN_RATIO}"

echo "[2/8] 执行拍照位到抓取位轨迹（manifest=${TRAJECTORY_MANIFEST}; trajectory=${PHOTO_TO_GRASP_TRAJECTORY}）"
run_trajectory "photo_to_grasp"

echo "[3/8] 回退至原始抓取中心法兰位（file=${GRASP_RESULT_FILE}; field=original_grasp_center_flange_pose_xyzrxryrz_mm_deg）"
run_flange_pose "original_grasp_center_flange_pose_xyzrxryrz_mm_deg"

echo "[4/8] 闭合夹爪（ratio=${GRIPPER_CLOSE_RATIO}, speed=${GRIPPER_SPEED}, force=${GRIPPER_FORCE}）"
run_gripper "${GRIPPER_CLOSE_RATIO}"

echo "[5/8] 前进至抓取法兰位/第二段轨迹起点（file=${GRASP_RESULT_FILE}; field=grasp_flange_pose_xyzrxryrz_mm_deg）"
run_flange_pose "grasp_flange_pose_xyzrxryrz_mm_deg"

echo "[6/8] 执行抓取位到放置位轨迹（manifest=${TRAJECTORY_MANIFEST}; trajectory=${GRASP_TO_PLACE_TRAJECTORY}）"
run_trajectory "grasp_to_place"

echo "[7/8] 打开夹爪（ratio=${GRIPPER_OPEN_RATIO}, speed=${GRIPPER_SPEED}, force=${GRIPPER_FORCE}）"
run_gripper "${GRIPPER_OPEN_RATIO}"

echo "[8/8] 回拍照位（file=${PLANNER_CONFIG_FILE}; field=fixed_stations.photo.q_deg; joint_acc=${PHOTO_JOINT_MOVE_ACC}; joint_vel=${PHOTO_JOINT_MOVE_VEL}）"
run_photo_joint_position

echo "[完成] 完整抓取循环执行完毕"
