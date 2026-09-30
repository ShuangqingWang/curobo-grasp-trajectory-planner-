#!/usr/bin/env python3
"""逐项准备历史拍照数据到轨迹规划 input；不启动、不触发任何服务。"""

from __future__ import annotations

import math
import re
import shutil
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path


# ============================================================================
# 仅在这里修改两个绝对路径。
# ============================================================================
DATA_ROOT = Path("/home/vecow/workspace/D405相机推理和拼接-新算法/vision_trigger/data_1")
INPUT_DIR = Path("/home/vecow/workspace/轨迹规划/grasp_planner_pkg/input")


DETECTION_RELATIVE_DIR = Path("pipeline_data/detection_stage")
RESULT_RELATIVE_PATH = Path("pipeline_data/result.txt")
INPUT_IMAGES = (
    "raw_depth.tiff",
    "raw_depth_left.tiff",
    "raw_depth_right.tiff",
    "raw_color.png",
)
WORKPIECE_HEADER = re.compile(r"^Workpiece\[(\d+)\]\s*-\s*T_base_obj:\s*$")
WORKPIECE_CODE = re.compile(r"-(QR\d+)$", re.IGNORECASE)
JOINTS = re.compile(r"^T_base_flange_joint_angles_deg:\s*\[([^]]+)\]\s*$")


@dataclass(frozen=True)
class TestItem:
    folder: Path
    workpiece_type: str
    workpiece_index: int
    workpiece_count: int
    t_base_obj: list[list[float]]
    t_base_flange: list[list[float]]
    joint_angles_deg: list[float]


def parse_matrix(lines: list[str], start: int, label: str) -> list[list[float]]:
    """从标题下一行起精确读取一个 4×4 数值矩阵。"""
    matrix: list[list[float]] = []
    index = start + 1
    while index < len(lines) and len(matrix) < 4:
        text = lines[index].strip()
        index += 1
        if not text:
            continue
        try:
            row = [float(value) for value in text.split()]
        except ValueError as error:
            raise ValueError(f"{label} 的第 {len(matrix) + 1} 行不是数值: {text}") from error
        if len(row) != 4 or not all(math.isfinite(value) for value in row):
            raise ValueError(f"{label} 必须是有限的 4×4 矩阵: {text}")
        matrix.append(row)
    if len(matrix) != 4:
        raise ValueError(f"{label} 缺少完整 4×4 矩阵")
    return matrix


def parse_folder(folder: Path) -> list[TestItem]:
    """解析一次拍照文件夹，展开为其中的多个工件测试项。"""
    code_match = WORKPIECE_CODE.search(folder.name)
    if not code_match:
        raise ValueError("文件夹名末尾缺少 -QRxxxx 工件号")
    workpiece_type = code_match.group(1).upper()
    detection = folder / DETECTION_RELATIVE_DIR
    result_path = folder / RESULT_RELATIVE_PATH
    missing = [str(detection / name) for name in INPUT_IMAGES if not (detection / name).is_file()]
    if not result_path.is_file():
        missing.append(str(result_path))
    if missing:
        raise ValueError("缺少必需文件:\n  " + "\n  ".join(missing))

    lines = result_path.read_text(encoding="utf-8", errors="strict").splitlines()
    flange_headers = [index for index, line in enumerate(lines) if line.strip() == "T_base_flange:"]
    if len(flange_headers) != 1:
        raise ValueError(f"T_base_flange 应恰好出现一次，实际 {len(flange_headers)} 次")
    t_base_flange = parse_matrix(lines, flange_headers[0], "T_base_flange")

    joint_values: list[float] | None = None
    for line in lines:
        match = JOINTS.match(line.strip())
        if not match:
            continue
        if joint_values is not None:
            raise ValueError("T_base_flange_joint_angles_deg 出现多次")
        joint_values = [float(value.strip()) for value in match.group(1).split(",")]
    if joint_values is None or len(joint_values) != 6 or not all(math.isfinite(value) for value in joint_values):
        raise ValueError("T_base_flange_joint_angles_deg 必须恰好包含 6 个有限数值")

    workpieces: list[tuple[int, list[list[float]]]] = []
    for index, line in enumerate(lines):
        match = WORKPIECE_HEADER.match(line.strip())
        if match:
            workpieces.append((int(match.group(1)), parse_matrix(lines, index, f"Workpiece[{match.group(1)}] T_base_obj")))
    if not workpieces:
        raise ValueError("result.txt 未找到任何 Workpiece[n] - T_base_obj")

    return [
        TestItem(folder, workpiece_type, workpiece_index, len(workpieces), t_base_obj,
                 t_base_flange, joint_values)
        for workpiece_index, t_base_obj in workpieces
    ]


def matrix_text(matrix: list[list[float]]) -> str:
    return "\n".join(" ".join(f"{value:.12g}" for value in row) for row in matrix)


def request_text(item: TestItem) -> str:
    joints = ", ".join(f"{value:.12g}" for value in item.joint_angles_deg)
    return f"""# 由 scripts/批量拍照数据测试/prepare_dataset_inputs.py 准备；不代表已触发规划
depth_main_path = raw_depth.tiff
depth_left_path = raw_depth_left.tiff
depth_right_path = raw_depth_right.tiff
rgb_main_path = raw_color.png

workpiece_type = {item.workpiece_type}

T_base_flange:
{matrix_text(item.t_base_flange)}

T_base_flange_joint_angles_deg: [{joints}]

T_B_O_target =
{matrix_text(item.t_base_obj)}
"""


def copy_file_atomic(source: Path, target: Path) -> None:
    """完整复制到同目录临时文件后原子替换，避免读取端见到半个文件。"""
    with tempfile.NamedTemporaryFile(dir=target.parent, prefix=f".{target.name}.", delete=False) as tmp:
        temporary = Path(tmp.name)
    try:
        shutil.copy2(source, temporary)
        temporary.replace(target)
    finally:
        if temporary.exists():
            temporary.unlink()


def prepare(item: TestItem) -> None:
    detection = item.folder / DETECTION_RELATIVE_DIR
    INPUT_DIR.mkdir(parents=True, exist_ok=True)
    for name in INPUT_IMAGES:
        copy_file_atomic(detection / name, INPUT_DIR / name)
    request_path = INPUT_DIR / "request.txt"
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=INPUT_DIR,
                                     prefix=".request.txt.", delete=False) as tmp:
        temporary = Path(tmp.name)
        tmp.write(request_text(item))
    try:
        temporary.replace(request_path)
    finally:
        if temporary.exists():
            temporary.unlink()


def main() -> int:
    if not DATA_ROOT.is_dir():
        print(f"错误：DATA_ROOT 不存在或不是目录: {DATA_ROOT}", file=sys.stderr)
        return 2
    items: list[TestItem] = []
    skipped: list[str] = []
    for folder in sorted(path for path in DATA_ROOT.iterdir() if path.is_dir()):
        try:
            items.extend(parse_folder(folder))
        except ValueError as error:
            skipped.append(f"{folder.name}: {error}")
    if not items:
        print("错误：没有找到可准备的工件测试项", file=sys.stderr)
        return 2

    print(f"已扫描拍照根目录：{DATA_ROOT}")
    print(f"有效工件测试项：{len(items)}；input 目录：{INPUT_DIR}")
    if skipped:
        print(f"跳过不完整文件夹：{len(skipped)} 个（仅显示前 10 个）")
        for message in skipped[:10]:
            print(f"  - {message}")
    print("\n本脚本不会启动服务、发送 TCP 或触发规划。")
    print("每次按 Enter：只覆盖 input 的四张图和 request.txt；然后由你手动触发规划。")

    for sequence, item in enumerate(items, start=1):
        try:
            command = input(
                f"\n[{sequence}/{len(items)}] 下一项：{item.folder.name} / "
                f"Workpiece[{item.workpiece_index}]。按 Enter 准备，输入 q 后 Enter 退出："
            ).strip().lower()
        except EOFError:
            print("\n收到 EOF，退出；未准备下一项。")
            return 0
        if command in {"q", "quit", "exit"}:
            print("已退出；未准备下一项。")
            return 0
        try:
            prepare(item)
        except OSError as error:
            print(f"错误：准备 input 失败: {error}", file=sys.stderr)
            return 1
        print("\n[input 已准备完成]")
        print(f"  拍照文件夹: {item.folder.name}")
        print(f"  工件号: {item.workpiece_type}")
        print(f"  工件位姿: Workpiece[{item.workpiece_index}] / 本拍照文件夹共 {item.workpiece_count} 个")
        print(f"  图片来源: {item.folder / DETECTION_RELATIVE_DIR}")
        print("  已覆盖: raw_depth.tiff, raw_depth_left.tiff, raw_depth_right.tiff, raw_color.png, request.txt")
        print("  下一步: 请手动触发规划；完成后返回此窗口按 Enter 准备下一项。")

    print("\n全部工件位姿均已准备完成。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
