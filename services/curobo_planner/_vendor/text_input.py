"""标准 txt 输入加载器。

标准输入由一个 key=value 格式的 `.txt` 文件和一张 uint16 深度图组成。
外部输入统一使用毫米和角度；加载器会把深度图反投影并转换到基座坐标系。
位姿优先使用 x y z rx ry rz；也兼容 4x4 矩阵，矩阵平移列 tx/ty/tz 使用毫米。
规划器内部统一使用机械臂基座坐标系 {B} 和米/弧度。

欧拉角约定：rx/ry/rz 为固定轴 X/Y/Z 角度，组合矩阵 R = Rz @ Ry @ Rx。
"""
from __future__ import annotations

from pathlib import Path
import re
import numpy as np

from .types import GraspCandidate, PlanRequest

MM_TO_M = 0.001


def load_plan_request_txt(
    path: str | Path,
    config_path: str | Path | None = None,
):
    """读取动态 txt，并与同项目 config.yaml 的固定字段合并。

    返回：
        (PlanRequest, safety_planes)，其中 safety_planes 可为空列表。
    """
    path = Path(path).resolve()
    from .project_config import load_effective_request_values

    kv = load_effective_request_values(path, config_path)
    base = path.parent

    cloud = _load_depth_cloud_B(kv, base)

    n_cands = int(_required(kv, "grasp_candidate_count"))
    candidates = []
    for i in range(n_cands):
        prefix = f"grasp_candidate_{i}_"
        candidates.append(GraspCandidate(
            T_B_F_grasp=_pose_from_kv(kv, prefix, "抓取候选法兰位姿"),
            score=float(kv.get(prefix + "score", "1.0")),
            workpiece_id=int(kv.get(prefix + "workpiece_id", "-1")),
        ))
    if not candidates:
        raise ValueError("grasp_candidate_count 必须 >= 1")
    roi_c = _optional_vector(kv.get("bin_roi_center_B_mm"), 3, "bin_roi_center_B_mm")
    roi_h = _optional_vector(kv.get("bin_roi_half_extents_mm"), 3, "bin_roi_half_extents_mm")
    if (roi_c is None) != (roi_h is None):
        raise ValueError("bin_roi_center_B_mm 和 bin_roi_half_extents_mm 必须同时提供，或同时省略")
    platform_negative = _optional_vector(
        kv.get("base_platform_negative_extent_xyz_mm"), 3,
        "base_platform_negative_extent_xyz_mm",
    )
    platform_positive = _optional_vector(
        kv.get("base_platform_positive_extent_xyz_mm"), 3,
        "base_platform_positive_extent_xyz_mm",
    )
    if (platform_negative is None) != (platform_positive is None):
        raise ValueError("基座平台正负 XYZ 延伸必须同时提供")
    if platform_negative is not None:
        if np.any(platform_negative < 0.0) or np.any(platform_positive < 0.0):
            raise ValueError("基座平台六个延伸值必须非负")
        if np.any(platform_negative + platform_positive <= 0.0):
            raise ValueError("基座平台每个轴的正负延伸之和必须大于 0")

    place_pose = None
    if "T_B_F_place_xyzrpy" in kv or "T_B_F_place" in kv:
        place_pose = _pose_from_kv(
            kv, "", "放置法兰位姿", matrix_key="T_B_F_place",
            xyzrpy_key="T_B_F_place_xyzrpy",
        )

    req = PlanRequest(
        q_start=np.deg2rad(_vector(_required(kv, "q_start_deg"), 6, "q_start_deg")),
        grasp_candidates=candidates,
        cloud_B=cloud,
        T_B_O_target=_pose_from_kv(kv, "", "目标工件位姿", matrix_key="T_B_O_target",
                                   xyzrpy_key="T_B_O_target_xyzrpy"),
        target_extents=_vector(_required(kv, "target_extents_mm"), 3, "target_extents_mm") * MM_TO_M,
        T_B_F_place=place_pose,
        return_to_start=_bool(kv.get("return_to_start", "true")),
        base_platform_min_B=(
            -platform_negative * MM_TO_M
            if platform_negative is not None else None
        ),
        base_platform_max_B=(
            platform_positive * MM_TO_M
            if platform_positive is not None else None
        ),
        bin_roi_center_B=roi_c * MM_TO_M if roi_c is not None else None,
        bin_roi_half_extents=roi_h * MM_TO_M if roi_h is not None else None,
    )
    return req, _safety_planes(kv)


def _read_key_values(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    pending_key = None
    pending_value = []
    bracket_depth = 0
    for lineno, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if pending_key is not None:
            pending_value.append(line)
            bracket_depth += line.count("[") - line.count("]")
            if bracket_depth <= 0:
                out[pending_key] = " ".join(pending_value)
                pending_key = None
                pending_value = []
            continue
        if "=" not in line:
            raise ValueError(f"{path}:{lineno}: 需要 'key = value' 格式")
        key, value = line.split("=", 1)
        key = key.strip()
        if not re.fullmatch(r"[A-Za-z0-9_]+", key):
            raise ValueError(f"{path}:{lineno}: 非法字段名 {key!r}")
        value = value.strip()
        bracket_depth = value.count("[") - value.count("]")
        if bracket_depth > 0:
            pending_key = key
            pending_value = [value]
        else:
            out[key] = value
    if pending_key is not None:
        raise ValueError(f"{path}: 字段 {pending_key} 的列表缺少闭合 ']'")
    return out


def _load_depth_cloud_B(kv: dict[str, str], base: Path) -> np.ndarray:
    """从 uint16 深度图反投影环境点，并变换到基座坐标系 {B}。

    深度图像素单位为毫米；返回点坐标单位为米。
    相机坐标按常用 pinhole/OpenCV 约定：X 向右、Y 向下、Z 沿光轴向前。
    """
    depth_path = Path(_required(kv, "depth_path"))
    if not depth_path.is_absolute():
        depth_path = base / depth_path
    depth_format = kv.get("depth_format", "png16_mm").strip().lower()
    if depth_format != "png16_mm":
        raise ValueError(f"不支持的 depth_format: {depth_format!r}，当前仅支持 png16_mm")

    try:
        from PIL import Image
    except ImportError as exc:
        raise ValueError("读取 depth_path 需要安装 Pillow") from exc

    depth_mm = np.asarray(Image.open(depth_path))
    if depth_mm.ndim != 2:
        raise ValueError(f"深度图必须是单通道 uint16: {depth_path}")
    if not np.issubdtype(depth_mm.dtype, np.integer):
        raise ValueError(f"深度图必须是整数毫米图: {depth_path}, dtype={depth_mm.dtype}")

    stride = int(float(kv.get("depth_stride", "1")))
    if stride < 1:
        raise ValueError("depth_stride 必须 >= 1")
    if stride > 1:
        depth_mm = depth_mm[::stride, ::stride]

    fx, fy, cx, cy = _vector(_required(kv, "camera_intrinsics"), 4, "camera_intrinsics")
    if fx <= 0 or fy <= 0:
        raise ValueError("camera_intrinsics 中 fx/fy 必须大于 0")
    if stride > 1:
        fx /= stride
        fy /= stride
        cx /= stride
        cy /= stride

    min_mm = float(kv.get("depth_min_mm", "1"))
    max_mm = float(kv.get("depth_max_mm", str(np.inf)))
    valid = (depth_mm > 0) & (depth_mm >= min_mm) & (depth_mm <= max_mm)
    if not np.any(valid):
        raise ValueError(f"深度图没有有效深度点: {depth_path}")

    v, u = np.nonzero(valid)
    z = depth_mm[valid].astype(np.float64) * MM_TO_M
    x = (u.astype(np.float64) - cx) * z / fx
    y = (v.astype(np.float64) - cy) * z / fy
    cloud_C = np.column_stack((x, y, z))

    T_B_F = _pose_from_kv(kv, "", "拍照瞬间法兰位姿",
                          matrix_key="T_B_F_photo",
                          xyzrpy_key="T_B_F_photo_xyzrpy")
    T_F_C = _matrix4_mm(_required(kv, "T_cam2flange"))
    T_B_C = T_B_F @ T_F_C
    cloud_B = cloud_C @ T_B_C[:3, :3].T + T_B_C[:3, 3]
    return cloud_B.astype(float, copy=False)


def _required(kv: dict[str, str], key: str) -> str:
    if key not in kv:
        raise ValueError(f"缺少必填字段: {key}")
    return kv[key]


def _pose_from_kv(kv: dict[str, str], prefix: str, label: str,
                  matrix_key: str | None = None,
                  xyzrpy_key: str | None = None) -> np.ndarray:
    """从 key-value 中读取位姿。

    优先读取 xyzrpy，兼容旧 4x4 矩阵。
    prefix 用于抓取候选，例如 `grasp_candidate_0_`。
    """
    xyzrpy_key = xyzrpy_key or (prefix + "xyzrpy")
    matrix_key = matrix_key or (prefix + "T_B_F_grasp")
    if xyzrpy_key in kv:
        return _xyzrpy_mm_deg(kv[xyzrpy_key], xyzrpy_key)
    if matrix_key in kv:
        return _matrix4_mm(kv[matrix_key])
    raise ValueError(f"缺少{label}: 需要字段 {xyzrpy_key} 或 {matrix_key}")


def _numbers(text: str) -> np.ndarray:
    # txt 支持单行数值，也支持 [1, 2] / [[...], [...]] 多行列表。
    normalized = text.replace("[", " ").replace("]", " ").replace(",", " ")
    vals = np.fromstring(normalized, sep=" ", dtype=float)
    if vals.size == 0:
        raise ValueError(f"需要数值字段，实际为 {text!r}")
    return vals


def _vector(text: str, n: int, name: str) -> np.ndarray:
    vals = _numbers(text)
    if vals.size != n:
        raise ValueError(f"{name} 需要 {n} 个数值，实际为 {vals.size} 个")
    return vals


def _optional_vector(text: str | None, n: int, name: str):
    if text is None:
        return None
    return _vector(text, n, name)


def _matrix4(text: str) -> np.ndarray:
    vals = _numbers(text)
    if vals.size != 16:
        raise ValueError(f"4x4 位姿矩阵需要按行优先填写 16 个数，实际为 {vals.size} 个")
    return vals.reshape(4, 4)


def _matrix4_mm(text: str) -> np.ndarray:
    T = _matrix4(text)
    T = T.copy()
    T[:3, 3] *= MM_TO_M
    return T


def _xyzrpy_mm_deg(text: str, name: str) -> np.ndarray:
    vals = _vector(text, 6, name)
    x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg = vals
    rx, ry, rz = np.deg2rad([rx_deg, ry_deg, rz_deg])
    T = np.eye(4)
    T[:3, :3] = _rot_z(rz) @ _rot_y(ry) @ _rot_x(rx)
    T[:3, 3] = np.array([x_mm, y_mm, z_mm], dtype=float) * MM_TO_M
    return T


def _rot_x(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def _rot_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def _rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def _bool(text: str) -> bool:
    value = text.strip().lower()
    if value in ("1", "true", "yes", "y", "on"):
        return True
    if value in ("0", "false", "no", "n", "off"):
        return False
    raise ValueError(f"需要布尔值 true/false，实际为 {text!r}")


def _safety_planes(kv: dict[str, str]):
    planes = []
    keys = sorted(
        (k for k in kv if re.fullmatch(r"safety_plane_[0-9]+", k)),
        key=lambda k: int(k.rsplit("_", 1)[1]),
    )
    for key in keys:
        vals = _vector(kv[key], 4, key)
        n = vals[:3]
        norm = float(np.linalg.norm(n))
        if norm < 1e-12:
            raise ValueError(f"{key}: 平面法向量不能为零")
        planes.append((n / norm, float(vals[3] * MM_TO_M / norm)))
    return planes
