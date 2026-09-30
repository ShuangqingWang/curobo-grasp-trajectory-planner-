"""加载同一台设备上固定的规划、相机标定和环境配置。"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
from typing import Any

import numpy as np
import yaml


FIXED_REQUEST_KEYS = frozenset({
    "planner_backend",
    "robot_kinematics",
    "camera_intrinsics",
    "depth_min_mm",
    "depth_max_mm",
    "T_cam2flange",
    "return_to_start",
    "planner_voxel_size_mm",
    "planner_target_dilation_mm",
    "base_platform_negative_extent_xyz_mm",
    "base_platform_positive_extent_xyz_mm",
    # 已废弃的旧字段仍列入固定字段，防止 request.txt 偷偷恢复旧语义。
    "base_platform_size_mm",
})


def _section(document: dict[str, Any], name: str) -> dict[str, Any]:
    value = document.get(name)
    if not isinstance(value, dict):
        raise ValueError(f"config.yaml 缺少 {name} 配置段")
    return value


def _array(value: Any, shape: tuple[int, ...], name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} 必须是形状 {shape} 的有限数值")
    return result


def _scalar(value: Any, name: str) -> float:
    result = float(value)
    if not np.isfinite(result):
        raise ValueError(f"{name} 必须是有限数值")
    return result


def _format_number(value: float) -> str:
    return f"{float(value):.12g}"


def _format_array(value: np.ndarray) -> str:
    if value.ndim == 1:
        return "[" + ", ".join(_format_number(x) for x in value) + "]"
    return "[" + ", ".join(_format_array(row) for row in value) + "]"


@dataclass(frozen=True)
class ProjectRuntimeConfig:
    planner_backend: str
    robot_kinematics: str
    camera_intrinsics: np.ndarray
    depth_min_mm: float
    depth_max_mm: float
    T_cam2flange_mm: np.ndarray
    return_to_start: bool
    planner_voxel_size_mm: float
    planner_target_dilation_mm: float
    base_platform_negative_extent_xyz_mm: np.ndarray
    base_platform_positive_extent_xyz_mm: np.ndarray
    source_path: Path
    source_sha256: str

    @property
    def base_platform_min_xyz_mm(self) -> np.ndarray:
        return -self.base_platform_negative_extent_xyz_mm

    @property
    def base_platform_max_xyz_mm(self) -> np.ndarray:
        return self.base_platform_positive_extent_xyz_mm

    @property
    def base_platform_center_xyz_mm(self) -> np.ndarray:
        return 0.5 * (
            self.base_platform_max_xyz_mm + self.base_platform_min_xyz_mm
        )

    @property
    def base_platform_size_xyz_mm(self) -> np.ndarray:
        return self.base_platform_max_xyz_mm - self.base_platform_min_xyz_mm

    def document(self) -> dict[str, Any]:
        return {
            "planner_backend": self.planner_backend,
            "robot_kinematics": self.robot_kinematics,
            "camera_intrinsics": self.camera_intrinsics.tolist(),
            "depth_min_mm": self.depth_min_mm,
            "depth_max_mm": self.depth_max_mm,
            "T_cam2flange": self.T_cam2flange_mm.tolist(),
            "return_to_start": self.return_to_start,
            "planner_voxel_size_mm": self.planner_voxel_size_mm,
            "planner_target_dilation_mm": self.planner_target_dilation_mm,
            "base_platform": {
                "frame": "base_link",
                "geometry_type": "axis_aligned_box",
                "negative_extent_xyz_mm": (
                    self.base_platform_negative_extent_xyz_mm.tolist()
                ),
                "positive_extent_xyz_mm": (
                    self.base_platform_positive_extent_xyz_mm.tolist()
                ),
                "box_min_xyz_mm": self.base_platform_min_xyz_mm.tolist(),
                "box_max_xyz_mm": self.base_platform_max_xyz_mm.tolist(),
                "center_xyz_mm": self.base_platform_center_xyz_mm.tolist(),
                "size_xyz_mm": self.base_platform_size_xyz_mm.tolist(),
            },
            "config_path": str(self.source_path),
            "config_sha256": self.source_sha256,
        }

    def request_values(self) -> dict[str, str]:
        """转成现有规划内核可直接消费的 key=value 表示。"""
        return {
            "planner_backend": self.planner_backend,
            "robot_kinematics": self.robot_kinematics,
            "camera_intrinsics": _format_array(self.camera_intrinsics),
            "depth_min_mm": _format_number(self.depth_min_mm),
            "depth_max_mm": _format_number(self.depth_max_mm),
            "T_cam2flange": _format_array(self.T_cam2flange_mm),
            "return_to_start": str(self.return_to_start).lower(),
            "planner_voxel_size_mm": _format_number(self.planner_voxel_size_mm),
            "planner_target_dilation_mm": _format_number(
                self.planner_target_dilation_mm
            ),
            "base_platform_negative_extent_xyz_mm": _format_array(
                self.base_platform_negative_extent_xyz_mm
            ),
            "base_platform_positive_extent_xyz_mm": _format_array(
                self.base_platform_positive_extent_xyz_mm
            ),
        }


def load_project_runtime_config(path: str | Path) -> ProjectRuntimeConfig:
    source = Path(path).resolve()
    if not source.is_file():
        raise FileNotFoundError(f"项目配置不存在: {source}")
    raw = source.read_bytes()
    document = yaml.safe_load(raw) or {}
    if int(document.get("schema_version", 0)) != 3:
        raise ValueError(f"{source}: schema_version 必须为 3")

    planner = _section(document, "planner")
    camera = _section(document, "camera")
    environment = _section(document, "environment")

    backend = str(planner.get("backend", "")).strip().lower()
    robot = str(planner.get("robot_kinematics", "")).strip().lower()
    if backend != "curobo_nvblox":
        raise ValueError("planner.backend 必须为 curobo_nvblox")
    if robot != "aubo_i10":
        raise ValueError("planner.robot_kinematics 必须为 aubo_i10")

    return_to_start = planner.get("return_to_start")
    if not isinstance(return_to_start, bool):
        raise ValueError("planner.return_to_start 必须为 true 或 false")
    voxel = _scalar(planner.get("voxel_size_mm"), "planner.voxel_size_mm")
    dilation = _scalar(
        planner.get("target_dilation_mm"), "planner.target_dilation_mm"
    )
    if voxel <= 0.0:
        raise ValueError("planner.voxel_size_mm 必须大于 0")
    if dilation < 0.0:
        raise ValueError("planner.target_dilation_mm 必须非负")

    intrinsics = _array(camera.get("intrinsics"), (4,), "camera.intrinsics")
    if np.any(intrinsics[:2] <= 0.0):
        raise ValueError("camera.intrinsics 中 fx/fy 必须大于 0")
    depth_min = _scalar(camera.get("depth_min_mm"), "camera.depth_min_mm")
    depth_max = _scalar(camera.get("depth_max_mm"), "camera.depth_max_mm")
    if depth_min < 0.0 or depth_max <= depth_min:
        raise ValueError("camera 深度范围必须满足 0 <= min < max")
    hand_eye = _array(camera.get("T_cam2flange"), (4, 4), "camera.T_cam2flange")
    if not np.allclose(hand_eye[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError("camera.T_cam2flange 最后一行必须为 [0,0,0,1]")
    rotation = hand_eye[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4):
        raise ValueError("camera.T_cam2flange 旋转部分不正交")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-4):
        raise ValueError("camera.T_cam2flange 旋转部分行列式必须为 +1")

    platform_section = _section(environment, "base_platform")
    platform_negative = _array(
        platform_section.get("negative_extent_xyz_mm"),
        (3,),
        "environment.base_platform.negative_extent_xyz_mm",
    )
    platform_positive = _array(
        platform_section.get("positive_extent_xyz_mm"),
        (3,),
        "environment.base_platform.positive_extent_xyz_mm",
    )
    if np.any(platform_negative < 0.0) or np.any(platform_positive < 0.0):
        raise ValueError("environment.base_platform 六个延伸值必须非负")
    if np.any(platform_negative + platform_positive <= 0.0):
        raise ValueError(
            "environment.base_platform 每个轴的正负延伸之和必须大于 0"
        )

    return ProjectRuntimeConfig(
        planner_backend=backend,
        robot_kinematics=robot,
        camera_intrinsics=intrinsics,
        depth_min_mm=depth_min,
        depth_max_mm=depth_max,
        T_cam2flange_mm=hand_eye,
        return_to_start=return_to_start,
        planner_voxel_size_mm=voxel,
        planner_target_dilation_mm=dilation,
        base_platform_negative_extent_xyz_mm=platform_negative,
        base_platform_positive_extent_xyz_mm=platform_positive,
        source_path=source,
        source_sha256=hashlib.sha256(raw).hexdigest(),
    )


def infer_project_config_path(request_path: str | Path) -> Path | None:
    request = Path(request_path).resolve()
    if request.parent.name != "input":
        return None
    candidate = request.parent.parent / "config/config.yaml"
    return candidate if candidate.is_file() else None


def load_effective_request_values(
    request_path: str | Path,
    config_path: str | Path | None = None,
) -> dict[str, str]:
    """合并动态 request.txt 与固定 config.yaml，并拒绝重复定义。"""
    from .text_input import _read_key_values

    request = Path(request_path).resolve()
    dynamic = _read_key_values(request)
    resolved_config = (
        Path(config_path).resolve()
        if config_path is not None
        else infer_project_config_path(request)
    )
    if resolved_config is None:
        return dynamic
    fixed = load_project_runtime_config(resolved_config).request_values()
    duplicates = sorted(FIXED_REQUEST_KEYS.intersection(dynamic))
    if duplicates:
        raise ValueError(
            "request.txt 不得重复定义 config.yaml 固定字段: "
            + ", ".join(duplicates)
        )
    return {**fixed, **dynamic}
