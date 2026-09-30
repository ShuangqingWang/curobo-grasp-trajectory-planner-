"""将单帧深度图经 nvblox ESDF 接入 cuRobo VoxelGrid。

本模块从运行时 ``request.txt`` 读取动态任务，从 ``config.yaml`` 读取
固定内参、手眼标定和 ESDF/平台参数；不读取或生成基座系 PLY。
所有 GPU 张量均为米，外部输入距离仍为毫米。
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import numpy as np

from .project_config import load_effective_request_values
from .text_input import MM_TO_M, _matrix4_mm, _pose_from_kv, _vector


@dataclass(frozen=True)
class EsdfSceneInfo:
    """一次深度 ESDF 融合的可记录统计。"""

    voxel_size_m: float
    grid_shape: tuple[int, int, int]
    grid_center_B_m: tuple[float, float, float]
    valid_depth_pixels: int
    masked_target_pixels: int


def build_curobo_scene_from_request(
    path: str | Path,
    config_path: str | Path | None = None,
):
    """合并 request.txt + config.yaml，返回 ``(SceneCfg, EsdfSceneInfo)``。

    深度图在 GPU 上融合到 nvblox。随后在同一 GPU 上分块查询 ESDF，得到 cuRobo
    所需的稠密 ``VoxelGrid``。该转换是当前两个库公开 Python API 之间明确、可检查的
    边界；没有任何 CPU EDT 或中心差分碰撞计算。
    """
    try:
        import torch
        from PIL import Image
        from nvblox_torch.mapper import Mapper, QueryType
        from nvblox_torch.sensor import Sensor
        from curobo._src.geom.types import Cuboid, SceneCfg, VoxelGrid
    except Exception as exc:  # pragma: no cover - 由 GPU 部署环境提供
        raise RuntimeError(f"nvblox/cuRobo 依赖不可用: {exc}") from exc

    if not torch.cuda.is_available():
        raise RuntimeError("nvblox ESDF 必须在 CUDA GPU 上运行")
    path = Path(path).resolve()
    kv = load_effective_request_values(path, config_path)
    base = path.parent
    depth_path = Path(kv["depth_path"])
    if not depth_path.is_absolute():
        depth_path = base / depth_path
    depth_mm = np.asarray(Image.open(depth_path))
    if depth_mm.ndim != 2 or not np.issubdtype(depth_mm.dtype, np.integer):
        raise ValueError("depth_path 必须是单通道整数毫米深度图")
    fx, fy, cx, cy = _vector(kv["camera_intrinsics"], 4, "camera_intrinsics")
    min_depth = float(kv.get("depth_min_mm", "1")) * MM_TO_M
    max_depth = float(kv.get("depth_max_mm", "2500")) * MM_TO_M
    voxel_size = float(kv.get("planner_voxel_size_mm", "5")) * MM_TO_M
    target_dilation = float(kv.get("planner_target_dilation_mm", "0")) * MM_TO_M
    if voxel_size <= 0.0:
        raise ValueError("planner_voxel_size_mm 必须大于 0")

    T_B_F = _pose_from_kv(kv, "", "拍照瞬间法兰位姿",
                          matrix_key="T_B_F_photo", xyzrpy_key="T_B_F_photo_xyzrpy")
    T_F_C = _matrix4_mm(kv["T_cam2flange"])
    T_B_C = T_B_F @ T_F_C
    T_B_O = _pose_from_kv(kv, "", "目标工件位姿", matrix_key="T_B_O_target",
                          xyzrpy_key="T_B_O_target_xyzrpy")
    half_extent = _vector(kv["target_extents_mm"], 3, "target_extents_mm") * MM_TO_M

    depth = torch.as_tensor(depth_mm.astype(np.float32) * MM_TO_M, device="cuda")
    valid = (depth >= min_depth) & (depth <= max_depth)
    target_mask = _target_projection_mask(
        depth.shape[1], depth.shape[0], fx, fy, cx, cy, np.linalg.inv(T_B_C) @ T_B_O,
        half_extent + target_dilation,
    )
    if target_mask is not None:
        mask_gpu = torch.as_tensor(target_mask, device="cuda")
        valid &= ~mask_gpu
    depth = torch.where(valid, depth, torch.zeros_like(depth))
    valid_count = int(valid.sum().item())
    if valid_count == 0:
        raise ValueError("深度图去除目标工件后没有有效环境点")

    # 从实际深度点计算 ESDF 工作区。还必须纳入基座、拍照起点、抓取候选和目标：
    # 只按深度点裁剪会让 q_start 的机器人球落在 VoxelGrid 外，cuRobo 随即把该
    # 状态判成不可行，即使该处没有任何深度障碍。
    u, v = torch.meshgrid(torch.arange(depth.shape[1], device="cuda", dtype=torch.float32),
                          torch.arange(depth.shape[0], device="cuda", dtype=torch.float32),
                          indexing="xy")
    z = depth[valid]
    # 与深度反投影和 mapper.add_depth_frame 使用同一套针孔模型；此前误把
    # fx 当成 cx，会令导出的 ESDF 查询网格在 X 方向整体偏移。
    p_c = torch.stack(((u[valid] - cx) * z / fx, (v[valid] - cy) * z / fy, z), dim=1)
    R = torch.as_tensor(T_B_C[:3, :3], dtype=torch.float32, device="cuda")
    t = torch.as_tensor(T_B_C[:3, 3], dtype=torch.float32, device="cuda")
    p_b = p_c @ R.T + t
    support_points = [np.zeros(3), T_B_F[:3, 3], T_B_O[:3, 3]]
    n_cands = int(kv["grasp_candidate_count"])
    for i in range(n_cands):
        support_points.append(_pose_from_kv(
            kv, f"grasp_candidate_{i}_", f"抓取候选 {i} 法兰位姿"
        )[:3, 3])
    if "T_B_F_place_xyzrpy" in kv or "T_B_F_place" in kv:
        support_points.append(_pose_from_kv(
            kv, "", "放置法兰位姿", matrix_key="T_B_F_place",
            xyzrpy_key="T_B_F_place_xyzrpy",
        )[:3, 3])
    support_B = torch.as_tensor(np.asarray(support_points), dtype=torch.float32, device="cuda")
    lo = torch.minimum(p_b.amin(dim=0), support_B.amin(dim=0)) - 0.050
    hi = torch.maximum(p_b.amax(dim=0), support_B.amax(dim=0)) + 0.050
    center = 0.5 * (lo + hi)
    shape = torch.ceil((hi - lo) / voxel_size).to(torch.int64)
    shape = torch.clamp(shape, min=2)
    dims = shape.to(torch.float32) * voxel_size

    mapper = Mapper(float(voxel_size))
    sensor = Sensor.from_camera(float(fx), float(fy), float(cx), float(cy),
                                int(depth.shape[1]), int(depth.shape[0]))
    # nvblox 的 t_w_c 是 CPU float32 4x4，深度本身必须驻留 GPU。
    mapper.add_depth_frame(depth.contiguous(),
                           torch.as_tensor(T_B_C, dtype=torch.float32), sensor)
    mapper.update_esdf()
    feature = _query_dense_esdf(mapper, QueryType, center, shape, voxel_size)
    grid = VoxelGrid(
        name="depth_esdf_B",
        pose=[*center.detach().cpu().tolist(), 1.0, 0.0, 0.0, 0.0],
        dims=dims.detach().cpu().tolist(),
        voxel_size=voxel_size,
        feature_tensor=feature,
        feature_dtype=torch.float16,
    )

    cuboids = []
    negative_key = "base_platform_negative_extent_xyz_mm"
    positive_key = "base_platform_positive_extent_xyz_mm"
    if negative_key in kv or positive_key in kv:
        if negative_key not in kv or positive_key not in kv:
            raise ValueError("基座平台正负 XYZ 延伸必须同时提供")
        negative = _vector(kv[negative_key], 3, negative_key) * MM_TO_M
        positive = _vector(kv[positive_key], 3, positive_key) * MM_TO_M
        if np.any(negative < 0.0) or np.any(positive < 0.0):
            raise ValueError("基座平台六个延伸值必须非负")
        dims = negative + positive
        if np.any(dims <= 0.0):
            raise ValueError("基座平台每个轴的正负延伸之和必须大于 0")
        platform_center = 0.5 * (positive - negative)
        cuboids.append(Cuboid(
            name="base_platform",
            pose=[*platform_center.tolist(), 1.0, 0.0, 0.0, 0.0],
            dims=dims.tolist(),
        ))
    info = EsdfSceneInfo(
        voxel_size_m=voxel_size,
        grid_shape=tuple(int(x) for x in shape.detach().cpu().tolist()),
        grid_center_B_m=tuple(float(x) for x in center.detach().cpu().tolist()),
        valid_depth_pixels=valid_count,
        masked_target_pixels=int(target_mask.sum()) if target_mask is not None else 0,
    )
    return SceneCfg(voxel=[grid], cuboid=cuboids), info


def _query_dense_esdf(mapper, query_type, center, shape, voxel_size: float):
    """以固定上限分块查询，避免 5 mm 大工作区一次分配过多临时显存。"""
    import torch

    nx, ny, nz = (int(x) for x in shape.detach().cpu().tolist())
    total = nx * ny * nz
    out = torch.empty(total, dtype=torch.float16, device="cuda")
    chunk = 1_000_000
    for start in range(0, total, chunk):
        end = min(start + chunk, total)
        linear = torch.arange(start, end, device="cuda")
        ix = torch.div(linear, ny * nz, rounding_mode="floor")
        iy = torch.div(linear, nz, rounding_mode="floor") % ny
        iz = linear % nz
        xyz = torch.stack((
            center[0] + (ix.to(torch.float32) - (nx - 1) * 0.5) * voxel_size,
            center[1] + (iy.to(torch.float32) - (ny - 1) * 0.5) * voxel_size,
            center[2] + (iz.to(torch.float32) - (nz - 1) * 0.5) * voxel_size,
        ), dim=1)
        # nvblox 0.0.10 的 CUDA 接口实际要求 Nx4 的 sphere 查询；最后一列半径为 0。
        query_spheres = torch.cat((xyz, torch.zeros((len(xyz), 1), device="cuda")), dim=1)
        out[start:end] = mapper.query_layer(query_type.ESDF, query_spheres).squeeze(1).to(torch.float16)
    return out.view(nx, ny, nz)


def _target_projection_mask(width, height, fx, fy, cx, cy, T_C_O, half_extent):
    """投影目标 Box 的 8 个角，得到包含目标的保守图像矩形掩膜。"""
    signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], float)
    corners_o = signs * half_extent
    corners_c = corners_o @ T_C_O[:3, :3].T + T_C_O[:3, 3]
    front = corners_c[:, 2] > 1e-6
    if not np.any(front):
        return None
    uv = np.empty((front.sum(), 2))
    uv[:, 0] = fx * corners_c[front, 0] / corners_c[front, 2] + cx
    uv[:, 1] = fy * corners_c[front, 1] / corners_c[front, 2] + cy
    u0, v0 = np.floor(uv.min(axis=0)).astype(int)
    u1, v1 = np.ceil(uv.max(axis=0)).astype(int)
    u0, v0 = max(0, u0), max(0, v0)
    u1, v1 = min(width - 1, u1), min(height - 1, v1)
    if u0 > u1 or v0 > v1:
        return None
    mask = np.zeros((height, width), dtype=bool)
    mask[v0:v1 + 1, u0:u1 + 1] = True
    return mask
