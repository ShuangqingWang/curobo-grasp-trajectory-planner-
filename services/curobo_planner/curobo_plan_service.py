"""cuRobo GPU 轨迹规划常驻服务。

对外只做一件事：**给起点关节角和目标法兰位姿，返回一条 cuRobo 规划并优化过的轨迹**。
抓取候选排序、IK 分支管理、A/B/C 评级、发布策略全部留在 C++ 侧。

协议：本地 TCP，按行分隔的 JSON（一行一个请求，一行一个应答）。

    {"cmd": "ping"}
        -> {"ok": true, "device": "NVIDIA ...", "warmed": true, "applied": {...}}
           applied 为规划器实际使用的构造参数（激活距离、图规划、体素缓存），
           C++ 启动时与 trajectory.yaml 逐项核对（文档 0.4）。

    {"cmd": "set_world",
     "depth_images": [{"path": "/abs/collision_depth_main.f32", "width": 1280, "height": 720,
                       "fx": .., "fy": .., "cx": .., "cy": .., "T_base_camera": [[4x4]]}, ×3],
     "depth_min_mm": 150.0, "depth_max_mm": 2500.0, "voxel_size_mm": 3.0, "collision_activation_mm": 3.0, "enable_graph_planner": false,
     "T_base_flange": [[4x4]], "world_key": "..."}
        -> {"ok": true, "grid_shape": [nx,ny,nz], "occupied": n, "build_ms": 12.3,
            "reused": false, "collision_world": {...}}

    碰撞世界按文档 2.5 建立：GPU 反投影全分辨率深度 → 截断 → 六方向长方体（不外扩）→ 体素化
    （点云 + 工作台）→ PBA 精确 EDT → fp16 ESDF。

    {"cmd": "plan", "q_start_rad": [6 个], "T_goal_mm": [[4x4]],
     "return_trajectories": 4}                       # 抓取段：位姿目标
    {"cmd": "plan_joint", "q_start_rad": [6 个], "q_goal_rad": [6 个],
     "return_trajectories": 4}                       # 放置段：关节目标
        -> {"ok": true,
            "steps": {"ik": {...}, "pad": {...}, "graph": {...},
                      "trajopt": {...}, "interp": {...}},
            "trajectories": [{"rank": 1, "dt": 0.003125, "points": [[6 个 rad], ...]},
                             {"rank": 2, ...}]}
        -> {"ok": false, "error": "...", "failed_step": "ik|graph|trajopt|interp",
            "steps": {...}}

    交出的是**全部收敛**的轨迹（最多 return_trajectories 条），按 cuRobo 自己的
    代价排序。⚠️ 该排序与工程的 A/B/C 评级无关——实测 cuRobo 会把绕行 8.6 倍、
    高出拍照点 1.28 m 的轨迹排在第 2 名。评级必须由调用方另做。

单位约定（与 C++ 侧一致）：关节角 rad，平移 mm。服务内部转换为 cuRobo 用的米。

深度文件格式：float32 小端，行主序 width×height，单位 mm（每路 1280×720 约 3.7MB）。
走文件而不是 TCP，避免大块数据挤在协议里。
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import socketserver
import sys
import threading
import time
import traceback
from pathlib import Path

import numpy as np

SERVICE_DIR = Path(__file__).resolve().parent
if str(SERVICE_DIR) not in sys.path:
    sys.path.insert(0, str(SERVICE_DIR))

MM_TO_M = 0.001

_PLANNER = None
_PLANNER_LOCK = threading.Lock()
_WORLD_KEY = None


def _log(message: str) -> None:
    """子进程日志：stdout 由 C++ 重定向到会话目录 curobo_service.log，每行带时间戳（文档 0.5）。"""
    now = time.time()
    stamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)) + f".{int(now * 1000) % 1000:03d}"
    print(f"{stamp} [curobo_plan_service] {message}", flush=True)


# PBA 站点坐标按每轴 10 bit 打包，单轴体素数硬上限 1024（附录 A.4）。
PBA_MAX_VOXELS_PER_AXIS = 1024
# 播种与求距离的分块大小（格），沿 X 方向切片（文档 2.5.7）。
ESDF_CHUNK_VOXELS = 2_000_000
# 长方体内无障碍 / 全为障碍时的填充距离（m）。
ESDF_FAR_M = 10.0


def _check_grid_limit(shape) -> None:
    """超过 PBA 打包上限时显式报错，不得静默算错。"""
    if any(int(v) > PBA_MAX_VOXELS_PER_AXIS for v in shape):
        raise ValueError(
            f"碰撞网格单轴体素数超过 ParallelBandingEDT 的打包上限 "
            f"{PBA_MAX_VOXELS_PER_AXIS}：shape={[int(v) for v in shape]}。")


def _signed_distance_fp16(occupied, voxel_size_m: float):
    """由占据栅格计算有符号 ESDF，直接输出 fp16（文档 2.5.7）。

    两遍 PBA 精确 EDT：
      第一遍以障碍格为站点 → 每格到最近障碍格的距离（正）；
      第二遍以自由格为站点 → 障碍格到最近自由格的距离（取负）。
    符号约定：**障碍外为正、障碍内为负**。反过来会让整个世界被当成实心。

    显存：只有四块与网格同尺寸的数组 —— 占据标记 1B、站点数组 4B、PBA 缓冲 4B、
    fp16 输出 2B，合计 11 字节/格。站点数组与 PBA 对象两遍共用；播种与求距离沿 X
    分块，本格坐标用一维序列广播，坐标差与平方和在 int32 中计算，最后才转浮点。

    Args:
        occupied: (nx, ny, nz) bool，CUDA 张量。
        voxel_size_m: 体素边长（m）。
    """
    import torch
    from curobo._src.perception.mapper.esdf.edt_parallel_banding import ParallelBandingEDT

    nx, ny, nz = (int(v) for v in occupied.shape)
    _check_grid_limit((nx, ny, nz))
    device = occupied.device
    output = torch.empty((nx, ny, nz), dtype=torch.float16, device=device)
    occupied_count = int(occupied.sum().item())
    if occupied_count == 0:
        output.fill_(ESDF_FAR_M)
        return output
    if occupied_count == nx * ny * nz:
        output.fill_(-ESDF_FAR_M)
        return output

    site = torch.empty((nx, ny, nz), dtype=torch.int32, device=device)
    pba = ParallelBandingEDT((nx, ny, nz), voxel_size_m, device)
    step = max(1, ESDF_CHUNK_VOXELS // (ny * nz))
    ys = torch.arange(ny, device=device, dtype=torch.int32).view(1, ny, 1)
    zs = torch.arange(nz, device=device, dtype=torch.int32).view(1, 1, nz)
    packed_yz = (ys << 10) | (zs << 20)
    no_site = torch.full((), -1, dtype=torch.int32, device=device)

    for inside in (False, True):
        # 播种：站点写入自身打包坐标，否则写 −1。
        for x0 in range(0, nx, step):
            x1 = min(nx, x0 + step)
            xs = torch.arange(x0, x1, device=device, dtype=torch.int32).view(-1, 1, 1)
            chunk = occupied[x0:x1]
            is_site = ~chunk if inside else chunk
            site[x0:x1] = torch.where(is_site, xs | packed_yz, no_site)
        pba.propagate(site)
        # 求距离：解码最近站点 → int32 平方和 → 开方 × 体素边长 → fp16。
        for x0 in range(0, nx, step):
            x1 = min(nx, x0 + step)
            xs = torch.arange(x0, x1, device=device, dtype=torch.int32).view(-1, 1, 1)
            nearest = site[x0:x1]
            dx = (nearest & 0x3FF) - xs
            dy = ((nearest >> 10) & 0x3FF) - ys
            dz = ((nearest >> 20) & 0x3FF) - zs
            distance = torch.sqrt((dx * dx + dy * dy + dz * dz).to(torch.float32)) * voxel_size_m
            if not inside:
                output[x0:x1] = distance.to(torch.float16)
            else:
                output[x0:x1] = torch.where(occupied[x0:x1], -distance.to(torch.float16),
                                            output[x0:x1])
    del site, pba
    return output


def _platform_from_app(app: dict):
    """读 app.json 的 base_platform：平台包围盒（m）+ 安装处圆柱豁免区。

    ⚠️ 豁免区与底座碰撞球是**二选一**（文档 2.5.11），由
    ``_check_platform_sphere_consistency`` 在启动时强制校验。

    Returns:
        (lo, hi, exclusion)；exclusion 为 None 或
        (center_xy, radius_m, z_low, z_high)，孔从平台顶面向下 depth_mm。
    """
    platform = app.get("base_platform") or {}
    if not platform:
        return None, None, None
    negative = np.asarray(platform["negative_extent_xyz_mm"], dtype=np.float64)
    positive = np.asarray(platform["positive_extent_xyz_mm"], dtype=np.float64)
    margin = float(platform.get("collision_margin_mm", 0.0))
    lo = (-negative - margin) * MM_TO_M
    hi = (positive + margin) * MM_TO_M

    node = platform.get("installation_exclusion") or {}
    exclusion = None
    if node.get("enabled"):
        if node.get("geometry_type", "cylinder") != "cylinder":
            raise ValueError(
                f"installation_exclusion.geometry_type 目前只支持 cylinder，收到 "
                f"{node.get('geometry_type')!r}")
        diameter_m = float(node["diameter_mm"]) * MM_TO_M
        depth_m = float(node["depth_mm"]) * MM_TO_M
        if diameter_m <= 0.0 or depth_m <= 0.0:
            raise ValueError("installation_exclusion 的 diameter_mm 与 depth_mm 必须为正")
        top_z = float(positive[2]) * MM_TO_M
        center_xy = np.asarray(node.get("center_xy_mm", [0.0, 0.0]), dtype=np.float64) * MM_TO_M
        exclusion = (center_xy, 0.5 * diameter_m, top_z - depth_m, top_z)
    return lo, hi, exclusion


def _check_platform_sphere_consistency(app: dict, robot_cfg_yml, exclusion) -> float | None:
    """强制校验"实心平台"与"底座球探入平台"不能同时成立（文档 2.5.11）。

    这两者配错时 cuRobo 不会报错，只会让**每一个构型都判撞**。静态配置，只在启动时查一次。

    Returns:
        base_link 碰撞球的最低球面 z（mm），供启动日志；无底座球返回 None。
    """
    import yaml

    with open(robot_cfg_yml, "r", encoding="utf-8") as handle:
        spheres = yaml.safe_load(handle)["kinematics"].get("collision_spheres") or {}
    base = spheres.get("base_link") or []
    lowest_mm = (min(float(s["center"][2]) - float(s["radius"]) for s in base) / MM_TO_M
                 if base else None)
    if exclusion is not None:
        return lowest_mm
    platform = app.get("base_platform") or {}
    if not platform:
        return lowest_mm
    top_z_m = float(platform["positive_extent_xyz_mm"][2]) * MM_TO_M
    intruding = [s for s in base if float(s["center"][2]) - float(s["radius"]) < top_z_m]
    if intruding:
        raise ValueError(
            f"平台按实心处理（installation_exclusion.enabled=false），但 base_link 仍有 "
            f"{len(intruding)} 个碰撞球的球面探入平台（最低 {lowest_mm:.1f}mm）。"
            f"这些球会永久判撞，导致所有构型的 IK 都不收敛。"
            f"二选一：启用 installation_exclusion，或删除这些底座球。")
    return lowest_mm


def _collision_cap_mm(app: dict) -> float:
    """XY 硬上限 CAP = 臂展 robot_reach_mm + 末端长度（文档 2.5.2）。"""
    if "robot_reach_mm" not in app:
        raise ValueError("app.json 缺少 robot_reach_mm，无法确定点云 XY 截断上限 CAP")
    end_effector = app.get("end_effector") or {}
    length_mm = float((end_effector.get("positive_extent_xyz_mm") or [0.0, 0.0, 0.0])[2])
    return float(app["robot_reach_mm"]) + length_mm


def _platform_floor_mm(app: dict) -> float:
    """工作台底面 z（mm），即长方体 −Z 下界（文档 2.5.1）。"""
    platform = app.get("base_platform") or {}
    if not platform:
        raise ValueError("app.json 缺少 base_platform，无法确定工作台底面")
    return -float(platform["negative_extent_xyz_mm"][2])


def _build_scene(request: dict, app: dict, cache_voxels: int):
    """由三路全分辨率深度图 + 工作台构建 cuRobo SceneCfg（文档 2.5.3~2.5.7）。

    ⓪ GPU 反投影全部有效像素 → ① 截断 → ② 确定长方体（六个方向，不外扩）
    → ③ 体素化（点云 + 工作台）→ ④ PBA 精确 EDT → fp16 ESDF。长方体之外视为不碰撞。

    全分辨率像素全部参与，体素量化误差硬上界 v·√3/2 对真实观测表面成立；
    按 stride 抽样的点云只供可视化，不参与碰撞世界。

    Returns:
        (scene_cfg, meta)；meta 写进 trajectory_manifest.json 的 collision_world 节。
    """
    import torch
    from curobo._src.geom.types import SceneCfg, VoxelGrid

    timings = {}
    started = time.perf_counter()
    voxel_mm = float(request["voxel_size_mm"])
    voxel_size_m = voxel_mm * MM_TO_M
    activation_mm = float(request["collision_activation_mm"])
    depth_min = float(request["depth_min_mm"])
    depth_max = float(request["depth_max_mm"])
    images = request.get("depth_images") or []
    if len(images) != 3:
        raise ValueError(f"set_world 需要主/左/右三路深度图，实际 {len(images)} 路")

    # ---- ⓪ 读入三路深度并在 GPU 上反投影全部有效像素（基座系，mm）----
    device = "cuda"
    parts = []
    pixel_total = 0
    grids = {}
    for image in images:
        width, height = int(image["width"]), int(image["height"])
        raw = np.fromfile(image["path"], dtype=np.float32)
        if raw.size != width * height:
            raise ValueError(f"深度文件长度不符：{image['path']} 期望 {width * height}，实际 {raw.size}")
        pixel_total += raw.size
        depth = torch.from_numpy(raw.reshape(height, width)).to(device)
        if (height, width) not in grids:
            grids[(height, width)] = torch.meshgrid(
                torch.arange(height, device=device, dtype=torch.float32),
                torch.arange(width, device=device, dtype=torch.float32), indexing="ij")
        rows, cols = grids[(height, width)]
        valid = torch.isfinite(depth) & (depth >= depth_min) & (depth <= depth_max)
        camera = torch.stack([(cols - float(image["cx"])) * depth / float(image["fx"]),
                              (rows - float(image["cy"])) * depth / float(image["fy"]),
                              depth], dim=-1)[valid]
        pose = torch.tensor(image["T_base_camera"], dtype=torch.float32, device=device)
        parts.append(camera @ pose[:3, :3].T + pose[:3, 3])
    points = torch.cat(parts)
    count = int(points.shape[0])
    torch.cuda.synchronize()
    timings["project_ms"] = 1000.0 * (time.perf_counter() - started)

    cap = _collision_cap_mm(app)
    floor_z = _platform_floor_mm(app)
    photo = np.asarray(request["T_base_flange"], dtype=np.float64).reshape(4, 4)[:3, 3]
    place_pose = (app.get("place") or {}).get("flange_xyzrxryrz_mm_deg")
    if place_pose is None:
        raise ValueError("app.json 缺少 place.flange_xyzrxryrz_mm_deg，无法确定长方体范围")
    place = np.asarray(place_pose[:3], dtype=np.float64)
    z_hi = max(float(photo[2]), float(place[2]))

    # ---- ① 截断 ----
    tick = time.perf_counter()
    over_x = points[:, 0].abs() > cap
    over_y = points[:, 1].abs() > cap
    below = points[:, 2] < floor_z
    above = points[:, 2] > z_hi
    keep = ~(over_x | over_y | below | above)
    kept = points[keep]
    kept_count = int(kept.shape[0])
    torch.cuda.synchronize()
    timings["truncate_ms"] = 1000.0 * (time.perf_counter() - tick)

    # ---- ② 确定长方体：截断后点 ∪ 拍照法兰 ∪ 放置法兰；−Z = max(工作台底面, 点最低)；不外扩 ----
    tick = time.perf_counter()
    if kept_count:
        kept_min = kept.min(dim=0).values.double().cpu().numpy()
        kept_max = kept.max(dim=0).values.double().cpu().numpy()
    lo = np.empty(3)
    hi = np.empty(3)
    lo_source = ["", "", ""]
    hi_source = ["", "", ""]
    for axis in range(2):
        candidates_lo = {"photo_flange": float(photo[axis]), "place_flange": float(place[axis])}
        candidates_hi = dict(candidates_lo)
        if kept_count:
            candidates_lo["cloud"] = float(kept_min[axis])
            candidates_hi["cloud"] = float(kept_max[axis])
        lo_source[axis] = min(candidates_lo, key=candidates_lo.get)
        hi_source[axis] = max(candidates_hi, key=candidates_hi.get)
        lo[axis] = candidates_lo[lo_source[axis]]
        hi[axis] = candidates_hi[hi_source[axis]]
    hi[2] = z_hi
    hi_source[2] = "photo_flange" if float(photo[2]) >= float(place[2]) else "place_flange"
    cloud_z_min = float(kept_min[2]) if kept_count else floor_z
    lo[2] = max(floor_z, cloud_z_min)
    lo_source[2] = "platform_floor" if floor_z >= cloud_z_min else "cloud"
    if not np.all(hi > lo):
        raise ValueError(f"碰撞长方体退化：lo={lo.tolist()} hi={hi.tolist()}")
    timings["box_ms"] = 1000.0 * (time.perf_counter() - tick)

    # ---- ③ 体素化 ----
    tick = time.perf_counter()
    shape = np.maximum(np.ceil((hi - lo) / voxel_mm - 1e-9).astype(np.int64), 2)
    _check_grid_limit(shape)
    voxel_count = int(np.prod(shape))
    if voxel_count > cache_voxels:
        raise ValueError(
            f"碰撞网格 {shape.tolist()} = {voxel_count} 格超过体素缓存容量 {cache_voxels} 格；"
            f"本轮 z_hi={z_hi:.1f}mm，检查 curobo.world_cache_z_hi_max_mm 是否足够。")
    center = 0.5 * (lo + hi)
    # 体素 i 的中心位于 center + (i − (n−1)/2) × v，与 cuRobo 的排布一致（C 序 nx, ny, nz）。
    origin = center - (shape.astype(np.float64) - 1.0) * 0.5 * voxel_mm
    # 占据栅格直接建在 GPU 上：省掉 CPU 大数组清零与整网格上传。
    occupied = torch.zeros(tuple(int(v) for v in shape), dtype=torch.bool, device="cuda")
    if kept_count:
        # index = rint((p − origin) / v)，torch.round 与 np.rint 同为四舍六入五成双。
        index = torch.round((kept.double() - torch.tensor(origin, device=device)) / voxel_mm).long()
        for axis in range(3):
            index[:, axis].clamp_(0, int(shape[axis]) - 1)
        occupied[index[:, 0], index[:, 1], index[:, 2]] = True
        del index
    del points, kept
    occupied_from_cloud = int(occupied.sum().item())

    platform_lo, platform_hi, platform_exclusion = _platform_from_app(app)
    if platform_lo is not None:
        p_lo = platform_lo / MM_TO_M
        p_hi = platform_hi / MM_TO_M
        start = np.clip(np.ceil((p_lo - origin) / voxel_mm - 1e-9).astype(np.int64), 0, shape)
        stop = np.clip(np.floor((p_hi - origin) / voxel_mm + 1e-9).astype(np.int64) + 1, 0, shape)
        if np.all(stop > start):
            occupied[start[0]:stop[0], start[1]:stop[1], start[2]:stop[2]] = True
            if platform_exclusion is not None:
                # 安装处圆柱豁免区：把孔内的格子重新清空（体素形式下没有额外查询成本）。
                center_xy, radius_m, z_low, z_high = platform_exclusion
                axis_x = origin[0] + np.arange(start[0], stop[0]) * voxel_mm
                axis_y = origin[1] + np.arange(start[1], stop[1]) * voxel_mm
                axis_z = origin[2] + np.arange(start[2], stop[2]) * voxel_mm
                cxy = center_xy / MM_TO_M
                in_circle = ((axis_x[:, None] - cxy[0]) ** 2 +
                             (axis_y[None, :] - cxy[1]) ** 2) <= (radius_m / MM_TO_M) ** 2
                in_depth = (axis_z >= z_low / MM_TO_M) & (axis_z <= z_high / MM_TO_M)
                hole = torch.from_numpy(in_circle[:, :, None] & in_depth[None, None, :]).to("cuda")
                occupied[start[0]:stop[0], start[1]:stop[1], start[2]:stop[2]] &= ~hole
    occupied_total = int(occupied.sum().item())
    torch.cuda.synchronize()
    timings["voxelize_ms"] = 1000.0 * (time.perf_counter() - tick)

    # ---- ④ ESDF ----
    tick = time.perf_counter()
    feature_tensor = _signed_distance_fp16(occupied, voxel_size_m).reshape(-1)
    del occupied
    torch.cuda.synchronize()
    timings["esdf_ms"] = 1000.0 * (time.perf_counter() - tick)

    grid = VoxelGrid(
        name="fused_cloud_B",
        pose=[*(center * MM_TO_M).tolist(), 1.0, 0.0, 0.0, 0.0],
        dims=(shape.astype(np.float64) * voxel_size_m).tolist(),
        voxel_size=voxel_size_m,
        feature_tensor=feature_tensor,
        feature_dtype=torch.float16,
    )
    meta = {
        "voxel_size_mm": round(voxel_mm, 3),
        "activation_distance_mm": round(activation_mm, 3),
        "cap_mm": round(cap, 1),
        "z_hi_mm": round(z_hi, 1),
        "platform_floor_mm": round(floor_z, 1),
        "expand_mm": 0.0,
        "source": "depth_full_res",
        "depth_pixels_total": pixel_total,
        "cloud_points": count,
        "cloud_points_kept": kept_count,
        "truncated": {"abs_x_over_cap": int(over_x.sum()), "abs_y_over_cap": int(over_y.sum()),
                      "z_below_floor": int(below.sum()), "z_above_z_hi": int(above.sum())},
        "grid_min_mm": [round(float(v), 1) for v in lo],
        "grid_max_mm": [round(float(v), 1) for v in hi],
        "grid_min_source": lo_source,
        "grid_max_source": hi_source,
        "grid_shape": [int(v) for v in shape],
        # 0 号体素的中心（满精度），可视化据此复现吸附 index = rint((p − origin) / voxel)。
        "voxel_origin_mm": [float(v) for v in origin],
        "voxel_count": voxel_count,
        "occupied_voxel_count": occupied_total,
        "occupied_from_cloud": occupied_from_cloud,
        "occupied_from_platform": occupied_total - occupied_from_cloud,
        "platform_exclusion": (None if platform_exclusion is None else {
            "geometry_type": "cylinder",
            "center_xy_mm": [round(float(v) / MM_TO_M, 1) for v in platform_exclusion[0]],
            "diameter_mm": round(2.0 * platform_exclusion[1] / MM_TO_M, 1),
            "z_range_mm": [round(platform_exclusion[2] / MM_TO_M, 1),
                           round(platform_exclusion[3] / MM_TO_M, 1)],
        }),
        "timings_ms": {k: round(v, 1) for k, v in timings.items()},
    }
    return SceneCfg(voxel=[grid], cuboid=[]), meta


def _max_sphere_radius_m(robot_cfg_yml, end_effector_spheres) -> float:
    """模型中所有碰撞球（本体 + 末端）的最大半径，单位米（日志与可视化用）。"""
    import yaml
    with open(robot_cfg_yml, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    radii = [float(item["radius"])
             for group in (config["kinematics"].get("collision_spheres") or {}).values()
             for item in (group or [])]
    radii.extend(float(item["radius"]) for item in (end_effector_spheres or []))
    if not radii:
        raise ValueError(f"{robot_cfg_yml}: 模型中没有任何碰撞球")
    return max(radii)


def _default_cache_dims_mm(app: dict, z_hi_max_mm: float) -> list:
    """体素缓存按最大可能长方体预分配（文档 0.4）：X、Y 各 2×CAP，Z 从工作台底面到 z_hi 上限。"""
    cap = _collision_cap_mm(app)
    return [2.0 * cap, 2.0 * cap, z_hi_max_mm - _platform_floor_mm(app)]


def _load_app_config(path: Path) -> dict:
    """读 config/base/app.json：末端几何与拍照工位来自同一份配置，不另建尺寸来源。"""
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def _ensure_planner(args, runtime: dict | None = None):
    """构造（或复用）常驻规划器（文档 0.3 S4.1~S4.4）。

    激活距离、图规划开关、体素缓存决定规划器构造，**由 C++ 在启动时作为参数传入**
    （文档 0.4），启动即构造并预热。之后 set_world 下发的值若与已构造实例不一致，
    显式报错，不得静默沿用旧值。
    """
    global _PLANNER
    runtime = runtime or {}
    if _PLANNER is not None:
        drift = []
        activation_mm = float(runtime.get("collision_activation_mm", _PLANNER.applied_activation_mm))
        enable_graph = bool(runtime.get("enable_graph_planner", _PLANNER.applied_enable_graph_planner))
        voxel_mm = float(runtime.get("voxel_size_mm", _PLANNER.applied_cache_voxel_mm))
        if abs(_PLANNER.applied_activation_mm - activation_mm) > 1e-9:
            drift.append(f"collision_activation_mm {_PLANNER.applied_activation_mm} -> {activation_mm}")
        if _PLANNER.applied_enable_graph_planner != enable_graph:
            drift.append(f"enable_graph_planner {_PLANNER.applied_enable_graph_planner} -> {enable_graph}")
        if abs(_PLANNER.applied_cache_voxel_mm - voxel_mm) > 1e-9:
            drift.append(f"world_voxel_size_mm {_PLANNER.applied_cache_voxel_mm} -> {voxel_mm}")
        if drift:
            raise ValueError(
                "碰撞/规划参数与启动参数不一致，无法热切换：" + "；".join(drift) +
                "。这些参数须在启动时传入 GPU 子进程（文档 0.4）。")
        return _PLANNER

    import torch
    from _vendor.curobo_adapter import (CuRoboFreeSegmentPlanner, describe_edge_spheres,
                                        generate_edge_spheres)
    from curobo._src.geom.types import VoxelGrid

    activation_mm = float(args.activation_mm)
    enable_graph = bool(args.enable_graph_planner)
    app = _load_app_config(Path(args.app_config))
    cache_voxel_mm = float(args.cache_voxel_size_mm)
    cache_dims_mm = (list(args.cache_dims_mm) if args.cache_dims_mm
                     else _default_cache_dims_mm(app, args.cache_z_hi_max_mm))
    cache_shape = VoxelGrid("cache", pose=[0, 0, 0, 1, 0, 0, 0],
                            dims=[v * MM_TO_M for v in cache_dims_mm],
                            voxel_size=cache_voxel_mm * MM_TO_M).get_grid_shape()[0]
    cache_shape = [int(v) for v in cache_shape]
    _check_grid_limit(cache_shape)
    _log(f"启动参数 激活距离={activation_mm}mm 体素缓存={cache_voxel_mm}mm×{cache_shape} "
         f"输出点间隔={args.interpolation_dt}s 稠密缓冲={args.interpolation_buffer_size}点 "
         f"图规划={'启用' if enable_graph else '未启用（配置关闭）'} "
         f"种子=IK {args.ik_seeds}/优化 {args.trajopt_seeds}")

    # ---- S4.1 末端棱线球（文档 2.5.8）----
    tick = time.perf_counter()
    end_effector = app.get("end_effector") or {}
    end_effector_frame = end_effector.get("parent_frame", "flange_link")
    spheres = []
    sphere_report = None
    if end_effector and not args.no_end_effector_spheres:
        layout = end_effector.get("sphere_layout", "edges")
        if layout != "edges":
            raise ValueError(f"end_effector.sphere_layout 只支持 edges，收到 {layout!r}")
        radius_mm = float(end_effector["sphere_radius_mm"])
        sphere_report = describe_edge_spheres(end_effector["negative_extent_xyz_mm"],
                                              end_effector["positive_extent_xyz_mm"], radius_mm)
        spheres = generate_edge_spheres(end_effector["negative_extent_xyz_mm"],
                                        end_effector["positive_extent_xyz_mm"], radius_mm)
        _log(f"末端挂载={end_effector_frame} 盒尺寸={sphere_report['box_size_mm']}mm 布球=棱线 "
             f"球数={sphere_report['count']} 半径={sphere_report['radius_mm']}mm "
             f"外凸={sphere_report['bulge_mm']}mm 每棱球数={sphere_report['edge_counts']} "
             f"耗时={1000.0 * (time.perf_counter() - tick):.1f}ms")

    # ---- S4.2 加载模型、构造规划器、预分配体素缓存 ----
    _log(f"加载模型 {args.robot_cfg} …")
    tick = time.perf_counter()
    memory_before = torch.cuda.memory_allocated() if torch.cuda.is_available() else 0
    planner = CuRoboFreeSegmentPlanner(
        args.robot_cfg,
        scene_cfg=None,
        num_ik_seeds=args.ik_seeds,
        num_trajopt_seeds=args.trajopt_seeds,
        use_cuda_graph=not args.no_cuda_graph,
        collision_activation_distance=activation_mm * MM_TO_M,
        cache_voxel_size=cache_voxel_mm * MM_TO_M,
        cache_dims_m=tuple(v * MM_TO_M for v in cache_dims_mm),
        cache_cuboids=args.cache_cuboids,
        interpolation_dt=args.interpolation_dt,
        interpolation_buffer_size=args.interpolation_buffer_size,
        position_tolerance_m=args.position_tolerance_mm * MM_TO_M,
        orientation_tolerance_deg=args.orientation_tolerance_deg,
        end_effector_spheres=spheres,
        end_effector_frame=end_effector_frame,
        enable_graph_planner=enable_graph,
    )
    torch.cuda.synchronize()
    _log(f"加载模型 完成 耗时={1000.0 * (time.perf_counter() - tick):.0f}ms")
    planner.applied_activation_mm = activation_mm
    planner.applied_enable_graph_planner = enable_graph
    planner.applied_cache_voxel_mm = cache_voxel_mm
    planner.applied_interpolation_dt = float(args.interpolation_dt)
    planner.applied_cache_dims_mm = [round(float(v), 1) for v in cache_dims_mm]
    planner.cache_shape = cache_shape
    planner.cache_voxels = int(np.prod(cache_shape))
    planner.max_sphere_radius_m = _max_sphere_radius_m(args.robot_cfg, spheres)
    planner.end_effector_sphere_report = sphere_report
    _log(f"本体球数={planner.body_sphere_count} 末端球数={planner.end_effector_sphere_count} "
         f"总球数={planner.total_sphere_count} "
         f"最大球半径={planner.max_sphere_radius_m / MM_TO_M:.1f}mm "
         f"激活距离={activation_mm}mm "
         f"图规划={'启用' if enable_graph else '未启用（配置关闭）'}")
    _log(f"体素缓存预分配 {cache_voxel_mm}mm×{cache_shape} 格数={planner.cache_voxels} "
         f"显存={planner.cache_voxels * 2 / 1048576:.0f}MB "
         f"（规划器合计 {(torch.cuda.memory_allocated() - memory_before) / 1048576:.0f}MB）")

    # ---- S4.3 工作台/底座球一致性 ----
    _, _, startup_exclusion = _platform_from_app(app)
    lowest_mm = _check_platform_sphere_consistency(app, args.robot_cfg, startup_exclusion)
    _log(f"工作台/底座球一致性=通过 base_link 最低球面 z="
         f"{'无' if lowest_mm is None else f'{lowest_mm:+.1f}mm'}")

    # ---- ESDF 路径预热：首次调用 PBA 内核有约 1s 的加载开销，不能落到第一次触发上 ----
    tick = time.perf_counter()
    probe = torch.zeros((64, 64, 64), dtype=torch.bool, device="cuda")
    probe[32, 32, 32] = True
    _signed_distance_fp16(probe, cache_voxel_mm * MM_TO_M)
    del probe
    torch.cuda.synchronize()
    _log(f"ESDF 路径预热 完成 耗时={1000.0 * (time.perf_counter() - tick):.0f}ms")

    # ---- S4.4 预热：必须覆盖生产实际用到的两条调用路径 ----
    _log("warmup（抓取位姿目标路径，CUDA graph 预热，只做一次）…")
    started = time.perf_counter()
    planner.warmup()
    pose_ms = 1000.0 * (time.perf_counter() - started)
    q_photo = np.deg2rad(np.asarray(app["photo"]["q_deg"], dtype=np.float64))
    joint_ms = planner.warmup_joint_goal(q_photo)
    _log(f"warmup 完成：抓取位姿目标路径={pose_ms:.0f}ms 放置关节目标路径={joint_ms:.0f}ms")
    _PLANNER = planner
    return planner


def _handle(request: dict, args) -> dict:
    global _WORLD_KEY
    command = request.get("cmd")

    if command == "ping":
        import torch
        return {"ok": True,
                # 本进程 pid：C++ 据此确认探活连上的是自己拉起的子进程，而不是端口上的残留进程。
                "pid": os.getpid(),
                "device": torch.cuda.get_device_name(0) if torch.cuda.is_available() else "cpu",
                "warmed": _PLANNER is not None,
                "body_spheres": getattr(_PLANNER, "body_sphere_count", 0),
                "end_effector_spheres": getattr(_PLANNER, "end_effector_sphere_count", 0),
                "total_spheres": getattr(_PLANNER, "total_sphere_count", 0),
                # 带回末端球的完整球心与半径，C++ 据此写固定模型快照，
                # 可视化不再自己拼一套碰撞球。
                "end_effector_sphere_detail": getattr(_PLANNER, "end_effector_sphere_report", None),
                "max_sphere_radius_mm": round(
                    getattr(_PLANNER, "max_sphere_radius_m", 0.0) / MM_TO_M, 3),
                # 规划器实际构造参数，C++ 启动时与 trajectory.yaml 逐项核对（文档 0.4）。
                "applied": (None if _PLANNER is None else {
                    "collision_activation_mm": _PLANNER.applied_activation_mm,
                    "enable_graph_planner": _PLANNER.applied_enable_graph_planner,
                    "cache_voxel_size_mm": _PLANNER.applied_cache_voxel_mm,
                    "cache_dims_mm": _PLANNER.applied_cache_dims_mm,
                    "cache_shape": _PLANNER.cache_shape,
                    "cache_voxels": _PLANNER.cache_voxels,
                    "interpolation_dt": _PLANNER.applied_interpolation_dt}),
                "world_key": _WORLD_KEY}

    if command == "set_world":
        planner = _ensure_planner(args, request)
        key = request.get("world_key")
        if key is not None and key == _WORLD_KEY:
            return {"ok": True, "reused": True, "world_key": key}
        import torch
        started = time.perf_counter()
        app = _load_app_config(Path(args.app_config))
        scene, meta = _build_scene(request, app, planner.cache_voxels)
        tick = time.perf_counter()
        planner.update_world_from_esdf(scene)
        del scene
        torch.cuda.synchronize()
        torch.cuda.empty_cache()
        meta["timings_ms"]["upload_ms"] = round(1000.0 * (time.perf_counter() - tick), 1)
        _WORLD_KEY = key
        response = {"ok": True, "reused": False, "world_key": key,
                    "grid_shape": meta["grid_shape"],
                    "occupied": meta["occupied_voxel_count"],
                    "collision_world": meta,
                    "build_ms": round(1000.0 * (time.perf_counter() - started), 1)}
        t = meta["timings_ms"]
        _log(f"set_world 全分辨率 像素={meta['depth_pixels_total']} 有效点={meta['cloud_points']}"
             f"→截断后 {meta['cloud_points_kept']} "
             f"CAP={meta['cap_mm']}mm z_hi={meta['z_hi_mm']}mm "
             f"长方体 min={meta['grid_min_mm']} max={meta['grid_max_mm']} 外扩=无 "
             f"网格={meta['grid_shape']} 体素={meta['voxel_size_mm']}mm "
             f"占据={meta['occupied_voxel_count']}"
             f"(点云 {meta['occupied_from_cloud']} + 平台 {meta['occupied_from_platform']}) "
             f"耗时 反投影={t['project_ms']} 截断={t['truncate_ms']} 定界={t['box_ms']} 体素化={t['voxelize_ms']} "
             f"ESDF={t['esdf_ms']} 写入={t['upload_ms']} 合计={response['build_ms']}ms "
             f"显存峰值={torch.cuda.max_memory_allocated() / 1048576:.0f}MB")
        return response

    if command == "plan":
        planner = _ensure_planner(args)
        if _WORLD_KEY is None and not args.allow_empty_world:
            return {"ok": False, "error": "尚未设置碰撞世界，拒绝在无障碍检测下规划"}
        q_start = np.asarray(request["q_start_rad"], dtype=np.float64).reshape(6)
        goal_mm = np.asarray(request["T_goal_mm"], dtype=np.float64).reshape(4, 4)
        goal_m = goal_mm.copy()
        goal_m[:3, 3] *= MM_TO_M
        want = int(request.get("return_trajectories", args.return_trajectories))
        started = time.perf_counter()
        result = planner.plan_pose_multi(q_start, goal_m, return_trajectories=want)
        response = {"ok": bool(result["ok"]),
                    "steps": result["steps"],
                    "total_ms": round(1000.0 * (time.perf_counter() - started), 1)}
        if not result["ok"]:
            response["error"] = result["error"]
            response["failed_step"] = result.get("failed_step", "")
            return response
        response["trajectories"] = [
            {"rank": item["rank"],
             "dt": item["dt"],
             "points": np.asarray(item["points"], dtype=np.float64).tolist()}
            for item in result["trajectories"]
        ]
        return response

    if command == "plan_joint":
        # 放置段：关节目标规划。目标是构型本身，不把 q_place 转位姿再求 IK。
        planner = _ensure_planner(args)
        if _WORLD_KEY is None and not args.allow_empty_world:
            return {"ok": False, "error": "尚未设置碰撞世界，拒绝在无障碍检测下规划"}
        q_start = np.asarray(request["q_start_rad"], dtype=np.float64).reshape(6)
        q_goal = np.asarray(request["q_goal_rad"], dtype=np.float64).reshape(6)
        want = int(request.get("return_trajectories", args.return_trajectories))
        started = time.perf_counter()
        result = planner.plan_joint_multi(q_start, q_goal, return_trajectories=want)
        response = {"ok": bool(result["ok"]),
                    "steps": result["steps"],
                    "total_ms": round(1000.0 * (time.perf_counter() - started), 1)}
        if not result["ok"]:
            response["error"] = result["error"]
            response["failed_step"] = result.get("failed_step", "")
            return response
        response["trajectories"] = [
            {"rank": item["rank"],
             "dt": item["dt"],
             "points": np.asarray(item["points"], dtype=np.float64).tolist()}
            for item in result["trajectories"]
        ]
        return response

    if command == "shutdown":
        return {"ok": True, "bye": True}

    return {"ok": False, "error": f"未知命令: {command!r}"}


class _Handler(socketserver.StreamRequestHandler):
    args = None

    def handle(self) -> None:
        for line in self.rfile:
            line = line.strip()
            if not line:
                continue
            try:
                request = json.loads(line)
            except Exception as exc:
                self._reply({"ok": False, "error": f"JSON 解析失败: {exc}"})
                continue
            try:
                # cuRobo/CUDA 不是线程安全的，串行化所有请求。
                with _PLANNER_LOCK:
                    response = _handle(request, self.args)
            except Exception as exc:
                _log("请求处理异常:\n" + traceback.format_exc())
                response = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
            self._reply(response)
            if request.get("cmd") == "shutdown":
                threading.Thread(target=self.server.shutdown, daemon=True).start()
                return

    def _reply(self, document: dict) -> None:
        self.wfile.write((json.dumps(document, ensure_ascii=False) + "\n").encode("utf-8"))
        self.wfile.flush()


class _Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main() -> int:
    # ⚠️ 必须与 C++ 侧 config/base/device/aubo.json 的 robot_type 一致（aubo_i12H）。
    # 装错机型不会报错，只会让轨迹终点系统性偏掉：实测装成 i10 时，
    # 同一组关节角正解出的法兰位置与配置差 41~68 mm，所有候选都会被端点复核挡下。
    default_model = SERVICE_DIR.parent.parent / "model" / "aubo_i12h_curobo.yml"
    parser = argparse.ArgumentParser(description="cuRobo GPU 轨迹规划服务")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=34567)
    parser.add_argument("--robot-cfg", default=str(default_model))
    # 默认值取 cuRobo 官方「单条规划」那一档（motion_planner_cfg.py:171-174）。
    parser.add_argument("--ik-seeds", type=int, default=32)
    parser.add_argument("--trajopt-seeds", type=int, default=4)
    parser.add_argument("--return-trajectories", type=int, default=4,
                        help="每次规划交出的候选轨迹条数上限（<= trajopt-seeds）")
    parser.add_argument("--interpolation-dt", type=float, default=0.003125,
                        help="输出轨迹点间隔（s），由 C++ 按 trajectory.yaml 的 trajectory_point_interval_s 传入")
    parser.add_argument("--interpolation-buffer-size", type=int, default=4000,
                        help="cuRobo 稠密轨迹缓冲点数上限；输出间隔变小时须加大")
    # 收敛判据，须与 C++ 的端点复核容差配套（C++ 留 1.5 倍余量）。
    # 统一端点标准（文档 5.2）：GPU 收敛判据与 C++ 复核容差同为 1.0mm / 0.2°。
    parser.add_argument("--position-tolerance-mm", type=float, default=1.0)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=0.2)
    parser.add_argument("--activation-mm", type=float, default=3.0,
                        help="激活距离（mm），由 C++ 按 trajectory.yaml 传入（文档 0.4）")
    parser.add_argument("--cache-voxel-size-mm", type=float, default=3.0,
                        help="体素缓存的体素尺寸（mm），须等于 world_voxel_size_mm")
    parser.add_argument("--cache-dims-mm", type=float, nargs=3, default=None,
                        help="体素缓存预分配尺寸 X Y Z（mm）；缺省按 2×CAP / 工作台底面到 z_hi 上限")
    parser.add_argument("--cache-z-hi-max-mm", type=float, default=600.0,
                        help="未给 --cache-dims-mm 时使用的 z_hi 上限（mm）")
    parser.add_argument("--cache-cuboids", type=int, default=1,
                        help="长方体显存预分配数量；平台已并入体素，不下发长方体")
    parser.add_argument("--app-config",
                        default=str(SERVICE_DIR.parent.parent / "config" / "base" / "app.json"),
                        help="末端几何与拍照工位来源；与 C++ 侧同一份 app.json")
    parser.add_argument("--no-end-effector-spheres", action="store_true",
                        help="调试用：不挂末端碰撞球（生产禁用，会丢失末端沿途碰撞保护）")
    parser.add_argument("--enable-graph-planner", action="store_true",
                        help="启用 PRM 图规划（默认关闭），由 C++ 按 trajectory.yaml 传入")
    parser.add_argument("--no-cuda-graph", action="store_true")
    parser.add_argument("--allow-empty-world", action="store_true",
                        help="调试用：允许在未设置碰撞世界时规划")
    parser.add_argument("--warmup-on-start", action="store_true")
    args = parser.parse_args()

    if args.warmup_on_start:
        _ensure_planner(args)

    _Handler.args = args
    with _Server((args.host, args.port), _Handler) as server:
        _log(f"监听 {args.host}:{args.port}")
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            _log("收到中断，退出")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
