"""自碰撞检查：用真实轨迹构型（归档里的 joint_trajectory_*.json），计算非忽略连杆对的最小间距，
比较两套本体球（含末端棱线球）。间距 < 0 即会被判自碰撞。

用法：python selfcoll_check.py <归档会话目录> [最多轨迹文件数，默认 600]"""
import glob, json, random, sys, xml.etree.ElementTree as ET
import numpy as np
from common import APP, BODY_SPHERES, CPP_DEVICE, DATA, PKG, URDF

src = (PKG / "services/curobo_planner/_vendor/curobo_adapter.py").read_text(encoding="utf-8")
ns = {"np": np}; exec(src[src.index("def _edge_count"):src.index("def describe_edge_spheres")], ns)
app = json.loads(APP.read_text(encoding="utf-8")); ee = app["end_effector"]
EE = ns["generate_edge_spheres"](ee["negative_extent_xyz_mm"], ee["positive_extent_xyz_mm"], ee["sphere_radius_mm"])

def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr], [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr], [-sp, cp*sr, cp*cr]])

joints = []
for j in ET.parse(URDF).getroot().findall("joint"):
    o = j.find("origin"); T = np.eye(4)
    if o is not None:
        T[:3, :3] = rpy(*[float(v) for v in o.get("rpy", "0 0 0").split()]); T[:3, 3] = [float(v) for v in o.get("xyz", "0 0 0").split()]
    ax = j.find("axis")
    joints.append((j.get("type"), j.find("parent").get("link"), j.find("child").get("link"), T,
                   np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else None))

def fk(q):
    frames = {"base_link": np.eye(4)}; k = 0
    for typ, parent, child, T, ax in joints:
        M = frames[parent] @ T
        if typ in ("revolute", "continuous"):
            a = ax / np.linalg.norm(ax); c, s = np.cos(q[k]), np.sin(q[k])
            K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            R = np.eye(4); R[:3, :3] = np.eye(3) + s * K + (1 - c) * K @ K; M = M @ R; k += 1
        frames[child] = M
    return frames

device = json.loads(CPP_DEVICE.read_text(encoding="utf-8")); ignore = device["self_collision_ignore"]; buffer = device["self_collision_buffer"]
flange = fk(np.deg2rad(app["place"]["q_deg"]))["flange_link"][:3, 3] * 1000
print("正解校验 q_place 法兰(mm) =", flange.round(1), " 配置 =", app["place"]["flange_xyzrxryrz_mm_deg"][:3])
files = glob.glob(f"{sys.argv[1]}/*/output/trajectory_planning/joint_trajectory_*.json")
random.seed(0); files = random.sample(files, min(int(sys.argv[2]) if len(sys.argv) > 2 else 600, len(files)))
Q = np.array([p["positions"] if isinstance(p, dict) else p
              for f in files for p in (json.load(open(f)).get("points") or [])[::3]], dtype=float)
print("构型数", len(Q), "轨迹文件", len(files))

def check(body):
    S = dict(body); S["flange_link"] = EE; links = list(S)
    pairs = [(a, b) for i, a in enumerate(links) for b in links[i + 1:] if b not in ignore.get(a, []) and a not in ignore.get(b, [])]
    worst = {p: 1e9 for p in pairs}
    for q in Q:
        F = fk(q)
        P = {l: (F[l][:3, :3] @ np.array([s["center"] for s in S[l]]).T).T + F[l][:3, 3] for l in links}
        R = {l: np.array([s["radius"] for s in S[l]]) + buffer.get(l, 0.0) for l in links}
        for a, b in pairs:
            worst[(a, b)] = min(worst[(a, b)], (np.linalg.norm(P[a][:, None] - P[b][None], axis=2) - R[a][:, None] - R[b][None]).min())
    return worst

old = check(json.loads((DATA / "body_spheres_387_curobo.json").read_text(encoding="utf-8")))
new = check(json.loads(BODY_SPHERES.read_text(encoding="utf-8")))
print(f"{'连杆对':34s} {'原387最小间距mm':>14s} {'新球最小间距mm':>14s}")
for p in sorted(new, key=lambda p: new[p])[:10]:
    print(f"{p[0] + ' - ' + p[1]:34s} {old[p] * 1000:14.1f} {new[p] * 1000:14.1f}")
bad = [p for p in new if new[p] < 0]
print("新球自碰撞连杆对:", bad if bad else "无")
