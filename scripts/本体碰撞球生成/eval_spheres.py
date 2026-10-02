"""评测本体碰撞球（单位 mm），可同时比较多套球：
表面覆盖/缝隙：原始 STL 表面 5 万点到球并集的距离（>0 即缝隙）；
体积覆盖：包围盒内随机点（按最近表面点法向判内外）被球覆盖的比例；
外凸：球并集外表面上位于连杆外侧的点，到连杆表面的距离。
注意：部分连杆网格不封闭、内部夹面片，外凸最大值可能含误判，以 P95 为准。

用法：python eval_spheres.py 名称=文件.json [名称=文件.json ...]
缺省比较 data/body_spheres_387_curobo.json 与 data/body_spheres_191.json。"""
import json, sys
import numpy as np, trimesh
from scipy.spatial import cKDTree
from common import BODY_SPHERES, DATA, LINKS, MESH_DIR

args = sys.argv[1:] or [f"原cuRobo贴面387={DATA / 'body_spheres_387_curobo.json'}", f"FOAM缩半径191={BODY_SPHERES}"]
sets = {a.split("=", 1)[0]: json.loads(open(a.split("=", 1)[1], encoding="utf-8").read()) for a in args}
rng = np.random.default_rng(0)

def gap(points, C, R, chunk=2000):
    return np.concatenate([(np.linalg.norm(points[i:i + chunk, None] - C[None], axis=2) - R[None]).min(1)
                           for i in range(0, len(points), chunk)])

refs = {}
for link in LINKS:
    mesh = trimesh.load(MESH_DIR / f"{link}.STL"); mesh.apply_scale(1000.0)
    surf, _ = trimesh.sample.sample_surface(mesh, 50000, seed=0)
    dense, face = trimesh.sample.sample_surface(mesh, 300000, seed=1)
    tree = cKDTree(dense); normals = mesh.face_normals[face]
    def outside_of(P, tree=tree, dense=dense, normals=normals):
        d, j = tree.query(P, workers=4)
        return d, np.einsum("ij,ij->i", P - dense[j], normals[j]) > 0
    lo, hi = mesh.bounds; box = rng.uniform(lo, hi, (80000, 3)); _, out = outside_of(box)
    refs[link] = (surf, box[~out], outside_of)

print(f"{'方案':16s} {'连杆':14s} {'球数':>4s} {'表面覆盖':>7s} {'最大缝隙':>7s} {'体积覆盖':>7s} {'外凸P95':>7s} {'外凸最大':>7s}")
for name, spheres in sets.items():
    total = 0
    for link in LINKS:
        surf, vol, outside_of = refs[link]
        C = np.array([s["center"] for s in spheres[link]]) * 1000.0; R = np.array([s["radius"] for s in spheres[link]]) * 1000.0
        g = gap(surf, C, R); gv = gap(vol, C, R); pts = []
        for c, r in zip(C, R):
            k = max(20, int(4 * np.pi * r * r / 6.0)); u = rng.normal(size=(k, 3)); u /= np.linalg.norm(u, axis=1, keepdims=True)
            pts.append(c + r * u)
        P = np.concatenate(pts); P = P[gap(P, C, R - 1e-6) >= -1e-6]
        if len(P) > 40000: P = P[rng.choice(len(P), 40000, replace=False)]
        d, out = outside_of(P); bulge = d[out] if out.any() else np.zeros(1)
        total += len(C)
        print(f"{name:16s} {link:14s} {len(C):4d} {np.mean(g <= 0):8.3f} {max(g.max(), 0):8.1f} {np.mean(gv <= 0):8.3f} "
              f"{np.percentile(bulge, 95):8.1f} {bulge.max():8.1f}", flush=True)
    print(f"{name}: 总球数 {total}\n")
