"""第 2 步：球心、球数不动，只缩半径：r' = min(r, 球心到连杆表面距离 d + 外溢上限)。

由射线出点论证：球面上任一点 p 若在连杆外，从球心 c 沿 cp 方向离开连杆的出点 e 满足
|c−e| ≥ d，故 p 到连杆的距离 ≤ |p−e| = r' − |c−e| ≤ r' − d ≤ 外溢上限（附录 A.7）。
d 由连杆表面 30 万个采样点的 KD 树近似，额外扣 0.5mm 采样余量。base_link 沿用现有球。
"""
import argparse, json
import numpy as np, trimesh, yaml
from scipy.spatial import cKDTree
from common import BODY_SPHERES, BULGE_CAP_MM, FOAM_RAW, GPU_MODEL, KEEP_LINKS, LINKS, MESH_DIR

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--input", default=str(FOAM_RAW))
parser.add_argument("--output", default=str(BODY_SPHERES))
args = parser.parse_args()
foam = json.loads(open(args.input, encoding="utf-8").read())
current = yaml.safe_load(open(GPU_MODEL, encoding="utf-8"))["kinematics"]["collision_spheres"]
out = {link: current[link] for link in KEEP_LINKS}
for link in LINKS:
    if link in KEEP_LINKS:
        continue
    cap = BULGE_CAP_MM[link]
    mesh = trimesh.load(MESH_DIR / f"{link}.STL"); mesh.apply_scale(1000.0)
    dense, face = trimesh.sample.sample_surface(mesh, 300000, seed=1)
    normals = mesh.face_normals[face]; tree = cKDTree(dense)
    C = np.array([s["center"] for s in foam[link]]) * 1000.0
    R = np.array([s["radius"] for s in foam[link]]) * 1000.0
    d, j = tree.query(C)
    inside = np.einsum("ij,ij->i", C - dense[j], normals[j]) < 0
    d_in = np.where(inside, d - 0.5, -(d + 0.5))             # 球心在连杆外时为负
    Rn = np.maximum(np.minimum(R, d_in + cap), 1.0)
    out[link] = [{"center": (c / 1000.0).tolist(), "radius": float(r / 1000.0)} for c, r in zip(C, Rn)]
    print(f"{link}: {len(C)} 球 外溢上限 {cap}mm 半径中位 {np.median(R):.1f}→{np.median(Rn):.1f}mm "
          f"缩小 {int((Rn < R - 1e-9).sum())} 个 球心在连杆外 {int((~inside).sum())} 个")
open(args.output, "w", encoding="utf-8").write(json.dumps(out, indent=1))
print(f"已写出 {args.output}  总球数 {sum(len(v) for v in out.values())}")
