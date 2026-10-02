"""第 1 步：调用 FOAM 对各连杆 STL 做中轴球化，输出 data/foam_half_raw.json（米，连杆坐标系）。

需在 FOAM 的 Python 环境中运行（见 README），例如：
    <foam>/../env/bin/python run_foam.py --foam-dir /path/to/foam
"""
import argparse, json, sys, time
from pathlib import Path
from common import FOAM_BRANCH, FOAM_RAW, MESH_DIR

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--foam-dir", required=True, help="FOAM 仓库根目录（已编译）")
parser.add_argument("--output", default=str(FOAM_RAW))
args = parser.parse_args()
sys.path.insert(0, args.foam_dir)
from foam import spherize_mesh  # noqa: E402

result = {}
for link, branch in FOAM_BRANCH.items():
    started = time.time()
    levels = spherize_mesh(link, MESH_DIR / f"{link}.STL",
                           spherization_kwargs=dict(depth=1, branch=branch, method="medial", testerLevels=2,
                                                    numCover=5000, minCover=5, initSpheres=1000, minSpheres=200,
                                                    erFact=2, expand=True, merge=True, burst=False, optimise=True,
                                                    maxOptLevel=1, balExcess=0.05, verify=True, num_samples=500,
                                                    min_samples=1),
                           process_kwargs=dict(manifold_leaves=1000, ratio=0.2))
    spheres = levels[1].spheres
    result[link] = [{"center": [float(v) for v in s.origin], "radius": float(s.radius)} for s in spheres]
    print(f"{link}: 目标 {branch} 球 → 实得 {len(spheres)} 球  耗时 {time.time() - started:.0f}s", flush=True)
Path(args.output).write_text(json.dumps(result, indent=1), encoding="utf-8")
print(f"已写出 {args.output}")
