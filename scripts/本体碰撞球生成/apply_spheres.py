"""第 3 步：把本体球同时写入 GPU 模型 yml 与 C++ aubo.json，写后逐球核对两边一致。

yml 只替换 kinematics.collision_spheres 块，其余文本逐字不动；aubo.json 只替换 collision_spheres 键。
加 --check 只核对不写入。
"""
import argparse, json
import numpy as np, yaml
from common import BODY_SPHERES, CPP_DEVICE, GPU_MODEL, LINKS

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--input", default=str(BODY_SPHERES))
parser.add_argument("--check", action="store_true", help="只核对 input 与两份配置是否逐球一致")
args = parser.parse_args()
spheres = json.loads(open(args.input, encoding="utf-8").read())

def same(a, b):
    return all(len(a[k]) == len(b[k]) and np.allclose([s["center"] + [s["radius"]] for s in a[k]],
                                                       [s["center"] + [s["radius"]] for s in b[k]], atol=1e-12)
               for k in LINKS)

if not args.check:
    text = GPU_MODEL.read_text(encoding="utf-8")
    head = "  collision_spheres:\n"; start = text.index(head) + len(head); stop = text.index("  cspace:\n")
    block = "".join(f"    {link}:\n" + "".join(
        f"    - center: [{s['center'][0]!r}, {s['center'][1]!r}, {s['center'][2]!r}]\n      radius: {s['radius']!r}\n"
        for s in spheres[link]) for link in LINKS)
    GPU_MODEL.write_text(text[:start] + block + text[stop:], encoding="utf-8")
    device = json.loads(CPP_DEVICE.read_text(encoding="utf-8"))
    device["collision_spheres"] = {link: [{"center": s["center"], "radius": s["radius"]} for s in spheres[link]]
                                   for link in LINKS}
    CPP_DEVICE.write_text(json.dumps(device, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
gpu = yaml.safe_load(GPU_MODEL.read_text(encoding="utf-8"))["kinematics"]["collision_spheres"]
cpp = json.loads(CPP_DEVICE.read_text(encoding="utf-8"))["collision_spheres"]
print("各连杆球数:", {k: len(gpu[k]) for k in LINKS}, "总计", sum(len(gpu[k]) for k in LINKS))
print("GPU 模型 与 输入一致:", same(gpu, spheres), " C++ 配置 与 输入一致:", same(cpp, spheres),
      " 两份配置一致:", same(gpu, cpp))
