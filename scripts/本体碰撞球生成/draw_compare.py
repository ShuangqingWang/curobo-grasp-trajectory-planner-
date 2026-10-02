"""出本体碰撞球对比图：每个连杆一张，三列 = 原 387 / FOAM 原样 / FOAM 缩半径 191，输出到 output/本体碰撞球对比/。"""
import json, yaml, numpy as np, trimesh, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from pathlib import Path
from matplotlib.patches import Circle
for f in ["/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc", "/usr/share/fonts/truetype/arphic/uming.ttc"]:
    if Path(f).exists():
        font_manager.fontManager.addfont(f); plt.rcParams["font.family"] = font_manager.FontProperties(fname=f).get_name(); break
plt.rcParams["axes.unicode_minus"] = False
from common import BODY_SPHERES, DATA, FOAM_RAW, PKG
OUT = PKG / "output/本体碰撞球对比"; OUT.mkdir(parents=True, exist_ok=True)
MESH = PKG / "scripts/simulation/aubo_description/meshes/aubo_i12h"
LINKS = ["base_link", "shoulder_Link", "upperArm_Link", "foreArm_Link", "wrist1_Link", "wrist2_Link", "wrist3_Link"]
sets = [("原 cuRobo 贴面球 387", json.load(open(DATA / "body_spheres_387_curobo.json"))),
        ("FOAM 原样（缩半径前）", {**json.load(open(FOAM_RAW)), "base_link": json.load(open(BODY_SPHERES))["base_link"]}),
        ("FOAM 缩半径 191（采用）", json.load(open(BODY_SPHERES)))]
def gap(P, C, R):
    return np.concatenate([(np.linalg.norm(P[i:i+2000, None] - C[None], axis=2) - R[None]).min(1) for i in range(0, len(P), 2000)])
for link in LINKS:
    m = trimesh.load(MESH / f"{link}.STL"); m.apply_scale(1000.0)
    surf, _ = trimesh.sample.sample_surface(m, 20000, seed=0)
    ext = m.bounds[1] - m.bounds[0]; ax_n = int(np.argmin(ext)); ctr = m.bounds.mean(0)
    others = [a for a in range(3) if a != ax_n]; names = "XYZ"
    normal = np.zeros(3); normal[ax_n] = 1
    sec = m.section(plane_origin=ctr, plane_normal=normal)
    fig = plt.figure(figsize=(18, 10)); gs = fig.add_gridspec(2, 3, height_ratios=[1.25, 1], hspace=0.12, wspace=0.08)
    for col, (title, S) in enumerate(sets):
        C = np.array([s["center"] for s in S[link]]) * 1000; R = np.array([s["radius"] for s in S[link]]) * 1000
        g = gap(surf, C, R); bad = surf[g > 0.5]
        ax = fig.add_subplot(gs[0, col], projection="3d")
        ax.scatter(*surf[::4].T, s=0.3, c="#888888", alpha=0.35)
        u, v = np.mgrid[0:2*np.pi:14j, 0:np.pi:8j]
        for c, r in zip(C, R):
            ax.plot_surface(c[0]+r*np.cos(u)*np.sin(v), c[1]+r*np.sin(u)*np.sin(v), c[2]+r*np.cos(v), color="#2f6fd6", alpha=0.12, linewidth=0)
        if len(bad): ax.scatter(*bad.T, s=1.2, c="red", alpha=0.8)
        lo = C.min(0) - R.max(); hi = C.max(0) + R.max(); lo = np.minimum(lo, m.bounds[0]); hi = np.maximum(hi, m.bounds[1])
        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
        ax.set_box_aspect(tuple(hi - lo), zoom=1.35); ax.view_init(elev=22, azim=-60)
        ax.set_title(f"{title}\n{len(C)} 球  表面露在球外 {np.mean(g>0)*100:.0f}%  最大缝隙 {max(g.max(),0):.1f}mm", fontsize=11)
        ax.set_axis_off()
        ax2 = fig.add_subplot(gs[1, col])
        if sec is not None:
            for ent in sec.discrete: ax2.plot(ent[:, others[0]], ent[:, others[1]], "k-", lw=1.0)
        d = C[:, ax_n] - ctr[ax_n]; hit = np.abs(d) < R
        for c, r, dd in zip(C[hit], R[hit], d[hit]):
            ax2.add_patch(Circle((c[others[0]], c[others[1]]), np.sqrt(r*r - dd*dd), fc="#2f6fd6", ec="#2f6fd6", alpha=0.18, lw=0.8))
        near = np.abs(surf[:, ax_n] - ctr[ax_n]) < 3.0
        nb = surf[near & (g > 0.5)]
        if len(nb): ax2.scatter(nb[:, others[0]], nb[:, others[1]], s=4, c="red", zorder=5)
        ax2.set_aspect("equal"); ax2.autoscale_view(); ax2.grid(alpha=0.3)
        ax2.set_xlabel(f"{names[others[0]]} (mm)"); ax2.set_ylabel(f"{names[others[1]]} (mm)")
        ax2.set_title(f"剖面 {names[ax_n]}={ctr[ax_n]:.0f}mm：黑线=连杆轮廓 蓝圈=球 红点=缝隙", fontsize=10)
    fig.suptitle(f"{link}：蓝=碰撞球，灰=连杆表面，红=没被任何球盖住的表面（缝隙）；球超出黑线的部分=虚胖", fontsize=13)
    fig.subplots_adjust(left=0.04, right=0.98, top=0.9, bottom=0.06); fig.savefig(OUT / f"{link}.png", dpi=110); plt.close(fig); print(link, "ok", flush=True)
