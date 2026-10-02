"""本体碰撞球生成的公共配置：路径、连杆、FOAM 目标球数、各连杆外溢上限。"""
from pathlib import Path

PKG = Path(__file__).resolve().parents[2]                       # grasp_planner_pkg
TOOL = Path(__file__).resolve().parent
DATA = TOOL / "data"
MESH_DIR = PKG / "scripts/simulation/aubo_description/meshes/aubo_i12h"
URDF = PKG / "scripts/simulation/aubo_description/urdf/aubo_i12h.urdf"
GPU_MODEL = PKG / "model/aubo_i12h_curobo.yml"                  # GPU 规划模型
CPP_DEVICE = PKG / "config/base/device/aubo.json"               # C++ 设备配置
APP = PKG / "config/base/app.json"

LINKS = ["base_link", "shoulder_Link", "upperArm_Link", "foreArm_Link",
         "wrist1_Link", "wrist2_Link", "wrist3_Link"]

# base_link 不重新球化：球面不得探入实心工作台（文档 2.5.11），沿用已删去探入球的 17 个球。
KEEP_LINKS = ["base_link"]

# FOAM 中轴球化的目标球数（branch）。FOAM 实际输出可能少于目标（upperArm 50 → 37）。
FOAM_BRANCH = {"shoulder_Link": 30, "upperArm_Link": 50, "foreArm_Link": 33,
               "wrist1_Link": 28, "wrist2_Link": 28, "wrist3_Link": 18}

# 各连杆外溢上限（mm）：取原 cuRobo 贴面球的外凸 P95，保证新球不比原来更“胖”。
BULGE_CAP_MM = {"shoulder_Link": 11.2, "upperArm_Link": 11.9, "foreArm_Link": 10.8,
                "wrist1_Link": 8.0, "wrist2_Link": 7.9, "wrist3_Link": 7.3}

FOAM_RAW = DATA / "foam_half_raw.json"            # FOAM 原始输出（米）
BODY_SPHERES = DATA / "body_spheres_191.json"     # 缩半径后的最终本体球（米）
