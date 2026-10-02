"use strict";

const URDF_PATH = "./aubo_description/urdf/aubo_i12h.urdf";
// 与正式规划使用同一份 AUBO 碰撞球数据；球心和半径的单位均为米。
// 改动十：碰撞球一律从规划服务导出的固定模型快照读取，与规划**同源**。
// 原先这里直读 config/base/device/aubo.json 自己再拼一套，违反主文档
// "可视化启动时读取一次，不在浏览器中生成另一套碰撞球"的要求，已删除该路径。
const COLLISION_MODEL_PATH = "../../output/visualization/collision_model.json";
// 快照里 body_spheres 用 link_index 表示所属连杆，顺序与 C++ 侧 LinkIndexByName 一致。
const COLLISION_LINK_NAMES = [
  "base_link", "shoulder_Link", "upperArm_Link", "foreArm_Link",
  "wrist1_Link", "wrist2_Link", "wrist3_Link",
];
const MM_TO_M = 0.001;
const PAGE_PARAMS = new URLSearchParams(window.location.search);
const PROJECT_CONFIG_PATH = PAGE_PARAMS.get("config") || "../../config/grasp_planner.json";
const PROJECT_REQUEST_PATH = "../../input/request.txt";
const GRASP_RESULT_PATH = "../../output/grasp_generation/grasp_result.json";
const GRASP_RESULT_POLL_INTERVAL_MS = 500;
const MAX_GRASP_CANDIDATES = 10;
const GRASP_CANDIDATE_AXIS_OPACITY = 0.35;
const GRASP_POSE_TYPES = [
  { field: "grasp_flange", axisLengthM: 0.065 },
  { field: "grasp_tcp", axisLengthM: 0.080 },
  { field: "trajectory_grasp_flange", axisLengthM: 0.035 },
  { field: "trajectory_grasp_tcp", axisLengthM: 0.050 },
];
const DEFAULT_OUTPUT_ROOT_ABSOLUTE_PATH =
  "/home/vecow/workspace/轨迹规划/grasp_planner_pkg/output";
const ABSOLUTE_FILE_ENDPOINT = "/__visualizer_absolute_file__";
const DEFAULT_POINT_COLOR = [0.40, 0.45, 0.49];
const JOINT_LABELS = {
  shoulder_joint: "J1",
  upperArm_joint: "J2",
  foreArm_joint: "J3",
  wrist1_joint: "J4",
  wrist2_joint: "J5",
  wrist3_joint: "J6",
};
// 坐标轴显示与六轴控制语义一一对应：J1-J5 显示各自子连杆坐标系，
// J6 的输出只显示现场使用的 flange_link。wrist3_Link 仍保留在完整 URDF
// 运动/网格链中，但不再作为第二套末端安装坐标轴显示。
const DISPLAY_FRAME_LINKS = [
  "base_link",
  "shoulder_Link",
  "upperArm_Link",
  "foreArm_Link",
  "wrist1_Link",
  "wrist2_Link",
  "flange_link",
];
const LINK_COLORS = {
  base_link: [0.30, 0.33, 0.33],
  shoulder_Link: [0.74, 0.79, 0.81],
  upperArm_Link: [0.45, 0.67, 0.76],
  foreArm_Link: [0.82, 0.80, 0.73],
  wrist1_Link: [0.78, 0.60, 0.32],
  wrist2_Link: [0.40, 0.66, 0.55],
  wrist3_Link: [0.67, 0.38, 0.34],
};
const PRESETS = {
  zero: [0, 0, 0, 0, 0, 0],
  home: [0, -35, 95, 0, 58, 0],
  reach: [22, -55, 72, 18, 54, 0],
  fold: [-28, -78, 118, 10, -45, 30],
};
const DEFAULT_CART = {
  negativeX: 0.4,
  positiveX: 0.4,
  negativeY: 0.4,
  positiveY: 0.4,
  negativeZ: 1.0,
  positiveZ: 0,
};
const DEFAULT_TOOL = {
  negativeX: 0.07,
  positiveX: 0.07,
  negativeY: 0.04,
  positiveY: 0.04,
  negativeZ: 0,
  positiveZ: 0.16,
};
const DEFAULT_TOOL_COLOR = [0.27, 0.58, 0.48];
const DEFAULT_POINT_CLOUD = [
  [-250, 350, 180],
  [-180, 370, 220],
  [-80, 340, 190],
  [20, 360, 260],
  [120, 330, 210],
  [200, 380, 230],
];
const DEFAULT_MAX_PLY_POINTS = 300000;
const GRASP_DWELL_SECONDS = 1.0;
const POSITION_PATH_DISPLAY_INTERVAL_S = 0.025; // 轨迹文件未带 dt_s 时的缺省显示间隔
const IK_MAX_ITERATIONS = 90;
const IK_POSITION_TOLERANCE_M = 0.001;
const IK_ROTATION_TOLERANCE_RAD = Math.PI / 360;
const IK_FINITE_STEP_RAD = 0.0001;
const IK_DAMPING = 0.08;
const IK_MAX_STEP_RAD = 0.22;

const state = {
  canvas: document.getElementById("viewport"),
  gl: null,
  program: null,
  lineProgram: null,
  pointProgram: null,
  robot: null,
  linkMeshes: new Map(),
  collisionSphereMesh: null,
  blendEnabled: null,            // setBlendMode 的状态缓存（null = 未初始化）
  collisionSpheres: [],
  endEffectorSpheres: [],        // 末端碰撞球（改动十），挂在 flange_link 上
  endEffectorSphereMesh: null,   // 末端球渲染网格（紫色）
  collisionActivationM: 0,       // 近障激活距离（米），用于激活距离层
  endEffectorBulgeMm: 0,         // 末端球外凸量（mm），用于面板标注
  // 改动十 10.3 的显示开关组。沿用主文档"只控制显示"的规则：关闭不卸载模型，
  // 再次打开不重新生成球，且不影响规划中的碰撞检测。
  // 本体碰撞球沿用已有的 showCollisionSpheres（默认关）。
  showEndEffectorSpheres: false, // 末端碰撞球（默认关，紫色）
  showActivationShell: false,    // 激活距离层（默认关）
  jointValues: new Map(),
  jointInputs: new Map(),
  linkWorld: new Map(),
  cartMesh: null,
  cartEdgeBuffer: null,
  toolMesh: null,
  toolSource: "fallback",
  configuredTool: null,
  configuredPlatform: null,
  configuredWorkpiece: null,
  configSourceLabel: PROJECT_CONFIG_PATH,
  workpieceExclusionEdgeBuffer: null,
  workpieceExclusionColor: [220 / 255, 38 / 255, 38 / 255],
  toolColor: [...DEFAULT_TOOL_COLOR],
  pointCloudBuffer: null,
  pointCloudColorBuffer: null,
  pointCloudHasColors: false,
  pointCloudSourceHasColors: false,
  pointCloudRecords: [],
  pointCloudMeta: {},
  trajectoryBuffer: null,
  trajectoryBuffer2: null,
  showMesh: true,
  showCollisionSpheres: false,
  showGrid: true,
  showFrames: true,
  showCart: true,
  showTool: true,
  showPointCloud: true,
  showWorkpieceExclusion: true,
  showTrajectory: true,
  cart: { ...DEFAULT_CART },
  tool: { ...DEFAULT_TOOL },
  pointSize: 4,
  trajectory: {
    frames: [],
    playing: false,
    startMs: 0,
    time: 0,
    speed: 1,
    duration: 0,
    sourceLabel: "",
    generationId: "",
    graspTime: null,
    transferStartTime: null,
  },
  camera: {
    target: [0.42, -0.03, 0.58],
    distance: 3.25,
    yaw: 0.86,
    pitch: 0.34,
  },
  pointer: {
    active: false,
    id: null,
    mode: "orbit",
    x: 0,
    y: 0,
  },
  gridBuffer: null,
  axisBuffer: null,
  frameBuffer: null,
  stagePoseBuffer: null,
  stageResult: null,
  graspCandidatePoseBuffers: {
    grasp_flange: null,
    grasp_tcp: null,
    trajectory_grasp_flange: null,
    trajectory_grasp_tcp: null,
  },
  showGraspCandidatePoses: true,
  showGraspPoseTypes: {
    grasp_flange: true,
    grasp_tcp: true,
    trajectory_grasp_flange: true,
    trajectory_grasp_tcp: true,
  },
  graspCandidatePoseWatcher: {
    active: false,
    timer: null,
    lastDocumentText: null,
    candidates: [],
    loadedAt: null,
    shownCount: 0,          // 实际画出坐标系的候选数（只画选中的，正常为 0 或 1）
    selectedIndex: null,    // 上次建缓冲时用的选中编号，用于发现选中变化后重建
  },
  triCount: 0,
};

const el = {
  loadStatus: document.getElementById("loadStatus"),
  jointControls: document.getElementById("jointControls"),
  applyJointAngles: document.getElementById("applyJointAngles"),
  readJointAngles: document.getElementById("readJointAngles"),
  jointAnglesList: document.getElementById("jointAnglesList"),
  applyJointAnglesList: document.getElementById("applyJointAnglesList"),
  jointStatus: document.getElementById("jointStatus"),
  triangleCount: document.getElementById("triangleCount"),
  tcpPosition: document.getElementById("tcpPosition"),
  flangePose: {
    x: document.getElementById("flangePoseX"),
    y: document.getElementById("flangePoseY"),
    z: document.getElementById("flangePoseZ"),
    rx: document.getElementById("flangePoseRx"),
    ry: document.getElementById("flangePoseRy"),
    rz: document.getElementById("flangePoseRz"),
  },
  applyFlangePose: document.getElementById("applyFlangePose"),
  readFlangePose: document.getElementById("readFlangePose"),
  flangePoseList: document.getElementById("flangePoseList"),
  applyFlangePoseList: document.getElementById("applyFlangePoseList"),
  flangePoseStatus: document.getElementById("flangePoseStatus"),
  toggleMesh: document.getElementById("toggleMesh"),
  toggleCollisionSpheres: document.getElementById("toggleCollisionSpheres"),
  toggleEndEffectorSpheres: document.getElementById("toggleEndEffectorSpheres"),
  toggleActivationShell: document.getElementById("toggleActivationShell"),
  toggleGrid: document.getElementById("toggleGrid"),
  toggleFrames: document.getElementById("toggleFrames"),
  toggleCart: document.getElementById("toggleCart"),
  toggleTool: document.getElementById("toggleTool"),
  togglePointCloud: document.getElementById("togglePointCloud"),
  toggleWorkpieceExclusion: document.getElementById("toggleWorkpieceExclusion"),
  toggleTrajectory: document.getElementById("toggleTrajectory"),
  toggleGraspCandidatePoses: document.getElementById("toggleGraspCandidatePoses"),
  toggleGraspFlangePoses: document.getElementById("toggleGraspFlangePoses"),
  toggleGraspTcpPoses: document.getElementById("toggleGraspTcpPoses"),
  toggleTrajectoryGraspFlangePoses: document.getElementById("toggleTrajectoryGraspFlangePoses"),
  toggleTrajectoryGraspTcpPoses: document.getElementById("toggleTrajectoryGraspTcpPoses"),
  graspCandidatePoseStatus: document.getElementById("graspCandidatePoseStatus"),
  toolSourceMode: document.getElementById("toolSourceMode"),
  loadConfiguredPlatform: document.getElementById("loadConfiguredPlatform"),
  platformStatus: document.getElementById("platformStatus"),
  loadConfiguredTool: document.getElementById("loadConfiguredTool"),
  applyCustomTool: document.getElementById("applyCustomTool"),
  toolStatus: document.getElementById("toolStatus"),
  loadConfiguredWorkpiece: document.getElementById("loadConfiguredWorkpiece"),
  workpieceStatus: document.getElementById("workpieceStatus"),
  pipelineOutputPath: document.getElementById("pipelineOutputPath"),
  loadLatestPipelineOutput: document.getElementById("loadLatestPipelineOutput"),
  loadLatestTrajectories: document.getElementById("loadLatestTrajectories"),
  pipelineStatus: document.getElementById("pipelineStatus"),
  stageWorkpiecePose: document.getElementById("stageWorkpiecePose"),
  stageTcpPose: document.getElementById("stageTcpPose"),
  stageFlangePose: document.getElementById("stageFlangePose"),
  pointSize: document.getElementById("pointSize"),
  pointCloudText: document.getElementById("pointCloudText"),
  pointCloudPath: document.getElementById("pointCloudPath"),
  pointCloudFile: document.getElementById("pointCloudFile"),
  loadPointCloud: document.getElementById("loadPointCloud"),
  loadPointCloudPath: document.getElementById("loadPointCloudPath"),
  clearPointCloud: document.getElementById("clearPointCloud"),
  pointCloudStatus: document.getElementById("pointCloudStatus"),
  photoToGraspTrajectoryFile: document.getElementById("photoToGraspTrajectoryFile"),
  graspToPlaceTrajectoryFile: document.getElementById("graspToPlaceTrajectoryFile"),
  photoToGraspTrajectoryPath: document.getElementById("photoToGraspTrajectoryPath"),
  graspToPlaceTrajectoryPath: document.getElementById("graspToPlaceTrajectoryPath"),
  photoToGraspTrajectorySelection: document.getElementById("photoToGraspTrajectorySelection"),
  graspToPlaceTrajectorySelection: document.getElementById("graspToPlaceTrajectorySelection"),
  loadTrajectory: document.getElementById("loadTrajectory"),
  loadTrajectoryPaths: document.getElementById("loadTrajectoryPaths"),
  playTrajectory: document.getElementById("playTrajectory"),
  pauseTrajectory: document.getElementById("pauseTrajectory"),
  stopTrajectory: document.getElementById("stopTrajectory"),
  trajectorySpeed: document.getElementById("trajectorySpeed"),
  trajectoryStatus: document.getElementById("trajectoryStatus"),
};

window.addEventListener("DOMContentLoaded", async () => {
  try {
    // 必须先完成 WebGL、固定几何与默认点云初始化。否则第三流程先写入的彩色点云
    // 会被 init() 末尾的默认白色点云覆盖，表现为“只能手动绝对路径加载才正确”。
    await init();
    await vizInitialize();
  } catch (error) {
    console.error(error);
    el.loadStatus.textContent = `加载失败: ${error.message}`;
    vizSetStatus(`可视化初始化失败：${error.message}`);
  }
});

window.AuboViz = {
  setCart(config) {
    Object.assign(state.cart, pickFiniteMmAsMeters(config, [
      "negativeX", "positiveX", "negativeY", "positiveY", "negativeZ", "positiveZ",
    ]));
    el.platformStatus.textContent = "仅本次可视化：API 临时平台；未进入规划碰撞检测";
    syncConfigInputs();
  },
  setToolExtents(config) {
    Object.assign(state.tool, pickFiniteMmAsMeters(config, [
      "negativeX", "positiveX", "negativeY", "positiveY", "negativeZ", "positiveZ",
    ]));
    if (!validToolExtents(state.tool)) {
      throw new Error("法兰末端六向延伸必须非负，且每个轴的正负延伸之和必须大于 0");
    }
    state.showTool = true;
    el.toggleTool.checked = true;
    markTemporaryTool("API 临时矩形");
    syncConfigInputs();
  },
  setPointCloud(points, options = {}) {
    if (Number.isFinite(Number(options.pointSize))) {
      state.pointSize = clamp(Number(options.pointSize), 1, 20);
      el.pointSize.value = String(state.pointSize);
    }
    setPointCloudData(points);
  },
  setPointCloudFromPlyBuffer(buffer, options = {}) {
    if (Number.isFinite(Number(options.pointSize))) {
      state.pointSize = clamp(Number(options.pointSize), 1, 20);
      el.pointSize.value = String(state.pointSize);
    }
    const result = parsePlyPointCloud(buffer, {
      maxPoints: Number(options.maxPoints) || DEFAULT_MAX_PLY_POINTS,
    });
    setPointCloudData(result.points, result);
  },
  setTrajectory(data) {
    setTrajectoryData(data);
  },
  async loadTrajectoryFiles(files) {
    return loadTrajectoryFiles(Array.from(files || []));
  },
  loadLatestPipelineOutput,
  playTrajectory,
  pauseTrajectory,
  stopTrajectory,
  setJointAngles(q, options = {}) {
    const unit = options.unit || "deg";
    setJointConfigurationRad(q.slice(0, 6).map((value) => unit === "rad" ? Number(value) : degToRad(Number(value))));
  },
  setFlangePose(pose, options = {}) {
    return solveAndApplyFlangePose(pose, options);
  },
};

async function init() {
  const gl = state.canvas.getContext("webgl", {
    antialias: true,
    alpha: false,
    preserveDrawingBuffer: true,
  });
  if (!gl) {
    throw new Error("当前浏览器不支持 WebGL");
  }

  state.gl = gl;
  state.program = createProgram(gl, MESH_VERTEX_SHADER, MESH_FRAGMENT_SHADER);
  state.lineProgram = createProgram(gl, LINE_VERTEX_SHADER, LINE_FRAGMENT_SHADER);
  state.pointProgram = createProgram(gl, POINT_VERTEX_SHADER, POINT_FRAGMENT_SHADER);
  state.gridBuffer = createLineBuffer(gl, buildGridVertices(1.8, 0.1));
  state.axisBuffer = createLineBuffer(gl, buildWorldAxisVertices());
  state.frameBuffer = createLineBuffer(gl, new Float32Array(0), true);
  state.stagePoseBuffer = createLineBuffer(gl, new Float32Array(0), true);
  for (const type of GRASP_POSE_TYPES) {
    state.graspCandidatePoseBuffers[type.field] = createLineBuffer(
      gl, new Float32Array(0), true
    );
  }
  state.cartMesh = createRenderableMesh(gl, buildUnitBoxMesh(), [0.37, 0.40, 0.39]);
  state.cartEdgeBuffer = createLineBuffer(gl, buildUnitBoxEdgeVertices());
  state.toolMesh = createRenderableMesh(gl, buildUnitBoxMesh(), state.toolColor);
  state.pointCloudBuffer = createLineBuffer(gl, new Float32Array(0), true);
  state.pointCloudColorBuffer = createLineBuffer(gl, new Float32Array(0), true);
  state.workpieceExclusionEdgeBuffer = createLineBuffer(gl, new Float32Array(0), true);
  state.trajectoryBuffer = createLineBuffer(gl, new Float32Array(0), true);
  state.trajectoryBuffer2 = createLineBuffer(gl, new Float32Array(0), true);

  if (PAGE_PARAMS.has("output")) {
    el.pipelineOutputPath.value = requireAbsolutePath(
      PAGE_PARAMS.get("output"), "URL 中的 output 文件夹"
    );
  }
  setupEvents();
  setupConfigControls();
  await loadConfiguredProjectGeometry();
  resizeCanvas();

  const robot = await loadRobot();
  state.robot = robot;
  initJointValues(robot);
  buildJointControls(robot);
  await loadMeshes(robot);
  await loadCollisionSpheres();
  applyPreset("home");
  setPointCloudData(DEFAULT_POINT_CLOUD);
  await loadPointCloudFromUrlParam();
  await refreshGraspCandidatePoses();
  startGraspCandidatePoseWatcher();
  // 第三流程固定使用项目相对目录 output/trajectory_planning 自动监听。
  // 不再响应旧的 ?autoload=latest 绝对路径加载器，避免两个加载器互相覆盖数据源。

  el.loadStatus.textContent = `${robot.links.length} link / ${robot.joints.length} joint`;
  el.triangleCount.textContent = state.triCount.toLocaleString("zh-CN");

  requestAnimationFrame(draw);
}

async function loadRobot() {
  const response = await fetch(URDF_PATH, { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`无法读取 ${URDF_PATH}`);
  }
  const text = await response.text();
  const xml = new DOMParser().parseFromString(text, "application/xml");
  const parseError = xml.querySelector("parsererror");
  if (parseError) {
    throw new Error("URDF XML 解析失败");
  }

  const linkNodes = Array.from(xml.querySelectorAll("robot > link"));
  const jointNodes = Array.from(xml.querySelectorAll("robot > joint"));
  const links = linkNodes.map((node) => {
    const name = node.getAttribute("name");
    const visualMesh = meshPathFromNode(node.querySelector("visual mesh"));
    const collisionMesh = meshPathFromNode(node.querySelector("collision mesh"));
    return {
      name,
      visualMesh,
      collisionMesh,
      mesh: collisionMesh || visualMesh,
    };
  });

  const joints = jointNodes.map((node) => {
    const name = node.getAttribute("name");
    const origin = node.querySelector("origin");
    const axis = node.querySelector("axis");
    const limit = node.querySelector("limit");
    return {
      name,
      type: node.getAttribute("type") || "fixed",
      parent: node.querySelector("parent").getAttribute("link"),
      child: node.querySelector("child").getAttribute("link"),
      xyz: parseNumberTriplet(origin?.getAttribute("xyz"), [0, 0, 0]),
      rpy: parseNumberTriplet(origin?.getAttribute("rpy"), [0, 0, 0]),
      axis: parseNumberTriplet(axis?.getAttribute("xyz"), [0, 0, 1]),
      lower: parseFloat(limit?.getAttribute("lower") ?? "-3.141592653589793"),
      upper: parseFloat(limit?.getAttribute("upper") ?? "3.141592653589793"),
    };
  });

  const childLinks = new Set(joints.map((joint) => joint.child));
  const root = links.find((link) => !childLinks.has(link.name))?.name || links[0]?.name;
  const jointsByParent = new Map();
  const jointByChild = new Map();
  for (const joint of joints) {
    if (!jointsByParent.has(joint.parent)) {
      jointsByParent.set(joint.parent, []);
    }
    jointsByParent.get(joint.parent).push(joint);
    jointByChild.set(joint.child, joint);
  }

  return {
    name: xml.querySelector("robot")?.getAttribute("name") || "aubo_i12h",
    links,
    joints,
    root,
    jointsByParent,
    jointByChild,
  };
}

function meshPathFromNode(meshNode) {
  if (!meshNode) {
    return null;
  }
  const filename = meshNode.getAttribute("filename");
  if (!filename) {
    return null;
  }
  return filename.replace(/^package:\/\/aubo_description\//, "./aubo_description/");
}

function parseNumberTriplet(value, fallback) {
  if (!value) {
    return fallback.slice();
  }
  const parsed = value
    .trim()
    .split(/\s+/)
    .map((part) => Number(part));
  return parsed.length === 3 && parsed.every(Number.isFinite) ? parsed : fallback.slice();
}

function initJointValues(robot) {
  for (const joint of robot.joints) {
    state.jointValues.set(joint.name, 0);
  }
}

function buildJointControls(robot) {
  el.jointControls.replaceChildren();
  state.jointInputs.clear();

  for (const joint of robot.joints) {
    if (joint.type !== "revolute" && joint.type !== "continuous") {
      continue;
    }

    const row = document.createElement("div");
    row.className = "joint-control";

    const name = document.createElement("span");
    name.className = "joint-name";
    name.textContent = JOINT_LABELS[joint.name] || joint.name;
    name.title = joint.name;

    const input = document.createElement("input");
    input.type = "range";
    input.min = String(radToDeg(joint.lower));
    input.max = String(radToDeg(joint.upper));
    input.step = "0.5";
    input.value = "0";
    input.setAttribute("aria-label", joint.name);

    const value = document.createElement("input");
    value.type = "number";
    value.className = "joint-value";
    value.min = String(radToDeg(joint.lower));
    value.max = String(radToDeg(joint.upper));
    value.step = "0.1";
    value.value = "0.0";
    value.setAttribute("aria-label", `${joint.name} deg`);

    input.addEventListener("input", () => {
      const degrees = Number(input.value);
      pauseTrajectoryForManualCommand();
      state.jointValues.set(joint.name, degToRad(degrees));
      value.value = degrees.toFixed(1);
      updateKinematics();
      el.jointStatus.textContent = "关节角已更新";
    });

    value.addEventListener("keydown", (event) => {
      if (event.key === "Enter") {
        event.preventDefault();
        applyJointAnglesFromInputs();
      }
    });

    row.append(name, input, value);
    el.jointControls.append(row);
    state.jointInputs.set(joint.name, { input, value });
  }

  for (const button of document.querySelectorAll(".preset")) {
    button.addEventListener("click", () => applyPreset(button.dataset.preset));
  }
}

function applyPreset(name) {
  const preset = PRESETS[name];
  if (!preset || !state.robot) {
    return;
  }
  setJointConfigurationRad(preset.map(degToRad));
}

async function loadMeshes(robot) {
  state.triCount = 0;
  for (const link of robot.links) {
    if (!link.mesh) {
      continue;
    }
    const response = await fetch(link.mesh);
    if (!response.ok) {
      throw new Error(`mesh 不存在: ${link.mesh}`);
    }
    const buffer = await response.arrayBuffer();
    const mesh = parseStl(buffer);
    mesh.color = LINK_COLORS[link.name] || [0.72, 0.74, 0.70];
    mesh.buffers = createMeshBuffers(state.gl, mesh);
    state.linkMeshes.set(link.name, mesh);
    state.triCount += mesh.triangleCount;
    el.loadStatus.textContent = `加载 ${link.name}`;
  }
}

/**
 * 加载规划器实际使用的碰撞球。它们覆盖在 STL 网格上，仅用于核对碰撞近似，
 * 不参与浏览器端的碰撞判定。
 *
 * 改动十：本体球与**末端球**都从 collision_model.json（规划服务导出的固定模型
 * 快照）读取，保证与规划同源。快照单位是 mm，这里转成渲染用的米。
 * 末端球挂在 end_effector_sphere_frame（flange_link）上，随法兰运动。
 */
async function loadCollisionSpheres() {
  const response = await fetch(COLLISION_MODEL_PATH, { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`固定模型快照不存在: ${COLLISION_MODEL_PATH}`);
  }
  const model = await response.json();

  const bodySpheres = [];
  for (const entry of Array.isArray(model?.body_spheres) ? model.body_spheres : []) {
    const linkName = COLLISION_LINK_NAMES[Number(entry?.link_index)];
    const center = entry?.center_mm;
    const radius = Number(entry?.radius_mm);
    if (!linkName || !Array.isArray(center) || center.length !== 3
        || !center.every(Number.isFinite) || !Number.isFinite(radius) || radius <= 0) {
      continue;
    }
    bodySpheres.push({
      linkName,
      center: center.map((value) => value * MM_TO_M),
      radius: radius * MM_TO_M,
    });
  }
  if (bodySpheres.length === 0) {
    throw new Error("collision_model.json 没有有效的本体碰撞球");
  }

  // 末端球：快照 v2 起带完整球心与半径；v1 只有数量，此时画不出来，降级为空。
  const endFrame = String(model?.end_effector_sphere_frame || "flange_link");
  const endSpheres = [];
  for (const entry of Array.isArray(model?.end_effector_spheres) ? model.end_effector_spheres : []) {
    const center = entry?.center_mm;
    const radius = Number(entry?.radius_mm);
    if (!Array.isArray(center) || center.length !== 3 || !center.every(Number.isFinite)
        || !Number.isFinite(radius) || radius <= 0) {
      continue;
    }
    endSpheres.push({
      linkName: endFrame,
      center: center.map((value) => value * MM_TO_M),
      radius: radius * MM_TO_M,
    });
  }

  state.collisionSpheres = bodySpheres;
  state.endEffectorSpheres = endSpheres;
  state.collisionActivationM = (Number(model?.collision_activation_mm) || 0) * MM_TO_M;
  state.endEffectorBulgeMm = Number(model?.end_effector_sphere_bulge_mm) || 0;
  state.collisionSphereMesh = createRenderableMesh(
    state.gl,
    buildUnitSphereMesh(16, 12),
    [0.98, 0.34, 0.08],
  );
  // 末端球用紫色，与抓取夹爪预览的橙色区分（改动十 10.3）。
  state.endEffectorSphereMesh = createRenderableMesh(
    state.gl,
    buildUnitSphereMesh(16, 12),
    [0.64, 0.35, 0.92],
  );
}

function parseStl(buffer) {
  const view = new DataView(buffer);
  const triangleCount = view.getUint32(80, true);
  const expectedLength = 84 + triangleCount * 50;
  if (expectedLength === buffer.byteLength) {
    return parseBinaryStl(view, triangleCount);
  }
  return parseAsciiStl(new TextDecoder().decode(buffer));
}

function parseBinaryStl(view, triangleCount) {
  const positions = new Float32Array(triangleCount * 9);
  const normals = new Float32Array(triangleCount * 9);
  const bounds = {
    min: [Infinity, Infinity, Infinity],
    max: [-Infinity, -Infinity, -Infinity],
  };
  let offset = 84;
  let write = 0;

  for (let tri = 0; tri < triangleCount; tri += 1) {
    const fileNormal = [
      view.getFloat32(offset, true),
      view.getFloat32(offset + 4, true),
      view.getFloat32(offset + 8, true),
    ];
    offset += 12;

    const verts = [];
    for (let vertex = 0; vertex < 3; vertex += 1) {
      const point = [
        view.getFloat32(offset, true),
        view.getFloat32(offset + 4, true),
        view.getFloat32(offset + 8, true),
      ];
      offset += 12;
      verts.push(point);
      expandBounds(bounds, point);
    }
    offset += 2;

    const normal =
      length3(fileNormal) > 0.000001 ? normalize3(fileNormal) : faceNormal(verts[0], verts[1], verts[2]);
    for (const point of verts) {
      positions[write] = point[0];
      normals[write] = normal[0];
      positions[write + 1] = point[1];
      normals[write + 1] = normal[1];
      positions[write + 2] = point[2];
      normals[write + 2] = normal[2];
      write += 3;
    }
  }

  return {
    positions,
    normals,
    triangleCount,
    vertexCount: triangleCount * 3,
    bounds,
  };
}

function parseAsciiStl(text) {
  const vertexMatches = Array.from(text.matchAll(/vertex\s+([^\s]+)\s+([^\s]+)\s+([^\s]+)/gi));
  const triangleCount = Math.floor(vertexMatches.length / 3);
  const positions = new Float32Array(triangleCount * 9);
  const normals = new Float32Array(triangleCount * 9);
  const bounds = {
    min: [Infinity, Infinity, Infinity],
    max: [-Infinity, -Infinity, -Infinity],
  };

  for (let tri = 0; tri < triangleCount; tri += 1) {
    const verts = [0, 1, 2].map((i) => {
      const match = vertexMatches[tri * 3 + i];
      return [Number(match[1]), Number(match[2]), Number(match[3])];
    });
    const normal = faceNormal(verts[0], verts[1], verts[2]);
    for (let i = 0; i < 3; i += 1) {
      const point = verts[i];
      const write = tri * 9 + i * 3;
      positions[write] = point[0];
      positions[write + 1] = point[1];
      positions[write + 2] = point[2];
      normals[write] = normal[0];
      normals[write + 1] = normal[1];
      normals[write + 2] = normal[2];
      expandBounds(bounds, point);
    }
  }

  return {
    positions,
    normals,
    triangleCount,
    vertexCount: triangleCount * 3,
    bounds,
  };
}

function createMeshBuffers(gl, mesh) {
  const position = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, position);
  gl.bufferData(gl.ARRAY_BUFFER, mesh.positions, gl.STATIC_DRAW);

  const normal = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, normal);
  gl.bufferData(gl.ARRAY_BUFFER, mesh.normals, gl.STATIC_DRAW);

  return { position, normal };
}

function createRenderableMesh(gl, geometry, color) {
  const mesh = {
    ...geometry,
    color,
  };
  mesh.buffers = createMeshBuffers(gl, mesh);
  return mesh;
}

function scaleGeometry(geometry, scale) {
  const positions = new Float32Array(geometry.positions.length);
  for (let i = 0; i < geometry.positions.length; i += 1) {
    positions[i] = geometry.positions[i] * scale;
  }
  return {
    ...geometry,
    positions,
  };
}

function buildUnitBoxMesh() {
  const faces = [
    [[1, 0, 0], [[0.5, -0.5, -0.5], [0.5, 0.5, -0.5], [0.5, 0.5, 0.5], [0.5, -0.5, 0.5]]],
    [[-1, 0, 0], [[-0.5, 0.5, -0.5], [-0.5, -0.5, -0.5], [-0.5, -0.5, 0.5], [-0.5, 0.5, 0.5]]],
    [[0, 1, 0], [[0.5, 0.5, -0.5], [-0.5, 0.5, -0.5], [-0.5, 0.5, 0.5], [0.5, 0.5, 0.5]]],
    [[0, -1, 0], [[-0.5, -0.5, -0.5], [0.5, -0.5, -0.5], [0.5, -0.5, 0.5], [-0.5, -0.5, 0.5]]],
    [[0, 0, 1], [[-0.5, -0.5, 0.5], [0.5, -0.5, 0.5], [0.5, 0.5, 0.5], [-0.5, 0.5, 0.5]]],
    [[0, 0, -1], [[-0.5, 0.5, -0.5], [0.5, 0.5, -0.5], [0.5, -0.5, -0.5], [-0.5, -0.5, -0.5]]],
  ];
  const positions = [];
  const normals = [];
  for (const [normal, quad] of faces) {
    const tris = [quad[0], quad[1], quad[2], quad[0], quad[2], quad[3]];
    for (const point of tris) {
      positions.push(...point);
      normals.push(...normal);
    }
  }
  return {
    positions: new Float32Array(positions),
    normals: new Float32Array(normals),
    triangleCount: positions.length / 9,
    vertexCount: positions.length / 3,
  };
}

function buildUnitSphereMesh(longitudeSegments, latitudeSegments) {
  const positions = [];
  const normals = [];
  const pointAt = (longitude, latitude) => {
    const theta = (longitude / longitudeSegments) * Math.PI * 2;
    const phi = (latitude / latitudeSegments) * Math.PI;
    const y = Math.cos(phi);
    const horizontal = Math.sin(phi);
    return [horizontal * Math.cos(theta), horizontal * Math.sin(theta), y];
  };
  for (let latitude = 0; latitude < latitudeSegments; latitude += 1) {
    for (let longitude = 0; longitude < longitudeSegments; longitude += 1) {
      const nextLongitude = (longitude + 1) % longitudeSegments;
      const a = pointAt(longitude, latitude);
      const b = pointAt(nextLongitude, latitude);
      const c = pointAt(nextLongitude, latitude + 1);
      const d = pointAt(longitude, latitude + 1);
      for (const point of [a, b, c, a, c, d]) {
        positions.push(...point);
        normals.push(...point);
      }
    }
  }
  return {
    positions: new Float32Array(positions),
    normals: new Float32Array(normals),
    triangleCount: positions.length / 9,
    vertexCount: positions.length / 3,
  };
}

function buildUnitBoxEdgeVertices() {
  const p = [
    [-0.5, -0.5, -0.5],
    [0.5, -0.5, -0.5],
    [0.5, 0.5, -0.5],
    [-0.5, 0.5, -0.5],
    [-0.5, -0.5, 0.5],
    [0.5, -0.5, 0.5],
    [0.5, 0.5, 0.5],
    [-0.5, 0.5, 0.5],
  ];
  const edges = [
    [0, 1], [1, 2], [2, 3], [3, 0],
    [4, 5], [5, 6], [6, 7], [7, 4],
    [0, 4], [1, 5], [2, 6], [3, 7],
  ];
  return new Float32Array(edges.flatMap(([a, b]) => [...p[a], ...p[b]]));
}

function createLineBuffer(gl, data, dynamic = false) {
  const buffer = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
  gl.bufferData(gl.ARRAY_BUFFER, data, dynamic ? gl.DYNAMIC_DRAW : gl.STATIC_DRAW);
  return {
    buffer,
    count: data.length / 3,
    dynamic,
  };
}

function updateLineBuffer(gl, lineBuffer, data) {
  gl.bindBuffer(gl.ARRAY_BUFFER, lineBuffer.buffer);
  gl.bufferData(gl.ARRAY_BUFFER, data, lineBuffer.dynamic ? gl.DYNAMIC_DRAW : gl.STATIC_DRAW);
  lineBuffer.count = data.length / 3;
}

function updateKinematics() {
  if (!state.robot) {
    return;
  }
  state.linkWorld = computeLinkWorld(state.jointValues);

  updateFrameLines();
  const tcp = getFlangeWorld();
  const p = transformPoint(tcp, [0, 0, 0]);
  const rpy = mat4ToRpy(tcp).map(radToDeg);
  el.tcpPosition.textContent = p.map((v) => mToMm(v).toFixed(1)).join(", ");
  updateFlangePoseDisplay(p, rpy);
}

function updateFlangePoseDisplay(positionM, rpyDeg, force = false) {
  setInputDisplayValue(el.flangePose.x, mToMm(positionM[0]).toFixed(1), force);
  setInputDisplayValue(el.flangePose.y, mToMm(positionM[1]).toFixed(1), force);
  setInputDisplayValue(el.flangePose.z, mToMm(positionM[2]).toFixed(1), force);
  setInputDisplayValue(el.flangePose.rx, rpyDeg[0].toFixed(2), force);
  setInputDisplayValue(el.flangePose.ry, rpyDeg[1].toFixed(2), force);
  setInputDisplayValue(el.flangePose.rz, rpyDeg[2].toFixed(2), force);
}

function setInputDisplayValue(input, value, force = false) {
  if (force || document.activeElement !== input) {
    input.value = value;
  }
}

function syncFlangePoseInputsFromCurrent(force = false) {
  const flange = getFlangeWorld();
  const position = transformPoint(flange, [0, 0, 0]);
  const rpy = mat4ToRpy(flange).map(radToDeg);
  updateFlangePoseDisplay(position, rpy, force);
}

function computeLinkWorld(jointValues) {
  const linkWorld = new Map();
  linkWorld.set(state.robot.root, mat4Identity());

  const visit = (linkName) => {
    const parentWorld = linkWorld.get(linkName);
    const childJoints = state.robot.jointsByParent.get(linkName) || [];
    for (const joint of childJoints) {
      const origin = mat4Multiply(
        mat4Translation(joint.xyz[0], joint.xyz[1], joint.xyz[2]),
        mat4FromRpy(joint.rpy[0], joint.rpy[1], joint.rpy[2]),
      );
      const q = jointValues.get(joint.name) || 0;
      const motion = joint.type === "revolute" || joint.type === "continuous"
        ? mat4FromAxisAngle(joint.axis, q)
        : mat4Identity();
      const childWorld = mat4Multiply(parentWorld, mat4Multiply(origin, motion));
      linkWorld.set(joint.child, childWorld);
      visit(joint.child);
    }
  };

  visit(state.robot.root);
  return linkWorld;
}

function jointMapFromQ(qRad) {
  const values = new Map();
  const joints = motionJoints();
  for (let index = 0; index < joints.length; index += 1) {
    const joint = joints[index];
    values.set(joint.name, qRad[index] || 0);
  }
  return values;
}

function setJointConfigurationRad(qRad) {
  if (!state.robot) {
    return;
  }
  const joints = motionJoints();
  for (let index = 0; index < joints.length; index += 1) {
    const joint = joints[index];
    const value = clamp(qRad[index] || 0, joint.lower, joint.upper);
    state.jointValues.set(joint.name, value);
    const controls = state.jointInputs.get(joint.name);
    if (controls) {
      const degrees = radToDeg(value);
      controls.input.value = String(degrees);
      setInputDisplayValue(controls.value, degrees.toFixed(1));
    }
  }
  updateKinematics();
}

function motionJoints() {
  return (state.robot?.joints || []).filter((joint) => joint.type === "revolute" || joint.type === "continuous");
}

function currentJointAnglesRad() {
  return motionJoints().map((joint) => state.jointValues.get(joint.name) || 0);
}

function syncJointAngleInputs(force = false) {
  for (const joint of motionJoints()) {
    const controls = state.jointInputs.get(joint.name);
    if (!controls) {
      continue;
    }
    const degrees = radToDeg(state.jointValues.get(joint.name) || 0);
    controls.input.value = String(degrees);
    setInputDisplayValue(controls.value, degrees.toFixed(1), force);
  }
}

function applyJointAnglesFromInputs() {
  const q = [];
  for (const joint of motionJoints()) {
    const controls = state.jointInputs.get(joint.name);
    const degrees = Number(controls?.value.value);
    if (!Number.isFinite(degrees)) {
      el.jointStatus.textContent = `${JOINT_LABELS[joint.name] || joint.name} 输入无效`;
      return;
    }
    q.push(degToRad(degrees));
  }
  pauseTrajectoryForManualCommand();
  setJointConfigurationRad(q);
  syncJointAngleInputs(true);
  el.jointStatus.textContent = "关节角执行完成";
}

function parseSixValueList(text, label) {
  const normalized = text.trim().replace(/[\[\](){}]/g, "").replace(/，/g, ",");
  const values = normalized.split(/[\s,]+/).filter(Boolean).map(Number);
  if (values.length !== 6 || !values.every(Number.isFinite)) {
    throw new Error(`${label}需要 6 个有效数值`);
  }
  return values;
}

function applyJointAnglesFromList() {
  try {
    const qDeg = parseSixValueList(el.jointAnglesList.value, "关节角列表");
    pauseTrajectoryForManualCommand();
    setJointConfigurationRad(qDeg.map(degToRad));
    syncJointAngleInputs(true);
    el.jointStatus.textContent = "列表关节角执行完成";
  } catch (error) {
    el.jointStatus.textContent = `输入无效: ${error.message}`;
  }
}

function pauseTrajectoryForManualCommand() {
  if (state.trajectory.playing) {
    state.trajectory.playing = false;
    updateTrajectoryStatus();
  }
}

function getFlangeWorld(linkWorld = state.linkWorld) {
  return linkWorld.get("flange_link") || mat4Identity();
}

function toolWorldMatrix() {
  const sizeX = state.tool.negativeX + state.tool.positiveX;
  const sizeY = state.tool.negativeY + state.tool.positiveY;
  const sizeZ = state.tool.negativeZ + state.tool.positiveZ;
  const centerX = 0.5 * (state.tool.positiveX - state.tool.negativeX);
  const centerY = 0.5 * (state.tool.positiveY - state.tool.negativeY);
  const centerZ = 0.5 * (state.tool.positiveZ - state.tool.negativeZ);
  return mat4Multiply(
    getFlangeWorld(),
    mat4Multiply(
      mat4Translation(centerX, centerY, centerZ),
      mat4Scale(sizeX, sizeY, sizeZ),
    ),
  );
}

function applyFlangePoseFromInputs() {
  const pose = {
    x: Number(el.flangePose.x.value),
    y: Number(el.flangePose.y.value),
    z: Number(el.flangePose.z.value),
    rx: Number(el.flangePose.rx.value),
    ry: Number(el.flangePose.ry.value),
    rz: Number(el.flangePose.rz.value),
  };
  solveAndApplyFlangePose(pose);
}

function applyFlangePoseFromList() {
  try {
    const [x, y, z, rx, ry, rz] = parseSixValueList(el.flangePoseList.value, "法兰姿态列表");
    solveAndApplyFlangePose({ x, y, z, rx, ry, rz });
  } catch (error) {
    el.flangePoseStatus.textContent = `输入无效: ${error.message}`;
  }
}

function solveAndApplyFlangePose(pose, options = {}) {
  let target;
  try {
    target = parseFlangePoseTarget(pose);
  } catch (error) {
    el.flangePoseStatus.textContent = `输入无效: ${error.message}`;
    return null;
  }

  pauseTrajectoryForManualCommand();
  const result = solveFlangeIk(target, options);
  if (!result) {
    el.flangePoseStatus.textContent = "IK失败";
    return null;
  }

  setJointConfigurationRad(result.q);
  syncJointAngleInputs(true);
  syncFlangePoseInputsFromCurrent(true);
  el.flangePoseStatus.textContent = formatIkStatus(result);
  return {
    converged: result.converged,
    iterations: result.iterations,
    positionErrorMm: result.positionError * 1000,
    rotationErrorDeg: radToDeg(result.rotationError),
    qDeg: result.q.map(radToDeg),
  };
}

function parseFlangePoseTarget(pose) {
  if (Array.isArray(pose)) {
    const [x, y, z, rx, ry, rz] = pose;
    pose = { x, y, z, rx, ry, rz };
  }
  const rx = pose.rx ?? pose.roll;
  const ry = pose.ry ?? pose.pitch;
  const rz = pose.rz ?? pose.yaw;
  const values = [pose.x, pose.y, pose.z, rx, ry, rz].map(Number);
  if (!values.every(Number.isFinite)) {
    throw new Error("需要 X/Y/Z/Rx/Ry/Rz");
  }
  return {
    position: [mmToM(values[0]), mmToM(values[1]), mmToM(values[2])],
    rpy: [degToRad(values[3]), degToRad(values[4]), degToRad(values[5])],
  };
}

function solveFlangeIk(target, options = {}) {
  const seeds = buildIkSeeds();
  let best = null;
  for (const seed of seeds) {
    const result = solveFlangeIkFromSeed(target, seed, options);
    if (!best || ikScore(result) < ikScore(best)) {
      best = result;
    }
    if (result.converged) {
      break;
    }
  }
  return best;
}

function buildIkSeeds() {
  const seeds = [currentJointAnglesRad()];
  for (const preset of Object.values(PRESETS)) {
    seeds.push(preset.map(degToRad));
  }
  return seeds;
}

function solveFlangeIkFromSeed(target, seed, options = {}) {
  const maxIterations = Math.max(1, Number(options.maxIterations) || IK_MAX_ITERATIONS);
  const damping = Number(options.damping) || IK_DAMPING;
  const joints = motionJoints();
  let q = clampJointAngles(seed.slice(0, joints.length));
  let best = null;

  for (let iteration = 0; iteration <= maxIterations; iteration += 1) {
    const error = flangePoseError(q, target);
    const result = {
      q: q.slice(),
      converged: error.positionNorm <= IK_POSITION_TOLERANCE_M && error.rotationNorm <= IK_ROTATION_TOLERANCE_RAD,
      iterations: iteration,
      positionError: error.positionNorm,
      rotationError: error.rotationNorm,
    };
    if (!best || ikScore(result) < ikScore(best)) {
      best = result;
    }
    if (result.converged || iteration === maxIterations) {
      return result.converged ? result : best;
    }

    const jacobian = finiteDifferenceIkJacobian(q, target, error.vector);
    const step = dampedLeastSquaresStep(jacobian, error.vector, damping);
    if (!step || !step.every(Number.isFinite)) {
      return best;
    }
    const limitedStep = limitIkStep(step);
    q = clampJointAngles(q.map((value, index) => value + limitedStep[index]));
  }

  return best;
}

function flangePoseError(q, target) {
  const pose = flangePoseFromJointAngles(q);
  const positionError = sub3(target.position, pose.position);
  const rotationError = [
    wrapAngle(target.rpy[0] - pose.rpy[0]),
    wrapAngle(target.rpy[1] - pose.rpy[1]),
    wrapAngle(target.rpy[2] - pose.rpy[2]),
  ];
  return {
    vector: [...positionError, ...rotationError],
    positionNorm: length3(positionError),
    rotationNorm: length3(rotationError),
  };
}

function flangePoseFromJointAngles(q) {
  const linkWorld = computeLinkWorld(jointMapFromQ(q));
  const flange = getFlangeWorld(linkWorld);
  return {
    position: transformPoint(flange, [0, 0, 0]),
    rpy: mat4ToRpy(flange),
  };
}

function finiteDifferenceIkJacobian(q, target, errorVector) {
  const rows = errorVector.length;
  const cols = q.length;
  const jacobian = Array.from({ length: rows }, () => new Array(cols).fill(0));
  for (let col = 0; col < cols; col += 1) {
    const perturbed = q.slice();
    perturbed[col] += IK_FINITE_STEP_RAD;
    const nextError = flangePoseError(perturbed, target).vector;
    for (let row = 0; row < rows; row += 1) {
      jacobian[row][col] = (errorVector[row] - nextError[row]) / IK_FINITE_STEP_RAD;
    }
  }
  return jacobian;
}

function dampedLeastSquaresStep(jacobian, error, damping) {
  const rows = jacobian.length;
  const cols = jacobian[0]?.length || 0;
  const a = Array.from({ length: rows }, () => new Array(rows).fill(0));
  for (let r = 0; r < rows; r += 1) {
    for (let c = 0; c < rows; c += 1) {
      let sum = 0;
      for (let k = 0; k < cols; k += 1) {
        sum += jacobian[r][k] * jacobian[c][k];
      }
      a[r][c] = sum + (r === c ? damping * damping : 0);
    }
  }
  const y = solveLinearSystem(a, error);
  if (!y) {
    return null;
  }
  const step = new Array(cols).fill(0);
  for (let col = 0; col < cols; col += 1) {
    for (let row = 0; row < rows; row += 1) {
      step[col] += jacobian[row][col] * y[row];
    }
  }
  return step;
}

function solveLinearSystem(matrix, rhs) {
  const n = rhs.length;
  const a = matrix.map((row, index) => row.slice().concat(rhs[index]));
  for (let col = 0; col < n; col += 1) {
    let pivot = col;
    for (let row = col + 1; row < n; row += 1) {
      if (Math.abs(a[row][col]) > Math.abs(a[pivot][col])) {
        pivot = row;
      }
    }
    if (Math.abs(a[pivot][col]) < 1e-10) {
      return null;
    }
    if (pivot !== col) {
      [a[col], a[pivot]] = [a[pivot], a[col]];
    }
    const divisor = a[col][col];
    for (let j = col; j <= n; j += 1) {
      a[col][j] /= divisor;
    }
    for (let row = 0; row < n; row += 1) {
      if (row === col) {
        continue;
      }
      const factor = a[row][col];
      for (let j = col; j <= n; j += 1) {
        a[row][j] -= factor * a[col][j];
      }
    }
  }
  return a.map((row) => row[n]);
}

function limitIkStep(step) {
  const maxAbs = Math.max(...step.map((value) => Math.abs(value)));
  if (maxAbs <= IK_MAX_STEP_RAD) {
    return step;
  }
  const scale = IK_MAX_STEP_RAD / maxAbs;
  return step.map((value) => value * scale);
}

function clampJointAngles(q) {
  return q.map((value, index) => {
    const joint = motionJoints()[index];
    return joint ? clamp(value, joint.lower, joint.upper) : value;
  });
}

function ikScore(result) {
  return result.positionError / IK_POSITION_TOLERANCE_M + result.rotationError / IK_ROTATION_TOLERANCE_RAD;
}

function formatIkStatus(result) {
  const prefix = result.converged ? "IK执行完成" : "IK未完全收敛";
  return `${prefix}: ${formatErrorMm(result.positionError)} mm / ${radToDeg(result.rotationError).toFixed(2)} deg`;
}

function formatErrorMm(valueM) {
  return (valueM * 1000).toFixed(valueM < 0.01 ? 2 : 1);
}

function updateFrameLines() {
  const vertices = [];
  const axisLength = 0.09;
  for (const linkName of DISPLAY_FRAME_LINKS) {
    const world = state.linkWorld.get(linkName);
    if (!world) {
      continue;
    }
    const origin = transformPoint(world, [0, 0, 0]);
    const x = transformPoint(world, [axisLength, 0, 0]);
    const y = transformPoint(world, [0, axisLength, 0]);
    const z = transformPoint(world, [0, 0, axisLength]);
    vertices.push(...origin, ...x, ...origin, ...y, ...origin, ...z);
  }
  updateLineBuffer(state.gl, state.frameBuffer, new Float32Array(vertices));
}

function draw() {
  updateTrajectoryPlayback(performance.now());
  resizeCanvas();
  const gl = state.gl;
  const width = gl.drawingBufferWidth;
  const height = gl.drawingBufferHeight;
  gl.viewport(0, 0, width, height);
  gl.clearColor(0.93, 0.95, 0.92, 1);
  gl.clearDepth(1);
  gl.enable(gl.DEPTH_TEST);
  gl.enable(gl.CULL_FACE);
  gl.cullFace(gl.BACK);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

  const projection = mat4Perspective(degToRad(42), width / Math.max(height, 1), 0.02, 20);
  const view = cameraViewMatrix();

  if (state.showGrid) {
    drawLines(state.gridBuffer, mat4Identity(), view, projection, [0.60, 0.64, 0.60], 1);
    drawLines(state.axisBuffer, mat4Identity(), view, projection, [0.24, 0.27, 0.26], 2);
  }

  // 打开"体素区域"时临时隐藏平台实体盒，让红色的平台占据格露出来。
  // 只是让位，不改动 showCart 开关本身的值。
  if (state.showCart && state.cartMesh && !viz.showVoxelRegion) {
    // 带孔平台：按固定模型快照的长方体分解绘制，孔是模型本身就有的，
    // 不是在完整实心平台外叠加的轮廓或透明圆柱。
    if (!vizDrawPlatformBoxes(view, projection)) {
      const cartModel = cartWorldMatrix();
      drawMesh(state.cartMesh, cartModel, view, projection);
      drawLines(state.cartEdgeBuffer, cartModel, view, projection, [0.16, 0.18, 0.17], 1);
    }
  }

  if (state.showPointCloud && state.pointCloudBuffer?.count > 0) {
    drawPoints(state.pointCloudBuffer, mat4Identity(), view, projection, DEFAULT_POINT_COLOR, state.pointSize);
  }

  if (state.showWorkpieceExclusion && state.workpieceExclusionEdgeBuffer?.count > 0) {
    drawLines(
      state.workpieceExclusionEdgeBuffer,
      mat4Identity(),
      view,
      projection,
      state.workpieceExclusionColor,
      2,
    );
  }

  if (state.showMesh) {
    for (const link of state.robot?.links || []) {
      const mesh = state.linkMeshes.get(link.name);
      const model = state.linkWorld.get(link.name);
      if (mesh && model) {
        drawMesh(mesh, model, view, projection);
      }
    }
  }

  if ((state.showCollisionSpheres || state.showEndEffectorSpheres || state.showActivationShell)
      && state.collisionSphereMesh) {
    drawCollisionSpheres(view, projection);
  }

  if (state.showTool && state.toolMesh) {
    drawMesh(state.toolMesh, toolWorldMatrix(), view, projection);
  }

  if (state.showTrajectory && state.trajectoryBuffer?.count > 0) {
    drawLines(state.trajectoryBuffer, mat4Identity(), view, projection, [0.12, 0.38, 0.92], 2);
  }

  if (state.showTrajectory && state.trajectoryBuffer2?.count > 0) {
    drawLines(state.trajectoryBuffer2, mat4Identity(), view, projection, [0.08, 0.65, 0.32], 2);
  }

  if (state.showFrames) {
    drawFrameAxes(view, projection);
    if (state.stagePoseBuffer?.count > 0) {
      drawAxesBuffer(state.stagePoseBuffer, view, projection);
    }
  }

  // 第三流程覆盖层：两组夹爪预览与安装豁免区轮廓（只用于对照，不加入碰撞世界）。
  vizDrawOverlays(view, projection);

  if (state.showGraspCandidatePoses) {
    for (const type of GRASP_POSE_TYPES) {
      const buffer = state.graspCandidatePoseBuffers[type.field];
      if (state.showGraspPoseTypes[type.field] && buffer?.count > 0) {
        drawAxesBuffer(
          buffer,
          view,
          projection,
          GRASP_CANDIDATE_AXIS_OPACITY,
          2,
        );
      }
    }
  }

  requestAnimationFrame(draw);
}

function cartWorldMatrix() {
  const sizeX = state.cart.negativeX + state.cart.positiveX;
  const sizeY = state.cart.negativeY + state.cart.positiveY;
  const sizeZ = state.cart.negativeZ + state.cart.positiveZ;
  const center = mat4Translation(
    0.5 * (state.cart.positiveX - state.cart.negativeX),
    0.5 * (state.cart.positiveY - state.cart.negativeY),
    0.5 * (state.cart.positiveZ - state.cart.negativeZ),
  );
  return mat4Multiply(center, mat4Scale(sizeX, sizeY, sizeZ));
}

/**
 * 统一管理 alpha 混合状态。
 *
 * 此前 `drawMeshWithOpacity` 只设 opacity uniform，把 gl.BLEND 的开关甩给调用方：
 * `drawCollisionSpheres` 自己开了，`drawLines` 按 opacity 判断，而**夹爪预览这条路
 * 一次都没开**——于是 VIZ_GRIPPER_OPACITY=0.35 被 GL 直接丢弃，两只夹爪被画成实心，
 * 后画的退让位盖住原始位（两者沿工具轴只错开 84mm，而夹爪盒长 200mm，重叠 58%），
 * 看上去就是"叠成一坨"。
 *
 * 现在由本函数统一管理，任何绘制路径都不可能再漏开混合。带状态缓存，重复调用是空操作。
 */
function setBlendMode(enabled) {
  const gl = state.gl;
  if (state.blendEnabled === enabled) {
    return;
  }
  if (enabled) {
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    // 半透明体之间不写深度，避免互相遮挡成实心块。
    gl.depthMask(false);
  } else {
    gl.depthMask(true);
    gl.disable(gl.BLEND);
  }
  state.blendEnabled = enabled;
}

function drawMesh(mesh, model, view, projection) {
  const gl = state.gl;
  setBlendMode(false);
  const program = state.program;
  gl.useProgram(program.program);

  gl.bindBuffer(gl.ARRAY_BUFFER, mesh.buffers.position);
  gl.enableVertexAttribArray(program.attributes.position);
  gl.vertexAttribPointer(program.attributes.position, 3, gl.FLOAT, false, 0, 0);

  gl.bindBuffer(gl.ARRAY_BUFFER, mesh.buffers.normal);
  gl.enableVertexAttribArray(program.attributes.normal);
  gl.vertexAttribPointer(program.attributes.normal, 3, gl.FLOAT, false, 0, 0);

  gl.uniformMatrix4fv(program.uniforms.model, false, model);
  gl.uniformMatrix4fv(program.uniforms.view, false, view);
  gl.uniformMatrix4fv(program.uniforms.projection, false, projection);
  gl.uniform3fv(program.uniforms.color, mesh.color);
  gl.uniform3fv(program.uniforms.lightDir, normalize3([-0.35, 0.55, 0.78]));
  gl.uniform3fv(program.uniforms.cameraPos, cameraPosition());
  gl.uniform1f(program.uniforms.opacity, 1.0);
  gl.drawArrays(gl.TRIANGLES, 0, mesh.vertexCount);
}

function drawCollisionSpheres(view, projection) {
  // 混合状态由 drawMeshWithOpacity 经 setBlendMode 统一管理，这里不再自己开关。
  const drawGroup = (spheres, mesh, opacity, radiusBonus) => {
    if (!mesh) {
      return;
    }
    for (const sphere of spheres) {
      const linkWorld = state.linkWorld.get(sphere.linkName);
      if (!linkWorld) {
        continue;
      }
      const radius = sphere.radius + radiusBonus;
      const model = mat4Multiply(
        linkWorld,
        mat4Multiply(
          mat4Translation(sphere.center[0], sphere.center[1], sphere.center[2]),
          mat4Scale(radius, radius, radius),
        ),
      );
      drawMeshWithOpacity(mesh, model, view, projection, opacity);
    }
  };

  // 改动十：三层各自独立开关，**只控制显示**，不影响规划中的碰撞检测。
  if (state.showCollisionSpheres) {
    drawGroup(state.collisionSpheres, state.collisionSphereMesh, 0.28, 0);
  }
  if (state.showEndEffectorSpheres) {
    drawGroup(state.endEffectorSpheres, state.endEffectorSphereMesh, 0.28, 0);
  }
  // 激活距离层：每个球外再画 半径 + 激活距离 的一层淡壳，直观显示安全余量厚度。
  if (state.showActivationShell && state.collisionActivationM > 0) {
    drawGroup(state.collisionSpheres, state.collisionSphereMesh, 0.10,
              state.collisionActivationM);
    drawGroup(state.endEffectorSpheres, state.endEffectorSphereMesh, 0.10,
              state.collisionActivationM);
  }
  setBlendMode(false);
}

function drawMeshWithOpacity(mesh, model, view, projection, opacity, colorOverride = null) {
  const gl = state.gl;
  setBlendMode(opacity < 1.0);
  const program = state.program;
  gl.useProgram(program.program);
  gl.bindBuffer(gl.ARRAY_BUFFER, mesh.buffers.position);
  gl.enableVertexAttribArray(program.attributes.position);
  gl.vertexAttribPointer(program.attributes.position, 3, gl.FLOAT, false, 0, 0);
  gl.bindBuffer(gl.ARRAY_BUFFER, mesh.buffers.normal);
  gl.enableVertexAttribArray(program.attributes.normal);
  gl.vertexAttribPointer(program.attributes.normal, 3, gl.FLOAT, false, 0, 0);
  gl.uniformMatrix4fv(program.uniforms.model, false, model);
  gl.uniformMatrix4fv(program.uniforms.view, false, view);
  gl.uniformMatrix4fv(program.uniforms.projection, false, projection);
  gl.uniform3fv(program.uniforms.color, colorOverride || mesh.color);
  gl.uniform3fv(program.uniforms.lightDir, normalize3([-0.35, 0.55, 0.78]));
  gl.uniform3fv(program.uniforms.cameraPos, cameraPosition());
  gl.uniform1f(program.uniforms.opacity, opacity);
  gl.drawArrays(gl.TRIANGLES, 0, mesh.vertexCount);
}

function drawLines(lineBuffer, model, view, projection, color, width) {
  const gl = state.gl;
  setBlendMode(false);
  const program = state.lineProgram;
  gl.useProgram(program.program);
  gl.bindBuffer(gl.ARRAY_BUFFER, lineBuffer.buffer);
  gl.enableVertexAttribArray(program.attributes.position);
  gl.vertexAttribPointer(program.attributes.position, 3, gl.FLOAT, false, 0, 0);
  gl.uniformMatrix4fv(program.uniforms.model, false, model);
  gl.uniformMatrix4fv(program.uniforms.view, false, view);
  gl.uniformMatrix4fv(program.uniforms.projection, false, projection);
  gl.uniform3fv(program.uniforms.color, color);
  gl.uniform1f(program.uniforms.opacity, 1.0);
  gl.lineWidth(width);
  gl.drawArrays(gl.LINES, 0, lineBuffer.count);
}

function drawPoints(pointBuffer, model, view, projection, color, pointSize) {
  const gl = state.gl;
  setBlendMode(false);
  const program = state.pointProgram;
  const usesVertexColor = state.pointCloudHasColors && state.pointCloudColorBuffer?.count === pointBuffer.count;
  gl.useProgram(program.program);
  gl.bindBuffer(gl.ARRAY_BUFFER, pointBuffer.buffer);
  gl.enableVertexAttribArray(program.attributes.position);
  gl.vertexAttribPointer(program.attributes.position, 3, gl.FLOAT, false, 0, 0);
  if (program.attributes.color >= 0) {
    if (usesVertexColor) {
      gl.bindBuffer(gl.ARRAY_BUFFER, state.pointCloudColorBuffer.buffer);
      gl.enableVertexAttribArray(program.attributes.color);
      gl.vertexAttribPointer(program.attributes.color, 3, gl.FLOAT, false, 0, 0);
    } else {
      gl.disableVertexAttribArray(program.attributes.color);
      gl.vertexAttrib3f(program.attributes.color, color[0], color[1], color[2]);
    }
  }
  gl.uniformMatrix4fv(program.uniforms.model, false, model);
  gl.uniformMatrix4fv(program.uniforms.view, false, view);
  gl.uniformMatrix4fv(program.uniforms.projection, false, projection);
  gl.uniform3fv(program.uniforms.color, color);
  gl.uniform1f(program.uniforms.pointSize, pointSize);
  gl.uniform1i(program.uniforms.useVertexColor, usesVertexColor ? 1 : 0);
  gl.drawArrays(gl.POINTS, 0, pointBuffer.count);
}

function drawFrameAxes(view, projection) {
  drawAxesBuffer(state.frameBuffer, view, projection);
}

function drawAxesBuffer(buffer, view, projection, opacity = 1.0, width = 1) {
  const gl = state.gl;
  const program = state.lineProgram;
  gl.useProgram(program.program);
  gl.bindBuffer(gl.ARRAY_BUFFER, buffer.buffer);
  gl.enableVertexAttribArray(program.attributes.position);
  gl.vertexAttribPointer(program.attributes.position, 3, gl.FLOAT, false, 0, 0);
  gl.uniformMatrix4fv(program.uniforms.model, false, mat4Identity());
  gl.uniformMatrix4fv(program.uniforms.view, false, view);
  gl.uniformMatrix4fv(program.uniforms.projection, false, projection);
  gl.uniform1f(program.uniforms.opacity, opacity);
  gl.lineWidth(width);

  setBlendMode(opacity < 1.0);

  const colors = [
    [0.78, 0.15, 0.12],
    [0.12, 0.55, 0.25],
    [0.13, 0.34, 0.82],
  ];
  for (let axis = 0; axis < 3; axis += 1) {
    gl.uniform3fv(program.uniforms.color, colors[axis]);
    const linkCount = buffer.count / 6;
    for (let link = 0; link < linkCount; link += 1) {
      gl.drawArrays(gl.LINES, link * 6 + axis * 2, 2);
    }
  }
  setBlendMode(false);
}

function setupEvents() {
  window.addEventListener("resize", resizeCanvas);
  state.canvas.addEventListener("contextmenu", (event) => event.preventDefault());

  state.canvas.addEventListener("pointerdown", (event) => {
    state.pointer.active = true;
    state.pointer.id = event.pointerId;
    state.pointer.mode = event.button === 2 ? "pan" : "orbit";
    state.pointer.x = event.clientX;
    state.pointer.y = event.clientY;
    state.canvas.setPointerCapture(event.pointerId);
  });

  state.canvas.addEventListener("pointermove", (event) => {
    if (!state.pointer.active || event.pointerId !== state.pointer.id) {
      return;
    }
    const dx = event.clientX - state.pointer.x;
    const dy = event.clientY - state.pointer.y;
    state.pointer.x = event.clientX;
    state.pointer.y = event.clientY;
    if (state.pointer.mode === "pan") {
      panCamera(dx, dy);
    } else {
      state.camera.yaw -= dx * 0.006;
      state.camera.pitch = clamp(state.camera.pitch + dy * 0.005, -1.15, 1.25);
    }
  });

  state.canvas.addEventListener("pointerup", endPointer);
  state.canvas.addEventListener("pointercancel", endPointer);

  state.canvas.addEventListener(
    "wheel",
    (event) => {
      event.preventDefault();
      const scale = Math.exp(event.deltaY * 0.001);
      state.camera.distance = clamp(state.camera.distance * scale, 0.7, 6.0);
    },
    { passive: false },
  );

  el.toggleMesh.addEventListener("change", () => {
    state.showMesh = el.toggleMesh.checked;
  });
  el.toggleCollisionSpheres.addEventListener("change", () => {
    state.showCollisionSpheres = el.toggleCollisionSpheres.checked;
  });
  // 改动十：末端球与激活距离层各自独立开关，互不影响；关闭不卸载模型，
  // 再次打开不重新生成球，也不影响规划中的碰撞检测。
  if (el.toggleEndEffectorSpheres) {
    el.toggleEndEffectorSpheres.addEventListener("change", () => {
      state.showEndEffectorSpheres = el.toggleEndEffectorSpheres.checked;
    });
  }
  if (el.toggleActivationShell) {
    el.toggleActivationShell.addEventListener("change", () => {
      state.showActivationShell = el.toggleActivationShell.checked;
    });
  }
  el.toggleCart.addEventListener("change", () => {
    state.showCart = el.toggleCart.checked;
  });
  el.toggleTool.addEventListener("change", () => {
    state.showTool = el.toggleTool.checked;
  });
  el.toggleGrid.addEventListener("change", () => {
    state.showGrid = el.toggleGrid.checked;
  });
  el.toggleFrames.addEventListener("change", () => {
    state.showFrames = el.toggleFrames.checked;
  });
  el.togglePointCloud.addEventListener("change", () => {
    state.showPointCloud = el.togglePointCloud.checked;
  });
  el.toggleWorkpieceExclusion.addEventListener("change", () => {
    state.showWorkpieceExclusion = el.toggleWorkpieceExclusion.checked;
  });
  el.toggleTrajectory.addEventListener("change", () => {
    state.showTrajectory = el.toggleTrajectory.checked;
  });
  el.toggleGraspCandidatePoses.addEventListener("change", () => {
    state.showGraspCandidatePoses = el.toggleGraspCandidatePoses.checked;
    updateGraspCandidatePoseStatus();
  });
  const graspPoseTypeInputs = {
    grasp_flange: el.toggleGraspFlangePoses,
    grasp_tcp: el.toggleGraspTcpPoses,
    trajectory_grasp_flange: el.toggleTrajectoryGraspFlangePoses,
    trajectory_grasp_tcp: el.toggleTrajectoryGraspTcpPoses,
  };
  for (const [field, input] of Object.entries(graspPoseTypeInputs)) {
    input.addEventListener("change", () => {
      state.showGraspPoseTypes[field] = input.checked;
      updateGraspCandidatePoseStatus();
    });
  }

  el.applyJointAngles.addEventListener("click", applyJointAnglesFromInputs);
  el.applyJointAnglesList.addEventListener("click", applyJointAnglesFromList);
  el.readJointAngles.addEventListener("click", () => {
    syncJointAngleInputs(true);
    el.jointStatus.textContent = "已读取当前关节角";
  });

  el.applyFlangePose.addEventListener("click", applyFlangePoseFromInputs);
  el.applyFlangePoseList.addEventListener("click", applyFlangePoseFromList);
  el.readFlangePose.addEventListener("click", () => {
    syncFlangePoseInputsFromCurrent(true);
    el.flangePoseStatus.textContent = "已读取当前法兰姿态";
  });
  for (const input of Object.values(el.flangePose)) {
    input.addEventListener("keydown", (event) => {
      if (event.key === "Enter") {
        event.preventDefault();
        applyFlangePoseFromInputs();
      }
    });
  }
  for (const input of [el.jointAnglesList, el.flangePoseList]) {
    input.addEventListener("keydown", (event) => {
      if (event.key === "Enter" && (event.ctrlKey || event.metaKey)) {
        event.preventDefault();
        if (input === el.jointAnglesList) {
          applyJointAnglesFromList();
        } else {
          applyFlangePoseFromList();
        }
      }
    });
  }
}

function yamlSection(text, path) {
  const lines = text.split(/\r?\n/);
  let start = 0;
  let end = lines.length;
  let parentIndent = -1;
  for (const key of path) {
    let found = -1;
    let foundIndent = -1;
    for (let index = start; index < end; index += 1) {
      const raw = lines[index];
      const clean = raw.replace(/\s+#.*$/, "");
      if (!clean.trim() || clean.trimStart().startsWith("#")) {
        continue;
      }
      const indent = raw.length - raw.trimStart().length;
      if (indent <= parentIndent || clean.trim() !== `${key}:`) {
        continue;
      }
      found = index;
      foundIndent = indent;
      break;
    }
    if (found < 0) {
      throw new Error(`config.yaml 缺少 ${path.join(".")}`);
    }
    start = found + 1;
    end = lines.length;
    for (let index = start; index < lines.length; index += 1) {
      const raw = lines[index];
      if (!raw.trim() || raw.trimStart().startsWith("#")) {
        continue;
      }
      const indent = raw.length - raw.trimStart().length;
      if (indent <= foundIndent) {
        end = index;
        break;
      }
    }
    parentIndent = foundIndent;
  }
  return lines.slice(start, end).join("\n");
}

function yamlField(text, key) {
  const escaped = key.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
  const match = text.match(new RegExp(`^\\s*${escaped}\\s*:\\s*([^#\\n]+)`, "m"));
  if (!match) {
    throw new Error(`config.yaml 缺少 ${key}`);
  }
  return match[1].trim().replace(/^['"]|['"]$/g, "");
}

function yamlNumberArray(text, key, length) {
  const value = yamlField(text, key);
  if (!value.startsWith("[") || !value.endsWith("]")) {
    throw new Error(`${key} 必须使用 [a, b, c] 格式`);
  }
  const numbers = value.slice(1, -1).split(",").map((item) => Number(item.trim()));
  if (numbers.length !== length || !numbers.every(Number.isFinite)) {
    throw new Error(`${key} 必须包含 ${length} 个有效数值`);
  }
  return numbers;
}

function configDirectory(path) {
  const index = path.lastIndexOf("/");
  return index < 0 ? "." : path.slice(0, index);
}

async function fetchText(path) {
  const response = await fetch(path, { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`无法读取 ${path}`);
  }
  return response.text();
}

async function expandJsonIncludes(value, basePath) {
  if (Array.isArray(value)) {
    return Promise.all(value.map((item) => expandJsonIncludes(item, basePath)));
  }
  if (value && typeof value === "object") {
    if (typeof value.$include === "string") {
      const includePath = value.$include.startsWith("/")
        ? value.$include
        : `${configDirectory(basePath)}/${value.$include}`;
      const included = JSON.parse(await fetchText(includePath));
      return expandJsonIncludes(included, includePath);
    }
    const output = {};
    await Promise.all(Object.entries(value).map(async ([key, child]) => {
      output[key] = await expandJsonIncludes(child, basePath);
    }));
    return output;
  }
  return value;
}

async function resolveStartupConfig(path, text) {
  const trimmed = text.trim();
  if (!trimmed.startsWith("{")) {
    return { kind: "yaml", text };
  }
  const document = await expandJsonIncludes(JSON.parse(trimmed), path);
  return { kind: "json", document, text };
}

function parseConfiguredPlatformJson(app) {
  const platform = app.base_platform || {};
  const negativeMm = (platform.negative_extent_xyz_mm || []).map(Number);
  const positiveMm = (platform.positive_extent_xyz_mm || []).map(Number);
  if (negativeMm.length !== 3 || positiveMm.length !== 3
      || ![...negativeMm, ...positiveMm].every(Number.isFinite)) {
    throw new Error("app.base_platform 六个延伸值必须是数值");
  }
  if ([...negativeMm, ...positiveMm].some((value) => value < 0)) {
    throw new Error("基座平台六个延伸值必须非负");
  }
  if (negativeMm.some((value, index) => value + positiveMm[index] <= 0)) {
    throw new Error("基座平台每个轴的正负延伸之和必须大于 0");
  }
  return { negativeMm, positiveMm };
}

function parseConfiguredToolJson(app) {
  const tool = app.end_effector || {};
  const negativeMm = (tool.negative_extent_xyz_mm || []).map(Number);
  const positiveMm = (tool.positive_extent_xyz_mm || []).map(Number);
  const colorRgb = (tool.color_rgb || [69, 148, 122]).map(Number);
  if (negativeMm.length !== 3 || positiveMm.length !== 3
      || ![...negativeMm, ...positiveMm].every(Number.isFinite)) {
    throw new Error("app.end_effector 六个延伸值必须是数值");
  }
  if ((tool.parent_frame || "flange_link") !== "flange_link") {
    throw new Error("parent_frame 当前必须为 flange_link");
  }
  if ((tool.geometry_type || "box") !== "box") {
    throw new Error("geometry.type 当前只支持 box");
  }
  if ([...negativeMm, ...positiveMm].some((value) => value < 0)) {
    throw new Error("末端六个方向延伸距离都必须非负");
  }
  if (negativeMm.some((value, index) => value + positiveMm[index] <= 0)) {
    throw new Error("启用末端时，每个轴的正负延伸之和都必须大于 0");
  }
  return {
    enabled: true,
    parentFrame: "flange_link",
    negativeMm,
    positiveMm,
    colorRgb,
    allowTemporaryOverride: tool.allow_temporary_override !== false,
  };
}

async function loadWorkpieceYamlText(resolved, requestText) {
  if (resolved.kind === "yaml") {
    return resolved.text;
  }
  return "";
}

function parseConfiguredPlatformYaml(text) {
  const section = yamlSection(text, ["environment", "base_platform"]);
  const negativeMm = yamlNumberArray(section, "negative_extent_xyz_mm", 3);
  const positiveMm = yamlNumberArray(section, "positive_extent_xyz_mm", 3);
  if ([...negativeMm, ...positiveMm].some((value) => value < 0)) {
    throw new Error("基座平台六个延伸值必须非负");
  }
  if (negativeMm.some((value, index) => value + positiveMm[index] <= 0)) {
    throw new Error("基座平台每个轴的正负延伸之和必须大于 0");
  }
  return { negativeMm, positiveMm };
}

function applyConfiguredPlatform(config) {
  state.configuredPlatform = config;
  state.cart = {
    negativeX: mmToM(config.negativeMm[0]),
    positiveX: mmToM(config.positiveMm[0]),
    negativeY: mmToM(config.negativeMm[1]),
    positiveY: mmToM(config.positiveMm[1]),
    negativeZ: mmToM(config.negativeMm[2]),
    positiveZ: mmToM(config.positiveMm[2]),
  };
  state.showCart = true;
  el.toggleCart.checked = true;
  syncConfigInputs();
  const min = config.negativeMm.map((value) => -value);
  const max = config.positiveMm;
  el.platformStatus.textContent = `已加载 ${state.configSourceLabel} / AGV 碰撞盒 X=${min[0]}..${max[0]}, Y=${min[1]}..${max[1]}, Z=${min[2]}..${max[2]} mm`;
}

function parseConfiguredToolYaml(text) {
  const schemaVersion = Number(yamlField(text, "schema_version"));
  const section = yamlSection(text, ["end_effector"]);
  const geometry = yamlSection(text, ["end_effector", "geometry"]);
  const visualization = yamlSection(text, ["end_effector", "visualization"]);
  const enabled = yamlField(section, "enabled").toLowerCase() === "true";
  const parentFrame = yamlField(section, "parent_frame");
  const geometryType = yamlField(geometry, "type").toLowerCase();
  const negativeMm = yamlNumberArray(geometry, "negative_extent_xyz_mm", 3);
  const positiveMm = yamlNumberArray(geometry, "positive_extent_xyz_mm", 3);
  const colorRgb = yamlNumberArray(visualization, "color_rgb", 3);
  const allowTemporaryOverride = yamlField(visualization, "allow_temporary_override").toLowerCase() === "true";
  if (schemaVersion !== 6) {
    throw new Error("schema_version 必须为 6");
  }
  if (parentFrame !== "flange_link") {
    throw new Error("parent_frame 当前必须为 flange_link");
  }
  if (geometryType !== "box") {
    throw new Error("geometry.type 当前只支持 box");
  }
  if ([...negativeMm, ...positiveMm].some((value) => value < 0)) {
    throw new Error("末端六个方向延伸距离都必须非负");
  }
  if (enabled && negativeMm.some((value, index) => value + positiveMm[index] <= 0)) {
    throw new Error("启用末端时，每个轴的正负延伸之和都必须大于 0");
  }
  if (!colorRgb.every((value) => value >= 0 && value <= 255)) {
    throw new Error("color_rgb 必须在 0..255");
  }
  return {
    enabled,
    parentFrame,
    negativeMm,
    positiveMm,
    colorRgb,
    allowTemporaryOverride,
  };
}

function applyConfiguredTool(config) {
  state.configuredTool = config;
  state.tool = {
    ...state.tool,
    negativeX: mmToM(config.negativeMm[0]),
    positiveX: mmToM(config.positiveMm[0]),
    negativeY: mmToM(config.negativeMm[1]),
    positiveY: mmToM(config.positiveMm[1]),
    negativeZ: mmToM(config.negativeMm[2]),
    positiveZ: mmToM(config.positiveMm[2]),
  };
  state.toolColor = config.colorRgb.map((value) => value / 255);
  state.toolSource = "config";
  state.toolMesh = createRenderableMesh(state.gl, buildUnitBoxMesh(), state.toolColor);
  state.showTool = config.enabled;
  el.toggleTool.checked = config.enabled;
  el.toolSourceMode.value = "config";
  syncConfigInputs();
  const min = config.negativeMm.map((value) => -value);
  const max = config.positiveMm;
  el.toolStatus.textContent = config.enabled
    ? `已加载 ${state.configSourceLabel} / 唯一 flange_link 坐标系 / X=${min[0]}..${max[0]}, Y=${min[1]}..${max[1]}, Z=${min[2]}..${max[2]} mm`
    : [...config.negativeMm, ...config.positiveMm].every((value) => value === 0)
      ? `已加载 ${state.configSourceLabel} / flange_link 六向延伸均为 0 mm，当前不显示`
      : `已加载 ${state.configSourceLabel} / 法兰末端矩形已禁用`;
}

function requestKeyValues(text) {
  const values = new Map();
  let pendingKey = null;
  let pending = [];
  let depth = 0;
  let rawMatrixRows = 0;
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.split("#", 1)[0].trim();
    if (!line) {
      continue;
    }
    if (pendingKey !== null) {
      pending.push(line);
      if (rawMatrixRows > 0) {
        rawMatrixRows -= 1;
      } else {
        depth += (line.match(/\[/g) || []).length - (line.match(/\]/g) || []).length;
      }
      if (rawMatrixRows === 0 && depth <= 0) {
        values.set(pendingKey, pending.join(" "));
        pendingKey = null;
        pending = [];
      }
      continue;
    }
    const equals = line.indexOf("=");
    if (equals < 0) {
      continue;
    }
    const key = line.slice(0, equals).trim();
    const value = line.slice(equals + 1).trim();
    if (key === "T_B_O_target" && value === "") {
      pendingKey = key;
      pending = [];
      rawMatrixRows = 4;
      depth = 0;
      continue;
    }
    depth = (value.match(/\[/g) || []).length - (value.match(/\]/g) || []).length;
    if (depth > 0) {
      pendingKey = key;
      pending = [value];
    } else {
      values.set(key, value);
    }
  }
  if (pendingKey !== null) {
    throw new Error(`input/request.txt 中 ${pendingKey} 的多行数据不完整`);
  }
  return values;
}

function requestField(text, key) {
  const value = requestKeyValues(text).get(key);
  if (value === undefined) {
    throw new Error(`input/request.txt 缺少 ${key}`);
  }
  return value.trim().replace(/^['"]|['"]$/g, "");
}

function requestNumberArray(text, key, length) {
  const numbers = requestField(text, key)
    .replace(/[\[\],]/g, " ")
    .trim()
    .split(/\s+/)
    .map(Number);
  if (numbers.length !== length || !numbers.every(Number.isFinite)) {
    throw new Error(`${key} 必须包含 ${length} 个有效数值`);
  }
  return numbers;
}

function parseConfiguredWorkpieceYaml(configText, requestText) {
  const code = requestField(requestText, "workpiece_type").toUpperCase();
  const section = yamlSection(configText, ["workpieces", "types", code]);
  const collision = yamlSection(
    configText, ["workpieces", "types", code, "collision_box"],
  );
  const exclusion = yamlSection(
    configText, ["workpieces", "types", code, "point_cloud_exclusion"],
  );
  const visualization = yamlSection(configText, ["workpieces", "visualization"]);
  const configured = yamlField(section, "configured").toLowerCase() === "true";
  const collisionNegativeMm = yamlNumberArray(collision, "negative_extent_xyz_mm", 3);
  const collisionPositiveMm = yamlNumberArray(collision, "positive_extent_xyz_mm", 3);
  const exclusionNegativeMm = yamlNumberArray(exclusion, "negative_extent_xyz_mm", 3);
  const exclusionPositiveMm = yamlNumberArray(exclusion, "positive_extent_xyz_mm", 3);
  const colorRgb = yamlNumberArray(visualization, "exclusion_color_rgb", 3);
  const requestValues = requestKeyValues(requestText);
  let pose;
  let poseLabel;
  let T_B_O;
  if (requestValues.has("T_B_O_target_xyzrpy")) {
    pose = requestNumberArray(requestText, "T_B_O_target_xyzrpy", 6);
    poseLabel = `{O}=[${pose.map((value) => Number(value.toFixed(4))).join(", ")}] mm/deg`;
    T_B_O = mat4Multiply(
      mat4Translation(mmToM(pose[0]), mmToM(pose[1]), mmToM(pose[2])),
      mat4FromRpy(degToRad(pose[3]), degToRad(pose[4]), degToRad(pose[5])),
    );
  } else {
    const matrix = requestNumberArray(requestText, "T_B_O_target", 16);
    T_B_O = mat4FromRows(
      [matrix[0], matrix[1], matrix[2], mmToM(matrix[3])],
      [matrix[4], matrix[5], matrix[6], mmToM(matrix[7])],
      [matrix[8], matrix[9], matrix[10], mmToM(matrix[11])],
      [matrix[12], matrix[13], matrix[14], matrix[15]],
    );
    poseLabel = "{O}=request 4×4 matrix（平移mm）";
    pose = null;
  }
  if (!configured) {
    throw new Error(`${code} 尚未配置，正式可视化拒绝使用`);
  }
  const extents = [
    ...collisionNegativeMm, ...collisionPositiveMm,
    ...exclusionNegativeMm, ...exclusionPositiveMm,
  ];
  if (extents.some((value) => value < 0)) {
    throw new Error(`${code} 六向延伸不能为负数`);
  }
  const collisionEnabled = [...collisionNegativeMm, ...collisionPositiveMm]
    .some((value) => value > 0);
  const exclusionEnabled = [...exclusionNegativeMm, ...exclusionPositiveMm]
    .some((value) => value > 0);
  if (collisionEnabled
      && collisionNegativeMm.some(
        (value, index) => value + collisionPositiveMm[index] <= 0
      )) {
    throw new Error(`${code} 实体盒必须全零，或者每个轴都有正尺寸`);
  }
  if (exclusionEnabled
      && exclusionNegativeMm.some(
        (value, index) => value + exclusionPositiveMm[index] <= 0
      )) {
    throw new Error(`${code} 点云豁免盒必须全零，或者每个轴都有正尺寸`);
  }
  if (collisionEnabled && exclusionEnabled
      && (exclusionNegativeMm.some((value, index) => value < collisionNegativeMm[index])
        || exclusionPositiveMm.some((value, index) => value < collisionPositiveMm[index]))) {
    throw new Error(`${code} 点云豁免盒必须完整包含实体碰撞盒`);
  }
  return {
    code,
    pose,
    poseLabel,
    T_B_O,
    collisionEnabled,
    exclusionEnabled,
    collisionNegativeMm,
    collisionPositiveMm,
    exclusionNegativeMm,
    exclusionPositiveMm,
    colorRgb,
  };
}

function transformedUnitBoxEdges(model) {
  const unit = buildUnitBoxEdgeVertices();
  const output = new Float32Array(unit.length);
  for (let index = 0; index < unit.length; index += 3) {
    const point = transformPoint(model, [unit[index], unit[index + 1], unit[index + 2]]);
    output.set(point, index);
  }
  return output;
}

function applyConfiguredWorkpiece(config) {
  state.configuredWorkpiece = config;
  state.workpieceExclusionColor = config.colorRgb.map((value) => value / 255);
  if (config.exclusionEnabled) {
    const negativeM = config.exclusionNegativeMm.map(mmToM);
    const positiveM = config.exclusionPositiveMm.map(mmToM);
    const centerO = positiveM.map((value, index) => 0.5 * (value - negativeM[index]));
    const size = positiveM.map((value, index) => value + negativeM[index]);
    const boxModel = mat4Multiply(
      config.T_B_O,
      mat4Multiply(
        mat4Translation(centerO[0], centerO[1], centerO[2]),
        mat4Scale(size[0], size[1], size[2]),
      ),
    );
    updateLineBuffer(
      state.gl,
      state.workpieceExclusionEdgeBuffer,
      transformedUnitBoxEdges(boxModel),
    );
  } else {
    updateLineBuffer(
      state.gl,
      state.workpieceExclusionEdgeBuffer,
      new Float32Array(0),
    );
  }
  const values = {
    negativeX: config.exclusionNegativeMm[0],
    positiveX: config.exclusionPositiveMm[0],
    negativeY: config.exclusionNegativeMm[1],
    positiveY: config.exclusionPositiveMm[1],
    negativeZ: config.exclusionNegativeMm[2],
    positiveZ: config.exclusionPositiveMm[2],
  };
  for (const input of document.querySelectorAll("[data-workpiece-field]")) {
    input.value = String(values[input.dataset.workpieceField]);
  }
  state.showWorkpieceExclusion = config.exclusionEnabled;
  el.toggleWorkpieceExclusion.checked = config.exclusionEnabled;
  const geometryLabel = config.exclusionEnabled
    ? "红色点=三维点云豁免；红框使用同一六向边界"
    : config.collisionEnabled
      ? "点云豁免盒为全零：仿真继续，不绘制豁免盒，不标红点云"
      : "实体盒/点云豁免盒为全零：仿真继续，不绘制豁免盒，不标红点云";
  el.workpieceStatus.textContent =
    `${config.code} / ${config.poseLabel} / ${geometryLabel}`;
  if (state.pointCloudRecords.length) {
    setPointCloudData(state.pointCloudRecords, state.pointCloudMeta);
  }
}

async function loadConfiguredProjectGeometry() {
  state.configSourceLabel = PROJECT_CONFIG_PATH;
  el.toolStatus.textContent = `正在读取 ${PROJECT_CONFIG_PATH}...`;
  el.platformStatus.textContent = `正在读取 ${PROJECT_CONFIG_PATH}...`;
  el.workpieceStatus.textContent = `正在读取 ${PROJECT_CONFIG_PATH} + input/request.txt...`;
  const [configResponse, requestResponse] = await Promise.all([
    fetch(PROJECT_CONFIG_PATH, { cache: "no-store" }),
    fetch(PROJECT_REQUEST_PATH, { cache: "no-store" }),
  ]);
  if (!configResponse.ok) {
    throw new Error(`无法读取 ${PROJECT_CONFIG_PATH}`);
  }
  if (!requestResponse.ok) {
    throw new Error(`无法读取 ${PROJECT_REQUEST_PATH}`);
  }
  const [configText, requestText] = await Promise.all([
    configResponse.text(), requestResponse.text(),
  ]);
  const resolved = await resolveStartupConfig(PROJECT_CONFIG_PATH, configText);
  if (resolved.kind === "json") {
    applyConfiguredPlatform(parseConfiguredPlatformJson(resolved.document.app || {}));
    applyConfiguredTool(parseConfiguredToolJson(resolved.document.app || {}));
  } else {
    applyConfiguredPlatform(parseConfiguredPlatformYaml(configText));
    applyConfiguredTool(parseConfiguredToolYaml(configText));
  }
  const workpieceYaml = await loadWorkpieceYamlText(resolved, requestText);
  if (workpieceYaml && workpieceYaml.includes("workpieces:")) {
    applyConfiguredWorkpiece(parseConfiguredWorkpieceYaml(workpieceYaml, requestText));
  } else if (el.workpieceStatus) {
    el.workpieceStatus.textContent = "未配置工件豁免盒：仿真继续，不标红点云";
  }
}

async function loadArchivedProjectGeometry(paths) {
  el.toolStatus.textContent = `正在读取 ${paths.config}...`;
  el.platformStatus.textContent = `正在读取 ${paths.config}...`;
  el.workpieceStatus.textContent =
    `正在读取 ${paths.config} + ${paths.request}...`;
  const [configBuffer, requestBuffer] = await Promise.all([
    fetchAbsoluteFile(paths.config),
    fetchAbsoluteFile(paths.request),
  ]);
  const decoder = new TextDecoder("utf-8");
  const configText = decoder.decode(configBuffer);
  const requestText = decoder.decode(requestBuffer);
  state.configSourceLabel = paths.config;
  applyConfiguredPlatform(parseConfiguredPlatformYaml(configText));
  applyConfiguredTool(parseConfiguredToolYaml(configText));
  applyConfiguredWorkpiece(
    parseConfiguredWorkpieceYaml(configText, requestText),
  );
}

async function loadConfiguredTool() {
  state.configSourceLabel = PROJECT_CONFIG_PATH;
  el.toolStatus.textContent = `正在读取 ${PROJECT_CONFIG_PATH}...`;
  const response = await fetch(PROJECT_CONFIG_PATH, { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`无法读取 ${PROJECT_CONFIG_PATH}`);
  }
  const text = await response.text();
  const resolved = await resolveStartupConfig(PROJECT_CONFIG_PATH, text);
  if (resolved.kind === "json") {
    applyConfiguredTool(parseConfiguredToolJson(resolved.document.app || {}));
  } else {
    applyConfiguredTool(parseConfiguredToolYaml(text));
  }
}

async function loadConfiguredPlatform() {
  state.configSourceLabel = PROJECT_CONFIG_PATH;
  el.platformStatus.textContent = `正在读取 ${PROJECT_CONFIG_PATH}...`;
  const response = await fetch(PROJECT_CONFIG_PATH, { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`无法读取 ${PROJECT_CONFIG_PATH}`);
  }
  const text = await response.text();
  const resolved = await resolveStartupConfig(PROJECT_CONFIG_PATH, text);
  if (resolved.kind === "json") {
    applyConfiguredPlatform(parseConfiguredPlatformJson(resolved.document.app || {}));
  } else {
    applyConfiguredPlatform(parseConfiguredPlatformYaml(text));
  }
}

function markTemporaryTool(label = "临时自定义矩形") {
  state.toolSource = "temporary";
  state.toolMesh = createRenderableMesh(state.gl, buildUnitBoxMesh(), [0.88, 0.48, 0.16]);
  el.toolSourceMode.value = "custom";
  el.toolStatus.textContent = `仅本次可视化：${label}；未进入轨迹规划和碰撞检测`;
}

function applyTemporaryToolFromInputs() {
  if (state.configuredTool && !state.configuredTool.allowTemporaryOverride) {
    el.toolStatus.textContent = "config.yaml 禁止临时自定义末端";
    el.toolSourceMode.value = "config";
    return;
  }
  if (!validToolExtents(state.tool)) {
    el.toolStatus.textContent = "临时矩形六向延伸必须非负，且每个轴的正负延伸之和必须大于 0";
    return;
  }
  state.showTool = true;
  el.toggleTool.checked = true;
  markTemporaryTool();
}

function setupConfigControls() {
  for (const input of document.querySelectorAll("[data-cart-field]")) {
    const key = input.dataset.cartField;
    input.value = formatMm(state.cart[key]);
    input.addEventListener("input", () => {
      const value = Number(input.value);
      if (Number.isFinite(value)) {
        state.cart[key] = mmToM(value);
        el.platformStatus.textContent = "仅本次可视化：已修改 config 平台的显示副本；未进入规划碰撞检测";
      }
    });
  }

  for (const input of document.querySelectorAll("[data-tool-field]")) {
    const key = input.dataset.toolField;
    input.value = formatMm(state.tool[key]);
    input.addEventListener("input", () => {
      if (state.configuredTool && !state.configuredTool.allowTemporaryOverride) {
        syncConfigInputs();
        el.toolSourceMode.value = "config";
        el.toolStatus.textContent = "config.yaml 禁止临时自定义末端";
        return;
      }
      const value = Number(input.value);
      if (Number.isFinite(value)) {
        state.tool[key] = mmToM(value);
        if (state.toolSource === "config") {
          markTemporaryTool("已修改 config 默认值的显示副本");
        }
      }
    });
  }

  el.loadConfiguredTool.addEventListener("click", () => {
    loadConfiguredTool().catch((error) => {
      el.toolStatus.textContent = `config 加载失败: ${error.message}`;
    });
  });
  el.loadConfiguredPlatform.addEventListener("click", () => {
    loadConfiguredPlatform().catch((error) => {
      el.platformStatus.textContent = `config 加载失败: ${error.message}`;
    });
  });
  el.loadConfiguredWorkpiece.addEventListener("click", () => {
    loadConfiguredProjectGeometry().catch((error) => {
      el.workpieceStatus.textContent = `工件配置加载失败: ${error.message}`;
    });
  });
  el.applyCustomTool.addEventListener("click", applyTemporaryToolFromInputs);
  el.toolSourceMode.addEventListener("change", () => {
    if (el.toolSourceMode.value === "config") {
      loadConfiguredTool().catch((error) => {
        el.toolStatus.textContent = `config 加载失败: ${error.message}`;
      });
    } else {
      applyTemporaryToolFromInputs();
    }
  });

  el.pointSize.addEventListener("input", () => {
    const value = Number(el.pointSize.value);
    if (Number.isFinite(value)) {
      state.pointSize = clamp(value, 1, 20);
    }
  });

  el.loadPointCloud.addEventListener("click", () => {
    try {
      setPointCloudData(parsePointCloudText(el.pointCloudText.value));
    } catch (error) {
      el.pointCloudStatus.textContent = `点云解析失败: ${error.message}`;
    }
  });
  el.loadPointCloudPath.addEventListener("click", async () => {
    try {
      const path = requireAbsolutePath(el.pointCloudPath.value, "点云文件");
      el.pointCloudPath.value = path;
      el.pointCloudStatus.textContent = `正在加载 ${path}`;
      const buffer = await fetchAbsoluteFile(path);
      const result = path.toLowerCase().endsWith(".ply")
        ? parsePlyPointCloud(buffer)
        : { points: parsePointCloudText(new TextDecoder("utf-8").decode(buffer)) };
      setPointCloudData(result.points, {
        ...result,
        sourceLabel: path,
      });
    } catch (error) {
      el.pointCloudStatus.textContent = `绝对路径点云加载失败: ${error.message}`;
    }
  });
  el.clearPointCloud.addEventListener("click", () => {
    setPointCloudData([]);
  });
  el.pointCloudFile.addEventListener("change", async () => {
    const file = el.pointCloudFile.files?.[0];
    if (!file) {
      return;
    }
    try {
      if (file.name.toLowerCase().endsWith(".ply")) {
        el.pointCloudStatus.textContent = `正在加载 PLY: ${file.name}`;
        const result = parsePlyPointCloud(await file.arrayBuffer());
        setPointCloudData(result.points, result);
      } else {
        setPointCloudData(parsePointCloudText(await file.text()));
      }
    } catch (error) {
      el.pointCloudStatus.textContent = `点云解析失败: ${error.message}`;
    }
  });

  el.trajectorySpeed.addEventListener("input", () => {
    const value = Number(el.trajectorySpeed.value);
    if (Number.isFinite(value)) {
      state.trajectory.speed = clamp(value, 0.05, 5);
    }
  });
  el.loadTrajectory.addEventListener("click", async () => {
    try {
      const photoFile = el.photoToGraspTrajectoryFile.files?.[0];
      const placeFile = el.graspToPlaceTrajectoryFile.files?.[0];
      if (!photoFile || !placeFile) {
        throw new Error("请分别选择拍照→抓取和抓取→放置两个轨迹文件");
      }
      await loadTrajectoryFiles([photoFile, placeFile], {
        expectedRoutes: ["photo_to_grasp", "grasp_to_place"],
        requirePair: true,
      });
    } catch (error) {
      el.trajectoryStatus.textContent = `轨迹解析失败: ${error.message}`;
    }
  });
  el.loadTrajectoryPaths.addEventListener("click", async () => {
    try {
      const paths = [
        requireAbsolutePath(
          el.photoToGraspTrajectoryPath.value, "拍照→抓取轨迹"
        ),
        requireAbsolutePath(
          el.graspToPlaceTrajectoryPath.value, "抓取→放置轨迹"
        ),
      ];
      el.photoToGraspTrajectoryPath.value = paths[0];
      el.graspToPlaceTrajectoryPath.value = paths[1];
      const files = await Promise.all(paths.map(fileFromAbsolutePath));
      await loadTrajectoryFiles(files, {
        expectedRoutes: ["photo_to_grasp", "grasp_to_place"],
        requirePair: true,
        validateAgainstCurrentWorkpiece: false,
        sourceLabel: paths.join(" + "),
      });
    } catch (error) {
      el.trajectoryStatus.textContent = `绝对路径轨迹加载失败: ${error.message}`;
    }
  });
  el.loadLatestPipelineOutput.addEventListener("click", async () => {
    try {
      await loadLatestPipelineOutput();
    } catch (error) {
      el.pipelineStatus.textContent = `最新两阶段 output 加载失败: ${error.message}`;
      el.trajectoryStatus.textContent = `轨迹未更新: ${error.message}`;
    }
  });
  el.loadLatestTrajectories.addEventListener("click", async () => {
    try {
      await loadLatestPipelineOutput({
        includeStageResult: false,
        includePointCloud: false,
      });
    } catch (error) {
      el.pipelineStatus.textContent = `最新 manifest 轨迹加载失败: ${error.message}`;
      el.trajectoryStatus.textContent = `轨迹未更新: ${error.message}`;
    }
  });
  el.photoToGraspTrajectoryFile.addEventListener("change", () => {
    const file = el.photoToGraspTrajectoryFile.files?.[0];
    el.photoToGraspTrajectorySelection.textContent = file?.name || "未选择文件";
    el.trajectoryStatus.textContent = "轨迹文件已更新，请点击“加载两条轨迹”";
  });
  el.graspToPlaceTrajectoryFile.addEventListener("change", () => {
    const file = el.graspToPlaceTrajectoryFile.files?.[0];
    el.graspToPlaceTrajectorySelection.textContent = file?.name || "未选择文件";
    el.trajectoryStatus.textContent = "轨迹文件已更新，请点击“加载两条轨迹”";
  });
  el.playTrajectory.addEventListener("click", playTrajectory);
  el.pauseTrajectory.addEventListener("click", pauseTrajectory);
  el.stopTrajectory.addEventListener("click", stopTrajectory);

  const defaultAbsolutePaths = [
    [el.pipelineOutputPath, DEFAULT_OUTPUT_ROOT_ABSOLUTE_PATH],
    [
      el.pointCloudPath,
      joinAbsolutePath(
        DEFAULT_OUTPUT_ROOT_ABSOLUTE_PATH,
        "trajectory_planning",
        "point_cloud_B.ply",
      ),
    ],
    [
      el.photoToGraspTrajectoryPath,
      joinAbsolutePath(
        DEFAULT_OUTPUT_ROOT_ABSOLUTE_PATH,
        "trajectory_planning",
        "joint_trajectory_photo_to_grasp.json",
      ),
    ],
    [
      el.graspToPlaceTrajectoryPath,
      joinAbsolutePath(
        DEFAULT_OUTPUT_ROOT_ABSOLUTE_PATH,
        "trajectory_planning",
        "joint_trajectory_grasp_to_place.json",
      ),
    ],
  ];
  for (const [input, defaultPath] of defaultAbsolutePaths) {
    if (!String(input.value || "").trim()) {
      input.value = defaultPath;
    }
    input.addEventListener("blur", () => {
      if (!String(input.value || "").trim()) {
        input.value = defaultPath;
      }
    });
  }

  el.pointCloudText.value = DEFAULT_POINT_CLOUD.map((p) => p.join(",")).join("\n");
}

function syncConfigInputs() {
  for (const input of document.querySelectorAll("[data-cart-field]")) {
    input.value = formatMm(state.cart[input.dataset.cartField]);
  }
  for (const input of document.querySelectorAll("[data-tool-field]")) {
    const key = input.dataset.toolField;
    input.value = formatMm(state.tool[key]);
  }
}

function pickFinite(source, keys) {
  const out = {};
  if (!source || typeof source !== "object") {
    return out;
  }
  for (const key of keys) {
    const value = Number(source[key]);
    if (Number.isFinite(value)) {
      out[key] = value;
    }
  }
  return out;
}

function pickFiniteMmAsMeters(source, keys) {
  const out = pickFinite(source, keys);
  for (const key of Object.keys(out)) {
    out[key] = mmToM(out[key]);
  }
  return out;
}

function validToolExtents(tool) {
  const negative = [tool.negativeX, tool.negativeY, tool.negativeZ];
  const positive = [tool.positiveX, tool.positiveY, tool.positiveZ];
  return [...negative, ...positive].every((value) => Number.isFinite(value) && value >= 0)
    && negative.every((value, index) => value + positive[index] > 0);
}

function formatMm(meters) {
  const mm = mToMm(Number(meters) || 0);
  return Number.isInteger(mm) ? String(mm) : mm.toFixed(1);
}

function requireAbsolutePath(value, label) {
  const rawPath = String(value || "").trim();
  if (!rawPath) {
    throw new Error(`${label}绝对路径不能为空`);
  }
  const path = rawPath === "/" ? rawPath : rawPath.replace(/\/+$/, "");
  if (!path.startsWith("/")) {
    throw new Error(`${label}必须填写以 / 开头的绝对路径`);
  }
  return path;
}

function joinAbsolutePath(root, ...parts) {
  const normalizedRoot = requireAbsolutePath(root, "根目录");
  return [
    normalizedRoot.replace(/\/+$/, ""),
    ...parts.map((part) => String(part).replace(/^\/+|\/+$/g, "")),
  ].filter(Boolean).join("/");
}

function absoluteFileUrl(path) {
  return `${ABSOLUTE_FILE_ENDPOINT}?path=${encodeURIComponent(
    requireAbsolutePath(path, "文件")
  )}`;
}

async function fetchAbsoluteFile(path) {
  const normalized = requireAbsolutePath(path, "文件");
  const response = await fetch(absoluteFileUrl(normalized), {
    cache: "no-store",
  });
  if (!response.ok) {
    let detail = "";
    try {
      const document = await response.json();
      detail = document?.error ? `: ${document.error}` : "";
    } catch (_error) {
      detail = "";
    }
    throw new Error(`${normalized}: HTTP ${response.status}${detail}`);
  }
  return response.arrayBuffer();
}

function absoluteBaseName(path) {
  const parts = requireAbsolutePath(path, "文件").split("/");
  return parts[parts.length - 1] || "artifact";
}

async function fileFromAbsolutePath(path) {
  const normalized = requireAbsolutePath(path, "轨迹文件");
  return new File(
    [await fetchAbsoluteFile(normalized)],
    absoluteBaseName(normalized),
    { type: "application/json" },
  );
}

function pipelineArtifactPaths(outputRoot) {
  const requestedRoot = requireAbsolutePath(
    outputRoot, "两阶段 output 文件夹"
  );
  const trajectoryDirectory = requestedRoot.endsWith("/trajectory_planning")
    ? requestedRoot
    : joinAbsolutePath(requestedRoot, "trajectory_planning");
  const root = requestedRoot.endsWith("/trajectory_planning")
    ? requestedRoot.slice(0, -"/trajectory_planning".length)
    : requestedRoot;
  return {
    root,
    config: joinAbsolutePath(root, "config_snapshot.yaml"),
    request: joinAbsolutePath(root, "request_snapshot.txt"),
    graspResult: joinAbsolutePath(
      root, "grasp_generation", "grasp_result.json"
    ),
    manifest: joinAbsolutePath(
      trajectoryDirectory, "trajectory_manifest.json"
    ),
    photoToGrasp: joinAbsolutePath(
      trajectoryDirectory, "joint_trajectory_photo_to_grasp.json"
    ),
    graspToPlace: joinAbsolutePath(
      trajectoryDirectory, "joint_trajectory_grasp_to_place.json"
    ),
    pointCloud: joinAbsolutePath(trajectoryDirectory, "point_cloud_B.ply"),
  };
}

async function loadPointCloudFromUrlParam() {
  const params = new URLSearchParams(window.location.search);
  const path = params.get("pointcloud") || params.get("ply");
  if (!path) {
    return;
  }
  try {
    const urlPointSize = Number(params.get("pointSize"));
    if (Number.isFinite(urlPointSize)) {
      state.pointSize = clamp(urlPointSize, 1, 20);
      el.pointSize.value = String(state.pointSize);
    }
    el.pointCloudStatus.textContent = `正在加载 PLY: ${path}`;
    const response = await fetch(path);
    if (!response.ok) {
      throw new Error(`${response.status} ${response.statusText}`);
    }
    const maxPoints = Number(params.get("maxPoints")) || DEFAULT_MAX_PLY_POINTS;
    const result = path.toLowerCase().endsWith(".ply")
      ? parsePlyPointCloud(await response.arrayBuffer(), { maxPoints })
      : { points: parsePointCloudText(await response.text()) };
    setPointCloudData(result.points, result);
  } catch (error) {
    el.pointCloudStatus.textContent = `点云URL加载失败: ${error.message}`;
  }
}

function endPointer(event) {
  if (event.pointerId !== state.pointer.id) {
    return;
  }
  state.pointer.active = false;
  state.pointer.id = null;
}

function panCamera(dx, dy) {
  const eye = cameraPosition();
  const forward = normalize3(sub3(state.camera.target, eye));
  let right = cross3(forward, [0, 0, 1]);
  if (length3(right) < 0.000001) {
    right = [1, 0, 0];
  } else {
    right = normalize3(right);
  }
  const screenUp = normalize3(cross3(right, forward));
  const canvasHeight = Math.max(state.canvas.clientHeight || state.canvas.height, 1);
  const metersPerPixel = (2 * state.camera.distance * Math.tan(degToRad(42) / 2)) / canvasHeight;
  const delta = add3(mul3(right, -dx * metersPerPixel), mul3(screenUp, dy * metersPerPixel));
  state.camera.target = add3(state.camera.target, delta);
}

function parsePointCloudText(text) {
  const trimmed = text.trim();
  if (!trimmed) {
    return [];
  }
  if (trimmed.startsWith("[") || trimmed.startsWith("{")) {
    const json = JSON.parse(trimmed);
    const points = Array.isArray(json) ? json : json.points || json.cloud || [];
    return normalizePointList(points);
  }
  const points = [];
  for (const line of trimmed.split(/\r?\n/)) {
    const clean = line.trim();
    if (!clean || clean.startsWith("#")) {
      continue;
    }
    const values = clean.split(/[,\s]+/).map(Number).filter(Number.isFinite);
    if (values.length >= 3) {
      points.push(values.slice(0, 6));
    }
  }
  return points;
}

function parsePlyPointCloud(buffer, options = {}) {
  const bytes = new Uint8Array(buffer);
  const headerEnd = findPlyHeaderEnd(bytes);
  if (headerEnd === null) {
    throw new Error("PLY header 未找到 end_header");
  }

  const headerText = new TextDecoder("ascii").decode(bytes.slice(0, headerEnd.headerBytes));
  const header = parsePlyHeader(headerText);
  if (!["ascii", "binary_little_endian", "binary_big_endian"].includes(header.format)) {
    throw new Error(`不支持的 PLY format: ${header.format}`);
  }
  if (!header.vertex || header.vertex.count <= 0) {
    return { points: [], sourceCount: 0, displayedCount: 0, stride: 1 };
  }

  const xIndex = header.vertex.properties.findIndex((prop) => prop.name === "x");
  const yIndex = header.vertex.properties.findIndex((prop) => prop.name === "y");
  const zIndex = header.vertex.properties.findIndex((prop) => prop.name === "z");
  if (xIndex < 0 || yIndex < 0 || zIndex < 0) {
    throw new Error("PLY vertex 缺少 x/y/z property");
  }
  const colorIndices = findPlyColorIndices(header.vertex.properties);

  const maxPoints = Math.max(1, Number(options.maxPoints) || DEFAULT_MAX_PLY_POINTS);
  const stride = Math.max(1, Math.ceil(header.vertex.count / maxPoints));
  let points;
  if (header.format === "ascii") {
    points = parseAsciiPlyPoints(
      bytes,
      headerEnd.dataOffset,
      header.vertex.count,
      xIndex,
      yIndex,
      zIndex,
      colorIndices,
      stride,
    );
  } else {
    points = parseBinaryPlyPoints(
      bytes,
      headerEnd.dataOffset,
      header.vertex,
      xIndex,
      yIndex,
      zIndex,
      colorIndices,
      header.format === "binary_little_endian",
      stride,
    );
  }
  return {
    points,
    sourceCount: header.vertex.count,
    displayedCount: points.length,
    stride,
    format: header.format,
    hasColors: Boolean(colorIndices),
  };
}

function findPlyHeaderEnd(bytes) {
  const marker = new TextEncoder().encode("end_header");
  for (let i = 0; i <= bytes.length - marker.length; i += 1) {
    let matched = true;
    for (let j = 0; j < marker.length; j += 1) {
      if (bytes[i + j] !== marker[j]) {
        matched = false;
        break;
      }
    }
    if (!matched) {
      continue;
    }
    let end = i + marker.length;
    if (bytes[end] === 13 && bytes[end + 1] === 10) {
      end += 2;
    } else if (bytes[end] === 10 || bytes[end] === 13) {
      end += 1;
    }
    return { headerBytes: i + marker.length, dataOffset: end };
  }
  return null;
}

function parsePlyHeader(headerText) {
  const lines = headerText.split(/\r?\n/).map((line) => line.trim()).filter(Boolean);
  if (lines[0] !== "ply") {
    throw new Error("不是有效 PLY 文件");
  }
  const header = {
    format: "",
    vertex: null,
  };
  let currentElement = null;

  for (const line of lines.slice(1)) {
    const parts = line.split(/\s+/);
    if (parts[0] === "comment" || parts[0] === "obj_info") {
      continue;
    }
    if (parts[0] === "format") {
      header.format = parts[1];
      continue;
    }
    if (parts[0] === "element") {
      currentElement = {
        name: parts[1],
        count: Number(parts[2]),
        properties: [],
      };
      if (currentElement.name === "vertex") {
        header.vertex = currentElement;
      }
      continue;
    }
    if (parts[0] === "property" && currentElement) {
      if (parts[1] === "list") {
        currentElement.properties.push({
          kind: "list",
          countType: parts[2],
          itemType: parts[3],
          name: parts[4],
        });
      } else {
        currentElement.properties.push({
          kind: "scalar",
          type: parts[1],
          name: parts[2],
        });
      }
    }
  }
  return header;
}

function findPlyColorIndices(properties) {
  const names = properties.map((prop) => prop.name);
  const candidates = [
    ["red", "green", "blue"],
    ["r", "g", "b"],
    ["diffuse_red", "diffuse_green", "diffuse_blue"],
    ["diffuse_r", "diffuse_g", "diffuse_b"],
  ];
  for (const candidate of candidates) {
    const indices = candidate.map((name) => names.indexOf(name));
    if (indices.every((index) => index >= 0)) {
      return indices;
    }
  }
  return null;
}

function parseAsciiPlyPoints(bytes, dataOffset, count, xIndex, yIndex, zIndex, colorIndices, stride) {
  const text = new TextDecoder("utf-8").decode(bytes.slice(dataOffset));
  const points = [];
  let sourceIndex = 0;
  for (const line of text.split(/\r?\n/)) {
    if (sourceIndex >= count) {
      break;
    }
    const parts = line.trim().split(/\s+/);
    if (parts.length <= Math.max(xIndex, yIndex, zIndex)) {
      continue;
    }
    if (sourceIndex % stride === 0) {
      const point = [Number(parts[xIndex]), Number(parts[yIndex]), Number(parts[zIndex])];
      if (colorIndices) {
        point.push(Number(parts[colorIndices[0]]), Number(parts[colorIndices[1]]), Number(parts[colorIndices[2]]));
      }
      points.push(point);
    }
    sourceIndex += 1;
  }
  return normalizePointRecords(points);
}

function parseBinaryPlyPoints(bytes, dataOffset, vertex, xIndex, yIndex, zIndex, colorIndices, littleEndian, stride) {
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const points = [];
  let offset = dataOffset;
  for (let i = 0; i < vertex.count; i += 1) {
    const values = [];
    for (const property of vertex.properties) {
      if (property.kind === "list") {
        const countValue = readPlyScalar(view, offset, property.countType, littleEndian);
        offset += plyTypeSize(property.countType);
        offset += countValue * plyTypeSize(property.itemType);
        values.push(undefined);
      } else {
        values.push(readPlyScalar(view, offset, property.type, littleEndian));
        offset += plyTypeSize(property.type);
      }
    }
    if (i % stride === 0) {
      const point = [values[xIndex], values[yIndex], values[zIndex]];
      if (colorIndices) {
        point.push(values[colorIndices[0]], values[colorIndices[1]], values[colorIndices[2]]);
      }
      points.push(point);
    }
  }
  return normalizePointRecords(points);
}

function readPlyScalar(view, offset, type, littleEndian) {
  switch (normalizePlyType(type)) {
    case "char": return view.getInt8(offset);
    case "uchar": return view.getUint8(offset);
    case "short": return view.getInt16(offset, littleEndian);
    case "ushort": return view.getUint16(offset, littleEndian);
    case "int": return view.getInt32(offset, littleEndian);
    case "uint": return view.getUint32(offset, littleEndian);
    case "float": return view.getFloat32(offset, littleEndian);
    case "double": return view.getFloat64(offset, littleEndian);
    default: throw new Error(`不支持的 PLY property 类型: ${type}`);
  }
}

function plyTypeSize(type) {
  switch (normalizePlyType(type)) {
    case "char":
    case "uchar":
      return 1;
    case "short":
    case "ushort":
      return 2;
    case "int":
    case "uint":
    case "float":
      return 4;
    case "double":
      return 8;
    default:
      throw new Error(`不支持的 PLY property 类型: ${type}`);
  }
}

function normalizePlyType(type) {
  const aliases = {
    int8: "char",
    uint8: "uchar",
    int16: "short",
    uint16: "ushort",
    int32: "int",
    uint32: "uint",
    float32: "float",
    float64: "double",
  };
  return aliases[type] || type;
}

function normalizePointList(points) {
  const out = [];
  for (const point of points) {
    if (Array.isArray(point) && point.length >= 3) {
      out.push([Number(point[0]), Number(point[1]), Number(point[2])]);
    } else if (point && typeof point === "object") {
      out.push([Number(point.x), Number(point.y), Number(point.z)]);
    }
  }
  return out.filter((point) => point.every(Number.isFinite));
}

function normalizePointRecords(points) {
  const out = [];
  for (const point of points) {
    if (Array.isArray(point) && point.length >= 3) {
      const record = [Number(point[0]), Number(point[1]), Number(point[2])];
      if (point.length >= 6) {
        record.push(Number(point[3]), Number(point[4]), Number(point[5]));
      }
      if (record.slice(0, 3).every(Number.isFinite) && (record.length === 3 || record.slice(3, 6).every(Number.isFinite))) {
        out.push(record);
      }
    } else if (point && typeof point === "object") {
      const record = [Number(point.x), Number(point.y), Number(point.z)];
      const r = point.r ?? point.red;
      const g = point.g ?? point.green;
      const b = point.b ?? point.blue;
      if (r !== undefined || g !== undefined || b !== undefined) {
        record.push(Number(r), Number(g), Number(b));
      }
      if (record.slice(0, 3).every(Number.isFinite) && (record.length === 3 || record.slice(3, 6).every(Number.isFinite))) {
        out.push(record);
      }
    }
  }
  return out;
}

function normalizePointColors(points, expectedCount) {
  const out = [];
  for (const point of points) {
    let color = null;
    if (Array.isArray(point) && point.length >= 6) {
      color = [Number(point[3]), Number(point[4]), Number(point[5])];
    } else if (point && typeof point === "object") {
      const r = point.r ?? point.red;
      const g = point.g ?? point.green;
      const b = point.b ?? point.blue;
      color = [Number(r), Number(g), Number(b)];
    }
    if (!color || !color.every(Number.isFinite)) {
      return null;
    }
    out.push(color.map(normalizeColorChannel));
  }
  return out.length === expectedCount ? out : null;
}

function normalizeColorChannel(value) {
  if (value > 1) {
    return clamp(value / 255, 0, 1);
  }
  return clamp(value, 0, 1);
}

function pointInConfiguredWorkpieceExclusion(pointMm) {
  const config = state.configuredWorkpiece;
  if (!config || !config.exclusionEnabled) {
    return false;
  }
  const point = pointMm.map(mmToM);
  const transform = config.T_B_O;
  const delta = [
    point[0] - transform[12],
    point[1] - transform[13],
    point[2] - transform[14],
  ];
  // column-major R 的转置：把 {B} 点转换到工件几何中心坐标系 {O}。
  const local = [
    transform[0] * delta[0] + transform[1] * delta[1] + transform[2] * delta[2],
    transform[4] * delta[0] + transform[5] * delta[1] + transform[6] * delta[2],
    transform[8] * delta[0] + transform[9] * delta[1] + transform[10] * delta[2],
  ];
  const epsilonMm = 1e-5;
  return local.every((value, index) => {
    const valueMm = mToMm(value);
    return valueMm >= -config.exclusionNegativeMm[index] - epsilonMm
      && valueMm <= config.exclusionPositiveMm[index] + epsilonMm;
  });
}

function setPointCloudData(points, meta = {}) {
  const records = normalizePointRecords(points);
  const normalized = normalizePointList(records);
  state.pointCloudRecords = records;
  state.pointCloudMeta = { ...meta };
  const data = new Float32Array(normalized.map((point) => point.map(mmToM)).flat());
  updateLineBuffer(state.gl, state.pointCloudBuffer, data);
  const sourceColors = normalizePointColors(records, normalized.length);
  const colors = sourceColors
    ? sourceColors.map((color) => color.slice())
    : normalized.map(() => DEFAULT_POINT_COLOR.slice());
  let excludedCount = 0;
  for (let index = 0; index < normalized.length; index += 1) {
    if (pointInConfiguredWorkpieceExclusion(normalized[index])) {
      colors[index] = state.workpieceExclusionColor.slice();
      excludedCount += 1;
    }
  }
  state.pointCloudHasColors = true;
  state.pointCloudSourceHasColors = Boolean(sourceColors);
  updateLineBuffer(
    state.gl,
    state.pointCloudColorBuffer,
    new Float32Array(colors.flat()),
  );
  const colorLabel = sourceColors ? " / 原始RGB保留" : " / 非目标灰色";
  const exclusionLabel = state.configuredWorkpiece?.exclusionEnabled
    ? ` / ${state.configuredWorkpiece.code} 红色豁免点 ${excludedCount.toLocaleString("zh-CN")}`
    : state.configuredWorkpiece
      ? ` / ${state.configuredWorkpiece.code} 豁免盒全零，未标红`
    : " / 工件豁免配置未加载";
  if (meta.sourceCount && meta.sourceCount !== normalized.length) {
    el.pointCloudStatus.textContent =
      `基座坐标系 / mm / 显示 ${normalized.length.toLocaleString("zh-CN")} 点` +
      ` / 原始 ${meta.sourceCount.toLocaleString("zh-CN")} 点 / stride ${meta.stride || 1}` +
      colorLabel + exclusionLabel;
  } else {
    el.pointCloudStatus.textContent =
      `基座坐标系 / mm / ${normalized.length.toLocaleString("zh-CN")} 点` +
      colorLabel + exclusionLabel;
  }
}

function parseTrajectoryText(text) {
  const trimmed = text.trim();
  if (!trimmed) {
    return { unit: "deg", points: [] };
  }
  if (trimmed.startsWith("[") || trimmed.startsWith("{")) {
    return JSON.parse(trimmed);
  }
  const points = [];
  for (const line of trimmed.split(/\r?\n/)) {
    const clean = line.trim();
    if (!clean || clean.startsWith("#")) {
      continue;
    }
    const values = clean.split(/[,\s]+/).map(Number).filter(Number.isFinite);
    if (values.length >= 7) {
      points.push({ t: values[0], q: values.slice(1, 7) });
    } else if (values.length >= 6) {
      points.push({ q: values.slice(0, 6) });
    }
  }
  return { unit: "deg", points };
}

function requireFinitePose(value, field) {
  if (!Array.isArray(value) || value.length !== 6) {
    throw new Error(`${field} 必须是 6 个数值`);
  }
  const pose = value.map(Number);
  if (!pose.every(Number.isFinite)) {
    throw new Error(`${field} 包含非有限数值`);
  }
  return pose;
}

function validateGraspCandidatePoseDocument(document) {
  if (!Array.isArray(document?.candidates)) {
    throw new Error("grasp_result.json 缺少 candidates 数组");
  }
  const candidates = document.candidates
    .slice(0, MAX_GRASP_CANDIDATES)
    .map((source, sourceIndex) => {
      const score = Number(source?.score);
      if (!Number.isFinite(score)) {
        throw new Error(`candidates[${sourceIndex}].score 不是有限数值`);
      }
      const candidate = { score, sourceIndex };
      for (const type of GRASP_POSE_TYPES) {
        candidate[type.field] = requireFinitePose(
          source?.[type.field],
          `candidates[${sourceIndex}].${type.field}`,
        );
      }
      return candidate;
    });
  candidates.sort((left, right) => (
    right.score - left.score || left.sourceIndex - right.sourceIndex
  ));
  return candidates;
}

/**
 * @brief 只为**本轮最终选中的那个候选**建立四组位姿坐标系缓冲。
 *
 * 原先为全部 10 个候选各画 4 个坐标系（共 40 个 / 120 条轴），全挤在抓取点附近；
 * 而标签库的标签绕工件一圈，工件紧贴料框时背面几个候选会落到框壁上（本轮实测
 * 5~9 号的 IK 全部 0/4 不收敛），画出来只会让人误以为抓取点跑偏。现在只画选中的那个。
 *
 * 选中编号来自 manifest 的 `grasp_preview.selected_candidate_index`，与本函数读的
 * `grasp_result.json` 是两份文件、两条时序。故：
 * - 选中编号还没到（manifest 未加载 / 本轮无解）→ **一个都不画**，不拿别的候选顶替；
 * - 选中编号后到或发生变化 → 由 `rebuildGraspCandidatePoses()` 重建，不必等
 *   grasp_result.json 再次变化。
 *
 * @param candidates 已校验的候选（按 score 降序，带 sourceIndex = 原始编号）
 */
function applyGraspCandidatePoses(candidates) {
  const verticesByType = Object.fromEntries(
    GRASP_POSE_TYPES.map((type) => [type.field, []])
  );
  const selected = Number(viz.preview?.selected_candidate_index);
  const shown = (Number.isFinite(selected) && selected >= 0)
    ? candidates.filter((item) => Number(item.sourceIndex) === selected)
    : [];
  state.graspCandidatePoseWatcher.shownCount = shown.length;
  state.graspCandidatePoseWatcher.selectedIndex =
    (Number.isFinite(selected) && selected >= 0) ? selected : null;
  for (const candidate of shown) {
    for (const type of GRASP_POSE_TYPES) {
      verticesByType[type.field].push(
        ...buildPoseAxisVertices(
          poseMmDegToMat4(candidate[type.field]),
          type.axisLengthM,
        ),
      );
    }
  }
  // 四个缓冲区在整份 JSON 校验成功后才一起替换，
  // 避免文件更新中途将半份候选显示到画面。
  for (const type of GRASP_POSE_TYPES) {
    updateLineBuffer(
      state.gl,
      state.graspCandidatePoseBuffers[type.field],
      new Float32Array(verticesByType[type.field]),
    );
  }
  state.graspCandidatePoseWatcher.candidates = candidates;
  state.graspCandidatePoseWatcher.loadedAt = new Date();
  updateGraspCandidatePoseStatus();
}

function updateGraspCandidatePoseStatus(error = null) {
  const watcher = state.graspCandidatePoseWatcher;
  const count = watcher.candidates.length;
  if (error) {
    const retained = count > 0 ? `；保留上一版 ${count} 组` : "";
    el.graspCandidatePoseStatus.textContent =
      `抓取坐标系读取失败: ${error.message}${retained}`;
    return;
  }
  const enabledTypeCount = GRASP_POSE_TYPES.filter(
    (type) => state.showGraspPoseTypes[type.field]
  ).length;
  // 只画选中的那一个候选，故可见数按"已显示的候选数"算，不是全部候选数。
  const shownCandidates = Number(watcher.shownCount) || 0;
  const visibleCount = state.showGraspCandidatePoses
    ? shownCandidates * enabledTypeCount
    : 0;
  const scoreLabel = count > 0
    ? ` / score ${watcher.candidates[0].score.toFixed(6)}..${watcher.candidates.at(-1).score.toFixed(6)}`
    : "";
  const timeLabel = watcher.loadedAt
    ? ` / ${watcher.loadedAt.toLocaleTimeString("zh-CN", { hour12: false })}`
    : "";
  const selected = Number(viz.preview?.selected_candidate_index);
  const selectedLabel = (Number.isFinite(selected) && selected >= 0)
    ? `选中 #${selected}`
    : "本轮未选出候选";
  el.graspCandidatePoseStatus.textContent =
    `${GRASP_RESULT_PATH} / 本轮 ${count} 组 / ${selectedLabel}` +
    ` / 只显示选中候选的坐标系 ${visibleCount} 个${scoreLabel}${timeLabel}`;
}

/**
 * @brief 用缓存的候选重建坐标系缓冲。选中编号变化时调用，无需重读 grasp_result.json。
 */
function rebuildGraspCandidatePoses() {
  const watcher = state.graspCandidatePoseWatcher;
  if (!Array.isArray(watcher.candidates) || watcher.candidates.length === 0) {
    return;
  }
  applyGraspCandidatePoses(watcher.candidates);
}

async function refreshGraspCandidatePoses() {
  const watcher = state.graspCandidatePoseWatcher;
  if (watcher.active) {
    return;
  }
  watcher.active = true;
  try {
    const response = await fetch(GRASP_RESULT_PATH, { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }
    const documentText = await response.text();
    if (documentText === watcher.lastDocumentText) {
      return;
    }
    const document = JSON.parse(documentText);
    const candidates = validateGraspCandidatePoseDocument(document);
    applyGraspCandidatePoses(candidates);
    watcher.lastDocumentText = documentText;
  } catch (error) {
    updateGraspCandidatePoseStatus(error);
  } finally {
    watcher.active = false;
  }
}

function startGraspCandidatePoseWatcher() {
  const watcher = state.graspCandidatePoseWatcher;
  if (watcher.timer !== null) {
    return;
  }
  watcher.timer = window.setInterval(
    refreshGraspCandidatePoses,
    GRASP_RESULT_POLL_INTERVAL_MS,
  );
  document.addEventListener("visibilitychange", () => {
    if (!document.hidden) {
      refreshGraspCandidatePoses();
    }
  });
  window.addEventListener("beforeunload", () => {
    if (watcher.timer !== null) {
      window.clearInterval(watcher.timer);
      watcher.timer = null;
    }
  });
}

function buildPoseAxisVertices(matrix, axisLengthM) {
  const origin = transformPoint(matrix, [0, 0, 0]);
  const x = transformPoint(matrix, [axisLengthM, 0, 0]);
  const y = transformPoint(matrix, [0, axisLengthM, 0]);
  const z = transformPoint(matrix, [0, 0, axisLengthM]);
  return [...origin, ...x, ...origin, ...y, ...origin, ...z];
}

function poseMmDegToMat4(pose) {
  // 6 数位姿 -> 4x4：平移 mm 转 m，旋转按固定轴 XYZ（R = Rz·Ry·Rx）。
  return mat4Multiply(
    mat4Translation(mmToM(pose[0]), mmToM(pose[1]), mmToM(pose[2])),
    mat4FromRpy(degToRad(pose[3]), degToRad(pose[4]), degToRad(pose[5])),
  );
}

function validateStageResult(document) {
  // 阶段1 输出只有 candidates：每组 = 分数 + 夹爪开口 + 四组 6 数位姿。
  const candidates = document?.candidates;
  if (!Array.isArray(candidates) || candidates.length === 0) {
    throw new Error("抓取结果缺少 candidates 数组");
  }
  // candidates 已按分数降序，取第 1 名用于显示坐标系。
  const best = candidates[0];
  // 显示退让后的位姿：那才是机器人真正走到的抓取点。
  const tcpPose = requireFinitePose(
    best?.trajectory_grasp_tcp,
    "candidates[0].trajectory_grasp_tcp",
  );
  const flangePose = requireFinitePose(
    best?.trajectory_grasp_flange,
    "candidates[0].trajectory_grasp_flange",
  );
  // 工件位姿与工件代号已不写进抓取结果，改用当前 input/request.txt 解析出的值。
  const configured = state.configuredWorkpiece;
  if (!configured) {
    throw new Error("尚未读取 input/request.txt，无法确定工件位姿");
  }
  const workpieceMatrix = configured.T_B_O;
  const [roll, pitch, yaw] = mat4ToRpy(workpieceMatrix);
  const workpiecePose = [
    mToMm(workpieceMatrix[12]),
    mToMm(workpieceMatrix[13]),
    mToMm(workpieceMatrix[14]),
    radToDeg(roll),
    radToDeg(pitch),
    radToDeg(yaw),
  ];
  return {
    document,
    workpieceType: configured.code,
    workpieceMatrix,
    tcpMatrix: poseMmDegToMat4(tcpPose),
    flangeMatrix: poseMmDegToMat4(flangePose),
    workpiecePose,
    tcpPose,
    flangePose,
  };
}

function formatStagePose(pose) {
  return `[${pose.map((value) => Number(value).toFixed(3)).join(", ")}] mm/deg`;
}

function applyStageResult(result) {
  state.stageResult = result;
  el.stageWorkpiecePose.textContent = formatStagePose(result.workpiecePose);
  el.stageTcpPose.textContent = formatStagePose(result.tcpPose);
  el.stageFlangePose.textContent = formatStagePose(result.flangePose);
  const vertices = [
    ...buildPoseAxisVertices(result.workpieceMatrix, 0.10),
    ...buildPoseAxisVertices(result.tcpMatrix, 0.08),
    ...buildPoseAxisVertices(result.flangeMatrix, 0.06),
  ];
  updateLineBuffer(state.gl, state.stagePoseBuffer, new Float32Array(vertices));
  state.showFrames = true;
  el.toggleFrames.checked = true;
}

function decodeJsonBuffer(buffer, label) {
  try {
    return JSON.parse(new TextDecoder("utf-8").decode(buffer));
  } catch (error) {
    throw new Error(`${label} JSON 解析失败: ${error.message}`);
  }
}

async function sha256Hex(buffer) {
  if (!window.crypto?.subtle) {
    throw new Error("当前浏览器不支持 Web Crypto SHA-256 校验");
  }
  const digest = await window.crypto.subtle.digest("SHA-256", buffer.slice(0));
  return Array.from(new Uint8Array(digest))
    .map((value) => value.toString(16).padStart(2, "0"))
    .join("");
}

async function validateManifestBundle(manifest, photoBuffer, placeBuffer) {
  if (manifest?.ready !== true) {
    throw new Error("trajectory_manifest.json 的 ready 不是 true");
  }
  if (manifest?.execution_contract !== "aubo_joint_path/v1"
      || manifest?.speed_profile_included !== false
      || manifest?.speed_control_owner !== "behavior_tree") {
    throw new Error("manifest 的正式轨迹或速度归属契约不正确");
  }
  const photo = decodeJsonBuffer(photoBuffer, "photo_to_grasp");
  const place = decodeJsonBuffer(placeBuffer, "grasp_to_place");
  const entries = [
    ["photo_to_grasp", manifest.photo_to_grasp, photo, photoBuffer],
    ["grasp_to_place", manifest.grasp_to_place, place, placeBuffer],
  ];
  for (const [route, entry, document, buffer] of entries) {
    if (!entry || typeof entry.sha256 !== "string") {
      throw new Error(`manifest 缺少 ${route} 条目`);
    }
    validateAuboJointPathContract(document, route);
    if (document.route !== route) {
      throw new Error(`${route} 轨迹的 route 字段错误`);
    }
    if (document.generation_id !== manifest.generation_id
        || document.request_id !== manifest.request_id
        || document.workpiece_type !== manifest.workpiece_type) {
      throw new Error(`${route} 与 manifest 的任务/代次/工件不一致`);
    }
    // manifest 的点数字段名随发布契约演进；两种写法都接受，但必须存在且一致。
    const expectedPoints = Number(entry.point_count ?? entry.points);
    if (!Number.isFinite(expectedPoints) || document.points.length !== expectedPoints) {
      throw new Error(`${route} 点数与 manifest 不一致`);
    }
    if (await sha256Hex(buffer) !== entry.sha256.toLowerCase()) {
      throw new Error(`${route} SHA-256 与 manifest 不一致`);
    }
  }
  const photoEnd = photo.points.at(-1).positions;
  const placeStart = place.points[0].positions;
  const maxJointGap = Math.max(...photoEnd.map(
    (value, index) => Math.abs(Number(value) - Number(placeStart[index])),
  ));
  if (!Number.isFinite(maxJointGap) || maxJointGap > 2e-6) {
    throw new Error(`两条轨迹抓取位衔接不连续，最大关节差 ${maxJointGap} rad`);
  }
  return { photo, place };
}

async function loadLatestPipelineOutput(options = {}) {
  const includeStageResult = options.includeStageResult !== false;
  const includePointCloud = options.includePointCloud !== false;
  const outputRoot = requireAbsolutePath(
    options.outputRoot || el.pipelineOutputPath.value,
    "两阶段 output 文件夹",
  );
  el.pipelineOutputPath.value = outputRoot;
  const artifacts = pipelineArtifactPaths(outputRoot);
  el.pipelineStatus.textContent = `正在并行读取 ${artifacts.root}...`;
  if (artifacts.root === DEFAULT_OUTPUT_ROOT_ABSOLUTE_PATH) {
    await loadConfiguredProjectGeometry();
  } else {
    await loadArchivedProjectGeometry(artifacts);
  }
  const paths = [
    artifacts.manifest,
    artifacts.photoToGrasp,
    artifacts.graspToPlace,
  ];
  if (includeStageResult) {
    paths.push(artifacts.graspResult);
  }
  if (includePointCloud) {
    paths.push(artifacts.pointCloud);
  }
  const buffers = await Promise.all(paths.map(fetchAbsoluteFile));
  const manifest = decodeJsonBuffer(buffers[0], "trajectory_manifest.json");
  const { photo, place } = await validateManifestBundle(manifest, buffers[1], buffers[2]);
  let cursor = 3;
  let stageResult = null;
  let pointCloudResult = null;
  if (includeStageResult) {
    stageResult = validateStageResult(decodeJsonBuffer(buffers[cursor], "grasp_result.json"));
    cursor += 1;
    if (stageResult.workpieceType !== String(manifest.workpiece_type || "").toUpperCase()) {
      throw new Error("阶段1抓取结果与阶段2轨迹的工件代号不一致");
    }
  }
  if (includePointCloud) {
    pointCloudResult = parsePlyPointCloud(buffers[cursor]);
  }
  const photoFile = new File(
    [buffers[1]], "joint_trajectory_photo_to_grasp.json", { type: "application/json" },
  );
  const placeFile = new File(
    [buffers[2]], "joint_trajectory_grasp_to_place.json", { type: "application/json" },
  );
  await loadTrajectoryFiles([photoFile, placeFile], {
    expectedRoutes: ["photo_to_grasp", "grasp_to_place"],
    requirePair: true,
    sourceLabel: `${artifacts.photoToGrasp} + ${artifacts.graspToPlace}`,
  });
  el.photoToGraspTrajectoryPath.value = artifacts.photoToGrasp;
  el.graspToPlaceTrajectoryPath.value = artifacts.graspToPlace;
  if (stageResult) {
    applyStageResult(stageResult);
  }
  if (pointCloudResult) {
    el.pointCloudPath.value = artifacts.pointCloud;
    setPointCloudData(pointCloudResult.points, {
      ...pointCloudResult,
      sourceLabel: artifacts.pointCloud,
    });
  }
  el.pipelineStatus.textContent =
    `已加载 ${manifest.workpiece_type} / request ${String(manifest.request_id).slice(-12)} / ` +
    `generation ${String(manifest.generation_id).slice(0, 12)} / ` +
    `${photo.points.length}+${place.points.length} 点 / SHA-256 通过`;
  return manifest;
}

function trajectoryTimeSeconds(item, index, intervalS = POSITION_PATH_DISPLAY_INTERVAL_S) {
  if (Number.isFinite(Number(item?.t))) {
    return Number(item.t);
  }
  const duration = item?.time_from_start;
  if (Number.isFinite(Number(duration))) {
    return Number(duration);
  }
  if (duration && typeof duration === "object") {
    const sec = Number(duration.sec || 0);
    const nanosec = Number(duration.nanosec ?? duration.nsec ?? 0);
    if (Number.isFinite(sec) && Number.isFinite(nanosec)) {
      return sec + nanosec * 1e-9;
    }
  }
  return index * intervalS;
}

function trajectoryFramesFromData(data, phaseOverride = null) {
  const unit = data?.unit || data?.units?.positions || "deg";
  const source = Array.isArray(data) ? data : data?.points || data?.frames || [];
  // 按文件自带的点间隔（aubo_joint_path/v1 的 dt_s）合成显示时间；缺省才用 0.025。
  const fileIntervalS = Number(data?.dt_s);
  const pointIntervalS = Number.isFinite(fileIntervalS) && fileIntervalS > 0
    ? fileIntervalS : POSITION_PATH_DISPLAY_INTERVAL_S;
  const frames = [];
  for (let index = 0; index < source.length; index += 1) {
    const item = source[index];
    const qRaw = Array.isArray(item)
      ? item
      : item.q || item.joints || item.position || item.positions;
    if (!Array.isArray(qRaw) || qRaw.length < 6) {
      continue;
    }
    const itemUnit = item.unit || unit;
    const q = qRaw.slice(0, 6).map((value) => itemUnit === "rad" ? Number(value) : degToRad(Number(value)));
    if (!q.every(Number.isFinite)) {
      continue;
    }
    frames.push({
      t: trajectoryTimeSeconds(item, index, pointIntervalS),
      q,
      phase: phaseOverride || item.phase || data?.route || null,
    });
  }
  frames.sort((a, b) => a.t - b.t);
  return frames;
}

function setTrajectoryData(data, metadata = {}) {
  const frames = trajectoryFramesFromData(data, metadata.phase || null);
  setTrajectoryFrames(frames, {
    sourceLabel: metadata.sourceLabel || "手动轨迹",
    generationId: metadata.generationId || data?.generation_id || "",
    graspTime: metadata.graspTime,
    transferStartTime: metadata.transferStartTime,
  });
}

function setTrajectoryFrames(frames, metadata = {}) {
  state.trajectory.frames = frames;
  state.trajectory.duration = frames.length ? frames[frames.length - 1].t : 0;
  state.trajectory.time = 0;
  state.trajectory.playing = false;
  state.trajectory.sourceLabel = metadata.sourceLabel || "";
  state.trajectory.generationId = metadata.generationId || "";
  state.trajectory.graspTime = Number.isFinite(metadata.graspTime) ? metadata.graspTime : null;
  state.trajectory.transferStartTime = Number.isFinite(metadata.transferStartTime)
    ? metadata.transferStartTime
    : null;
  if (frames[0]) {
    setJointConfigurationRad(frames[0].q);
  }
  updateTrajectoryPath();
  updateTrajectoryStatus();
}

function trajectoryRoute(data, fileName) {
  const explicit = String(data?.route || "").toLowerCase();
  const name = String(fileName || "").toLowerCase();
  if (explicit === "photo_to_grasp" || name.includes("photo_to_grasp")) {
    return "photo_to_grasp";
  }
  if (explicit === "grasp_to_place" || name.includes("grasp_to_place")) {
    return "grasp_to_place";
  }
  return "";
}

function validateTrajectoryWorkpiece(data, fileName, validateAgainstCurrent = true) {
  const code = String(data?.workpiece_type || "").trim().toUpperCase();
  if (
    validateAgainstCurrent
    && code
    && state.configuredWorkpiece
    && code !== state.configuredWorkpiece.code
  ) {
    throw new Error(
      `${fileName} 属于 ${code}，当前 request/config 是 ${state.configuredWorkpiece.code}，禁止混用`,
    );
  }
  return code;
}

function validateAuboJointPathContract(data, fileName) {
  if (data?.format !== "aubo_joint_path/v1") {
    throw new Error(`${fileName} 不是正式 aubo_joint_path/v1 位置路径`);
  }
  if (data?.units?.positions !== "rad") {
    throw new Error(`${fileName} 的六轴关节位置单位不是 rad`);
  }
  if (data?.speed_profile_included !== false
      || data?.speed_control_owner !== "behavior_tree") {
    throw new Error(`${fileName} 的速度控制归属不是 behavior_tree`);
  }
  const points = Array.isArray(data?.points) ? data.points : [];
  if (points.length < 2) {
    throw new Error(`${fileName} 至少需要 2 个轨迹点`);
  }
  if (points.some((point) => {
    const keys = Object.keys(point || {});
    const positions = point?.positions;
    return keys.length !== 1 || keys[0] !== "positions"
      || !Array.isArray(positions) || positions.length !== 6
      || !positions.map(Number).every(Number.isFinite);
  })) {
    throw new Error(`${fileName} 每点必须且只能包含 6 个有限的 positions(rad)`);
  }
}

async function loadTrajectoryFiles(files, options = {}) {
  if (files.length === 0) {
    throw new Error("请先选择轨迹文件");
  }
  if (options.requirePair && files.length !== 2) {
    throw new Error("必须同时加载两个轨迹文件");
  }
  if (files.length > 2) {
    throw new Error("一次最多加载两个轨迹文件");
  }

  const loaded = [];
  for (const file of files) {
    const item = {
      file,
      data: parseTrajectoryText(await file.text()),
    };
    if (options.requirePair) {
      validateAuboJointPathContract(item.data, item.file.name);
    }
    loaded.push(item);
  }

  if (options.expectedRoutes) {
    if (options.expectedRoutes.length !== loaded.length) {
      throw new Error("轨迹输入槽与文件数量不一致");
    }
    for (let index = 0; index < loaded.length; index += 1) {
      const actual = trajectoryRoute(loaded[index].data, loaded[index].file.name);
      const expected = options.expectedRoutes[index];
      if (actual !== expected) {
        const label = expected === "photo_to_grasp"
          ? "拍照→抓取"
          : "抓取→放置";
        throw new Error(`${label} 输入框选择了错误路线文件: ${loaded[index].file.name}`);
      }
    }
  }

  if (loaded.length === 1) {
    const item = loaded[0];
    validateTrajectoryWorkpiece(
      item.data,
      item.file.name,
      options.validateAgainstCurrentWorkpiece !== false,
    );
    const phase = trajectoryRoute(item.data, item.file.name) || null;
    const frames = trajectoryFramesFromData(item.data, phase);
    if (frames.length === 0) {
      throw new Error(`${item.file.name} 中没有有效的 J1-J6 轨迹点`);
    }
    setTrajectoryFrames(frames, {
      sourceLabel: item.file.name,
      generationId: item.data?.generation_id || "",
    });
    return;
  }

  loaded.sort((a, b) => {
    const rank = { photo_to_grasp: 0, grasp_to_place: 1, "": 2 };
    return rank[trajectoryRoute(a.data, a.file.name)] - rank[trajectoryRoute(b.data, b.file.name)];
  });
  const firstRoute = trajectoryRoute(loaded[0].data, loaded[0].file.name);
  const secondRoute = trajectoryRoute(loaded[1].data, loaded[1].file.name);
  if (firstRoute !== "photo_to_grasp" || secondRoute !== "grasp_to_place") {
    throw new Error("两个文件必须分别是 photo_to_grasp 和 grasp_to_place");
  }

  const generation1 = loaded[0].data?.generation_id || "";
  const generation2 = loaded[1].data?.generation_id || "";
  if (generation1 && generation2 && generation1 !== generation2) {
    throw new Error("两个轨迹文件的 generation_id 不一致，禁止混用");
  }
  const workpiece1 = validateTrajectoryWorkpiece(
    loaded[0].data,
    loaded[0].file.name,
    options.validateAgainstCurrentWorkpiece !== false,
  );
  const workpiece2 = validateTrajectoryWorkpiece(
    loaded[1].data,
    loaded[1].file.name,
    options.validateAgainstCurrentWorkpiece !== false,
  );
  if (workpiece1 && workpiece2 && workpiece1 !== workpiece2) {
    throw new Error("两个轨迹文件的 workpiece_type 不一致，禁止混用");
  }

  const photoFrames = trajectoryFramesFromData(loaded[0].data, "photo_to_grasp");
  const placeFrames = trajectoryFramesFromData(loaded[1].data, "grasp_to_place");
  if (photoFrames.length === 0 || placeFrames.length === 0) {
    throw new Error("轨迹文件中没有有效的 J1-J6 轨迹点");
  }

  const photoStart = photoFrames[0].t;
  const normalizedPhoto = photoFrames.map((frame) => ({
    ...frame,
    t: frame.t - photoStart,
  }));
  const graspTime = normalizedPhoto[normalizedPhoto.length - 1].t;
  const transferStartTime = graspTime + GRASP_DWELL_SECONDS;
  const placeStart = placeFrames[0].t;
  const combined = normalizedPhoto.slice();
  combined.push({
    t: transferStartTime,
    q: normalizedPhoto[normalizedPhoto.length - 1].q.slice(),
    phase: "grasp_dwell",
  });
  combined.push(...placeFrames.map((frame) => ({
    ...frame,
    t: transferStartTime + frame.t - placeStart,
  })));

  setTrajectoryFrames(combined, {
    sourceLabel:
      options.sourceLabel
      || `${loaded[0].file.name} + ${loaded[1].file.name}`,
    generationId: generation1 || generation2,
    graspTime,
    transferStartTime,
  });
}

function updateTrajectoryPath() {
  if (!state.robot || !state.trajectoryBuffer || !state.trajectoryBuffer2) {
    return;
  }
  const photoPoints = [];
  const placePoints = [];
  for (const frame of state.trajectory.frames) {
    const linkWorld = computeLinkWorld(jointMapFromQ(frame.q));
    const point = transformPoint(getFlangeWorld(linkWorld), [0, 0, 0]);
    if (frame.phase === "grasp_dwell") {
      continue;
    }
    if (frame.phase === "grasp_to_place") {
      placePoints.push(point);
    } else {
      photoPoints.push(point);
    }
  }

  updateLineBuffer(state.gl, state.trajectoryBuffer, buildTrajectoryLineVertices(photoPoints));
  updateLineBuffer(state.gl, state.trajectoryBuffer2, buildTrajectoryLineVertices(placePoints));
}

function buildTrajectoryLineVertices(points) {
  const lineVertices = [];
  for (let index = 1; index < points.length; index += 1) {
    lineVertices.push(...points[index - 1], ...points[index]);
  }
  return new Float32Array(lineVertices);
}

function playTrajectory() {
  if (state.trajectory.frames.length === 0) {
    return;
  }
  if (state.trajectory.time >= state.trajectory.duration) {
    state.trajectory.time = 0;
    setJointConfigurationRad(state.trajectory.frames[0].q);
  }
  state.trajectory.playing = true;
  state.trajectory.startMs = performance.now() - (state.trajectory.time / state.trajectory.speed) * 1000;
  updateTrajectoryStatus();
}

function pauseTrajectory() {
  state.trajectory.playing = false;
  updateTrajectoryStatus();
}

function stopTrajectory() {
  state.trajectory.playing = false;
  state.trajectory.time = 0;
  if (state.trajectory.frames[0]) {
    setJointConfigurationRad(state.trajectory.frames[0].q);
  }
  updateTrajectoryStatus();
}

function updateTrajectoryPlayback(nowMs) {
  if (!state.trajectory.playing || state.trajectory.frames.length === 0) {
    return;
  }
  const duration = state.trajectory.duration;
  if (duration <= 0) {
    setJointConfigurationRad(state.trajectory.frames[0].q);
    state.trajectory.playing = false;
    return;
  }
  let time = ((nowMs - state.trajectory.startMs) / 1000) * state.trajectory.speed;
  if (time >= duration) {
    time = duration;
    state.trajectory.playing = false;
  }
  state.trajectory.time = time;
  setJointConfigurationRad(sampleTrajectory(time));
  updateTrajectoryStatus();
}

function sampleTrajectory(time) {
  const frames = state.trajectory.frames;
  if (time <= frames[0].t) {
    return frames[0].q.slice();
  }
  for (let i = 1; i < frames.length; i += 1) {
    const a = frames[i - 1];
    const b = frames[i];
    if (time <= b.t) {
      const denom = Math.max(b.t - a.t, 0.000001);
      const u = clamp((time - a.t) / denom, 0, 1);
      return a.q.map((value, index) => value + (b.q[index] - value) * u);
    }
  }
  return frames[frames.length - 1].q.slice();
}

function updateTrajectoryStatus() {
  if (state.trajectory.frames.length === 0) {
    el.trajectoryStatus.textContent = "等待加载拍照→抓取和抓取→放置两条轨迹";
    return;
  }
  const status = state.trajectory.playing ? "播放中" : "就绪";
  let phase = state.trajectory.frames[0]?.phase || "";
  if (state.trajectory.graspTime !== null && state.trajectory.transferStartTime !== null) {
    if (state.trajectory.time < state.trajectory.graspTime) {
      phase = "photo_to_grasp";
    } else if (state.trajectory.time < state.trajectory.transferStartTime) {
      phase = "grasp_dwell";
    } else {
      phase = "grasp_to_place";
    }
  }
  const phaseLabels = {
    photo_to_grasp: "拍照→抓取",
    grasp_dwell: "抓取位停留",
    grasp_to_place: "抓取→放置",
  };
  const source = state.trajectory.sourceLabel ? ` / ${state.trajectory.sourceLabel}` : "";
  const generation = state.trajectory.generationId
    ? ` / generation ${state.trajectory.generationId.slice(0, 8)}`
    : "";
  const phaseText = phaseLabels[phase] ? ` / ${phaseLabels[phase]}` : "";
  el.trajectoryStatus.textContent =
    `${state.trajectory.frames.length.toLocaleString("zh-CN")} 帧 / ` +
    `${state.trajectory.time.toFixed(2)}s / ${status}${phaseText}${generation}${source}`;
}

function resizeCanvas() {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const rect = state.canvas.getBoundingClientRect();
  const width = Math.max(1, Math.round(rect.width * dpr));
  const height = Math.max(1, Math.round(rect.height * dpr));
  if (state.canvas.width !== width || state.canvas.height !== height) {
    state.canvas.width = width;
    state.canvas.height = height;
  }
}

function cameraPosition() {
  const { target, distance, yaw, pitch } = state.camera;
  const cp = Math.cos(pitch);
  return [
    target[0] + distance * cp * Math.sin(yaw),
    target[1] - distance * cp * Math.cos(yaw),
    target[2] + distance * Math.sin(pitch),
  ];
}

function cameraViewMatrix() {
  return mat4LookAt(cameraPosition(), state.camera.target, [0, 0, 1]);
}

function buildGridVertices(size, step) {
  const vertices = [];
  const count = Math.round(size / step);
  const half = (count * step) / 2;
  for (let i = 0; i <= count; i += 1) {
    const p = -half + i * step;
    vertices.push(-half, p, 0, half, p, 0);
    vertices.push(p, -half, 0, p, half, 0);
  }
  return new Float32Array(vertices);
}

function buildWorldAxisVertices() {
  return new Float32Array([
    0, 0, 0, 0.42, 0, 0,
    0, 0, 0, 0, 0.42, 0,
    0, 0, 0, 0, 0, 0.42,
  ]);
}

function createProgram(gl, vertexSource, fragmentSource) {
  const vertex = compileShader(gl, gl.VERTEX_SHADER, vertexSource);
  const fragment = compileShader(gl, gl.FRAGMENT_SHADER, fragmentSource);
  const program = gl.createProgram();
  gl.attachShader(program, vertex);
  gl.attachShader(program, fragment);
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
    throw new Error(gl.getProgramInfoLog(program));
  }
  return {
    program,
    attributes: {
      position: gl.getAttribLocation(program, "aPosition"),
      normal: gl.getAttribLocation(program, "aNormal"),
      color: gl.getAttribLocation(program, "aColor"),
    },
    uniforms: {
      model: gl.getUniformLocation(program, "uModel"),
      view: gl.getUniformLocation(program, "uView"),
      projection: gl.getUniformLocation(program, "uProjection"),
      color: gl.getUniformLocation(program, "uColor"),
      lightDir: gl.getUniformLocation(program, "uLightDir"),
      cameraPos: gl.getUniformLocation(program, "uCameraPos"),
      opacity: gl.getUniformLocation(program, "uOpacity"),
      pointSize: gl.getUniformLocation(program, "uPointSize"),
      useVertexColor: gl.getUniformLocation(program, "uUseVertexColor"),
    },
  };
}

function compileShader(gl, type, source) {
  const shader = gl.createShader(type);
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    throw new Error(gl.getShaderInfoLog(shader));
  }
  return shader;
}

function mat4Identity() {
  return new Float32Array([
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
  ]);
}

function mat4FromRows(r0, r1, r2, r3) {
  return new Float32Array([
    r0[0], r1[0], r2[0], r3[0],
    r0[1], r1[1], r2[1], r3[1],
    r0[2], r1[2], r2[2], r3[2],
    r0[3], r1[3], r2[3], r3[3],
  ]);
}

function mat4Multiply(a, b) {
  const out = new Float32Array(16);
  for (let col = 0; col < 4; col += 1) {
    for (let row = 0; row < 4; row += 1) {
      out[col * 4 + row] =
        a[0 * 4 + row] * b[col * 4 + 0] +
        a[1 * 4 + row] * b[col * 4 + 1] +
        a[2 * 4 + row] * b[col * 4 + 2] +
        a[3 * 4 + row] * b[col * 4 + 3];
    }
  }
  return out;
}

function mat4Translation(x, y, z) {
  return mat4FromRows(
    [1, 0, 0, x],
    [0, 1, 0, y],
    [0, 0, 1, z],
    [0, 0, 0, 1],
  );
}

function mat4Scale(x, y, z) {
  return mat4FromRows(
    [x, 0, 0, 0],
    [0, y, 0, 0],
    [0, 0, z, 0],
    [0, 0, 0, 1],
  );
}

function mat4FromRpy(roll, pitch, yaw) {
  const rx = mat4FromRows(
    [1, 0, 0, 0],
    [0, Math.cos(roll), -Math.sin(roll), 0],
    [0, Math.sin(roll), Math.cos(roll), 0],
    [0, 0, 0, 1],
  );
  const ry = mat4FromRows(
    [Math.cos(pitch), 0, Math.sin(pitch), 0],
    [0, 1, 0, 0],
    [-Math.sin(pitch), 0, Math.cos(pitch), 0],
    [0, 0, 0, 1],
  );
  const rz = mat4FromRows(
    [Math.cos(yaw), -Math.sin(yaw), 0, 0],
    [Math.sin(yaw), Math.cos(yaw), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  );
  return mat4Multiply(rz, mat4Multiply(ry, rx));
}

function mat4ToRpy(m) {
  const pitch = Math.asin(clamp(-m[2], -1, 1));
  const cosPitch = Math.cos(pitch);
  let roll = 0;
  let yaw = 0;
  if (Math.abs(cosPitch) > 0.000001) {
    roll = Math.atan2(m[6], m[10]);
    yaw = Math.atan2(m[1], m[0]);
  } else {
    yaw = Math.atan2(-m[4], m[5]);
  }
  return [roll, pitch, yaw];
}

function mat4FromAxisAngle(axis, angle) {
  const [x, y, z] = normalize3(axis);
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const t = 1 - c;
  return mat4FromRows(
    [t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0],
    [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0],
    [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0],
    [0, 0, 0, 1],
  );
}

function mat4Perspective(fovy, aspect, near, far) {
  const f = 1 / Math.tan(fovy / 2);
  const nf = 1 / (near - far);
  return new Float32Array([
    f / aspect, 0, 0, 0,
    0, f, 0, 0,
    0, 0, (far + near) * nf, -1,
    0, 0, 2 * far * near * nf, 0,
  ]);
}

function mat4LookAt(eye, center, up) {
  const z = normalize3(sub3(eye, center));
  const x = normalize3(cross3(up, z));
  const y = cross3(z, x);
  return new Float32Array([
    x[0], y[0], z[0], 0,
    x[1], y[1], z[1], 0,
    x[2], y[2], z[2], 0,
    -dot3(x, eye), -dot3(y, eye), -dot3(z, eye), 1,
  ]);
}

function transformPoint(m, p) {
  const x = p[0];
  const y = p[1];
  const z = p[2];
  return [
    m[0] * x + m[4] * y + m[8] * z + m[12],
    m[1] * x + m[5] * y + m[9] * z + m[13],
    m[2] * x + m[6] * y + m[10] * z + m[14],
  ];
}

function faceNormal(a, b, c) {
  return normalize3(cross3(sub3(b, a), sub3(c, a)));
}

function expandBounds(bounds, point) {
  for (let i = 0; i < 3; i += 1) {
    bounds.min[i] = Math.min(bounds.min[i], point[i]);
    bounds.max[i] = Math.max(bounds.max[i], point[i]);
  }
}

function sub3(a, b) {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function add3(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function mul3(a, scalar) {
  return [a[0] * scalar, a[1] * scalar, a[2] * scalar];
}

function cross3(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function dot3(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function length3(v) {
  return Math.hypot(v[0], v[1], v[2]);
}

function normalize3(v) {
  const length = length3(v);
  if (length < 0.000001) {
    return [0, 0, 1];
  }
  return [v[0] / length, v[1] / length, v[2] / length];
}

function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

function wrapAngle(radians) {
  let value = radians;
  while (value > Math.PI) {
    value -= Math.PI * 2;
  }
  while (value < -Math.PI) {
    value += Math.PI * 2;
  }
  return value;
}

function degToRad(degrees) {
  return (degrees * Math.PI) / 180;
}

function radToDeg(radians) {
  return (radians * 180) / Math.PI;
}

function mmToM(mm) {
  return mm / 1000;
}

function mToMm(meters) {
  return meters * 1000;
}

const MESH_VERTEX_SHADER = `
attribute vec3 aPosition;
attribute vec3 aNormal;
uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
varying vec3 vNormal;
varying vec3 vWorld;

void main() {
  vec4 world = uModel * vec4(aPosition, 1.0);
  vWorld = world.xyz;
  vNormal = normalize(mat3(uModel) * aNormal);
  gl_Position = uProjection * uView * world;
}
`;

const MESH_FRAGMENT_SHADER = `
precision mediump float;
uniform vec3 uColor;
uniform vec3 uLightDir;
uniform vec3 uCameraPos;
uniform float uOpacity;
varying vec3 vNormal;
varying vec3 vWorld;

void main() {
  vec3 normal = normalize(vNormal);
  vec3 light = normalize(uLightDir);
  float diffuse = max(dot(normal, light), 0.0);
  vec3 viewDir = normalize(uCameraPos - vWorld);
  vec3 halfDir = normalize(light + viewDir);
  float specular = pow(max(dot(normal, halfDir), 0.0), 38.0) * 0.22;
  vec3 color = uColor * (0.34 + diffuse * 0.72) + vec3(specular);
  gl_FragColor = vec4(color, uOpacity);
}
`;

const POINT_VERTEX_SHADER = `
attribute vec3 aPosition;
attribute vec3 aColor;
uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
uniform float uPointSize;
varying vec3 vColor;

void main() {
  gl_Position = uProjection * uView * uModel * vec4(aPosition, 1.0);
  gl_PointSize = uPointSize;
  vColor = aColor;
}
`;

const POINT_FRAGMENT_SHADER = `
precision mediump float;
uniform vec3 uColor;
uniform bool uUseVertexColor;
varying vec3 vColor;

void main() {
  vec2 p = gl_PointCoord * 2.0 - 1.0;
  if (dot(p, p) > 1.0) {
    discard;
  }
  gl_FragColor = vec4(uUseVertexColor ? vColor : uColor, 1.0);
}
`;

const LINE_VERTEX_SHADER = `
attribute vec3 aPosition;
uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;

void main() {
  gl_Position = uProjection * uView * uModel * vec4(aPosition, 1.0);
}
`;

const LINE_FRAGMENT_SHADER = `
precision mediump float;
uniform vec3 uColor;
uniform float uOpacity;

void main() {
  gl_FragColor = vec4(uColor, uOpacity);
}
`;

// ============================================================================
// 第三流程 flow=visualization：读取结果、展示场景、回放轨迹
//
// 四个 stage（沿用「中文数字标识 stage、阿拉伯数字标识 stage 内部步骤」的日志规则）：
//   一 initialize_visualization  加载固定资源，建立显示模型
//   二 load_result_directory     解析并确认数据父目录，管理数据源切换
//   三 update_visualization      检查发布状态，读取、校验并整体更新动态数据
//   四 play_trajectory           两段轨迹的顺序回放和播放控制
//
// 它是独立启用的结果消费流程：通过第二流程正式发布的文件衔接，不参与规划或碰撞验收，
// 也不向真实机器人发送运动指令。关闭可视化不影响前两流程的触发、输出和归档。
// ============================================================================

const VIZ_PROJECT_ROOT_ENDPOINT = "/__visualizer_project_root__";
const VIZ_DEFAULT_RELATIVE_DIRECTORY = "output/trajectory_planning";
const VIZ_COLLISION_MODEL_RELATIVE_PATH = "output/visualization/collision_model.json";
const VIZ_POLL_INTERVAL_MS = 500;
const VIZ_ORIGINAL_GRIPPER_COLOR = [0.95, 0.55, 0.16];
const VIZ_RETREAT_GRIPPER_COLOR = [0.16, 0.72, 0.78];
const VIZ_GRIPPER_OPACITY = 0.35;
// 占据体素：偏红，与点云的灰色明确区分——一眼能看出"原始点"与"离散后的格子"的位移。
const VIZ_OCCUPIED_VOXEL_COLOR = [0.95, 0.22, 0.18];   // 占据格：红
const VIZ_OCCUPIED_VOXEL_POINT_SIZE = 2;
const VIZ_VOXEL_REGION_COLOR = [0.45, 0.62, 0.85];     // 测距区域：淡蓝半透明
const VIZ_VOXEL_REGION_OPACITY = 0.07;
const VIZ_VOXEL_FACE_GRID_COLOR = [0.42, 0.55, 0.72];  // 六面 10mm 格线
const VIZ_EXCLUSION_COLOR = [0.16, 0.72, 0.78];
const VIZ_EXCLUSION_SEGMENTS = 48;

const viz = {
  sessionId: `viewer_${Date.now().toString(36)}`,
  projectRoot: "",
  pathMode: "relative",
  // 「解析后的路径」与「当前生效目录」分开：待加载目录不能被误认为已经加载成功。
  resolvedDirectory: "",
  activeDirectory: "",
  autoFollow: true,
  pollTimer: null,
  // 同一生效目录、同一代次不因轮询反复播放。
  appliedGenerationId: "",
  // 轨迹与点云分两次到达同一代次：前者进入画面即可开始回放，后者后台写完后补上。
  appliedPointCloudGenerationId: "",
  loading: false,
  // 目录切换后旧目录的迟到响应不得覆盖新数据源。
  requestEpoch: 0,
  fixedModel: null,
  fixedModelLoaded: false,
  exclusionBuffer: null,
  platformBoxes: [],
  platformRepresentation: "boxes", // 改动十一起为 "voxel"（平台并入体素距离场）
  // 改动十 10.2：本轮体素网格范围。网格每轮都变，来源是 manifest 的 collision_world 节，
  // 不是固定模型快照。**默认打开** —— 由文档 2.6，球心一旦离开网格即静默漏检，
  // 这是判断"机械臂有没有伸出网格"的唯一直观手段。
  gridBuffer: null,
  gridMeta: null,
  showWorldGrid: true,
  // 改动十 10.3：**体素区域**——ESDF 测距覆盖的整个网格。
  // 半透明盒给出范围，六面 10mm 格线给出真实格子大小，占据格标红。
  // 内部 1193 万个格子无法逐个绘制（是遮挡问题，不是性能问题），
  // 故只画边界面上的格线；被占据的格子则全部按格心画出来。
  voxelRegionModel: null,        // 半透明盒的 model 矩阵
  voxelFaceGridBuffer: null,     // 六个面上的 10mm 格线
  occupiedVoxelBuffer: null,     // 占据格心（点云 + 平台），红色
  occupiedVoxelCount: 0,
  occupiedFromCloud: 0,
  occupiedFromPlatform: 0,
  showVoxelRegion: false,        // 默认关闭，想看时再打开
  // 体素区域的构建要遍历 68 万个格子（实测约 385ms），而这个开关平时是关的，
  // 每轮都预先算一遍纯属浪费。故只记下素材，等开关第一次打开时再建。
  voxelSource: null,             // { collisionWorld, pointsMm, fixedModel }
  voxelBuilt: false,
  preview: null,
  // 抓取夹爪预览**只画最终选中的那个候选**。
  // 原先可切"全部候选/指定候选"，但标签库的标签绕工件一圈，工件紧贴料框时
  // 背面那几个标签变换到基座系后会扎进框壁（本轮实测 10 个候选中 5~9 号
  // 的 IK 全部 0/4 不收敛）。这些候选规划阶段本就会被拒，画出来只会让人
  // 误以为抓取点跑到了料框上，故不再提供显示它们的入口。
  showOriginalGripper: true,
  showRetreatGripper: true,
  showExclusion: true,
  toolExtentsM: null,
};

const vizEl = {};

/** @brief 第三流程日志：统一带 flow=visualization 与可视化会话标识。 */
function vizLog(stage, step, message) {
  const prefix = `[flow=visualization]${stage ? `[${stage}]` : ""}${step ? `[${step}]` : ""}`;
  console.log(`${prefix} viewer_session_id=${viz.sessionId} ${message}`);
}

/** @brief 状态行只反映第三流程自身的状态，不改写规划结果。 */
function vizSetStatus(text) {
  if (vizEl.status) {
    vizEl.status.textContent = text;
  }
}

// ---------------------------------------------------------------------------
// 一 initialize_visualization：一次性加载固定资源
// ---------------------------------------------------------------------------

/**
 * @brief 取得可视化服务确定的本项目根目录。
 *
 * 相对路径以它为基准，而不是网页所在的 scripts/simulation/ 或浏览器电脑目录；
 * 项目移动或重新部署后仍能正确定位。
 */
async function vizFetchProjectRoot() {
  try {
    const response = await fetch(VIZ_PROJECT_ROOT_ENDPOINT, { cache: "no-store" });
    if (response.ok) {
      const document = await response.json();
      if (document?.project_root) {
        return String(document.project_root);
      }
    }
  } catch (_error) {
    // 旧版服务没有该端点；退回按页面地址推断，仍不写死任何绝对路径。
  }
  return "";
}

/**
 * @brief 加载规划服务导出的固定模型快照（与规划同源）。
 *
 * 快照包含实际碰撞球、固定平台、圆柱豁免区，以及模型标识、挂载坐标系和单位。
 * 只在初始化时读一次；切换结果目录、收到新结果或开始回放都不重新加载。
 */
async function vizLoadFixedModel() {
  try {
    const response = await fetch(`/${VIZ_COLLISION_MODEL_RELATIVE_PATH}`, { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }
    const model = await response.json();
    viz.fixedModel = model;
    viz.fixedModelLoaded = true;
    // 改动十一起平台不再分解为长方体（representation="voxel"），这里拿到空数组，
    // vizDrawPlatformBoxes 会退回"单个平台盒 + 圆柱孔轮廓"的画法。
    viz.platformBoxes = Array.isArray(model?.base_platform?.boxes) ? model.base_platform.boxes : [];
    viz.platformRepresentation = String(model?.base_platform?.representation || "boxes");
    viz.toolExtentsM = vizToolExtentsFromModel(model);
    vizBuildExclusionBuffer(model?.installation_exclusion);

    const bodyCount = Number(model?.body_sphere_count) || 0;
    const endCount = Number(model?.end_effector_sphere_count) || 0;
    const totalCount = Number(model?.total_sphere_count) || bodyCount + endCount;
    const exclusion = model?.installation_exclusion || {};
    if (vizEl.modelStatus) {
      vizEl.modelStatus.textContent =
        `固定模型=${model?.model_id || "未知"} 本体碰撞球=${bodyCount} 末端碰撞球=${endCount} ` +
        `总数=${bodyCount} + ${endCount} = ${totalCount}；` +
        `平台=${viz.platformRepresentation === "voxel" ? "并入体素距离场" : `长方体${viz.platformBoxes.length}个`} ` +
        `圆柱豁免=${exclusion.enabled ? `直径${exclusion.diameter_mm}mm/深度${exclusion.depth_mm}mm` : "未启用"}；` +
        `碰撞球由规划服务导出，本页不另生成一套`;
    }
    vizLog("一 initialize_visualization", "2/3",
      `本体球=${bodyCount} 末端球=${endCount} 总球数=${totalCount} ` +
      `固定平台=加载完成(${viz.platformRepresentation}) 圆柱豁免=${exclusion.enabled ? "启用" : "未启用"} ` +
      `末端外凸=${Number(model?.end_effector_sphere_bulge_mm) || 0}mm ` +
      `激活距离=${Number(model?.collision_activation_mm) || 0}mm`);
    vizLog("一 initialize_visualization", "3/3",
      `夹爪预览模板=加载完成 配置退让量=${model?.camera_clearance_along_tcp_z_mm}mm 默认路径模式=相对 自动跟随=开启`);
    return true;
  } catch (error) {
    viz.fixedModelLoaded = false;
    if (vizEl.modelStatus) {
      // 固定资源尚未准备好时明确提示，不能把缺少末端球的模型当成完整模型。
      vizEl.modelStatus.textContent = `等待固定模型初始化：${error.message}`;
    }
    vizLog("一 initialize_visualization", "2/3", `固定模型未就绪: ${error.message}`);
    return false;
  }
}

/** @brief 末端盒六向延伸（米），夹爪预览模板与规划使用同一份几何。 */
function vizToolExtentsFromModel(model) {
  const end = model?.end_effector;
  if (!end) {
    return null;
  }
  const negative = end.negative_extent_xyz_mm || [0, 0, 0];
  const positive = end.positive_extent_xyz_mm || [0, 0, 0];
  return {
    negativeX: mmToM(Number(negative[0]) || 0),
    negativeY: mmToM(Number(negative[1]) || 0),
    negativeZ: mmToM(Number(negative[2]) || 0),
    positiveX: mmToM(Number(positive[0]) || 0),
    positiveY: mmToM(Number(positive[1]) || 0),
    positiveZ: mmToM(Number(positive[2]) || 0),
  };
}

/**
 * @brief 生成圆柱豁免区的轮廓线（顶圈、底圈与竖线）。
 *
 * 轮廓只是辅助标注：关掉它平台仍然有孔，不会变回完整实心盒。
 */
/**
 * 与 numpy 的 `np.rint` 同规则取整：**四舍六入五成双**。
 *
 * JS 的 `Math.round` 对 .5 一律向上进位（Math.round(2.5)=3、Math.round(-1.5)=-1），
 * numpy 则取到偶数（np.rint(2.5)=2、np.rint(-1.5)=-2）。服务端体素吸附用的是
 * np.rint，前端若用 Math.round，落在格边界正中的点会被分到相邻格 —— 实测 172800
 * 点里差十几格。这是复现服务端结果的必要条件。
 */
function rintHalfToEven(value) {
  const rounded = Math.round(value);
  if (Math.abs(value % 1) !== 0.5) {
    return rounded;
  }
  return rounded % 2 === 0 ? rounded : rounded - 1;
}

/**
 * @brief 由本轮点云复现服务端的占据体素，构建格心点缓冲（改动十 10.3）。
 *
 * 不需要新增落盘文件：服务端的吸附公式是
 *
 *     grid_center = (grid_min + grid_max) / 2
 *     origin      = grid_center − (shape − 1) / 2 × voxel      // 0 号格的中心
 *     index       = rint((p − origin) / voxel)
 *
 * manifest 的 `collision_world` 已带齐 grid_min_mm / grid_max_mm / grid_shape /
 * voxel_size_mm，前端用同一套公式即可得到与服务端**逐格一致**的结果。
 *
 * 只画点云那部分（本轮 41387 格）。平台那部分（640000 格）是一整块实心长方体，
 * 逐格画既看不出东西又拖垮渲染，它已经以实体盒的形式显示。
 *
 * @param collisionWorld manifest 的 collision_world 节
 * @param pointsMm 本轮点云（毫米，parsePlyPointCloud 的输出）
 * @return 占据格子数；无法复现时返回 0
 */
function vizBuildOccupiedVoxelBuffer(collisionWorld, pointsMm, fixedModel) {
  viz.occupiedVoxelBuffer = null;
  viz.occupiedVoxelCount = 0;
  viz.occupiedFromCloud = 0;
  viz.occupiedFromPlatform = 0;
  if (!collisionWorld || !state.gl) {
    return 0;
  }
  const voxel = Number(collisionWorld.voxel_size_mm);
  const shape = collisionWorld.grid_shape;
  const lo = collisionWorld.grid_min_mm;
  const hi = collisionWorld.grid_max_mm;
  if (!(voxel > 0) || !Array.isArray(shape) || shape.length !== 3
      || !Array.isArray(lo) || !Array.isArray(hi)) {
    return 0;
  }
  // 优先用服务端给的满精度原点；缺失（旧归档）时退回由 grid_min/max 反推。
  // 反推值受 grid_min_mm 取整到 0.1mm 的影响，边界点可能翻格，仅作降级用。
  const published = collisionWorld.voxel_origin_mm;
  const origin = Array.isArray(published) && published.length === 3
    ? published.map(Number)
    : [0, 1, 2].map(
      (axis) => 0.5 * (Number(lo[axis]) + Number(hi[axis])) - (Number(shape[axis]) - 1) * 0.5 * voxel,
    );
  const nx = Number(shape[0]);
  const ny = Number(shape[1]);
  const nz = Number(shape[2]);

  // 同一格里的多个点只画一次。键为线性下标，避免字符串拼接开销。
  const seen = new Set();
  const centers = [];
  const addVoxel = (ix, iy, iz) => {
    if (ix < 0 || iy < 0 || iz < 0 || ix >= nx || iy >= ny || iz >= nz) {
      return false;   // 与服务端一致：落在网格外的直接丢弃
    }
    const key = (ix * ny + iy) * nz + iz;
    if (seen.has(key)) {
      return false;
    }
    seen.add(key);
    centers.push(
      mmToM(origin[0] + ix * voxel),
      mmToM(origin[1] + iy * voxel),
      mmToM(origin[2] + iz * voxel),
    );
    return true;
  };

  // ---- 点云占据格：复刻服务端 index = rint((p - origin) / voxel) ----
  if (Array.isArray(pointsMm)) {
    for (const point of pointsMm) {
      addVoxel(
        rintHalfToEven((point[0] - origin[0]) / voxel),
        rintHalfToEven((point[1] - origin[1]) / voxel),
        rintHalfToEven((point[2] - origin[2]) / voxel),
      );
    }
  }
  viz.occupiedFromCloud = seen.size;

  // ---- 平台占据格：复刻服务端的整数区间填充 ----
  //   start = clip(ceil((lo_box  - origin) / voxel), 0, shape)
  //   stop  = clip(floor((hi_box - origin) / voxel) + 1, 0, shape)
  // 再按 installation_exclusion 挖掉圆柱孔（当前配置 enabled=false，即实心平台）。
  const platform = fixedModel?.base_platform;
  if (platform) {
    const negative = platform.negative_extent_xyz_mm || [0, 0, 0];
    const positive = platform.positive_extent_xyz_mm || [0, 0, 0];
    const loBox = [0, 1, 2].map((axis) => -Number(negative[axis]));
    const hiBox = [0, 1, 2].map((axis) => Number(positive[axis]));
    const start = [0, 1, 2].map(
      (axis) => clamp(Math.ceil((loBox[axis] - origin[axis]) / voxel), 0, [nx, ny, nz][axis]),
    );
    const stop = [0, 1, 2].map(
      (axis) => clamp(Math.floor((hiBox[axis] - origin[axis]) / voxel) + 1, 0, [nx, ny, nz][axis]),
    );
    const exclusion = fixedModel?.installation_exclusion;
    const holeOn = Boolean(exclusion?.enabled);
    const holeCenter = exclusion?.center_xy_mm || [0, 0];
    const holeRadius = (Number(exclusion?.diameter_mm) || 0) * 0.5;
    const holeZ = exclusion?.z_range_mm || [0, 0];
    for (let ix = start[0]; ix < stop[0]; ix += 1) {
      const x = origin[0] + ix * voxel;
      for (let iy = start[1]; iy < stop[1]; iy += 1) {
        const y = origin[1] + iy * voxel;
        const inCircle = holeOn
          && ((x - Number(holeCenter[0])) ** 2 + (y - Number(holeCenter[1])) ** 2)
             <= holeRadius * holeRadius;
        for (let iz = start[2]; iz < stop[2]; iz += 1) {
          if (inCircle) {
            const z = origin[2] + iz * voxel;
            if (z >= Number(holeZ[0]) && z <= Number(holeZ[1])) {
              continue;   // 孔内不算障碍
            }
          }
          addVoxel(ix, iy, iz);
        }
      }
    }
  }
  viz.occupiedFromPlatform = seen.size - viz.occupiedFromCloud;

  viz.occupiedVoxelBuffer = createLineBuffer(state.gl, new Float32Array(centers));
  viz.occupiedVoxelCount = seen.size;
  return seen.size;
}

/**
 * @brief 按需构建体素区域（半透明盒 + 六面格线 + 占据格）。
 *
 * 构建一次约 385ms（要遍历 68 万个占据格），而开关默认关闭且平时不开，
 * 所以推迟到真正要显示时才做；同一轮内重复调用是空操作。
 * @return 可以显示返回 true
 */
function vizEnsureVoxelRegion() {
  if (viz.voxelBuilt) {
    return Boolean(viz.occupiedVoxelBuffer || viz.voxelRegionModel);
  }
  const source = viz.voxelSource;
  if (!source) {
    return false;
  }
  const started = performance.now();
  vizBuildVoxelRegion(source.collisionWorld);
  vizBuildOccupiedVoxelBuffer(source.collisionWorld, source.pointsMm, viz.fixedModel);
  viz.voxelBuilt = true;

  // 前端复现的格数应与服务端逐格一致；不一致说明吸附公式或网格参数对不上。
  // 例外：点云超过 DEFAULT_MAX_PLY_POINTS 时会按 stride 抽样，格数必然偏少。
  const cw = source.collisionWorld;
  const expectCloud = Number(cw.occupied_from_cloud);
  const expectPlatform = Number(cw.occupied_from_platform);
  const cloudVerdict = source.stride > 1
    ? `（按 stride=${source.stride} 抽样显示，偏少属正常）`
    : (viz.occupiedFromCloud === expectCloud ? "一致" : "⚠ 与服务端不符");
  const platformVerdict =
    viz.occupiedFromPlatform === expectPlatform ? "一致" : "⚠ 与服务端不符";
  vizLog("三 update_visualization", "3/3",
    `体素区域已构建 耗时=${Math.round(performance.now() - started)}ms；` +
    `占据格 点云 ${viz.occupiedFromCloud}/${expectCloud} ${cloudVerdict}、` +
    `平台 ${viz.occupiedFromPlatform}/${expectPlatform} ${platformVerdict}`);
  return true;
}

/**
 * @brief 构建"测距区域"的半透明盒与六个面上的 10mm 格线（改动十 10.3）。
 *
 * 内部 1193 万个格子不逐个绘制：377 层线从任何角度看都叠成一团雾，画了等于没画。
 * 范围由半透明盒给出，真实格子大小由六个面上的格线给出。
 */
function vizBuildVoxelRegion(collisionWorld) {
  viz.voxelRegionModel = null;
  viz.voxelFaceGridBuffer = null;
  if (!collisionWorld || !state.gl || !state.cartMesh) {
    return false;
  }
  const voxel = Number(collisionWorld.voxel_size_mm);
  const shape = collisionWorld.grid_shape;
  if (!(voxel > 0) || !Array.isArray(shape) || shape.length !== 3) {
    return false;
  }
  const published = collisionWorld.voxel_origin_mm;
  const lo = collisionWorld.grid_min_mm;
  const hi = collisionWorld.grid_max_mm;
  const origin = Array.isArray(published) && published.length === 3
    ? published.map(Number)
    : [0, 1, 2].map(
      (axis) => 0.5 * (Number(lo[axis]) + Number(hi[axis])) - (Number(shape[axis]) - 1) * 0.5 * voxel,
    );
  // 网格的真实外沿：0 号格心往外半格，到 n-1 号格心再往外半格。
  const boxLo = [0, 1, 2].map((axis) => origin[axis] - 0.5 * voxel);
  const boxHi = [0, 1, 2].map((axis) => origin[axis] + (Number(shape[axis]) - 0.5) * voxel);

  const sizeM = [0, 1, 2].map((axis) => mmToM(boxHi[axis] - boxLo[axis]));
  const centerM = [0, 1, 2].map((axis) => mmToM(0.5 * (boxLo[axis] + boxHi[axis])));
  viz.voxelRegionModel = mat4Multiply(
    mat4Translation(centerM[0], centerM[1], centerM[2]),
    mat4Scale(sizeM[0], sizeM[1], sizeM[2]),
  );

  // 六个面上的 10mm 格线。只画边界面，内部不画。
  const data = [];
  const seg = (ax, ay, az, bx, by, bz) => data.push(
    mmToM(ax), mmToM(ay), mmToM(az), mmToM(bx), mmToM(by), mmToM(bz),
  );
  const line = (axis, u, v) => {
    // axis = 线的走向；u/v 为另两轴上的固定坐标
    const p0 = [0, 0, 0];
    const p1 = [0, 0, 0];
    const other = [0, 1, 2].filter((a) => a !== axis);
    p0[axis] = boxLo[axis];
    p1[axis] = boxHi[axis];
    p0[other[0]] = u; p1[other[0]] = u;
    p0[other[1]] = v; p1[other[1]] = v;
    seg(p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]);
  };
  for (let axis = 0; axis < 3; axis += 1) {
    const other = [0, 1, 2].filter((a) => a !== axis);
    const [uAxis, vAxis] = other;
    const uCount = Number(shape[uAxis]) + 1;
    const vCount = Number(shape[vAxis]) + 1;
    // 该走向的线只画在垂直于另两轴的四个面上，故 u 或 v 取两端值。
    for (let i = 0; i < uCount; i += 1) {
      const u = boxLo[uAxis] + i * voxel;
      line(axis, u, boxLo[vAxis]);
      line(axis, u, boxHi[vAxis]);
    }
    for (let j = 0; j < vCount; j += 1) {
      const v = boxLo[vAxis] + j * voxel;
      line(axis, boxLo[uAxis], v);
      line(axis, boxHi[uAxis], v);
    }
  }
  viz.voxelFaceGridBuffer = createLineBuffer(state.gl, new Float32Array(data));
  return true;
}

/**
 * @brief 由 manifest 的 collision_world 节构建本轮体素网格范围线框（改动十 10.2）。
 *
 * 旧归档缺该节时清空线框并返回 false，由调用方提示降级，
 * **不沿用上一轮的网格冒充本轮**。
 */
function vizBuildWorldGridBuffer(collisionWorld) {
  viz.gridBuffer = null;
  viz.gridMeta = null;
  if (!collisionWorld || !state.gl) {
    return false;
  }
  const lo = collisionWorld.grid_min_mm;
  const hi = collisionWorld.grid_max_mm;
  if (!Array.isArray(lo) || !Array.isArray(hi) || lo.length !== 3 || hi.length !== 3) {
    return false;
  }
  const a = lo.map((value) => mmToM(Number(value) || 0));
  const b = hi.map((value) => mmToM(Number(value) || 0));
  // 八个角点，十二条棱。
  const corner = (index) => [
    (index & 1) ? b[0] : a[0],
    (index & 2) ? b[1] : a[1],
    (index & 4) ? b[2] : a[2],
  ];
  const edges = [
    [0, 1], [2, 3], [4, 5], [6, 7],
    [0, 2], [1, 3], [4, 6], [5, 7],
    [0, 4], [1, 5], [2, 6], [3, 7],
  ];
  const data = [];
  for (const [from, to] of edges) {
    data.push(...corner(from), ...corner(to));
  }
  viz.gridBuffer = createLineBuffer(state.gl, data);
  viz.gridMeta = collisionWorld;
  return true;
}

function vizBuildExclusionBuffer(exclusion) {
  viz.exclusionBuffer = null;
  if (!exclusion?.enabled || !state.gl) {
    return;
  }
  const centerX = mmToM(Number(exclusion.center_xy_mm?.[0]) || 0);
  const centerY = mmToM(Number(exclusion.center_xy_mm?.[1]) || 0);
  const radius = mmToM((Number(exclusion.diameter_mm) || 0) * 0.5);
  const range = exclusion.z_range_mm || [0, 0];
  const bottom = mmToM(Number(range[0]) || 0);
  const top = mmToM(Number(range[1]) || 0);
  if (radius <= 0) {
    return;
  }
  const data = [];
  const pushSegment = (ax, ay, az, bx, by, bz) => data.push(ax, ay, az, bx, by, bz);
  for (let index = 0; index < VIZ_EXCLUSION_SEGMENTS; index += 1) {
    const a = (2 * Math.PI * index) / VIZ_EXCLUSION_SEGMENTS;
    const b = (2 * Math.PI * (index + 1)) / VIZ_EXCLUSION_SEGMENTS;
    const ax = centerX + radius * Math.cos(a);
    const ay = centerY + radius * Math.sin(a);
    const bx = centerX + radius * Math.cos(b);
    const by = centerY + radius * Math.sin(b);
    pushSegment(ax, ay, top, bx, by, top);
    pushSegment(ax, ay, bottom, bx, by, bottom);
    if (index % 6 === 0) {
      pushSegment(ax, ay, bottom, ax, ay, top);
    }
  }
  viz.exclusionBuffer = createLineBuffer(state.gl, new Float32Array(data));
}

// ---------------------------------------------------------------------------
// 二 load_result_directory：解析并确认数据父目录
// ---------------------------------------------------------------------------

/** @brief 规范化目录串，便于判断「同一生效目录」。 */
function vizNormalizeDirectory(path) {
  return String(path || "").replace(/\/+$/, "");
}

/**
 * @brief 按当前路径模式解析输入目录。
 *
 * 相对路径不允许越出项目根目录；读取项目外的数据时切换绝对路径。
 */
function vizResolveDirectory() {
  const raw = String(vizEl.directoryInput?.value || "").trim();
  if (!raw) {
    throw new Error("父文件夹不能为空");
  }
  if (viz.pathMode === "absolute") {
    if (!raw.startsWith("/")) {
      throw new Error("绝对路径必须以 / 开头");
    }
    return { display: vizNormalizeDirectory(raw), absolute: vizNormalizeDirectory(raw), relative: "" };
  }
  if (raw.startsWith("/")) {
    throw new Error("项目相对路径不能以 / 开头；读取项目外的数据请切换绝对路径");
  }
  const normalized = vizNormalizeDirectory(raw.replace(/^\.\//, ""));
  if (normalized.split("/").includes("..")) {
    throw new Error("项目相对路径不允许越出项目根目录");
  }
  return {
    display: viz.projectRoot ? `${viz.projectRoot}/${normalized}` : normalized,
    absolute: viz.projectRoot ? `${viz.projectRoot}/${normalized}` : "",
    relative: normalized,
  };
}

/** @brief 读取生效目录下的一个文件，返回 ArrayBuffer。 */
async function vizFetchArtifact(name) {
  const target = viz.activeTarget;
  if (!target) {
    throw new Error("尚无生效目录");
  }
  if (target.relative) {
    const response = await fetch(`/${target.relative}/${name}`, { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`${name}: HTTP ${response.status}`);
    }
    return response.arrayBuffer();
  }
  return fetchAbsoluteFile(`${target.absolute}/${name}`);
}

/**
 * @brief 用户点击「加载」：解析校验新目录，通过后才整体切换当前生效目录。
 *
 * 输入框变化本身不启动新目录读取；加载失败保留旧画面及其来源。
 */
async function vizLoadDirectory() {
  let target;
  try {
    target = vizResolveDirectory();
  } catch (error) {
    vizSetStatus(`加载失败：${error.message}`);
    vizLog("二 load_result_directory", "2/2 失败", `原因=${error.message}`);
    return;
  }
  if (vizEl.resolvedPath) {
    vizEl.resolvedPath.textContent = target.display;
  }
  vizLog("二 load_result_directory", "1/2",
    `路径模式=${viz.pathMode === "relative" ? "相对" : "绝对"} 输入目录=${vizEl.directoryInput.value} 旧目录监听=停止 旧读取任务=作废`);

  // 暂停旧目录监听及当前回放，保留旧画面，先显示「目录待加载」。
  vizStopPolling();
  pauseTrajectory();
  viz.requestEpoch += 1;
  vizSetStatus("目录待加载");

  const epoch = viz.requestEpoch;
  const previousTarget = viz.activeTarget;
  viz.activeTarget = target;
  try {
    const manifest = await vizReadManifest();
    if (epoch !== viz.requestEpoch) {
      return; // 迟到的响应不能应用到新数据源
    }
    viz.activeDirectory = target.display;
    viz.appliedGenerationId = "";
    viz.appliedPointCloudGenerationId = "";
    if (vizEl.activeDirectory) {
      vizEl.activeDirectory.textContent = target.display;
    }
    vizLog("二 load_result_directory", "2/2", `实际目录=${target.display} 目录校验=通过 固定模型=复用`);
    await vizApplyManifest(manifest, epoch, { manual: true });
  } catch (error) {
    if (epoch !== viz.requestEpoch) {
      return;
    }
    // 默认相对目录可能在页面打开时尚未产生 manifest（或正由发布器原子替换）。
    // 这不是目录无效：保留目标并持续轮询，首轮发布后即可自动接入，绝不要求用户改绝对路径。
    if (target.relative) {
      viz.activeDirectory = target.display;
      viz.appliedGenerationId = "";
      viz.appliedPointCloudGenerationId = "";
      if (vizEl.activeDirectory) {
        vizEl.activeDirectory.textContent = target.display;
      }
      vizSetStatus(`正在自动监听 ${target.relative}（等待 trajectory_manifest.json）`);
      vizLog("二 load_result_directory", "2/2 等待", `相对目录=${target.relative} 原因=${error.message} 监听=保留`);
      return;
    }
    // 新目录未生效：保留旧画面并标明其实际来源。
    viz.activeTarget = previousTarget;
    vizSetStatus(`加载失败：${error.message}；保留上一轮结果（来源 ${viz.activeDirectory || "无"}）`);
    vizLog("二 load_result_directory", "2/2 失败",
      `实际目录=${target.display} 原因=${error.message} 新目录=未生效 旧画面=保留并标明来源`);
  } finally {
    if (viz.autoFollow) {
      vizStartPolling();
    }
  }
}

// ---------------------------------------------------------------------------
// 三 update_visualization：检查发布状态并整体更新动态数据
// ---------------------------------------------------------------------------

/** @brief 读取当前生效目录的 manifest。 */
async function vizReadManifest() {
  const buffer = await vizFetchArtifact("trajectory_manifest.json");
  return JSON.parse(new TextDecoder("utf-8").decode(new Uint8Array(buffer)));
}

/**
 * @brief 一次轮询：只读 manifest；没有新结果时不重复下载轨迹和点云。
 */
async function vizPollOnce() {
  if (viz.loading || !viz.activeTarget) {
    return;
  }
  const epoch = viz.requestEpoch;
  let manifest = null;
  try {
    manifest = await vizReadManifest();
  } catch (_error) {
    return; // 轮询读不到 manifest 不刷屏，也不改写规划结果
  }
  if (epoch !== viz.requestEpoch) {
    return;
  }
  await vizApplyManifest(manifest, epoch, { manual: false });
}

/**
 * @brief 按 manifest 状态决定是否整体更新画面。
 *
 * 1. 确认 status=completed、ready=true，且是待处理的新代次；
 * 2. 读取两个轨迹文件和点云；
 * 3. 校验任务代次、文件哈希、点数及两段衔接；
 * 4. 核对固定模型关联，准备本轮抓取预览；
 * 5. 再次读取 manifest，确认读取期间发布状态和代次未切换；
 * 6. 全部准备完成后一次性替换动态画面。
 */
async function vizApplyManifest(manifest, epoch, options) {
  const status = String(manifest?.status || "");
  const generationId = String(manifest?.generation_id || "");
  const requestId = String(manifest?.request_id || "");
  if (vizEl.requestId) {
    vizEl.requestId.textContent = requestId || "未取得";
  }
  if (vizEl.generationId) {
    vizEl.generationId.textContent = generationId || "未取得";
  }

  if (status === "processing") {
    pauseTrajectory();
    vizSetStatus("等待本轮结果（status=processing）");
    return;
  }
  if (status === "failed" || manifest?.ready !== true) {
    const reason = manifest?.reason || "本轮未发布可执行轨迹";
    vizSetStatus(`本轮失败/无解：${reason}`);
    vizLog("三 update_visualization", "1/3", `request_id=${requestId} 本轮 ready=false 原因=${reason}`);
    return;
  }
  const pointCloudReady = manifest?.point_cloud?.status === "ready";
  if (!options.manual && generationId && generationId === viz.appliedGenerationId &&
      (!pointCloudReady || generationId === viz.appliedPointCloudGenerationId)) {
    return; // 同一代次的轨迹与点云都已经应用，不重复下载或回放
  }

  viz.loading = true;
  try {
    vizSetStatus("加载中");
    const started = performance.now();
    const artifacts = await Promise.all([
      vizFetchArtifact("joint_trajectory_photo_to_grasp.json"),
      vizFetchArtifact("joint_trajectory_grasp_to_place.json"),
      ...(pointCloudReady ? [vizFetchArtifact("point_cloud_B.ply")] : []),
    ]);
    const [photoBuffer, placeBuffer, cloudBuffer] = artifacts;
    if (epoch !== viz.requestEpoch) {
      return;
    }

    // 校验发布一致性、文件哈希与关节衔接；任一不过都不应用半套结果。
    await validateManifestBundle(manifest, photoBuffer, placeBuffer);
    if (cloudBuffer) {
      const cloudSha = await sha256Hex(cloudBuffer);
      const expectedCloudSha = String(manifest?.point_cloud?.sha256 || "");
      if (expectedCloudSha && cloudSha !== expectedCloudSha) {
        throw new Error("点云 SHA-256 与 manifest 不一致");
      }
    }

    // 核对固定模型关联：机型同为 i12h 不足以认定历史平台/末端/标定相同。
    const manifestModelId = String(manifest?.collision_model?.model_id || "");
    const loadedModelId = String(viz.fixedModel?.model_id || "");
    let modelMatch = "未提供关联信息";
    if (manifestModelId && loadedModelId) {
      modelMatch = manifestModelId === loadedModelId ? "匹配" : "不匹配";
    }
    if (modelMatch === "不匹配") {
      vizSetStatus(`固定模型不匹配（本轮 ${manifestModelId} / 已加载 ${loadedModelId}），请用匹配配置重新初始化可视化`);
      vizLog("三 update_visualization", "2/3 失败", "原因=固定模型标识不匹配，停止自动回放");
      return;
    }

    // 再次读取 manifest，确认读取期间发布状态与代次未切换。
    const recheck = await vizReadManifest();
    if (epoch !== viz.requestEpoch) {
      return;
    }
    if (String(recheck?.generation_id || "") !== generationId || recheck?.ready !== true) {
      vizSetStatus("读取期间本轮代次已切换，放弃本次应用");
      return;
    }

    // ---- 本轮体素网格范围（改动十 10.2）----
    const collisionWorld = recheck?.collision_world || manifest?.collision_world || null;
    const gridOk = vizBuildWorldGridBuffer(collisionWorld);
    if (gridOk) {
      const shape = collisionWorld.grid_shape || [];
      vizLog("三 update_visualization", "3/3",
        `体素网格=${shape.join("×")} 体素=${collisionWorld.voxel_size_mm}mm ` +
        `CAP=${collisionWorld.cap_mm}mm z_hi=${collisionWorld.z_hi_mm}mm 外扩=无 ` +
        `占据=${collisionWorld.occupied_voxel_count}` +
        `(点云 ${collisionWorld.occupied_from_cloud} + 平台 ${collisionWorld.occupied_from_platform}) ` +
        `范围 x[${collisionWorld.grid_min_mm?.[0]}, ${collisionWorld.grid_max_mm?.[0]}] ` +
        `y[${collisionWorld.grid_min_mm?.[1]}, ${collisionWorld.grid_max_mm?.[1]}] ` +
        `z[${collisionWorld.grid_min_mm?.[2]}, ${collisionWorld.grid_max_mm?.[2]}]mm`);
    } else {
      // 旧归档没有 collision_world 节：隐藏网格并提示，不沿用上一轮网格冒充本轮。
      vizLog("三 update_visualization", "3/3",
        "该归档缺少碰撞世界元数据，已隐藏体素网格显示（不沿用上一轮网格）");
    }

    // ---- 点云在后台产出：轨迹先进入画面；同代次 manifest 补发 ready 后再补点云。 ----
    let cloud = null;
    if (cloudBuffer) {
      cloud = parsePlyPointCloud(cloudBuffer, { maxPoints: DEFAULT_MAX_PLY_POINTS });
      setPointCloudData(cloud.points, cloud);
      viz.appliedPointCloudGenerationId = generationId;
      // 本轮体素区域素材必须在点云解析之后准备。
      viz.voxelSource = gridOk
        ? { collisionWorld, pointsMm: cloud.points, stride: Number(cloud.stride) || 1 }
        : null;
      viz.voxelBuilt = false;
      viz.voxelRegionModel = null;
      viz.voxelFaceGridBuffer = null;
      viz.occupiedVoxelBuffer = null;
      viz.occupiedVoxelCount = 0;
      if (gridOk && viz.showVoxelRegion) {
        vizEnsureVoxelRegion();
      }
    }

    const photoFile = new File([photoBuffer], "joint_trajectory_photo_to_grasp.json",
      { type: "application/json" });
    const placeFile = new File([placeBuffer], "joint_trajectory_grasp_to_place.json",
      { type: "application/json" });
    await loadTrajectoryFiles([photoFile, placeFile], { autoplay: false });

    vizApplyGraspPreview(manifest);
    viz.appliedGenerationId = generationId;
    viz.activeDirectory = viz.activeTarget.display;
    if (vizEl.activeDirectory) {
      vizEl.activeDirectory.textContent = viz.activeDirectory;
    }

    const elapsed = Math.round(performance.now() - started);
    vizLog("三 update_visualization", "1/3",
      `request_id=${requestId} generation_id=${generationId} 两条轨迹=${cloud ? "读取完成，点云已补入" : "已优先读取，等待后台点云"}` +
      `${cloud ? ` 点数=${cloud.points.length}` : ""} 耗时=${elapsed}ms`);
    vizLog("三 update_visualization", "2/3",
      `发布一致性/文件哈希/模型关联(${modelMatch})/关节衔接=校验通过`);
    vizLog("三 update_visualization", "3/3",
      `${cloud ? "轨迹及彩色点云" : "轨迹"}=已应用 当前生效目录=${viz.activeDirectory} 固定模型=复用`);
    vizSetStatus(`${cloud ? "已加载轨迹与彩色点云" : "轨迹已就绪，等待彩色点云"} ${viz.activeDirectory}（代次 ${generationId.slice(0, 12)}…，模型关联 ${modelMatch}）`);

    // 自动开关开启就自动播放一次；关闭则只加载显示，把机器人置于轨迹实际起点。
    if (viz.autoFollow) {
      stopTrajectory();
      playTrajectory();
      vizLog("四 play_trajectory", "1/3", `request_id=${requestId} 开始=拍照位→抓取位`);
    } else {
      stopTrajectory();
    }
  } catch (error) {
    if (epoch === viz.requestEpoch) {
      vizSetStatus(`本轮未应用：${error.message}`);
      vizLog("三 update_visualization", "2/3 失败",
        `generation_id=${generationId} 原因=${error.message} 本轮画面=未应用 自动回放=未启动`);
    }
  } finally {
    viz.loading = false;
  }
}

/**
 * @brief 准备本轮抓取夹爪预览；旧归档缺少 grasp_preview 时明确降级。
 */
function vizApplyGraspPreview(manifest) {
  const preview = manifest?.grasp_preview;
  if (!preview || !Array.isArray(preview.candidates) || preview.candidates.length === 0) {
    viz.preview = null;
    // 选中候选没了：坐标系也要跟着清掉，不能留着上一轮的。
    rebuildGraspCandidatePoses();
    if (vizEl.previewStatus) {
      // 不保留上一轮夹爪冒充本轮；已通过校验的轨迹和点云仍可显示。
      vizEl.previewStatus.textContent = "该归档缺少抓取夹爪预览数据，已隐藏夹爪预览（轨迹与点云不受影响）";
    }
    vizLog("三 update_visualization", "3/3 预览不可用", "原因=归档缺少 grasp_preview");
    return;
  }
  viz.preview = preview;
  const selected = Number(preview.selected_candidate_index);
  // 抓取候选坐标系是另一条独立轮询 grasp_result.json 的路径，选中编号在这里才拿到。
  // 编号变了就重建它的缓冲，否则画面会停在上一轮选中的那个候选上。
  const previousSelected = state.graspCandidatePoseWatcher.selectedIndex;
  const currentSelected = (Number.isFinite(selected) && selected >= 0) ? selected : null;
  if (previousSelected !== currentSelected) {
    rebuildGraspCandidatePoses();
    updateGraspCandidatePoseStatus();
  }
  const clearance = Number(preview.camera_clearance_along_tcp_z_mm);
  if (vizEl.previewStatus) {
    const chosen = preview.candidates.find(
      (item) => Number(item.candidate_index) === selected
    );
    const width = chosen ? `${(Number(chosen.gripper_width) * 1000).toFixed(1)}mm` : "-";
    const score = chosen ? Number(chosen.score).toFixed(4) : "-";
    const hasSelection = Number.isFinite(selected) && selected >= 0;
    vizEl.previewStatus.textContent =
      `本轮候选 ${preview.candidates.length} 个，` +
      (hasSelection
        ? `只显示最终选中的 #${selected}（分数=${score} 开口=${width}）`
        : "本轮未选出候选，不显示夹爪") +
      `；本轮实际退让量=${clearance}mm（沿 TCP 局部 ${clearance < 0 ? "−Z" : "+Z"}）`;
  }
}

// ---------------------------------------------------------------------------
// 四 play_trajectory / 渲染：夹爪预览、带孔平台与豁免区轮廓
// ---------------------------------------------------------------------------

/**
 * @brief 选出要显示的预览条目：**只有本轮最终选中的那个候选**。
 *
 * 未选出候选（本轮无解）时返回空，不退而求其次画别的候选顶替。
 */
function vizVisiblePreviewCandidates() {
  const candidates = viz.preview?.candidates;
  if (!Array.isArray(candidates) || candidates.length === 0) {
    return [];
  }
  const selected = Number(viz.preview.selected_candidate_index);
  if (!Number.isFinite(selected) || selected < 0) {
    return [];
  }
  const match = candidates.find((item) => Number(item.candidate_index) === selected);
  return match ? [match] : [];
}

/**
 * @brief 画一只预览夹爪：直接使用已发布的位姿，不再叠加退让量。
 */
function vizDrawGripperAt(pose, color, view, projection) {
  if (!state.toolMesh || !Array.isArray(pose) || pose.length < 6) {
    return;
  }
  const extents = viz.toolExtentsM || state.tool;
  const sizeX = extents.negativeX + extents.positiveX;
  const sizeY = extents.negativeY + extents.positiveY;
  const sizeZ = extents.negativeZ + extents.positiveZ;
  if (!(sizeX > 0 && sizeY > 0 && sizeZ > 0)) {
    return;
  }
  const flange = poseMmDegToMat4(pose);
  const center = mat4Translation(
    0.5 * (extents.positiveX - extents.negativeX),
    0.5 * (extents.positiveY - extents.negativeY),
    0.5 * (extents.positiveZ - extents.negativeZ),
  );
  const model = mat4Multiply(flange, mat4Multiply(center, mat4Scale(sizeX, sizeY, sizeZ)));
  drawMeshWithOpacity(state.toolMesh, model, view, projection, VIZ_GRIPPER_OPACITY, color);
}

/**
 * @brief 第三流程的附加渲染：带孔平台、豁免区轮廓与两组夹爪预览。
 *
 * 两组预览只用于对照，不加入碰撞世界，也不新增「退让位→原始抓取位」的运动。
 */
function vizDrawOverlays(view, projection) {
  // 体素区域（改动十 10.3，默认关闭）：ESDF 测距覆盖的整个网格。
  if (viz.showVoxelRegion) {
    // ① 占据格标红：点云 + 平台，全部画出来。
    if (viz.occupiedVoxelBuffer?.count > 0) {
      drawPoints(viz.occupiedVoxelBuffer, mat4Identity(), view, projection,
                 VIZ_OCCUPIED_VOXEL_COLOR, VIZ_OCCUPIED_VOXEL_POINT_SIZE);
    }
    // ② 六个面上的 10mm 真实格线：看清格子大小。
    if (viz.voxelFaceGridBuffer?.count > 0) {
      drawLines(viz.voxelFaceGridBuffer, mat4Identity(), view, projection,
                VIZ_VOXEL_FACE_GRID_COLOR, 1);
    }
    // ③ 半透明盒：给出测距区域的整体范围。最后画，压在内容之上。
    if (viz.voxelRegionModel) {
      drawMeshWithOpacity(state.cartMesh, viz.voxelRegionModel, view, projection,
                          VIZ_VOXEL_REGION_OPACITY, VIZ_VOXEL_REGION_COLOR);
    }
  }
  // 体素网格范围（改动十 10.2）：黄色线框，默认打开。
  if (viz.showWorldGrid && viz.gridBuffer?.count > 0) {
    drawLines(viz.gridBuffer, mat4Identity(), view, projection, [0.95, 0.78, 0.15], 2);
  }
  if (viz.showExclusion && viz.exclusionBuffer?.count > 0) {
    drawLines(viz.exclusionBuffer, mat4Identity(), view, projection, VIZ_EXCLUSION_COLOR, 2);
  }
  const candidates = vizVisiblePreviewCandidates();
  for (const candidate of candidates) {
    if (viz.showOriginalGripper) {
      vizDrawGripperAt(candidate.grasp_flange, VIZ_ORIGINAL_GRIPPER_COLOR, view, projection);
    }
    if (viz.showRetreatGripper) {
      vizDrawGripperAt(candidate.trajectory_grasp_flange, VIZ_RETREAT_GRIPPER_COLOR, view, projection);
    }
  }
}

/**
 * @brief 带孔平台：按固定模型快照的长方体分解绘制，孔是真的，不是叠加的轮廓。
 * @return 已绘制返回 true；没有快照时返回 false，由调用方退回单个平台盒
 */
function vizDrawPlatformBoxes(view, projection) {
  if (!state.cartMesh || viz.platformBoxes.length === 0) {
    return false;
  }
  for (const box of viz.platformBoxes) {
    const center = box.center_mm || [0, 0, 0];
    const dims = box.dims_mm || [0, 0, 0];
    const sizeX = mmToM(Number(dims[0]) || 0);
    const sizeY = mmToM(Number(dims[1]) || 0);
    const sizeZ = mmToM(Number(dims[2]) || 0);
    if (!(sizeX > 0 && sizeY > 0 && sizeZ > 0)) {
      continue;
    }
    const model = mat4Multiply(
      mat4Translation(mmToM(Number(center[0]) || 0), mmToM(Number(center[1]) || 0), mmToM(Number(center[2]) || 0)),
      mat4Scale(sizeX, sizeY, sizeZ),
    );
    drawMesh(state.cartMesh, model, view, projection);
  }
  return true;
}

// ---------------------------------------------------------------------------
// 轮询与控件
// ---------------------------------------------------------------------------

function vizStartPolling() {
  if (viz.pollTimer !== null) {
    return;
  }
  viz.pollTimer = window.setInterval(() => {
    vizPollOnce().catch(() => {});
  }, VIZ_POLL_INTERVAL_MS);
}

function vizStopPolling() {
  if (viz.pollTimer !== null) {
    window.clearInterval(viz.pollTimer);
    viz.pollTimer = null;
  }
}

function vizBindElements() {
  vizEl.pathModeRelative = document.getElementById("vizPathModeRelative");
  vizEl.pathModeAbsolute = document.getElementById("vizPathModeAbsolute");
  vizEl.directoryInput = document.getElementById("vizDirectoryInput");
  vizEl.loadDirectory = document.getElementById("vizLoadDirectory");
  vizEl.resolvedPath = document.getElementById("vizResolvedPath");
  vizEl.activeDirectory = document.getElementById("vizActiveDirectory");
  vizEl.requestId = document.getElementById("vizRequestId");
  vizEl.generationId = document.getElementById("vizGenerationId");
  vizEl.autoFollow = document.getElementById("vizAutoFollow");
  vizEl.status = document.getElementById("vizStatus");
  vizEl.showOriginalGripper = document.getElementById("vizShowOriginalGripper");
  vizEl.showRetreatGripper = document.getElementById("vizShowRetreatGripper");
  vizEl.previewStatus = document.getElementById("vizPreviewStatus");
  vizEl.showExclusion = document.getElementById("vizShowInstallationExclusion");
  vizEl.showWorldGrid = document.getElementById("vizShowWorldGrid");
  vizEl.showVoxelRegion = document.getElementById("vizShowVoxelRegion");
  vizEl.modelStatus = document.getElementById("vizModelStatus");
}

function vizSetupEvents() {
  // 路径模式只决定如何解析目录；自动开关决定是否跟随并播放，二者相互独立。
  const onModeChange = () => {
    viz.pathMode = vizEl.pathModeAbsolute?.checked ? "absolute" : "relative";
    vizStopPolling();
    pauseTrajectory();
    viz.requestEpoch += 1;
    vizSetStatus("目录待加载");
    try {
      const target = vizResolveDirectory();
      vizEl.resolvedPath.textContent = target.display;
    } catch (error) {
      vizEl.resolvedPath.textContent = `无法解析：${error.message}`;
    }
  };
  vizEl.pathModeRelative?.addEventListener("change", onModeChange);
  vizEl.pathModeAbsolute?.addEventListener("change", onModeChange);
  // 输入框变化只更新「解析后的路径」，不启动新目录读取。
  vizEl.directoryInput?.addEventListener("input", () => {
    try {
      vizEl.resolvedPath.textContent = vizResolveDirectory().display;
    } catch (error) {
      vizEl.resolvedPath.textContent = `无法解析：${error.message}`;
    }
  });
  vizEl.loadDirectory?.addEventListener("click", () => {
    vizLoadDirectory().catch((error) => vizSetStatus(`加载失败：${error.message}`));
  });
  vizEl.autoFollow?.addEventListener("change", () => {
    viz.autoFollow = Boolean(vizEl.autoFollow.checked);
    if (viz.autoFollow) {
      // 重新开启时先检查当前生效目录，有新代次才自动更新。
      vizStartPolling();
      vizLog("", "开关变化", `自动跟随=开启 监听=启动 当前目录=${viz.activeDirectory}`);
    } else {
      vizStopPolling();
      pauseTrajectory();
      vizLog("", "开关变化", `自动跟随=关闭 监听=停止 回放=暂停 当前目录=${viz.activeDirectory}`);
    }
  });
  vizEl.showOriginalGripper?.addEventListener("change", () => {
    viz.showOriginalGripper = Boolean(vizEl.showOriginalGripper.checked);
  });
  vizEl.showRetreatGripper?.addEventListener("change", () => {
    viz.showRetreatGripper = Boolean(vizEl.showRetreatGripper.checked);
  });
  vizEl.showExclusion?.addEventListener("change", () => {
    // 关闭只隐藏辅助轮廓与标注，平台仍然有孔。
    viz.showExclusion = Boolean(vizEl.showExclusion.checked);
    vizLog("", "开关变化", `安装豁免区轮廓=${viz.showExclusion ? "显示" : "隐藏"}（平台孔洞仍存在）`);
  });
  // 改动十：只控制显示，不重建网格，也不影响规划中的碰撞检测。
  vizEl.showWorldGrid?.addEventListener("change", () => {
    viz.showWorldGrid = Boolean(vizEl.showWorldGrid.checked);
    vizLog("", "开关变化", `体素网格范围=${viz.showWorldGrid ? "显示" : "隐藏"}（网格本身不变）`);
  });
  vizEl.showVoxelRegion?.addEventListener("change", () => {
    viz.showVoxelRegion = Boolean(vizEl.showVoxelRegion.checked);
    if (viz.showVoxelRegion) {
      vizEnsureVoxelRegion();
    }
    vizLog("", "开关变化",
      `体素区域=${viz.showVoxelRegion ? "显示（平台实体盒临时让位）" : "隐藏"} ` +
      `占据格 ${viz.occupiedVoxelCount} 个（点云 ${viz.occupiedFromCloud} + ` +
      `平台 ${viz.occupiedFromPlatform}）` +
      `（只控制显示，不重新体素化，也不影响规划中的碰撞检测）`);
  });
}

/**
 * @brief 第三流程启动：加载固定资源 → 进入默认目录 → 开启自动跟随。
 */
async function vizInitialize() {
  vizBindElements();
  if (!vizEl.directoryInput) {
    return; // 页面未包含第三流程面板
  }
  vizLog("", "初始化开始", "stage_count=4");
  vizSetupEvents();
  viz.projectRoot = await vizFetchProjectRoot();
  vizLog("一 initialize_visualization", "1/3",
    `机型=aubo_i12h 项目根目录=${viz.projectRoot || "（服务未提供，按页面相对路径解析）"}`);
  await vizLoadFixedModel();
  vizEl.directoryInput.value = VIZ_DEFAULT_RELATIVE_DIRECTORY;
  viz.autoFollow = Boolean(vizEl.autoFollow?.checked);
  try {
    vizEl.resolvedPath.textContent = vizResolveDirectory().display;
  } catch (_error) {
    // 解析失败时状态行已说明，保持「目录待加载」。
  }
  vizLog("", "初始化完成", `model_id=${viz.fixedModel?.model_id || "未取得"}`);
  await vizLoadDirectory();
}
