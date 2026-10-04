// Franka Panda の作業台(ブロックと箱つき)。Panda 系の全アプリの土台。
// pick_place.py の build_spec / Robot / pick_and_place の移植(IK だけ mink ではなく ik.js の減衰最小二乗法)。
//
//   const cell = await PandaCell.create(mujoco, load, { blocks, extraWorldXml })
//   for (const _ of cell.run()) { ... }   // 1 回の yield = 制御 1 ステップ(0.02 秒)
import { mountFiles, writeText, siteId } from "./mujoco.js";
import { ArmIK } from "./ik.js";
import { mulberry32, smooth } from "./math.js";

export const BLOCK = 0.02; // ブロックの半辺 [m](4 cm 角)
export const BLOCKS = [
  { name: "red", xy: [0.50, 0.25], rgba: [0.85, 0.25, 0.20, 1] },
  { name: "blue", xy: [0.62, 0.08], rgba: [0.20, 0.50, 0.85, 1] },
  { name: "green", xy: [0.45, -0.02], rgba: [0.25, 0.65, 0.35, 1] },
];
export const BOX_CENTER = [0.40, -0.38];
export const BOX_HALF = 0.10, BOX_WALL = 0.008, BOX_HEIGHT = 0.05;
export const DROP_SPOTS = [[-0.04, -0.04], [0.04, 0.04], [0.04, -0.04]]; // 箱の中で少しずつずらして落とす
export const GRIP_OPEN = 255, GRIP_CLOSED = 0;
export const SAFE_Z = 0.25;
export const N_PHYSICS = 10; // 制御 1 ステップあたりの物理ステップ数
export const CONTROL_DT = 0.002 * N_PHYSICS; // 0.02 秒
export const N_ARM = 7;

// 衝突メッシュだけに絞ったモデル(trim_panda.py で Menagerie から作る)。panda.xml は /panda に置く
export const PANDA_FILES = [
  "panda.xml",
  "assets/link0.stl", "assets/link1.stl", "assets/link2.stl", "assets/link3.stl", "assets/link4.stl",
  "assets/link5_collision_0.obj", "assets/link5_collision_1.obj", "assets/link5_collision_2.obj",
  "assets/link6.stl", "assets/link7.stl", "assets/hand.stl", "assets/finger_0.obj",
];

export function blockXml({ name, xy, rgba, yaw = 0, half = BLOCK, mass = 0.05, friction = "1.5 0.01 0.001" }) {
  const quat = yaw ? `${Math.cos(yaw / 2)} 0 0 ${Math.sin(yaw / 2)}` : "1 0 0 0";
  return `<body name="block_${name}" pos="${xy[0]} ${xy[1]} ${half}" quat="${quat}"><freejoint/>
      <geom type="box" size="${half} ${half} ${half}" rgba="${rgba.join(" ")}" mass="${mass}" friction="${friction}"/></body>`;
}

export function boxXml(center = BOX_CENTER, half = BOX_HALF, wall = BOX_WALL, height = BOX_HEIGHT) {
  const h = height / 2;
  const walls = [[half, 0, wall, half], [-half, 0, wall, half], [0, half, half, wall], [0, -half, half, wall]]
    .map(([dx, dy, sx, sy]) => `<geom type="box" pos="${dx} ${dy} ${h}" size="${sx} ${sy} ${h}" rgba="0.55 0.42 0.3 1"/>`).join("\n");
  return `<body name="box" pos="${center[0]} ${center[1]} 0">${walls}</body>`;
}

export function pandaSceneXml({ blocks = BLOCKS, box = true, extraAssetXml = "", extraWorldXml = "" } = {}) {
  return `<mujoco model="panda cell">
  <include file="panda.xml"/>
  <asset>${extraAssetXml}</asset>
  <worldbody>
    <geom name="floor" type="plane" size="0 0 0.05" rgba="0.85 0.84 0.8 1"/>
    ${blocks.map(blockXml).join("\n")}
    ${box ? boxXml() : ""}
    ${extraWorldXml}
  </worldbody>
</mujoco>`;
}

export class PandaCell {
  // load: (file) => Promise<Uint8Array>。ファイルは _web/assets/panda/ からの相対パス
  static async create(mujoco, load, opts = {}) {
    await mountFiles(mujoco, "/panda", PANDA_FILES, (f) => load(`assets/panda/${f}`));
    return new this(mujoco, opts);
  }

  constructor(mujoco, opts = {}) {
    this.mj = mujoco;
    this.model = null;
    this.reset(opts);
  }

  // blocks に seed に従う位置のゆらぎ(jitter [m])を足して、場面を作り直す
  reset({ seed = 1, jitter = 0, blocks = BLOCKS, box = true, extraAssetXml = "", extraWorldXml = "", keyframe = "home" } = {}) {
    const mj = this.mj;
    this.opts = { seed, jitter, blocks, box, extraAssetXml, extraWorldXml, keyframe };
    const rnd = mulberry32(seed);
    this.blocks = blocks.map((b) => (jitter > 0
      ? { ...b, xy: [b.xy[0] + (rnd() * 2 - 1) * jitter, b.xy[1] + (rnd() * 2 - 1) * jitter] } : b));
    this.dispose();
    writeText(mj, "/panda/scene.xml", pandaSceneXml({ blocks: this.blocks, box, extraAssetXml, extraWorldXml }));
    this.model = mj.MjModel.from_xml_path("/panda/scene.xml");
    this.data = new mj.MjData(this.model);
    const m = this.model;
    this.dt = m.opt.timestep * N_PHYSICS;
    // ホーム姿勢はロボットの 9 個の関節分だけ使う。ブロックは置いた場所のまま
    const home = m.key(keyframe);
    this.data.qpos.set(home.qpos.slice(0, 9));
    this.data.ctrl.set(home.ctrl);
    mj.mj_forward(m, this.data);
    this.siteId = siteId(mj, m, "tcp");
    this.ik = new ArmIK(mj, m, { siteId: this.siteId, q0: Array.from(this.data.qpos.slice(0, N_ARM)) });
    this.grip = GRIP_OPEN;
    this.log = [];
    this.steps = 0;
  }

  dispose() {
    this.ik?.dispose();
    this.data?.delete();
    this.model?.delete();
    this.model = this.data = this.ik = null;
  }

  get time() { return this.data.time; }
  get q() { return this.ik.q; }

  tcpTarget() { return this.ik.target(); } // IK で積分している「目標の関節角」での手先位置
  tcp() { // 実際の手先位置
    const p = this.data.site_xpos, i = 3 * this.siteId;
    return [p[i], p[i + 1], p[i + 2]];
  }

  blockPos(name) {
    const p = this.data.body(`block_${name}`).xpos;
    return [p[0], p[1], p[2]];
  }

  // 物理だけを n ステップ進める(IK は使わず、いまの ctrl のまま)
  physics(n = N_PHYSICS) {
    for (let k = 0; k < n; k++) this.mj.mj_step(this.model, this.data);
  }

  // IK を 1 回解いて関節の目標角度を出し、物理を N_PHYSICS ステップ進める
  step(target) {
    const q = this.ik.solve(target);
    for (let i = 0; i < N_ARM; i++) this.data.ctrl[i] = q[i]; // 各関節の位置サーボに目標角度を渡す
    this.data.ctrl[7] = this.grip; // 指は開閉の指令だけ
    this.physics();
    this.steps++;
  }

  *move(goal, duration) {
    const start = this.tcpTarget();
    const steps = Math.floor(duration / this.dt);
    for (let k = 1; k <= steps; k++) {
      const u = smooth(k / steps);
      this.step(start.map((s, i) => s + (goal[i] - s) * u));
      yield;
    }
  }

  *gripper(value, duration) {
    this.grip = value;
    const hold = this.tcpTarget();
    for (let k = 0, n = Math.floor(duration / this.dt); k < n; k++) {
      this.step(hold);
      yield;
    }
  }

  *pickAndPlace(name, drop, { pos = null, yaw = 0 } = {}) {
    const [x, y] = pos ?? this.blockPos(name); // 1. 見つける(既定ではシミュレーターから位置を直接読む)
    this.ik.yaw = yaw;
    this.log.push(`${name}: 位置 (${x.toFixed(2)}, ${y.toFixed(2)}) を見つけた`);
    yield* this.move([x, y, SAFE_Z], 1.6); // 真上へ
    yield* this.move([x, y, BLOCK + 0.002], 1.0); // 4. つかむ: 真下に降りて
    yield* this.gripper(GRIP_CLOSED, 0.6); //    指を閉じる
    yield* this.move([x, y, SAFE_Z], 1.0); // 5. 運ぶ: 持ち上げて
    const z = this.blockPos(name)[2];
    this.log.push(`${name}: 持ち上げ${z > 0.1 ? "成功" : "失敗(すべり落ちた)"} 高さ ${z.toFixed(3)} m`);
    const [bx, by] = [BOX_CENTER[0] + drop[0], BOX_CENTER[1] + drop[1]];
    this.ik.yaw = 0;
    yield* this.move([bx, by, SAFE_Z], 1.6); //    箱の上へ
    yield* this.move([bx, by, BOX_HEIGHT + 0.06], 0.8);
    yield* this.gripper(GRIP_OPEN, 0.5); // 6. はなす
    yield* this.move([bx, by, SAFE_Z], 0.8);
  }

  // 全体の作業(pick_place.py の run と同じ)
  *run() {
    yield* this.gripper(GRIP_OPEN, 0.5);
    for (let i = 0; i < this.blocks.length; i++) yield* this.pickAndPlace(this.blocks[i].name, DROP_SPOTS[i % DROP_SPOTS.length]);
    const t = this.tcpTarget();
    yield* this.move([t[0] + 0.1, t[1] + 0.25, t[2] + 0.1], 1.5);
    yield* this.gripper(GRIP_OPEN, 1.0); // ブロックが落ち着くまで待つ
  }

  inBox(p) {
    return Math.abs(p[0] - BOX_CENTER[0]) < BOX_HALF && Math.abs(p[1] - BOX_CENTER[1]) < BOX_HALF && p[2] < BOX_HEIGHT;
  }

  results() {
    return Object.fromEntries(this.blocks.map(({ name }) => [name, this.inBox(this.blockPos(name))]));
  }
}
