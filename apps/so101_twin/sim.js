// SO-101 のデジタルツイン(sim_so101.py と pick_place.py の移植)。tomato_harvest も同じ場面とモーター模擬を使う。
//
//   const robot = await SimSO101.create(mujoco, load, { maxRelativeTarget: 10 });
//   robot.connect();
//   const obs = robot.getObservation();        // { "shoulder_pan.pos": 度, ..., "gripper.pos": 0〜100 }
//   const sent = robot.sendAction(action);     // 同じ形の辞書。安全制限で切った値を返す
//   for (const _ of planAndRun(robot, ...)) {} // 1 回の yield = 制御 1 ステップ(40 Hz)
//
// 元の Python との違い:
//  - 逆運動学は mink(QP)ではなく、減衰つき最小二乗法(位置と向きに重みをつけた、_web/ik.js と同じ考え方)
//  - モデルは描画用メッシュを省いた軽量版(_web/trim_so101.py)。物理は元と同じ
//  - 手首カメラの映像は出さない(Python 版は動画に使っていた)
import { mountFiles, writeText } from "../_web/mujoco.js";
import { smooth, solve, rotZ, mul3, rotationError } from "../_web/math.js";

export const MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"];
export const COUNTS_PER_TURN = 4096;
export const GRIP_OPEN = 60, GRIP_CLOSED = 3; // gripper.pos(0 閉 〜 100 開)
export const SAFE_Z = 0.10, GRASP_Z = 0.012, DROP_Z = 0.07;
export const CUBE_HALF = 0.015;
export const TRAY_HALF = 0.05, TRAY_WALL = 0.004, TRAY_H = 0.025;
export const DEFAULTS = { cubeXY: [0.24, 0.10], cubeYaw: 20, trayXY: [0.18, -0.18] }; // 角度は度

export const SO101_FILES = [
  "so101.xml",
  "assets/wrist_roll_follower_so101_gripper_part0_v1.stl",
  "assets/moving_jaw_so101_gripper_part0_v1.stl",
  "assets/moving_jaw_so101_gripper_part1_v1.stl",
];

const deg = (r) => (r * 180) / Math.PI;
const rad = (d) => (d * Math.PI) / 180;
const clamp = (x, lo, hi) => Math.min(Math.max(x, lo), hi);

export function trayXml(center, { half = TRAY_HALF, wall = TRAY_WALL, height = TRAY_H, rgba = "0.3 0.55 0.85 1", name = "tray" } = {}) {
  const h = height / 2;
  const walls = [[half, 0, wall, half], [-half, 0, wall, half], [0, half, half, wall], [0, -half, half, wall]]
    .map(([dx, dy, sx, sy]) => `<geom type="box" pos="${dx} ${dy} ${h}" size="${sx} ${sy} ${h}" rgba="${rgba}"/>`).join("\n");
  return `<body name="${name}" pos="${center[0]} ${center[1]} 0">
      <geom type="box" pos="0 0 0.002" size="${half} ${half} 0.002" rgba="${rgba}"/>
      ${walls}</body>`;
}

export function cubeXml(xy, yaw) {
  return `<body name="cube" pos="${xy[0]} ${xy[1]} ${CUBE_HALF}" quat="${Math.cos(yaw / 2)} 0 0 ${Math.sin(yaw / 2)}"><freejoint/>
      <geom type="box" size="${CUBE_HALF} ${CUBE_HALF} ${CUBE_HALF}" rgba="0.9 0.25 0.2 1" mass="0.02" friction="1.5 0.01 0.001" condim="4"/></body>`;
}

export class SimSO101 {
  // load: (file) => Promise<Uint8Array>。ファイルは _web/assets/ からの相対パス
  static async create(mujoco, load, cfg = {}) {
    await mountFiles(mujoco, "/so101", SO101_FILES, (f) => load(`assets/so101/${f}`));
    return new this(mujoco, cfg);
  }

  constructor(mujoco, cfg = {}) {
    this.mj = mujoco;
    this.config = {
      fps: 40, // 制御の周期。物理は 5 ms 刻みなので 40 Hz なら 1 回の指令で 5 ステップ進む
      maxRelativeTarget: null, // 1 回の指令で動かせる量の上限 [度]。LeRobot と同じ意味
      quantize: true,
      cubeXY: DEFAULTS.cubeXY, cubeYaw: rad(DEFAULTS.cubeYaw), trayXY: DEFAULTS.trayXY,
      ...cfg,
    };
    this.model = null;
    this.data = null;
    this.build();
  }

  // 場面の XML。tomato_harvest は、これを置き換えて鉢植えとかごを置く
  sceneXml() {
    const c = this.config;
    return `<mujoco model="so101 scene">
  <include file="so101.xml"/>
  <worldbody>
    <geom name="floor" type="plane" size="0 0 0.05" rgba="0.85 0.84 0.8 1"/>
    ${cubeXml(c.cubeXY, c.cubeYaw)}
    ${trayXml(c.trayXY)}
  </worldbody>
</mujoco>`;
  }

  build() {
    const mj = this.mj;
    this.dispose();
    writeText(mj, "/so101/scene.xml", this.sceneXml());
    this.model = mj.MjModel.from_xml_path("/so101/scene.xml");
    const m = this.model;
    const id = (type, name) => mj.mj_name2id(m, mj.mjtObj[type].value, name);
    this.qadr = MOTORS.map((n) => m.jnt_qposadr[id("mjOBJ_JOINT", n)]);
    this.act = MOTORS.map((n) => id("mjOBJ_ACTUATOR", n));
    this.id = id;
    const g = id("mjOBJ_JOINT", "gripper");
    this.gripRange = [m.jnt_range[2 * g], m.jnt_range[2 * g + 1]];
    this.nSub = Math.max(1, Math.round(1 / (this.config.fps * m.opt.timestep)));
    this.dt = 1 / this.config.fps;
    this.data = new mj.MjData(m);
    this.connected = false;
    this.steps = 0;
  }

  dispose() {
    this.data?.delete();
    this.model?.delete();
    this.data = this.model = null;
  }

  get time() { return this.data.time; }

  // ---------- LeRobot と同じ口 ----------
  connect() {
    this.data?.delete();
    this.data = new this.mj.MjData(this.model);
    this.mj.mj_forward(this.model, this.data);
    this.act.forEach((a, i) => { this.data.ctrl[a] = this.data.qpos[this.qadr[i]]; });
    this.connected = true;
    this.steps = 0;
  }

  disconnect() { this.connected = false; }

  getObservation() {
    let q = this.qadr.map((a) => this.data.qpos[a]);
    if (this.config.quantize) { // エンコーダの分解能に丸める
      const step = (2 * Math.PI) / COUNTS_PER_TURN;
      q = q.map((v) => Math.round(v / step) * step);
    }
    return Object.fromEntries(MOTORS.map((m, i) => [`${m}.pos`, this.toUser(q)[i]]));
  }

  sendAction(action) {
    let goal = MOTORS.map((m) => action[`${m}.pos`]);
    const present = this.toUser(this.qadr.map((a) => this.data.qpos[a]));
    const cap = this.config.maxRelativeTarget;
    if (cap != null) goal = goal.map((g, i) => present[i] + clamp(g - present[i], -cap, cap)); // ensure_safe_goal_position
    const r = this.toRad(goal);
    this.act.forEach((a, i) => {
      this.data.ctrl[a] = clamp(r[i], this.model.actuator_ctrlrange[2 * a], this.model.actuator_ctrlrange[2 * a + 1]);
    });
    for (let k = 0; k < this.nSub; k++) this.mj.mj_step(this.model, this.data);
    this.steps++;
    return Object.fromEntries(MOTORS.map((m, i) => [`${m}.pos`, goal[i]]));
  }

  // ---------- シミュレーターだけの便利機能 ----------
  cubePose() {
    const b = this.data.body("cube");
    return { pos: [b.xpos[0], b.xpos[1], b.xpos[2]], yaw: 2 * Math.atan2(b.xquat[3], b.xquat[0]) };
  }

  cubeInTray() {
    const { pos } = this.cubePose();
    const t = this.config.trayXY;
    return Math.abs(pos[0] - t[0]) < TRAY_HALF && Math.abs(pos[1] - t[1]) < TRAY_HALF && pos[2] < TRAY_H;
  }

  jointTorques() { return this.act.map((a) => this.data.actuator_force[a]); }

  // 単位の変換(LeRobot の use_degrees=True に合わせる)
  toUser(r) {
    const out = r.map(deg);
    const [lo, hi] = this.gripRange;
    out[5] = ((r[5] - lo) / (hi - lo)) * 100;
    return out;
  }

  toRad(u) {
    const out = u.map(rad);
    const [lo, hi] = this.gripRange;
    out[5] = lo + (u[5] / 100) * (hi - lo);
    return out;
  }
}

// ---------- 回転の補間用 ----------
const T3 = (R) => [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]];
const trace = (R) => R[0] + R[4] + R[8];

// 回転行列 → 回転ベクトル(軸 × 角度)
export function logSO3(R) {
  const c = clamp((trace(R) - 1) / 2, -1, 1);
  const angle = Math.acos(c);
  const v = [R[7] - R[5], R[2] - R[6], R[3] - R[1]];
  if (angle < 1e-9) return [0, 0, 0];
  if (Math.PI - angle < 1e-5) { // 180° 付近: 対角成分から軸を出す
    const x = Math.sqrt(Math.max((R[0] + 1) / 2, 0)), y = Math.sqrt(Math.max((R[4] + 1) / 2, 0)), z = Math.sqrt(Math.max((R[8] + 1) / 2, 0));
    return [x * angle, Math.sign(v[1] || y) * y * angle, Math.sign(v[2] || z) * z * angle];
  }
  const k = angle / (2 * Math.sin(angle));
  return v.map((x) => x * k);
}

// 回転ベクトル → 回転行列(ロドリゲスの式)
export function expSO3(w) {
  const a = Math.hypot(...w);
  if (a < 1e-12) return [1, 0, 0, 0, 1, 0, 0, 0, 1];
  const [x, y, z] = w.map((v) => v / a);
  const c = Math.cos(a), s = Math.sin(a), C = 1 - c;
  return [c + x * x * C, x * y * C - z * s, x * z * C + y * s,
    y * x * C + z * s, c + y * y * C, y * z * C - x * s,
    z * x * C - y * s, z * y * C + x * s, c + z * z * C];
}

// 手先(gripperframe)の目標位置・向きから、関節の目標角度を出す。ロボット本体とは別の計算用モデルを持つ
export class Planner {
  // posW / rotW: 位置・向きの重み(元の mink の position_cost / orientation_cost と同じ意味)
  constructor(mj, model, { rotW = 0.3, posW = 1, jointIdx = [0, 1, 2, 3, 4] } = {}) {
    this.mj = mj;
    this.model = model;
    this.rotW = rotW;
    this.posW = posW;
    this.n = 5;
    this.nv = model.nv;
    this.siteId = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE.value, "gripperframe");
    this.jacp = new mj.DoubleBuffer(3 * model.nv);
    this.jacr = new mj.DoubleBuffer(3 * model.nv);
    this.kin = new mj.MjData(model);
    this.lo = Array.from({ length: 5 }, (_, i) => model.jnt_range[2 * i]);
    this.hi = Array.from({ length: 5 }, (_, i) => model.jnt_range[2 * i + 1]);
    this.q = [0, 0, 0, 0, 0];
    this.qRef = [0, 0, 0, 0, 0];
    this.forward();
    // 手先の目印(gripperframe)は x 軸が指の向き。真下を向かせる回転(x→-z)
    this.down = [0, 0, 1, 0, 1, 0, -1, 0, 0];
  }

  // 観測(度)に合わせる。余った自由度は「いまの姿勢のまま」という弱い目標で使う
  sync(obs) {
    this.q = MOTORS.slice(0, 5).map((m) => rad(obs[`${m}.pos`]));
    this.qRef = [...this.q];
    this.forward();
  }

  forward() {
    this.kin.qpos.set(this.q, 0);
    this.mj.mj_kinematics(this.model, this.kin);
    this.mj.mj_comPos(this.model, this.kin);
  }

  tcp() {
    const p = this.kin.site_xpos, i = 3 * this.siteId;
    return [p[i], p[i + 1], p[i + 2]];
  }

  tcpRot() { return Array.from(this.kin.site_xmat.slice(9 * this.siteId, 9 * this.siteId + 9)); }

  // 真下を向いて、鉛直軸まわりに yaw だけ回った手先の向き
  graspRot(yaw) { return mul3(rotZ(yaw), this.down); }

  // 手先を (pos, rot) へ近づける。iters 回の反復で解く。関節角度(度)を返す
  solve(pos, rot, iters = 3) {
    const { mj, model, kin, nv, siteId, n } = this;
    const w = [this.posW, this.posW, this.posW, this.rotW, this.rotW, this.rotW];
    for (let it = 0; it < iters; it++) {
      const xc = this.tcp(), Rc = this.tcpRot();
      const e = [...pos.map((t, i) => t - xc[i]), ...rotationError(rot, Rc)].map((v, r) => v * w[r]);
      mj.mj_jacSite(model, kin, this.jacp, this.jacr, siteId);
      const jp = this.jacp.GetView(), jr = this.jacr.GetView();
      const J = Array.from({ length: 6 }, (_, r) => Array.from({ length: n }, (_, c) => w[r] * (r < 3 ? jp[r * nv + c] : jr[(r - 3) * nv + c])));
      const A = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => {
        let s = i === j ? 1e-6 + 1e-3 : 0;
        for (let r = 0; r < 6; r++) s += J[r][i] * J[r][j];
        return s;
      }));
      const b = Array.from({ length: n }, (_, i) => {
        let s = 1e-6 * (this.qRef[i] - this.q[i]);
        for (let r = 0; r < 6; r++) s += J[r][i] * e[r];
        return s;
      });
      const dq = solve(A, b, n);
      this.q = this.q.map((q, i) => clamp(q + dq[i], this.lo[i], this.hi[i]));
      this.forward();
    }
    return this.q.map(deg);
  }

  dispose() { for (const o of [this.kin, this.jacp, this.jacr]) o?.delete(); }
}

// 手先と指を、いまの値から目標へなめらかに動かして robot に送る(1 ステップごとに yield)。
// 向きは回転の補間(R0・exp(u・log(R0⁻¹R1)))でなめらかにする。
export class Mover {
  constructor(robot, planner, { onStep = null } = {}) {
    this.robot = robot;
    this.pl = planner;
    this.onStep = onStep;
    this.sync();
  }

  sync() {
    const obs = this.robot.getObservation();
    this.pl.sync(obs);
    this.grip = obs["gripper.pos"];
  }

  *move(pos, rot, grip, dur, label = "") {
    const p0 = this.pl.tcp(), r0 = this.pl.tcpRot(), g0 = this.grip;
    const delta = logSO3(mul3(T3(r0), rot));
    const n = Math.max(1, Math.round(dur / this.robot.dt));
    for (let k = 1; k <= n; k++) {
      const u = smooth(k / n);
      const body = this.pl.solve(p0.map((p, i) => p + (pos[i] - p) * u), mul3(r0, expSO3(delta.map((d) => d * u))));
      const action = Object.fromEntries(MOTORS.slice(0, 5).map((m, i) => [`${m}.pos`, body[i]]));
      action["gripper.pos"] = g0 + (grip - g0) * u;
      this.step(action, label);
      yield label;
    }
    this.grip = grip;
  }

  // 関節角度(度)で動く(待機姿勢へ戻るときなど)
  *joints(degs, grip, dur, label = "") {
    const obs = this.robot.getObservation();
    const q0 = MOTORS.map((m) => obs[`${m}.pos`]);
    const q1 = [...degs];
    q1[5] = grip;
    const n = Math.max(1, Math.round(dur / this.robot.dt));
    for (let k = 1; k <= n; k++) {
      const u = smooth(k / n);
      this.step(Object.fromEntries(MOTORS.map((m, i) => [`${m}.pos`, q0[i] + (q1[i] - q0[i]) * u])), label);
      yield label;
    }
    this.sync();
  }

  step(action, label) {
    const sent = this.robot.sendAction(action);
    this.onStep?.(sent, label);
  }
}

// 指令と読み値のログ 1 行: { t, step, cmd_<motor>, obs_<motor> }
export function logRow(robot, sent, label, t) {
  const obs = robot.getObservation();
  const row = { t, step: label };
  for (const m of MOTORS) row[`cmd_${m}`] = sent[`${m}.pos`];
  for (const m of MOTORS) row[`obs_${m}`] = obs[`${m}.pos`];
  return row;
}

// 片付けの手順(pick_place.py の plan_and_run)。1 回の yield = 制御 1 ステップ
export function* planAndRun(robot, { cubeXY, cubeYaw, trayXY }, log) {
  const planner = new Planner(robot.mj, robot.model);
  const mv = new Mover(robot, planner, { onStep: (sent, label) => log.push(logRow(robot, sent, label, log.length / robot.config.fps)) });
  const [x, y] = cubeXY, [tx, ty] = trayXY;
  const yaw = ((((cubeYaw + Math.PI / 4) % (Math.PI / 2)) + Math.PI / 2) % (Math.PI / 2)) - Math.PI / 4; // キューブは 90° ごとに同じ
  // 手先は鉛直軸まわりに「肩の向き + キューブの向き」だけ回す(5 軸アームなので、肩の向きで決まる分がある)
  const face = (px, py) => Math.atan2(py, px);
  const steps = [["真上へ", [x, y, SAFE_Z], face(x, y) + yaw, GRIP_OPEN, 1.5],
    ["降りる", [x, y, GRASP_Z], face(x, y) + yaw, GRIP_OPEN, 1.0],
    ["つかむ", [x, y, GRASP_Z], face(x, y) + yaw, GRIP_CLOSED, 0.8],
    ["持ち上げる", [x, y, SAFE_Z], face(x, y) + yaw, GRIP_CLOSED, 1.0],
    ["トレイの上へ", [tx, ty, SAFE_Z], face(tx, ty), GRIP_CLOSED, 1.5],
    ["下ろす", [tx, ty, DROP_Z], face(tx, ty), GRIP_CLOSED, 0.6],
    ["はなす", [tx, ty, DROP_Z], face(tx, ty), GRIP_OPEN, 0.6],
    ["戻る", [0.20, 0.0, 0.15], 0.0, GRIP_OPEN, 1.5]];
  for (const [name, goal, gyaw, ggrip, dur] of steps) {
    yield* mv.move(goal, planner.graspRot(gyaw), ggrip, dur, name);
  }
  planner.dispose();
}

// 指令と実際の角度の差(追従誤差)を関節ごとに調べる: { motor: [最大, 平均] } [度]
export function checkLog(log) {
  const worst = {};
  for (const m of MOTORS.slice(0, 5)) {
    const err = log.map((r) => Math.abs(r[`cmd_${m}`] - r[`obs_${m}`]));
    worst[m] = [Math.max(...err), err.reduce((a, b) => a + b, 0) / err.length];
  }
  return worst;
}

export function logToCsv(log) {
  if (!log.length) return "";
  const keys = Object.keys(log[0]);
  return [keys.join(","), ...log.map((r) => keys.map((k) => (typeof r[k] === "number" ? +r[k].toFixed(5) : r[k])).join(","))].join("\n") + "\n";
}

