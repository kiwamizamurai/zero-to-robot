// ドローンを飛ばすシミュレーション(drone.py の移植)。
// 位置の制御 → 姿勢の制御 → ミキサー の 3 段のカスケード制御を、物理 1 ステップ(0.01 秒)ごとに回す。
import { mountFiles, writeText, siteId, bodyId } from "../_web/mujoco.js";
import { solve } from "../_web/math.js";

export const G = 9.81;
// 巡回する点 [x, y, z, 機首の向き(度)] と、そこまでにかける時間 [秒]
export const WAYPOINTS = [
  [[0, 0, 1.0, 0], 2.5], [[1.2, 0, 1.0, 0], 2.5], [[1.2, 1.2, 1.4, 90], 3.0],
  [[0, 1.2, 1.0, 90], 2.5], [[0, 0, 1.0, 90], 2.5], [[0, 0, 1.0, 90], 2.5], [[0, 0, 0.12, 90], 3.0],
];
export const GUST = { start: 14.0, end: 15.0, force: [0, 5, 0] }; // 14〜15 秒に横から 5 N
export const T_TOTAL = WAYPOINTS.reduce((s, [, T]) => s + T, 0) + 1.0;
export const WEAK_GAIN = 0.1; // 姿勢の制御を弱くした版のゲイン倍率
const X2_FILES = ["x2.xml", "assets/X2_lowpoly.obj"];

const sceneXml = () => `<mujoco model="x2 scene">
  <include file="x2.xml"/>
  <statistic meansize=".05"/>
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" rgba="0.82 0.84 0.86 1"/>
    ${WAYPOINTS.slice(0, 5).map(([[x, y, z]]) => `<geom type="sphere" size="0.04" pos="${x} ${y} ${z}" rgba="1 0.6 0.1 0.8" contype="0" conaffinity="0"/>`).join("\n    ")}
  </worldbody>
</mujoco>`;

// 5 次多項式(05_planning): 位置・速度・加速度。両端で速度も加速度も 0
function minJerk(p0, p1, T, t) {
  const s = Math.min(Math.max(t / T, 0), 1), d = p1 - p0;
  return [p0 + d * (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5),
    d * (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) / T,
    d * (60 * s - 180 * s ** 2 + 120 * s ** 3) / T ** 2];
}

// 時刻 t の目標(位置・速度・加速度・機首の向き)
export function reference(t, start) {
  let p0 = [...start], y0 = 0, t0 = 0;
  for (const [wp, T] of WAYPOINTS) {
    const p1 = wp.slice(0, 3), y1 = (wp[3] * Math.PI) / 180;
    if (t < t0 + T) {
      const c = [0, 1, 2].map((i) => minJerk(p0[i], p1[i], T, t - t0));
      return { pos: c.map((x) => x[0]), vel: c.map((x) => x[1]), acc: c.map((x) => x[2]), yaw: minJerk(y0, y1, T, t - t0)[0] };
    }
    p0 = p1; y0 = y1; t0 += T;
  }
  return { pos: p0, vel: [0, 0, 0], acc: [0, 0, 0], yaw: y0 };
}

const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => Math.hypot(...a);

export class DroneSim {
  static async create(mujoco, load, opts = {}) {
    await mountFiles(mujoco, "/x2", X2_FILES, (f) => load(`assets/skydio_x2/${f}`));
    return new this(mujoco, opts);
  }

  constructor(mujoco, opts = {}) {
    this.mj = mujoco;
    writeText(mujoco, "/x2/scene.xml", sceneXml());
    this.model = mujoco.MjModel.from_xml_path("/x2/scene.xml");
    this.data = new mujoco.MjData(this.model);
    const m = this.model;
    this.body = bodyId(mujoco, m, "x2");
    this.mass = m.body_subtreemass[this.body];
    this.I = Array.from(m.body_inertia.slice(3 * this.body, 3 * this.body + 3));
    this.fmax = Array.from({ length: 4 }, (_, i) => m.actuator_ctrlrange[2 * i + 1]);
    this.thrustSites = [1, 2, 3, 4].map((i) => siteId(mujoco, m, `thrust${i}`));
    this.reset(opts);
  }

  // weak: 姿勢の制御を弱くした版
  reset({ weak = false } = {}) {
    const { mj, model: m, data: d } = this;
    this.weak = weak;
    mj.mj_resetData(m, d);
    const key = m.key("hover");
    d.qpos.set(key.qpos);
    d.ctrl.set(key.ctrl);
    d.qpos[2] = 0.12; // 地面から離陸する
    d.xfrc_applied.fill(0);
    mj.mj_forward(m, d);
    const g = weak ? WEAK_GAIN : 1;
    // 外側(位置)はゆっくり、内側(姿勢)は速く。内側が外側より十分速いのがカスケードの前提
    this.kp = [12, 12, 14]; this.kd = [6, 6, 7];
    this.kR = [60 * g, 60 * g, 15 * g]; this.kw = [12 * g, 12 * g, 5 * g];
    this.mixerInv = this.buildMixer();
    this.start = Array.from(d.qpos.slice(0, 3));
    this.log = [];
    this.manualGust = null; // { until, force }
    this.crashed = false; this.finished = false;
    this.f = [0, 0, 0, 0];
  }

  // 4 つの推力 f → (全体の推力, x・y・z 軸まわりのトルク) の行列 M を作り、その逆行列を返す
  buildMixer() {
    const { model: m, data: d } = this;
    const com = d.subtree_com.slice(3 * this.body, 3 * this.body + 3);
    const M = [[], [], [], []];
    this.thrustSites.forEach((s, i) => {
      const r = [0, 1, 2].map((k) => d.site_xpos[3 * s + k] - com[k]);
      const yawCoeff = m.actuator_gear[6 * i + 5]; // プロペラの回転の反作用で生まれる、z 軸まわりのトルク
      [1, r[1], -r[0], yawCoeff].forEach((v, row) => { M[row][i] = v; });
    });
    // 逆行列(単位ベクトルごとに連立方程式を解く)
    const cols = [0, 1, 2, 3].map((j) => solve(M, [0, 1, 2, 3].map((k) => (k === j ? 1 : 0)), 4));
    return [0, 1, 2, 3].map((i) => [0, 1, 2, 3].map((j) => cols[j][i]));
  }

  get time() { return this.data.time; }
  pos() { return Array.from(this.data.qpos.slice(0, 3)); }

  tiltDeg() {
    const R22 = this.data.xmat[9 * this.body + 8];
    return (Math.acos(Math.min(Math.max(R22, -1), 1)) * 180) / Math.PI;
  }

  // 3 段のカスケード制御。4 つのプロペラの推力を返す
  control(ref) {
    const d = this.data;
    const p = d.qpos, v = d.qvel;
    const R = Array.from(d.xmat.slice(9 * this.body, 9 * this.body + 9));
    const w = [v[3], v[4], v[5]]; // 機体座標の角速度(MuJoCo の自由関節は回転速度を機体座標で持つ)
    // 1. 位置の制御: 欲しい加速度(重力を打ち消す分も足す)
    const a = [0, 1, 2].map((i) => ref.acc[i] + this.kp[i] * (ref.pos[i] - p[i]) + this.kd[i] * (ref.vel[i] - v[i]) + (i === 2 ? G : 0));
    const thrust = this.mass * (a[0] * R[2] + a[1] * R[5] + a[2] * R[8]);
    // 2. 姿勢の制御: 機体の上向き(z 軸)を a の向きにそろえ、機首を yaw に向ける
    const na = norm(a), zb = a.map((x) => x / na);
    const xc = [Math.cos(ref.yaw), Math.sin(ref.yaw), 0];
    let yb = cross(zb, xc);
    const ny = norm(yb); yb = yb.map((x) => x / ny);
    const xb = cross(yb, zb);
    const Rd = [xb[0], yb[0], zb[0], xb[1], yb[1], zb[1], xb[2], yb[2], zb[2]]; // 列が xb, yb, zb
    const E = (i, j) => { // 0.5 (Rdᵀ R − Rᵀ Rd)
      let s = 0;
      for (let k = 0; k < 3; k++) s += Rd[k * 3 + i] * R[k * 3 + j] - R[k * 3 + i] * Rd[k * 3 + j];
      return 0.5 * s;
    };
    const eR = [E(2, 1), E(0, 2), E(1, 0)]; // 向きのずれ(回転行列の差から取り出す)
    const Iw = this.I.map((x, i) => x * w[i]);
    const gyro = cross(w, Iw);
    const torque = [0, 1, 2].map((i) => this.I[i] * (-this.kR[i] * eR[i] - this.kw[i] * w[i]) + gyro[i]);
    // 3. ミキサー: 4 つのプロペラの推力に配分する(出せる範囲に切る)
    const u = [thrust, ...torque];
    return this.mixerInv.map((row, i) => Math.min(Math.max(row.reduce((s, c, k) => s + c * u[k], 0), 0), this.fmax[i]));
  }

  // 手動の突風(force [N] を duration 秒だけ)
  gust(force, duration = 1.0) {
    this.manualGust = { until: this.time + duration, force };
  }

  gustForce(t) {
    if (t >= GUST.start && t < GUST.end) return GUST.force;
    if (this.manualGust && t < this.manualGust.until) return this.manualGust.force;
    return null;
  }

  // 物理 1 ステップ(0.01 秒)進める。終わり(最後まで飛んだ/墜落)なら false
  step() {
    if (this.finished) return false;
    const { mj, model: m, data: d } = this;
    const t = d.time;
    const ref = reference(t, this.start);
    const f = this.control(ref);
    for (let i = 0; i < 4; i++) d.ctrl[i] = f[i];
    this.f = f;
    const gf = this.gustForce(t), o = 6 * this.body;
    for (let k = 0; k < 3; k++) d.xfrc_applied[o + k] = gf ? gf[k] : 0;
    mj.mj_step(m, d);
    const pos = this.pos(), tilt = this.tiltDeg();
    const err = Math.hypot(ref.pos[0] - pos[0], ref.pos[1] - pos[1], ref.pos[2] - pos[2]);
    this.log.push({ t, ref: ref.pos, pos, f, tilt, err, gust: !!gf });
    if (tilt > 80 || pos[2] < 0.03) { this.crashed = true; this.finished = true; return false; }
    if (d.time >= T_TOTAL) { this.finished = true; return false; }
    return true;
  }

  *run() {
    while (this.step()) yield;
  }

  // 結果の数値(README の主張に沿う: 平均ずれ、突風を受けたときの最大のずれと傾き、着陸位置)
  results() {
    const L = this.log;
    if (!L.length) return null;
    const n = L.length, mean = L.reduce((s, r) => s + r.err, 0) / n;
    const out = {
      crashed: this.crashed, time: L[n - 1].t, meanErr: mean * 100, maxErr: Math.max(...L.map((r) => r.err)) * 100,
      landing: L[n - 1].pos, gustErr: null, gustTilt: null,
    };
    const g = L.filter((r) => r.t >= GUST.start && r.t < GUST.start + 2.5);
    if (g.length && !this.crashed) {
      out.gustErr = Math.max(...g.map((r) => r.err)) * 100;
      out.gustTilt = Math.max(...g.map((r) => r.tilt));
    }
    return out;
  }

  dispose() {
    this.data?.delete();
    this.model?.delete();
    this.model = this.data = null;
  }
}
