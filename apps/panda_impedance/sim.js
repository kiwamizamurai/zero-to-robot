// 力を加減してなぞるシミュレーション(impedance.py の移植)。
// Panda の台は ../_web/panda.js を土台にし、モーターを「トルク指令」に差し替えてペンと机を足す。
import { mountFiles } from "../_web/mujoco.js";
import { PandaCell, PANDA_FILES, N_PHYSICS } from "../_web/panda.js";
import { smooth } from "../_web/math.js";
import { solve } from "../_web/math.js";

export const TABLE_Z = 0.20; // 机の上面の高さの見込み [m]
export const CENTER = [0.50, 0.0], RADIUS = 0.08;
export const PEN_R = 0.012;
export const MODES = {
  stiff: { name: "硬い制御(位置制御に近い)", kz: 8000, depth: 0.00125 },
  soft: { name: "柔らかい制御(インピーダンス制御)", kz: 400, depth: 0.025 },
};
export const FORCE_GOAL = 10;
export const T_MOVE = 2.0, T_DOWN = 1.5, T_CIRCLE = 6.0, T_HOLD = 0.5;
export const T_END = T_MOVE + T_DOWN + T_CIRCLE + T_HOLD;
const HAND_PEN_Z = 0.1034;

// panda.xml を書き換える。(1) モーター 1〜7 を位置サーボからトルク直接指令に (2) 手先にペン (3) 接触の円錐を楕円に
function patchPandaXml(xml) {
  const motors = [1, 2, 3, 4, 5, 6, 7].map((i) => {
    const lim = i >= 5 ? 12 : 87;
    return `<general class="panda" name="actuator${i}" joint="joint${i}" gainprm="1" biastype="none" biasprm="0 0 0" ctrlrange="-${lim} ${lim}"${i >= 5 ? ' forcerange="-12 12"' : ""}/>`;
  }).join("\n    ");
  xml = xml.replace(/<general class="panda" name="actuator[1-7]"[^>]*\/>/g, (m, off, s) => (m.includes('name="actuator1"') ? motors : ""));
  xml = xml.replace('<option integrator="implicitfast"/>', '<option integrator="implicitfast" cone="elliptic"/>');
  const pen = `<geom name="pen" type="capsule" size="${PEN_R} 0.04" pos="0 0 ${HAND_PEN_Z + 0.04}" rgba="0.15 0.15 0.2 1" friction="0.05 0.005 0.0001" mass="0.05"/>
                      <site name="pen_tip" pos="0 0 ${HAND_PEN_Z + 0.08 + PEN_R}"/>`;
  return xml.replace('<site name="tcp" pos="0 0 0.1034"/>', `<site name="tcp" pos="0 0 0.1034"/>\n${pen}`);
}

const tableXml = (offset) => {
  const top = TABLE_Z + offset;
  return `<body name="table" pos="0.5 0 ${top / 2}"><geom name="table" type="box" size="0.25 0.3 ${top / 2}" rgba="0.93 0.93 0.9 1" friction="0.05 0.005 0.0001"/></body>`;
};

export class ImpedanceSim extends PandaCell {
  static async create(mujoco, load, opts = {}) {
    await mountFiles(mujoco, "/panda", PANDA_FILES, (f) => load(`assets/panda/${f}`));
    const text = new TextDecoder().decode(mujoco.FS.readFile("/panda/panda.xml"));
    mujoco.FS.writeFile("/panda/panda.xml", patchPandaXml(text));
    return new this(mujoco, opts);
  }

  // mode: "stiff" | "soft"、offset: 机の高さのずれ [m]
  reset({ mode = "soft", offset = 0 } = {}) {
    super.reset({ blocks: [], box: false, extraWorldXml: tableXml(offset) });
    this.mode = mode; this.offset = offset;
    const m = this.model, mj = this.mj;
    this.penSite = mj.mj_name2id(m, mj.mjtObj.mjOBJ_SITE.value, "pen_tip");
    this.penGeom = mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM.value, "pen");
    this.tableGeom = mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM.value, "table");
    this.jp = new mj.DoubleBuffer(3 * m.nv);
    this.jr = new mj.DoubleBuffer(3 * m.nv);
    this.f6 = new mj.DoubleBuffer(6);
    this.qHome = Array.from(this.data.qpos.slice(0, 7));
    this.Rd = null;
    const kz = MODES[mode].kz;
    this.K = [1500, 1500, kz];
    this.D = [2 * Math.sqrt(1500 * 2), 2 * Math.sqrt(1500 * 2), 2 * Math.sqrt(kz * 2)];
    this.Kr = 60; this.Dr = 4;
    this.start = this.pen();
    this.history = []; // { t, f(生の力), drawing, z }
    this.ink = [];     // 触れた点 [x, y]
    this.press = 0;    // 直近 10 ステップの平均の力(画面表示用)
    this.recent = [];
  }

  dispose() {
    this.jp?.delete(); this.jr?.delete(); this.f6?.delete();
    this.jp = this.jr = this.f6 = null;
    super.dispose();
  }

  pen() {
    const p = this.data.site_xpos, i = 3 * this.penSite;
    return [p[i], p[i + 1], p[i + 2]];
  }

  // 目標の位置から関節トルクを出す。τ = Jᵀ[F; M] + 零空間の姿勢保持 + qfrc_bias
  torque(goal) {
    const { mj, model: m, data: d } = this;
    mj.mj_jacSite(m, d, this.jp, this.jr, this.penSite);
    const jp = this.jp.GetView(), jr = this.jr.GetView(), nv = m.nv;
    const J = Array.from({ length: 6 }, (_, r) => Array.from({ length: 7 }, (_, c) => (r < 3 ? jp[r * nv + c] : jr[(r - 3) * nv + c])));
    const qd = Array.from(d.qvel.slice(0, 7));
    const x = this.pen();
    const R = Array.from(d.site_xmat.slice(9 * this.penSite, 9 * this.penSite + 9));
    if (!this.Rd) this.Rd = [...R];
    const Jq = J.map((row) => row.reduce((s, a, c) => s + a * qd[c], 0)); // [v; w]
    const F = [0, 1, 2].map((i) => this.K[i] * (goal[i] - x[i]) - this.D[i] * Jq[i]);
    // E = 0.5 (Rdᵀ R − Rᵀ Rd)、eR = R [E21, E02, E10]
    const Rd = this.Rd, E = (i, j) => {
      let a = 0, b = 0;
      for (let k = 0; k < 3; k++) { a += Rd[k * 3 + i] * R[k * 3 + j]; b += R[k * 3 + i] * Rd[k * 3 + j]; }
      return 0.5 * (a - b);
    };
    const ev = [E(2, 1), E(0, 2), E(1, 0)];
    const eR = [0, 1, 2].map((i) => R[i * 3] * ev[0] + R[i * 3 + 1] * ev[1] + R[i * 3 + 2] * ev[2]);
    const M = [0, 1, 2].map((i) => -this.Kr * eR[i] - this.Dr * Jq[3 + i]);
    const W = [...F, ...M];
    const tau = Array.from({ length: 7 }, (_, c) => J.reduce((s, row, r) => s + row[c] * W[r], 0));
    // 零空間: (I − Jᵀ J⁺ᵀ) u,  J⁺ᵀ = (J Jᵀ)⁻¹ J
    const u = qd.map((v, i) => 10 * (this.qHome[i] - d.qpos[i]) - 2 * v);
    const Ju = J.map((row) => row.reduce((s, a, c) => s + a * u[c], 0));
    const A = J.map((ri) => J.map((rj) => ri.reduce((s, a, c) => s + a * rj[c], 0) + 1e-9));
    const y = solve(A, Ju, 6);
    return tau.map((t, c) => t + u[c] - J.reduce((s, row, r) => s + row[c] * y[r], 0) + d.qfrc_bias[c]);
  }

  // ペンと机の接触力(机を押す向きの成分)と接触点
  penForce() {
    const { mj, model: m, data: d } = this;
    let total = 0, point = null;
    for (let i = 0; i < d.ncon; i++) {
      const c = d.contact.get(i);
      const g1 = c.geom1, g2 = c.geom2;
      if ((g1 === this.penGeom && g2 === this.tableGeom) || (g2 === this.penGeom && g1 === this.tableGeom)) {
        mj.mj_contactForce(m, d, i, this.f6);
        total += this.f6.GetView()[0];
        point = [c.pos[0], c.pos[1]];
      }
      c.delete?.();
    }
    return { f: total, point };
  }

  goalAt(t) {
    const { depth } = MODES[this.mode];
    const above = [CENTER[0] + RADIUS, CENTER[1], TABLE_Z + 0.05];
    const press = [CENTER[0] + RADIUS, CENTER[1], TABLE_Z - depth];
    const lerp = (a, b, u) => a.map((v, i) => v + (b[i] - v) * u);
    if (t < T_MOVE) return lerp(this.start, above, smooth(t / T_MOVE));
    if (t < T_MOVE + T_DOWN) return lerp(above, press, smooth((t - T_MOVE) / T_DOWN));
    if (t < T_MOVE + T_DOWN + T_CIRCLE) {
      const a = 2 * Math.PI * smooth((t - T_MOVE - T_DOWN) / T_CIRCLE);
      return [CENTER[0] + RADIUS * Math.cos(a), CENTER[1] + RADIUS * Math.sin(a), TABLE_Z - depth];
    }
    return press;
  }

  // 物理 1 ステップ(2 ms)ごとにトルクを計算して進める(元の Python と同じ)
  tick() {
    const { model: m, data: d } = this;
    const t = d.time;
    const tau = this.torque(this.goalAt(t));
    for (let i = 0; i < 7; i++) d.ctrl[i] = Math.min(Math.max(tau[i], m.actuator_ctrlrange[2 * i]), m.actuator_ctrlrange[2 * i + 1]);
    d.ctrl[7] = 0;
    this.mj.mj_step(m, d);
    const { f, point } = this.penForce();
    const drawing = t >= T_MOVE + T_DOWN && t < T_MOVE + T_DOWN + T_CIRCLE;
    if (point && f > 0.5) this.ink.push(point);
    this.history.push({ t, f, drawing, z: this.pen()[2] });
    this.recent = [...this.recent, f].slice(-10);
    this.press = this.recent.reduce((a, b) => a + b, 0) / this.recent.length;
  }

  // 1 回の yield = 制御 N_PHYSICS ステップ(0.02 秒)
  *run() {
    while (this.data.time < T_END) {
      for (let k = 0; k < N_PHYSICS; k++) this.tick();
      yield;
    }
  }

  // 20 ms(10 ステップ)の移動平均(力センサーと同じ見方)。numpy の convolve(mode="same") と同じ中央そろえ
  smoothForce(n = 10) {
    const f = this.history.map((r) => r.f), N = f.length, out = new Array(N).fill(0);
    const half = Math.floor(n / 2);
    for (let i = 0; i < N; i++) {
      let s = 0;
      for (let k = 0; k < n; k++) { const j = i + k - (n - 1 - half); if (j >= 0 && j < N) s += f[j]; }
      out[i] = s / n;
    }
    return out;
  }

  summary() {
    const fs = this.smoothForce();
    const f = fs.filter((_, i) => this.history[i].drawing);
    const mean = f.reduce((a, b) => a + b, 0) / Math.max(f.length, 1);
    const max = f.length ? Math.max(...f) : 0;
    const touching = (f.filter((v) => v > 0.5).length / Math.max(f.length, 1)) * 100;
    let covered = 0;
    for (let k = 0; k < 360; k++) {
      const a = (2 * Math.PI * k) / 360, cx = CENTER[0] + RADIUS * Math.cos(a), cy = CENTER[1] + RADIUS * Math.sin(a);
      if (this.ink.some((p) => Math.hypot(p[0] - cx, p[1] - cy) < 0.006)) covered++;
    }
    return { mean, max, touching, covered: (covered / 360) * 100 };
  }
}
export { ImpedanceSim as default };
