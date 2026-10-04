// 減衰つき最小二乗法による逆運動学(mink.solve_ik の代わり)。
// 目的関数: 手先(site)を目標の位置・向きへ + 余った自由度は「ホーム姿勢に近く」という弱い目標で使う
// (01 の冗長マニピュレータの零空間)。1 ステップで誤差を消す gain = 1(dead-beat)。
import { rotationError, rotZ, mul3, solve } from "./math.js";

// mink は cost を二乗して目的関数に入れるので、Posture(cost=1e-2) の重みは 1e-4
const POSTURE_W = 1e-2 ** 2, DAMPING = 1e-3;

export class ArmIK {
  // q0: 初期の関節角(配列)。site の姿勢のうち、手先の向きの基準(down)を最初の姿勢から取る
  constructor(mj, model, { siteId, q0, jointRange = null, postureW = POSTURE_W, damping = DAMPING, posOnly = false }) {
    this.mj = mj;
    this.model = model;
    this.siteId = siteId;
    this.n = q0.length;
    this.nv = model.nv;
    this.qHome = [...q0];
    this.q = [...q0];
    this.postureW = postureW;
    this.damping = damping;
    this.posOnly = posOnly; // true なら向きは拘束しない(関節が足りないロボット用)
    this.lo = Array.from({ length: this.n }, (_, i) => (jointRange ? jointRange[i][0] : model.jnt_range[2 * i]));
    this.hi = Array.from({ length: this.n }, (_, i) => (jointRange ? jointRange[i][1] : model.jnt_range[2 * i + 1]));
    this.kin = new mj.MjData(model); // 「目標の関節角」での姿勢計算用(物理は進めない)
    this.jacp = new mj.DoubleBuffer(3 * model.nv);
    this.jacr = new mj.DoubleBuffer(3 * model.nv);
    this.yaw = 0; // 手先を真下に向けたまま、鉛直軸まわりに回す角度 [rad]
    this.forward();
    this.down = Array.from(this.kin.site_xmat.slice(9 * siteId, 9 * siteId + 9));
  }

  // kin のうち、モデルの先頭 n 関節だけ q を入れ、他(ブロックなど)は kin 側の qpos を使う
  syncOthers(data) {
    this.kin.qpos.set(data.qpos);
    this.kin.qpos.set(this.q, 0);
  }

  forward() {
    const { mj, model, kin } = this;
    kin.qpos.set(this.q, 0);
    mj.mj_kinematics(model, kin);
    mj.mj_comPos(model, kin);
  }

  target() { // 目標の関節角での手先位置
    const p = this.kin.site_xpos, i = 3 * this.siteId;
    return [p[i], p[i + 1], p[i + 2]];
  }

  // 手先を target_pos(と下向き + yaw の姿勢)へ近づける。q を更新して返す
  solve(targetPos) {
    const { mj, model, kin, nv, siteId, n } = this;
    const xc = this.target();
    const Rc = kin.site_xmat.slice(9 * siteId, 9 * siteId + 9);
    const Rt = this.yaw === 0 ? this.down : mul3(rotZ(this.yaw), this.down);
    const rows = this.posOnly ? 3 : 6;
    const e = [...targetPos.map((t, i) => t - xc[i]), ...(this.posOnly ? [] : rotationError(Rt, Rc))];
    mj.mj_jacSite(model, kin, this.jacp, this.jacr, siteId);
    const jp = this.jacp.GetView(), jr = this.jacr.GetView();
    const J = Array.from({ length: rows }, (_, r) => Array.from({ length: n }, (_, c) => (r < 3 ? jp[r * nv + c] : jr[(r - 3) * nv + c])));
    const A = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => {
      let s = i === j ? this.postureW + this.damping : 0;
      for (let r = 0; r < rows; r++) s += J[r][i] * J[r][j];
      return s;
    }));
    const b = Array.from({ length: n }, (_, i) => {
      let s = this.postureW * (this.qHome[i] - this.q[i]);
      for (let r = 0; r < rows; r++) s += J[r][i] * e[r];
      return s;
    });
    const dq = solve(A, b, n);
    this.q = this.q.map((q, i) => Math.min(Math.max(q + dq[i], this.lo[i]), this.hi[i]));
    this.forward();
    return this.q;
  }

  dispose() {
    for (const o of [this.kin, this.jacp, this.jacr]) o?.delete();
  }
}
