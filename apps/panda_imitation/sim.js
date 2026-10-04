// お手本から学ぶ(模倣学習)のシミュレーション(imitation.py の Env / teacher / rollout の移植)。
// Panda の台は ../_web/panda.js。ここでは「ブロック 1 個を箱に入れる」作業と、
// 先生(手順書)・学習済みの方策(MLP)が使う「状態」と「指令」の形を決める。
import { PandaCell, BLOCK, BOX_CENTER, BOX_HALF, BOX_HEIGHT, SAFE_Z, GRIP_OPEN, GRIP_CLOSED, N_ARM } from "../_web/panda.js";
import { mulberry32, gauss, smooth } from "../_web/math.js";

export { BLOCK, BOX_CENTER, BOX_HALF, BOX_HEIGHT };
export const PICK_AREA = [[0.38, 0.66], [-0.12, 0.32]];
export const EPISODE_SEC = 10.0;
export const CTRL_DT = 0.02; // 制御周期(物理 10 ステップ)
export const EPISODE_STEPS = Math.round(EPISODE_SEC / CTRL_DT);
export const NOISE_STD = 0.006; // お手本に混ぜる揺れの大きさ [m]
export const OBS_DIM = 12, ACT_DIM = 5;
const BLOCK_RGBA = [0.95, 0.80, 0.15, 1];
const GRASP_Z = BLOCK + 0.002, DROP_Z = BOX_HEIGHT + 0.06;

// 置き方(x, y, yaw)を seed から決める。rnd は mulberry32 の返す関数
export function randomPlacement(rnd) {
  const u = (a, b) => a + (b - a) * rnd();
  return [u(...PICK_AREA[0]), u(...PICK_AREA[1]), u(-Math.PI / 4, Math.PI / 4)];
}
export const placementFromSeed = (seed) => randomPlacement(mulberry32(seed));

// 先生の手順(panda_pick_place と同じ)を、制御周期ごとの (手先の目標, 手首の向き, 区間の番号) の列にする
export function teacherPlan(x, y, yaw, start) {
  const [bx, by] = BOX_CENTER;
  const segs = [[[x, y, SAFE_Z], yaw, 1.6], [[x, y, GRASP_Z], yaw, 1.0], [[x, y, GRASP_Z], yaw, 0.6], [[x, y, SAFE_Z], yaw, 1.0],
    [[bx, by, SAFE_Z], 0, 1.6], [[bx, by, DROP_Z], 0, 0.8], [[bx, by, DROP_Z], 0, 0.5], [[bx, by, SAFE_Z], 0, 0.8]];
  const plan = [];
  let p0 = [...start], y0 = 0;
  segs.forEach(([goal, yw, dur], i) => {
    const n = Math.round(dur / CTRL_DT);
    for (let k = 1; k <= n; k++) {
      const u = smooth(k / n);
      plan.push([p0.map((p, j) => p + (goal[j] - p) * u), y0 + (yw - y0) * u, i]);
    }
    p0 = goal; y0 = yw;
  });
  return plan;
}

// 先生の指の指令。手が実際につかむ高さ・はなす高さまで下りたら切り替える(一度切り替えたら戻さない)。
// 計画の時刻で切り替えると、直前と直後で状態がほとんど同じなのに指令だけが違い、真似する側が迷うため
export function teacherGrip(seg, tcpZ, closing) {
  if (seg === 0 || seg === 7) return false;
  if (seg === 1 || seg === 2) return closing || tcpZ < GRASP_Z + 0.006;
  if (seg === 3 || seg === 4) return true;
  return closing && tcpZ > DROP_Z + 0.01; // 5, 6: 箱へ降りきったら開く
}

export class ImitationSim extends PandaCell {
  // 毎回モデルを作り直さずに置き方だけ変える(学習用に大量に回すため)。reset は場面ごと作り直す
  reset({ seed = 1, placement = null } = {}) {
    this.placement = placement ?? placementFromSeed(seed);
    const [x, y, yaw] = this.placement;
    super.reset({ blocks: [{ name: "block", xy: [x, y], rgba: BLOCK_RGBA, yaw }], keyframe: "home" });
    this.homeQpos = Array.from(this.data.qpos.slice(0, 9));
    this.homeCtrl = Array.from(this.data.ctrl);
    this.lastCmd = new Float32Array(ACT_DIM); // 直近の指令(手先の動き 3・手首の回転・指を閉じるか)。画面に出す用
    this.down = Array.from(this.data.site_xmat.slice(9 * this.siteId, 9 * this.siteId + 9));
  }

  place(x, y, yaw) {
    const { mj, model, data } = this;
    this.placement = [x, y, yaw];
    mj.mj_resetData(model, data);
    data.qpos.set(this.homeQpos, 0);
    data.qpos.set([x, y, BLOCK, Math.cos(yaw / 2), 0, 0, Math.sin(yaw / 2)], 9);
    data.ctrl.set(this.homeCtrl);
    mj.mj_forward(model, data);
    this.ik.q = this.ik.qHome.slice();
    this.ik.forward();
    this.ik.yaw = 0;
    this.grip = GRIP_OPEN;
    this.steps = 0;
  }

  blockBody() { return this.data.body("block_block"); }
  blockXY() { const p = this.blockBody().xpos; return [p[0], p[1]]; }
  success() { return this.inBox(this.blockPos("block")); }

  // ブロックの向き。立方体なので 90° ごとに同じ → [-45°, 45°) にそろえる
  blockYaw() {
    const q = this.blockBody().xquat;
    const yaw = 2 * Math.atan2(q[3], q[0]);
    const m = (yaw + Math.PI / 4) % (Math.PI / 2);
    return (m < 0 ? m + Math.PI / 2 : m) - Math.PI / 4;
  }

  handYaw() {
    const M = this.data.site_xmat, o = 9 * this.siteId, D = this.down;
    const r00 = M[o] * D[0] + M[o + 1] * D[1] + M[o + 2] * D[2];
    const r10 = M[o + 3] * D[0] + M[o + 4] * D[1] + M[o + 5] * D[2];
    return Math.atan2(r10, r00);
  }

  // ネットに見せる状態(12 個の数): 手先の位置・指の開き・ブロックの位置と向き・手先から見たブロック・手首の向き
  obs(out = new Float32Array(OBS_DIM)) {
    const tcp = this.tcp(), b = this.blockBody().xpos, q = this.data.qpos;
    out.set(tcp, 0);
    out[3] = q[7] + q[8];
    out[4] = b[0]; out[5] = b[1]; out[6] = b[2];
    out[7] = b[0] - tcp[0]; out[8] = b[1] - tcp[1]; out[9] = b[2] - tcp[2];
    out[10] = this.blockYaw();
    out[11] = this.handYaw();
    return out;
  }

  act(target, yaw, close) {
    this.ik.yaw = yaw;
    this.grip = close ? GRIP_CLOSED : GRIP_OPEN;
    this.step(target);
  }

  // 先生(手順書)に片付けさせる。onSample(状態, 指令) に毎ステップの組を渡す。
  // noiseStd > 0 なら、ゆっくり変わる揺れで先生の手先をわざと道から外す(DART)。記録するのは外れていない先生の指令
  *runTeacher({ noiseStd = 0, rnd = Math.random, onSample = null } = {}) {
    const [x, y, yaw] = this.placement;
    const plan = teacherPlan(x, y, yaw, this.tcp());
    const drift = [0, 0, 0], a = 0.9, s = noiseStd * Math.sqrt(1 - a * a);
    let close = false;
    const cmd = this.lastCmd, obs = new Float32Array(OBS_DIM);
    for (const [target, yawCmd, seg] of plan) {
      const tcp = this.tcp();
      close = teacherGrip(seg, tcp[2], close);
      cmd[0] = target[0] - tcp[0]; cmd[1] = target[1] - tcp[1]; cmd[2] = target[2] - tcp[2];
      cmd[3] = yawCmd - this.handYaw(); cmd[4] = close ? 1 : 0;
      if (onSample) onSample(this.obs(obs), cmd);
      if (noiseStd > 0) {
        for (let i = 0; i < 3; i++) drift[i] = a * drift[i] + s * gauss(rnd);
        if (target[2] < 0.12) drift.fill(0); // つかむ・はなす瞬間の近くでは揺らさない
      }
      this.act([target[0] + drift[0], target[1] + drift[1], target[2] + drift[2]], yawCmd, close);
      yield;
    }
  }

  // 学習済みの方策だけで片付けさせる(手順書は使わない)。1 回 10 秒(500 ステップ)
  *runPolicy(policy, steps = EPISODE_STEPS) {
    const obs = new Float32Array(OBS_DIM);
    for (let k = 0; k < steps; k++) {
      const a = policy.act(this.obs(obs));
      this.lastCmd.set(a);
      const tcp = this.tcp();
      const target = [tcp[0] + a[0], tcp[1] + a[1], Math.max(tcp[2] + a[2], BLOCK)]; // 床より下は目指さない(安全のための制限)
      this.act(target, this.handYaw() + a[3], a[4] > 0.5);
      yield;
    }
  }
}
export { ImitationSim as default, N_ARM };
