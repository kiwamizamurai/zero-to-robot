// 倒立振子(Gymnasium の InvertedPendulum-v5 と同じ場面)を MuJoCo WASM で動かす環境と、学習済みの方策(MLP)の計算。
// ブラウザ(view.js)・Node の学習(tools/train.mjs)・テスト(test_sim.mjs)で同じこのファイルを使う。
import { writeText, bodyId } from "../_web/mujoco.js";

export const DT = 0.04;            // 1 ステップ = 0.04 秒(物理 0.02 秒 × frame_skip 2)
export const FRAME_SKIP = 2;
export const MAX_STEPS = 1000;     // 1000 ステップ(40 秒)立てたら成功として打ち切り
export const FALL_ANGLE = 0.2;     // 棒が 0.2 rad(約 11 度)以上傾いたら終わり
export const PUSH_STEPS = 3;       // 押す外乱は 3 ステップ(0.12 秒)つづける

// Gymnasium の inverted_pendulum.xml と同じ。床(見た目だけ)を足してある。
export const XML = `<mujoco model="inverted pendulum">
  <compiler angle="radian" autolimits="true"/>
  <option gravity="0 0 -9.81" integrator="RK4" timestep="0.02"/>
  <default>
    <joint damping="0.05"/>
    <geom contype="0" conaffinity="0" friction="1 0.1 0.1" rgba="0.7 0.7 0 1"/>
  </default>
  <worldbody>
    <geom name="floor" type="box" pos="0 0 -0.16" size="1.6 0.5 0.02" rgba="0.62 0.6 0.56 1"/>
    <geom name="rail" pos="0 0 0" quat="0.707 0 0.707 0" size="0.02 1" type="capsule" rgba="0.3 0.3 0.7 1"/>
    <body name="cart" pos="0 0 0">
      <joint name="slider" type="slide" axis="1 0 0" limited="true" range="-1 1" solreflimit=".08 1" damping="11"/>
      <geom name="cart" pos="0 0 0" quat="0.707 0 0.707 0" size="0.1 0.1" type="capsule"/>
      <body name="pole" pos="0 0 0">
        <joint name="hinge" type="hinge" axis="0 1 0" damping="0.05"/>
        <geom name="cpole" fromto="0 0 0 0.001 0 0.6" size="0.049 0.3" type="capsule" rgba="0 0.7 0.7 1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor ctrllimited="true" ctrlrange="-3 3" gear="100" joint="slider" name="slide"/>
  </actuator>
</mujoco>`;

// seed 固定の乱数(mulberry32)
export function makeRng(seed) {
  let a = seed >>> 0;
  const uniform = () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  const normal = () => Math.sqrt(-2 * Math.log(1 - uniform())) * Math.cos(2 * Math.PI * uniform());
  return { uniform, normal };
}

export class BalanceSim {
  static async create(mujoco, opts = {}) {
    writeText(mujoco, "/rl/model.xml", XML);
    const model = mujoco.MjModel.from_xml_path("/rl/model.xml");
    return new BalanceSim(mujoco, model, opts);
  }

  constructor(mujoco, model, { seed = 0 } = {}) {
    this.mj = mujoco;
    this.model = model;
    this.data = new mujoco.MjData(model);
    this.pole = bodyId(mujoco, model, "pole");
    this.rng = makeRng(seed);
    this.obs = new Float64Array(4);
    this.steps = 0;
    this.pushLeft = 0;
    this.pushForce = 0;
    this.result = null;
    this.reset();
  }

  // seed を渡すとその乱数から始める。省略すると前回の続き
  reset(seed) {
    if (seed !== undefined) this.rng = makeRng(seed);
    const { mj, model, data, rng } = this;
    mj.mj_resetData(model, data);
    // 初期状態に ±0.01 のゆらぎ(Gymnasium と同じ)
    data.qpos[0] = (rng.uniform() * 2 - 1) * 0.01;
    data.qpos[1] = (rng.uniform() * 2 - 1) * 0.01;
    data.qvel[0] = (rng.uniform() * 2 - 1) * 0.01;
    data.qvel[1] = (rng.uniform() * 2 - 1) * 0.01;
    mj.mj_forward(model, data);
    this.steps = 0; this.pushLeft = 0; this.pushForce = 0; this.result = null;
    this.#observe();
    return this.obs;
  }

  #observe() {
    const { qpos, qvel } = this.data;
    this.obs[0] = qpos[0]; this.obs[1] = qpos[1]; this.obs[2] = qvel[0]; this.obs[3] = qvel[1];
  }

  get time() { return this.steps * DT; }
  get tiltDeg() { return (this.data.qpos[1] * 180) / Math.PI; }

  // 棒の中ほどを横から force[N] で PUSH_STEPS ステップ押す(正が右)
  push(force) { this.pushForce = force; this.pushLeft = PUSH_STEPS; }
  get pushing() { return this.pushLeft > 0; }

  // 行動(-3〜3)を 1 つ与えて 1 ステップ進める。倒れたら terminated=true(報酬は 1 点/ステップ、倒れたステップは 0 点)
  step(action) {
    const { mj, model, data } = this;
    data.ctrl[0] = Math.max(-3, Math.min(3, action));
    data.xfrc_applied[6 * this.pole] = this.pushLeft > 0 ? this.pushForce : 0;
    if (this.pushLeft > 0) this.pushLeft--;
    for (let k = 0; k < FRAME_SKIP; k++) mj.mj_step(model, data);
    data.xfrc_applied[6 * this.pole] = 0;
    this.steps++;
    this.#observe();
    const o = this.obs;
    const terminated = !(Number.isFinite(o[0]) && Number.isFinite(o[1]) && Number.isFinite(o[2]) && Number.isFinite(o[3])) || Math.abs(o[1]) > FALL_ANGLE;
    return { obs: o, reward: terminated ? 0 : 1, terminated, truncated: !terminated && this.steps >= MAX_STEPS };
  }

  // 倒れるか時間切れまで、方策 policy(obs)->行動 で 1 ステップずつ進める(1 回の yield = 1 ステップ)。
  // 倒れたあとは、倒れていく様子が見えるように、力を加えずに少しだけ物理を進める。
  *run(policy) {
    while (true) {
      const { terminated, truncated } = this.step(policy(this.obs));
      yield;
      if (terminated || truncated) {
        this.result = { length: this.steps, fell: terminated };
        break;
      }
    }
    if (this.result.fell) {
      for (let k = 0; k < 20; k++) {
        this.data.ctrl[0] = 0;
        for (let j = 0; j < FRAME_SKIP; j++) this.mj.mj_step(this.model, this.data);
        yield;
      }
    }
  }

  dispose() { this.data.delete(); this.model.delete(); }
}

// でたらめに動かす方策(Python 版と同じく -3〜3 の一様乱数)
export function randomPolicy(seed) {
  const rng = makeRng(seed);
  return () => rng.uniform() * 6 - 3;
}

// 学習済みの方策(4 → 64 → 64 → 1 の tanh ネットワーク)。決定的に平均の行動を返す。
// w = { W1, b1, W2, b2, W3, b3 }(行列は 1 次元に並べたもの。W[i * 入力数 + j] が出力 i の入力 j への重み)
export function mlpPolicy(w, H = 64) {
  const h1 = new Float64Array(H), h2 = new Float64Array(H);
  return (obs) => {
    for (let i = 0; i < H; i++) {
      let s = w.b1[i];
      for (let j = 0; j < 4; j++) s += w.W1[i * 4 + j] * obs[j];
      h1[i] = Math.tanh(s);
    }
    for (let i = 0; i < H; i++) {
      let s = w.b2[i];
      for (let j = 0; j < H; j++) s += w.W2[i * H + j] * h1[j];
      h2[i] = Math.tanh(s);
    }
    let a = w.b3[0];
    for (let j = 0; j < H; j++) a += w.W3[j] * h2[j];
    return Math.max(-3, Math.min(3, a));
  };
}

// 方策を 1 回分(最大 1000 ステップ)走らせ、倒れるまでのステップ数(倒れたステップも数える。Python 版と同じ)を返す。pushAt/pushForce で途中に押せる
export function evaluate(sim, policy, seed, { pushAt = null, pushForce = 0 } = {}) {
  sim.reset(seed);
  while (true) {
    if (sim.steps === pushAt) sim.push(pushForce);
    const { terminated, truncated } = sim.step(policy(sim.obs));
    if (terminated || truncated) return sim.steps;
  }
}
