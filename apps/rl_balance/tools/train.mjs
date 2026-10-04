// 倒立振子を PPO で学習し、方策の重みと学習曲線を data/policy.json に書き出す(ブラウザでは学習せず、この結果を再生する)。
// Stable-Baselines3 の PPO(MlpPolicy)と同じ設定を、純 JS で書き直したもの。
//   node apps/rl_balance/tools/train.mjs [--steps 40960] [--seed 0]
// 同じ seed なら同じ結果になる(同じ Node・同じ MuJoCo WASM で)。
import { writeFile } from "node:fs/promises";
import { initMujoco } from "../../_web/mujoco.js";
import { BalanceSim, makeRng, mlpPolicy, evaluate, randomPolicy, MAX_STEPS } from "../sim.js";

const arg = (name, d) => { const i = process.argv.indexOf(`--${name}`); return i > 0 ? Number(process.argv[i + 1]) : d; };
const TOTAL = arg("steps", 40000);
const SEED = arg("seed", 0);

// SB3 の PPO の既定値
const N_STEPS = 2048, BATCH = 64, EPOCHS = 10, LR = 3e-4, GAMMA = 0.99, LAMBDA = 0.95, CLIP = 0.2, VF_COEF = 0.5, MAX_GRAD = 0.5;
const H = 64;

const rng = makeRng(SEED + 1000);

// ---- ネットワーク(4 → 64 → 64 → 1、tanh)と手書きの逆伝播 ----
function orthogonal(rows, cols, gain) {
  const m = Array.from({ length: rows }, () => Array.from({ length: cols }, () => rng.normal()));
  // 行数 >= 列数なら列を、そうでなければ行を、グラム・シュミットで直交化
  const vecs = rows >= cols ? Array.from({ length: cols }, (_, j) => m.map((r) => r[j])) : m;
  const out = [];
  for (const v of vecs) {
    const u = v.slice();
    for (const q of out) { let d = 0; for (let i = 0; i < u.length; i++) d += u[i] * q[i]; for (let i = 0; i < u.length; i++) u[i] -= d * q[i]; }
    const n = Math.hypot(...u);
    out.push(u.map((x) => x / n));
  }
  const w = new Float64Array(rows * cols);
  for (let i = 0; i < rows; i++) for (let j = 0; j < cols; j++) w[i * cols + j] = gain * (rows >= cols ? out[j][i] : out[i][j]);
  return w;
}

class Net {
  constructor(outGain) {
    this.p = {
      W1: orthogonal(H, 4, Math.SQRT2), b1: new Float64Array(H),
      W2: orthogonal(H, H, Math.SQRT2), b2: new Float64Array(H),
      W3: orthogonal(1, H, outGain), b3: new Float64Array(1),
    };
    this.g = Object.fromEntries(Object.entries(this.p).map(([k, v]) => [k, new Float64Array(v.length)]));
    this.a1 = new Float64Array(H); this.a2 = new Float64Array(H);
  }
  forward(x) {
    const { W1, b1, W2, b2, W3, b3 } = this.p, { a1, a2 } = this;
    for (let i = 0; i < H; i++) { let s = b1[i]; for (let j = 0; j < 4; j++) s += W1[i * 4 + j] * x[j]; a1[i] = Math.tanh(s); }
    for (let i = 0; i < H; i++) { let s = b2[i]; for (let j = 0; j < H; j++) s += W2[i * H + j] * a1[j]; a2[i] = Math.tanh(s); }
    let o = b3[0]; for (let j = 0; j < H; j++) o += W3[j] * a2[j];
    return o;
  }
  // forward(x) の直後に呼ぶ。出力への勾配 dout を勾配バッファへ足し込む
  backward(x, dout) {
    const { W2, W3 } = this.p, { a1, a2 } = this, g = this.g;
    g.b3[0] += dout;
    const gz2 = new Float64Array(H), gz1 = new Float64Array(H);
    for (let j = 0; j < H; j++) { g.W3[j] += dout * a2[j]; gz2[j] = dout * W3[j] * (1 - a2[j] * a2[j]); }
    for (let i = 0; i < H; i++) { g.b2[i] += gz2[i]; for (let j = 0; j < H; j++) { g.W2[i * H + j] += gz2[i] * a1[j]; gz1[j] += W2[i * H + j] * gz2[i]; } }
    for (let j = 0; j < H; j++) gz1[j] *= 1 - a1[j] * a1[j];
    for (let i = 0; i < H; i++) { g.b1[i] += gz1[i]; for (let j = 0; j < 4; j++) g.W1[i * 4 + j] += gz1[i] * x[j]; }
  }
  zero() { for (const v of Object.values(this.g)) v.fill(0); }
}

const actor = new Net(0.01), critic = new Net(1);
const logStd = { p: new Float64Array(1), g: new Float64Array(1) }; // 行動のばらつき(学習で変わる)

// Adam(SB3 と同じ eps=1e-5)
const params = [...Object.keys(actor.p).map((k) => [actor.p[k], actor.g[k]]), ...Object.keys(critic.p).map((k) => [critic.p[k], critic.g[k]]), [logStd.p, logStd.g]];
const adam = params.map(([p]) => ({ m: new Float64Array(p.length), v: new Float64Array(p.length) }));
let adamT = 0;
function adamStep() {
  let norm = 0;
  for (const [, g] of params) for (const x of g) norm += x * x;
  norm = Math.sqrt(norm);
  const scale = norm > MAX_GRAD ? MAX_GRAD / (norm + 1e-6) : 1;
  adamT++;
  const c1 = 1 - 0.9 ** adamT, c2 = 1 - 0.999 ** adamT;
  params.forEach(([p, g], k) => {
    const { m, v } = adam[k];
    for (let i = 0; i < p.length; i++) {
      const gi = g[i] * scale;
      m[i] = 0.9 * m[i] + 0.1 * gi; v[i] = 0.999 * v[i] + 0.001 * gi * gi;
      p[i] -= (LR * (m[i] / c1)) / (Math.sqrt(v[i] / c2) + 1e-5);
    }
  });
}

// ---- 学習 ----
const mujoco = await initMujoco();
const sim = await BalanceSim.create(mujoco, { seed: SEED });
const evalSim = await BalanceSim.create(mujoco, { seed: 0 });
const t0 = performance.now();

const snapshot = (net) => Object.fromEntries(Object.entries(net.p).map(([k, v]) => [k, Array.from(v, (x) => Math.round(x * 1e4) / 1e4)]));
const evalMean = (w, n = 10) => { const pol = mlpPolicy(w); let s = 0; for (let i = 0; i < n; i++) s += evaluate(evalSim, pol, 5000 + i); return s / n; };

let obs = Float64Array.from(sim.reset(SEED));
let numSteps = 0, epLen = 0;
const curve = { at: [], length: [], fell: [] }; // エピソードごと(終わった時点の経験量、立っていたステップ数、倒れたか)
const iters = [];                               // 更新ごとの方策のスナップショットと評価
const nIter = Math.ceil(TOTAL / N_STEPS);

for (let it = 0; it < nIter; it++) {
  const O = new Float64Array(N_STEPS * 4), A = new Float64Array(N_STEPS), LP = new Float64Array(N_STEPS);
  const R = new Float64Array(N_STEPS), V = new Float64Array(N_STEPS), D = new Uint8Array(N_STEPS);
  for (let t = 0; t < N_STEPS; t++) {
    O.set(obs, 4 * t);
    const mu = actor.forward(obs), std = Math.exp(logStd.p[0]);
    const a = mu + std * rng.normal();
    A[t] = a; V[t] = critic.forward(obs);
    LP[t] = -0.5 * ((a - mu) / std) ** 2 - logStd.p[0] - 0.5 * Math.log(2 * Math.PI);
    const r = sim.step(a); // 行動は -3〜3 に切ってから環境へ(記録は切る前の値)
    numSteps++; epLen++;
    R[t] = r.reward;
    if (r.truncated) R[t] += GAMMA * critic.forward(r.obs); // 時間切れは「倒れた」のではないので、先の価値を足す
    D[t] = r.terminated || r.truncated ? 1 : 0;
    if (D[t]) {
      curve.at.push(numSteps); curve.length.push(epLen); curve.fell.push(r.terminated ? 1 : 0);
      epLen = 0; obs = Float64Array.from(sim.reset());
    } else obs = Float64Array.from(r.obs);
  }
  // GAE(どの行動が思ったより良かったか)の計算
  const lastV = critic.forward(obs);
  const ADV = new Float64Array(N_STEPS), RET = new Float64Array(N_STEPS);
  let gae = 0;
  for (let t = N_STEPS - 1; t >= 0; t--) {
    const nextV = t === N_STEPS - 1 ? lastV : V[t + 1];
    const nonTerm = 1 - D[t];
    const delta = R[t] + GAMMA * nextV * nonTerm - V[t];
    gae = delta + GAMMA * LAMBDA * nonTerm * gae;
    ADV[t] = gae; RET[t] = gae + V[t];
  }
  // 同じデータで EPOCHS 回、ミニバッチごとに更新
  const idx = Array.from({ length: N_STEPS }, (_, i) => i);
  for (let ep = 0; ep < EPOCHS; ep++) {
    for (let i = N_STEPS - 1; i > 0; i--) { const j = Math.floor(rng.uniform() * (i + 1)); [idx[i], idx[j]] = [idx[j], idx[i]]; }
    for (let b = 0; b + BATCH <= N_STEPS; b += BATCH) {
      actor.zero(); critic.zero(); logStd.g[0] = 0;
      let mean = 0; for (let k = 0; k < BATCH; k++) mean += ADV[idx[b + k]]; mean /= BATCH;
      let sd = 0; for (let k = 0; k < BATCH; k++) sd += (ADV[idx[b + k]] - mean) ** 2; sd = Math.sqrt(sd / (BATCH - 1));
      const std = Math.exp(logStd.p[0]);
      for (let k = 0; k < BATCH; k++) {
        const i = idx[b + k], x = O.subarray(4 * i, 4 * i + 4);
        const adv = (ADV[i] - mean) / (sd + 1e-8);
        const mu = actor.forward(x);
        const lp = -0.5 * ((A[i] - mu) / std) ** 2 - logStd.p[0] - 0.5 * Math.log(2 * Math.PI);
        const ratio = Math.exp(lp - LP[i]);
        const active = adv >= 0 ? ratio <= 1 + CLIP : ratio >= 1 - CLIP; // クリップされていなければ勾配が流れる
        if (active) {
          const dlogp = (-adv * ratio) / BATCH;
          actor.backward(x, (dlogp * (A[i] - mu)) / (std * std));
          logStd.g[0] += dlogp * (((A[i] - mu) / std) ** 2 - 1);
        }
        const v = critic.forward(x);
        critic.backward(x, (VF_COEF * 2 * (v - RET[i])) / BATCH);
      }
      adamStep();
    }
  }
  const w = snapshot(actor);
  const score = evalMean(w);
  iters.push({ steps: numSteps, score, w });
  const recent = curve.length.slice(-10);
  console.log(`更新 ${String(it + 1).padStart(2)}/${nIter}  経験 ${numSteps} ステップ  エピソード ${curve.at.length} 回  直近 10 回の平均 ${(recent.reduce((a, b) => a + b, 0) / recent.length).toFixed(1)}  評価(10 回の平均) ${score.toFixed(1)}  std ${Math.exp(logStd.p[0]).toFixed(2)}`);
}
const seconds = (performance.now() - t0) / 1000;

// ---- 書き出し ----
const final = iters[iters.length - 1];
// 「学習の途中」: 評価が最初に 60 ステップ・250 ステップを超えた更新(どちらもまだ最後までは立てない)
const firstAbove = (th) => iters.find((x) => x.score >= th);
const mids = [];
const m1 = firstAbove(60), m2 = firstAbove(250);
for (const m of [m1, m2]) if (m && m !== final && !mids.includes(m)) mids.push(m);
const randomLens = Array.from({ length: 20 }, (_, s) => evaluate(evalSim, randomPolicy(s), s));
const randomMean = randomLens.reduce((a, b) => a + b, 0) / randomLens.length;
const firstFull = curve.length.findIndex((l) => l >= MAX_STEPS);

const out = {
  meta: {
    method: "PPO(Stable-Baselines3 の既定設定を純 JavaScript で書き直したもの)",
    seed: SEED, totalSteps: numSteps, hidden: H, episodes: curve.at.length,
    firstFullEpisode: firstFull >= 0 ? firstFull + 1 : null,
    randomMean, trainSeconds: Math.round(seconds),
  },
  curve,
  policies: [
    ...mids.map((m, i) => ({ id: `mid${i + 1}`, label: `学習の途中(${m.steps.toLocaleString("ja-JP")} ステップ時点)`, steps: m.steps, score: m.score, w: m.w })),
    { id: "final", label: `学習後(${final.steps.toLocaleString("ja-JP")} ステップ)`, steps: final.steps, score: final.score, w: final.w },
  ],
};
const json = JSON.stringify(out);
await writeFile(new URL("../data/policy.json", import.meta.url), json);
console.log(`学習時間 ${seconds.toFixed(0)} 秒 / data/policy.json ${(json.length / 1024).toFixed(0)} KB / エピソード ${curve.at.length} 回(倒れた ${curve.fell.reduce((a, b) => a + b, 0)} 回)/ 初めて 1000 ステップ立ったのは ${firstFull >= 0 ? firstFull + 1 : "なし"} 回目 / でたらめの平均 ${randomMean.toFixed(1)} ステップ`);
console.log("書き出した方策:", out.policies.map((p) => `${p.id}(評価 ${p.score.toFixed(1)})`).join(", "));
