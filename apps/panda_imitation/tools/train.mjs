// お手本集め → 学習 → 評価を Node で行い、学習済みの重みと成功率の表を data/ に書き出す。
//   node apps/panda_imitation/tools/train.mjs            # 全部(数分)
//   node apps/panda_imitation/tools/train.mjs --quick    # 小さな設定で動作確認
// ブラウザはここで作った data/*.json を読んで再生するだけ(ブラウザの中では学習しない)。
import { Worker } from "node:worker_threads";
import { writeFileSync, statSync } from "node:fs";
import { cpus } from "node:os";
import { randomPlacement, placementFromSeed } from "../sim.js";
import { mulberry32 } from "../../_web/math.js";

const quick = process.argv.includes("--quick");
const N_MAX = quick ? 40 : 200;
const SIZES = process.env.SIZES ? process.env.SIZES.split(",").map(Number) : quick ? [10, 20, 40] : [10, 25, 50, 100, 200];
const SEEDS = (process.env.SEEDS ?? (quick ? "0" : "0,1,2")).split(",").map(Number); // 学習の乱数の seed(複数なら成功率は平均)
const DRY = quick || process.argv.includes("--dry"); // true なら data/ に書かない(条件を試すとき)
const N_EVAL = quick ? 20 : 100;
const HIDDEN = Number(process.env.HIDDEN ?? 128), EPOCHS = Number(process.env.EPOCHS ?? 40), BATCH = Number(process.env.BATCH ?? 256);
const SHIPPED = DRY ? [] : [[10, "noise"], [50, "noise"], [200, "noise"], [200, "clean"]]; // 配信する方策(他は成功率の表だけ)
const dataDir = new URL("../data/", import.meta.url);

// ---- 作業者のプール ----
const nWorkers = Math.max(2, Math.min(cpus().length - 1, 10));
const workers = [], idle = [], queue = [], pending = new Map();
let nextId = 0;
const ready = Promise.all(Array.from({ length: nWorkers }, () => new Promise((res) => {
  const w = new Worker(new URL("./worker.mjs", import.meta.url));
  w.on("message", (m) => {
    if (m.ready) { idle.push(w); res(); return; }
    const p = pending.get(m.id);
    pending.delete(m.id);
    idle.push(w);
    m.error ? p.reject(new Error(m.error)) : p.resolve(m.result);
    pump();
  });
  w.on("error", (e) => { console.error(e); process.exit(1); });
  workers.push(w);
})));
function pump() {
  while (idle.length && queue.length) {
    const w = idle.pop(), job = queue.shift();
    pending.set(job.id, job);
    w.postMessage({ id: job.id, type: job.type, args: job.args });
  }
}
const run = (type, args) => new Promise((resolve, reject) => { queue.push({ id: nextId++, type, args, resolve, reject }); pump(); });
const chunks = (a, k) => Array.from({ length: k }, (_, i) => a.filter((_, j) => j % k === i));
const sec = (t0) => `${((performance.now() - t0) / 1000).toFixed(0)} 秒`;

await ready;
const T0 = performance.now();
const rng = mulberry32(0);
// 学習用の置き方(4 つ目は DART の揺れの seed。置き方ごとに別の揺れ)
const trainPlaces = Array.from({ length: N_MAX }, (_, i) => [...randomPlacement(rng), 1000 + i]);
const evalPlaces = Array.from({ length: N_EVAL }, (_, i) => placementFromSeed(1000 + i)); // 学習に使わない置き方

// 1. お手本を集める(先生に片付けさせて、毎ステップの(状態, 指令)を記録)
async function collect(noisy) {
  const t0 = performance.now();
  const parts = await Promise.all(chunks(trainPlaces, nWorkers * 2).map((placements) => run("collect", { placements, noisy })));
  // 置き方の順(= お手本の数を増やしたときの先頭からの部分集合)に戻すため、1 回ずつの組の数で並べ直す
  const per = parts.map((p, i) => ({ i, n: chunks(trainPlaces, nWorkers * 2)[i].length, p }));
  const eps = [];
  for (let j = 0; j < N_MAX; j++) {
    const c = j % (nWorkers * 2), k = Math.floor(j / (nWorkers * 2)), part = per[c];
    const len = part.p.Y.length / 5 / part.n; // 1 回のお手本の組の数(どの置き方でも同じ長さ)
    eps.push([part.p.X.subarray(k * len * 12, (k + 1) * len * 12), part.p.Y.subarray(k * len * 5, (k + 1) * len * 5)]);
  }
  const ok = parts.reduce((s, p) => s + p.ok, 0);
  console.log(`1. お手本(揺れ${noisy ? "あり" : "なし"}) ${N_MAX} 回を集めた: ${eps.length * eps[0][1].length / 5} 組、先生の成功 ${ok}/${N_MAX} [${sec(t0)}]`);
  return { eps, ok };
}
const join = (eps, n) => {
  const X = new Float32Array(n * eps[0][0].length), Y = new Float32Array(n * eps[0][1].length);
  for (let i = 0; i < n; i++) { X.set(eps[i][0], i * eps[0][0].length); Y.set(eps[i][1], i * eps[0][1].length); }
  return { X, Y };
};
const noisy = await collect(true), clean = await collect(false);

// 2・3. 学習して、新しい置き方で試す
const results = { noise: [], clean: [] };
const jobs = [];
for (const [kind, set] of [["noise", noisy], ["clean", clean]]) {
  for (const n of kind === "clean" ? [N_MAX] : SIZES) { // 揺れなしは最大のお手本数だけ(元の図と同じ)
    jobs.push((async () => {
      const t0 = performance.now();
      const { X, Y } = join(set.eps, n);
      const per = await Promise.all(SEEDS.map(async (seed) => {
        const { json, loss } = await run("train", { X, Y, opts: { hidden: HIDDEN, epochs: EPOCHS, batch: BATCH, seed }, meta: { demos: n, noise: kind === "noise" } });
        const outs = (await Promise.all(chunks(evalPlaces, nWorkers).map((placements) => run("evaluate", { json, placements })))).flat();
        return { seed, json, loss, rate: outs.filter(Boolean).length / N_EVAL };
      }));
      const mean = (f) => per.reduce((s, p) => s + f(p), 0) / per.length;
      const rate = mean((p) => p.rate), loss = mean((p) => p.loss);
      results[kind].push({ demos: n, rate, rates: per.map((p) => p.rate), loss, train_sec: Math.round((performance.now() - t0) / 1000) });
      console.log(`2・3. 揺れ${kind === "noise" ? "あり" : "なし"} ${String(n).padStart(3)} 回で学習(誤差 ${loss.toFixed(3)}) → 新しい置き方 ${N_EVAL} 回で成功率 ${(rate * 100).toFixed(0)}% (学習 seed ごと: ${per.map((p) => (p.rate * 100).toFixed(0) + "%").join(", ")}) [${sec(t0)}]`);
      if (SHIPPED.some(([sn, sk]) => sn === n && sk === kind)) {
        // 学習の seed でぶれが大きいので、同梱するのは「平均にいちばん近い成功率だった 1 つ」(表の平均とずれないように)
        const pick = per.reduce((a, b) => (Math.abs(b.rate - rate) < Math.abs(a.rate - rate) ? b : a));
        pick.json.meta.success_rate = pick.rate;
        pick.json.meta.train_seed = pick.seed;
        writeFileSync(new URL(`policy_n${n}_${kind}.json`, dataDir), JSON.stringify(pick.json));
        results[kind].find((r) => r.demos === n).shipped = { train_seed: pick.seed, rate: pick.rate };
      }
    })());
  }
}
const teacherOuts = chunks(evalPlaces, nWorkers).map((placements) => run("evaluate", { teacher: true, placements }));
await Promise.all(jobs);
const teacherRate = (await Promise.all(teacherOuts)).flat().filter(Boolean).length / N_EVAL;
console.log(`先生(手順書)の成功率: ${(teacherRate * 100).toFixed(0)}% (同じ ${N_EVAL} 通り)`);
for (const k of ["noise", "clean"]) results[k].sort((a, b) => a.demos - b.demos);

const table = {
  eval_count: N_EVAL, eval_seeds: [1000, 1000 + N_EVAL - 1], teacher: teacherRate,
  teacher_demo_success: { noise: noisy.ok / N_MAX, clean: clean.ok / N_MAX },
  train_seeds: SEEDS, hidden: HIDDEN, epochs: EPOCHS, batch: BATCH, noise_std: 0.006, noise: results.noise, clean: results.clean,
};
if (!DRY) writeFileSync(new URL("success_rate.json", dataDir), JSON.stringify(table, null, 1));
else console.log(JSON.stringify(table));
for (const [n, k] of SHIPPED) console.log(`data/policy_n${n}_${k}.json: ${(statSync(new URL(`policy_n${n}_${k}.json`, dataDir)).size / 1024).toFixed(0)} KB`);
console.log(`完了 [${sec(T0)}]`);
for (const w of workers) w.terminate();
