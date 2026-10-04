// ブラウザ版と同じ sim.js / policy.js を Node で headless 実行し、同梱した学習済みの方策を検証する。
// 実行: node apps/panda_imitation/test_sim.mjs
// 学習時の評価(seed 1000〜)とは別の置き方(seed 5000〜)で、先生(手順書)と 4 つの方策の成功率を測る。
import { readFileSync, statSync, readdirSync } from "node:fs";
import { initMujoco, nodeLoader } from "../_web/mujoco.js";
import { ImitationSim, placementFromSeed } from "./sim.js";
import { Policy } from "./policy.js";

const N = 40;
const dataDir = new URL("./data/", import.meta.url);
const json = (f) => JSON.parse(readFileSync(new URL(f, dataDir), "utf8"));
const mujoco = await initMujoco();
const sim = await ImitationSim.create(mujoco, nodeLoader(new URL("../_web/", import.meta.url)), { seed: 1 });
const places = Array.from({ length: N }, (_, i) => placementFromSeed(5000 + i));

let failed = false;
const check = (ok, msg) => { console.log(`${ok ? "OK  " : "NG  "}${msg}`); if (!ok) failed = true; };

const rate = (policy) => {
  let ok = 0;
  for (const p of places) {
    sim.place(...p);
    for (const _ of policy ? sim.runPolicy(policy) : sim.runTeacher());
    ok += sim.success();
  }
  return ok / N;
};

// 1. 先生(手順書)
let t0 = performance.now();
const teacher = rate(null);
console.log(`先生(手順書): ${(teacher * 100).toFixed(0)}% [${((performance.now() - t0) / 1000).toFixed(0)} 秒]`);
check(teacher >= 0.9, "先生は 9 割以上の置き方で片付けられる");

// 2. 学習済みの方策
const R = {};
for (const name of ["n10_noise", "n50_noise", "n200_noise", "n200_clean"]) {
  t0 = performance.now();
  const j = json(`policy_${name}.json`);
  R[name] = rate(new Policy(j));
  console.log(`${name.padEnd(11)}: ${(R[name] * 100).toFixed(0)}% (${N} 通り、学習時の評価では ${(j.meta.success_rate * 100).toFixed(0)}%) [${((performance.now() - t0) / 1000).toFixed(0)} 秒]`);
}
check(R.n10_noise <= 0.1, "お手本 10 回では、ほとんど成功しない");
check(R.n200_noise > R.n10_noise + 0.3 && R.n200_noise > R.n50_noise, "お手本が多いほど成功率が上がる(10 回 < 50 回 < 200 回)");
check(R.n200_noise > 0.4, "お手本 200 回(揺れあり)で 4 割以上成功する");
console.log(`    揺れあり 200 回 ${(R.n200_noise * 100).toFixed(0)}% / 揺れなし 200 回 ${(R.n200_clean * 100).toFixed(0)}% (この 1 組だけでは差は学習のぶれの範囲)`);

// 3. 事前計算した表(学習 seed 3 回の平均)
const table = json("success_rate.json");
const last = table.noise[table.noise.length - 1], clean = table.clean[0];
const rates = table.noise.map((r) => r.rate);
console.log(`表(学習 seed ${table.train_seeds.length} 回の平均、評価 ${table.eval_count} 通り): ${table.noise.map((r) => `${r.demos}回 ${(r.rate * 100).toFixed(0)}%`).join(", ")} / 揺れなし ${clean.demos}回 ${(clean.rate * 100).toFixed(0)}% / 先生 ${(table.teacher * 100).toFixed(0)}%`);
check(rates.every((r, i) => i === 0 || r >= rates[i - 1]), "表: お手本が増えるほど成功率が上がる(単調)");
check(last.rate > clean.rate, "表: 同じ 200 回でも、揺れを混ぜたほうが成功率が高い");

// 4. 容量
const size = readdirSync(dataDir).reduce((s, f) => s + statSync(new URL(f, dataDir)).size, 0);
console.log(`data/ 合計 ${(size / 1024).toFixed(0)} KB`);
check(size < 1024 * 1024, "data/ は合計 1MB 未満");

// 5. ブラウザと同じ経路(reset で場面を作り直す → 方策を再生)
sim.reset({ seed: 3 });
let steps = 0;
for (const _ of sim.runPolicy(new Policy(json("policy_n200_noise.json")))) steps++;
check(steps === 500, `reset(seed=3) からの再生が 500 ステップで終わる(ブロック ${sim.blockPos("block").map((v) => v.toFixed(2)).join(", ")})`);

sim.dispose();
if (failed) { console.error("検証に失敗した項目がある"); process.exit(1); }
