// ブラウザ版と同じ sim.js・data/policy.json を Node で headless 実行し、学習前はすぐ倒れ、学習後は立ち続けることを確認する。
// 実行: node apps/rl_balance/test_sim.mjs
import { readFile } from "node:fs/promises";
import { initMujoco } from "../_web/mujoco.js";
import { BalanceSim, mlpPolicy, randomPolicy, evaluate, MAX_STEPS } from "./sim.js";

const data = JSON.parse(await readFile(new URL("./data/policy.json", import.meta.url)));
const mujoco = await initMujoco();
const sim = await BalanceSim.create(mujoco);
const mean = (a) => a.reduce((x, y) => x + y, 0) / a.length;
let failed = false;
const check = (ok, msg) => { console.log(`${ok ? "OK " : "NG "} ${msg}`); if (!ok) failed = true; };

const random = Array.from({ length: 20 }, (_, s) => evaluate(sim, randomPolicy(s), s));
console.log(`でたらめ(20 回): 平均 ${mean(random).toFixed(1)} ステップ  [${random.join(",")}]`);
check(mean(random) < 30, "でたらめに動かすとすぐ倒れる(平均 30 ステップ未満)");

const scores = {};
for (const p of data.policies) {
  const pol = mlpPolicy(p.w);
  const lens = Array.from({ length: 10 }, (_, s) => evaluate(sim, pol, 100 + s));
  scores[p.id] = mean(lens);
  console.log(`${p.id.padEnd(6)} ${p.label}: 平均 ${mean(lens).toFixed(1)} ステップ  [${lens.join(",")}]`);
}
const final = mlpPolicy(data.policies.at(-1).w);
const lens = Array.from({ length: 10 }, (_, s) => evaluate(sim, final, 100 + s));
check(lens.every((l) => l === MAX_STEPS), `学習後は 10 seed とも ${MAX_STEPS} ステップ立ち続ける`);
const ids = data.policies.map((p) => p.id);
check(ids.slice(0, -1).every((id) => scores[id] < scores.final), "途中の方策は学習後より短い");

// 押されたときの立ち直り(学習中は押していないので、強い力には耐えられないことがある。参考値)
for (const f of [10, 20, 40, 60, 100]) {
  const r = Array.from({ length: 5 }, (_, s) => evaluate(sim, final, 200 + s, { pushAt: 150, pushForce: s % 2 ? -f : f }));
  console.log(`押す ${String(f).padStart(3)} N: ${r.filter((l) => l === MAX_STEPS).length}/5 回立ち直った  [${r.join(",")}]`);
}
// 画面と同じ流れ(sim.run)でも同じ結果になること
sim.reset(100);
let n = 0; for (const _ of sim.run(final)) n++;
check(sim.result.length === MAX_STEPS && !sim.result.fell, `sim.run でも ${MAX_STEPS} ステップ立ち続ける(${n} 回 yield)`);

sim.dispose();
if (failed) process.exit(1);
