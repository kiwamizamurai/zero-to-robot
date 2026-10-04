// ブラウザ版と同じ sim.js を Node で headless 実行し、RRT-Connect の道が壁をよけることを確認する。
// 実行: node apps/panda_rrt/test_sim.mjs
import { initMujoco, nodeLoader } from "../_web/mujoco.js";
import { RrtSim } from "./sim.js";

const mujoco = await initMujoco();
const load = nodeLoader(new URL("../_web/", import.meta.url));
const sim = await RrtSim.create(mujoco, load, { seed: 0 });

let failed = false;
const check = (ok, msg) => { if (!ok) { console.error(`NG: ${msg}`); failed = true; } };
for (const seed of [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]) {
  sim.reset({ seed });
  const t0 = performance.now();
  for (const _ of sim.run());
  const s = sim.stats;
  console.log(`--- seed=${seed} (${((performance.now() - t0) / 1000).toFixed(1)} 秒)`);
  console.log(sim.log.join("\n"));
  check(sim.phase === "done", `seed=${seed}: 計画が成功する (phase=${sim.phase})`);
  if (sim.phase !== "done") continue;
  check(s.startFree && s.goalFree, `seed=${seed}: 始まり・終わりの姿勢が衝突しない`);
  check(!s.straightFree, `seed=${seed}: 直線経路が壁にぶつかる`);
  // 経路上の全ての姿勢(マージンつきで)衝突しない
  const free = sim.path.every((q, i) => i === 0 || sim.chk.edgeFree(sim.path[i - 1], q, 0.01)) && sim.chk.free(sim.path[0]);
  check(free, `seed=${seed}: 経路上の全ての姿勢が衝突しない`);
  check(s.hitsStraight > 0, `seed=${seed}: まっすぐ実行は壁に触れる (${s.hitsStraight})`);
  check(s.hitsRrt === 0, `seed=${seed}: RRT の道の実行で接触 0 (${s.hitsRrt})`);
  check(s.err < 3, `seed=${seed}: 着いた位置のずれ 3 cm 未満 (${s.err.toFixed(1)} cm)`);
}
sim.dispose();
if (failed) process.exit(1);
console.log("OK");
