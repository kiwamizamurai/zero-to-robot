// ブラウザ版と同じ sim.js を Node で headless 実行し、3 つのブロックが箱に入ることを確認する。
// 実行: node apps/panda_pick_place/test_sim.mjs
import { initMujoco, nodeLoader } from "../_web/mujoco.js";
import { BLOCKS, PickPlaceSim } from "./sim.js";

const mujoco = await initMujoco();
const load = nodeLoader(new URL("../_web/", import.meta.url));

let failed = false;
for (const opts of [{ seed: 1, jitter: 0 }, { seed: 7, jitter: 0.02 }, { seed: 42, jitter: 0.02 }]) {
  const sim = await PickPlaceSim.create(mujoco, load, opts);
  const t0 = performance.now();
  for (const _ of sim.run());
  console.log(`--- seed=${opts.seed} jitter=${opts.jitter} (${((performance.now() - t0) / 1000).toFixed(1)} 秒)`);
  console.log(sim.log.join("\n"));
  const results = sim.results();
  for (const { name } of BLOCKS) {
    const p = sim.blockPos(name);
    console.log(`${name.padEnd(5)}: (${p[0].toFixed(3)}, ${p[1].toFixed(3)}, ${p[2].toFixed(3)}) ${results[name] ? "箱の中" : "箱の外"}`);
  }
  console.log(`シミュレーション時間 ${sim.time.toFixed(1)} 秒`);
  if (!Object.values(results).every(Boolean)) failed = true;
  sim.dispose();
}
if (failed) {
  console.error("箱に入らなかったブロックがある");
  process.exit(1);
}
