// ブラウザ版と同じ sim.js を Node で headless 実行し、README の主張どおりになることを確認する。
// 実行: node apps/drone_flight/test_sim.mjs
import { initMujoco, nodeLoader } from "../_web/mujoco.js";
import { DroneSim, GUST } from "./sim.js";

const mujoco = await initMujoco();
const load = nodeLoader(new URL("../_web/", import.meta.url));
const sim = await DroneSim.create(mujoco, load);
const checks = [];
const check = (ok, msg) => { checks.push(ok); console.log(`${ok ? "OK " : "NG "} ${msg}`); };

for (const weak of [false, true]) {
  sim.reset({ weak });
  const t0 = performance.now();
  for (const _ of sim.run());
  const r = sim.results();
  console.log(`--- ${weak ? "姿勢の制御を 1/10 に弱めた版" : "普通の制御"} (${((performance.now() - t0) / 1000).toFixed(1)} 秒)`);
  console.log(`${r.crashed ? "墜落" : "最後まで飛んだ"} ${r.time.toFixed(2)} 秒 / ずれ 平均 ${r.meanErr.toFixed(1)} cm 最大 ${r.maxErr.toFixed(1)} cm`);
  console.log(`着陸位置 (${r.landing.map((x) => x.toFixed(2)).join(", ")})`);
  if (!weak) {
    console.log(`突風: 最大のずれ ${r.gustErr.toFixed(1)} cm、最大の傾き ${r.gustTilt.toFixed(1)} 度`);
    const final = sim.log[sim.log.length - 1];
    check(!r.crashed && r.time > 19, "ルートを飛び終えた");
    check(r.meanErr < 10, `平均のずれ ${r.meanErr.toFixed(1)} cm < 10 cm`);
    check(r.gustErr > 10 && r.gustErr < 80, `突風で流されたが ${r.gustErr.toFixed(1)} cm にとどまった`);
    check(final.err < 0.05, `最終の位置誤差 ${(final.err * 100).toFixed(1)} cm < 5 cm`);
    check(Math.hypot(r.landing[0], r.landing[1]) < 0.1 && r.landing[2] < 0.2, "元の場所に着陸した");
  } else {
    check(r.crashed && r.time < GUST.start, `姿勢の制御を弱めると ${r.time.toFixed(1)} 秒で墜落した`);
  }
}
// 手動の突風も耐えること
sim.reset();
for (const _ of (function* () { while (sim.time < 4 && sim.step()) yield; })());
sim.gust([5, 0, 0], 1.0);
for (const _ of sim.run());
check(!sim.crashed, "手動の突風(x 方向 5 N)を受けても最後まで飛んだ");
sim.dispose();
if (checks.includes(false)) { console.error("失敗"); process.exit(1); }
