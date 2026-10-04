// ブラウザ版と同じ sim.js を Node で headless 実行し、6 通り(硬い/柔らかい × 机 -1cm/0/+1cm)の接触力を比べる。
// 実行: node apps/panda_impedance/test_sim.mjs
import { initMujoco, nodeLoader } from "../_web/mujoco.js";
import { ImpedanceSim, MODES } from "./sim.js";

const mujoco = await initMujoco();
const load = nodeLoader(new URL("../_web/", import.meta.url));
const sim = await ImpedanceSim.create(mujoco, load, {});
const WHERE = { "-0.01": "1 cm 低い", "0": "見込みどおり", "0.01": "1 cm 高い" };
const res = {};
let failed = false;
const check = (ok, msg) => { if (!ok) { console.error("NG: " + msg); failed = true; } };
console.log("制御                     机の高さ     平均の力  最大の力  触れた時間  描けた円周");
const t0 = performance.now();
for (const mode of ["stiff", "soft"]) {
  for (const offset of [-0.01, 0, 0.01]) {
    sim.reset({ mode, offset });
    for (const _ of sim.run());
    const s = res[`${mode}${offset}`] = sim.summary();
    console.log(`${MODES[mode].name.padEnd(18, "　")} ${WHERE[offset].padEnd(8, "　")} ${s.mean.toFixed(1).padStart(7)} N ${s.max.toFixed(1).padStart(7)} N ${s.touching.toFixed(0).padStart(7)} % ${s.covered.toFixed(0).padStart(7)} %`);
  }
}
console.log(`所要 ${((performance.now() - t0) / 1000).toFixed(1)} 秒`);
// 柔らかい制御: どの高さでも 5〜15 N で、円周の 90% 以上が描ける
for (const off of [-0.01, 0, 0.01]) {
  const s = res[`soft${off}`];
  check(s.mean > 5 && s.mean < 15 && s.covered > 90, `柔らかい制御(${WHERE[off]})が狙った力で描けていない`);
}
// 硬い制御: 机が低いと触れず、高いと見込みの数倍の力になる
check(res["stiff-0.01"].mean < 1 && res["stiff-0.01"].covered < 5, "硬い制御で机が低いのに触れている");
check(res["stiff0"].mean > 5 && res["stiff0"].mean < 15, "硬い制御で見込みどおりの机でも 10 N 前後にならない");
check(res["stiff0.01"].mean > 40, "硬い制御で机が高いのに力が大きくならない");
// 力の変わり方: 柔らかい制御は硬い制御よりずっと鈍感
const spreadSoft = res["soft0.01"].mean - res["soft-0.01"].mean, spreadStiff = res["stiff0.01"].mean - res["stiff-0.01"].mean;
check(spreadSoft * 4 < spreadStiff, "柔らかい制御の力の変化が十分小さくない");
sim.dispose();
if (failed) process.exit(1);
console.log("OK");
