// ブラウザ版と同じ sim.js を Node で headless 実行し、(1) 指令への追従 (2) キューブがトレイに入ること、を確認する。
// 実行: node apps/so101_twin/test_sim.mjs
import { initMujoco, nodeLoader } from "../_web/mujoco.js";
import { SimSO101, planAndRun, checkLog, logToCsv, DEFAULTS, MOTORS } from "./sim.js";

const mujoco = await initMujoco();
const load = nodeLoader(new URL("../_web/", import.meta.url));
// 5 関節それぞれの追従誤差の許容。最大は、降りたときに指先が机に当たって肩が止まる分(約 4°)を見込んで 5°、平均は 0.5°
const TRACK_LIMIT_DEG = 5.0, TRACK_MEAN_DEG = 0.5;

let failed = false;
const cases = [
  { name: "既定", cube: DEFAULTS.cubeXY, yaw: DEFAULTS.cubeYaw },
  { name: "位置と向きを変える", cube: [0.20, 0.05], yaw: -30 },
  { name: "遠く・斜め", cube: [0.27, 0.13], yaw: 45 },
];
for (const c of cases) {
  const t0 = performance.now();
  const robot = await SimSO101.create(mujoco, load, { maxRelativeTarget: 10, cubeXY: c.cube, cubeYaw: (c.yaw * Math.PI) / 180 });
  robot.connect();
  const log = [];
  for (const _ of planAndRun(robot, robot.config, log));
  const ok = robot.cubeInTray();
  const p = robot.cubePose().pos;
  console.log(`--- ${c.name}: cube=(${c.cube}) yaw=${c.yaw}° (${((performance.now() - t0) / 1000).toFixed(1)} 秒)`);
  console.log(`キューブ (${p.map((v) => v.toFixed(3))}) → ${ok ? "トレイの中" : "トレイの外"}、${log.length} ステップ、${(log.length / robot.config.fps).toFixed(1)} 秒`);
  let worstAll = 0, meanAll = 0;
  for (const [m, [mx, mean]] of Object.entries(checkLog(log))) {
    console.log(`  追従誤差 ${m.padEnd(13)}: 最大 ${mx.toFixed(2)}°, 平均 ${mean.toFixed(2)}°`);
    worstAll = Math.max(worstAll, mx);
    meanAll = Math.max(meanAll, mean);
  }
  if (!ok) { console.error("  キューブがトレイに入らなかった"); failed = true; }
  if (worstAll > TRACK_LIMIT_DEG || meanAll > TRACK_MEAN_DEG) { console.error(`  追従誤差が許容(最大 ${TRACK_LIMIT_DEG}°、平均 ${TRACK_MEAN_DEG}°)を超えた`); failed = true; }
  // CSV が書き出せること、観測が 6 関節ぶんあること
  const csv = logToCsv(log);
  if (csv.split("\n")[0].split(",").length !== 2 + 2 * MOTORS.length) { console.error("  CSV の列数が違う"); failed = true; }
  robot.dispose();
}
// 安全制限: 1 回の指令で max_relative_target を超えて動かない
{
  const robot = await SimSO101.create(mujoco, load, { maxRelativeTarget: 5 });
  robot.connect();
  const sent = robot.sendAction({ "shoulder_pan.pos": 90, "shoulder_lift.pos": 0, "elbow_flex.pos": 0, "wrist_flex.pos": 0, "wrist_roll.pos": 0, "gripper.pos": 50 });
  const ok = Math.abs(sent["shoulder_pan.pos"] - 5) < 0.1;
  console.log(`安全制限: 90° を指令 → 送った値 ${sent["shoulder_pan.pos"].toFixed(2)}° ${ok ? "OK" : "NG"}`);
  if (!ok) failed = true;
  robot.dispose();
}
if (failed) process.exit(1);
console.log("OK");
