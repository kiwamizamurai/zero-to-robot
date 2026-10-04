// ブラウザ版と同じ sim.js を Node で headless 実行する。
//  (1) 検出関数を、合成した画像で確かめる
//  (2) 赤い実だけを収穫し、緑の実には触れず、かごに入ることを確かめる(カメラ画像は投影で作る: synth.mjs)
// 実行: node apps/tomato_harvest/test_sim.mjs
import { initMujoco, nodeLoader } from "../_web/mujoco.js";
import { TomatoSO101, FRUITS, EYE, detectFruits, harvestRun, harvestResult, DEFAULT_PARAMS, cameraBasis, rgbToHsv } from "./sim.js";
import { renderEye } from "./synth.mjs";

let failed = false;
const check = (ok, msg) => { console.log(`${ok ? "OK " : "NG "} ${msg}`); if (!ok) failed = true; };

// ---- (1) 検出関数: 単純な合成画像 ----
{
  const [w, h] = [200, 150];
  const data = new Uint8Array(w * h * 4);
  const fill = (rgb) => { for (let i = 0; i < w * h; i++) data.set([...rgb, 255], 4 * i); };
  const disc = (cx, cy, r, rgb) => { for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) if ((x - cx) ** 2 + (y - cy) ** 2 <= r * r) data.set([...rgb, 255], 4 * (y * w + x)); };
  fill([200, 195, 180]);
  const cam = { ...EYE, width: w, height: h };
  // 正面を向いたカメラの前の、同じ距離に 5 つ並べる(半径 13 mm の球が見える大きさ: 距離 0.30 m)
  const f = (h / 2) / Math.tan((cam.fovy * Math.PI) / 360);
  const r = (f * 0.013) / 0.30;
  disc(40, 40, r, [217, 31, 20]);    // 赤
  disc(100, 40, r, [240, 130, 30]);  // オレンジ
  disc(160, 40, r, [90, 165, 50]);   // 緑
  disc(40, 100, r, [178, 102, 64]);  // 鉢の茶色(彩度が低い)
  disc(100, 100, r, [255, 209, 30]); // アームの黄色(色相が外れる)
  const found = detectFruits({ width: w, height: h, data }, { threshold: 0.7, satMin: 185, hueMax: 20 }, { ...cam, pos: [0, 0, 0.2], lookAt: [0.3, 0, 0.2] });
  check(found.length === 2, `赤とオレンジだけ見つかる(緑・茶色・黄色は外れる): ${found.length} 個`);
  const red = found.find((x) => x.px[0] < 70), org = found.find((x) => x.px[0] > 70);
  check(red && red.ripeness > 0.95 && red.harvest, `赤い実: 熟し具合 ${red?.ripeness.toFixed(2)} → 収穫する`);
  check(org && org.ripeness < 0.05 && !org.harvest, `オレンジの実: 熟し具合 ${org?.ripeness.toFixed(2)} → まだ`);
  const all = detectFruits({ width: w, height: h, data }, { threshold: 0, satMin: 185, hueMax: 20 }, { ...cam, pos: [0, 0, 0.2], lookAt: [0.3, 0, 0.2] });
  check(all.length === 2 && all.every((x) => x.harvest), "しきい値 0 ではどちらも収穫の対象");
  // 距離の推定: 距離 0.30 m に置いたので、前方 0.30 m 付近に出る
  check(Math.abs(red.pos[0] - 0.30) < 0.02, `距離の推定: 前方 ${red.pos[0].toFixed(3)} m(正解 0.300)`);
  // 行が下から上の順(three.js の readRenderTargetPixels)の画像も同じ結果になる
  const flipped = new Uint8Array(w * h * 4);
  for (let y = 0; y < h; y++) flipped.set(data.subarray(4 * y * w, 4 * (y + 1) * w), 4 * (h - 1 - y) * w);
  const f2 = detectFruits({ width: w, height: h, data: flipped, flipY: true }, { threshold: 0.7, satMin: 185, hueMax: 20 }, { ...cam, pos: [0, 0, 0.2], lookAt: [0.3, 0, 0.2] });
  check(f2.length === 2 && f2.some((x) => x.px[0] === red.px[0] && x.px[1] === red.px[1]), "flipY の画像も同じ結果");
  const [H, S, V] = rgbToHsv(255, 0, 0);
  check(H === 0 && S === 255 && V === 255, "HSV 変換(赤)");
}

// ---- (2) 収穫の通し ----
const mujoco = await initMujoco();
const load = nodeLoader(new URL("../_web/", import.meta.url));

async function runCase(label, threshold) {
  const t0 = performance.now();
  const robot = await TomatoSO101.create(mujoco, load, { maxRelativeTarget: 10 });
  robot.connect();
  // 置いたままで、実が落ちたり動いたりしないこと(へたの拘束が効いていること)
  for (let i = 0; i < 40; i++) robot.sendAction(robot.getObservation());
  const drift = Math.max(...FRUITS.map(([n, x, y, z]) => Math.hypot(...robot.fruitPos(n).map((v, i) => v - [x, y, z][i])) * 1000));
  console.log(`--- ${label} (threshold=${threshold}) 静止 1 秒での実の動き 最大 ${drift.toFixed(2)} mm`);
  check(drift < 2, "待機中、実が動かない");
  const lines = [];
  const task = harvestRun(robot, { look: () => renderEye(robot), params: () => ({ ...DEFAULT_PARAMS, threshold }), onLog: (t) => lines.push(t) });
  let steps = 0;
  for (const _ of task) steps++;
  console.log(lines.join("\n"));
  const res = harvestResult(robot);
  console.log(`(${steps} ステップ = ${(steps / robot.config.fps).toFixed(0)} 秒、計算 ${((performance.now() - t0) / 1000).toFixed(1)} 秒)`);
  for (const [n, r] of Object.entries(res)) {
    console.log(`   ${n}: 熟し具合 ${r.ripeness.toFixed(2)} → ${r.basket ? "かごの中" : r.picked ? "採ったが、かごの外" : "木に残っている"}${!r.picked ? `(動き ${r.moved.toFixed(1)} mm)` : ""}`);
  }
  robot.dispose();
  return res;
}

const a = await runCase("既定", 0.7);
for (const [n, r] of Object.entries(a)) {
  if (r.ripeness >= 0.8) check(r.basket, `${n}(熟した)がかごに入った`);
  if (r.ripeness < 0.6) check(!r.picked, `${n}(熟していない)は採っていない`);
  if (r.ripeness < 0.3) check(!r.picked && r.moved < 5, `${n}(緑)は触れていない(動き ${r.moved.toFixed(1)} mm)`);
}
const b = await runCase("基準を下げる", 0);
check(b.t5.basket, "しきい値 0 ではオレンジの実 t5 も採ってかごに入る");
check(!b.t3.picked && !b.t6.picked, "しきい値 0 でも緑の実(t3, t6)は採らない");

if (failed) { console.error("失敗あり"); process.exit(1); }
console.log("すべて OK");
