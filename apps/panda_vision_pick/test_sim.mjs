// 画像から位置・向きを見つける部分(合成画像)と、見つけた結果でつかんで箱に片付けるシミュレーションを Node で確認する。
// 実行: node apps/panda_vision_pick/test_sim.mjs
import assert from "node:assert/strict";
import { initMujoco, nodeLoader } from "../_web/mujoco.js";
import { mulberry32 } from "../_web/math.js";
import {
  COLORS, NOMINAL_CAMERA, OVERHEAD, VisionPickSim, detectBlocks, inBoxArea, randomBlocks, rgbToHsv, squareFit, wrapYaw,
} from "./sim.js";
import { renderSynthetic } from "./synth.mjs";

const deg = (r) => (r * 180) / Math.PI;
const wrapDiff = (a, b) => wrapYaw(a - b);
let checks = 0;
const ok = (cond, msg) => { assert.ok(cond, msg); checks++; };

// 1. HSV 変換(OpenCV と同じ尺度: H 0〜180、S・V 0〜255)
{
  const near = (a, b) => a.every((x, i) => Math.abs(x - b[i]) < 1);
  ok(near(rgbToHsv(255, 0, 0), [0, 255, 255]), "赤");
  ok(near(rgbToHsv(0, 255, 0), [60, 255, 255]), "緑");
  ok(near(rgbToHsv(0, 0, 255), [120, 255, 255]), "青");
  ok(near(rgbToHsv(128, 128, 128), [0, 0, 128]), "灰色は彩度 0");
  console.log("HSV 変換: OK");
}

// 2. 正方形のかたまり(画素)の中心と傾きを当てる
{
  let worst = 0;
  for (let k = 0; k < 40; k++) {
    const rnd = mulberry32(k + 1);
    const th = (rnd() - 0.5) * (Math.PI / 2), cx = 100 + rnd() * 100, cy = 100 + rnd() * 100, half = 8 + rnd() * 4;
    const pts = [];
    for (let y = 0; y < 300; y++) {
      for (let x = 0; x < 300; x++) {
        const dx = x - cx, dy = y - cy, a = Math.cos(th) * dx + Math.sin(th) * dy, b = -Math.sin(th) * dx + Math.cos(th) * dy;
        if (Math.abs(a) <= half && Math.abs(b) <= half) pts.push([x, y]);
      }
    }
    const r = squareFit(pts);
    ok(Math.hypot(r.center[0] - cx, r.center[1] - cy) < 0.3, "正方形の中心");
    const err = Math.abs(deg(wrapYaw(r.angle - th)));
    worst = Math.max(worst, err);
    ok(err < 3, `正方形の向き(誤差 ${err.toFixed(2)}°)`);
  }
  console.log(`正方形の中心・向き(画素の粗さつき 40 通り): 向きの誤差の最大 ${worst.toFixed(2)}°`);
}

// 3. 合成画像からの検出(真下を向く設計図どおりのカメラ)
const truthErr = (found, blocks) => blocks.map((b) => {
  const f = found.find((x) => x.name === b.name);
  assert.ok(f, `${b.name} が見つからない`);
  return { name: b.name, pos: Math.hypot(f.pos[0] - b.xy[0], f.pos[1] - b.xy[1]) * 1000, yaw: Math.abs(deg(wrapDiff(f.yaw, b.yaw))) };
});
const POS_TOL = 2.5, YAW_TOL = 4.0; // [mm], [°] 画素の大きさは約 2.2 mm
{
  let worstP = 0, worstY = 0;
  for (let seed = 0; seed < 6; seed++) {
    const blocks = randomBlocks(seed);
    const rnd = mulberry32(100 + seed);
    const { image, depth } = renderSynthetic({ cam: NOMINAL_CAMERA, blocks, noise: 5, rnd, speckles: 12 });
    const { found } = detectBlocks(image, { depth, extrinsics: NOMINAL_CAMERA, fovy: OVERHEAD.fovy, inBox: inBoxArea });
    ok(found.length === COLORS.length, `seed ${seed}: 4 個ちょうど見つかる(雑音の点・箱のふちは無視)。実際 ${found.length}`);
    for (const e of truthErr(found, blocks)) {
      worstP = Math.max(worstP, e.pos); worstY = Math.max(worstY, e.yaw);
      ok(e.pos < POS_TOL, `seed ${seed} ${e.name}: 位置の誤差 ${e.pos.toFixed(2)} mm`);
      ok(e.yaw < YAW_TOL, `seed ${seed} ${e.name}: 向きの誤差 ${e.yaw.toFixed(2)}°`);
    }
    // 深度を 1 点だけ測る方式(ブラウザの 3D 画面から撮る場合)も同じ答えになる
    const { found: found2 } = detectBlocks(image, {
      depthAt: (u, v) => depth[Math.floor(v) * image.width + Math.floor(u)], extrinsics: NOMINAL_CAMERA, inBox: inBoxArea,
    });
    ok(found2.length === COLORS.length, "depthAt 方式でも 4 個見つかる");
    for (const e of truthErr(found2, blocks)) ok(e.pos < POS_TOL + 1 && e.yaw < YAW_TOL + 2, `depthAt 方式の誤差 ${e.pos.toFixed(2)} mm ${e.yaw.toFixed(2)}°`);
  }
  console.log(`合成画像(設計図どおりのカメラ, 6 通り, 雑音あり): 位置の誤差の最大 ${worstP.toFixed(2)} mm, 向きの誤差の最大 ${worstY.toFixed(2)}°`);
}

// 4. 箱の中にあるブロックは捨てる
{
  const blocks = [...randomBlocks(1).slice(0, 3), { name: "yellow", xy: [0.40, -0.38], yaw: 0.3, rgba: COLORS[3].rgba }];
  const { image, depth } = renderSynthetic({ cam: NOMINAL_CAMERA, blocks });
  const { found } = detectBlocks(image, { depth, extrinsics: NOMINAL_CAMERA, inBox: inBoxArea });
  ok(found.length === 3 && !found.some((f) => f.name === "yellow"), "箱の中の黄色は見つけない");
  console.log("箱の中のブロックを捨てる: OK");
}

// 5. カメラがずれて付いているとき: 正しい姿勢を使えば当たり、設計図の姿勢を使うと大きくずれる
{
  const rad = Math.PI / 180;
  const Rx = (a) => [1, 0, 0, 0, Math.cos(a), -Math.sin(a), 0, Math.sin(a), Math.cos(a)];
  const cam = { R: Rx(3 * rad), t: [OVERHEAD.pos[0] + 0.03, OVERHEAD.pos[1] - 0.02, OVERHEAD.pos[2] + 0.01] };
  const blocks = randomBlocks(2);
  const { image, depth } = renderSynthetic({ cam, blocks });
  const right = detectBlocks(image, { depth, extrinsics: cam, inBox: inBoxArea }).found;
  const wrong = detectBlocks(image, { depth, extrinsics: NOMINAL_CAMERA, inBox: inBoxArea }).found;
  const eR = truthErr(right, blocks), eW = truthErr(wrong, blocks);
  for (const e of eR) ok(e.pos < POS_TOL && e.yaw < YAW_TOL + 2, `正しい姿勢: 誤差 ${e.pos.toFixed(2)} mm ${e.yaw.toFixed(2)}°`);
  for (const e of eW) ok(e.pos > 20, `設計図の姿勢だと大きくずれる(${e.pos.toFixed(1)} mm)`);
  console.log(`ずれて付いたカメラ: 正しい姿勢 最大 ${Math.max(...eR.map((e) => e.pos)).toFixed(2)} mm / 設計図の姿勢 最小 ${Math.min(...eW.map((e) => e.pos)).toFixed(1)} mm`);
}

// 6. シミュレーション(光線で撮った画像 → 検出 → つかんで箱へ)
const mujoco = await initMujoco();
const load = nodeLoader(new URL("../_web/", import.meta.url));
let failed = 0;
for (const seed of [0, 1, 2, 3, 4]) {
  const sim = await VisionPickSim.create(mujoco, load, { seed });
  const t0 = performance.now();
  for (const _ of sim.run());
  const res = sim.results(), shot = sim.shots[0];
  const errs = shot.found.map((f) => `${f.name} ${(f.posError * 1000).toFixed(1)}mm/${deg(f.yawError).toFixed(1)}°`).join(", ");
  const inside = Object.values(res).filter(Boolean).length;
  console.log(`seed=${seed}: 最初の検出 ${errs} → 箱の中 ${inside}/${COLORS.length}、つかみに行った回数 ${sim.placed} (${((performance.now() - t0) / 1000).toFixed(1)} 秒)`);
  ok(shot.found.length === COLORS.length, `seed ${seed}: 最初の撮影で 4 個見つかる`);
  for (const f of shot.found) {
    ok(f.posError * 1000 < 4, `seed ${seed} ${f.name}: 位置の誤差 ${(f.posError * 1000).toFixed(1)} mm`);
    ok(Math.abs(deg(f.yawError)) < 7, `seed ${seed} ${f.name}: 向きの誤差 ${deg(f.yawError).toFixed(1)}°`);
  }
  if (inside !== COLORS.length) failed++;
  ok(inside === COLORS.length, `seed ${seed}: 4 個とも箱に入る`);
  sim.dispose();
}

// 7. 画像に雑音が乗っても片付けられる(撮り方を差し替えて、画素に雑音を足す)
{
  const sim = await VisionPickSim.create(mujoco, load, { seed: 6 });
  const rnd = mulberry32(9);
  sim.shooter = (s, cam) => {
    const shot = s.ray.shoot(s.model, s.data, cam);
    const d = shot.image.data;
    for (let i = 0; i < d.length; i++) if (i % 4 < 3) d[i] += (rnd() - 0.5) * 24;
    return shot;
  };
  for (const _ of sim.run());
  const inside = Object.values(sim.results()).filter(Boolean).length;
  console.log(`画素雑音(±12)つき seed=6: 箱の中 ${inside}/${COLORS.length}`);
  ok(inside === COLORS.length, "雑音つきでも 4 個とも箱に入る");
  sim.dispose();
}
console.log(`確認 ${checks} 項目 OK`);
if (failed) process.exit(1);
