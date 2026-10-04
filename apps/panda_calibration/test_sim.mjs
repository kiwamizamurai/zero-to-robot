// Kabsch 法(SVD)・印の検出(合成画像)と、キャリブレーション前後でつかめるかのシミュレーションを Node で確認する。
// 実行: node apps/panda_calibration/test_sim.mjs
import assert from "node:assert/strict";
import { initMujoco, nodeLoader } from "../_web/mujoco.js";
import { mulberry32, gauss } from "../_web/math.js";
import {
  CalibrationSim, COLORS, MARKER_R, NOMINAL_CAMERA, OVERHEAD, det3, findMarker, kabsch, mountError, mountedCamera,
  mulMat, mulVec3, rotDiffDeg, svd3, transpose3,
} from "./sim.js";
import { renderSynthetic } from "../panda_vision_pick/synth.mjs";

let checks = 0;
const ok = (cond, msg) => { assert.ok(cond, msg); checks++; };
const near = (a, b, tol, msg) => ok(Math.abs(a - b) <= tol, `${msg}: ${a} vs ${b}`);
const I3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];
const maxAbs = (A, B) => Math.max(...A.map((x, i) => Math.abs(x - B[i])));

// 乱数の回転(軸と角度から Rodrigues の式)
function randRot(rnd) {
  let a = [gauss(rnd), gauss(rnd), gauss(rnd)];
  const n = Math.hypot(...a); a = a.map((x) => x / n);
  const th = rnd() * Math.PI, c = Math.cos(th), s = Math.sin(th), [x, y, z] = a;
  return [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s,
    y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s,
    z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)];
}

// 1. 3x3 の SVD: A = U diag(S) V^T を再構成でき、U と V は直交行列
{
  const rnd = mulberry32(3);
  for (let k = 0; k < 50; k++) {
    const A = Array.from({ length: 9 }, () => gauss(rnd));
    const { U, S, V } = svd3(A);
    const rec = mulMat(mulMat(U, [S[0], 0, 0, 0, S[1], 0, 0, 0, S[2]]), transpose3(V));
    ok(maxAbs(rec, A) < 1e-10, "SVD の再構成");
    ok(maxAbs(mulMat(transpose3(U), U), I3) < 1e-10 && maxAbs(mulMat(transpose3(V), V), I3) < 1e-10, "U, V は直交");
    ok(S[0] >= S[1] && S[1] >= S[2] && S[2] >= 0, "特異値は大きい順");
  }
  // ランク 2(一平面上の点から作る共分散行列)でも直交行列が返る
  const A2 = [1, 2, 0, 3, 4, 0, 0, 0, 0];
  const r = svd3(A2);
  ok(maxAbs(mulMat(transpose3(r.U), r.U), I3) < 1e-10 && r.S[2] < 1e-12, "ランク落ちの SVD");
  console.log("3x3 の SVD: OK");
}

// 2. Kabsch 法: 既知の回転・平行移動を復元する
{
  const rnd = mulberry32(11);
  let worstR = 0, worstT = 0, worstNoisy = 0;
  for (let k = 0; k < 30; k++) {
    const R = randRot(rnd), t = [rnd() - 0.5, rnd() - 0.5, rnd() - 0.5];
    const P = Array.from({ length: 6 + (k % 6) }, () => [rnd() * 0.4, rnd() * 0.4, rnd() * 0.3]);
    const Q = P.map((p) => mulVec3(R, p).map((x, i) => x + t[i]));
    const sol = kabsch(P, Q);
    worstR = Math.max(worstR, maxAbs(sol.R, R)); worstT = Math.max(worstT, Math.hypot(...sol.t.map((x, i) => x - t[i])));
    ok(Math.abs(det3(sol.R) - 1) < 1e-9, "回転行列(行列式 +1)");
    ok(sol.rms < 1e-9, "雑音なしなら残差はほぼ 0");
    // 1 mm の雑音を足しても、回転は 1° 以内、点の集まりの中心の変換は 1 mm 以内(平行移動そのものは回転の誤差が原点からの距離ぶん増幅される)
    const Qn = Q.map((q) => q.map((x) => x + gauss(rnd) * 0.001));
    const sn = kabsch(P, Qn);
    const dR = rotDiffDeg(sn.R, R);
    const cen = [0, 1, 2].map((i) => P.reduce((a, p) => a + p[i], 0) / P.length);
    const dt = Math.hypot(...mulVec3(sn.R, cen).map((x, i) => x + sn.t[i] - (mulVec3(R, cen)[i] + t[i]))) * 1000;
    worstNoisy = Math.max(worstNoisy, dR);
    ok(dR < 1 && dt < 1, `雑音ありの Kabsch: ${dR.toFixed(3)}° ${dt.toFixed(2)} mm`);
  }
  ok(worstR < 1e-9 && worstT < 1e-9, "既知の変換の復元");
  // 点がすべて一平面上にあっても(特異値が 0 の列)鏡映にならず、正しい回転が出る
  const R = randRot(rnd), t = [0.1, -0.2, 0.3];
  const P = Array.from({ length: 8 }, () => [rnd(), rnd(), 0]);
  const sol = kabsch(P, P.map((p) => mulVec3(R, p).map((x, i) => x + t[i])));
  ok(maxAbs(sol.R, R) < 1e-9 && Math.abs(det3(sol.R) - 1) < 1e-9, "平面上の点でも復元できる");
  console.log(`Kabsch 法: 既知の変換の復元誤差 回転 ${worstR.toExponential(1)}, 平行移動 ${worstT.toExponential(1)} m、1 mm の雑音で回転誤差の最大 ${worstNoisy.toFixed(3)}°`);
}

// 3. 印の検出(合成画像): ずれて付いたカメラから見た印の位置を mm の精度で当てる
{
  const cam = mountedCamera(1);
  const rnd = mulberry32(21);
  let worst = 0;
  for (let k = 0; k < 8; k++) {
    const c = [0.4 + rnd() * 0.2, -0.1 + rnd() * 0.4, 0.2 + rnd() * 0.15];
    const { image, depth } = renderSynthetic({ cam, spheres: [{ c, r: MARKER_R, rgba: [0.9, 0.1, 0.8, 1] }], noise: 4, rnd });
    const m = findMarker(image, { depth, fovy: OVERHEAD.fovy, radius: MARKER_R });
    ok(m, "印が見つかる");
    // 世界座標へ: カメラ座標の点を、本当のカメラの姿勢で回して足す
    const w = mulVec3(cam.R, m.center).map((x, i) => x + cam.t[i]);
    const err = Math.hypot(w[0] - c[0], w[1] - c[1], w[2] - c[2]) * 1000;
    worst = Math.max(worst, err);
    ok(err < 5, `印の位置の誤差 ${err.toFixed(2)} mm`);
  }
  const { image, depth } = renderSynthetic({ cam, spheres: [] });
  ok(findMarker(image, { depth, fovy: OVERHEAD.fovy, radius: MARKER_R }) === null, "印がなければ null");
  console.log(`印の検出(合成画像 8 通り): 位置の誤差の最大 ${worst.toFixed(2)} mm`);
}

// 4. 合成画像で、印の組 → Kabsch → カメラの姿勢まで通す
{
  const cam = mountedCamera(1), rnd = mulberry32(31);
  const P = [], Q = [];
  for (let k = 0; k < 10; k++) {
    const c = [0.4 + rnd() * 0.2, -0.1 + rnd() * 0.4, 0.18 + (k % 2) * 0.14];
    const { image, depth } = renderSynthetic({ cam, spheres: [{ c, r: MARKER_R, rgba: [0.9, 0.1, 0.8, 1] }] });
    const m = findMarker(image, { depth, fovy: OVERHEAD.fovy, radius: MARKER_R });
    P.push(m.center); Q.push(c);
  }
  const sol = kabsch(P, Q);
  const dpos = Math.hypot(...sol.t.map((x, i) => x - cam.t[i])) * 1000, dang = rotDiffDeg(sol.R, cam.R);
  ok(dpos < 4 && dang < 0.3, `合成画像のキャリブレーション: 位置 ${dpos.toFixed(2)} mm, 向き ${dang.toFixed(3)}°`);
  console.log(`合成画像のキャリブレーション: カメラの位置のずれ ${dpos.toFixed(2)} mm, 向きのずれ ${dang.toFixed(3)}° (設計図のままなら 37.4 mm, 3.19°)`);
}

// 5. シミュレーション: A. 設計図を信じる → つかめない / B. 測り直す / C. 測り直した値 → つかめる
const mujoco = await initMujoco();
const load = nodeLoader(new URL("../_web/", import.meta.url));
{
  const e = mountError(1);
  near(e.mm, 37.4, 0.1, "ずれの大きさ(位置)"); near(e.deg, 3.19, 0.02, "ずれの大きさ(向き)");
}
let sim = await CalibrationSim.create(mujoco, load, { seed: 0, mountScale: 1 });
for (const seed of [0, 1, 2]) {
  sim.reset({ seed, mountScale: 1 });
  const t0 = performance.now();
  for (const _ of sim.run());
  const { A, B, C } = sim.stages;
  console.log(`seed=${seed}: A 誤差 ${Object.values(A.errMm).map((x) => x.toFixed(0)).join("/")} mm → 箱 ${A.ok}/4 | B ${B.n} 点 RMS ${B.rmsMm.toFixed(1)} mm、位置 ${B.before.mm.toFixed(1)}→${B.after.mm.toFixed(1)} mm、向き ${B.before.deg.toFixed(2)}→${B.after.deg.toFixed(2)}° | C 誤差 ${Object.values(C.errMm).map((x) => x.toFixed(0)).join("/")} mm → 箱 ${C.ok}/4 (${((performance.now() - t0) / 1000).toFixed(1)} 秒)`);
  ok(A.found.length === COLORS.length && C.found.length === COLORS.length, "A も C も 4 個見つかる");
  ok(Math.min(...Object.values(A.errMm)) > 40, `A: 設計図を信じると 4 つとも 4 cm 以上ずれる(最小 ${Math.min(...Object.values(A.errMm)).toFixed(0)} mm)`);
  ok(A.ok <= 1, `A: ほとんどつかめない(${A.ok}/4)`);
  ok(B.n >= 6, `B: 印が 6 か所以上で見える(${B.n})`);
  ok(B.rmsMm < 2, `B: 当てはまりの誤差 ${B.rmsMm.toFixed(2)} mm`);
  ok(B.after.deg < 0.2 && B.after.deg < B.before.deg / 10, `B: 向きのずれが 1/10 以下(${B.before.deg.toFixed(2)}° → ${B.after.deg.toFixed(2)}°)`);
  ok(B.after.mm < B.before.mm / 5, `B: 位置のずれが 1/5 以下(${B.before.mm.toFixed(1)} → ${B.after.mm.toFixed(1)} mm)`);
  ok(Math.max(...Object.values(C.errMm)) < 6, `C: 位置の誤差が小さい(最大 ${Math.max(...Object.values(C.errMm)).toFixed(1)} mm)`);
  ok(C.meanErrMm < A.meanErrMm / 5, "C の誤差は A の 1/5 未満");
  ok(C.ok === COLORS.length, `C: 4 個とも箱に入る(${C.ok}/4)`);
}
// ずれがなければ(ずれ量 0)、設計図どおりでもつかめる。ずれが 2 倍でも測り直せば つかめる
for (const scale of [0, 2]) {
  sim.reset({ seed: 3, mountScale: scale });
  for (const _ of sim.run());
  const { A, B, C } = sim.stages;
  console.log(`ずれ量 ${scale}x (${mountError(scale).mm.toFixed(1)} mm, ${mountError(scale).deg.toFixed(2)}°): A 箱 ${A.ok}/4、B 後のずれ ${B.after.mm.toFixed(1)} mm ${B.after.deg.toFixed(2)}°、C 箱 ${C.ok}/4`);
  if (scale === 0) ok(A.ok === COLORS.length, `ずれ 0 なら A でもつかめる(${A.ok}/4)`);
  if (scale === 2) ok(A.ok <= 1 && C.ok === COLORS.length, `ずれ 2 倍: A ${A.ok}/4 → C ${C.ok}/4`);
}
sim.dispose();
console.log(`確認 ${checks} 項目 OK`);
