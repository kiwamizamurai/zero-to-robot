// panda_calibration の画面。物理・キャリブレーションは sim.js、カメラ画像の描画は ../panda_vision_pick/camview.js。
import { initMujoco, browserLoader } from "../_web/mujoco.js";
import { createViewer } from "../_web/viewer.js";
import { Runner, $ } from "../_web/runner.js";
import { CalibrationSim, COLORS, NOMINAL_CAMERA, BOX_CENTER, BOX_HALF, mountError } from "./sim.js";
import { addFrustum, drawShot } from "../panda_vision_pick/camview.js";

const viewer = createViewer($("view"));
let sim, shownLogs = 0, extras = [];
const css = (name, fallback) => getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback;
const STAGE_NAMES = { A: "A. 設計図を信じる", B: "B. キャリブレーション", C: "C. 測り直した値で見つける" };
const COLOR = { true: 0x1f6fb2, nominal: 0xd9534f, est: 0x2f8a4b };

// ---- 結果の図: 机を上から見た地図 ----
const X0 = 0.25, X1 = 0.85, Y0 = -0.52, Y1 = 0.42;
function drawMap(canvas, result, { calibPts = null } = {}) {
  const W = 360, H = Math.round((W * (Y1 - Y0)) / (X1 - X0));
  const dpr = Math.min(devicePixelRatio || 1, 2);
  canvas.width = W * dpr; canvas.height = H * dpr;
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);
  const pad = { l: 40, r: 8, t: 30, b: 28 };
  const px = (x) => pad.l + ((x - X0) / (X1 - X0)) * (W - pad.l - pad.r);
  const py = (y) => pad.t + ((Y1 - y) / (Y1 - Y0)) * (H - pad.t - pad.b);
  const ink = css("--ink", "#222"), muted = css("--muted", "#777"), line = css("--line", "#ddd");
  ctx.font = "11px ui-monospace, Menlo, monospace";
  ctx.strokeStyle = line; ctx.fillStyle = muted; ctx.lineWidth = 1;
  for (let x = 0.3; x <= 0.81; x += 0.1) { ctx.beginPath(); ctx.moveTo(px(x), pad.t); ctx.lineTo(px(x), H - pad.b); ctx.stroke(); ctx.textAlign = "center"; ctx.fillText(x.toFixed(1), px(x), H - pad.b + 14); }
  for (let y = -0.4; y <= 0.41; y += 0.2) { ctx.beginPath(); ctx.moveTo(pad.l, py(y)); ctx.lineTo(W - pad.r, py(y)); ctx.stroke(); ctx.textAlign = "right"; ctx.fillText(y.toFixed(1), pad.l - 5, py(y) + 4); }
  ctx.textAlign = "center"; ctx.fillText("x [m]", (pad.l + W - pad.r) / 2, H - 4);
  // 箱
  ctx.strokeStyle = "#8b5a2b"; ctx.lineWidth = 2;
  ctx.strokeRect(px(BOX_CENTER[0] - BOX_HALF), py(BOX_CENTER[1] + BOX_HALF), px(BOX_CENTER[0] + BOX_HALF) - px(BOX_CENTER[0] - BOX_HALF), py(BOX_CENTER[1] - BOX_HALF) - py(BOX_CENTER[1] + BOX_HALF));
  if (!result) {
    ctx.fillStyle = muted; ctx.textAlign = "center"; ctx.font = "13px sans-serif";
    ctx.fillText("まだ結果がありません", W / 2, H / 2);
    return;
  }
  const half = 0.02;
  for (const b of result.blocks) { // 四角 = 本当のブロック
    const [r, g, bl] = b.rgba.slice(0, 3).map((x) => Math.round(x * 255));
    ctx.fillStyle = `rgba(${r},${g},${bl},0.4)`;
    ctx.fillRect(px(b.xy[0] - half), py(b.xy[1] + half), px(b.xy[0] + half) - px(b.xy[0] - half), py(b.xy[1] - half) - py(b.xy[1] + half));
    ctx.strokeStyle = ink; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(px(b.xy[0]) - 4, py(b.xy[1])); ctx.lineTo(px(b.xy[0]) + 4, py(b.xy[1])); ctx.moveTo(px(b.xy[0]), py(b.xy[1]) - 4); ctx.lineTo(px(b.xy[0]), py(b.xy[1]) + 4); ctx.stroke();
  }
  for (const f of result.found) { // 丸 = カメラが「ここ」と思った位置、矢印 = そのずれ
    const b = result.blocks.find((x) => x.name === f.name);
    const [x0, y0, x1, y1] = [px(b.xy[0]), py(b.xy[1]), px(f.pos[0]), py(f.pos[1])];
    ctx.strokeStyle = ink; ctx.fillStyle = ink; ctx.lineWidth = 1.3;
    ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
    const a = Math.atan2(y1 - y0, x1 - x0);
    if (Math.hypot(x1 - x0, y1 - y0) > 8) {
      ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x1 - 7 * Math.cos(a - 0.4), y1 - 7 * Math.sin(a - 0.4)); ctx.lineTo(x1 - 7 * Math.cos(a + 0.4), y1 - 7 * Math.sin(a + 0.4)); ctx.closePath(); ctx.fill();
    }
    const c = COLORS.find((q) => q.name === f.name).rgba.slice(0, 3).map((q) => Math.round(q * 255));
    ctx.fillStyle = `rgb(${c})`; ctx.beginPath(); ctx.arc(x1, y1, 5, 0, 7); ctx.fill(); ctx.strokeStyle = ink; ctx.stroke();
  }
  if (calibPts) {
    ctx.strokeStyle = "#c0239f"; ctx.lineWidth = 1.5;
    for (const p of calibPts) { const x = px(p[0]), y = py(p[1]); ctx.beginPath(); ctx.moveTo(x - 4, y - 4); ctx.lineTo(x + 4, y + 4); ctx.moveTo(x - 4, y + 4); ctx.lineTo(x + 4, y - 4); ctx.stroke(); }
  }
  ctx.fillStyle = ink; ctx.textAlign = "left"; ctx.font = "13px sans-serif";
  ctx.fillText(`平均の誤差 ${result.meanErrMm.toFixed(0)} mm、箱に入った ${result.ok}/${COLORS.length}`, pad.l, 18);
}

// ---- 表示の更新 ----
function setStatus() {
  $("time").textContent = `${sim.time.toFixed(1)} 秒`;
  const st = sim.stages;
  const line = (key, text) => {
    const cls = st[key] ? "ok" : "";
    const state = st[key] ? text(st[key]) : sim.stage === key ? "実行中…" : "待機中";
    return `<li>${STAGE_NAMES[key]}<b class="${cls}">${state}</b></li>`;
  };
  $("stages").innerHTML = [
    line("A", (r) => `箱に ${r.ok}/${COLORS.length}`),
    line("B", (r) => (r ? `${r.n} 点で解いた` : "解けなかった")),
    line("C", (r) => `箱に ${r.ok}/${COLORS.length}`),
  ].join("");
  if (sim.log.length !== shownLogs) {
    $("log").textContent = sim.log.join("\n");
    $("log").scrollTop = $("log").scrollHeight;
    shownLogs = sim.log.length;
  }
}

function summary() {
  const { A, B, C } = sim.stages;
  const err = (r) => Object.entries(r.errMm).map(([n, e]) => `${n} ${e.toFixed(0)}`).join(" / ");
  const res = (r) => `<span class="${r.ok === COLORS.length ? "good" : "badc"}">箱に ${r.ok} / ${COLORS.length} 個</span>`;
  const rows = [];
  if (A) rows.push(`<tr><td>A. 設計図を信じる</td><td>設計図どおりの位置・向きだと信じて、画像から位置を出してつかみに行く</td><td class="num">見つけた位置の誤差 [mm]: ${err(A)}<br>${res(A)}</td></tr>`);
  if (B) {
    rows.push(`<tr><td>B. キャリブレーション</td><td>手の球を ${12} か所へ動かし、見えた ${B.n} 点の組から Kabsch 法(SVD)でカメラの位置・向きを求める</td><td class="num">当てはまりの誤差(RMS) ${B.rmsMm.toFixed(1)} mm<br>位置のずれ ${B.before.mm.toFixed(1)} → ${B.after.mm.toFixed(1)} mm<br>向きのずれ ${B.before.deg.toFixed(2)}° → ${B.after.deg.toFixed(2)}°</td></tr>`);
  } else if (A && sim.stage !== "A" && sim.stage !== "B") {
    rows.push(`<tr><td>B. キャリブレーション</td><td>球が 3 か所以上で見えなかった</td><td>—</td></tr>`);
  }
  if (C) rows.push(`<tr><td>C. 測り直した値で見つける</td><td>求めた位置・向きで画像から位置を出し直して、つかみに行く</td><td class="num">見つけた位置の誤差 [mm]: ${err(C)}<br>${res(C)}</td></tr>`);
  $("summary").innerHTML = rows.join("");
  $("summary-wrap").hidden = !rows.length;
}

function drawFrustums(est = null) {
  viewer.clearExtras();
  extras = [
    ...addFrustum(viewer, sim.trueCamera(), { fovy: sim.cameraCfg.fovy, color: COLOR.true }),
    ...addFrustum(viewer, NOMINAL_CAMERA, { fovy: sim.cameraCfg.fovy, color: COLOR.nominal }),
    ...(est ? addFrustum(viewer, est, { fovy: sim.cameraCfg.fovy, color: COLOR.est }) : []),
  ];
}

function updateScaleLabel() {
  const e = mountError(Number($("scale").value));
  $("scalev").textContent = `${$("scale").value}x(${e.mm.toFixed(1)} mm, ${e.deg.toFixed(1)}°)`;
}

const runner = new Runner({
  getSpeed: () => Number($("speed").value),
  onFrame: () => { viewer.sync(sim.data); setStatus(); },
  onState: (r) => { $("play").textContent = r.finished ? "もう一度" : r.running ? "一時停止" : r.task ? "再開" : "スタート"; },
  onDone: () => {
    const { A, C } = sim.stages;
    $("done").textContent = `完了: 設計図を信じると ${A.ok}/${COLORS.length} 個、測り直すと ${C.ok}/${COLORS.length} 個が箱に入りました`;
    setStatus(); summary();
  },
});

function attachCallbacks() {
  sim.onShot = (shot) => {
    drawShot($("cam"), shot.image, { found: shot.found, title: `${sim.stage === "C" ? "C 測り直し後" : "A 設計図どおり"}の見つけ方` });
    $("camtitle").textContent = "天井カメラの画像(ブロックの検出)";
    $("camcap").textContent = "白い枠が見つけたブロックの輪郭。左下の数字は、信じているカメラの位置・向きで直した机の上の座標です。";
  };
  sim.onMarker = (pt, all) => {
    drawShot($("cam"), pt.image, { title: `marker ${pt.seen ? "seen" : "hidden"} (${all.filter((p) => p.seen).length} points)` });
    if (pt.uv) {
      const ctx = $("cam").getContext("2d");
      ctx.strokeStyle = "#fff"; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(pt.uv[0], pt.uv[1], 11, 0, 7); ctx.moveTo(pt.uv[0] - 16, pt.uv[1]); ctx.lineTo(pt.uv[0] + 16, pt.uv[1]); ctx.moveTo(pt.uv[0], pt.uv[1] - 16); ctx.lineTo(pt.uv[0], pt.uv[1] + 16); ctx.stroke();
    }
    $("camtitle").textContent = `キャリブレーション(${pt.k + 1} / 12 か所目)`;
    $("camcap").textContent = pt.seen ? "マゼンタの球が見えました。ロボットが知っている球の位置と、この画像から測った位置を 1 組として記録します。" : "球が腕に隠れたか視界の外で、この点は使いません。";
  };
  sim.onStage = (key, result) => {
    if (key === "B" && result) drawFrustums(result.est);
    if (key === "A" && result) drawMap($("mapA"), result);
    if (key === "C" && result) drawMap($("mapC"), result, { calibPts: sim.stages.B?.points });
    setStatus(); summary();
  };
}

function resetSim() {
  const seed = Math.max(0, Math.floor(Number($("seed").value) || 0));
  sim.reset({ seed, mountScale: Number($("scale").value) });
  attachCallbacks();
  viewer.build(sim.model);
  drawFrustums();
  runner.stop();
  shownLogs = 0;
  $("log").textContent = "";
  $("done").textContent = "";
  $("camtitle").textContent = "天井カメラの画像";
  $("camcap").textContent = "スタートすると、まず設計図を信じて片付けを試し、次に球を動かして測り直し、最後に測り直した値でもう一度試します。";
  const pre = sim.ray.shoot(sim.model, sim.data, sim.trueCamera());
  drawShot($("cam"), pre.image, { title: "overhead camera (撮影前)" });
  drawMap($("mapA"), null); drawMap($("mapC"), null);
  $("summary").innerHTML = ""; $("summary-wrap").hidden = true;
  updateScaleLabel();
  viewer.sync(sim.data);
  setStatus();
}

async function main() {
  $("status").textContent = "物理エンジン(MuJoCo)を読み込み中…";
  const mujoco = await initMujoco();
  sim = await CalibrationSim.create(mujoco, browserLoader(new URL("../_web/", import.meta.url)), { seed: 0, mountScale: 1 });
  $("status").hidden = true;
  $("app").hidden = false;
  $("below").hidden = false;
  resetSim();
  $("play").onclick = () => {
    if (runner.finished) resetSim();
    if (!runner.task) runner.start(sim.run()); else runner.toggle();
  };
  $("reset").onclick = resetSim;
  $("seed").onchange = resetSim;
  $("scale").oninput = updateScaleLabel;
  $("scale").onchange = resetSim;
  $("reseed").onclick = () => { $("seed").value = Math.floor(Math.random() * 10000); resetSim(); };
  runner.onState(runner);
  runner.loop(() => viewer.render());
  window.__app = { get sim() { return sim; }, get runner() { return runner; }, get extras() { return extras; } }; // 動作確認用のフック
}

main().catch((e) => {
  console.error(e);
  $("status").hidden = false;
  $("status").textContent = `読み込みに失敗しました: ${e.message}`;
  $("status").className = "status error";
});
