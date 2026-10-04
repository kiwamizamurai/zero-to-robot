// tomato_harvest の画面。物理と収穫の手順は sim.js、描画と再生制御は ../_web の共通部品。
import { initMujoco, browserLoader } from "../_web/mujoco.js";
import { createViewer } from "../_web/viewer.js";
import { Runner, $ } from "../_web/runner.js";
import { TomatoSO101, FRUITS, EYE, ripenessRgba, detectFruits, harvestRun, harvestResult } from "./sim.js";

const viewer = createViewer($("view"), { camera: [0.62, -0.48, 0.42], target: [0.18, -0.05, 0.10] });
let sim, run = null, phase = "待機";
const logLines = [];
const eyeCtx = $("eye").getContext("2d");

function params() {
  return { threshold: Number($("thr").value) / 100, satMin: Number($("sat").value), hueMax: Number($("hue").value) };
}

function showParams() {
  $("thrv").textContent = `${$("thr").value}% 以上`;
  $("satv").textContent = `${$("sat").value} 以上(0〜255)`;
  $("huev").textContent = `${$("hue").value} 以下(0〜180。${Number($("hue").value) * 2}° 相当)`;
}

// 実を見るカメラの画像。three.js の通常の描き方(影・照明つき)だと実の色が変わってしまうので、
// 一時的に「照明なし・色そのまま」の材質に差し替えて撮る。実機のカメラの代わり
function look() {
  viewer.sync(sim.data);
  const { THREE, scene } = viewer;
  const saved = [], temp = [], bg = scene.background;
  scene.traverse((o) => {
    if (o.isMesh && o.material?.color) {
      saved.push([o, o.material]);
      const m = new THREE.MeshBasicMaterial({ color: o.material.color.clone() });
      temp.push(m);
      o.material = m;
    }
  });
  scene.background = new THREE.Color(0.8, 0.78, 0.72);
  try {
    return viewer.capture(EYE.pos, EYE.lookAt, { width: EYE.width, height: EYE.height, fov: EYE.fovy });
  } finally {
    for (const [o, m] of saved) o.material = m;
    for (const m of temp) m.dispose();
    scene.background = bg;
  }
}

// 画像を描いて、検出した実に枠と数字を重ねる
function drawEye(img, found) {
  const { width: w, height: h } = img;
  const out = new ImageData(w, h);
  for (let y = 0; y < h; y++) {
    const sy = img.flipY ? h - 1 - y : y;
    out.data.set(img.data.subarray(4 * sy * w, 4 * (sy + 1) * w), 4 * y * w);
  }
  eyeCtx.putImageData(out, 0, 0);
  eyeCtx.font = "13px 'Noto Sans JP', sans-serif";
  for (const f of found) {
    const [x, y, bw, bh] = f.box;
    eyeCtx.lineWidth = 2;
    eyeCtx.strokeStyle = f.harvest ? "#ffffff" : "#ffa030";
    eyeCtx.strokeRect(x - 3, y - 3, bw + 6, bh + 6);
    const text = `${Math.round(f.ripeness * 100)}% ${f.harvest ? "収穫" : "まだ"}`;
    eyeCtx.lineWidth = 3; eyeCtx.strokeStyle = "#000"; eyeCtx.strokeText(text, x + bw + 6, y + bh / 2 + 4);
    eyeCtx.fillStyle = "#fff"; eyeCtx.fillText(text, x + bw + 6, y + bh / 2 + 4);
  }
}

function detectNow() { // 動いていないときに、しきい値を変えた結果をすぐ見せる
  if (run && !runner.finished) return;
  const img = look();
  drawEye(img, detectFruits(img, params()));
}

function setStatus() {
  $("time").textContent = `${sim.time.toFixed(1)} 秒`;
  $("phase").textContent = phase;
  const res = harvestResult(sim);
  $("fruits").innerHTML = FRUITS.map(([name, , , , r]) => {
    const [cr, cg, cb] = ripenessRgba(r).slice(0, 3).map((x) => Math.round(x * 255));
    const s = res[name];
    const state = s.basket ? "かごの中" : s.picked ? "採ったが、かごの外" : "木に残っている";
    return `<li><span class="dot" style="background:rgb(${cr},${cg},${cb})"></span>${name}(熟し具合 ${r.toFixed(2)})<b class="${s.basket ? "ok" : s.picked ? "bad" : ""}">${state}</b></li>`;
  }).join("");
}

function* task() {
  yield* harvestRun(sim, {
    look,
    params,
    onLook: (found, img) => drawEye(img, found),
    onLog: (t) => { logLines.push(t); $("log").textContent = logLines.join("\n"); $("log").scrollTop = $("log").scrollHeight; },
    onStep: (_, label) => { phase = label; },
  });
}

const runner = new Runner({
  stepsPerSec: 40,
  getSpeed: () => Number($("speed").value),
  onFrame: () => { viewer.sync(sim.data); setStatus(); },
  onState: (r) => { $("play").textContent = r.finished ? "もう一度" : r.running ? "一時停止" : r.task ? "再開" : "スタート"; },
  onDone: () => {
    phase = "完了";
    setStatus();
    const res = harvestResult(sim);
    const ripe = FRUITS.filter(([, , , , r]) => r >= 0.8).length;
    const inB = Object.values(res).filter((s) => s.basket).length;
    const left = Object.values(res).filter((s) => !s.picked).length;
    $("done").textContent = `完了: かごに ${inB} 個(熟した実は全部で ${ripe} 個)、木に ${left} 個残しました`;
  },
});

function resetSim() {
  runner.stop();
  sim.build();
  sim.connect();
  viewer.build(sim.model);
  run = null; phase = "待機"; logLines.length = 0;
  $("log").textContent = ""; $("done").textContent = "";
  viewer.sync(sim.data); setStatus();
  detectNow();
}

async function main() {
  $("status").textContent = "物理エンジン(MuJoCo)を読み込み中…";
  const mujoco = await initMujoco();
  sim = await TomatoSO101.create(mujoco, browserLoader(new URL("../_web/", import.meta.url)), { maxRelativeTarget: 10 });
  sim.connect();
  showParams();
  viewer.build(sim.model); viewer.sync(sim.data); setStatus();
  $("status").hidden = true;
  $("app").hidden = false;
  viewer.render(); // 画面を一度描いてから、カメラ画像を撮る
  detectNow();
  $("play").onclick = () => {
    if (runner.finished) resetSim();
    if (!runner.task) { run = task(); runner.start(run); } else runner.toggle();
  };
  $("reset").onclick = resetSim;
  for (const id of ["thr", "sat", "hue"]) $(id).oninput = () => { showParams(); detectNow(); };
  $("speed").oninput = () => { $("speedv").textContent = `${$("speed").value}x`; };
  runner.onState(runner);
  runner.loop(() => viewer.render());
  window.__app = { get sim() { return sim; }, get runner() { return runner; }, look, detect: () => detectFruits(look(), params()) }; // 動作確認用のフック
}

main().catch((e) => {
  console.error(e);
  $("status").textContent = `読み込みに失敗しました: ${e.message}`;
  $("status").className = "status error";
});
