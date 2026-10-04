// drone_flight の画面。物理と制御は sim.js、描画と再生制御は ../_web の共通部品。
import { initMujoco, browserLoader } from "../_web/mujoco.js";
import { createViewer } from "../_web/viewer.js";
import { Runner, $ } from "../_web/runner.js";
import { lineChart } from "../_web/chart.js";
import { DroneSim, T_TOTAL } from "./sim.js";

const viewer = createViewer($("view"), { camera: [3.6, -2.4, 2.0], target: [0.6, 0.6, 0.8] });
const { THREE } = viewer;
let sim, weak = false, trail, trailN = 0, frames = 0, chartedTo = 0;
const MAX_TRAIL = 2200;

const NOTES = {
  normal: "内側の姿勢の制御が十分に速いので、傾けた分だけすぐ追いつき、ルートどおりに飛べます。",
  weak: "内側の姿勢の制御を 1/10 に弱めます。傾きが目標に追いつかず、行きすぎては戻しすぎて揺れが育ち、墜落します。",
};

function makeTrail() {
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.BufferAttribute(new Float32Array(3 * MAX_TRAIL), 3));
  g.setDrawRange(0, 0);
  trail = viewer.addExtra(new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0x1f6fb2 })));
  trail.frustumCulled = false;
}

function setStatus() {
  const L = sim.log, last = L[L.length - 1];
  $("time").textContent = `${sim.time.toFixed(1)} 秒`;
  $("err").textContent = last ? (last.err * 100).toFixed(1) : "0.0";
  $("alt").textContent = `${sim.data.qpos[2].toFixed(2)} m`;
  $("tilt").textContent = `${(last ? last.tilt : 0).toFixed(1)}°`;
}

function drawCharts() {
  const L = sim.log, step = 4;
  const xs = [], ys = { z: [], zr: [], tilt: [], err: [] };
  for (let i = 0; i < L.length; i += step) {
    const r = L[i];
    xs.push(r.t); ys.z.push(r.pos[2]); ys.zr.push(r.ref[2]); ys.tilt.push(r.tilt); ys.err.push(r.err * 100);
  }
  chartedTo = L.length;
  const size = { w: 400, h: 260 }, o = { xLabel: "時間 [秒]", xMin: 0, xMax: T_TOTAL, size };
  lineChart($("c-alt"), [{ name: "目標", xs, ys: ys.zr, dashed: true, color: "#8a8f92" }, { name: "実際", xs, ys: ys.z }], { ...o, yMin: 0, yMax: 1.6 });
  lineChart($("c-tilt"), [{ name: "傾き", xs, ys: ys.tilt }], { ...o, yMin: 0, yMax: 90 });
  lineChart($("c-err"), [{ name: "ずれ", xs, ys: ys.err, color: "#d9534f" }], { ...o, yMin: 0, yMax: 60 });
}

function updateTrail() {
  const L = sim.log;
  if (trailN >= L.length || trailN >= MAX_TRAIL) return;
  const a = trail.geometry.attributes.position.array;
  for (; trailN < L.length && trailN < MAX_TRAIL; trailN++) a.set(L[trailN].pos, 3 * trailN);
  trail.geometry.attributes.position.needsUpdate = true;
  trail.geometry.setDrawRange(0, trailN);
}

function follow() {
  if ($("cam").value !== "follow") return;
  const p = sim.data.qpos, c = viewer.controls.target;
  const dx = p[0] - c.x, dy = p[1] - c.y, dz = p[2] - c.z;
  viewer.camera.position.x += dx; viewer.camera.position.y += dy; viewer.camera.position.z += dz;
  c.set(p[0], p[1], p[2]);
}

function showResult() {
  const r = sim.results();
  if (!r) return;
  const pct = (x) => `${x.toFixed(1)} cm`;
  $("done").textContent = r.crashed ? `墜落しました(${r.time.toFixed(1)} 秒)` : `最後まで飛びました(${r.time.toFixed(1)} 秒)`;
  $("done").className = r.crashed ? "result bad" : "result";
  const rows = [["目標とのずれ(平均)", pct(r.meanErr)], ["目標とのずれ(最大)", pct(r.maxErr)]];
  if (r.gustErr != null) rows.push(["突風での最大のずれ", pct(r.gustErr)], ["突風での最大の傾き", `${r.gustTilt.toFixed(1)}°`]);
  rows.push(["最後の位置", `(${r.landing.map((x) => (x >= 0 ? "+" : "") + x.toFixed(2)).join(", ")}) m`]);
  $("summary").innerHTML = rows.map(([k, v]) => `<li>${k}<b>${v}</b></li>`).join("");
}

const runner = new Runner({
  stepsPerSec: 100, // 物理 1 ステップ = 0.01 秒なので、速さ 1x で実時間
  getSpeed: () => Number($("speed").value),
  onFrame: () => {
    viewer.sync(sim.data); updateTrail(); follow(); setStatus();
    if (++frames % 8 === 0 && sim.log.length > chartedTo) drawCharts();
  },
  onState: (r) => { $("play").textContent = r.finished ? "もう一度" : r.running ? "一時停止" : r.task ? "再開" : "スタート"; },
  onDone: () => { drawCharts(); showResult(); },
});

function resetSim() {
  sim.reset({ weak });
  runner.stop();
  trailN = 0; trail.geometry.setDrawRange(0, 0); chartedTo = 0;
  $("done").textContent = ""; $("summary").innerHTML = "";
  for (const id of ["c-alt", "c-tilt", "c-err"]) $(id).getContext("2d").clearRect(0, 0, 1e5, 1e5);
  viewer.sync(sim.data); setStatus();
}

function setMode(w) {
  weak = w;
  $("mode-normal").setAttribute("aria-pressed", String(!w));
  $("mode-weak").setAttribute("aria-pressed", String(w));
  $("modenote").textContent = NOTES[w ? "weak" : "normal"];
  resetSim();
}

async function main() {
  $("status").textContent = "物理エンジン(MuJoCo)を読み込み中…";
  const mujoco = await initMujoco();
  sim = await DroneSim.create(mujoco, browserLoader(new URL("../_web/", import.meta.url)));
  // 衝突用(group 3)の箱や楕円体は描かない。機体のメッシュと床、巡回点の目印だけ描く
  viewer.build(sim.model, { hide: (i) => sim.model.geom_group[i] === 3 });
  makeTrail();
  viewer.sync(sim.data); setStatus();
  $("status").hidden = true;
  $("app").hidden = false;
  $("charts").hidden = false;
  $("modenote").textContent = NOTES.normal;
  $("play").onclick = () => {
    if (runner.finished) resetSim();
    if (!runner.task) runner.start(sim.run()); else runner.toggle();
  };
  $("reset").onclick = resetSim;
  $("mode-normal").onclick = () => setMode(false);
  $("mode-weak").onclick = () => setMode(true);
  $("speed").oninput = () => { $("speedv").textContent = `${$("speed").value}x`; };
  $("cam").onchange = () => { viewer.controls.target.set(...($("cam").value === "follow" ? Array.from(sim.data.qpos.slice(0, 3)) : [0.6, 0.6, 0.8])); };
  $("gust").onclick = () => {
    if (runner.finished) resetSim();
    if (!runner.task) runner.start(sim.run()); // 飛んでいないときは、まず飛ばす
    else if (!runner.running && !runner.finished) runner.toggle();
    sim.gust($("gustdir").value.split(",").map(Number), 1.0);
  };
  runner.onState(runner);
  runner.loop(() => viewer.render());
  window.__app = { get sim() { return sim; }, get runner() { return runner; } }; // 動作確認用のフック
}

main().catch((e) => {
  console.error(e);
  $("status").textContent = `読み込みに失敗しました: ${e.message}`;
  $("status").className = "status error";
});
