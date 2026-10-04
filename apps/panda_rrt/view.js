// panda_rrt の画面。探索・実行は sim.js、描画と再生制御は ../_web の共通部品。
import { initMujoco, browserLoader } from "../_web/mujoco.js";
import { createViewer } from "../_web/viewer.js";
import { Runner, $ } from "../_web/runner.js";
import { lineChart } from "../_web/chart.js";
import { RrtSim, WALL, START_TCP, GOAL_TCP } from "./sim.js";

const viewer = createViewer($("view"), { camera: [1.55, -1.05, 1.0], target: [0.35, 0, 0.35] });
const { THREE } = viewer;
let sim, seed = 0, shownLogs = 0, drawn = [0, 0], shownPaths = "", drawnTrace = [0, 0], charted = false;

const PHASES = {
  ready: "スタートを押すと、探索が始まります。",
  ik: "手先の位置から、始まりと終わりの 7 つの関節角度を計算しています(逆運動学)。",
  straight: "2 つの姿勢の関節角度をまっすぐ補間すると壁にぶつかるか、MuJoCo に聞いています。",
  rrt: "RRT-Connect で道を探しています。青と緑の木が、ぶつからない姿勢だけをたどって伸びていきます(線は手先の動き)。",
  shortcut: "道がつながりました。ジグザグな道を、まっすぐつなげる所は飛ばして短くしています。",
  "exec-straight": "実行 1/2: まっすぐ補間した動き。腕が壁に当たって、それ以上進めません。",
  "exec-rrt": "実行 2/2: 見つけた道に沿って、5 次多項式でなめらかに動かします。",
  done: "完了。",
  failed: "道が見つかりませんでした(試行回数の上限)。",
};

// 伸びる線(点を足すたびに作り直す)
function polyline(color) {
  const obj = viewer.addExtra(new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color })));
  obj.frustumCulled = false;
  return { obj, set(points) { obj.geometry.dispose(); obj.geometry = new THREE.BufferGeometry().setFromPoints(points.map((p) => new THREE.Vector3(...p))); } };
}
const MAX_SEG = 40000;
function segments(color) {
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.BufferAttribute(new Float32Array(MAX_SEG * 6), 3));
  g.setDrawRange(0, 0);
  const obj = viewer.addExtra(new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color })));
  obj.frustumCulled = false;
  return obj;
}
let treeLines, rawLine, shortLine, traceLines;
function buildOverlays() {
  viewer.clearExtras();
  treeLines = [segments(0x1f6fb2), segments(0x2f8a4b)];
  rawLine = polyline(0xe69f00);
  shortLine = polyline(0xd62ea8);
  traceLines = [polyline(0xd9534f), polyline(0x16a3a3)];
  drawn = [0, 0]; shownPaths = ""; drawnTrace = [0, 0];
}

function syncOverlays() {
  sim.trees.forEach((t, k) => {
    if (t.q.length === drawn[k]) return;
    const attr = treeLines[k].geometry.attributes.position;
    for (let i = Math.max(drawn[k], 1); i < t.q.length && i < MAX_SEG; i++) {
      attr.set([...t.p[t.parent[i]], ...t.p[i]], 6 * (i - 1));
    }
    drawn[k] = t.q.length;
    attr.needsUpdate = true;
    treeLines[k].geometry.setDrawRange(0, 2 * Math.min(t.q.length - 1, MAX_SEG));
  });
  if (sim.rawPath && !shownPaths.includes("r")) { rawLine.set(sim.pathTcp(sim.rawPath)); shownPaths += "r"; }
  if (sim.path && !shownPaths.includes("s")) { shortLine.set(sim.pathTcp(sim.path)); shownPaths += "s"; }
  [sim.trace.straight, sim.trace.rrt].forEach((tr, k) => {
    if (tr.length !== drawnTrace[k] && tr.length > 1) { traceLines[k].set(tr); drawnTrace[k] = tr.length; }
  });
}

function setStatus() {
  const s = sim.stats;
  const ph = sim.phase;
  $("phase").textContent = PHASES[ph];
  const row = (label, value, cls = "") => `<li>${label}<b class="${cls}">${value}</b></li>`;
  const rows = [];
  rows.push(row("探索の試行回数", s.iters ?? "-"));
  rows.push(row("木の点の数", s.nodes ?? "-"));
  rows.push(row("計画にかかった時間", s.tPlan !== undefined ? `${s.tPlan.toFixed(2)} 秒` : "-"));
  rows.push(row("当たり判定を聞いた回数", s.calls ?? "-"));
  rows.push(row("まっすぐ補間", s.straightFree === undefined ? "-" : s.straightFree ? "ぶつからない" : "壁にぶつかる", s.straightFree === false ? "bad" : ""));
  rows.push(row("経由点の数", s.rawPoints ? `${s.rawPoints} → ${s.shortPoints ?? "…"}` : "-"));
  rows.push(row("関節空間の道のり", s.rawLen !== undefined ? `${s.rawLen.toFixed(1)} → ${s.shortLen.toFixed(1)} rad` : "-"));
  rows.push(row("実行中の接触(まっすぐ)", s.hitsStraight !== undefined ? `${s.hitsStraight} ステップ` : "-", s.hitsStraight > 0 ? "bad" : ""));
  rows.push(row("実行中の接触(RRT の道)", s.hitsRrt !== undefined ? `${s.hitsRrt} ステップ` : "-", s.hitsRrt === 0 ? "ok" : s.hitsRrt > 0 ? "bad" : ""));
  rows.push(row("着いた位置のずれ", s.err !== undefined ? `${s.err.toFixed(1)} cm` : "-"));
  $("stats").innerHTML = rows.join("");
  if (sim.log.length !== shownLogs) {
    $("log").textContent = sim.log.join("\n");
    $("log").scrollTop = $("log").scrollHeight;
    shownLogs = sim.log.length;
  }
}

function drawCharts() {
  $("charts").hidden = false;
  const wallY = [-1, 1, 1, -1, -1].map((s) => s * WALL.half[1] + WALL.pos[1]);
  const wallZ = [0, 0, 2 * WALL.half[2], 2 * WALL.half[2], 0];
  const yz = (tr) => ({ xs: tr.map((p) => p[1]), ys: tr.map((p) => p[2]) });
  lineChart($("chartPath"), [
    { name: "壁(60 cm)", xs: wallY, ys: wallZ, color: "#c07250" },
    { name: "まっすぐ補間(壁に当たる)", ...yz(sim.trace.straight), color: "#d9534f", dashed: true },
    { name: "RRT-Connect + ショートカット", ...yz(sim.trace.rrt), color: "#1f6fb2" },
    { name: "始まり", xs: [START_TCP[1]], ys: [START_TCP[2]], color: "#3399ff", points: true },
    { name: "終わり", xs: [GOAL_TCP[1]], ys: [GOAL_TCP[2]], color: "#2f8a4b", points: true },
  ], { xLabel: "y [m](前から見たところ)", yLabel: "高さ z [m]", xMin: -0.4, xMax: 0.4, yMin: 0, size: { w: 640, h: 380 } });
  const q = sim.path;
  lineChart($("chartJoint"), Array.from({ length: 7 }, (_, j) => ({ name: `j${j + 1}`, xs: q.map((_, i) => i), ys: q.map((v) => v[j]), points: true })),
    { xLabel: `経由点(${q.length} 個)`, yLabel: "角度 [rad]", size: { w: 640, h: 320 } });
}

const runner = new Runner({
  getSpeed: () => Number($("speed").value),
  onFrame: () => { syncOverlays(); viewer.sync(sim.data); setStatus(); },
  onState: (r) => { $("play").textContent = r.finished ? "もう一度" : r.running ? "一時停止" : r.task ? "再開" : "スタート"; },
  onDone: () => {
    syncOverlays(); setStatus();
    const s = sim.stats;
    if (sim.phase !== "done") { $("done").textContent = "道が見つかりませんでした"; return; }
    $("done").textContent = `壁をよけて緑の玉に着きました(接触 ${s.hitsRrt} ステップ)`;
    drawCharts();
  },
});

function resetSim() {
  sim.reset({ seed });
  viewer.build(sim.model);
  buildOverlays();
  runner.stop();
  shownLogs = 0;
  $("log").textContent = "";
  $("done").textContent = "";
  $("charts").hidden = true;
  $("seed").value = seed;
  viewer.sync(sim.data); setStatus();
}

async function main() {
  $("status").textContent = "物理エンジン(MuJoCo)を読み込み中…";
  const mujoco = await initMujoco();
  sim = await RrtSim.create(mujoco, browserLoader(new URL("../_web/", import.meta.url)), { seed });
  viewer.build(sim.model);
  buildOverlays();
  viewer.sync(sim.data); setStatus();
  $("status").hidden = true;
  $("app").hidden = false;
  $("play").onclick = () => {
    if (runner.finished) resetSim();
    if (!runner.task) runner.start(sim.run()); else runner.toggle();
  };
  $("reset").onclick = resetSim;
  $("newseed").onclick = () => { seed = Math.floor(Math.random() * 1e6); resetSim(); };
  $("seed").onchange = () => { seed = Math.max(0, Math.floor(Number($("seed").value) || 0)); resetSim(); };
  $("speed").oninput = () => { $("speedv").textContent = `${$("speed").value}x`; };
  runner.onState(runner);
  runner.loop(() => viewer.render());
  window.__app = { get sim() { return sim; }, get runner() { return runner; } }; // 動作確認用のフック
}

main().catch((e) => {
  console.error(e);
  $("status").textContent = `読み込みに失敗しました: ${e.message}`;
  $("status").className = "status error";
});
