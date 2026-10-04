// panda_impedance の画面。物理は sim.js、描画と再生制御は ../_web の共通部品。
import { initMujoco, browserLoader } from "../_web/mujoco.js";
import { createViewer } from "../_web/viewer.js";
import { Runner, $ } from "../_web/runner.js";
import { lineChart } from "../_web/chart.js";
import { ImpedanceSim, MODES, CENTER, RADIUS, FORCE_GOAL, TABLE_Z, T_END } from "./sim.js";

const viewer = createViewer($("view"), { camera: [1.25, -0.75, 0.75], target: [0.45, 0, 0.22] });
const { THREE } = viewer;
const COLORS = { stiff: "#d9542f", soft: "#1f6fb2" };
const MODE_NOTE = {
  stiff: "ばねが硬いので、机の高さが少しずれるだけで力が大きく変わります(位置の指令に近い状態)。",
  soft: "ばねが柔らかいので、机の高さがずれても押す力はあまり変わりません。",
};
let sim, mujoco, mode = "soft", live = { xs: [], ys: [], n: 0 }, inkMesh, goalMesh;
const MAX_INK = 8000;

function offsetM() { return Number($("offset").value) / 1000; }

function setupExtras() {
  viewer.clearExtras();
  // 机にできた線(青い点)
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.BufferAttribute(new Float32Array(MAX_INK * 3), 3));
  g.setDrawRange(0, 0);
  inkMesh = viewer.addExtra(new THREE.Points(g, new THREE.PointsMaterial({ color: 0x1a3fcc, size: 5, sizeAttenuation: false })));
  inkMesh.frustumCulled = false;
  // 目標の位置(机の中に沈んでいることがある赤い球)。手前に描く
  goalMesh = viewer.sphere([0, 0, 0], 0.008, 0xd9342f);
  goalMesh.material.depthTest = false;
  goalMesh.renderOrder = 10;
}

function updateExtras() {
  const n = Math.min(sim.ink.length, MAX_INK), pos = inkMesh.geometry.attributes.position;
  for (let i = live.inkN ?? 0; i < n; i++) pos.setXYZ(i, sim.ink[i][0], sim.ink[i][1], TABLE_Z + sim.offset + 0.0015);
  live.inkN = n;
  pos.needsUpdate = true;
  inkMesh.geometry.setDrawRange(0, n);
  goalMesh.position.set(...sim.goalAt(sim.data.time));
}

function drawLive() {
  const h = sim.history;
  for (; live.n < h.length; live.n += 5) {
    const i = live.n, lo = Math.max(0, i - 9);
    let s = 0;
    for (let k = lo; k <= i; k++) s += h[k].f;
    live.xs.push(h[i].t); live.ys.push(s / (i - lo + 1));
  }
  lineChart($("chart"), [
    { name: MODES[sim.mode].name, xs: live.xs.length ? live.xs : [0], ys: live.ys.length ? live.ys : [0], color: COLORS[sim.mode] },
    { name: "目標 10 N", xs: [0, T_END], ys: [FORCE_GOAL, FORCE_GOAL], color: "#888", dashed: true },
  ], { xLabel: "時間 [秒]", yLabel: "押す力 [N]", xMin: 0, xMax: T_END, yMin: 0, yMax: 100, size: { w: 760, h: 260 } });
}

function setStatus() {
  $("time").textContent = `${sim.time.toFixed(1)} 秒`;
  $("force").textContent = sim.press.toFixed(1);
}

const runner = new Runner({
  getSpeed: () => Number($("speed").value),
  onFrame: () => { viewer.sync(sim.data); updateExtras(); setStatus(); drawLive(); },
  onState: (r) => { $("play").textContent = r.finished ? "もう一度" : r.running ? "一時停止" : r.task ? "再開" : "スタート"; },
  onDone: () => {
    const s = sim.summary();
    $("done").textContent = `円を描いている間の平均の力 ${s.mean.toFixed(1)} N、描けた円周 ${s.covered.toFixed(0)} %`;
  },
});

function resetSim() {
  sim.reset({ mode, offset: offsetM() });
  viewer.build(sim.model);
  setupExtras();
  runner.stop();
  live = { xs: [], ys: [], n: 0 };
  $("done").textContent = "";
  viewer.sync(sim.data); updateExtras(); setStatus(); drawLive();
}

function setMode(m) {
  mode = m;
  for (const k of ["stiff", "soft"]) $(`mode-${k}`).setAttribute("aria-pressed", String(k === m));
  $("modenote").textContent = MODE_NOTE[m];
  if (sim) resetSim();
}

// --- 6 通りの比較(force_compare.png に相当) ---
const OFFS = [-0.01, 0, 0.01], WHERE = { "-0.01": "机が 1 cm 低い", 0: "机が見込みどおり", 0.01: "机が 1 cm 高い" };

function drawInk(canvas, results, off) {
  const w = 320, h = 260, dpr = Math.min(devicePixelRatio || 1, 2);
  canvas.width = w * dpr; canvas.height = h * dpr;
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const sc = 1000, cx = w / 2, cy = h / 2; // 1 m = 1000 px(半径 8 cm = 80 px)
  const X = (x) => cx + (x - CENTER[0]) * sc * 1.4, Y = (y) => cy - (y - CENTER[1]) * sc * 1.4;
  ctx.strokeStyle = "#888"; ctx.setLineDash([5, 4]); ctx.lineWidth = 1;
  ctx.beginPath(); ctx.arc(cx, cy, RADIUS * sc * 1.4, 0, 7); ctx.stroke(); ctx.setLineDash([]);
  for (const [m, size, alpha] of [["stiff", 5, 0.5], ["soft", 2, 1]]) {
    ctx.fillStyle = COLORS[m]; ctx.globalAlpha = alpha;
    for (const p of results[`${m}${off}`].ink) ctx.fillRect(X(p[0]) - size / 2, Y(p[1]) - size / 2, size, size);
  }
  ctx.globalAlpha = 1;
}

function runCompare() {
  $("cmp-wait").textContent = "計算中…(数秒)";
  setTimeout(() => {
    const cmp = new ImpedanceSim(mujoco, {}), results = {};
    for (const m of ["stiff", "soft"]) for (const off of OFFS) {
      cmp.reset({ mode: m, offset: off });
      for (const _ of cmp.run());
      const times = cmp.history.map((r) => r.t), f = cmp.smoothForce();
      results[`${m}${off}`] = { ...cmp.summary(), ink: cmp.ink.slice(), times, f };
    }
    cmp.dispose();
    $("cmp-lines").innerHTML = OFFS.map((o, i) => `<figure><canvas id="cl${i}"></canvas><figcaption>${WHERE[o]}</figcaption></figure>`).join("");
    $("cmp-inks").innerHTML = OFFS.map((o, i) => `<figure><canvas id="ci${i}"></canvas><figcaption>机に残った線(太い赤: 硬い、細い青: 柔らかい、破線: 描きたい円)</figcaption></figure>`).join("");
    OFFS.forEach((o, i) => {
      const ser = ["stiff", "soft"].map((m) => {
        const r = results[`${m}${o}`], xs = [], ys = [];
        for (let k = 0; k < r.f.length; k += 5) { xs.push(r.times[k]); ys.push(r.f[k]); }
        return { name: m === "stiff" ? "硬い 8000 N/m" : "柔らかい 400 N/m", xs, ys, color: COLORS[m] };
      });
      ser.push({ name: "目標 10 N", xs: [0, T_END], ys: [FORCE_GOAL, FORCE_GOAL], color: "#888", dashed: true });
      lineChart($(`cl${i}`), ser, { xLabel: "時間 [秒]", yLabel: "押す力 [N]", xMin: 0, xMax: T_END, yMin: 0, yMax: 100, size: { w: 400, h: 280 } });
      drawInk($(`ci${i}`), results, o);
    });
    const rows = ["stiff", "soft"].flatMap((m) => OFFS.map((o) => {
      const r = results[`${m}${o}`];
      return `<tr><td>${m === "stiff" ? "硬い" : "柔らかい"}</td><td>${WHERE[o]}</td><td>${r.mean.toFixed(1)} N</td><td>${r.max.toFixed(1)} N</td><td>${r.touching.toFixed(0)} %</td><td>${r.covered.toFixed(0)} %</td></tr>`;
    })).join("");
    $("cmp-table").innerHTML = `<tr><th>制御</th><th>机の高さ</th><th>平均の力</th><th>最大の力</th><th>触れていた時間</th><th>描けた円周</th></tr>${rows}`;
    const g = (k) => results[k].mean;
    $("cmp-claim").textContent = `位置の指令に近い硬い制御では、机が 1 cm 低いと力は ${g("stiff-0.01").toFixed(0)} N(一度も触れない)、1 cm 高いと ${g("stiff0.01").toFixed(0)} N まで跳ね上がります。ばねのように振る舞う柔らかい制御では、同じずれでも ${g("soft-0.01").toFixed(1)}〜${g("soft0.01").toFixed(1)} N に収まります。`;
    $("cmp-out").hidden = false;
    $("cmp-wait").textContent = "";
  }, 30);
}

async function main() {
  $("status").textContent = "物理エンジン(MuJoCo)を読み込み中…";
  mujoco = await initMujoco();
  sim = await ImpedanceSim.create(mujoco, browserLoader(new URL("../_web/", import.meta.url)), {});
  $("status").hidden = true;
  $("app").hidden = false; $("live").hidden = false; $("compare").hidden = false;
  setMode("soft");
  resetSim();
  $("play").onclick = () => {
    if (runner.finished) resetSim();
    if (!runner.task) runner.start(sim.run()); else runner.toggle();
  };
  $("reset").onclick = resetSim;
  $("mode-stiff").onclick = () => setMode("stiff");
  $("mode-soft").onclick = () => setMode("soft");
  $("offset").oninput = () => { $("offsetv").textContent = `${$("offset").value} mm`; resetSim(); };
  $("speed").oninput = () => { $("speedv").textContent = `${$("speed").value}x`; };
  $("compare-run").onclick = runCompare;
  runner.onState(runner);
  runner.loop(() => viewer.render());
  window.__app = { get sim() { return sim; }, get runner() { return runner; } };
}

main().catch((e) => {
  console.error(e);
  $("status").hidden = false;
  $("status").textContent = `読み込みに失敗しました: ${e.message}`;
  $("status").className = "status error";
});
