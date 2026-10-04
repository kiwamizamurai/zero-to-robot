// so101_twin の画面。物理は sim.js、描画と再生制御は ../_web の共通部品。
import { initMujoco, browserLoader } from "../_web/mujoco.js";
import { createViewer } from "../_web/viewer.js";
import { Runner, $ } from "../_web/runner.js";
import { lineChart } from "../_web/chart.js";
import { SimSO101, MOTORS, planAndRun, checkLog, logToCsv } from "./sim.js";

const viewer = createViewer($("view"), { camera: [0.62, 0.52, 0.42], target: [0.16, -0.03, 0.07] });
let sim, log = [], phase = "待機", frames = 0;

const NAMES = { shoulder_pan: "肩の回転", shoulder_lift: "肩の上げ下げ", elbow_flex: "ひじ", wrist_flex: "手首の曲げ", wrist_roll: "手首のひねり", gripper: "グリッパー" };

// チャートの canvas を 6 つ作る
$("charts").innerHTML = MOTORS.map((m) => `<figure><canvas id="ch_${m}"></canvas><figcaption><span>${NAMES[m]}</span><span class="mono">${m}</span></figcaption></figure>`).join("");

function params() {
  return { cubeXY: [Number($("cx").value), Number($("cy").value)], cubeYaw: (Number($("cyaw").value) * Math.PI) / 180, trayXY: [0.18, -0.18] };
}

function showParams() {
  $("cxv").textContent = `x = ${Number($("cx").value).toFixed(2)} m`;
  $("cyv").textContent = `y = ${Number($("cy").value).toFixed(2)} m`;
  $("cyawv").textContent = `${$("cyaw").value}°`;
  $("capv").textContent = `${$("cap").value}° / 回`;
}

function drawCharts() {
  const xs = log.map((r) => r.t);
  for (const m of MOTORS) {
    lineChart($(`ch_${m}`), [
      { name: "指令", xs, ys: log.map((r) => r[`cmd_${m}`]) },
      { name: "読み値", xs, ys: log.map((r) => r[`obs_${m}`]), dashed: true },
    ], { xLabel: "時間 [秒]", yLabel: m === "gripper" ? "0=閉 100=開" : "度", size: { w: 320, h: 190 } });
  }
}

function showErrors() {
  if (!log.length) { $("errs").innerHTML = ""; return; }
  $("errs").innerHTML = Object.entries(checkLog(log)).map(([m, [mx, mean]]) =>
    `<tr><td>${NAMES[m]}</td><td class="mono">${mx.toFixed(2)}</td><td class="mono">${mean.toFixed(2)}</td></tr>`).join("");
}

function setStatus() {
  $("time").textContent = `${sim.time.toFixed(1)} 秒`;
  $("phase").textContent = phase;
  const ok = sim.cubeInTray();
  $("cube").textContent = ok ? "トレイの中" : "机の上・運搬中";
  $("cube").className = ok ? "ok" : "";
}

function* task() {
  for (const label of planAndRun(sim, params(), log)) { phase = label; yield; }
}

const runner = new Runner({
  stepsPerSec: 40,
  getSpeed: () => Number($("speed").value),
  onFrame: () => {
    viewer.sync(sim.data); setStatus();
    if (++frames % 6 === 0) { drawCharts(); showErrors(); }
  },
  onState: (r) => { $("play").textContent = r.finished ? "もう一度" : r.running ? "一時停止" : r.task ? "再開" : "スタート"; },
  onDone: () => {
    phase = "完了";
    drawCharts(); showErrors(); setStatus();
    $("csv").disabled = false;
    $("done").textContent = sim.cubeInTray() ? `完了: キューブがトレイに入りました(${log.length} ステップ、${(log.length / sim.config.fps).toFixed(1)} 秒)` : "完了: キューブはトレイに入りませんでした";
  },
});

function resetSim() {
  runner.stop();
  sim.config = { ...sim.config, ...params(), maxRelativeTarget: Number($("cap").value) };
  sim.build();
  sim.connect();
  viewer.build(sim.model);
  log = []; phase = "待機"; frames = 0;
  $("done").textContent = ""; $("csv").disabled = true;
  viewer.sync(sim.data); setStatus(); drawCharts(); showErrors();
  for (const m of MOTORS) { const c = $(`ch_${m}`); c.getContext("2d").clearRect(0, 0, c.width, c.height); }
}

async function main() {
  $("status").textContent = "物理エンジン(MuJoCo)を読み込み中…";
  const mujoco = await initMujoco();
  sim = await SimSO101.create(mujoco, browserLoader(new URL("../_web/", import.meta.url)), { maxRelativeTarget: 10, ...params() });
  sim.connect();
  showParams();
  viewer.build(sim.model); viewer.sync(sim.data); setStatus();
  $("status").hidden = true;
  $("app").hidden = false;
  resetSim();
  $("play").onclick = () => {
    if (runner.finished) resetSim();
    if (!runner.task) runner.start(task()); else runner.toggle();
  };
  $("reset").onclick = resetSim;
  for (const id of ["cx", "cy", "cyaw", "cap"]) $(id).oninput = () => { showParams(); };
  for (const id of ["cx", "cy", "cyaw", "cap"]) $(id).onchange = resetSim;
  $("speed").oninput = () => { $("speedv").textContent = `${$("speed").value}x`; };
  $("csv").onclick = () => {
    const url = URL.createObjectURL(new Blob([logToCsv(log)], { type: "text/csv" }));
    const a = Object.assign(document.createElement("a"), { href: url, download: "sim_log.csv" });
    a.click();
    URL.revokeObjectURL(url);
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
