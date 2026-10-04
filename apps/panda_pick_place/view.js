// panda_pick_place の画面。物理は sim.js(../_web/panda.js)、描画と再生制御は ../_web の共通部品。
import { initMujoco, browserLoader } from "../_web/mujoco.js";
import { createViewer } from "../_web/viewer.js";
import { Runner, $ } from "../_web/runner.js";
import { PickPlaceSim, BLOCKS } from "./sim.js";

const viewer = createViewer($("view"));
let sim, seed = 1, shownLogs = 0;

function setStatus() {
  $("time").textContent = `${sim.time.toFixed(1)} 秒`;
  const res = sim.results();
  $("blocks").innerHTML = BLOCKS.map(({ name, rgba }) => {
    const [r, g, b] = rgba.map((x) => Math.round(x * 255));
    return `<li><span class="dot" style="background:rgb(${r},${g},${b})"></span>${name}<b class="${res[name] ? "ok" : ""}">${res[name] ? "箱の中" : "机の上・運搬中"}</b></li>`;
  }).join("");
  if (sim.log.length !== shownLogs) {
    $("log").textContent = sim.log.join("\n");
    $("log").scrollTop = $("log").scrollHeight;
    shownLogs = sim.log.length;
  }
}

const runner = new Runner({
  getSpeed: () => Number($("speed").value),
  onFrame: () => { viewer.sync(sim.data); setStatus(); },
  onState: (r) => { $("play").textContent = r.finished ? "もう一度" : r.running ? "一時停止" : r.task ? "再開" : "スタート"; },
  onDone: () => {
    const n = Object.values(sim.results()).filter(Boolean).length;
    $("done").textContent = `完了: ${BLOCKS.length} 個中 ${n} 個が箱に入りました`;
  },
});

function resetSim() {
  const jitter = $("random").checked ? 0.025 : 0;
  if ($("random").checked) seed = Math.floor(Math.random() * 1e6);
  sim.reset({ seed, jitter });
  viewer.build(sim.model);
  runner.stop();
  shownLogs = 0;
  $("log").textContent = "";
  $("done").textContent = "";
  viewer.sync(sim.data); setStatus();
}

async function main() {
  $("status").textContent = "物理エンジン(MuJoCo)を読み込み中…";
  const mujoco = await initMujoco();
  sim = await PickPlaceSim.create(mujoco, browserLoader(new URL("../_web/", import.meta.url)), { seed, jitter: 0 });
  viewer.build(sim.model); viewer.sync(sim.data); setStatus();
  $("status").hidden = true;
  $("app").hidden = false;
  $("play").onclick = () => {
    if (runner.finished) resetSim();
    if (!runner.task) runner.start(sim.run()); else runner.toggle();
  };
  $("reset").onclick = resetSim;
  $("random").onchange = resetSim;
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
