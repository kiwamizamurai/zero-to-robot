// panda_vision_pick の画面。物理・撮影・検出は sim.js / vision.js、描画と再生制御は ../_web の共通部品。
import { initMujoco, browserLoader } from "../_web/mujoco.js";
import { createViewer } from "../_web/viewer.js";
import { Runner, $ } from "../_web/runner.js";
import { VisionPickSim, COLORS } from "./sim.js";
import { addFrustum, captureCamera, drawEmpty, drawMask, drawShot, rgb255 } from "./camview.js";

const viewer = createViewer($("view"));
let sim, seed = 0, shownLogs = 0, frustum = [];

const dot = (rgba) => `<span class="dot" style="background:${rgb255(rgba)}"></span>`;

function setStatus() {
  $("time").textContent = `${sim.time.toFixed(1)} 秒`;
  $("shots").textContent = String(sim.shots.length);
  const res = sim.results();
  $("blocks").innerHTML = COLORS.map(({ name, rgba }) => `<li>${dot(rgba)}${name}<b class="${res[name] ? "ok" : ""}">${res[name] ? "箱の中" : "机の上・運搬中"}</b></li>`).join("");
  if (sim.log.length !== shownLogs) {
    $("log").textContent = sim.log.join("\n");
    $("log").scrollTop = $("log").scrollHeight;
    shownLogs = sim.log.length;
  }
}

// 撮影のたびに呼ばれる: 画像と検出結果を並べて見せる
function showShot(shot) {
  const n = shot.image.width;
  drawShot($("cam"), shot.image, { found: shot.found, title: `overhead camera (撮影 ${sim.shots.length} 回目)` });
  drawMask($("mask"), shot.labels, n, n);
  $("camcap").textContent = `${sim.shots.length} 回目の撮影。白い枠が見つけたブロックの輪郭、点が中心です(画像の右が机の +x、上が +y)。`;
  $("dets").innerHTML = shot.found.length
    ? shot.found.map((f) => {
      const c = COLORS.find((x) => x.name === f.name);
      return `<tr><td>${dot(c.rgba)}${f.name}</td><td>${(f.pos[0] * 100).toFixed(1)}, ${(f.pos[1] * 100).toFixed(1)}</td><td>${((f.yaw * 180) / Math.PI).toFixed(0)}°</td><td>${(f.posError * 1000).toFixed(1)} mm, ${((f.yawError * 180) / Math.PI).toFixed(1)}°</td></tr>`;
    }).join("")
    : `<tr><td colspan="4">箱の外には何も見つかりませんでした</td></tr>`;
}

// 3D 画面を撮った画像で見つける(試験的)。深度は 1 点だけ光線で測る
function captureShooter(s, cam) {
  viewer.sync(s.data);
  const image = captureCamera(viewer, cam, { size: s.cameraCfg.size, fovy: s.cameraCfg.fovy, hide: frustum });
  return { image, depthAt: (u, v) => s.rayDepth(cam, u, v) };
}

const runner = new Runner({
  getSpeed: () => Number($("speed").value),
  onFrame: () => { viewer.sync(sim.data); setStatus(); },
  onState: (r) => { $("play").textContent = r.finished ? "もう一度" : r.running ? "一時停止" : r.task ? "再開" : "スタート"; },
  onDone: () => {
    const n = Object.values(sim.results()).filter(Boolean).length;
    $("done").textContent = `完了: ${COLORS.length} 個中 ${n} 個が箱に入りました(つかみに行った回数 ${sim.placed})`;
  },
});

function resetSim() {
  seed = Math.max(0, Math.floor(Number($("seed").value) || 0));
  sim.reset({ seed });
  sim.onShot = showShot;
  sim.shooter = $("usecap").checked ? captureShooter : null;
  viewer.build(sim.model);
  viewer.clearExtras();
  frustum = addFrustum(viewer, sim.trueCamera(), { fovy: sim.cameraCfg.fovy });
  runner.stop();
  shownLogs = 0;
  $("log").textContent = "";
  $("done").textContent = "";
  $("dets").innerHTML = "";
  $("camcap").textContent = "天井カメラの画像。スタートすると、アームが視界からよけてから撮ります。";
  drawEmpty($("mask"), "まだ撮っていません", sim.cameraCfg.size);
  viewer.sync(sim.data);
  // 撮影前の景色(アームが写っている)
  const pre = sim.ray.shoot(sim.model, sim.data, sim.trueCamera());
  drawShot($("cam"), pre.image, { title: "overhead camera (撮影前)" });
  setStatus();
}

async function main() {
  $("status").textContent = "物理エンジン(MuJoCo)を読み込み中…";
  const mujoco = await initMujoco();
  sim = await VisionPickSim.create(mujoco, browserLoader(new URL("../_web/", import.meta.url)), { seed });
  $("status").hidden = true;
  $("app").hidden = false;
  $("cams").hidden = false;
  resetSim();
  $("play").onclick = () => {
    if (runner.finished) resetSim();
    if (!runner.task) runner.start(sim.run()); else runner.toggle();
  };
  $("reset").onclick = resetSim;
  $("seed").onchange = resetSim;
  $("reseed").onclick = () => { $("seed").value = Math.floor(Math.random() * 10000); resetSim(); };
  $("usecap").onchange = () => { sim.shooter = $("usecap").checked ? captureShooter : null; };
  $("speed").oninput = () => { $("speedv").textContent = `${$("speed").value}x`; };
  runner.onState(runner);
  runner.loop(() => viewer.render());
  window.__app = { get sim() { return sim; }, get runner() { return runner; } }; // 動作確認用のフック
}

main().catch((e) => {
  console.error(e);
  $("status").hidden = false;
  $("status").textContent = `読み込みに失敗しました: ${e.message}`;
  $("status").className = "status error";
});
