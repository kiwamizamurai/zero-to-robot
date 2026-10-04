// rl_balance の画面。物理と方策の計算は sim.js、描画と再生制御は ../_web の共通部品。
// 学習はここではしない。tools/train.mjs で済ませた結果(data/policy.json)を再生する。
import { initMujoco } from "../_web/mujoco.js";
import { createViewer } from "../_web/viewer.js";
import { Runner, $ } from "../_web/runner.js";
import { lineChart } from "../_web/chart.js";
import { BalanceSim, mlpPolicy, randomPolicy, MAX_STEPS, DT } from "./sim.js";

const viewer = createViewer($("view"), { camera: [0.5, -2.6, 0.9], target: [0, 0, 0.25] });
let sim, data, seed = 1;
let arrow = null;                       // 押している間だけ出す矢印
const tally = { tries: 0, falls: 0, wins: 0 };

// 方策の一覧: ランダム + 保存済みの方策(学習の途中・学習後)
let policies = [];
const current = () => policies[$("policy").selectedIndex];

function makePolicy() {
  const p = current();
  return p.w ? mlpPolicy(p.w) : randomPolicy(seed);
}

function showArrow(dir) {
  clearArrow();
  const y = -0.12, z = 0.3, x0 = dir > 0 ? -0.55 : 0.55, x1 = dir > 0 ? -0.22 : 0.22;
  const { THREE } = viewer;
  viewer.line([[x0, y, z], [x1, y, z]], 0xd9534f, 3);
  const cone = viewer.addExtra(new THREE.Mesh(new THREE.ConeGeometry(0.05, 0.12, 12), new THREE.MeshBasicMaterial({ color: 0xd9534f })));
  cone.position.set(x1, y, z);
  cone.rotation.set(0, 0, dir > 0 ? -Math.PI / 2 : Math.PI / 2);
  arrow = cone;
}
function clearArrow() { if (arrow) { viewer.clearExtras(); arrow = null; } }

function setStatus() {
  $("time").textContent = `${sim.time.toFixed(1)} 秒`;
  $("tilt").textContent = `${sim.tiltDeg.toFixed(1)}°`;
  $("cart").textContent = `${sim.data.qpos[0].toFixed(2)} m`;
  if (arrow && !sim.pushing) clearArrow();
}

function showTally() {
  $("tries").textContent = `${tally.tries} 回`;
  $("falls").textContent = `${tally.falls} 回`;
  $("wins").textContent = `${tally.wins} 回`;
}

const runner = new Runner({
  stepsPerSec: 1 / DT,
  getSpeed: () => Number($("speed").value),
  onFrame: () => { viewer.sync(sim.data); setStatus(); },
  onState: (r) => { $("play").textContent = r.finished ? "もう一度" : r.running ? "一時停止" : r.task ? "再開" : "スタート"; },
  onDone: () => {
    const { length, fell } = sim.result;
    tally.tries++; fell ? tally.falls++ : tally.wins++;
    showTally();
    $("done").textContent = fell ? `${length} ステップ(${(length * DT).toFixed(2)} 秒)で倒れました` : `${MAX_STEPS} ステップ(${MAX_STEPS * DT} 秒)、最後まで立ち続けました`;
    $("done").style.color = fell ? "var(--bad)" : "";
  },
});

function resetSim() {
  sim.reset(seed);
  viewer.build(sim.model);
  clearArrow();
  runner.stop();
  $("done").textContent = "";
  viewer.sync(sim.data); setStatus();
}

function startNew() {
  seed++;
  resetSim();
  runner.start(sim.run(makePolicy()));
}

function drawCharts() {
  const { at, length, fell } = data.curve;
  const total = data.meta.totalSteps;
  const K = 10, avg = [], ax = [];
  let s = 0;
  for (let i = 0; i < length.length; i++) {
    s += length[i];
    if (i >= K) s -= length[i - K];
    if (i >= K - 1) { avg.push(s / K); ax.push(at[i]); }
  }
  const p = current();
  const mark = p.w ? p.steps : 0;
  lineChart($("chartLen"), [
    { name: "直近 10 回の平均", xs: ax, ys: avg },
    { name: "でたらめ", xs: [0, total], ys: [data.meta.randomMean, data.meta.randomMean], color: "#888", dashed: true },
    { name: "上限(最後まで立てた)", xs: [0, total], ys: [MAX_STEPS, MAX_STEPS], color: "#2f8a4b", dashed: true },
    { name: "いま再生中の方策", xs: [mark, mark], ys: [0, MAX_STEPS], color: "#d9534f", dashed: true },
  ], { xLabel: "それまでに積んだ経験(ステップ数。1 ステップ = 0.04 秒)", yLabel: "倒れるまでのステップ数", yMin: 0, yMax: 1100, xMin: 0, xMax: total, size: { w: 640, h: 300 } });
  const cum = []; let c = 0;
  for (const f of fell) cum.push((c += f));
  lineChart($("chartFall"), [
    { name: "倒れた回数(合計)", xs: at, ys: cum, color: "#d9534f" },
    { name: "いま再生中の方策", xs: [mark, mark], ys: [0, Math.max(...cum)], color: "#888", dashed: true },
  ], { xLabel: "それまでに積んだ経験(ステップ数)", yLabel: "倒れた回数", yMin: 0, xMin: 0, xMax: total, size: { w: 640, h: 260 } });
}

function explain() {
  const p = current();
  $("policyNote").textContent = p.note;
}

async function main() {
  $("status").textContent = "物理エンジン(MuJoCo)と学習済みの方策を読み込み中…";
  const [mujoco, res] = await Promise.all([initMujoco(), fetch(new URL("./data/policy.json", import.meta.url))]);
  if (!res.ok) throw new Error(`data/policy.json を読み込めません (${res.status})`);
  data = await res.json();
  const m = data.meta;
  policies = [
    { id: "random", label: "学習前(でたらめに動かす)", note: "どう動けば立つかをまだ何も知らない状態。台車を -3〜3 の力でランダムに押す。" },
    ...data.policies.map((p) => ({ ...p, note: p.id === "final" ? "学習のおわりの方策。ほとんどの回で最後まで立ち続ける。" : `学習の途中。評価では平均 ${Math.round(p.score)} ステップ立っていられる。` })),
  ];
  $("policy").innerHTML = policies.map((p, i) => `<option value="${i}">${p.label}</option>`).join("");
  $("policy").selectedIndex = policies.length - 1;
  sim = await BalanceSim.create(mujoco, { seed });
  viewer.build(sim.model); viewer.sync(sim.data); setStatus(); showTally(); explain();

  $("capLen").textContent = `学習中の ${m.episodes} 回のエピソード(倒れるか時間切れまでを 1 回)のうち、最初のうちはでたらめと変わらず数ステップで倒れていた。初めて最後まで立てたのは ${m.firstFullEpisode} 回目。`;
  $("capFall").textContent = `学習中に倒れた回数の合計。強化学習は、最初は何もできない時間が長く、失敗をたくさん重ねてから急に覚える。`;
  $("method").textContent = `学習手法: ${m.method}。方策は 4 → ${m.hidden} → ${m.hidden} → 1 のニューラルネットワーク、経験 ${m.totalSteps.toLocaleString("ja-JP")} ステップ、乱数の種 ${m.seed}。元の Python 版が使う Stable-Baselines3 の PPO の既定値に合わせて、この画面用に自分で書き直したものです。`;

  $("status").hidden = true;
  $("app").hidden = false; $("banner").hidden = false; $("charts").hidden = false;
  drawCharts();

  $("play").onclick = () => {
    if (runner.finished) startNew();
    else if (!runner.task) runner.start(sim.run(makePolicy()));
    else runner.toggle();
  };
  $("reset").onclick = resetSim;
  $("policy").onchange = () => { explain(); drawCharts(); startNew(); };
  $("speed").oninput = () => { $("speedv").textContent = `${$("speed").value}x`; };
  $("force").oninput = () => { $("forcev").textContent = `${$("force").value} N`; };
  const push = (dir) => {
    if (!runner.running) return;
    sim.push(dir * Number($("force").value));
    showArrow(dir);
  };
  $("pushL").onclick = () => push(-1);
  $("pushR").onclick = () => push(1);
  runner.onState(runner);
  runner.loop(() => viewer.render());
  window.__app = { get sim() { return sim; }, get runner() { return runner; } }; // 動作確認用のフック
}

main().catch((e) => {
  console.error(e);
  $("status").textContent = `読み込みに失敗しました: ${e.message}`;
  $("status").className = "status error";
});
