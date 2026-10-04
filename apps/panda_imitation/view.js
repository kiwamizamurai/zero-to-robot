// panda_imitation の画面。物理は sim.js、方策の推論は policy.js、描画と再生制御は ../_web の共通部品。
// 学習はここではしない(tools/train.mjs で事前に済ませた data/*.json を読んで再生するだけ)。
import { initMujoco, browserLoader } from "../_web/mujoco.js";
import { createViewer } from "../_web/viewer.js";
import { Runner, $ } from "../_web/runner.js";
import { barChart } from "../_web/chart.js";
import { ImitationSim, placementFromSeed } from "./sim.js";
import { Policy } from "./policy.js";

// 動かせるもの。file が null なら先生(手順書)
const CHOICES = [
  { id: "teacher", name: "先生(手順書)", file: null, note: "手順書どおりに動く、お手本を作った側。学習したネットではありません。" },
  { id: "n200_noise", name: "学習済み: お手本 200 回(揺れあり)", file: "policy_n200_noise.json", note: "いちばんよく学べたもの。" },
  { id: "n200_clean", name: "学習済み: お手本 200 回(揺れなし)", file: "policy_n200_clean.json", note: "先生がいつも完璧な道を通ったお手本だけで学んだもの。" },
  { id: "n50_noise", name: "学習済み: お手本 50 回(揺れあり)", file: "policy_n50_noise.json", note: "お手本が足りず、ほとんど成功しません。" },
  { id: "n10_noise", name: "学習済み: お手本 10 回(揺れあり)", file: "policy_n10_noise.json", note: "お手本が少なすぎて、ブロックまでたどり着けません。" },
];
const dataUrl = (f) => new URL(`data/${f}`, import.meta.url);

const viewer = createViewer($("view"));
const cache = new Map();
let sim, evalSim, mujoco, choice = CHOICES[1], policy = null;
const pct = (v) => `${(v * 100).toFixed(0)}%`;

async function loadPolicy(c) {
  if (!c.file) return null;
  if (!cache.has(c.file)) {
    const res = await fetch(dataUrl(c.file));
    if (!res.ok) throw new Error(`${c.file} を読み込めません (${res.status})`);
    cache.set(c.file, new Policy(await res.json()));
  }
  return cache.get(c.file);
}

function seedValue() { return Math.max(0, Math.floor(Number($("seed").value) || 0)); }

function setStatus() {
  const c = sim.lastCmd, started = sim.steps > 0;
  $("time").textContent = `${sim.time.toFixed(1)} 秒`;
  $("cmd").textContent = started ? `(${(c[0] * 1000).toFixed(0)}, ${(c[1] * 1000).toFixed(0)}, ${(c[2] * 1000).toFixed(0)}) mm` : "-";
  $("grip").textContent = started ? (c[4] > 0.5 ? "閉じる" : "開く") : "-";
  const ok = sim.success();
  $("blockstate").textContent = ok ? "箱の中" : "机の上・運搬中";
  $("blockstate").className = ok ? "ok" : "";
}

const runner = new Runner({
  getSpeed: () => Number($("speed").value),
  onFrame: () => { viewer.sync(sim.data); setStatus(); },
  onState: (r) => { $("play").textContent = r.finished ? "もう一度" : r.running ? "一時停止" : r.task ? "再開" : "スタート"; },
  onDone: () => {
    const ok = sim.success();
    $("done").textContent = ok ? "ブロックが箱に入りました" : "ブロックは箱に入りませんでした";
    $("done").style.color = ok ? "" : "var(--bad)";
  },
});

function resetSim() {
  sim.place(...placementFromSeed(seedValue()));
  runner.stop();
  $("done").textContent = "";
  viewer.sync(sim.data); setStatus();
}

function startRun() {
  runner.start(choice.file ? sim.runPolicy(policy) : sim.runTeacher());
}

async function choose(id) {
  choice = CHOICES.find((c) => c.id === id);
  $("policynote").textContent = "読み込み中…";
  policy = await loadPolicy(choice);
  const m = policy?.meta;
  $("policynote").textContent = choice.note + (m?.success_rate != null ? ` 新しい置き方での成功率は、学習時の評価で ${pct(m.success_rate)}。` : "");
  resetSim();
}

// 事前計算した成功率の表とグラフ
async function showTable() {
  const res = await fetch(dataUrl("success_rate.json"));
  if (!res.ok) return;
  const t = await res.json();
  const bars = [{ label: "先生", value: t.teacher * 100, color: "#888" },
    ...t.noise.map((r) => ({ label: `${r.demos} 回`, value: r.rate * 100, color: "#1f6fb2" })),
    ...t.clean.map((r) => ({ label: `${r.demos} 回 揺れなし`, value: r.rate * 100, color: "#d9542f" }))];
  barChart($("chart"), bars, { yLabel: "成功率 [%]", yMax: 100, size: { w: 760, h: 300 } });
  $("chartnote").textContent = `横軸はお手本の数(青: 揺れあり、赤: 揺れなし、灰色: 先生の手順書そのもの)、縦軸は学習に使わない ${t.eval_count} 通りの置き方で試した成功率。学習の乱数を変えた ${t.train_seeds.length} 回の平均です。事前に Node で計算した値で、このブラウザで学習したものではありません。`;
  const row = (kind, r) => `<tr><td>${r.demos} 回</td><td>${kind}</td><td>${pct(r.rate)}</td><td>${r.rates.map(pct).join(" / ")}</td><td>${r.loss.toFixed(3)}</td></tr>`;
  $("table").innerHTML = `<tr><th>お手本</th><th>揺れ</th><th>成功率(平均)</th><th>学習の乱数ごと</th><th>学習の誤差</th></tr>`
    + t.noise.map((r) => row("あり", r)).join("") + t.clean.map((r) => row("なし", r)).join("");
  $("chartsec").hidden = false;
}

// 別のインスタンスで 20 通りを(画面に出さず)回す。1 回ごとに画面へ制御を返す
async function evaluate() {
  const btn = $("evalbtn");
  btn.disabled = true;
  const c = choice, p = policy, N = 20;
  let ok = 0;
  for (let i = 0; i < N; i++) {
    evalSim.place(...placementFromSeed(5000 + i));
    for (const _ of c.file ? evalSim.runPolicy(p) : evalSim.runTeacher());
    ok += evalSim.success();
    $("evalout").textContent = `${i + 1} / ${N} 回目まで: ${ok} 回成功`;
    await new Promise((r) => setTimeout(r, 0));
  }
  $("evalout").textContent = `${c.name}: ${N} 回中 ${ok} 回成功(${pct(ok / N)})。20 回なので ±10% 程度のぶれがあります`;
  btn.disabled = false;
}

async function main() {
  $("status").textContent = "物理エンジン(MuJoCo)を読み込み中…";
  mujoco = await initMujoco();
  const load = browserLoader(new URL("../_web/", import.meta.url));
  sim = await ImitationSim.create(mujoco, load, { seed: seedValue() });
  evalSim = new ImitationSim(mujoco, { seed: 1 });
  viewer.build(sim.model);
  $("policy").innerHTML = CHOICES.map((c) => `<option value="${c.id}">${c.name}</option>`).join("");
  $("policy").value = choice.id;
  await choose(choice.id);
  $("status").hidden = true;
  $("app").hidden = false;
  $("evalsec").hidden = false;
  showTable().catch(console.error);
  $("play").onclick = () => {
    if (runner.finished) resetSim();
    if (!runner.task) startRun(); else runner.toggle();
  };
  $("reset").onclick = resetSim;
  $("policy").onchange = () => choose($("policy").value).catch(fail);
  $("seed").onchange = resetSim;
  $("shuffle").onclick = () => { $("seed").value = Math.floor(Math.random() * 100000); resetSim(); };
  $("speed").oninput = () => { $("speedv").textContent = `${$("speed").value}x`; };
  $("evalbtn").onclick = () => evaluate().catch(fail);
  runner.onState(runner);
  runner.loop(() => viewer.render());
  window.__app = { get sim() { return sim; }, get runner() { return runner; } }; // 動作確認用のフック
}

function fail(e) {
  console.error(e);
  $("status").hidden = false;
  $("status").textContent = `読み込みに失敗しました: ${e.message}`;
  $("status").className = "status error";
}
main().catch(fail);
