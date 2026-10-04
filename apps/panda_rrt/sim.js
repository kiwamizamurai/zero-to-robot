// 壁をよけて腕を動かすシミュレーション(rrt_plan.py の移植)。DOM には触らない。
// 場面と IK は ../_web/panda.js(PandaCell)、当たり判定は MuJoCo の mj_collision に聞く。
//
//   const sim = await RrtSim.create(mujoco, load, { seed: 0 });
//   for (const _ of sim.run());   // 1 回の yield = 探索 1 回 / 実行の制御 1 ステップ(0.02 秒)
import { PandaCell, pandaSceneXml, N_PHYSICS } from "../_web/panda.js";
import { writeText } from "../_web/mujoco.js";
import { mulberry32, smooth } from "../_web/math.js";

export const WALL = { pos: [0.48, 0.0, 0.30], half: [0.12, 0.015, 0.30] }; // 高さ 60 cm の壁
export const START_TCP = [0.50, -0.28, 0.18], GOAL_TCP = [0.50, 0.28, 0.18];
export const ROBOT_BODIES = ["link1", "link2", "link3", "link4", "link5", "link6", "link7", "hand", "left_finger", "right_finger"];
export const MARGIN = 0.03; // 計画用には壁と床をこの分だけ大きく見る [m]
export const STEP = 0.15, CHECK_STEP = 0.01, MAX_ITER = 20000, SHORTCUT_TRIES = 200;

const scene = () => ({
  blocks: [], box: false,
  extraWorldXml: `
    <body name="wall" pos="${WALL.pos.join(" ")}"><geom type="box" size="${WALL.half.join(" ")}" rgba="0.75 0.45 0.3 1"/></body>
    <geom name="mark_start" type="sphere" size="0.025" pos="${START_TCP.join(" ")}" rgba="0.2 0.6 1 0.9" contype="0" conaffinity="0"/>
    <geom name="mark_goal" type="sphere" size="0.025" pos="${GOAL_TCP.join(" ")}" rgba="0.2 0.8 0.3 0.9" contype="0" conaffinity="0"/>`,
});

const norm = (v) => Math.hypot(...v);
const sub = (a, b) => a.map((x, i) => x - b[i]);
const lerp = (a, b, t) => a.map((x, i) => x + (b[i] - x) * t);
export const pathLength = (path) => path.slice(1).reduce((s, q, i) => s + norm(sub(q, path[i])), 0);

// 関節角度 q のとき、腕が壁や床にぶつかっていないかを調べる(計画用。壁と床は MARGIN だけ大きく見る)
export class Checker {
  constructor(mj, model, margin = MARGIN) {
    this.mj = mj;
    this.m = model;
    this.d = new mj.MjData(model);
    this.robot = new Set(ROBOT_BODIES.map((b) => mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY.value, b)));
    for (let g = 0; g < model.ngeom; g++) if (!this.robot.has(model.geom_bodyid[g])) model.geom_margin[g] = margin;
    this.lo = Array.from(model.jnt_range.subarray(0, 14)).filter((_, i) => i % 2 === 0);
    this.hi = Array.from(model.jnt_range.subarray(0, 14)).filter((_, i) => i % 2 === 1);
    this.calls = 0;
    this.tcp = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE.value, "tcp");
  }
  free(q) {
    this.calls++;
    for (let i = 0; i < 7; i++) if (q[i] < this.lo[i] || q[i] > this.hi[i]) return false;
    const { mj, m, d } = this;
    for (let i = 0; i < 7; i++) d.qpos[i] = q[i];
    d.qpos[7] = d.qpos[8] = 0.04;
    mj.mj_kinematics(m, d);
    mj.mj_collision(m, d);
    for (let i = 0; i < d.ncon; i++) {
      const c = d.contact.get(i);
      if (this.robot.has(m.geom_bodyid[c.geom1]) !== this.robot.has(m.geom_bodyid[c.geom2])) return false; // 腕と、腕でないものの接触
    }
    return true;
  }
  edgeFree(a, b, step = CHECK_STEP) {
    const n = Math.max(2, Math.ceil(Math.max(...sub(b, a).map(Math.abs)) / step));
    for (let k = 1; k <= n; k++) if (!this.free(lerp(a, b, k / n))) return false;
    return true;
  }
  // q のときの手先位置(木の枝を 3D に描くのに使う。直前の free(q) の姿勢をそのまま読む)
  tcpPos() {
    const p = this.d.site_xpos, i = 3 * this.tcp;
    return [p[i], p[i + 1], p[i + 2]];
  }
  dispose() { this.d.delete(); this.m.delete(); }
}

export class RrtSim extends PandaCell {
  reset(opts = {}) {
    super.reset({ ...scene(), ...opts });
    const { seed = 0 } = opts;
    this.seed = seed;
    this.rnd = mulberry32(seed);
    writeText(this.mj, "/panda/scene_plan.xml", pandaSceneXml(scene()));
    this.chk?.dispose();
    this.chk = new Checker(this.mj, this.mj.MjModel.from_xml_path("/panda/scene_plan.xml"));
    this.phase = "ready";
    this.log = [];
    this.trees = [{ q: [], p: [], parent: [] }, { q: [], p: [], parent: [] }];
    this.qs = this.qg = this.rawPath = this.path = null;
    this.stats = {};
    this.trace = { straight: [], rrt: [] };
  }

  dispose() { this.chk?.dispose(); this.chk = null; super.dispose(); }

  // 関節角度 q のときの手先位置(当たり判定の回数には数えない)
  tcpOf(q) {
    const { mj, chk } = this;
    for (let i = 0; i < 7; i++) chk.d.qpos[i] = q[i];
    mj.mj_kinematics(chk.m, chk.d);
    return chk.tcpPos();
  }

  // 道(関節角度の列)を、関節空間でまっすぐつないだときの手先の軌跡(sub 分割)
  pathTcp(path, sub = 12) {
    const out = [this.tcpOf(path[0])];
    for (let i = 1; i < path.length; i++) for (let k = 1; k <= sub; k++) out.push(this.tcpOf(lerp(path[i - 1], path[i], k / sub)));
    return out;
  }

  uniform() { return this.chk.lo.map((lo, i) => lo + (this.chk.hi[i] - lo) * this.rnd()); }

  // 手先を target へ向ける逆運動学。q0 の近くの解を返す(Python 版の ik)
  solveIK(target, q0) {
    const ik = this.ik;
    ik.qHome = [...q0]; ik.q = [...q0]; ik.yaw = 0;
    ik.forward();
    for (let k = 0; k < 300; k++) {
      const before = ik.q;
      ik.solve(target);
      if (Math.max(...ik.q.map((x, i) => Math.abs(x - before[i]))) < 1e-9) break;
    }
    return [...ik.q];
  }

  setPose(q) { // 表示用の腕の姿勢
    const { mj, model, data } = this;
    for (let i = 0; i < 7; i++) { data.qpos[i] = q[i]; data.qvel[i] = 0; data.ctrl[i] = q[i]; }
    mj.mj_forward(model, data);
  }

  // RRT-Connect。始まりと終わりの両方から木を伸ばし、つながったら道を返す(1 回の yield = 1 回の試行)
  *rrtConnect() {
    const { chk, trees } = this;
    const add = (tree, q, parent) => { tree.q.push(q); tree.parent.push(parent); chk.free(q); tree.p.push(chk.tcpPos()); return tree.q.length - 1; };
    add(trees[0], this.qs, -1); add(trees[1], this.qg, -1);
    const nearest = (tree, q) => {
      let best = 0, bd = Infinity;
      for (let i = 0; i < tree.q.length; i++) {
        let s = 0;
        for (let k = 0; k < 7; k++) s += (tree.q[i][k] - q[k]) ** 2;
        if (s < bd) { bd = s; best = i; }
      }
      return best;
    };
    const extend = (tree, q) => {
      const i = nearest(tree, q), qn = tree.q[i];
      const d = sub(q, qn), dist = norm(d);
      const reached = dist <= STEP;
      const nw = reached ? q : qn.map((x, k) => x + (d[k] / dist) * STEP);
      if (!chk.edgeFree(qn, nw)) return [null, false];
      return [add(tree, nw, i), reached];
    };
    const toRoot = (tree, i) => { const out = []; for (; i !== -1; i = tree.parent[i]) out.push(tree.q[i]); return out; };
    let a = 0, b = 1, tPlan = 0;
    for (let it = 0; it < MAX_ITER; it++) {
      const t0 = performance.now();
      const [ia] = extend(trees[a], this.uniform());
      let found = null;
      if (ia !== null) {
        const target = trees[a].q[ia];
        for (;;) { // もう一方の木を、いま伸ばした点に向かって伸ばせるだけ伸ばす(connect)
          const [ib, reached] = extend(trees[b], target);
          if (ib === null) break;
          if (reached) {
            const pa = toRoot(trees[a], ia), pb = toRoot(trees[b], ib);
            found = a === 0 ? [...pa.reverse(), ...pb] : [...pb.reverse(), ...pa];
            break;
          }
        }
      }
      tPlan += performance.now() - t0;
      this.stats.iters = it + 1;
      this.stats.nodes = trees[0].q.length + trees[1].q.length;
      this.stats.tPlan = tPlan / 1000;
      if (found) return found;
      [a, b] = [b, a];
      yield;
    }
    return null;
  }

  // 道の途中の 2 点をまっすぐつなげられるなら、その間を飛ばす
  *shortcut(path) {
    path = [...path];
    for (let t = 0; t < SHORTCUT_TRIES; t++) {
      if (path.length < 3) break;
      let i = Math.floor(this.rnd() * path.length), j = Math.floor(this.rnd() * (path.length - 1));
      if (j >= i) j++;
      if (i > j) [i, j] = [j, i];
      if (j - i >= 2 && this.chk.edgeFree(path[i], path[j])) path = [...path.slice(0, i + 1), ...path.slice(j)];
      this.shortTries = t + 1;
      if (t % 4 === 3) yield;
    }
    return path;
  }

  // 道を弧長でたどり、5 次多項式でなめらかに動かす。壁・床に触れた物理ステップの数と、手先の軌跡を返す
  *execute(path, duration, trace) {
    const { mj, model, data } = this;
    this.setPose(path[0]);
    data.qpos[7] = data.qpos[8] = 0.04;
    data.ctrl[7] = 255;
    mj.mj_forward(model, data);
    const seg = [0];
    for (let i = 1; i < path.length; i++) seg.push(seg[i - 1] + norm(sub(path[i], path[i - 1])));
    const total = seg[seg.length - 1];
    const robot = this.chk.robot;
    const dt = model.opt.timestep;
    const n = Math.floor(duration / dt), all = n + Math.floor(0.8 / dt);
    let hits = 0;
    for (let k = 0; k < all; k++) {
      const s = smooth(k / n) * total;
      let i = 0;
      while (i < path.length - 2 && seg[i + 1] <= s) i++;
      const u = (s - seg[i]) / Math.max(seg[i + 1] - seg[i], 1e-9);
      for (let j = 0; j < 7; j++) data.ctrl[j] = path[i][j] + (path[i + 1][j] - path[i][j]) * u;
      mj.mj_step(model, data);
      for (let c = 0; c < data.ncon; c++) {
        const g = data.contact.get(c);
        if (robot.has(model.geom_bodyid[g.geom1]) !== robot.has(model.geom_bodyid[g.geom2])) { hits++; break; }
      }
      if ((k + 1) % N_PHYSICS === 0) { trace.push(this.tcp()); yield; }
    }
    return hits;
  }

  // rrt_plan.py の main と同じ手順。phase が「いま何をしているか」を表す
  *run() {
    const { chk, stats, log } = this;
    this.phase = "ik";
    this.setPose(this.model.key("home").qpos);
    this.qs = this.solveIK(START_TCP, Array.from(this.model.key("home").qpos.slice(0, 7)));
    this.qg = this.solveIK(GOAL_TCP, this.qs);
    stats.startFree = chk.free(this.qs); stats.goalFree = chk.free(this.qg);
    this.setPose(this.qs);
    log.push(`1. 始まり・終わりの姿勢: ぶつかっていないか ${stats.startFree} / ${stats.goalFree}`);
    yield;

    this.phase = "straight";
    stats.straightFree = chk.edgeFree(this.qs, this.qg);
    log.push(`2. 関節角度をまっすぐ補間すると: ${stats.straightFree ? "ぶつからない" : "壁にぶつかる"}`);
    yield;

    this.phase = "rrt";
    chk.calls = 0;
    const raw = yield* this.rrtConnect();
    stats.calls = chk.calls;
    if (!raw) { this.phase = "failed"; log.push("道が見つからなかった"); return; }
    this.rawPath = raw;
    log.push(`3. RRT-Connect: ${stats.iters} 回で道が見つかった(${stats.tPlan.toFixed(2)} 秒、木の点 ${stats.nodes} 個、当たり判定 ${stats.calls} 回)`);

    this.phase = "shortcut";
    this.path = yield* this.shortcut(raw);
    stats.rawLen = pathLength(raw); stats.shortLen = pathLength(this.path);
    stats.rawPoints = raw.length; stats.shortPoints = this.path.length;
    log.push(`4. ショートカット: 経由点 ${raw.length} → ${this.path.length} 個、関節空間での道のり ${stats.rawLen.toFixed(1)} → ${stats.shortLen.toFixed(1)} rad`);

    this.phase = "exec-straight";
    stats.hitsStraight = yield* this.execute([this.qs, this.qg], 3.0, this.trace.straight);
    this.phase = "exec-rrt";
    stats.hitsRrt = yield* this.execute(this.path, 4.0, this.trace.rrt);
    const end = this.tcp();
    stats.err = norm(sub(end, GOAL_TCP)) * 100;
    log.push(`5. 実行: まっすぐ = 壁や床との接触 ${stats.hitsStraight} ステップ / RRT の道 = 接触 ${stats.hitsRrt} ステップ、着いた位置のずれ ${stats.err.toFixed(1)} cm`);
    this.phase = "done";
  }
}
export { RrtSim as default };
