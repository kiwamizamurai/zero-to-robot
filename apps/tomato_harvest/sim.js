// ミニトマト収穫ロボット(harvest.py と scene.py の移植)。SO-101 の場面とモーター模擬は ../so101_twin/sim.js のものを使う。
//
//   見る → 見分ける(HSV)→ 位置に直す → 斜め上から近づく → つかんで切る → かごへ
//
// 元の Python との違い:
//  - 画像処理は OpenCV ではなく自前の小さな実装(HSV への変換、色の範囲での切り出し、収縮・膨張、ラベリング)
//  - 距離カメラ(RGB-D の D)はブラウザで出せないので、「実の大きさは決まっている(半径 13 mm)」ことから、
//    画面に映った実の大きさで距離を推定する
//  - カメラの画像は、ブラウザでは three.js で描いた画像(影なし・色そのままの描き方)を、Node のテストでは投影で作った画像を受け取る
import { SimSO101, MOTORS, Planner, Mover, trayXml } from "../so101_twin/sim.js";
import { mulberry32, mul3, rotZ } from "../_web/math.js";

export const FRUIT_R = 0.013; // ミニトマトの半径 [m](直径 2.6 cm)
export const POT = [0.345, 0.0];
// [名前, x, y, z, 熟し具合 0=緑 〜 1=真っ赤]
export const FRUITS = [
  ["t1", 0.280, 0.050, 0.110, 1.00],
  ["t2", 0.270, -0.030, 0.140, 0.95],
  ["t3", 0.285, 0.010, 0.085, 0.15],
  ["t4", 0.280, -0.065, 0.095, 0.90],
  ["t5", 0.275, 0.075, 0.150, 0.55],
  ["t6", 0.290, 0.035, 0.170, 0.10],
];
export const BASKET = [0.05, -0.22];
export const BASKET_HALF = 0.07, BASKET_H = 0.04;
export const REST = [-60, -100, 100, 0, 0, 30]; // 待機姿勢 [度]。カメラの視界から外れる
export const GRIP_OPEN = 55, GRIP_CLOSED = 2;
export const PITCH = (20 * Math.PI) / 180; // 手先を 20° 下に向けて近づく
export const APPROACH = 0.07; // 実の手前 7 cm で一度止まる
// 調整した値。実は握ると少しずつ手前(手のひら側)にずれるので、かごへ運ぶときの手先は、かごの中心から 0 だけ手前に止める
// (Python 版は 3 cm 手前)。落とすときの手首のひねりは、Python 版にはない(下の harvestOne を参照)
export const TUNING = { rotW: 0.1, graspDepth: 0.014, carryTime: 2.8, carryBack: 0.0, dropRoll: -Math.PI / 2 };

// 実を見るカメラ(ロボットの斜め後ろ上)。画像は 480 x 360、縦の視野角 55°
export const EYE = { pos: [0.06, -0.20, 0.26], lookAt: [0.285, 0.0, 0.13], fovy: 55, width: 480, height: 360 };

// 熟し具合を色にする: 緑 → オレンジ → 赤
export function ripenessRgba(r) {
  const g = [0.35, 0.65, 0.2], o = [0.95, 0.55, 0.12], rd = [0.85, 0.12, 0.08];
  const c = r < 0.5 ? g.map((v, i) => v + (o[i] - v) * (r / 0.5)) : o.map((v, i) => v + (rd[i] - v) * ((r - 0.5) / 0.5));
  return [...c, 1];
}

// 葉の位置(見た目だけ)。乱数の種を固定してあるので、毎回同じ
export const LEAVES = (() => {
  const rnd = mulberry32(3);
  return Array.from({ length: 14 }, () => {
    let a = rnd() * 2 * Math.PI;
    const z = 0.10 + rnd() * 0.17;
    if (Math.cos(a) < -0.2 && z < 0.2) a += Math.PI; // ロボット側の手前は葉を少なくして、実が見えるようにする
    return { a, z, r: 0.035 };
  });
})();

export class TomatoSO101 extends SimSO101 {
  sceneXml() {
    const [px, py] = POT;
    const leaves = LEAVES.map(({ a, z, r }) => `<geom type="ellipsoid" size="0.03 0.016 0.003" pos="${r * Math.cos(a)} ${r * Math.sin(a)} ${z}" euler="0.3 -0.4 ${a}" rgba="0.22 0.55 0.2 1" contype="0" conaffinity="0"/>`).join("\n      ");
    const fruits = FRUITS.map(([name, x, y, z, r]) => `
    <body name="${name}" pos="${x} ${y} ${z}"><freejoint/>
      <geom name="${name}_g" type="sphere" size="${FRUIT_R}" rgba="${ripenessRgba(r).join(" ")}" mass="0.008" friction="1.2 0.02 0.001" condim="4"/>
      <geom type="cylinder" size="0.004 0.002" pos="0 0 ${FRUIT_R}" rgba="0.2 0.45 0.15 1" contype="0" conaffinity="0"/>
      <geom type="capsule" size="0.0015" fromto="0 0 ${FRUIT_R} ${px - x} ${py - y} ${FRUIT_R + 0.02}" rgba="0.25 0.5 0.18 1" contype="0" conaffinity="0"/>
    </body>`).join("");
    // へた(果柄)で茎につながっている。つながりは「溶接」の拘束で表し、切ると外れる。
    // 基準点(アンカー)を実の位置に置く。鉢の根元のままだと、実が根元を中心に振り子のように回って落ちる
    const welds = FRUITS.map(([name, x, y, z]) => `<weld name="${name}_stem" body1="${name}" body2="plant" anchor="${x - px} ${y - py} ${z}"/>`).join("\n    ");
    return `<mujoco model="tomato scene">
  <include file="so101.xml"/>
  <worldbody>
    <geom name="floor" type="plane" size="0 0 0.05" rgba="0.80 0.78 0.72 1"/>
    <body name="plant" pos="${px} ${py} 0">
      <geom type="cylinder" size="0.05 0.03" pos="0 0 0.03" rgba="0.7 0.4 0.25 1"/>
      <geom type="cylinder" size="0.046 0.002" pos="0 0 0.061" rgba="0.3 0.22 0.15 1"/>
      <geom name="stem" type="capsule" size="0.006 0.11" pos="0 0 0.17" rgba="0.25 0.5 0.18 1"/>
      ${leaves}
    </body>${fruits}
    ${trayXml(BASKET, { half: BASKET_HALF, wall: 0.003, height: BASKET_H, rgba: "0.75 0.6 0.35 1", name: "basket" })}
  </worldbody>
  <equality>
    ${welds}
  </equality>
</mujoco>`;
  }

  build() {
    super.build();
    this.eqId = Object.fromEntries(FRUITS.map(([n]) => [n, this.mj.mj_name2id(this.model, this.mj.mjtObj.mjOBJ_EQUALITY.value, `${n}_stem`)]));
    this.obj2_0 = Array.from(this.model.eq_obj2id);
  }

  connect() {
    super.connect();
    this.model.eq_obj2id.set(this.obj2_0); // 切ったへたを元に戻す
    this.cutSet = new Set();
    const user = this.toRad(REST);
    this.qadr.forEach((a, i) => { this.data.qpos[a] = user[i]; });
    this.act.forEach((a, i) => { this.data.ctrl[a] = user[i]; });
    this.mj.mj_forward(this.model, this.data);
  }

  // ---- シミュレーターだけの機能(実機では、ハサミのサーボや人の目で置き換える) ----
  // へたを切る: 茎と実をつなぐ拘束を外す。ブラウザ版の MuJoCo は data.eq_active を読み書きできないので、
  // 拘束の相手(body2)を実自身に付け替える。実と実自身の溶接は何も拘束しないので、実は自由になる
  cut(name) {
    this.model.eq_obj2id[this.eqId[name]] = this.mj.mj_name2id(this.model, this.mj.mjtObj.mjOBJ_BODY.value, name);
    this.cutSet.add(name);
  }
  attached(name) { return !this.cutSet.has(name); }
  fruitPos(name) { const p = this.data.body(name).xpos; return [p[0], p[1], p[2]]; }
  inBasket(name) {
    const p = this.fruitPos(name);
    return Math.abs(p[0] - BASKET[0]) < BASKET_HALF && Math.abs(p[1] - BASKET[1]) < BASKET_HALF && p[2] < BASKET_H;
  }
}

// ---------------------------------------------------------------- 見る(画像処理)
// 画像は { width, height, data: RGBA(Uint8Array), flipY?: true なら行が下から上の順 }。

export function cameraBasis({ pos, lookAt }) {
  const f = lookAt.map((v, i) => v - pos[i]);
  const n = Math.hypot(...f);
  const fw = f.map((v) => v / n);
  const rt = [fw[1], -fw[0], 0]; // fw × (0,0,1)
  const rn = Math.hypot(...rt);
  const right = rt.map((v) => v / rn);
  const up = [right[1] * fw[2] - right[2] * fw[1], right[2] * fw[0] - right[0] * fw[2], right[0] * fw[1] - right[1] * fw[0]];
  // 列が (右, 上, 後ろ)。MuJoCo のカメラと同じ(カメラは -z を向く)
  return [right[0], up[0], -fw[0], right[1], up[1], -fw[1], right[2], up[2], -fw[2]];
}

// RGB(0〜255)→ HSV。OpenCV と同じ尺度(H は 0〜180、S・V は 0〜255)
export function rgbToHsv(r, g, b) {
  const mx = Math.max(r, g, b), mn = Math.min(r, g, b), d = mx - mn;
  let h = 0;
  if (d > 0) {
    if (mx === r) h = (60 * (g - b)) / d;
    else if (mx === g) h = 120 + (60 * (b - r)) / d;
    else h = 240 + (60 * (r - g)) / d;
    if (h < 0) h += 360;
  }
  return [h / 2, mx === 0 ? 0 : (255 * d) / mx, mx];
}

// 3x3 の収縮 → 膨張(小さなノイズを消す)。mask は Uint8Array(0/1)
function open3(mask, w, h) {
  const er = new Uint8Array(w * h);
  for (let y = 1; y < h - 1; y++) {
    for (let x = 1; x < w - 1; x++) {
      let all = 1;
      for (let dy = -1; dy <= 1 && all; dy++) for (let dx = -1; dx <= 1; dx++) if (!mask[(y + dy) * w + x + dx]) { all = 0; break; }
      er[y * w + x] = all;
    }
  }
  const out = new Uint8Array(w * h);
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (!er[y * w + x]) continue;
      for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) out[(y + dy) * w + x + dx] = 1;
    }
  }
  return out;
}

// 8 近傍の連結成分。{ labels, stats: [{ x, y, w, h, area, pixels }] }(ラベルは 1 から)
function components(mask, w, h) {
  const labels = new Int32Array(w * h);
  const stats = [null];
  const stack = [];
  for (let s = 0; s < w * h; s++) {
    if (!mask[s] || labels[s]) continue;
    const id = stats.length;
    const st = { x: w, y: h, x1: 0, y1: 0, area: 0, pixels: [] };
    labels[s] = id; stack.push(s);
    while (stack.length) {
      const p = stack.pop();
      const px = p % w, py = (p - px) / w;
      st.area++; st.pixels.push(p);
      if (px < st.x) st.x = px; if (px > st.x1) st.x1 = px;
      if (py < st.y) st.y = py; if (py > st.y1) st.y1 = py;
      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          const nx = px + dx, ny = py + dy;
          if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
          const q = ny * w + nx;
          if (mask[q] && !labels[q]) { labels[q] = id; stack.push(q); }
        }
      }
    }
    st.w = st.x1 - st.x + 1; st.h = st.y1 - st.y + 1;
    stats.push(st);
  }
  return { labels, stats };
}

export const DEFAULT_PARAMS = { threshold: 0.7, satMin: 185, hueMax: 20 };

// 画像から熟した実を見つけ、3 次元の位置(世界座標 [m])にして返す。画面に映る実を 1 個ずつ:
//   { pos, ripeness: 0〜1, harvest: 熟しているか, px: [u, v], box: [x, y, w, h] }
// 実の候補は「赤〜オレンジの鮮やかな画素」。緑の実は葉・茎と同じ色なので、そもそも拾わない。
//   - 彩度 satMin 以上: 鉢(素焼きの茶色)は彩度が低いので外れる
//   - 色相 hueMax まで(OpenCV の尺度で 20 = 40°): アーム(黄色)は外れる
//   - 赤い画素の割合 = 熟し具合。しきい値 threshold 以上なら収穫の対象
export function detectFruits(img, { threshold, satMin, hueMax } = DEFAULT_PARAMS, cam = EYE) {
  const { width: w, height: h, data } = img;
  const f = (h / 2) / Math.tan((cam.fovy * Math.PI) / 360);
  const warm = new Uint8Array(w * h), red = new Uint8Array(w * h);
  for (let y = 0; y < h; y++) {
    const sy = img.flipY ? h - 1 - y : y;
    for (let x = 0; x < w; x++) {
      const i = 4 * (sy * w + x);
      const [H, S, V] = rgbToHsv(data[i], data[i + 1], data[i + 2]);
      if (S < satMin || V < 60) continue;
      const p = y * w + x;
      if (H <= hueMax || H >= 170) warm[p] = 1;
      if (H <= 8 || H >= 170) red[p] = 1;
    }
  }
  const { stats } = components(open3(warm, w, h), w, h);
  const R = cameraBasis(cam);
  const found = [];
  for (const st of stats) {
    if (!st || st.area < 30) continue;
    let nRed = 0;
    for (const p of st.pixels) nRed += red[p];
    const ripeness = nRed / st.area; // 赤い画素の割合 = 熟し具合
    // 距離: 半径 FRUIT_R の球が画面で直径 D 画素に見える → 距離 = f · 2R / D
    const D = Math.max(st.w, st.h);
    const z = (f * 2 * FRUIT_R) / D;
    const u = st.x + st.w / 2, v = st.y + st.h / 2;
    const pc = [((u - w / 2) * z) / f, (-(v - h / 2) * z) / f, -z];
    const pos = cam.pos.map((c, i) => c + R[3 * i] * pc[0] + R[3 * i + 1] * pc[1] + R[3 * i + 2] * pc[2]);
    // 形と場所で絞る: 丸くないもの、鉢植えのまわり(作業する範囲)にないものは実ではない
    const roundness = st.area / (Math.PI * (D / 2) ** 2);
    const inArea = pos[0] > 0.18 && pos[0] < 0.40 && Math.abs(pos[1]) < 0.15 && pos[2] > 0.05 && pos[2] < 0.30;
    if (roundness < 0.55 || !inArea) continue;
    found.push({ pos, ripeness, harvest: ripeness >= threshold, px: [Math.round(u), Math.round(v)], box: [st.x, st.y, st.w, st.h] });
  }
  return found;
}

// ---------------------------------------------------------------- 収穫の動き
const rotX = (a) => [1, 0, 0, 0, Math.cos(a), -Math.sin(a), 0, Math.sin(a), Math.cos(a)];
const rotY = (a) => [Math.cos(a), 0, Math.sin(a), 0, 1, 0, -Math.sin(a), 0, Math.cos(a)];
export const approachRot = (yaw) => mul3(rotZ(yaw), rotY(PITCH));

// 1 個を収穫する。names: いま木についている実の名前
function* harvestOne(mv, robot, target, names) {
  const p = target.pos;
  const yaw = Math.atan2(p[1], p[0]);
  const rot = approachRot(yaw);
  const fwd = [rot[0], rot[3], rot[6]]; // 手先の向き(指の先の方向)
  const pre = p.map((v, i) => v - APPROACH * fwd[i]);
  const grasp = p.map((v, i) => v + TUNING.graspDepth * fwd[i]); // 実を指の奥まで入れる(指先でつまむと、運ぶ途中で落ちる)
  const up = (v, dz) => [v[0], v[1], v[2] + dz];
  yield* mv.move(up(pre, 0.02), rot, GRIP_OPEN, 1.6, "近づく(手前で止まる)");
  yield* mv.move(pre, rot, GRIP_OPEN, 0.6, "近づく(手前で止まる)");
  yield* mv.move(grasp, rot, GRIP_OPEN, 1.0, "実に手を入れる");
  yield* mv.move(grasp, rot, GRIP_CLOSED, 0.7, "指を閉じる");
  // いちばん近い実の拘束を外す(=へたを切る)。実機では、手先に付けたハサミや、ひねりで切る
  const name = names.reduce((a, b) => (dist(robot.fruitPos(a), p) <= dist(robot.fruitPos(b), p) ? a : b));
  robot.cut(name);
  yield* mv.move(grasp, rot, GRIP_CLOSED, 0.3, "へたを切る");
  yield* mv.move(pre, rot, GRIP_CLOSED, 1.2, "引き抜く");
  yield* mv.move(up(pre, 0.04), rot, GRIP_CLOSED, 0.8, "引き抜く");
  // かごまでは、手首の向きを大きく変えずに運ぶ(向きを急に変えると、遠心力で実が抜ける)
  const [bx, by] = BASKET;
  const carry = approachRot(Math.atan2(by, bx));
  const back = [carry[0], carry[3], carry[6]].map((v) => v * TUNING.carryBack);
  const at = (z) => [bx - back[0], by - back[1], z - back[2]];
  yield* mv.move(at(0.13), carry, GRIP_CLOSED, TUNING.carryTime, "かごへ運ぶ");
  yield* mv.move(at(0.085), carry, GRIP_CLOSED, 0.6, "かごへ運ぶ");
  yield* mv.move(at(0.085), carry, GRIP_OPEN, 0.5, "指を開く");
  // 実は、開いた指(下側の板)の上に乗ったままになることがある。手首を 90° ひねって、板を縦にして落とす(Python 版にはない動き。Menagerie の指の形と、こちらの逆運動学で姿勢が少し違うため必要になった)
  yield* mv.move(at(0.085), mul3(carry, rotX(TUNING.dropRoll)), GRIP_OPEN, 0.6, "手首をひねって落とす");
  yield* mv.joints(REST, GRIP_OPEN, 1.5, "待機姿勢へ戻る");
  return name;
}

const dist = (a, b) => Math.hypot(...a.map((v, i) => v - b[i]));

// 全体の作業(harvest.py の run)。1 回の yield = 制御 1 ステップ。
//   look(): 鉢植えを撮った画像を返す関数(ブラウザは three.js、テストは投影で作った画像)
//   params: () => ({ threshold, satMin, hueMax })。1 個とるたびに読み直すので、途中で変えられる
//   hooks: onLook(found, img) / onPick(record) / onLog(text)
export function* harvestRun(robot, { look, params = () => DEFAULT_PARAMS, onLook = () => {}, onPick = () => {}, onLog = () => {}, onStep = null } = {}) {
  const planner = new Planner(robot.mj, robot.model, { rotW: TUNING.rotW }); // 収穫では位置のずれのほうが困るので、向きより位置を優先する
  const mv = new Mover(robot, planner, { onStep });
  const names = FRUITS.map(([n]) => n);
  const truth = Object.fromEntries(FRUITS.map(([n, , , , r]) => [n, r]));
  yield* mv.joints(REST, GRIP_OPEN, 0.8, "見る");
  let first = true;
  const picked = [];
  for (let attempt = 0; attempt < 8; attempt++) {
    yield* mv.joints(REST, GRIP_OPEN, 0.4, "見る");
    const img = look();
    const p = params();
    const found = detectFruits(img, p);
    onLook(found, img);
    if (first) {
      first = false;
      onLog(`見つけた実: ${found.length} 個(赤〜オレンジの実だけ。緑の実は葉と区別しないので数えない)`);
      for (const fr of [...found].sort((a, b) => b.ripeness - a.ripeness)) {
        const near = names.reduce((a, b) => (dist(robot.fruitPos(a), fr.pos) <= dist(robot.fruitPos(b), fr.pos) ? a : b));
        const err = dist(robot.fruitPos(near), fr.pos) * 1000;
        onLog(`  赤さ ${(fr.ripeness * 100).toFixed(0).padStart(3)}% → ${fr.harvest ? "収穫する" : "まだ"}(本当の熟し具合 ${truth[near].toFixed(2)}、位置の誤差 ${err.toFixed(1)} mm)`);
      }
    }
    const todo = found.filter((fr) => fr.harvest);
    if (!todo.length) break;
    const target = todo.reduce((a, b) => (a.pos[2] >= b.pos[2] ? a : b)); // 上の実から採る(下の実に当たらないように)
    const onPlant = names.filter((n) => robot.attached(n));
    const name = yield* harvestOne(mv, robot, target, onPlant);
    picked.push(name);
    const inB = robot.inBasket(name);
    onPick({ attempt: attempt + 1, name, ripeness: truth[name], basket: inB });
    onLog(`${attempt + 1} 回目: ${name}(熟し具合 ${truth[name].toFixed(2)})を収穫 → ${inB ? "かごに入った" : "かごに入らなかった"}`);
  }
  yield* mv.joints(REST, GRIP_OPEN, 1.0, "完了");
  planner.dispose();
  return picked;
}

// 結果: 実ごとに { ripeness, picked, basket, moved(木に残った実が動いた量 mm) }
export function harvestResult(robot) {
  return Object.fromEntries(FRUITS.map(([n, x, y, z, r]) => {
    const p = robot.fruitPos(n);
    return [n, { ripeness: r, picked: !robot.attached(n), basket: robot.inBasket(n), moved: dist(p, [x, y, z]) * 1000 }];
  }));
}

export { MOTORS };
