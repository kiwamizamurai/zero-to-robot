// カメラで見つけて片付けるシミュレーション(vision_pick.py の移植)。
// 見つける部分(画像 → 位置・向き)は vision.js の純粋な関数。ここは MuJoCo の場面づくりと、天井カメラ、作業の流れ。
//
// 天井カメラの画像は 2 通りの方法で撮れる。
//  - RayCamera: MuJoCo の mj_multiRay(光線を画素ごとに飛ばす)で色と深度を作る。Node でもブラウザでも同じに動く(既定)
//  - ブラウザの 3D 画面から撮る(view.js が sim.shooter に差し込む)。深度は 1 点だけ光線で測る
import { PandaCell, BLOCK, BOX_CENTER, BOX_HALF, SAFE_Z, GRIP_OPEN } from "../_web/panda.js";
import { mulberry32 } from "../_web/math.js";
import { COLORS, detectBlocks } from "./vision.js";

export * from "./vision.js";
export { BOX_CENTER, BOX_HALF, BLOCK };

export const PICK_AREA = { x: [0.35, 0.68], y: [-0.15, 0.35] }; // ブロックを置く範囲 [m]
export const DROP_SPOTS = [[-0.045, -0.045], [0.045, 0.045], [0.045, -0.045], [-0.045, 0.045]];
export const OVERHEAD = { pos: [0.50, 0.08, 1.10], fovy: 42.0, size: 352 };
export const LOOK_POSE = [0.05, -0.42, 0.40]; // 撮影のあいだ、手先をカメラの視界の外へどける
export const NOMINAL_CAMERA = { R: [1, 0, 0, 0, 1, 0, 0, 0, 1], t: OVERHEAD.pos }; // 設計図どおり: 真下を向く(回転なし)

// ブロックを互いに 9 cm 以上離して、ランダムな位置・向きに置く
export function randomBlocks(seed) {
  const rnd = mulberry32(seed + 1);
  const uni = (lo, hi) => lo + (hi - lo) * rnd();
  const placed = [];
  for (const { name, rgba } of COLORS) {
    let xy;
    do {
      xy = [uni(...PICK_AREA.x), uni(...PICK_AREA.y)];
    } while (!placed.every((p) => Math.hypot(xy[0] - p.xy[0], xy[1] - p.xy[1]) > 0.09));
    placed.push({ name, xy, rgba, yaw: uni(-Math.PI / 4, Math.PI / 4) });
  }
  return placed;
}

// 箱の中(ふちの外 2 cm まで)。ブロックを見つけるときに捨てる範囲
export const inBoxArea = (p) => Math.abs(p[0] - BOX_CENTER[0]) < BOX_HALF + 0.02 && Math.abs(p[1] - BOX_CENTER[1]) < BOX_HALF + 0.02;

// 光線で撮るカメラ。画素ごとに光線を飛ばし、当たった物の色(法線で簡単な陰影つき)と距離(深度)を返す
export class RayCamera {
  constructor(mj, { size = OVERHEAD.size, fovy = OVERHEAD.fovy } = {}) {
    this.mj = mj; this.size = size; this.fovy = fovy;
    const n = size * size;
    this.geomId = new mj.IntBuffer(n);
    this.dist = new mj.DoubleBuffer(n);
    this.normal = new mj.DoubleBuffer(3 * n);
    this.one = { g: new mj.IntBuffer(1), d: new mj.DoubleBuffer(1), n: new mj.DoubleBuffer(3) };
    // カメラ座標での光線の向き(前方向の成分が -1 なので、返る距離がそのまま深度になる)
    const f = size / 2 / Math.tan((fovy * Math.PI) / 360);
    this.local = new Float64Array(3 * n);
    for (let v = 0; v < size; v++) {
      for (let u = 0; u < size; u++) {
        const i = 3 * (v * size + u);
        this.local[i] = (u + 0.5 - size / 2) / f;
        this.local[i + 1] = -(v + 0.5 - size / 2) / f;
        this.local[i + 2] = -1;
      }
    }
    this.world = new Float64Array(3 * n);
  }

  // cam = { R, t }: 本当のカメラの位置・向き。{ image, depth } を返す
  shoot(model, data, cam) {
    const { mj, size } = this, n = size * size, { R, t } = cam;
    const L = this.local, W = this.world;
    for (let i = 0; i < n; i++) {
      const x = L[3 * i], y = L[3 * i + 1], z = L[3 * i + 2];
      for (let k = 0; k < 3; k++) W[3 * i + k] = R[3 * k] * x + R[3 * k + 1] * y + R[3 * k + 2] * z;
    }
    mj.mj_multiRay(model, data, Array.from(t), W, null, true, -1, this.geomId, this.dist, this.normal, n, 1e9);
    const gid = this.geomId.GetView(), dist = this.dist.GetView(), nrm = this.normal.GetView();
    const rgba = model.geom_rgba;
    const img = new Uint8ClampedArray(4 * n), depth = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      const g = gid[i];
      depth[i] = g < 0 ? 1e3 : dist[i];
      let r = 0.1, gg = 0.1, b = 0.12; // 何にも当たらなかった(空)
      if (g >= 0) {
        const explicit = Math.abs(rgba[4 * g] - 0.5) > 1e-6 || Math.abs(rgba[4 * g + 1] - 0.5) > 1e-6;
        [r, gg, b] = explicit ? [rgba[4 * g], rgba[4 * g + 1], rgba[4 * g + 2]] : [0.92, 0.92, 0.92]; // 色指定なしはアームの白い外装
        // 上から斜めに当たる光。上向きの面は明るく、横の面は暗い
        const lit = Math.max(0, (0.35 * nrm[3 * i] - 0.25 * nrm[3 * i + 1] + nrm[3 * i + 2]) / 1.2);
        const shade = 0.45 + 0.55 * Math.min(1, lit);
        r *= shade; gg *= shade; b *= shade;
      }
      img[4 * i] = r * 255; img[4 * i + 1] = gg * 255; img[4 * i + 2] = b * 255; img[4 * i + 3] = 255;
    }
    return { image: { width: size, height: size, data: img }, depth };
  }

  // 画素 (u, v) の 1 点だけの深度
  depthAt(model, data, cam, u, v) {
    const { mj, size } = this, f = size / 2 / Math.tan((this.fovy * Math.PI) / 360), { R, t } = cam;
    const x = (u - size / 2) / f, y = -(v - size / 2) / f, z = -1;
    const dir = [0, 1, 2].map((k) => R[3 * k] * x + R[3 * k + 1] * y + R[3 * k + 2] * z);
    mj.mj_multiRay(model, data, Array.from(t), dir, null, true, -1, this.one.g, this.one.d, this.one.n, 1, 1e9);
    return this.one.d.GetView()[0];
  }

  dispose() {
    for (const b of [this.geomId, this.dist, this.normal, this.one.g, this.one.d, this.one.n]) b.delete();
  }
}

const wrapDiff = (a, b) => { const q = Math.PI / 2; return ((((a - b + q / 2) % q) + q) % q) - q / 2; };

export class VisionPickSim extends PandaCell {
  // PandaCell.reset を、seed からブロックの置き方を決める形にする(seed を指定しなければ blocks をそのまま使う)
  reset(opts = {}) {
    const seed = opts.seed ?? this.opts?.seed ?? 0;
    const blocks = opts.blocks ?? randomBlocks(seed);
    super.reset({ ...opts, seed, jitter: 0, blocks });
    this.cameraCfg = { size: OVERHEAD.size, fovy: OVERHEAD.fovy };
    this.ray?.dispose();
    this.ray = new RayCamera(this.mj, this.cameraCfg);
    this.extrinsics = null; // 「こうだと信じているカメラの位置・向き」。null のあいだは本当の値を使う
    this.shooter = this.shooter ?? null; // ブラウザが差し込む撮り方: (sim, cam) => { image, depth | depthAt }
    this.shots = [];
    this.attempts = 0;
    this.placed = 0;
    this.onShot = this.onShot ?? null;
  }

  dispose() {
    this.ray?.dispose();
    this.ray = null;
    super.dispose();
  }

  // 本当のカメラの位置・向き(サブクラスがずらす)
  trueCamera() { return NOMINAL_CAMERA; }

  rayDepth(cam, u, v) { return this.ray.depthAt(this.model, this.data, cam, u, v); }

  // 撮って(画像と深度)、見つけて、世界座標に直す。信じているカメラの姿勢(this.extrinsics)で位置を出す
  observe() {
    const cam = this.trueCamera();
    const shot = this.shooter ? this.shooter(this, cam) : this.ray.shoot(this.model, this.data, cam);
    const { found, labels } = detectBlocks(shot.image, {
      depth: shot.depth ?? null, depthAt: shot.depthAt ?? null,
      extrinsics: this.extrinsics ?? cam, fovy: this.cameraCfg.fovy, inBox: inBoxArea,
    });
    const errors = {};
    for (const f of found) {
      const tp = this.blockPos(f.name), q = this.data.body(`block_${f.name}`).xquat;
      f.posError = Math.hypot(f.pos[0] - tp[0], f.pos[1] - tp[1]); // [m]
      f.yawError = wrapDiff(f.yaw, 2 * Math.atan2(q[3], q[0])); // [rad]
      errors[f.name] = f.posError;
    }
    const result = { image: shot.image, found, labels, errors, time: this.time };
    this.shots.push(result);
    this.onShot?.(result);
    return result;
  }

  // 撮影のために手先をカメラの視界の外へどけて、揺れが収まるのを待つ
  *lookAway() {
    this.ik.yaw = 0;
    yield* this.move(LOOK_POSE, 1.4);
    yield* this.gripper(this.grip, 0.3);
  }

  // 全体の作業(vision_pick.py の run): 見る → 見つける → 一番近いものを運ぶ、を撮り直しながら繰り返す
  *run() {
    this.attempts = 0; this.placed = 0;
    while (this.attempts < 8) {
      this.attempts++;
      yield* this.lookAway();
      const { found } = this.observe();
      if (this.attempts === 1) {
        for (const f of found) {
          this.log.push(`見つけた ${f.name}: 位置の誤差 ${(f.posError * 1000).toFixed(1)} mm, 向きの誤差 ${((f.yawError * 180) / Math.PI).toFixed(1)}°`);
        }
      }
      if (!found.length) { this.log.push("箱の外に何も見つからなかったので終了"); break; }
      const target = found.reduce((a, b) => (Math.hypot(a.pos[0], a.pos[1]) <= Math.hypot(b.pos[0], b.pos[1]) ? a : b)); // 近いものから
      this.log.push(`${this.attempts} 回目: ${target.name} をつかみに行く`);
      // 箱の中の置き場所は、すでに入っている個数で決める(つかみ損ねてやり直しても、同じ場所に重ねない)
      const inside = Object.values(this.results()).filter(Boolean).length;
      yield* this.pickAndPlace(target.name, DROP_SPOTS[inside % DROP_SPOTS.length], { pos: target.pos, yaw: target.yaw });
      this.placed++;
    }
    yield* this.gripper(GRIP_OPEN, 1.0);
    const res = this.results();
    this.log.push(`つかみに行った回数 ${this.placed} / ブロック ${this.blocks.length} 個、箱の中 ${Object.values(res).filter(Boolean).length} 個`);
  }

  // 場面を初期状態(ブロックは最初の置き方、アームはホーム姿勢)に戻す。モデルは作り直さない
  restoreState() {
    const mj = this.mj, m = this.model, d = this.data;
    mj.mj_resetData(m, d);
    const home = m.key("home");
    d.qpos.set(home.qpos.slice(0, 9));
    d.ctrl.set(home.ctrl);
    mj.mj_forward(m, d);
    this.ik.q = [...this.ik.qHome];
    this.ik.yaw = 0;
    this.ik.forward();
    this.grip = GRIP_OPEN;
  }
}
export { VisionPickSim as default, SAFE_Z };
