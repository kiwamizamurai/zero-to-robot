// ハンドアイ・キャリブレーションのシミュレーション(calibrate.py の移植)。
// 天井カメラの撮影・検出・つかむ流れは ../panda_vision_pick/sim.js をそのまま使い、ここでは
//  - 本当のカメラを設計図からずらして取り付ける
//  - 手の横にマゼンタの印(球)を付けて、12 か所へ動かして「ロボットが知っている位置」と「カメラが見た位置」を集める
//  - その組から Kabsch 法(SVD)でカメラの本当の位置・向きを求める
// の 3 段階(A. 設計図を信じる → B. 測り直す → C. 測り直した値で見つける)を作る。
import { mountFiles, writeText, siteId } from "../_web/mujoco.js";
import { PANDA_FILES } from "../_web/panda.js";
import {
  COLORS, DROP_SPOTS, LOOK_POSE, NOMINAL_CAMERA, OVERHEAD, VisionPickSim, findMarker,
} from "../panda_vision_pick/sim.js";
import { GRIP_OPEN, BOX_CENTER, BOX_HALF } from "../_web/panda.js";
import { kabsch, rotDiffDeg, mulMat } from "./kabsch.js";

export * from "./kabsch.js";
export { COLORS, DROP_SPOTS, LOOK_POSE, NOMINAL_CAMERA, OVERHEAD, BOX_CENTER, BOX_HALF, findMarker };

// 本当のカメラは、設計図(OVERHEAD)から少しずれて付いている
export const MOUNT_SHIFT = [0.03, -0.02, 0.01]; // [m]
export const MOUNT_TILT_DEG = [2.0, -2.0, 1.5]; // x, y, z 軸まわり [°]
export const MARKER_R = 0.012; // 手に付けた印(球)の半径 [m]
export const MARKER_POS_IN_HAND = [0.075, 0.0, 0.03];
export const CALIB_POINTS = [];
for (const z of [0.18, 0.32]) for (const x of [0.40, 0.58]) for (const y of [-0.12, 0.08, 0.28]) CALIB_POINTS.push([x, y, z]);

const rad = (d) => (d * Math.PI) / 180;
const Rx = (a) => [1, 0, 0, 0, Math.cos(a), -Math.sin(a), 0, Math.sin(a), Math.cos(a)];
const Ry = (a) => [Math.cos(a), 0, Math.sin(a), 0, 1, 0, -Math.sin(a), 0, Math.cos(a)];
const Rz = (a) => [Math.cos(a), -Math.sin(a), 0, Math.sin(a), Math.cos(a), 0, 0, 0, 1];

// scale 倍のずれで取り付けたカメラの本当の位置・向き(scale = 1 が README の 3.7 cm・3.2°)
export function mountedCamera(scale = 1) {
  const R = mulMat(mulMat(Rx(rad(MOUNT_TILT_DEG[0] * scale)), Ry(rad(MOUNT_TILT_DEG[1] * scale))), Rz(rad(MOUNT_TILT_DEG[2] * scale)));
  return { R, t: OVERHEAD.pos.map((x, i) => x + MOUNT_SHIFT[i] * scale) };
}

// ずれの大きさ: 位置 [mm]、向き [°]
export function mountError(scale) {
  return { mm: Math.hypot(...MOUNT_SHIFT) * scale * 1000, deg: rotDiffDeg(NOMINAL_CAMERA.R, mountedCamera(scale).R) };
}

export const poseError = (a, b) => ({ mm: Math.hypot(a.t[0] - b.t[0], a.t[1] - b.t[1], a.t[2] - b.t[2]) * 1000, deg: rotDiffDeg(a.R, b.R) });

export class CalibrationSim extends VisionPickSim {
  // Panda の手に印(マゼンタの球と、位置を読むための site)を足したモデルを読み込む。
  // 手(hand)は panda.xml の中にあるので、仮想ファイルシステム上の panda.xml の文字列に差し込む(元のファイルは変えない)
  static async create(mujoco, load, opts = {}) {
    await mountFiles(mujoco, "/panda", PANDA_FILES, (f) => load(`assets/panda/${f}`));
    const xml = mujoco.FS.readFile("/panda/panda.xml", { encoding: "utf8" });
    const hook = '<site name="tcp" pos="0 0 0.1034"/>';
    if (!xml.includes(hook)) throw new Error("panda.xml に tcp の site が見つかりません");
    const p = MARKER_POS_IN_HAND.join(" ");
    writeText(mujoco, "/panda/panda.xml", xml.replace(hook, `${hook}
                      <geom name="marker" type="sphere" size="${MARKER_R}" pos="${p}" rgba="0.9 0.1 0.8 1" contype="0" conaffinity="0" mass="0.001"/>
                      <site name="marker" pos="${p}"/>`));
    return new this(mujoco, opts);
  }

  reset(opts = {}) {
    super.reset(opts);
    this.mountScale = opts.mountScale ?? this.mountScale ?? 1;
    this.cam = mountedCamera(this.mountScale);
    this.extrinsics = NOMINAL_CAMERA; // 最初は設計図どおりだと信じている
    this.markerSite = siteId(this.mj, this.model, "marker");
    this.stage = null;
    this.stages = {};     // A / B / C ごとの結果
    this.markerPoints = []; // B で集めた点 { cam, world, seen, k }
    this.onStage = this.onStage ?? null;
    this.onMarker = this.onMarker ?? null;
  }

  trueCamera() { return this.cam; }

  markerWorld() {
    const p = this.data.site_xpos, i = 3 * this.markerSite;
    return [p[i], p[i + 1], p[i + 2]];
  }

  // 撮って(A と C)、見つけた全部を 1 回ずつつかみに行く。見つけた位置の誤差と、箱に入った数を返す
  *tryPicks(label, extrinsics) {
    this.restoreState();
    this.extrinsics = extrinsics;
    yield* this.lookAway();
    const shot = this.observe();
    const sorted = [...shot.found].sort((a, b) => Math.hypot(a.pos[0], a.pos[1]) - Math.hypot(b.pos[0], b.pos[1]));
    for (let i = 0; i < sorted.length; i++) {
      const f = sorted[i];
      yield* this.pickAndPlace(f.name, DROP_SPOTS[i % DROP_SPOTS.length], { pos: f.pos, yaw: f.yaw });
    }
    yield* this.gripper(GRIP_OPEN, 1.0);
    const inBox = this.results();
    const ok = Object.values(inBox).filter(Boolean).length;
    const errMm = Object.fromEntries(shot.found.map((f) => [f.name, f.posError * 1000]));
    const vals = Object.values(errMm);
    const result = {
      label, found: shot.found, errMm, inBox, ok, shot,
      meanErrMm: vals.length ? vals.reduce((a, b) => a + b, 0) / vals.length : NaN,
      blocks: this.blocks.map((b) => ({ name: b.name, xy: b.xy, rgba: b.rgba })),
    };
    this.log.push(`[${label}] 見つけた位置の誤差: ${Object.entries(errMm).map(([n, e]) => `${n} ${e.toFixed(0)} mm`).join(", ")} → 箱に入ったのは ${ok} / ${COLORS.length} 個`);
    return result;
  }

  // 手の印を 12 か所へ動かし、ロボットが知っている位置とカメラが見た位置の組を集めて、カメラの位置・向きを解く
  *calibrate() {
    this.restoreState();
    this.markerPoints = [];
    for (let k = 0; k < CALIB_POINTS.length; k++) {
      this.ik.yaw = rad(20 * ((k % 3) - 1)); // 手首の向きも少しずつ変える
      yield* this.move(CALIB_POINTS[k], 1.0);
      yield* this.gripper(this.grip, 0.3); // 揺れが収まるのを待ってから撮る
      const shot = this.ray.shoot(this.model, this.data, this.cam);
      const seen = findMarker(shot.image, { depth: shot.depth, fovy: this.cameraCfg.fovy, radius: MARKER_R });
      const pt = { k, seen: !!seen, cam: seen?.center ?? null, world: this.markerWorld(), image: shot.image, uv: seen ? [seen.u, seen.v] : null };
      this.markerPoints.push(pt);
      this.onMarker?.(pt, this.markerPoints);
      this.log.push(seen ? `${k + 1}: 印が見えた(使う点は ${this.markerPoints.filter((p) => p.seen).length} 個)` : `${k + 1}: 印が見えなかった(腕に隠れたか、視界の外)ので使わない`);
    }
    const used = this.markerPoints.filter((p) => p.seen);
    if (used.length < 3) {
      this.log.push("印が 3 か所以上で見えなかったので、キャリブレーションできない");
      return null;
    }
    const sol = kabsch(used.map((p) => p.cam), used.map((p) => p.world));
    const est = { R: sol.R, t: sol.t };
    const result = {
      est, n: used.length, rmsMm: sol.rms * 1000,
      before: poseError(NOMINAL_CAMERA, this.cam), after: poseError(est, this.cam), points: used.map((p) => p.world),
    };
    this.log.push(`[B キャリブレーション] 使った点 ${used.length} 個、当てはまりの誤差(RMS) ${result.rmsMm.toFixed(1)} mm`);
    this.log.push(`  設計図のカメラ: 位置のずれ ${result.before.mm.toFixed(1)} mm, 向きのずれ ${result.before.deg.toFixed(2)}°`);
    this.log.push(`  測り直した後  : 位置のずれ ${result.after.mm.toFixed(1)} mm, 向きのずれ ${result.after.deg.toFixed(2)}°`);
    return result;
  }

  // 全体: A. 設計図を信じる → B. キャリブレーション → C. 測り直した位置・向きで見つけ直す
  *run() {
    this.stages = {};
    this.stage = "A";
    this.onStage?.("A", null);
    this.stages.A = yield* this.tryPicks("A 設計図どおりと信じる", NOMINAL_CAMERA);
    this.onStage?.("A", this.stages.A);
    this.stage = "B";
    this.onStage?.("B", null);
    this.stages.B = yield* this.calibrate();
    this.onStage?.("B", this.stages.B);
    this.stage = "C";
    this.onStage?.("C", null);
    this.stages.C = yield* this.tryPicks("C キャリブレーション後", this.stages.B ? this.stages.B.est : NOMINAL_CAMERA);
    this.onStage?.("C", this.stages.C);
    this.stage = "done";
  }
}
export { CalibrationSim as default };
