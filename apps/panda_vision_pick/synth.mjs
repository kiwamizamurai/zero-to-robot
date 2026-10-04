// テスト用の合成画像。既知の位置・色の「ブロックの上の面」(四角)と球を、指定のカメラから見た画像として作る。
// 光線と平面(床 z=0、ブロックの上の面 z=0.04)・球の交点を画素ごとに求めるだけの小さな描画器。
import { gauss } from "../_web/math.js";

const BOX_CENTER = [0.40, -0.38], BOX_HALF = 0.10, BOX_WALL = 0.008;

// blocks: [{ xy, yaw, rgba }] / spheres: [{ c: [x, y, z], r, rgba }] / cam: { R, t }
export function renderSynthetic({ size = 352, fovy = 42, cam, blocks = [], spheres = [], noise = 0, rnd = null, speckles = 0 }) {
  const f = size / 2 / Math.tan((fovy * Math.PI) / 360), { R, t } = cam;
  const data = new Uint8ClampedArray(4 * size * size), depth = new Float32Array(size * size);
  const topZ = 0.04;
  for (let v = 0; v < size; v++) {
    for (let u = 0; u < size; u++) {
      const x = (u + 0.5 - size / 2) / f, y = -(v + 0.5 - size / 2) / f;
      const dir = [0, 1, 2].map((k) => R[3 * k] * x + R[3 * k + 1] * y - R[3 * k + 2]); // カメラの前方向の成分が 1 なので、距離 = 深度
      let best = Infinity, color = [0.1, 0.1, 0.12];
      const hit = (s, c) => { if (s > 0 && s < best) { best = s; color = c; } };
      // 床と箱のふち
      if (dir[2] < 0) {
        const s = -t[2] / dir[2], px = t[0] + s * dir[0], py = t[1] + s * dir[1];
        const dx = Math.abs(px - BOX_CENTER[0]), dy = Math.abs(py - BOX_CENTER[1]);
        const wall = Math.max(dx, dy) > BOX_HALF - BOX_WALL && Math.max(dx, dy) < BOX_HALF + BOX_WALL && dx < BOX_HALF + BOX_WALL && dy < BOX_HALF + BOX_WALL;
        hit(s, wall ? [0.55, 0.42, 0.3] : [0.85, 0.84, 0.8]);
      }
      // ブロックの上の面
      if (dir[2] < 0) {
        const s = (topZ - t[2]) / dir[2];
        const px = t[0] + s * dir[0], py = t[1] + s * dir[1];
        for (const b of blocks) {
          const c = Math.cos(b.yaw), sn = Math.sin(b.yaw), rx = px - b.xy[0], ry = py - b.xy[1];
          const lx = c * rx + sn * ry, ly = -sn * rx + c * ry;
          if (Math.abs(lx) <= 0.02 && Math.abs(ly) <= 0.02) hit(s, b.rgba.slice(0, 3).map((q) => q * 0.95));
        }
      }
      // 球
      for (const sp of spheres) {
        const oc = [t[0] - sp.c[0], t[1] - sp.c[1], t[2] - sp.c[2]];
        const a = dir[0] ** 2 + dir[1] ** 2 + dir[2] ** 2, bq = oc[0] * dir[0] + oc[1] * dir[1] + oc[2] * dir[2];
        const disc = bq * bq - a * (oc[0] ** 2 + oc[1] ** 2 + oc[2] ** 2 - sp.r * sp.r);
        if (disc >= 0) hit((-bq - Math.sqrt(disc)) / a, sp.rgba.slice(0, 3).map((q) => q * 0.9));
      }
      const i = v * size + u;
      depth[i] = best;
      for (let k = 0; k < 3; k++) data[4 * i + k] = (color[k] + (noise ? (gauss(rnd) * noise) / 255 : 0)) * 255;
      data[4 * i + 3] = 255;
    }
  }
  // 孤立した小さな色の点(雑音)。大きさ 2x2 画素
  for (let k = 0; k < speckles; k++) {
    const u = 5 + Math.floor(rnd() * (size - 10)), v = 5 + Math.floor(rnd() * (size - 10));
    for (const [du, dv] of [[0, 0], [1, 0], [0, 1], [1, 1]]) {
      const i = (v + dv) * size + u + du;
      data.set([220, 40, 40, 255], 4 * i);
    }
  }
  return { image: { width: size, height: size, data }, depth };
}
