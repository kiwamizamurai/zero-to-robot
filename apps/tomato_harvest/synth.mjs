// Node には WebGL が無いので、シミュレーターの状態(実・鉢・葉の位置)をカメラへ投影して、検出用の画像を作る。
// ブラウザでは three.js で描いた画像を使う。ここでは、影なし・色そのままの描き方と同じ見た目になるようにしてある。
import { FRUITS, FRUIT_R, LEAVES, POT, EYE, ripenessRgba, cameraBasis } from "./sim.js";

export function renderEye(robot, cam = EYE, { noise = 3, seed = 1 } = {}) {
  const { width: w, height: h } = cam;
  const data = new Uint8Array(w * h * 4);
  const f = (h / 2) / Math.tan((cam.fovy * Math.PI) / 360);
  const R = cameraBasis(cam);
  // 世界座標 → 画素 [u, v, 奥行き]
  const project = (p) => {
    const d = p.map((v, i) => v - cam.pos[i]);
    const x = R[0] * d[0] + R[3] * d[1] + R[6] * d[2];
    const y = R[1] * d[0] + R[4] * d[1] + R[7] * d[2];
    const z = -(R[2] * d[0] + R[5] * d[1] + R[8] * d[2]);
    return [w / 2 + (f * x) / z, h / 2 - (f * y) / z, z];
  };
  let s = seed;
  const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
  const put = (x, y, rgb) => {
    const i = 4 * (y * w + x);
    for (let k = 0; k < 3; k++) data[i + k] = Math.max(0, Math.min(255, Math.round(rgb[k] * 255 + (rnd() * 2 - 1) * noise)));
    data[i + 3] = 255;
  };
  const fillAll = (rgb) => { for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) put(x, y, rgb); };
  const disc = (p, r, rgb) => {
    const [u, v, z] = project(p);
    const rp = (f * r) / z;
    for (let y = Math.max(0, Math.floor(v - rp)); y <= Math.min(h - 1, Math.ceil(v + rp)); y++)
      for (let x = Math.max(0, Math.floor(u - rp)); x <= Math.min(w - 1, Math.ceil(u + rp)); x++)
        if ((x + 0.5 - u) ** 2 + (y + 0.5 - v) ** 2 <= rp * rp) put(x, y, rgb);
  };
  fillAll([0.80, 0.78, 0.72]); // 床
  // 鉢(素焼きの茶色): 円柱の上下の円の投影を囲む四角
  const pts = [];
  for (let k = 0; k < 24; k++) for (const z of [0, 0.06]) pts.push(project([POT[0] + 0.05 * Math.cos((k * Math.PI) / 12), POT[1] + 0.05 * Math.sin((k * Math.PI) / 12), z]));
  const [u0, u1] = [Math.min(...pts.map((q) => q[0])), Math.max(...pts.map((q) => q[0]))];
  const [v0, v1] = [Math.min(...pts.map((q) => q[1])), Math.max(...pts.map((q) => q[1]))];
  for (let y = Math.max(0, Math.floor(v0)); y <= Math.min(h - 1, Math.ceil(v1)); y++) for (let x = Math.max(0, Math.floor(u0)); x <= Math.min(w - 1, Math.ceil(u1)); x++) put(x, y, [0.7, 0.4, 0.25]);
  // 茎・葉・実を、奥から順に描く(実のほうが手前になることが多い)
  const items = [];
  for (let z = 0.07; z < 0.28; z += 0.01) items.push({ p: [POT[0], POT[1], z], r: 0.006, rgb: [0.25, 0.5, 0.18] });
  for (const { a, z, r } of LEAVES) items.push({ p: [POT[0] + r * Math.cos(a), POT[1] + r * Math.sin(a), z], r: 0.022, rgb: [0.22, 0.55, 0.2] });
  for (const [name, , , , rp] of FRUITS) {
    const q = robot.fruitPos(name);
    items.push({ p: q, r: FRUIT_R, rgb: ripenessRgba(rp).slice(0, 3) });
  }
  items.sort((a, b) => project(b.p)[2] - project(a.p)[2]);
  for (const it of items) disc(it.p, it.r, it.rgb);
  return { width: w, height: h, data, flipY: false };
}
