// 画像からブロックを見つける部分(OpenCV の代わりの小さな自前実装)。DOM にも MuJoCo にも依存しない純粋な関数だけ。
// 画像は { width, height, data: Uint8ClampedArray | Uint8Array(RGBA) } で、行は上から下の順。
//   detectBlocks(image, { depth | depthAt, extrinsics, fovy })  → 見つけたブロックの位置・向き
//   findMarker(image, { depth, fovy })                          → キャリブレーション用の印(マゼンタの球)

export const BLOCK = 0.02; // ブロックの半辺 [m](panda.js と同じ値)

// 色の定義。hue は OpenCV と同じ HSV の色相(0〜180 が一周。度数の半分)の範囲 [lo, hi)
export const COLORS = [
  { name: "red", rgba: [0.85, 0.20, 0.18, 1], hue: [[0, 8], [172, 180]] },
  { name: "blue", rgba: [0.18, 0.45, 0.90, 1], hue: [[85, 130]] },
  { name: "green", rgba: [0.20, 0.70, 0.30, 1], hue: [[50, 80]] },
  { name: "yellow", rgba: [0.95, 0.80, 0.15, 1], hue: [[20, 35]] },
];
// 「鮮やかで明るい画素」の下限(床・箱・影・アームをよける)。OpenCV の 0〜255 の値
export const SAT_MIN = 64, VAL_MIN = 90;
export const MARKER_HUE = [140, 165]; // マゼンタ(紫がかったピンク)

// RGB(0〜255)→ HSV。OpenCV と同じ尺度: H は 0〜180、S と V は 0〜255
export function rgbToHsv(r, g, b) {
  const mx = Math.max(r, g, b), mn = Math.min(r, g, b), d = mx - mn;
  const v = mx, s = mx === 0 ? 0 : (255 * d) / mx;
  let h = 0;
  if (d > 0) {
    if (mx === r) h = (60 * (g - b)) / d;
    else if (mx === g) h = 120 + (60 * (b - r)) / d;
    else h = 240 + (60 * (r - g)) / d;
    if (h < 0) h += 360;
  }
  return [Math.round(h / 2) % 180, s, v];
}

const inRanges = (h, ranges) => ranges.some(([lo, hi]) => h >= lo && h < hi);

// 3x3 の「開く」処理(縮めてから広げる)。ごま塩のような孤立した画素を消す。画像の外は無視する
export function morphOpen(mask, w, h) {
  const pass = (src, erode) => {
    const out = new Uint8Array(w * h);
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        let all = 1, any = 0;
        for (let dy = -1; dy <= 1; dy++) {
          const yy = y + dy;
          if (yy < 0 || yy >= h) continue;
          for (let dx = -1; dx <= 1; dx++) {
            const xx = x + dx;
            if (xx < 0 || xx >= w) continue;
            if (src[yy * w + xx]) any = 1; else all = 0;
          }
        }
        out[y * w + x] = erode ? all : any;
      }
    }
    return out;
  };
  return pass(pass(mask, true), false);
}

// 連結成分(8 近傍)。各成分の画素の添字の配列を返す
export function components(mask, w, h) {
  const seen = new Uint8Array(w * h), out = [];
  for (let i0 = 0; i0 < w * h; i0++) {
    if (!mask[i0] || seen[i0]) continue;
    const stack = [i0], pix = [];
    seen[i0] = 1;
    while (stack.length) {
      const i = stack.pop();
      pix.push(i);
      const x = i % w, y = (i / w) | 0;
      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          const xx = x + dx, yy = y + dy;
          if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
          const j = yy * w + xx;
          if (mask[j] && !seen[j]) { seen[j] = 1; stack.push(j); }
        }
      }
    }
    out.push(pix);
  }
  return out;
}

// 正方形のかたまりの中心と傾き(1 辺の向き。90° ごとに同じ)。
// 中心は画素の重心。傾きは「4 回回して重ねる」モーメント: 点 (dx, dy) を複素数 z とみなして Σ z^4 の偏角を見る。
// 正方形は 90° 回すと同じ形なので z^4 の向きがそろい、辺が軸に沿っていれば角(遠い点)が引いて負の向きになる。
// OpenCV の minAreaRect(輪郭を囲む最小の長方形)は、縁が階段状の粗い画像だと数度ずれることがあった。全画素を使うこちらの方が安定する。
export function squareFit(pts) {
  const n = pts.length;
  let cx = 0, cy = 0;
  for (const [x, y] of pts) { cx += x; cy += y; }
  cx /= n; cy /= n;
  let re = 0, im = 0;
  for (const [x, y] of pts) {
    const dx = x - cx, dy = y - cy;
    const r2 = dx * dx - dy * dy, i2 = 2 * dx * dy; // z^2
    re += r2 * r2 - i2 * i2; im += 2 * r2 * i2;       // z^4
  }
  const angle = (Math.atan2(im, re) - Math.PI) / 4;
  const half = Math.sqrt(n) / 2, c = Math.cos(angle), s = Math.sin(angle);
  const corners = [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([a, b]) => [cx + half * (a * c - b * s), cy + half * (a * s + b * c)]);
  return { center: [cx, cy], angle, corners };
}

// 立方体は 90° 回しても同じ形なので、向きは -45°〜+45° にそろえる(手首のひねりを最小にする)
export function wrapYaw(a) {
  const q = Math.PI / 2;
  return ((((a + q / 2) % q) + q) % q) - q / 2;
}

const median = (arr) => {
  const a = Float32Array.from(arr).sort();
  return a[a.length >> 1];
};

// 画角から焦点距離(画素単位)を出す。カメラの内部パラメータ
export function intrinsics(width, height, fovy) {
  return { f: height / 2 / Math.tan((fovy * Math.PI) / 360), cx: width / 2, cy: height / 2 };
}

// 画素 (u, v) と深度 d [m] を、カメラから見た 3 次元の点へ。MuJoCo/three.js のカメラは自分の -z 方向を見る
// (画像の右が +x、上が +y)。画像の下向き(v が増える向き)は -y
export function pixelToCamera(u, v, d, { f, cx, cy }) {
  return [((u - cx) * d) / f, (-(v - cy) * d) / f, -d];
}

// カメラの姿勢(R: 行優先 9 要素, t)で回して足し、世界座標へ
export function cameraToWorld(p, { R, t }) {
  return [0, 1, 2].map((i) => t[i] + R[3 * i] * p[0] + R[3 * i + 1] * p[1] + R[3 * i + 2] * p[2]);
}

export const pixelToWorld = (u, v, d, intr, extr) => cameraToWorld(pixelToCamera(u, v, d, intr), extr);

// 画像から色ごとにブロックを見つける。
//   depth: 画素ごとの深度(カメラの前方向の距離 [m])の配列。または depthAt(u, v): 1 点だけ深度を返す関数
//   extrinsics: { R, t } 「こうだと信じているカメラの位置・向き」
//   inBox(world): true を返した位置は捨てる(箱の位置は分かっている)
//   minArea: これより小さいかたまりは雑音として捨てる [画素]。既定は 352x352 で 40 画素だったものを画像の大きさに合わせる
export function detectBlocks(image, { depth = null, depthAt = null, extrinsics, fovy = 42, inBox = () => false, minArea = null } = {}) {
  const { width: w, height: h, data } = image;
  const intr = intrinsics(w, h, fovy);
  const minPix = minArea ?? Math.max(8, Math.round(40 * (w * h) / (352 * 352)));
  const hsv = new Float32Array(3 * w * h);
  for (let i = 0; i < w * h; i++) hsv.set(rgbToHsv(data[4 * i], data[4 * i + 1], data[4 * i + 2]), 3 * i);
  const found = [], labels = new Uint8Array(w * h); // labels: 0 = なし、k + 1 = COLORS[k] と判定された画素
  COLORS.forEach(({ name, hue }, k) => {
    let mask = new Uint8Array(w * h);
    for (let i = 0; i < w * h; i++) {
      mask[i] = hsv[3 * i + 1] >= SAT_MIN && hsv[3 * i + 2] >= VAL_MIN && inRanges(hsv[3 * i], hue) ? 1 : 0;
    }
    mask = morphOpen(mask, w, h);
    for (const pix of components(mask, w, h)) {
      if (pix.length < minPix) continue;
      let region = pix, d = null;
      if (depth) {
        d = median(pix.map((i) => depth[i])); // 上の面までの距離
        // 深度が分かるなら、上の面だけで長方形を当てはめる(横の面が少し写り込むと、中心が視界の中心側へずれるので)
        const top = pix.filter((i) => depth[i] < d + 0.01);
        if (top.length >= minPix / 2) region = top;
      }
      const sq = squareFit(region.map((i) => [i % w, (i / w) | 0]));
      const [u, v] = sq.center;
      if (d === null) d = depthAt(u + 0.5, v + 0.5);
      const top = pixelToWorld(u + 0.5, v + 0.5, d, intr, extrinsics);
      if (inBox(top)) continue; // 箱の中のブロックや、照明で黄色っぽく写る箱のふちは対象外
      for (const i of pix) labels[i] = k + 1;
      // 向き: 正方形の傾き。画像の下向きは世界の -y なので符号を反転
      const yaw = wrapYaw(-sq.angle);
      found.push({
        name, yaw, depth: d, area: pix.length,
        pos: [top[0], top[1], top[2] - BLOCK], // 上の面から半辺ぶん下がブロックの中心
        u: u + 0.5, v: v + 0.5,
        corners: sq.corners.map(([x, y]) => [x + 0.5, y + 0.5]),
      });
    }
  });
  return { found, labels, intr };
}

// キャリブレーション用の印(マゼンタの球)を探し、カメラから見た球の中心 [m] を返す。見つからなければ null。
// 見えているのは球の表面なので、視線の方向へ半径のぶん奥にずらして中心にする
export function findMarker(image, { depth, fovy = 42, radius = 0.012, minPixels = null } = {}) {
  const { width: w, height: h, data } = image;
  const intr = intrinsics(w, h, fovy);
  const need = minPixels ?? Math.max(5, Math.round(15 * (w * h) / (352 * 352)));
  let n = 0, su = 0, sv = 0;
  const ds = [];
  for (let i = 0; i < w * h; i++) {
    const [hh, s, v] = rgbToHsv(data[4 * i], data[4 * i + 1], data[4 * i + 2]);
    if (hh >= MARKER_HUE[0] && hh <= MARKER_HUE[1] && s >= 80 && v >= 80) {
      n++; su += i % w; sv += (i / w) | 0; ds.push(depth[i]);
    }
  }
  if (n < need) return null;
  const surface = pixelToCamera(su / n + 0.5, sv / n + 0.5, median(ds), intr);
  const len = Math.hypot(...surface);
  return { center: surface.map((x) => x + (radius * x) / len), pixels: n, u: su / n + 0.5, v: sv / n + 0.5 };
}

// 画像が bottom-up(WebGL の読み出し)なら上下を反転して、上から下の順にそろえる
export function topDown(img) {
  if (!img.flipY) return img;
  const { width: w, height: h, data } = img, out = new Uint8ClampedArray(data.length), row = 4 * w;
  for (let y = 0; y < h; y++) out.set(data.subarray((h - 1 - y) * row, (h - y) * row), y * row);
  return { width: w, height: h, data: out };
}
