// 天井カメラの画像を画面(canvas / three.js)に出す部品。ブラウザ専用(panda_vision_pick と panda_calibration の画面で共有)。
import { COLORS, topDown } from "./vision.js";

const css = (name, fallback) => getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback;
const rgb255 = (rgba) => `rgb(${rgba.slice(0, 3).map((x) => Math.round(x * 255)).join(",")})`;

// 黒でふちどりした文字(どんな背景でも読める)
function outlined(ctx, text, x, y, size = 12) {
  ctx.font = `${size}px ui-monospace, Menlo, monospace`;
  ctx.lineWidth = 3; ctx.strokeStyle = "rgba(0,0,0,0.85)"; ctx.strokeText(text, x, y);
  ctx.fillStyle = "#fff"; ctx.fillText(text, x, y);
}

// 画像を canvas に描く。found があれば、見つけた枠・中心・名前・一覧を重ねる
export function drawShot(canvas, image, { found = [], title = "" } = {}) {
  const { width: w, height: h } = image;
  canvas.width = w; canvas.height = h;
  const ctx = canvas.getContext("2d");
  ctx.putImageData(new ImageData(new Uint8ClampedArray(image.data), w, h), 0, 0);
  ctx.textBaseline = "alphabetic"; ctx.textAlign = "left";
  for (const f of found) {
    ctx.strokeStyle = "#fff"; ctx.lineWidth = 1.5;
    ctx.beginPath();
    f.corners.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
    ctx.closePath(); ctx.stroke();
    ctx.fillStyle = "#fff";
    ctx.beginPath(); ctx.arc(f.u, f.v, 3, 0, 7); ctx.fill();
    outlined(ctx, f.name, f.u + 9, f.v + 4, 12);
  }
  if (title) outlined(ctx, title, 8, 18, 13);
  found.forEach((f, i) => {
    const [x, y] = f.pos.map((p) => p * 100);
    outlined(ctx, `${f.name.padEnd(6)} (${x.toFixed(1).padStart(4)}, ${y.toFixed(1).padStart(5)}) cm ${((f.yaw * 180) / Math.PI).toFixed(0).padStart(3)} deg`,
      8, h - 10 - 15 * (found.length - 1 - i), 11);
  });
}

// 「どの画素を、どの色のブロックの一部と判定したか」を、色つきで描く(判定されなかった画素は暗い)
export function drawMask(canvas, labels, w, h) {
  canvas.width = w; canvas.height = h;
  const ctx = canvas.getContext("2d"), img = ctx.createImageData(w, h);
  const bg = matchMedia("(prefers-color-scheme: dark)").matches ? 30 : 222;
  const cols = COLORS.map((c) => c.rgba.slice(0, 3).map((x) => x * 255));
  for (let i = 0; i < w * h; i++) {
    const c = labels[i] ? cols[labels[i] - 1] : [bg, bg, bg];
    img.data[4 * i] = c[0]; img.data[4 * i + 1] = c[1]; img.data[4 * i + 2] = c[2]; img.data[4 * i + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
}

// 何も撮っていないとき
export function drawEmpty(canvas, text, size = 352) {
  canvas.width = size; canvas.height = size;
  const ctx = canvas.getContext("2d");
  ctx.fillStyle = css("--line", "#ddd"); ctx.fillRect(0, 0, size, size);
  ctx.fillStyle = css("--muted", "#777"); ctx.font = "14px sans-serif"; ctx.textAlign = "center";
  ctx.fillText(text, size / 2, size / 2);
}

// 3D 画面にカメラの視野(カメラの位置と、床に映る範囲の四角)を線で足す。返り値の配列は撮影のときに隠すために使う
export function addFrustum(viewer, cam, { fovy, color = 0x1f6fb2, floorZ = 0.001 } = {}) {
  const { R, t } = cam, a = Math.tan((fovy * Math.PI) / 360), d = t[2] - floorZ; // 真下を向くので、床までの奥行きはほぼ高さ
  const world = ([x, y, z]) => [0, 1, 2].map((i) => t[i] + R[3 * i] * x + R[3 * i + 1] * y + R[3 * i + 2] * z);
  const corners = [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([sx, sy]) => world([sx * a * d, sy * a * d, -d]));
  const objs = [viewer.sphere(t, 0.03, color)];
  for (const c of corners) objs.push(viewer.line([t, c], color));
  objs.push(viewer.line([...corners, corners[0]], color));
  return objs;
}

// 3D 画面(three.js)を、カメラ cam = { R, t }(MuJoCo と同じ向き: 自分の -z を見て、右が +x、上が +y)から描いた画像を返す。
// WebGL の画像は下から上の順なので、上から下の順に直す。色は表示と同じ sRGB で受け取る。
export function captureCamera(viewer, cam, { size, fovy, hide = [] }) {
  const { THREE, renderer, scene } = viewer, { R, t } = cam;
  const camera = new THREE.PerspectiveCamera(fovy, 1, 0.02, 40);
  new THREE.Matrix4().set(R[0], R[1], R[2], t[0], R[3], R[4], R[5], t[1], R[6], R[7], R[8], t[2], 0, 0, 0, 1)
    .decompose(camera.position, camera.quaternion, camera.scale);
  camera.updateMatrixWorld(true);
  const target = new THREE.WebGLRenderTarget(size, size);
  target.texture.colorSpace = THREE.SRGBColorSpace;
  const old = hide.map((o) => o.visible);
  hide.forEach((o) => { o.visible = false; });
  renderer.setRenderTarget(target);
  renderer.render(scene, camera);
  renderer.setRenderTarget(null);
  hide.forEach((o, i) => { o.visible = old[i]; });
  const buf = new Uint8Array(size * size * 4);
  renderer.readRenderTargetPixels(target, 0, 0, size, size, buf);
  target.dispose();
  return topDown({ width: size, height: size, data: buf, flipY: true });
}

export { rgb255 };
