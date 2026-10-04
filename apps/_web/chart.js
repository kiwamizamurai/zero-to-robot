// 依存なしの小さなグラフ描画(canvas)。色は CSS 変数(--ink, --muted, --line, --accent)から取る。
const css = (name, fallback) => getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback;
export const PALETTE = ["#1f6fb2", "#d9534f", "#2f8a4b", "#e69f00", "#8e44ad", "#16a3a3"];

function frame(canvas, { w = 640, h = 320 } = {}) {
  const dpr = Math.min(devicePixelRatio || 1, 2);
  canvas.width = w * dpr; canvas.height = h * dpr;
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  ctx.font = "12px ui-monospace, Menlo, monospace";
  return { ctx, w, h, ink: css("--ink", "#222"), muted: css("--muted", "#777"), line: css("--line", "#ddd") };
}

function axes(c, { x0, x1, y0, y1, xLabel, yLabel, pad }) {
  const { ctx, w, h, muted, line } = c;
  ctx.strokeStyle = line; ctx.fillStyle = muted; ctx.lineWidth = 1;
  const ticks = (a, b, n = 5) => Array.from({ length: n + 1 }, (_, i) => a + ((b - a) * i) / n);
  for (const y of ticks(y0, y1)) {
    const py = pad.t + (1 - (y - y0) / (y1 - y0)) * (h - pad.t - pad.b);
    ctx.beginPath(); ctx.moveTo(pad.l, py); ctx.lineTo(w - pad.r, py); ctx.stroke();
    ctx.textAlign = "right"; ctx.fillText(+y.toPrecision(3) + "", pad.l - 6, py + 4);
  }
  for (const x of ticks(x0, x1)) {
    const px = pad.l + ((x - x0) / (x1 - x0)) * (w - pad.l - pad.r);
    ctx.textAlign = "center"; ctx.fillText(+x.toPrecision(3) + "", px, h - pad.b + 16);
  }
  ctx.textAlign = "center";
  if (xLabel) ctx.fillText(xLabel, pad.l + (w - pad.l - pad.r) / 2, h - 4);
  if (yLabel) { ctx.save(); ctx.translate(12, pad.t + (h - pad.t - pad.b) / 2); ctx.rotate(-Math.PI / 2); ctx.fillText(yLabel, 0, 0); ctx.restore(); }
}

// series: [{ name, xs, ys, color?, dashed?, points? }]
export function lineChart(canvas, series, { xLabel = "", yLabel = "", yMin = null, yMax = null, xMin = null, xMax = null, size } = {}) {
  const c = frame(canvas, size);
  const pad = { l: 52, r: 12, t: 12, b: 36 };
  const all = series.flatMap((s) => s.ys.map((y, i) => [s.xs[i], y]));
  if (!all.length) return;
  const x0 = xMin ?? Math.min(...all.map((p) => p[0])), x1 = xMax ?? Math.max(...all.map((p) => p[0]));
  let y0 = yMin ?? Math.min(...all.map((p) => p[1])), y1 = yMax ?? Math.max(...all.map((p) => p[1]));
  if (y0 === y1) { y0 -= 1; y1 += 1; }
  axes(c, { x0, x1: x1 === x0 ? x0 + 1 : x1, y0, y1, xLabel, yLabel, pad });
  const px = (x) => pad.l + ((x - x0) / ((x1 === x0 ? x0 + 1 : x1) - x0)) * (c.w - pad.l - pad.r);
  const py = (y) => pad.t + (1 - (y - y0) / (y1 - y0)) * (c.h - pad.t - pad.b);
  series.forEach((s, k) => {
    const color = s.color ?? PALETTE[k % PALETTE.length];
    c.ctx.strokeStyle = color; c.ctx.fillStyle = color; c.ctx.lineWidth = 2;
    c.ctx.setLineDash(s.dashed ? [5, 4] : []);
    c.ctx.beginPath();
    s.ys.forEach((y, i) => (i ? c.ctx.lineTo(px(s.xs[i]), py(y)) : c.ctx.moveTo(px(s.xs[i]), py(y))));
    c.ctx.stroke();
    c.ctx.setLineDash([]);
    if (s.points) s.ys.forEach((y, i) => { c.ctx.beginPath(); c.ctx.arc(px(s.xs[i]), py(y), 3, 0, 7); c.ctx.fill(); });
  });
  // 凡例
  let lx = pad.l + 8;
  series.forEach((s, k) => {
    c.ctx.fillStyle = s.color ?? PALETTE[k % PALETTE.length];
    c.ctx.fillRect(lx, pad.t + 4, 14, 3);
    c.ctx.fillStyle = c.ink; c.ctx.textAlign = "left";
    c.ctx.fillText(s.name ?? "", lx + 18, pad.t + 10);
    lx += 28 + c.ctx.measureText(s.name ?? "").width;
  });
}

// bars: [{ label, value, color? }]
export function barChart(canvas, bars, { yLabel = "", yMax = null, size } = {}) {
  const c = frame(canvas, size);
  const pad = { l: 52, r: 12, t: 12, b: 36 };
  const y1 = yMax ?? Math.max(...bars.map((b) => b.value), 1e-9) * 1.1;
  axes(c, { x0: 0, x1: bars.length, y0: 0, y1, yLabel, pad });
  const bw = (c.w - pad.l - pad.r) / bars.length;
  bars.forEach((b, i) => {
    const hgt = (b.value / y1) * (c.h - pad.t - pad.b);
    c.ctx.fillStyle = b.color ?? PALETTE[i % PALETTE.length];
    c.ctx.fillRect(pad.l + i * bw + bw * 0.15, c.h - pad.b - hgt, bw * 0.7, hgt);
    c.ctx.fillStyle = c.ink; c.ctx.textAlign = "center";
    c.ctx.fillText(b.label, pad.l + (i + 0.5) * bw, c.h - pad.b + 16);
    c.ctx.fillText(+b.value.toPrecision(3) + "", pad.l + (i + 0.5) * bw, c.h - pad.b - hgt - 4);
  });
}
