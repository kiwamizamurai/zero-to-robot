// 自前の MLP(ReLU)+ Adam。PyTorch の代わり。imitation.py の train() と同じ流れ:
// 入力・出力を平均 0・分散 1 に正規化して、二乗誤差を、コサインで学習率を下げながら Adam で最小化する。
import { mulberry32, gauss } from "../../_web/math.js";

export function trainMLP(X, Y, { hidden = 128, epochs = 40, batch = 256, lr = 1e-3, seed = 0 } = {}) {
  const n = Y.length / 5, din = X.length / n, dout = 5;
  const stats = (A, d) => {
    const m = new Float64Array(d), s = new Float64Array(d);
    for (let i = 0; i < n; i++) for (let j = 0; j < d; j++) m[j] += A[i * d + j] / n;
    for (let i = 0; i < n; i++) for (let j = 0; j < d; j++) s[j] += (A[i * d + j] - m[j]) ** 2 / n;
    return [Float32Array.from(m), Float32Array.from(s, (v) => Math.sqrt(v) + 1e-6)];
  };
  const [xm, xs] = stats(X, din), [ym, ys] = stats(Y, dout);
  const Xn = new Float32Array(X.length), Yn = new Float32Array(Y.length);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < din; j++) Xn[i * din + j] = (X[i * din + j] - xm[j]) / xs[j];
    for (let j = 0; j < dout; j++) Yn[i * dout + j] = (Y[i * dout + j] - ym[j]) / ys[j];
  }

  const sizes = [din, hidden, hidden, hidden, dout], L = sizes.length - 1;
  const rnd = mulberry32(seed * 7919 + 1);
  const W = [], B = [], gW = [], gB = [], mW = [], vW = [], mB = [], vB = [];
  for (let l = 0; l < L; l++) {
    const nin = sizes[l], nout = sizes[l + 1], bound = 1 / Math.sqrt(nin); // PyTorch の nn.Linear と同じ初期化
    W.push(Float32Array.from({ length: nin * nout }, () => (rnd() * 2 - 1) * bound));
    B.push(Float32Array.from({ length: nout }, () => (rnd() * 2 - 1) * bound));
    gW.push(new Float32Array(nin * nout)); gB.push(new Float32Array(nout));
    mW.push(new Float32Array(nin * nout)); vW.push(new Float32Array(nin * nout));
    mB.push(new Float32Array(nout)); vB.push(new Float32Array(nout));
  }
  const act = sizes.map((s) => new Float32Array(batch * s)); // 各層の出力(ReLU 後)
  const grad = sizes.map((s) => new Float32Array(batch * s));
  const steps = Math.max(1, Math.floor((epochs * n) / 512)); // 1 エポック = 512 組を n/512 回、と同じ総量
  const b1 = 0.9, b2 = 0.999, eps = 1e-8;
  let loss = 0;

  for (let t = 1; t <= steps; t++) {
    // ミニバッチ(重複ありの無作為抽出)
    for (let b = 0; b < batch; b++) {
      const i = Math.floor(rnd() * n);
      act[0].set(Xn.subarray(i * din, (i + 1) * din), b * din);
      grad[L].set(Yn.subarray(i * dout, (i + 1) * dout), b * dout); // 一時的に目標を入れておく
    }
    for (let l = 0; l < L; l++) {
      const nin = sizes[l], nout = sizes[l + 1], x = act[l], y = act[l + 1], w = W[l], bias = B[l];
      for (let b = 0; b < batch; b++) {
        const xo = b * nin, yo = b * nout;
        for (let o = 0; o < nout; o++) {
          let s = bias[o];
          const r = o * nin;
          for (let i = 0; i < nin; i++) s += w[r + i] * x[xo + i];
          y[yo + o] = l === L - 1 || s > 0 ? s : 0;
        }
      }
    }
    // 損失と出力の勾配: mean((y - t)^2) を(バッチ × 出力)で平均
    const gl = grad[L], yl = act[L];
    loss = 0;
    const scale = 2 / (batch * dout);
    for (let k = 0; k < batch * dout; k++) {
      const d = yl[k] - gl[k];
      loss += d * d;
      gl[k] = d * scale;
    }
    loss /= batch * dout;
    for (let l = L - 1; l >= 0; l--) {
      const nin = sizes[l], nout = sizes[l + 1], x = act[l], y = act[l + 1], dy = grad[l + 1], dx = grad[l], w = W[l];
      gW[l].fill(0); gB[l].fill(0); dx.fill(0);
      const gw = gW[l], gb = gB[l];
      for (let b = 0; b < batch; b++) {
        const xo = b * nin, yo = b * nout;
        for (let o = 0; o < nout; o++) {
          if (l < L - 1 && y[yo + o] <= 0) continue; // ReLU が 0 のところは勾配も 0
          const g = dy[yo + o];
          gb[o] += g;
          const r = o * nin;
          for (let i = 0; i < nin; i++) {
            gw[r + i] += g * x[xo + i];
            dx[xo + i] += g * w[r + i];
          }
        }
      }
    }
    // Adam(学習率はコサインで 0 へ)
    const cur = (lr * (1 + Math.cos((Math.PI * (t - 1)) / steps))) / 2;
    const c1 = 1 - b1 ** t, c2 = 1 - b2 ** t;
    for (let l = 0; l < L; l++) {
      for (const [P, G, M, V] of [[W[l], gW[l], mW[l], vW[l]], [B[l], gB[l], mB[l], vB[l]]]) {
        for (let k = 0; k < P.length; k++) {
          const g = G[k];
          M[k] = b1 * M[k] + (1 - b1) * g;
          V[k] = b2 * V[k] + (1 - b2) * g * g;
          P[k] -= (cur * (M[k] / c1)) / (Math.sqrt(V[k] / c2) + eps);
        }
      }
    }
  }
  return { sizes, xm, xs, ym, ys, W, B, loss, steps };
}

// Int16 + 倍率で base64 にする(policy.js の decode と対)
function enc(arr) {
  let mx = 0;
  for (const v of arr) mx = Math.max(mx, Math.abs(v));
  const s = mx / 32767 || 1, q = new Int16Array(arr.length);
  for (let i = 0; i < arr.length; i++) q[i] = Math.round(arr[i] / s);
  return { q: Buffer.from(q.buffer).toString("base64"), s };
}

export function toJSON(net, meta = {}) {
  const r = (a) => Array.from(a, (v) => +v.toPrecision(6));
  return {
    meta, sizes: net.sizes, xm: r(net.xm), xs: r(net.xs), ym: r(net.ym), ys: r(net.ys),
    layers: net.W.map((w, l) => ({ w: enc(w), b: enc(net.B[l]) })),
  };
}
export { gauss };
