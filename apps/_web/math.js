// 各アプリで共通の小さな数学ユーティリティ(DOM 非依存)。

// 5 次多項式の時間スケーリング: ゆっくり動き出してゆっくり止まる(05_planning と同じ)
export function smooth(t) {
  t = Math.min(Math.max(t, 0), 1);
  return t ** 3 * (10 - 15 * t + 6 * t * t);
}

// 乱数の種を固定できる乱数生成器(同じ seed なら同じ列)
export function mulberry32(a) {
  return () => {
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// 標準正規分布の乱数(Box-Muller)。rnd は mulberry32 の返す関数
export function gauss(rnd) {
  const u = Math.max(rnd(), 1e-12), v = rnd();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

// n x n の連立一次方程式 A x = b(部分ピボット付きのガウス消去法)
export function solve(A, b, n) {
  const M = A.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = c + 1; r < n; r++) {
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let s = M[r][n];
    for (let k = r + 1; k < n; k++) s -= M[r][k] * x[k];
    x[r] = s / M[r][r];
  }
  return x;
}

// 3x3 行列(行優先 9 要素)の積 A * B
export function mul3(A, B) {
  const C = new Array(9).fill(0);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) for (let k = 0; k < 3; k++) C[i * 3 + j] += A[i * 3 + k] * B[k * 3 + j];
  return C;
}

// z 軸まわりに yaw [rad] 回す回転行列
export function rotZ(yaw) {
  const c = Math.cos(yaw), s = Math.sin(yaw);
  return [c, -s, 0, s, c, 0, 0, 0, 1];
}

// 回転行列の差 R_target * R_current^T を回転ベクトルにする(姿勢誤差)
export function rotationError(Rt, Rc) {
  const E = new Array(9).fill(0);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) for (let k = 0; k < 3; k++) E[i * 3 + j] += Rt[i * 3 + k] * Rc[j * 3 + k];
  const v = [E[7] - E[5], E[2] - E[6], E[3] - E[1]];
  const s = Math.hypot(...v) / 2;
  const c = (E[0] + E[4] + E[8] - 1) / 2;
  const angle = Math.atan2(s, c);
  const k = s < 1e-9 ? 0.5 : angle / (2 * s);
  return v.map((x) => x * k);
}

export const dist = (a, b) => Math.hypot(...a.map((x, i) => x - b[i]));
