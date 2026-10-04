// 点の組から回転と平行移動を最小二乗で求める Kabsch 法と、そこで使う 3x3 の特異値分解(SVD)。
// DOM にも MuJoCo にも依存しない純粋な関数。行列は行優先の 9 要素の配列。
const mulMat = (A, B) => {
  const C = new Array(9).fill(0);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) for (let k = 0; k < 3; k++) C[3 * i + j] += A[3 * i + k] * B[3 * k + j];
  return C;
};
export const transpose3 = (A) => [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];
export const det3 = (A) => A[0] * (A[4] * A[8] - A[5] * A[7]) - A[1] * (A[3] * A[8] - A[5] * A[6]) + A[2] * (A[3] * A[7] - A[4] * A[6]);
export const mulVec3 = (A, p) => [0, 1, 2].map((i) => A[3 * i] * p[0] + A[3 * i + 1] * p[1] + A[3 * i + 2] * p[2]);

// 3x3 行列 A = U diag(S) V^T(S は大きい順)。一側ヤコビ法: 列どうしが直交するまで、2 列ずつ回転させる。
// 列の長さが特異値、列の向きが U、回した累積が V になる。
export function svd3(A) {
  const W = [[A[0], A[3], A[6]], [A[1], A[4], A[7]], [A[2], A[5], A[8]]]; // W[j] = A の第 j 列
  const V = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];                              // V[j] = V の第 j 列
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  for (let sweep = 0; sweep < 60; sweep++) {
    let rotated = false;
    for (const [p, q] of [[0, 1], [0, 2], [1, 2]]) {
      const alpha = dot(W[p], W[p]), beta = dot(W[q], W[q]), gamma = dot(W[p], W[q]);
      if (Math.abs(gamma) <= 1e-15 * Math.sqrt(alpha * beta) || gamma === 0) continue;
      rotated = true;
      const zeta = (beta - alpha) / (2 * gamma);
      const t = Math.sign(zeta || 1) / (Math.abs(zeta) + Math.sqrt(1 + zeta * zeta));
      const c = 1 / Math.sqrt(1 + t * t), s = c * t;
      for (const M of [W, V]) {
        const mp = M[p], mq = M[q];
        M[p] = mp.map((x, i) => c * x - s * mq[i]);
        M[q] = mp.map((x, i) => s * x + c * mq[i]);
      }
    }
    if (!rotated) break;
  }
  const order = [0, 1, 2].sort((a, b) => dot(W[b], W[b]) - dot(W[a], W[a]));
  const S = order.map((j) => Math.sqrt(dot(W[j], W[j])));
  const Vc = order.map((j) => V[j]);
  const Uc = order.map((j, k) => (S[k] > 1e-12 * (S[0] || 1) ? W[j].map((x) => x / S[k]) : null));
  // 特異値が 0 の列(点が一直線・一平面のとき): 残りの列と直交する単位ベクトルで埋める
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const unit = (a) => { const n = Math.hypot(...a); return a.map((x) => x / n); };
  for (let k = 0; k < 3; k++) {
    if (Uc[k]) continue;
    const known = Uc.filter(Boolean);
    if (known.length === 2) Uc[k] = unit(cross(known[0], known[1]));
    else if (known.length === 1) {
      const a = known[0], e = Math.abs(a[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
      Uc[k] = unit(cross(a, e));
    } else Uc[k] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]][k];
  }
  const cols = (C) => [C[0][0], C[1][0], C[2][0], C[0][1], C[1][1], C[2][1], C[0][2], C[1][2], C[2][2]]; // 列の配列 → 行優先
  return { U: cols(Uc), S, V: cols(Vc) };
}

// P_world ≈ R · P_cam + t となる回転 R・平行移動 t を最小二乗で求める(SVD を使う Kabsch 法)。
// 点の重心をそろえてから、共分散行列 H の SVD で回転を出す。行列式の符号を直して鏡映にならないようにする。
export function kabsch(Pcam, Pworld) {
  const n = Pcam.length;
  if (n < 3) throw new Error("点が 3 つ以上必要です");
  const mean = (P) => [0, 1, 2].map((k) => P.reduce((s, p) => s + p[k], 0) / n);
  const mc = mean(Pcam), mw = mean(Pworld);
  const H = new Array(9).fill(0);
  for (let m = 0; m < n; m++) {
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) H[3 * i + j] += (Pcam[m][i] - mc[i]) * (Pworld[m][j] - mw[j]);
  }
  const { U, V, S } = svd3(H);
  const d = Math.sign(det3(mulMat(V, transpose3(U)))) || 1;
  const R = mulMat(mulMat(V, [1, 0, 0, 0, 1, 0, 0, 0, d]), transpose3(U));
  const Rm = mulVec3(R, mc);
  const t = mw.map((x, i) => x - Rm[i]);
  const residual = Pcam.map((p, m) => {
    const q = mulVec3(R, p);
    return Math.hypot(q[0] + t[0] - Pworld[m][0], q[1] + t[1] - Pworld[m][1], q[2] + t[2] - Pworld[m][2]);
  });
  return { R, t, residual, rms: Math.sqrt(residual.reduce((s, r) => s + r * r, 0) / n), singular: S };
}

// 回転行列が表す回転の大きさ [度]
export const rotAngleDeg = (R) => (Math.acos(Math.min(1, Math.max(-1, (R[0] + R[4] + R[8] - 1) / 2))) * 180) / Math.PI;
// 2 つの姿勢のあいだの回転の大きさ [度](R1^T R2)
export const rotDiffDeg = (R1, R2) => rotAngleDeg(mulMat(transpose3(R1), R2));
export { mulMat };
