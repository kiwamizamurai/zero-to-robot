// 学習済みの方策(MLP)を JS で推論するだけの小さなクラス。学習は tools/train.mjs(ブラウザでは学習しない)。
// 重みの JSON: { sizes:[12,H,H,H,5], xm, xs, ym, ys, layers:[{w, b}] }。
// 容量を抑えるため、w と b は「16 ビット整数を base64 にしたもの + 倍率」({ q, s })で入っている
function decode({ q, s }) {
  const bin = atob(q), bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  const ints = new Int16Array(bytes.buffer), out = new Float32Array(ints.length);
  for (let i = 0; i < ints.length; i++) out[i] = ints[i] * s;
  return out; // w は 出力 × 入力 の行優先
}

export class Policy {
  constructor(j) {
    this.meta = j.meta ?? {};
    this.sizes = j.sizes;
    this.xm = Float32Array.from(j.xm); this.xs = Float32Array.from(j.xs);
    this.ym = Float32Array.from(j.ym); this.ys = Float32Array.from(j.ys);
    this.layers = j.layers.map((l) => ({ w: decode(l.w), b: decode(l.b) }));
    this.bufs = this.sizes.map((n) => new Float32Array(n));
    this.out = new Float32Array(this.sizes[this.sizes.length - 1]);
  }

  // 状態 → 指令(元のスケール)。ReLU の MLP を正規化した空間で動かす
  act(obs) {
    const { sizes, layers, bufs } = this;
    for (let i = 0; i < sizes[0]; i++) bufs[0][i] = (obs[i] - this.xm[i]) / this.xs[i];
    for (let l = 0; l < layers.length; l++) {
      const { w, b } = layers[l], x = bufs[l], y = bufs[l + 1], nin = sizes[l], nout = sizes[l + 1];
      const last = l === layers.length - 1;
      for (let o = 0; o < nout; o++) {
        let s = b[o];
        const r = o * nin;
        for (let i = 0; i < nin; i++) s += w[r + i] * x[i];
        y[o] = last || s > 0 ? s : 0;
      }
    }
    const y = bufs[bufs.length - 1];
    for (let i = 0; i < this.out.length; i++) this.out[i] = y[i] * this.ys[i] + this.ym[i];
    return this.out;
  }
}
