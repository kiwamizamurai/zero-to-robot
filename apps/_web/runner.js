// ジェネレーターで書いた作業(1 回の yield = 制御 1 ステップ)を、再生・一時停止・速度つきで進める。
//
//   const runner = new Runner({ stepsPerSec: 50, onFrame: () => {...}, onDone: () => {...} });
//   runner.start(sim.run());  runner.toggle();  runner.reset();
export class Runner {
  constructor({ stepsPerSec = 50, getSpeed = () => 1, onFrame = () => {}, onDone = () => {}, onState = () => {}, budgetMs = 12 } = {}) {
    Object.assign(this, { stepsPerSec, getSpeed, onFrame, onDone, onState, budgetMs });
    this.task = null; this.running = false; this.finished = false; this.budget = 0; this.last = 0;
  }
  start(task) { this.task = task; this.running = true; this.finished = false; this.budget = 0; this.onState(this); }
  toggle() { if (this.task && !this.finished) { this.running = !this.running; this.onState(this); } }
  stop() { this.task = null; this.running = false; this.finished = false; this.budget = 0; this.onState(this); }
  // requestAnimationFrame のコールバックから毎フレーム呼ぶ。進めた制御ステップ数を返す
  frame(now) {
    const dt = Math.min((now - (this.last || now)) / 1000, 0.05);
    this.last = now;
    let n = 0;
    if (this.running) {
      this.budget += dt * this.stepsPerSec * this.getSpeed();
      const t0 = performance.now();
      while (this.budget >= 1 && performance.now() - t0 < this.budgetMs) {
        this.budget -= 1;
        n++;
        if (this.task.next().done) {
          this.running = false; this.finished = true;
          this.onState(this); this.onDone();
          break;
        }
      }
      this.budget = Math.min(this.budget, 5); // 重い端末では遅れを取り戻さず、スローで進める
      this.onFrame(n);
    }
    return n;
  }
  loop(draw) {
    const tick = (now) => { requestAnimationFrame(tick); this.frame(now); draw(); };
    requestAnimationFrame((t) => { this.last = t; tick(t); });
  }
}

export const $ = (id) => document.getElementById(id);
