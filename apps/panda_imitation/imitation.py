"""お手本から学ぶ(模倣学習 / Behavior Cloning)。

これまでは「真上へ行く → 降りる → つかむ → …」という手順を人がプログラムに書いていた。
ここではその手順書を「先生」として使い、お手本をたくさん集めて、ニューラルネット(PyTorch)に真似させる。
学習したネットは手順書を持たず、「いまの状態 → 次にどう動くか」だけを答える。

1. お手本を集める: ブロックをランダムな位置・向きに置き、先生(手順書)に片付けさせて、
   毎ステップの「状態」と「先生の指令」を記録する。先生の動きにはわざと小さな揺れを混ぜる(後述)
2. 学習する: 状態 → 指令 を当てる MLP を、二乗誤差を小さくするように学習する(教師あり学習)
3. 試す: 学習に使っていない置き方で、ネットだけに片付けさせ、成功率を測る

比べるもの:
- お手本の数(10 / 50 / 200 回)で成功率がどう変わるか
- お手本に揺れを混ぜない場合(200 回)。先生はいつも完璧な道を通るので「道から外れたときの直し方」を
  一度も見ておらず、ネットが少しずれると戻れなくなる(covariate shift)。揺れを混ぜるのは DART という方法

実行:
    python apps/panda_imitation/imitation.py           # 全部(約 7 分)。output/ に成功率の図と動画
    python apps/panda_imitation/imitation.py --quick   # 小さめの設定で動作確認(約 1.5 分)
"""
import argparse
import os
import sys
import time
from pathlib import Path

if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
import torch
from torch import nn

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "panda_pick_place"))
import pick_place as base  # noqa: E402

OUTPUT_DIR = Path(__file__).parent / "output"
PICK_AREA = ((0.38, 0.66), (-0.12, 0.32))
EPISODE_SEC = 10.0
CTRL_DT = 0.02  # 制御周期(物理 10 ステップ)
NOISE_STD = 0.006  # お手本に混ぜる揺れの大きさ [m]


# ---------- 環境 ----------
class Env:
    def __init__(self):
        self.model = base.build_spec([("block", (0.5, 0.0), (0.95, 0.80, 0.15, 1))]).compile()
        self.data = mujoco.MjData(self.model)
        self.qadr = self.model.joint(self.model.body("block_block").jntadr[0]).qposadr[0]

    def reset(self, x, y, yaw):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qadr:self.qadr + 7] = [x, y, base.BLOCK, np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        self.robot = base.Robot(self.model, self.data)
        self.down = self.robot.down.as_matrix()
        return self.obs()

    def block(self):
        b = self.data.body("block_block")
        yaw = 2 * np.arctan2(b.xquat[3], b.xquat[0])
        return b.xpos.copy(), (yaw + np.pi / 4) % (np.pi / 2) - np.pi / 4  # 立方体なので 90° ごとに同じ

    def hand_yaw(self):
        R = self.data.site("tcp").xmat.reshape(3, 3) @ self.down.T
        return np.arctan2(R[1, 0], R[0, 0])

    def obs(self):
        """ネットに見せる状態(12 個の数): 手先の位置・指の開き・ブロックの位置と向き・手首の向き。
        どれもロボットが自分で測れるか、カメラで分かるもの。"""
        tcp = self.data.site("tcp").xpos.copy()
        fingers = self.data.qpos[7] + self.data.qpos[8]
        bpos, byaw = self.block()
        return np.concatenate([tcp, [fingers], bpos, bpos - tcp, [byaw, self.hand_yaw()]]).astype(np.float32)

    def act(self, target, yaw, close):
        self.robot.yaw = yaw
        self.robot.grip = base.GRIP_CLOSED if close else base.GRIP_OPEN
        self.robot.step(target)

    def success(self):
        return base.in_box(self.data.body("block_block").xpos)


# ---------- 先生(手順書) ----------
def teacher_plan(block_xy, block_yaw, start):
    """片付けの手順を、制御周期ごとの (手先の目標, 手首の向き, 指を閉じるか) の列にする。panda_pick_place と同じ手順。

    指の開け閉めは、ここでは決めずに collect() で「手が実際にその高さまで来たか」で決める(teacher_grip)。
    """
    x, y = block_xy
    bx, by = base.BOX_CENTER
    segs = [((x, y, base.SAFE_Z), block_yaw, 0, 1.6), ((x, y, base.BLOCK + 0.002), block_yaw, 0, 1.0),
            ((x, y, base.BLOCK + 0.002), block_yaw, 1, 0.6), ((x, y, base.SAFE_Z), block_yaw, 1, 1.0),
            ((bx, by, base.SAFE_Z), 0.0, 1, 1.6), ((bx, by, base.BOX_HEIGHT + 0.06), 0.0, 1, 0.8),
            ((bx, by, base.BOX_HEIGHT + 0.06), 0.0, 0, 0.5), ((bx, by, base.SAFE_Z), 0.0, 0, 0.8)]
    plan, p0, y0 = [], np.asarray(start), 0.0
    for i, (goal, yaw, close, dur) in enumerate(segs):
        n = int(round(dur / CTRL_DT))
        for k in range(1, n + 1):
            u = base.smooth(k / n)
            plan.append((p0 + (np.asarray(goal) - p0) * u, y0 + (yaw - y0) * u, i))
        p0, y0 = np.asarray(goal), yaw
    return plan


GRASP_Z, DROP_Z = base.BLOCK + 0.002, base.BOX_HEIGHT + 0.06


def teacher_grip(seg, tcp_z, closing):
    """先生の指の指令。手が実際につかむ高さ・はなす高さまで下りたら切り替える(一度切り替えたら戻さない)。

    計画の時刻で切り替えると、切り替える直前と直後で状態がほとんど同じなのに指令だけが違う。
    真似する側はその平均(たぶん開く)を出し続けて止まってしまう。状態で決めればこの食い違いが起きない。
    """
    if seg in (0, 7):
        return False
    if seg in (1, 2):
        return closing or tcp_z < GRASP_Z + 0.006
    if seg in (3, 4):
        return True
    return closing and tcp_z > DROP_Z + 0.01  # 5, 6: 箱へ降りきったら開く


def random_placement(rng):
    return rng.uniform(*PICK_AREA[0]), rng.uniform(*PICK_AREA[1]), rng.uniform(-np.pi / 4, np.pi / 4)


def collect(env, placements, noisy, rng):
    """先生に片付けさせて、(状態, 指令) の組を集める。
    指令は「手先の目標 − いまの手先」「手首の目標 − いまの手首の向き」「閉じるか」。どれも「いまからどれだけ動くか」の形にする。"""
    X, Y, ok = [], [], 0
    for x, y, yaw in placements:
        obs = env.reset(x, y, yaw)
        plan = teacher_plan((x, y), yaw, env.data.site("tcp").xpos.copy())
        drift, close = np.zeros(3), False
        for target, yaw_cmd, seg in plan:
            tcp = env.data.site("tcp").xpos
            close = teacher_grip(seg, tcp[2], close)
            X.append(obs)
            Y.append(np.concatenate([target - tcp, [yaw_cmd - env.hand_yaw(), close]]).astype(np.float32))
            if noisy:  # ゆっくり変わる揺れ(先生が少し道を外れる)。記録するのは外れていない先生の指令
                drift = 0.9 * drift + rng.normal(0, NOISE_STD * np.sqrt(1 - 0.9 ** 2), 3)
                if target[2] < 0.12:  # つかむ・はなす瞬間の近くでは揺らさない(先生が失敗しないように)
                    drift *= 0.0
            env.act(target + drift, yaw_cmd, close)
            obs = env.obs()
        ok += env.success()
    return np.array(X), np.array(Y), ok


# ---------- 生徒(ニューラルネット) ----------
class Policy(nn.Module):
    def __init__(self, X, Y):
        super().__init__()
        self.register_buffer("xm", torch.tensor(X.mean(0)))
        self.register_buffer("xs", torch.tensor(X.std(0) + 1e-6))
        self.register_buffer("ym", torch.tensor(Y.mean(0)))
        self.register_buffer("ys", torch.tensor(Y.std(0) + 1e-6))
        self.net = nn.Sequential(nn.Linear(X.shape[1], 256), nn.ReLU(), nn.Linear(256, 256), nn.ReLU(),
                                 nn.Linear(256, 256), nn.ReLU(), nn.Linear(256, Y.shape[1]))

    def forward(self, x):  # 正規化した空間で予測する
        return self.net((x - self.xm) / self.xs)

    def act(self, obs):
        with torch.no_grad():
            y = self.forward(torch.tensor(obs)[None])[0] * self.ys + self.ym
        return y.numpy()


def train(X, Y, epochs, seed=0):
    torch.manual_seed(seed)
    pol = Policy(X, Y)
    opt = torch.optim.Adam(pol.parameters(), lr=1e-3)
    Xt, Yt = torch.tensor(X), (torch.tensor(Y) - pol.ym) / pol.ys
    n = len(X)
    steps = max(1, epochs * n // 512)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, steps)
    for _ in range(steps):
        idx = torch.randint(0, n, (512,))
        loss = ((pol(Xt[idx]) - Yt[idx]) ** 2).mean()
        opt.zero_grad()
        loss.backward()
        opt.step()
        sched.step()
    return pol, loss.item()


def rollout(env, pol, placement, record=None):
    """ネットだけで片付けさせる。手順書は使わない。"""
    obs = env.reset(*placement)
    for _ in range(int(EPISODE_SEC / CTRL_DT)):
        a = pol.act(obs)
        tcp = env.data.site("tcp").xpos.copy()
        target = tcp + a[:3]
        target[2] = max(target[2], base.BLOCK)  # 床より下は目指さない(安全のための制限)
        env.act(target, env.hand_yaw() + float(a[3]), a[4] > 0.5)
        if record:
            record(env.data)
        obs = env.obs()
    return env.success()


def evaluate(env, pol, placements):
    return np.mean([rollout(env, pol, p) for p in placements])


def plot(results, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = list(results)
    vals = [results[k] * 100 for k in labels]
    colors = ["#9ecae1", "#4292c6", "#08519c", "#d9542f"][:len(labels)]
    fig, ax = plt.subplots(figsize=(6.4, 3.8))
    bars = ax.bar(labels, vals, color=colors)
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width() / 2, v + 2, f"{v:.0f}%", ha="center", fontsize=11)
    ax.set_ylim(0, 110)
    ax.set_ylabel("success rate on new placements [%]")
    ax.set_title("Imitation learning: learned policy, no script", fontsize=11)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main(quick, record_video):
    rng = np.random.default_rng(0)
    env = Env()
    n_max = 60 if quick else 200
    sizes = [10, 30, 60] if quick else [10, 50, 200]
    n_eval = 10 if quick else 50
    epochs = 40
    train_places = [random_placement(rng) for _ in range(n_max)]
    eval_places = [random_placement(np.random.default_rng(1000 + i)) for i in range(n_eval)]  # 学習に使わない置き方

    t0 = time.time()
    Xn, Yn, okn = collect(env, train_places, noisy=True, rng=rng)
    per_ep = len(Xn) // n_max
    print(f"1. お手本(揺れあり) {n_max} 回を集めた: {len(Xn)} 組の(状態, 指令)、先生の成功 {okn}/{n_max} [{time.time() - t0:.0f} 秒]")
    t0 = time.time()
    Xc, Yc, okc = collect(env, train_places, noisy=False, rng=rng)
    print(f"   お手本(揺れなし) {n_max} 回を集めた: 先生の成功 {okc}/{n_max} [{time.time() - t0:.0f} 秒]")

    results, best = {}, None
    for n in sizes:
        t0 = time.time()
        pol, loss = train(Xn[:n * per_ep], Yn[:n * per_ep], epochs)
        rate = evaluate(env, pol, eval_places)
        results[f"{n} demos"] = rate
        print(f"2・3. 揺れあり {n:3d} 回で学習(誤差 {loss:.3f}) → 新しい置き方 {n_eval} 回で成功率 {rate * 100:.0f}% [{time.time() - t0:.0f} 秒]")
        best = pol
    pol_c, loss = train(Xc, Yc, epochs)
    rate = evaluate(env, pol_c, eval_places)
    results[f"{n_max} demos\nno noise"] = rate
    print(f"      揺れなし {n_max} 回で学習(誤差 {loss:.3f}) → 成功率 {rate * 100:.0f}%")

    OUTPUT_DIR.mkdir(exist_ok=True)
    plot(results, OUTPUT_DIR / "success_rate.png")
    print(f"図を保存: {OUTPUT_DIR / 'success_rate.png'}")

    if record_video:
        scene = mujoco.Renderer(env.model, height=352, width=480)
        front = env.model.camera("front").id
        frames, k = [], [0]

        def record(d):
            k[0] += 1
            if k[0] % 3 == 0:
                scene.update_scene(d, camera=front)
                img = scene.render().copy()
                import cv2
                for color, th in (((0, 0, 0), 3), ((255, 255, 255), 1)):
                    cv2.putText(img, "learned policy (no script)", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, th, cv2.LINE_AA)
                frames.append(img)

        demo_rng = np.random.default_rng(7)
        for _ in range(3):
            rollout(env, best, random_placement(demo_rng), record)
        scene.close()
        import subprocess

        import imageio.v2 as imageio
        import imageio_ffmpeg
        mp4 = OUTPUT_DIR / "learned_policy.mp4"
        imageio.mimsave(mp4, frames, fps=int(round(1 / (3 * CTRL_DT))))
        subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-loglevel", "error", "-y", "-i", str(mp4), "-vf",
                        "fps=8,scale=400:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=48[p];[b][p]paletteuse=dither=bayer",
                        str(OUTPUT_DIR / "learned_policy.gif")], check=True)
        print(f"動画を保存: {mp4}, {OUTPUT_DIR / 'learned_policy.gif'}")
    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--quick", action="store_true", help="お手本・評価の回数を減らして速く回す")
    parser.add_argument("--no-video", action="store_true")
    args = parser.parse_args()
    torch.set_num_threads(max(1, os.cpu_count() // 2))
    main(args.quick, record_video=not args.no_video)
