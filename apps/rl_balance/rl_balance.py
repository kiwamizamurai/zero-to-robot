"""強化学習で倒立振子を立たせる(Gymnasium × MuJoCo × Stable-Baselines3 の PPO)。

balance_bot では、運動方程式から LQR で「どう動けば倒れないか」を計算した。模倣学習(panda_imitation)では、先生のお手本を真似した。
ここでは、どちらもない。ロボットが知っているのは「倒れずにいられたら 1 点」という**ごほうび(報酬)**だけ。
でたらめに動いて、倒れて、また始めて…をくりかえすうちに、点が多くもらえる動き方を自分で見つける。

1. 学習前: でたらめに動かすと、すぐ倒れる
2. 学習: PPO で 4 万ステップ(約 27 分ぶんの経験)試行錯誤させ、1 回ごとに何ステップ立っていられたかを記録する
3. 学習後: 1000 ステップ(40 秒)立ち続けられるか、途中で棒を押しても立ち直るかを試す

使うもの:
- Gymnasium の InvertedPendulum-v5(MuJoCo で動く台車つき倒立振子。棒が 0.2 rad ≒ 11° 以上傾いたら終わり)
- Stable-Baselines3 の PPO(いま最もよく使われる強化学習の方法の 1 つ)

実行:
    python apps/rl_balance/rl_balance.py            # 学習(数分)→ 図と動画を output/ に書き出す
    python apps/rl_balance/rl_balance.py --steps 10000   # 学習を短くすると、どこまで覚えられるか
"""
import argparse
import os
import sys
from pathlib import Path

if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")

import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback

OUTPUT_DIR = Path(__file__).parent / "output"
ENV_ID = "InvertedPendulum-v5"


class EpisodeLog(BaseCallback):
    """学習中に、1 回(エピソード)ごとの長さ = 倒れずにいられたステップ数を記録する。"""

    def __init__(self):
        super().__init__()
        self.lengths, self.at = [], []

    def _on_step(self):
        for info in self.locals["infos"]:
            if "episode" in info:
                self.lengths.append(info["episode"]["l"])
                self.at.append(self.num_timesteps)
        return True


def rollout(env, policy=None, seed=0, push_at=None, push_force=0.0, frames=None, label=""):
    """1 回分走らせて、立っていられたステップ数を返す。policy=None ならでたらめに動かす。"""
    obs, _ = env.reset(seed=seed)
    rng = np.random.default_rng(seed)
    data = env.unwrapped.data
    pole = env.unwrapped.model.body("pole").id
    for t in range(1000):
        if policy is None:
            action = rng.uniform(-3, 3, size=1)
        else:
            action, _ = policy.predict(obs, deterministic=True)
        pushing = push_at is not None and push_at <= t < push_at + 3
        data.xfrc_applied[pole, 0] = push_force if pushing else 0.0  # 棒を横から押す
        obs, _, terminated, truncated, _ = env.step(action)
        if frames is not None and t % 2 == 0:
            frames.append(annotate(env.render(), label, t, obs, pushing))
        if terminated or truncated:
            if frames is not None:
                for _ in range(10):  # 倒れたところで少し止める
                    frames.append(annotate(env.render(), label + "  -> fell", t, obs, False))
            return t + 1
    return 1000


def annotate(img, label, t, obs, pushing):
    import cv2

    img = np.ascontiguousarray(img)
    lines = [label, f"step {t:4d}   tilt {np.rad2deg(obs[1]):+5.1f} deg"]
    if pushing:
        lines.append("PUSH!")
    for i, s in enumerate(lines):
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):  # 黒いふちどり
            cv2.putText(img, s, (10 + dx, 24 + 22 * i + dy), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(img, s, (10, 24 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def plot_curve(log, path, random_len):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    at, ln = np.array(log.at), np.array(log.lengths)
    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    ax.scatter(at, ln, s=6, alpha=0.35, label="each episode")
    if len(ln) >= 10:
        k = 10
        ax.plot(at[k - 1:], np.convolve(ln, np.ones(k) / k, mode="valid"), lw=2, label="average of last 10")
    ax.axhline(random_len, color="gray", ls="--", lw=1, label=f"random actions ({random_len:.0f} steps)")
    ax.axhline(1000, color="green", ls=":", lw=1, label="max (1000 steps = 40 s)")
    ax.set_xlabel("experience so far [steps]  (1 step = 0.04 s)")
    ax.set_ylabel("steps before falling")
    ax.set_title("Reinforcement learning (PPO): learning to balance by trial and error", fontsize=11)
    ax.legend(fontsize=8, loc="center right")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main(steps, video):
    OUTPUT_DIR.mkdir(exist_ok=True)
    env = gym.make(ENV_ID)

    # 1. 学習前
    random_lens = [rollout(env, None, seed=s) for s in range(20)]
    print(f"1. でたらめに動かすと: 平均 {np.mean(random_lens):.1f} ステップ({np.mean(random_lens) * 0.04:.2f} 秒)で倒れる")

    # 2. 学習
    log = EpisodeLog()
    model = PPO("MlpPolicy", env, seed=0, verbose=0)
    model.learn(total_timesteps=steps, callback=log)
    first = next((i for i, l in enumerate(log.lengths) if l >= 1000), None)
    print(f"2. 学習: {len(log.lengths)} 回のエピソード(倒れるか時間切れまでを 1 回)、合計 {steps} ステップ({steps * 0.04 / 60:.0f} 分ぶん)の経験。"
          + (f" 初めて 1000 ステップ立てたのは {first + 1} 回目" if first is not None else " 1000 ステップには届かなかった"))

    # 3. 学習後
    trained = [rollout(env, model, seed=100 + s) for s in range(10)]
    pushed = [rollout(env, model, seed=200 + s, push_at=150, push_force=f) for s, f in enumerate([20, 40, 60, 80, 100])]
    print(f"3. 学習後: 10 回中 {sum(l == 1000 for l in trained)} 回、最後(1000 ステップ = 40 秒)まで倒れなかった")
    for f, l in zip([20, 40, 60, 80, 100], pushed):
        print(f"   途中で棒を {f} N で押すと: {'立ち直った' if l == 1000 else f'{l} ステップで倒れた'}")
    plot_curve(log, OUTPUT_DIR / "learning_curve.png", np.mean(random_lens))
    print(f"図を保存: {OUTPUT_DIR / 'learning_curve.png'}")

    if video:
        venv = gym.make(ENV_ID, render_mode="rgb_array", width=480, height=320)
        frames = []
        rollout(venv, None, seed=1, frames=frames, label="BEFORE learning (random)")
        rollout(venv, model, seed=2, push_at=120, push_force=20, frames=frames, label="AFTER learning (PPO)")
        frames = frames[:400]
        venv.close()
        import subprocess

        import imageio.v2 as imageio
        import imageio_ffmpeg
        mp4 = OUTPUT_DIR / "rl_balance.mp4"
        imageio.mimsave(mp4, frames, fps=12)
        subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-loglevel", "error", "-y", "-i", str(mp4), "-vf",
                        "fps=12,scale=480:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=64[p];[b][p]paletteuse=dither=bayer",
                        str(OUTPUT_DIR / "rl_balance.gif")], check=True)
        print(f"動画を保存: {mp4}")
    return np.mean(random_lens), trained, pushed


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=40000, help="学習に使うステップ数(経験の量)")
    ap.add_argument("--no-video", action="store_true")
    args = ap.parse_args()
    rnd, trained, _ = main(args.steps, not args.no_video)
    assert np.mean(trained) > 10 * rnd, "学習後も、でたらめに動かすのと大差なかった"
