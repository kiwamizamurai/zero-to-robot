"""SO-101 でキューブをトレイに入れる。シミュレーターでも実機でも、同じコードで動かす。

ロボットとのやりとりは get_observation() と send_action() だけ。ロボットの中身(シミュレーターか実機か)は知らない。
逆運動学には、ロボットとは別に持っている「計算用の SO-101 モデル」(Menagerie の MJCF)と mink を使う。

実行:
    python apps/so101_twin/pick_place.py                      # シミュレーター。動画とログを output/ に書き出す
    python apps/so101_twin/pick_place.py --robot real --port /dev/ttyACM0 --cube 0.24 0.10
        # 実機(LeRobot が必要)。先に同じ計画をシミュレーターで試し、成功したときだけ実機に送る。
        # 実機ではキューブの位置を測れないので、--cube で机の上の印の位置 [m] を渡す
"""
import argparse
import csv
import os
import sys
import time
from pathlib import Path

# 画面のない Linux では動画の書き出しに EGL を使う。mujoco の import より前に決める必要がある
if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")

import mink
import mujoco
import numpy as np

from sim_so101 import MOTORS, SO101_SCENE, SimSO101, SimSO101Config

OUTPUT_DIR = Path(__file__).parent / "output"
GRIP_OPEN, GRIP_CLOSED = 60.0, 3.0  # gripper.pos(0 閉 〜 100 開)
SAFE_Z, GRASP_Z, DROP_Z = 0.10, 0.012, 0.07


def smooth(t):
    t = np.clip(t, 0.0, 1.0)
    return t**3 * (10 - 15 * t + 6 * t * t)


class Planner:
    """手先(gripperframe)の目標位置・向きから、関節の目標角度を出す。ロボット本体とは別の計算用モデルを使う。"""

    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(SO101_SCENE))
        self.cfg = mink.Configuration(self.model)
        self.hand = mink.FrameTask("gripperframe", "site", position_cost=1.0, orientation_cost=0.3)
        self.posture = mink.PostureTask(self.model, cost=1e-3)
        self.limits = [mink.ConfigurationLimit(self.model)]
        self.qadr = [self.model.joint(m).qposadr[0] for m in MOTORS]
        lo, hi = self.model.joint("gripper").range
        self.grip_range = (lo, hi)
        # 手先の目印(gripperframe)は x 軸が指の向き。真下を向かせる回転(x→-z)
        self.down = mink.SO3.from_matrix(np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float))

    def sync(self, obs):
        q = np.deg2rad([obs[f"{m}.pos"] for m in MOTORS])
        lo, hi = self.grip_range
        q[-1] = lo + obs["gripper.pos"] / 100 * (hi - lo)
        self.cfg.update(q)
        self.posture.set_target_from_configuration(self.cfg)

    def tcp(self):
        return self.cfg.get_transform_frame_to_world("gripperframe", "site").translation()

    def tcp_rot(self):
        return self.cfg.get_transform_frame_to_world("gripperframe", "site").rotation()

    def grasp_rot(self, yaw):
        """真下を向いて、鉛直軸まわりに yaw だけ回った手先の向き。"""
        return mink.SO3.from_z_radians(yaw) @ self.down

    def solve(self, pos, rot, dt, iters=3):
        self.hand.set_target(mink.SE3.from_rotation_and_translation(rot, np.asarray(pos, float)))
        for _ in range(iters):
            vel = mink.solve_ik(self.cfg, [self.hand, self.posture], dt / iters, solver="daqp", damping=1e-3, limits=self.limits)
            self.cfg.integrate_inplace(vel, dt / iters)
        return np.rad2deg(self.cfg.q[self.qadr[:5]])


def plan_and_run(robot, cube_xy, cube_yaw, tray_xy, log, frames=None, record=None):
    """片付けの手順を、制御周期ごとの指令にして robot に送る。robot は SimSO101 でも SO101Follower でもよい。"""
    fps = getattr(robot.config, "fps", 30)
    dt = 1.0 / fps
    planner = Planner()
    obs = robot.get_observation()
    planner.sync(obs)
    grip = obs["gripper.pos"]
    x, y = cube_xy
    tx, ty = tray_xy
    yaw = (cube_yaw + np.pi / 4) % (np.pi / 2) - np.pi / 4  # キューブは 90° ごとに同じ
    # 手先は鉛直軸まわりに「肩の向き + キューブの向き」だけ回す(5 軸アームなので、肩の向きで決まる分がある)
    face = lambda px, py: np.arctan2(py, px)
    steps = [("真上へ", (x, y, SAFE_Z), face(x, y) + yaw, GRIP_OPEN, 1.5),
             ("降りる", (x, y, GRASP_Z), face(x, y) + yaw, GRIP_OPEN, 1.0),
             ("つかむ", (x, y, GRASP_Z), face(x, y) + yaw, GRIP_CLOSED, 0.8),
             ("持ち上げる", (x, y, SAFE_Z), face(x, y) + yaw, GRIP_CLOSED, 1.0),
             ("トレイの上へ", (tx, ty, SAFE_Z), face(tx, ty), GRIP_CLOSED, 1.5),
             ("下ろす", (tx, ty, DROP_Z), face(tx, ty), GRIP_CLOSED, 0.6),
             ("はなす", (tx, ty, DROP_Z), face(tx, ty), GRIP_OPEN, 0.6),
             ("戻る", (0.20, 0.0, 0.15), 0.0, GRIP_OPEN, 1.5)]
    # 位置も向きも、いまの値から目標へなめらかに変える(向きは回転の補間: R0・exp(u・log(R0⁻¹R1)))
    p0, r0, g0 = planner.tcp(), planner.tcp_rot(), grip
    t0 = time.perf_counter()
    for name, goal, gyaw, ggrip, dur in steps:
        n = int(round(dur * fps))
        r1 = planner.grasp_rot(gyaw)
        delta = (r0.inverse() @ r1).log()
        for k in range(1, n + 1):
            u = smooth(k / n)
            target = p0 + (np.asarray(goal) - p0) * u
            body = planner.solve(target, r0 @ mink.SO3.exp(delta * u), dt)
            g = g0 + (ggrip - g0) * u
            action = {f"{m}.pos": float(v) for m, v in zip(MOTORS[:5], body)}
            action["gripper.pos"] = float(g)
            sent = robot.send_action(action)
            obs = robot.get_observation()
            log.append({"t": len(log) / fps, "step": name, **{f"cmd_{m}": sent[f"{m}.pos"] for m in MOTORS},
                        **{f"obs_{m}": obs[f"{m}.pos"] for m in MOTORS}})
            if record:
                record(obs)
            if not isinstance(robot, SimSO101):  # 実機は実時間で回す
                time.sleep(max(0.0, t0 + len(log) * dt - time.perf_counter()))
        p0, r0, g0 = np.asarray(goal, float), r1, ggrip
    return log


def check_log(log, limit_deg=3.0):
    """指令と実際の角度の差(追従誤差)を関節ごとに調べる。大きすぎる関節があれば、重さやトルク不足を疑う。"""
    worst = {}
    for m in MOTORS[:5]:
        err = [abs(r[f"cmd_{m}"] - r[f"obs_{m}"]) for r in log]
        worst[m] = (float(np.max(err)), float(np.mean(err)))
    return worst


def save_log(log, path):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(log[0]))
        w.writeheader()
        w.writerows(log)


def plot_tracking(log, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = [r["t"] for r in log]
    fig, axes = plt.subplots(3, 2, figsize=(9, 6.5), sharex=True)
    for ax, m in zip(axes.flat, MOTORS):
        ax.plot(t, [r[f"cmd_{m}"] for r in log], label="command (send_action)", lw=1.6)
        ax.plot(t, [r[f"obs_{m}"] for r in log], label="measured (get_observation)", lw=1.2, ls="--")
        ax.set_title(m, fontsize=10)
        ax.grid(alpha=0.3)
    axes[0, 0].legend(fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel("time [s]")
    axes[0, 0].set_ylabel("deg")
    axes[2, 1].set_ylabel("0=closed, 100=open")
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def run_sim(cube_xy, cube_yaw, tray_xy, video=False, quiet=False):
    cams = {"front": (300, 400), "wrist": (300, 400)} if video else {}
    robot = SimSO101(SimSO101Config(cameras=cams, cube_xy=tuple(cube_xy), cube_yaw=cube_yaw, tray_xy=tuple(tray_xy),
                                    max_relative_target=10.0))
    robot.connect()
    frames, k = [], [0]

    def record(obs):
        k[0] += 1
        if video and k[0] % 2 == 0:
            frames.append(np.hstack([obs["front"], obs["wrist"]]))

    log = plan_and_run(robot, cube_xy, cube_yaw, tray_xy, [], record=record if video else None)
    ok = robot.cube_in_tray()
    robot.disconnect()
    if not quiet:
        print(f"[シミュレーター] キューブ {'トレイに入った' if ok else 'トレイに入らなかった'}({len(log)} ステップ、{len(log) / robot.config.fps:.1f} 秒)")
    return ok, log, frames


def write_video(frames, fps):
    import subprocess

    import imageio.v2 as imageio
    import imageio_ffmpeg

    mp4 = OUTPUT_DIR / "so101_pick_place.mp4"
    imageio.mimsave(mp4, frames, fps=fps)
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-loglevel", "error", "-y", "-i", str(mp4), "-vf",
                    "fps=10,scale=640:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=64[p];[b][p]paletteuse=dither=bayer",
                    str(OUTPUT_DIR / "so101_pick_place.gif")], check=True)
    return mp4


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", choices=["sim", "real"], default="sim")
    ap.add_argument("--port", default="/dev/ttyACM0", help="実機のサーボ基板のポート")
    ap.add_argument("--id", default="my_follower", help="LeRobot の校正ファイルの名前")
    ap.add_argument("--cube", type=float, nargs=2, default=[0.24, 0.10], help="キューブの位置 x y [m]")
    ap.add_argument("--cube-yaw", type=float, default=20.0, help="キューブの向き [度]")
    ap.add_argument("--tray", type=float, nargs=2, default=[0.18, -0.18], help="トレイの位置 x y [m]")
    ap.add_argument("--no-video", action="store_true")
    args = ap.parse_args()
    OUTPUT_DIR.mkdir(exist_ok=True)
    yaw = np.deg2rad(args.cube_yaw)

    # 1. どちらの場合も、まずシミュレーターで通しで試す(実機を壊さないための予行演習)
    ok, log, frames = run_sim(args.cube, yaw, args.tray, video=(args.robot == "sim" and not args.no_video))
    worst = check_log(log)
    for m, (mx, mean) in worst.items():
        print(f"  追従誤差 {m:13s}: 最大 {mx:5.2f}°, 平均 {mean:4.2f}°")
    save_log(log, OUTPUT_DIR / "sim_log.csv")
    plot_tracking(log, OUTPUT_DIR / "sim_tracking.png")
    if frames:
        print(f"動画を保存: {write_video(frames, 20)}")

    if args.robot == "sim":
        assert ok, "シミュレーターでキューブがトレイに入らなかった"
        return
    if not ok:
        raise SystemExit("シミュレーターで失敗したので、実機には送らない。キューブとトレイの位置を見直してください")

    # 2. 実機。同じ plan_and_run を、LeRobot の SO101Follower に対して呼ぶだけ
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.id, max_relative_target=5.0))
    robot.connect()
    try:
        input("キューブを置き、まわりに人や物がないことを確かめてから ENTER(Ctrl+C で中止)...")
        real_log = plan_and_run(robot, args.cube, yaw, args.tray, [])
    finally:
        robot.disconnect()
    save_log(real_log, OUTPUT_DIR / "real_log.csv")
    plot_tracking(real_log, OUTPUT_DIR / "real_tracking.png")
    for m, (mx, mean) in check_log(real_log).items():
        print(f"  実機の追従誤差 {m:13s}: 最大 {mx:5.2f}°, 平均 {mean:4.2f}°")
    print("output/sim_tracking.png と real_tracking.png を見比べると、シミュレーターと実機のずれが分かる")


if __name__ == "__main__":
    main()
