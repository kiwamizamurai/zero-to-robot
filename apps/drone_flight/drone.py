"""ドローンを飛ばす(MuJoCo Menagerie の Skydio X2 × カスケード制御)。

ドローンが動かせるのは 4 つのプロペラの推力だけ。横に進むための「横向きの力」を直接は出せない。
横に進みたいときは、まず機体を傾けて、推力の向きを斜めにする。だから制御を 3 段に重ねる(カスケード制御)。

1. 位置の制御(外側、ゆっくり): 「目標の位置へ行くには、どの向きにどれだけ加速したいか」を決める
2. 姿勢の制御(内側、速い): 「その向きに推力を向けるには、機体をどう傾けるか」を決め、回転させるトルクを出す
3. ミキサー: 「全体の推力」と「3 軸のトルク」を、4 つのプロペラそれぞれの推力に配分する

飛行の流れ: 離陸 → 四角く巡回(途中で向きを 90° 変える)→ 横からの突風に耐える → 戻って着陸

実行:
    python apps/drone_flight/drone.py                # 図と動画を output/ に書き出す
    python apps/drone_flight/drone.py --no-attitude  # 姿勢の制御を弱くすると何が起きるか
"""
import argparse
import os
import sys
from pathlib import Path

if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from robot_descriptions import panda_mj_description  # Menagerie の置き場所を借りる

MENAGERIE = Path(panda_mj_description.REPOSITORY_PATH)
SCENE = MENAGERIE / "skydio_x2" / "scene.xml"
OUTPUT_DIR = Path(__file__).parent / "output"
G = 9.81

# 巡回する点 (x, y, z, 機首の向き[度]) と、そこまでにかける時間 [s]
WAYPOINTS = [((0, 0, 1.0, 0), 2.5), ((1.2, 0, 1.0, 0), 2.5), ((1.2, 1.2, 1.4, 90), 3.0),
             ((0, 1.2, 1.0, 90), 2.5), ((0, 0, 1.0, 90), 2.5), ((0, 0, 1.0, 90), 2.5), ((0, 0, 0.12, 90), 3.0)]
GUST = (14.0, 15.0, np.array([0.0, 5.0, 0.0]))  # 14〜15 秒に横から 5 N の突風


def build():
    spec = mujoco.MjSpec.from_file(str(SCENE))
    # 巡回点の目印(ぶつからない、見えるだけの玉)
    for i, ((x, y, z, _), _) in enumerate(WAYPOINTS[:5]):
        spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.04, 0, 0], pos=[x, y, z],
                                rgba=[1, 0.6, 0.1, 0.8], contype=0, conaffinity=0)
    spec.worldbody.add_body(name="look_at", pos=[0.6, 0.6, 0.8])
    cam = spec.worldbody.add_camera(name="chase", pos=[3.6, -2.4, 2.6])
    cam.mode = mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY
    cam.targetbody = "look_at"
    return spec.compile()


def min_jerk(p0, p1, T, t):
    """5 次多項式(05_planning): 位置・速度・加速度を返す。両端で速度も加速度も 0。"""
    s = np.clip(t / T, 0, 1)
    pos = p0 + (p1 - p0) * (10 * s**3 - 15 * s**4 + 6 * s**5)
    vel = (p1 - p0) * (30 * s**2 - 60 * s**3 + 30 * s**4) / T
    acc = (p1 - p0) * (60 * s - 180 * s**2 + 120 * s**3) / T**2
    return pos, vel, acc


def reference(t, start):
    """時刻 t の目標(位置・速度・加速度・機首の向き)。"""
    p0, y0, t0 = np.array(start), 0.0, 0.0
    for (wp, T) in WAYPOINTS:
        p1, y1 = np.array(wp[:3], float), np.deg2rad(wp[3])
        if t < t0 + T:
            pos, vel, acc = min_jerk(p0, p1, T, t - t0)
            yaw, _, _ = min_jerk(np.array([y0]), np.array([y1]), T, t - t0)
            return pos, vel, acc, yaw[0]
        p0, y0, t0 = p1, y1, t0 + T
    return p0, np.zeros(3), np.zeros(3), y0


class Controller:
    def __init__(self, model, data, attitude_gain=1.0):
        self.m, self.d = model, data
        self.body = model.body("x2").id
        self.mass = model.body_subtreemass[self.body]
        self.I = model.body_inertia[self.body]
        # ゲイン: 外側(位置)はゆっくり、内側(姿勢)は速く。内側が外側より十分速いのがカスケードの前提
        self.kp, self.kd = np.array([12.0, 12.0, 14.0]), np.array([6.0, 6.0, 7.0])
        self.kR = attitude_gain * np.array([60.0, 60.0, 15.0])
        self.kw = attitude_gain * np.array([12.0, 12.0, 5.0])
        self.mixer_inv = self._mixer()
        self.fmax = model.actuator_ctrlrange[:, 1]

    def _mixer(self):
        """4 つの推力 f → (全体の推力, x・y・z 軸まわりのトルク) の行列 M を作り、その逆行列を返す。"""
        mujoco.mj_forward(self.m, self.d)
        com = self.d.subtree_com[self.body]
        M = np.zeros((4, 4))
        for i in range(4):
            r = self.d.site(f"thrust{i + 1}").xpos - com
            yaw_coeff = self.m.actuator_gear[i, 5]  # プロペラの回転の反作用で生まれる、z 軸まわりのトルク
            M[:, i] = [1.0, r[1], -r[0], yaw_coeff]
        return np.linalg.inv(M)

    def __call__(self, pos_ref, vel_ref, acc_ref, yaw_ref):
        d = self.d
        p, v = d.qpos[:3], d.qvel[:3]
        R = d.xmat[self.body].reshape(3, 3)
        w = d.qvel[3:6]  # 機体座標の角速度(MuJoCo の自由関節は回転速度を機体座標で持つ)
        # 1. 位置の制御: 欲しい加速度(重力を打ち消す分も足す)
        a = acc_ref + self.kp * (pos_ref - p) + self.kd * (vel_ref - v) + np.array([0, 0, G])
        thrust = self.mass * a @ R[:, 2]
        # 2. 姿勢の制御: 機体の上向き(z 軸)を a の向きにそろえ、機首を yaw_ref に向ける
        zb = a / np.linalg.norm(a)
        xc = np.array([np.cos(yaw_ref), np.sin(yaw_ref), 0])
        yb = np.cross(zb, xc)
        yb /= np.linalg.norm(yb)
        Rd = np.column_stack([np.cross(yb, zb), yb, zb])
        E = 0.5 * (Rd.T @ R - R.T @ Rd)
        eR = np.array([E[2, 1], E[0, 2], E[1, 0]])  # 向きのずれ(回転行列の差から取り出す)
        torque = self.I * (-self.kR * eR - self.kw * w) + np.cross(w, self.I * w)
        # 3. ミキサー: 4 つのプロペラの推力に配分する(出せる範囲に切る)
        f = self.mixer_inv @ np.concatenate([[thrust], torque])
        return np.clip(f, 0, self.fmax), Rd


def fly(attitude_gain=1.0, video=False):
    model = build()
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("hover").id)
    data.qpos[2] = 0.12  # 地面から離陸する
    mujoco.mj_forward(model, data)
    ctl = Controller(model, data, attitude_gain)
    start = data.qpos[:3].copy()
    total = sum(T for _, T in WAYPOINTS) + 1.0
    log, frames = [], []
    renderer = mujoco.Renderer(model, height=360, width=540) if video else None
    body = model.body("x2").id
    crashed = False
    while data.time < total:
        t = data.time
        pos_ref, vel_ref, acc_ref, yaw_ref = reference(t, start)
        f, _ = ctl(pos_ref, vel_ref, acc_ref, yaw_ref)
        data.ctrl[:] = f
        gusting = GUST[0] <= t < GUST[1]
        data.xfrc_applied[body, :3] = GUST[2] if gusting else 0.0
        mujoco.mj_step(model, data)
        R = data.xmat[body].reshape(3, 3)
        tilt = np.rad2deg(np.arccos(np.clip(R[2, 2], -1, 1)))
        log.append(dict(t=t, ref=pos_ref.copy(), pos=data.qpos[:3].copy(), f=f.copy(), tilt=tilt, gust=gusting))
        if tilt > 80 or data.qpos[2] < 0.03:
            crashed = True
            break
        if video and len(log) % 4 == 0:
            renderer.update_scene(data, camera="chase")
            frames.append(label(renderer.render(), t, pos_ref, data.qpos[:3], tilt, gusting))
    if renderer:
        renderer.close()
    return log, frames, crashed


def label(img, t, ref, pos, tilt, gust):
    import cv2

    img = np.ascontiguousarray(img)
    err = np.linalg.norm(ref - pos) * 100
    lines = [f"t {t:5.1f} s   error {err:5.1f} cm   tilt {tilt:4.1f} deg"]
    if gust:
        lines.append("GUST! 5 N from the side")
    for i, s in enumerate(lines):
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):  # 黒いふちどり
            cv2.putText(img, s, (10 + dx, 24 + 22 * i + dy), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(img, s, (10, 24 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def plot(log, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.array([r["t"] for r in log])
    ref, pos = np.array([r["ref"] for r in log]), np.array([r["pos"] for r in log])
    f, tilt = np.array([r["f"] for r in log]), np.array([r["tilt"] for r in log])
    fig = plt.figure(figsize=(10, 6.2))
    ax = fig.add_subplot(2, 2, (1, 3))
    ax.plot(ref[:, 0], ref[:, 1], "--", color="gray", label="reference")
    ax.plot(pos[:, 0], pos[:, 1], lw=1.8, label="actual")
    g = np.array([r["gust"] for r in log])
    if g.any():
        ax.plot(pos[g, 0], pos[g, 1], "r", lw=3, label="during gust")
    for (wp, _) in WAYPOINTS[:5]:
        ax.plot(wp[0], wp[1], "o", color="orange")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("top view", fontsize=10)
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(t, np.linalg.norm(ref - pos, axis=1) * 100, label="position error [cm]")
    ax2.plot(t, tilt, label="tilt [deg]")
    ax2.axvspan(GUST[0], GUST[1], color="red", alpha=0.15, label="gust")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)
    ax3 = fig.add_subplot(2, 2, 4, sharex=ax2)
    for i in range(4):
        ax3.plot(t, f[:, i], lw=1, label=f"rotor {i + 1}")
    ax3.axhline(13, color="k", ls=":", lw=1)
    ax3.set_ylabel("thrust [N]")
    ax3.set_xlabel("time [s]")
    ax3.legend(fontsize=7, ncol=4)
    ax3.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-attitude", action="store_true", help="姿勢の制御を 1/10 に弱める")
    ap.add_argument("--no-video", action="store_true")
    args = ap.parse_args()
    OUTPUT_DIR.mkdir(exist_ok=True)
    gain = 0.1 if args.no_attitude else 1.0
    log, frames, crashed = fly(gain, video=not args.no_video)
    t = np.array([r["t"] for r in log])
    err = np.array([np.linalg.norm(r["ref"] - r["pos"]) for r in log]) * 100
    gust = np.array([r["gust"] for r in log])
    print(f"姿勢の制御 {gain:.1f} 倍: {'墜落した' if crashed else '最後まで飛んだ'}({t[-1]:.1f} 秒)")
    print(f"  目標とのずれ: 平均 {err.mean():.1f} cm、最大 {err.max():.1f} cm")
    if gust.any() and not crashed:
        after = (t >= GUST[0]) & (t < GUST[0] + 2.5)
        print(f"  突風(5 N、1 秒)を受けたときの最大のずれ: {err[after].max():.1f} cm、最大の傾き {max(r['tilt'] for r, a in zip(log, after) if a):.1f}°")
    print(f"  着陸時の位置: ({log[-1]['pos'][0]:+.2f}, {log[-1]['pos'][1]:+.2f}, {log[-1]['pos'][2]:.2f}) m")
    name = "flight_weak_attitude" if args.no_attitude else "flight"
    plot(log, OUTPUT_DIR / f"{name}.png")
    print(f"図を保存: {OUTPUT_DIR / f'{name}.png'}")
    if frames:
        import subprocess

        import imageio.v2 as imageio
        import imageio_ffmpeg
        mp4 = OUTPUT_DIR / f"{name}.mp4"
        imageio.mimsave(mp4, frames, fps=25)
        subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-loglevel", "error", "-y", "-i", str(mp4), "-vf",
                        "fps=8,scale=440:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=48[p];[b][p]paletteuse=dither=bayer",
                        str(OUTPUT_DIR / f"{name}.gif")], check=True)
        print(f"動画を保存: {mp4}")
    if not args.no_attitude:
        assert not crashed and err.mean() < 10, "巡回できなかった"


if __name__ == "__main__":
    main()
