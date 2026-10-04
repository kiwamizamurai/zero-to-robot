"""カメラの取り付けがずれていたら？ ハンドアイ・キャリブレーションで直す。

panda_vision_pick では、天井カメラの取り付け位置・向き(外部パラメータ)を正確に知っている前提だった。
実機では、カメラは設計図どおりには付かない。ここでは本当のカメラを設計図から
3.7 cm ずらし、3.2° 傾けて取り付けておき、次の 3 段階を比べる。

A. 設計図どおりだと信じて見つける → 数 cm ずれて、つかみ損ねる
B. キャリブレーション: 手に付けた印(マゼンタの球)を 12 か所へ動かし、
   「ロボットが知っている印の位置(順運動学)」と「カメラが見た印の位置」の組から、
   カメラの本当の位置・向きを最小二乗で求める(Kabsch 法 / SVD)
C. 求めた位置・向きで見つけ直す → mm の精度に戻り、つかめる

実行:
    python apps/panda_calibration/calibrate.py             # 結果の図とキャリブレーションの動画を output/ に書き出す
    python apps/panda_calibration/calibrate.py --no-video  # 動画なし(速い)
"""
import argparse
import os
import sys
from pathlib import Path

if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")

import cv2
import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "panda_vision_pick"))
import vision_pick as vp  # noqa: E402

base = vp.base
OUTPUT_DIR = Path(__file__).parent / "output"

# 本当のカメラは、設計図(vp.OVERHEAD)から少しずれて付いている
MOUNT_SHIFT = np.array([0.03, -0.02, 0.01])  # [m]
MOUNT_TILT_DEG = (2.0, -2.0, 1.5)            # x, y, z 軸まわり [°]
MARKER_R = 0.012                              # 手に付けた印(球)の半径 [m]
MARKER_POS_IN_HAND = [0.075, 0.0, 0.03]
CALIB_POINTS = [(x, y, z) for z in (0.18, 0.32) for x in (0.40, 0.58) for y in (-0.12, 0.08, 0.28)]


def quat_from_axis_angles(ax, ay, az):
    q = np.array([1.0, 0, 0, 0])
    for axis, deg in zip(np.eye(3), (ax, ay, az)):
        qi = np.zeros(4)
        mujoco.mju_axisAngle2Quat(qi, axis, np.deg2rad(deg))
        mujoco.mju_mulQuat(q, q, qi)
    return q


def build_model(blocks):
    spec = vp.build_spec(blocks)
    cam = spec.camera("overhead")
    cam.pos = list(np.array(vp.OVERHEAD["pos"]) + MOUNT_SHIFT)
    cam.quat = list(quat_from_axis_angles(*MOUNT_TILT_DEG))
    # キャリブレーション用の印。手の横に付けて、上から見えるようにする。ぶつからない(接触しない)ようにする
    spec.body("hand").add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[MARKER_R, 0, 0], pos=MARKER_POS_IN_HAND,
                               rgba=[0.9, 0.1, 0.8, 1], contype=0, conaffinity=0, mass=0.001)
    spec.body("hand").add_site(name="marker", pos=MARKER_POS_IN_HAND)
    return spec.compile()


def nominal_extrinsics():
    """設計図どおりのカメラ: 指定の位置から真下を向く(回転なし)。"""
    return np.eye(3), np.array(vp.OVERHEAD["pos"], dtype=float)


def find_marker(eye, data):
    """カメラ画像からマゼンタの球を探し、カメラから見た球の中心 [m] を返す。"""
    rgb, depth = eye.shoot(data)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (140, 80, 80), (165, 255, 255))
    if mask.sum() < 255 * 15:
        return None, rgb
    v, u = np.nonzero(mask)
    d = float(np.median(depth[mask > 0]))
    surface = eye.pixel_to_camera(u.mean() + 0.5, v.mean() + 0.5, d)
    center = surface + MARKER_R * surface / np.linalg.norm(surface)  # 見えているのは球の表面なので、奥へ半径分
    return center, rgb


def kabsch(P_cam, P_world):
    """P_world ≈ R @ P_cam + t となる回転 R・平行移動 t を最小二乗で求める(SVD を使う Kabsch 法)。"""
    mc, mw = P_cam.mean(0), P_world.mean(0)
    H = (P_cam - mc).T @ (P_world - mw)
    U, _, Vt = np.linalg.svd(H)
    S = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])  # 鏡映にならないように
    R = Vt.T @ S @ U.T
    return R, mw - R @ mc


def rot_angle_deg(R):
    return np.rad2deg(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def try_picks(model_blocks, extrinsics, label, record=None):
    """見つけて、4 個つかみに行く(1 個につき 1 回だけ)。見つけた位置の誤差と成功数を返す。"""
    model = build_model(model_blocks)
    data = mujoco.MjData(model)
    robot = base.Robot(model, data)
    eye = vp.Eye(model)
    eye.extrinsics = extrinsics
    robot.move(vp.LOOK_POSE, 1.4, record)
    robot.gripper(robot.grip, 0.3, record)
    found, annotated = eye.detect(data)
    errors = {}
    for f in found:
        errors[f["name"]] = np.linalg.norm(f["pos"][:2] - data.body(f"block_{f['name']}").xpos[:2]) * 1000
    for i, f in enumerate(sorted(found, key=lambda f: np.hypot(*f["pos"][:2]))):
        vp.pick(robot, f, vp.DROP_SPOTS[i % 4], record)
    robot.gripper(base.GRIP_OPEN, 1.0, record)
    ok = sum(base.in_box(data.body(f"block_{n}").xpos) for n in vp.COLORS)
    eye.close()
    print(f"[{label}] 見つけた位置の誤差: " + ", ".join(f"{n} {e:.0f} mm" for n, e in errors.items())
          + f" → 箱に入ったのは {ok} / {len(vp.COLORS)} 個")
    return errors, ok, found, data, annotated


def calibrate(blocks, record=None, record_eye=None):
    """手の印を 12 か所へ動かし、ロボットが知っている位置とカメラが見た位置の組を集めて解く。"""
    model = build_model(blocks)
    data = mujoco.MjData(model)
    robot = base.Robot(model, data)
    eye = vp.Eye(model)
    P_cam, P_world = [], []
    for k, p in enumerate(CALIB_POINTS):
        robot.yaw = np.deg2rad(20 * ((k % 3) - 1))  # 手首の向きも少しずつ変える
        robot.move(p, 1.0, record)
        robot.gripper(robot.grip, 0.3, record)  # 揺れが収まるのを待ってから撮る
        seen, rgb = find_marker(eye, data)
        world = data.site("marker").xpos.copy()  # ロボットは関節角度から手の位置を知っている(順運動学)
        if record_eye:
            record_eye(rgb, seen is not None)
        if seen is None:
            print(f"  {k + 1:2d}: 印が見えなかった(腕に隠れたか、視界の外)ので使わない")
            continue
        P_cam.append(seen)
        P_world.append(world)
    P_cam, P_world = np.array(P_cam), np.array(P_world)
    R, t = kabsch(P_cam, P_world)
    residual = np.linalg.norm(P_world - (P_cam @ R.T + t), axis=1)
    true_R, true_t = data.cam_xmat[eye.cam].reshape(3, 3), data.cam_xpos[eye.cam]
    eye.close()
    print(f"[B キャリブレーション] 使った点 {len(P_cam)} 個、当てはまりの誤差(RMS) {np.sqrt((residual ** 2).mean()) * 1000:.1f} mm")
    nR, nt = nominal_extrinsics()
    print(f"  設計図のカメラ     : 位置のずれ {np.linalg.norm(nt - true_t) * 1000:5.1f} mm, 向きのずれ {rot_angle_deg(nR.T @ true_R):.2f}°")
    print(f"  キャリブレーション後: 位置のずれ {np.linalg.norm(t - true_t) * 1000:5.1f} mm, 向きのずれ {rot_angle_deg(R.T @ true_R):.2f}°")
    return (R, t), P_world


def plot(blocks, before, after, calib_pts, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(8, 5.2), sharex=True, sharey=True)
    for ax, (title, found, errors, ok) in zip(axes, [before, after]):
        for name, (x, y), rgba, _ in blocks:
            ax.add_patch(plt.Rectangle((x - 0.02, y - 0.02), 0.04, 0.04, color=rgba[:3], alpha=0.35))
            ax.plot(x, y, "+", color="k", ms=8)
        for f in found:
            name = f["name"]
            tx, ty = next(p for n, p, _, _ in blocks if n == name)
            ax.annotate("", xy=f["pos"][:2], xytext=(tx, ty), arrowprops=dict(arrowstyle="->", color="k", lw=1.2))
            ax.plot(*f["pos"][:2], "o", color=vp.COLORS[name][0][:3], mec="k", ms=6)
        ax.add_patch(plt.Rectangle((base.BOX_CENTER[0] - base.BOX_HALF, base.BOX_CENTER[1] - base.BOX_HALF),
                                   2 * base.BOX_HALF, 2 * base.BOX_HALF, fill=False, ec="saddlebrown", lw=2))
        mean = np.mean(list(errors.values()))
        ax.set_title(f"{title}\nmean error {mean:.0f} mm, {ok}/4 in the box", fontsize=11)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.set_xlabel("x [m]")
    axes[1].plot(calib_pts[:, 0], calib_pts[:, 1], "x", color="m", ms=7, label="marker positions used\nfor calibration")
    axes[1].legend(loc="lower right", fontsize=8, framealpha=0.95)
    axes[0].set_ylabel("y [m]")
    axes[0].set_xlim(0.25, 0.85)
    axes[0].set_ylim(-0.52, 0.42)
    fig.suptitle("square = true block, dot = where the camera thought it was", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main(seed, record_video):
    rng = np.random.default_rng(seed)
    blocks = vp.random_blocks(rng)
    OUTPUT_DIR.mkdir(exist_ok=True)

    # A. 設計図どおりだと信じる
    err_a, ok_a, found_a, _, _ = try_picks(blocks, nominal_extrinsics(), "A 設計図どおりと信じる")

    # B. キャリブレーション(ここだけ動画にする)
    frames, record, record_eye, scene = [], None, None, None
    if record_video:
        model = build_model(blocks)
        scene = mujoco.Renderer(model, height=352, width=480)
        front = model.camera("front").id
        state = {"eye": np.zeros((352, 352, 3), np.uint8), "n": 0, "k": 0}

        def record(d):
            state["k"] += 1
            if state["k"] % 5:
                return
            scene.update_scene(d, camera=front)
            frames.append(np.hstack([scene.render(), state["eye"]]))

        def record_eye(rgb, seen):
            img = rgb.copy()
            state["n"] += seen
            vp.put_text(img, "overhead camera", (8, 18), 0.45)
            vp.put_text(img, f"marker {'seen' if seen else 'hidden'}  ({state['n']} points)", (8, 340), 0.45)
            state["eye"] = img

    extr, calib_pts = calibrate(blocks, record, record_eye)

    # C. 求めた位置・向きで見つけ直す
    err_c, ok_c, found_c, _, _ = try_picks(blocks, extr, "C キャリブレーション後")

    plot(blocks, ("A: trust the drawing", found_a, err_a, ok_a), ("C: after calibration", found_c, err_c, ok_c),
         calib_pts, OUTPUT_DIR / "before_after.png")
    print(f"図を保存: {OUTPUT_DIR / 'before_after.png'}")

    if record_video:
        scene.close()
        import subprocess

        import imageio.v2 as imageio
        import imageio_ffmpeg
        mp4 = OUTPUT_DIR / "calibration.mp4"
        imageio.mimsave(mp4, frames, fps=10)
        subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-loglevel", "error", "-y", "-i", str(mp4), "-vf",
                        "fps=6,scale=640:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=48[p];[b][p]paletteuse=dither=bayer",
                        str(OUTPUT_DIR / "calibration.gif")], check=True)
        print(f"動画を保存: {mp4}, {OUTPUT_DIR / 'calibration.gif'}")
    return ok_a, ok_c, err_a, err_c


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--no-video", action="store_true")
    args = parser.parse_args()
    ok_a, ok_c, err_a, err_c = main(args.seed, record_video=not args.no_video)
    assert ok_c == len(vp.COLORS), "キャリブレーション後に箱へ入らなかったブロックがある"
    assert np.mean(list(err_c.values())) < np.mean(list(err_a.values())) / 5, "キャリブレーションで誤差が十分に減らなかった"
