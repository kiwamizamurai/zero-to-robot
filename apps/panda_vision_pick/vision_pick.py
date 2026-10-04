"""カメラで見つけて片付ける(Franka Panda × MuJoCo × mink × OpenCV)。

apps/panda_pick_place ではブロックの位置をシミュレーターから直接読んでいた。
実機ではそれはできないので、ここでは天井カメラの画像だけからブロックを見つける。

1. 見る:     アームをカメラの視界からどけて、天井カメラで色画像と距離画像(深度)を撮る
2. 見つける: OpenCV で色ごとに領域を切り出し、中心と向きを求める
3. 位置に直す: 画素の位置と深度を、カメラの向き・画角を使って 3 次元の座標に変換する
4. つかむ:   見つけた位置・向きに合わせて手首をひねって、真上からつかむ(panda_pick_place と同じ)
5. 見直す:   1 個運ぶたびに撮り直す。落としたブロックも次の撮影で見つかるので、やり直せる

ブロックは実行のたびにランダムな位置・向きに置く(--seed で固定できる)。

実行:
    python apps/panda_vision_pick/vision_pick.py              # output/ に動画と検出画像を書き出す
    python apps/panda_vision_pick/vision_pick.py --seed 3     # 置き方を変える
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

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "panda_pick_place"))
import pick_place as base  # noqa: E402

OUTPUT_DIR = Path(__file__).parent / "output"
COLORS = {  # 名前: (RGBA, OpenCV の HSV で色相の範囲 [0, 180))
    "red": ((0.85, 0.20, 0.18, 1), [(0, 8), (172, 180)]),
    "blue": ((0.18, 0.45, 0.90, 1), [(85, 130)]),
    "green": ((0.20, 0.70, 0.30, 1), [(50, 80)]),
    "yellow": ((0.95, 0.80, 0.15, 1), [(20, 35)]),
}
PICK_AREA = ((0.35, 0.68), (-0.15, 0.35))  # ブロックを置く範囲 x, y [m]
DROP_SPOTS = [(-0.045, -0.045), (0.045, 0.045), (0.045, -0.045), (-0.045, 0.045)]
OVERHEAD = dict(pos=(0.50, 0.08, 1.10), fovy=42.0, size=352)
LOOK_POSE = (0.05, -0.42, 0.40)  # 撮影のあいだ、手先をカメラの視界の外へどける


def random_blocks(rng):
    """ブロックを互いに 9 cm 以上離して、ランダムな位置・向きに置く。"""
    placed = []
    for name, (rgba, _) in COLORS.items():
        while True:
            x, y = rng.uniform(*PICK_AREA[0]), rng.uniform(*PICK_AREA[1])
            if all(np.hypot(x - px, y - py) > 0.09 for _, (px, py), _, _ in placed):
                break
        placed.append((name, (x, y), rgba, rng.uniform(-np.pi / 4, np.pi / 4)))
    return placed


def build_spec(blocks):
    spec = base.build_spec([(n, p, rgba) for n, p, rgba, _ in blocks])
    for name, _, _, yaw in blocks:
        spec.body(f"block_{name}").quat = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
    # 天井カメラ: 真下を向く(MuJoCo のカメラは自分の -z 方向を見る。画像の右が世界の +x、上が +y)
    cam = spec.worldbody.add_camera(name="overhead", pos=list(OVERHEAD["pos"]))
    # 床を青から灰色に変える。青い床の上の青いブロックは、色だけでは見分けられない(実機でも背景選びは大事)
    tex = spec.texture("groundplane")
    tex.rgb1, tex.rgb2, tex.markrgb = [0.34, 0.34, 0.33], [0.27, 0.27, 0.26], [0.6, 0.6, 0.6]
    cam.fovy = OVERHEAD["fovy"]
    return spec


def build_model(blocks):
    return build_spec(blocks).compile()


def put_text(img, text, org, scale):
    """黒でふちどりした白い文字(どんな背景でも読める)。"""
    for color, thick in (((0, 0, 0), 3), ((255, 255, 255), 1)):
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


class Eye:
    """天井カメラで撮って、ブロックを見つけ、世界座標に直す。"""

    def __init__(self, model):
        n = OVERHEAD["size"]
        self.model, self.cam = model, model.camera("overhead").id
        self.rgb = mujoco.Renderer(model, height=n, width=n)
        self.depth = mujoco.Renderer(model, height=n, width=n)
        self.depth.enable_depth_rendering()
        # 画角から焦点距離(画素単位)を出す。カメラの内部パラメータ
        self.f = (n / 2) / np.tan(np.deg2rad(OVERHEAD["fovy"]) / 2)
        self.c = n / 2
        # カメラの取り付け位置・向き(外部パラメータ)。None のあいだはシミュレーターの本当の値を使う
        self.extrinsics = None

    def shoot(self, data):
        self.rgb.update_scene(data, camera=self.cam)
        self.depth.update_scene(data, camera=self.cam)
        return self.rgb.render().copy(), self.depth.render().copy()

    def pixel_to_camera(self, u, v, d):
        """画素 (u, v) と深度 d [m] を、カメラから見た 3 次元の点へ(内部パラメータだけで決まる)。"""
        return np.array([(u - self.c) * d / self.f, -(v - self.c) * d / self.f, -d])

    def pixel_to_world(self, data, u, v, d):
        """画素 (u, v) と深度 d [m] を世界座標へ。カメラ座標で点を作り、カメラの姿勢で回して足す。"""
        if self.extrinsics is None:
            R, t = data.cam_xmat[self.cam].reshape(3, 3), data.cam_xpos[self.cam]
        else:
            R, t = self.extrinsics
        return t + R @ self.pixel_to_camera(u, v, d)

    def detect(self, data):
        rgb, depth = self.shoot(data)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        found, annotated = [], cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        for name, (_, hue_ranges) in COLORS.items():
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for lo, hi in hue_ranges:  # 鮮やかで明るい画素だけ(床・箱・影をよける)
                mask |= cv2.inRange(hsv, (lo, 80, 100), (hi - 1, 255, 255))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) < 40:
                    continue
                (u, v), _, _ = cv2.minAreaRect(cnt)
                region = np.zeros_like(mask)
                cv2.drawContours(region, [cnt], -1, 255, -1)
                d = float(np.median(depth[region > 0]))  # 上の面までの距離
                top = self.pixel_to_world(data, u + 0.5, v + 0.5, d)
                if in_box(top):  # 箱の中のブロックや、照明で黄色っぽく写る箱のふちは対象外(箱の位置は分かっている)
                    continue
                # 向き: 輪郭を囲む最小の長方形の一辺の傾き。画像の下向きは世界の -y なので符号を反転
                box = cv2.boxPoints(cv2.minAreaRect(cnt))
                e = box[1] - box[0]
                yaw = (np.arctan2(-e[1], e[0]) + np.pi / 4) % (np.pi / 2) - np.pi / 4
                found.append(dict(name=name, pos=np.array([top[0], top[1], top[2] - base.BLOCK]), yaw=yaw))
                cv2.drawContours(annotated, [np.intp(box)], -1, (255, 255, 255), 1)
                cv2.circle(annotated, (int(u), int(v)), 3, (255, 255, 255), -1)
                put_text(annotated, name, (int(u) + 9, int(v) + 4), 0.42)
        put_text(annotated, "overhead camera", (8, 18), 0.45)
        for i, f in enumerate(found):  # 見つけたものの一覧: 名前, 位置 (x, y) [cm], 向き [deg]
            x, y, _ = f["pos"] * 100
            put_text(annotated, f"{f['name']:6s} ({x:4.1f}, {y:5.1f}) cm {np.rad2deg(f['yaw']):+4.0f} deg",
                     (8, annotated.shape[0] - 10 - 16 * (len(found) - 1 - i)), 0.4)
        return found, annotated

    def close(self):
        self.rgb.close()
        self.depth.close()


def in_box(p):
    return abs(p[0] - base.BOX_CENTER[0]) < base.BOX_HALF + 0.02 and abs(p[1] - base.BOX_CENTER[1]) < base.BOX_HALF + 0.02


def pick(robot, target, drop, record):
    x, y, _ = target["pos"]
    robot.yaw = target["yaw"]                          # ブロックの向きに合わせて手首をひねる
    robot.move([x, y, base.SAFE_Z], 1.6, record)
    robot.move([x, y, base.BLOCK + 0.002], 1.0, record)
    robot.gripper(base.GRIP_CLOSED, 0.6, record)
    robot.move([x, y, base.SAFE_Z], 1.0, record)
    robot.yaw = 0.0
    bx, by = base.BOX_CENTER[0] + drop[0], base.BOX_CENTER[1] + drop[1]
    robot.move([bx, by, base.SAFE_Z], 1.6, record)
    robot.move([bx, by, base.BOX_HEIGHT + 0.06], 0.8, record)
    robot.gripper(base.GRIP_OPEN, 0.5, record)
    robot.move([bx, by, base.SAFE_Z], 0.8, record)


def run(seed, record_video=True):
    rng = np.random.default_rng(seed)
    blocks = random_blocks(rng)
    model = build_model(blocks)
    data = mujoco.MjData(model)
    robot = base.Robot(model, data)
    eye = Eye(model)

    frames, view = [], {"img": None}
    record = None
    if record_video:
        scene = mujoco.Renderer(model, height=352, width=480)
        front = model.camera("front").id
        every = int(round(1 / (20 * model.opt.timestep * 10)))
        counter = [0]

        def record(d):
            counter[0] += 1
            if counter[0] % every:
                return
            scene.update_scene(d, camera=front)
            left = scene.render()
            right = view["img"] if view["img"] is not None else np.zeros((352, 352, 3), np.uint8)
            frames.append(np.hstack([left, right]))

    print(f"seed={seed}")
    placed, attempts = 0, 0
    while attempts < 8:
        attempts += 1
        robot.move(LOOK_POSE, 1.4, record)                 # 1. 見る: カメラの視界からどく
        robot.gripper(robot.grip, 0.3, record)            #    揺れが収まるのを待つ
        found, annotated = eye.detect(data)               # 2・3. 見つけて、位置に直す
        view["img"] = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
        if attempts == 1:
            OUTPUT_DIR.mkdir(exist_ok=True)
            cv2.imwrite(str(OUTPUT_DIR / "detection.png"), annotated)
            for f in found:
                true = data.body(f"block_{f['name']}")
                true_yaw = 2 * np.arctan2(true.xquat[3], true.xquat[0])
                dyaw = (f["yaw"] - true_yaw + np.pi / 4) % (np.pi / 2) - np.pi / 4
                err = np.linalg.norm(f["pos"][:2] - true.xpos[:2]) * 1000
                print(f"  見つけた {f['name']:6s}: 位置の誤差 {err:4.1f} mm, 向きの誤差 {np.rad2deg(dyaw):+5.1f}°")
        if not found:
            break
        target = min(found, key=lambda f: np.hypot(*f["pos"][:2]))  # 近いものから
        print(f"  {attempts} 回目: {target['name']} をつかみに行く")
        pick(robot, target, DROP_SPOTS[placed % len(DROP_SPOTS)], record)  # 4. つかむ
        placed += 1

    robot.gripper(base.GRIP_OPEN, 1.0, record)
    results = {name: base.in_box(data.body(f"block_{name}").xpos) for name in COLORS}
    for name, ok in results.items():
        print(f"  {name:6s}: {'箱の中' if ok else '箱の外'}")
    print(f"  つかみに行った回数 {placed} / ブロック {len(COLORS)} 個")
    eye.close()

    if record_video:
        scene.close()
        import subprocess

        import imageio.v2 as imageio
        import imageio_ffmpeg
        mp4 = OUTPUT_DIR / "vision_pick.mp4"
        imageio.mimsave(mp4, frames, fps=20)
        subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-loglevel", "error", "-y", "-i", str(mp4), "-vf",
                        "fps=6,scale=640:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=48[p];[b][p]paletteuse=dither=bayer",
                        str(OUTPUT_DIR / "vision_pick.gif")], check=True)
        print(f"動画を保存: {mp4}, {OUTPUT_DIR / 'vision_pick.gif'}, {OUTPUT_DIR / 'detection.png'}")
    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=0, help="ブロックの置き方を決める乱数の種")
    parser.add_argument("--no-video", action="store_true", help="動画を書き出さない(速い)")
    args = parser.parse_args()
    results = run(args.seed, record_video=not args.no_video)
    assert all(results.values()), "箱に入らなかったブロックがある"
