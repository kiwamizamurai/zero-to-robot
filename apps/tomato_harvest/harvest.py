"""ミニトマト収穫ロボット(SO-101 × MuJoCo × OpenCV)。

鉢植えのミニトマトから、**赤く熟した実だけ**を選んで収穫し、かごに入れる。緑やオレンジの実は残す。
使うアームは、これから買う SO-101(so101_twin と同じ、LeRobot と同じ口のシミュレーター)。

1. 見る:     アームをカメラの視界からどけて、RGB-D カメラ(色 + 距離)で鉢植えを撮る
2. 見分ける: 色(HSV の色相)で実を切り出し、赤の割合で熟し具合を判定する。緑の実は葉と同じ色なので、そもそも狙わない
3. 位置に直す: 画素と距離から、実の 3 次元の位置を求める
4. 近づく:   実はぶら下がっているので、真上からではなく、斜め上から水平に近い向きで近づく(手前で一度止まる)
5. つかんで切る: 指を閉じてから、へた(果柄)を切る。シミュレーターでは「茎と実をつなぐ拘束」を外して表す
6. かごへ:   引き抜いて、かごの上で指を開く。1 個とるたびに撮り直す

実行:
    python apps/tomato_harvest/harvest.py              # 動画・画像を output/ に書き出す
    python apps/tomato_harvest/harvest.py --no-video   # 結果だけ(速い)
    python apps/tomato_harvest/harvest.py --threshold 0    # 「熟した」の基準を 0 にすると、オレンジの実も採る
"""
import argparse
import os
import sys
from pathlib import Path

if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")

import cv2
import mink
import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "so101_twin"))
sys.path.insert(0, str(HERE))
import pick_place as twin  # noqa: E402  (計算用の SO-101 モデルと逆運動学)
import scene  # noqa: E402
from sim_so101 import MOTORS, SimSO101, SimSO101Config  # noqa: E402

OUTPUT_DIR = HERE / "output"
REST = [-60, -100, 100, 0, 0, 30]     # 待機姿勢 [度]。カメラの視界から外れる
GRIP_OPEN, GRIP_CLOSED = 55.0, 2.0
PITCH = np.deg2rad(20)               # 手先を 20° 下に向けて近づく
APPROACH = 0.07                      # 実の手前 7 cm で一度止まる


class TomatoSO101(SimSO101):
    """SO-101 のシミュレーター(LeRobot と同じ口)に、ミニトマトの鉢植えとかごを置いたもの。"""

    def __init__(self, config):
        self.config = config
        self.id = config.id
        self.model = scene.build()
        self.data = mujoco.MjData(self.model)
        self.n_sub = max(1, int(round(1 / (config.fps * self.model.opt.timestep))))
        self.qadr = [self.model.joint(m).qposadr[0] for m in MOTORS]
        self.act = [self.model.actuator(m).id for m in MOTORS]
        self.grip_range = tuple(self.model.joint("gripper").range)
        self._renderers = {}
        self._connected = False

    def connect(self, calibrate=True):
        super().connect(calibrate)
        q = np.deg2rad(REST)
        self.data.qpos[self.qadr] = q
        self.data.ctrl[self.act] = q
        mujoco.mj_forward(self.model, self.data)

    # ---- シミュレーターだけの機能(実機では、ハサミのサーボや人の目で置き換える) ----
    def cut(self, name):
        """へたを切る: 茎と実をつなぐ拘束を外す。"""
        self.data.eq_active[self.model.equality(f"{name}_stem").id] = 0

    def attached(self, name):
        return bool(self.data.eq_active[self.model.equality(f"{name}_stem").id])

    def fruit_pos(self, name):
        return self.data.body(name).xpos.copy()

    def in_basket(self, name):
        p = self.fruit_pos(name)
        return abs(p[0] - scene.BASKET[0]) < scene.BASKET_HALF and abs(p[1] - scene.BASKET[1]) < scene.BASKET_HALF and p[2] < scene.BASKET_H


class Eye:
    """鉢植えを見る RGB-D カメラ(実機なら RealSense などの深度カメラ)。"""

    def __init__(self, model, size=(360, 480)):
        self.m, self.cam = model, model.camera("eye").id
        self.h, self.w = size
        self.rgb = mujoco.Renderer(model, height=self.h, width=self.w)
        self.depth = mujoco.Renderer(model, height=self.h, width=self.w)
        self.depth.enable_depth_rendering()
        self.f = (self.h / 2) / np.tan(np.deg2rad(model.cam_fovy[self.cam]) / 2)

    def look(self, data, threshold):
        self.rgb.update_scene(data, camera=self.cam)
        self.depth.update_scene(data, camera=self.cam)
        rgb, depth = self.rgb.render().copy(), self.depth.render().copy()
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        # 実の候補: 赤〜オレンジの鮮やかな画素(緑の実は葉・茎と同じ色なので、ここでは拾わない)
        #   - 彩度 185 以上: 鉢(素焼きの茶色)は彩度が低いので外れる。鉢の前にぶら下がった実が、鉢とくっつかない
        #   - 色相 20 まで: アーム(黄色)は外れる
        warm = cv2.inRange(hsv, (0, 185, 60), (20, 255, 255)) | cv2.inRange(hsv, (170, 185, 60), (180, 255, 255))
        red = cv2.inRange(hsv, (0, 185, 60), (8, 255, 255)) | cv2.inRange(hsv, (170, 185, 60), (180, 255, 255))
        warm = cv2.morphologyEx(warm, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, labels, stats, cents = cv2.connectedComponentsWithStats(warm)
        found, view = [], cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        R, t = data.cam_xmat[self.cam].reshape(3, 3), data.cam_xpos[self.cam]
        for k in range(1, n):
            if stats[k, cv2.CC_STAT_AREA] < 30:
                continue
            mask = labels == k
            ripeness = float((red[mask] > 0).mean())          # 赤い画素の割合 = 熟し具合
            u, v = cents[k]
            d = float(np.median(depth[mask]))
            p_cam = np.array([(u + 0.5 - self.w / 2) * d / self.f, -(v + 0.5 - self.h / 2) * d / self.f, -d])
            ray = p_cam / np.linalg.norm(p_cam)
            center = t + R @ (p_cam + ray * scene.FRUIT_R)   # 見えているのは表面なので、半径の分だけ奥へ
            # 形と場所で絞る: 丸くないもの、鉢植えのまわり(作業する範囲)にないものは実ではない
            x0, y0, w0, h0 = stats[k, :4]
            roundness = stats[k, cv2.CC_STAT_AREA] / (np.pi * (max(w0, h0) / 2) ** 2)
            in_area = 0.18 < center[0] < 0.40 and abs(center[1]) < 0.15 and 0.05 < center[2] < 0.30
            if roundness < 0.55 or not in_area:
                continue
            ok = ripeness >= threshold
            found.append(dict(pos=center, ripeness=ripeness, harvest=ok, px=(int(u), int(v))))
            cv2.rectangle(view, (x0 - 3, y0 - 3), (x0 + w0 + 3, y0 + h0 + 3), (255, 255, 255) if ok else (0, 200, 255), 1)
            put(view, f"{ripeness * 100:.0f}% {'PICK' if ok else 'wait'}", (x0 + w0 + 5, y0 + h0 // 2 + 4), 0.42)
        put(view, "RGB-D camera  (white=ripe, orange=not yet)", (8, 18), 0.45)
        return found, view

    def close(self):
        self.rgb.close()
        self.depth.close()


def put(img, text, org, scale):
    for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        cv2.putText(img, text, (org[0] + dx, org[1] + dy), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)


class Mover:
    """手先の目標(位置・向き・指)を、なめらかに補間して SO-101 に送る(so101_twin と同じやり方)。"""

    def __init__(self, robot, record=None):
        self.robot, self.record = robot, record
        self.pl = twin.Planner()
        # 収穫では位置のずれのほうが困る(実を指の奥に入れたい)ので、向きより位置を優先する
        self.pl.hand = mink.FrameTask("gripperframe", "site", position_cost=1.0, orientation_cost=0.1)
        self.dt = 1.0 / robot.config.fps
        self.sync()

    def sync(self):
        obs = self.robot.get_observation()
        self.pl.sync(obs)
        self.grip = obs["gripper.pos"]

    def move(self, pos, rot, grip, dur, label=""):
        p0, r0, g0 = self.pl.tcp(), self.pl.tcp_rot(), self.grip
        delta = (r0.inverse() @ rot).log()
        n = max(1, int(round(dur / self.dt)))
        for k in range(1, n + 1):
            u = twin.smooth(k / n)
            body = self.pl.solve(p0 + (np.asarray(pos) - p0) * u, r0 @ mink.SO3.exp(delta * u), self.dt)
            g = g0 + (grip - g0) * u
            action = {f"{m}.pos": float(v) for m, v in zip(MOTORS[:5], body)}
            action["gripper.pos"] = float(g)
            self.robot.send_action(action)
            if self.record:
                self.record(label)
        self.grip = grip

    def joints(self, deg, grip, dur, label=""):
        """関節角度で待機姿勢へ戻る。"""
        obs = self.robot.get_observation()
        q0 = np.array([obs[f"{m}.pos"] for m in MOTORS])
        q1 = np.array(deg, float)
        q1[-1] = grip
        n = max(1, int(round(dur / self.dt)))
        for k in range(1, n + 1):
            q = q0 + (q1 - q0) * twin.smooth(k / n)
            self.robot.send_action({f"{m}.pos": float(v) for m, v in zip(MOTORS, q)})
            if self.record:
                self.record(label)
        self.sync()


def approach_rot(yaw):
    return mink.SO3.from_z_radians(yaw) @ mink.SO3.from_y_radians(PITCH)


def harvest_one(mv, robot, target, names_pos):
    p = target["pos"]
    yaw = np.arctan2(p[1], p[0])
    rot = approach_rot(yaw)
    fwd = rot.as_matrix()[:, 0]                    # 手先の向き(指の先の方向)
    pre = p - APPROACH * fwd
    grasp = p + 0.014 * fwd                        # 実を指の奥まで入れる(指先でつまむと、運ぶ途中で落ちる)
    mv.move(pre + [0, 0, 0.02], rot, GRIP_OPEN, 1.6, "4. approach (stop in front)")
    mv.move(pre, rot, GRIP_OPEN, 0.6, "4. approach (stop in front)")
    mv.move(grasp, rot, GRIP_OPEN, 1.0, "4. move in")
    mv.move(grasp, rot, GRIP_CLOSED, 0.7, "5. close fingers")
    # いちばん近い実の拘束を外す(=へたを切る)。実機では、手先に付けたハサミや、ひねりで切る
    name = min(names_pos, key=lambda n: np.linalg.norm(robot.fruit_pos(n) - p))
    robot.cut(name)
    mv.move(grasp, rot, GRIP_CLOSED, 0.3, "5. cut the stem")
    mv.move(pre, rot, GRIP_CLOSED, 1.2, "6. pull out")
    mv.move(pre + [0, 0, 0.04], rot, GRIP_CLOSED, 0.8, "6. pull out")
    # かごまでは、手首の向きを大きく変えずに運ぶ(向きを急に変えると、遠心力で実が抜ける)
    bx, by = scene.BASKET
    carry = approach_rot(np.arctan2(by, bx))
    back = carry.as_matrix()[:, 0] * 0.03
    mv.move(np.array([bx, by, 0.13]) - back, carry, GRIP_CLOSED, 2.8, "6. to the basket")
    mv.move(np.array([bx, by, 0.085]) - back, carry, GRIP_CLOSED, 0.6, "6. to the basket")
    mv.move(np.array([bx, by, 0.085]) - back, carry, GRIP_OPEN, 0.5, "6. release")
    mv.joints(REST, GRIP_OPEN, 1.5, "1. back to rest")
    return name


def run(threshold=0.7, video=False):
    cams = {"front": (360, 480)} if video else {}
    robot = TomatoSO101(SimSO101Config(cameras=cams, max_relative_target=10.0))
    robot.connect()
    eye = Eye(robot.model)
    frames, view = [], {"eye": cv2.cvtColor(eye.look(robot.data, threshold)[1], cv2.COLOR_BGR2RGB)}
    k = [0]

    def record(label):
        k[0] += 1
        if not video or k[0] % 3:
            return
        img = np.ascontiguousarray(robot.get_observation()["front"])
        put(img, label, (8, 18), 0.5)
        frames.append(np.hstack([img, view["eye"]]))

    mv = Mover(robot, record)
    mv.joints(REST, GRIP_OPEN, 0.8, "1. look")
    names = [n for n, *_ in scene.FRUITS]
    truth = {n: r for n, *_, r in scene.FRUITS}
    picked, first_view = [], None
    for attempt in range(8):
        mv.joints(REST, GRIP_OPEN, 0.4, "1. look")
        found, img = eye.look(robot.data, threshold)
        view["eye"] = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        if first_view is None:
            first_view = img
            print(f"1〜3. 見つけた実: {len(found)} 個(赤〜オレンジの実だけ。緑の実は葉と区別しないので数えない)")
            for f in sorted(found, key=lambda f: -f["ripeness"]):
                near = min(names, key=lambda n: np.linalg.norm(robot.fruit_pos(n) - f["pos"]))
                err = np.linalg.norm(robot.fruit_pos(near) - f["pos"]) * 1000
                print(f"     赤さ {f['ripeness'] * 100:3.0f}% → {'収穫する' if f['harvest'] else 'まだ'}"
                      f"(本当の熟し具合 {truth[near]:.2f}、位置の誤差 {err:.1f} mm)")
        todo = [f for f in found if f["harvest"]]
        if not todo:
            break
        target = max(todo, key=lambda f: f["pos"][2])          # 上の実から採る(下の実に当たらないように)
        on_plant = [n for n in names if robot.attached(n)]
        name = harvest_one(mv, robot, target, on_plant)
        picked.append(name)
        print(f"   {attempt + 1} 回目: {name}(熟し具合 {truth[name]:.2f})を収穫 → {'かごに入った' if robot.in_basket(name) else 'かごに入らなかった'}")
    mv.joints(REST, GRIP_OPEN, 1.0, "done")
    eye.close()
    result = {n: dict(ripeness=truth[n], picked=not robot.attached(n), basket=robot.in_basket(n),
                      moved=np.linalg.norm(robot.fruit_pos(n) - np.array(next(f[1:4] for f in scene.FRUITS if f[0] == n))) * 1000)
              for n in names}
    robot.disconnect()
    return result, frames, first_view


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--threshold", type=float, default=0.7, help="「熟した」とみなす赤い画素の割合")
    ap.add_argument("--no-video", action="store_true")
    args = ap.parse_args()
    OUTPUT_DIR.mkdir(exist_ok=True)
    result, frames, first_view = run(args.threshold, video=not args.no_video)
    print("結果:")
    for n, r in result.items():
        state = "かごの中" if r["basket"] else ("採ったが、かごの外" if r["picked"] else "木に残っている")
        print(f"   {n}: 熟し具合 {r['ripeness']:.2f} → {state}" + (f"(木に残った実が {r['moved']:.0f} mm 動いた)" if not r["picked"] and r["moved"] > 5 else ""))
    if first_view is not None:
        cv2.imwrite(str(OUTPUT_DIR / "detection.png"), first_view)
    if frames:
        import subprocess

        import imageio.v2 as imageio
        import imageio_ffmpeg
        mp4 = OUTPUT_DIR / "tomato_harvest.mp4"
        imageio.mimsave(mp4, frames, fps=13)
        subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-loglevel", "error", "-y", "-i", str(mp4), "-vf",
                        "fps=8,scale=640:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=64[p];[b][p]paletteuse=dither=bayer",
                        str(OUTPUT_DIR / "tomato_harvest.gif")], check=True)
        print(f"動画を保存: {mp4}")
    ripe = [n for n, r in result.items() if r["ripeness"] >= 0.8]
    assert all(result[n]["basket"] for n in ripe), "熟した実をかごに入れられなかった"
    if args.threshold >= 0.7:
        assert not any(result[n]["picked"] for n, r in result.items() if r["ripeness"] < 0.6), "熟していない実を採ってしまった"


if __name__ == "__main__":
    main()
