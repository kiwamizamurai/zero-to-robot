"""本物のロボットアーム(Franka Panda)で、ブロックを箱に片付ける。

apps/tidy_arm のブラウザ版と同じ作業を、実務で使うライブラリだけで組み立てる。

- ロボットのモデル: MuJoCo Menagerie の Franka Panda(robot_descriptions が初回に自動でダウンロード)
- 場面づくり: mujoco.MjSpec でブロックと箱を足す
- 角度を計算(逆運動学): mink.solve_ik
- なめらかに動かす: 5次多項式(05_planning と同じ)
- モーター: Panda の各関節は MuJoCo の位置サーボ。data.ctrl に目標角度を入れるだけ
- 物理: mujoco.mj_step。ブロックは指との摩擦だけで持ち上がる(つかみ損ねることもある)

実行:
    python apps/panda_pick_place/pick_place.py            # output/pick_place.mp4 と .gif を書き出す
    python apps/panda_pick_place/pick_place.py --viewer   # 3D ビューアで見る(macOS は mjpython で実行)
"""
import argparse
import os
import sys
from pathlib import Path

# 画面のない Linux(サーバーや CI)では、動画の書き出しに EGL を使う。mujoco の import より前に決める必要がある
if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")

import mink
import mujoco
import numpy as np
from robot_descriptions import panda_mj_description

OUTPUT_DIR = Path(__file__).parent / "output"
SCENE = Path(panda_mj_description.MJCF_PATH).parent / "scene.xml"

BLOCK = 0.02  # ブロックの半辺 [m](4 cm 角)
BLOCKS = [("red", (0.50, 0.25), (0.85, 0.25, 0.20, 1)),
          ("blue", (0.62, 0.08), (0.20, 0.50, 0.85, 1)),
          ("green", (0.45, -0.02), (0.25, 0.65, 0.35, 1))]
BOX_CENTER = (0.40, -0.38)
BOX_HALF, BOX_WALL, BOX_HEIGHT = 0.10, 0.008, 0.05
DROP_SPOTS = [(-0.04, -0.04), (0.04, 0.04), (0.04, -0.04)]  # 箱の中で少しずつずらして落とす

GRIP_OPEN, GRIP_CLOSED = 255.0, 0.0
CAM_POS = (1.45, 0.95, 1.05)
SAFE_Z = 0.25


def build_spec(blocks=BLOCKS):
    """Menagerie の Panda の場面に、手先の目印(tcp)・ブロック・箱を足す(compile 前の MjSpec を返す)。"""
    spec = mujoco.MjSpec.from_file(str(SCENE))
    # 指先の間に目印を置く。IK はこの点を目標へ動かす
    spec.body("hand").add_site(name="tcp", pos=[0, 0, 0.1034])

    for name, (x, y), rgba in blocks:
        body = spec.worldbody.add_body(name=f"block_{name}", pos=[x, y, BLOCK])
        body.add_freejoint()
        body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[BLOCK] * 3, rgba=rgba,
                      mass=0.05, friction=[1.5, 0.01, 0.001])

    box = spec.worldbody.add_body(name="box", pos=[*BOX_CENTER, 0])
    h = BOX_HEIGHT / 2
    for dx, dy, sx, sy in [(BOX_HALF, 0, BOX_WALL, BOX_HALF), (-BOX_HALF, 0, BOX_WALL, BOX_HALF),
                           (0, BOX_HALF, BOX_HALF, BOX_WALL), (0, -BOX_HALF, BOX_HALF, BOX_WALL)]:
        box.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, pos=[dx, dy, h], size=[sx, sy, h], rgba=[0.55, 0.42, 0.3, 1])

    # 動画用のカメラ。ロボット・ブロック・箱がまとめて映る点を狙う
    spec.worldbody.add_body(name="look_at", pos=[0.3, -0.05, 0.28])
    cam = spec.worldbody.add_camera(name="front", pos=[CAM_POS[0], CAM_POS[1], CAM_POS[2]])
    cam.mode = mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY
    cam.targetbody = "look_at"
    spec.material("groundplane").reflectance = 0.0  # 床の映り込みを消して見やすくする
    return spec


def build_model():
    return build_spec().compile()


def smooth(t):
    """5次多項式の時間スケーリング: ゆっくり動き出してゆっくり止まる。"""
    t = np.clip(t, 0.0, 1.0)
    return t**3 * (10 - 15 * t + 6 * t * t)


class Robot:
    def __init__(self, model, data):
        self.model, self.data = model, data
        # ホーム姿勢(Menagerie のキーフレーム)はロボットの 9 個の関節分だけ使う。ブロックは置いた場所のまま
        home = model.key("home")
        data.qpos[:9] = home.qpos[:9]
        data.ctrl[:] = home.ctrl
        mujoco.mj_forward(model, data)
        # 2. 角度を計算: 手先(tcp)を目標の位置・向きへ。7 関節で 1 つ余るので、
        #    余った自由度は「ホーム姿勢に近く」という弱い目標で使う(01 の冗長マニピュレータの零空間)
        self.cfg = mink.Configuration(model)
        self.cfg.update(data.qpos)
        self.hand_task = mink.FrameTask("tcp", "site", position_cost=1.0, orientation_cost=1.0)
        self.posture = mink.PostureTask(model, cost=1e-2)
        self.posture.set_target_from_configuration(self.cfg)
        self.limits = [mink.ConfigurationLimit(model)]
        # 手先はずっと真下を向ける。ホーム姿勢の手先の向きをそのまま使う
        self.down = self.cfg.get_transform_frame_to_world("tcp", "site").rotation()
        self.yaw = 0.0  # 手先を真下に向けたまま、鉛直軸まわりに回す角度 [rad]
        self.grip = GRIP_OPEN
        self.frames, self.log = [], []

    def tcp(self):
        return self.data.site("tcp").xpos.copy()

    def step(self, target_pos, n_physics=10, record=None):
        """IK を 1 回解いて関節の目標角度を出し、物理を n_physics ステップ進める。"""
        dt = self.model.opt.timestep * n_physics
        rot = mink.SO3.from_z_radians(self.yaw) @ self.down
        self.hand_task.set_target(mink.SE3.from_rotation_and_translation(rot, np.asarray(target_pos)))
        vel = mink.solve_ik(self.cfg, [self.hand_task, self.posture], dt, solver="daqp", damping=1e-3, limits=self.limits)
        self.cfg.integrate_inplace(vel, dt)
        # 3. 動かす: 各関節の位置サーボに目標角度を渡す。指は開閉の指令だけ
        self.data.ctrl[:7] = self.cfg.q[:7]
        self.data.ctrl[7] = self.grip
        for _ in range(n_physics):
            mujoco.mj_step(self.model, self.data)
        if record:
            record(self.data)

    def move(self, goal, duration, record=None):
        start = self.cfg.get_transform_frame_to_world("tcp", "site").translation()
        steps = int(duration / (self.model.opt.timestep * 10))
        for k in range(1, steps + 1):
            u = smooth(k / steps)
            self.step(start + (np.asarray(goal) - start) * u, record=record)

    def gripper(self, value, duration, record=None):
        self.grip = value
        hold = self.cfg.get_transform_frame_to_world("tcp", "site").translation()
        for _ in range(int(duration / (self.model.opt.timestep * 10))):
            self.step(hold, record=record)


def block_pos(data, name):
    return data.body(f"block_{name}").xpos.copy()


def pick_and_place(robot, name, drop, record):
    d = robot.data
    x, y, _ = block_pos(d, name)                       # 1. 見つける(ここではシミュレーターから位置を直接読む)
    robot.log.append(f"{name}: 位置 ({x:.2f}, {y:.2f}) を見つけた")
    robot.move([x, y, SAFE_Z], 1.6, record)            # 真上へ
    robot.move([x, y, BLOCK + 0.002], 1.0, record)      # 4. つかむ: 真下に降りて
    robot.gripper(GRIP_CLOSED, 0.6, record)             #    指を閉じる
    robot.move([x, y, SAFE_Z], 1.0, record)             # 5. 運ぶ: 持ち上げて
    lifted = block_pos(d, name)[2] > 0.1
    robot.log.append(f"{name}: 持ち上げ{'成功' if lifted else '失敗(すべり落ちた)'} 高さ {block_pos(d, name)[2]:.3f} m")
    bx, by = BOX_CENTER[0] + drop[0], BOX_CENTER[1] + drop[1]
    robot.move([bx, by, SAFE_Z], 1.6, record)           #    箱の上へ
    robot.move([bx, by, BOX_HEIGHT + 0.06], 0.8, record)
    robot.gripper(GRIP_OPEN, 0.5, record)               # 6. はなす
    robot.move([bx, by, SAFE_Z], 0.8, record)


def in_box(p):
    return abs(p[0] - BOX_CENTER[0]) < BOX_HALF and abs(p[1] - BOX_CENTER[1]) < BOX_HALF and p[2] < BOX_HEIGHT


def run(record_video=True):
    model = build_model()
    data = mujoco.MjData(model)
    robot = Robot(model, data)

    renderer, frames = None, []
    if record_video:
        renderer = mujoco.Renderer(model, height=352, width=480)
        cam_id = model.camera("front").id
        fps, counter = 20, [0]
        every = int(round(1 / (fps * model.opt.timestep * 10)))

        def record(d):
            counter[0] += 1
            if counter[0] % every == 0:
                renderer.update_scene(d, camera=cam_id)
                frames.append(renderer.render().copy())
    else:
        record = None

    robot.gripper(GRIP_OPEN, 0.5, record)
    for (name, _, _), drop in zip(BLOCKS, DROP_SPOTS):
        pick_and_place(robot, name, drop, record)
    robot.move(robot.cfg.get_transform_frame_to_world("tcp", "site").translation() + [0.1, 0.25, 0.1], 1.5, record)
    robot.gripper(GRIP_OPEN, 1.0, record)  # ブロックが落ち着くまで待つ

    print("\n".join(robot.log))
    results = {name: in_box(block_pos(data, name)) for name, _, _ in BLOCKS}
    for name, ok in results.items():
        p = block_pos(data, name)
        print(f"{name:5s}: ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:.3f}) {'箱の中' if ok else '箱の外'}")
    print(f"シミュレーション時間 {data.time:.1f} 秒")

    if record_video:
        renderer.close()
        import imageio.v2 as imageio
        OUTPUT_DIR.mkdir(exist_ok=True)
        mp4 = OUTPUT_DIR / "pick_place.mp4"
        imageio.mimsave(mp4, frames, fps=fps)
        # README に埋め込む軽い GIF。色数を減らしたパレットで ffmpeg に変換させる
        import subprocess

        import imageio_ffmpeg
        subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-loglevel", "error", "-y", "-i", str(mp4), "-vf",
                        "fps=8,scale=320:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=64[p];[b][p]paletteuse=dither=bayer",
                        str(OUTPUT_DIR / "pick_place.gif")], check=True)
        print(f"動画を保存: {OUTPUT_DIR / 'pick_place.mp4'}, {OUTPUT_DIR / 'pick_place.gif'}")
    return results


def run_viewer():
    import time

    import mujoco.viewer

    model = build_model()
    data = mujoco.MjData(model)
    robot = Robot(model, data)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        def record(d):
            viewer.sync()
            time.sleep(model.opt.timestep * 10)

        robot.gripper(GRIP_OPEN, 0.5, record)
        for (name, _, _), drop in zip(BLOCKS, DROP_SPOTS):
            pick_and_place(robot, name, drop, record)
        while viewer.is_running():
            robot.step(robot.tcp(), record=record)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--viewer", action="store_true", help="3D ビューアで見る(動画は書き出さない)")
    args = parser.parse_args()
    if args.viewer:
        run_viewer()
    else:
        results = run()
        assert all(results.values()), "箱に入らなかったブロックがある"
