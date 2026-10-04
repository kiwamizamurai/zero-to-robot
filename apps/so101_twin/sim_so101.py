"""シミュレーターの SO-101。LeRobot の実機クラス(lerobot.robots.so_follower.SO101Follower)と同じ使い方ができる。

同じにしたもの:
- connect() / disconnect() / is_connected / observation_features / action_features
- get_observation() は {"shoulder_pan.pos": 度, ..., "gripper.pos": 0〜100} と、カメラ名ごとの画像を返す
- send_action(action) は同じ形の辞書を受け取り、max_relative_target で 1 回に動かす量を制限して、実際に送った値を返す

モデルは MuJoCo Menagerie の robotstudio_so101(STS3215 サーボの PD ゲイン・最大トルク・摩擦が実機に合わせて調整済み)。

実機との対応で仮定していること(実機が届いたら確かめる):
- 角度の 0 度: LeRobot の校正は「各関節の可動域の真ん中」を 0 にする。Menagerie のモデルも同じ校正(new_calib)由来なので、
  度 = ラジアン × 180/π でそろうとみなしている
- グリッパー: 0 が閉、100 が開。モデルの可動域(-10°〜100°)を 0〜100 に線形に対応させている
- 位置センサー: STS3215 は 1 回転 4096 段階(約 0.088°)。読み取りをその刻みに丸める
"""
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from robot_descriptions import panda_mj_description  # Menagerie の置き場所を借りる(初回に自動ダウンロード)

MENAGERIE = Path(panda_mj_description.REPOSITORY_PATH)
SO101_SCENE = MENAGERIE / "robotstudio_so101" / "scene.xml"
MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
COUNTS_PER_TURN = 4096


@dataclass
class SimSO101Config:
    id: str = "sim_follower"
    fps: int = 40  # 制御の周期。物理は 5 ms 刻みなので 40 Hz なら 1 回の指令で 5 ステップ進む
    max_relative_target: float | None = None  # 1 回の指令で動かせる量の上限 [度]。LeRobot と同じ意味
    cameras: dict[str, tuple[int, int]] = field(default_factory=dict)  # 名前: (高さ, 幅)。"front", "wrist" が使える
    cube_xy: tuple[float, float] = (0.24, 0.10)
    cube_yaw: float = 0.0
    tray_xy: tuple[float, float] = (0.18, -0.18)
    quantize: bool = True


CUBE_HALF = 0.015
TRAY_HALF, TRAY_WALL, TRAY_H = 0.05, 0.004, 0.025


def build_scene(cfg: SimSO101Config):
    spec = mujoco.MjSpec.from_file(str(SO101_SCENE))
    cube = spec.worldbody.add_body(name="cube", pos=[*cfg.cube_xy, CUBE_HALF],
                                   quat=[np.cos(cfg.cube_yaw / 2), 0, 0, np.sin(cfg.cube_yaw / 2)])
    cube.add_freejoint()
    cube.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[CUBE_HALF] * 3, rgba=[0.9, 0.25, 0.2, 1],
                  mass=0.02, friction=[1.5, 0.01, 0.001], condim=4)
    tray = spec.worldbody.add_body(name="tray", pos=[*cfg.tray_xy, 0])
    tray.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, pos=[0, 0, 0.002], size=[TRAY_HALF, TRAY_HALF, 0.002],
                  rgba=[0.3, 0.55, 0.85, 1])
    h = TRAY_H / 2
    for dx, dy, sx, sy in [(TRAY_HALF, 0, TRAY_WALL, TRAY_HALF), (-TRAY_HALF, 0, TRAY_WALL, TRAY_HALF),
                           (0, TRAY_HALF, TRAY_HALF, TRAY_WALL), (0, -TRAY_HALF, TRAY_HALF, TRAY_WALL)]:
        tray.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, pos=[dx, dy, h], size=[sx, sy, h], rgba=[0.3, 0.55, 0.85, 1])
    spec.worldbody.add_body(name="look_at", pos=[0.15, -0.03, 0.10])
    cam = spec.worldbody.add_camera(name="front", pos=[0.68, 0.42, 0.45])
    cam.mode = mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY
    cam.targetbody = "look_at"
    spec.material("groundplane").reflectance = 0.0
    return spec.compile()


class SimSO101:
    name = "sim_so101"

    def __init__(self, config: SimSO101Config):
        self.config = config
        self.id = config.id
        self.model = build_scene(config)
        self.data = mujoco.MjData(self.model)
        self.n_sub = max(1, int(round(1 / (config.fps * self.model.opt.timestep))))
        self.qadr = [self.model.joint(m).qposadr[0] for m in MOTORS]
        self.act = [self.model.actuator(m).id for m in MOTORS]
        lo, hi = self.model.joint("gripper").range
        self.grip_range = (lo, hi)
        self._renderers = {}
        self._connected = False

    # ---------- LeRobot と同じ口 ----------
    @property
    def observation_features(self):
        feats = {f"{m}.pos": float for m in MOTORS}
        feats.update({name: (h, w, 3) for name, (h, w) in self.config.cameras.items()})
        return feats

    @property
    def action_features(self):
        return {f"{m}.pos": float for m in MOTORS}

    @property
    def is_connected(self):
        return self._connected

    @property
    def is_calibrated(self):
        return True

    def connect(self, calibrate: bool = True):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.data.ctrl[self.act] = self.data.qpos[self.qadr]
        for name, (h, w) in self.config.cameras.items():
            self._renderers[name] = (mujoco.Renderer(self.model, height=h, width=w),
                                     "wrist_cam" if name == "wrist" else name)
        self._connected = True

    def calibrate(self):
        pass  # シミュレーターでは要らない(実機では connect() のときに対話で行う)

    def configure(self):
        pass

    def disconnect(self):
        for r, _ in self._renderers.values():
            r.close()
        self._renderers = {}
        self._connected = False

    def get_observation(self):
        q = self.data.qpos[self.qadr].copy()
        if self.config.quantize:  # エンコーダの分解能に丸める
            step = 2 * np.pi / COUNTS_PER_TURN
            q = np.round(q / step) * step
        obs = {f"{m}.pos": float(v) for m, v in zip(MOTORS, self._to_user(q))}
        for name, (r, cam) in self._renderers.items():
            r.update_scene(self.data, camera=cam)
            obs[name] = r.render().copy()
        return obs

    def send_action(self, action):
        goal = np.array([action[f"{m}.pos"] for m in MOTORS], dtype=float)
        present = self._to_user(self.data.qpos[self.qadr])
        if self.config.max_relative_target is not None:  # LeRobot の ensure_safe_goal_position と同じ
            cap = self.config.max_relative_target
            goal = present + np.clip(goal - present, -cap, cap)
        rad = self._to_rad(goal)
        lo, hi = self.model.actuator_ctrlrange[self.act].T
        self.data.ctrl[self.act] = np.clip(rad, lo, hi)
        for _ in range(self.n_sub):
            mujoco.mj_step(self.model, self.data)
        return {f"{m}.pos": float(v) for m, v in zip(MOTORS, goal)}

    # ---------- シミュレーターだけの便利機能(実機にはない) ----------
    def cube_pose(self):
        b = self.data.body("cube")
        return b.xpos.copy(), 2 * np.arctan2(b.xquat[3], b.xquat[0])

    def cube_in_tray(self):
        p, _ = self.cube_pose()
        t = self.config.tray_xy
        return abs(p[0] - t[0]) < TRAY_HALF and abs(p[1] - t[1]) < TRAY_HALF and p[2] < TRAY_H

    def joint_torques(self):
        return self.data.actuator_force[self.act].copy()

    # ---------- 単位の変換(LeRobot の use_degrees=True に合わせる) ----------
    def _to_user(self, rad):
        out = np.rad2deg(rad)
        lo, hi = self.grip_range
        out[-1] = (rad[-1] - lo) / (hi - lo) * 100
        return out

    def _to_rad(self, user):
        out = np.deg2rad(user)
        lo, hi = self.grip_range
        out[-1] = lo + user[-1] / 100 * (hi - lo)
        return out
