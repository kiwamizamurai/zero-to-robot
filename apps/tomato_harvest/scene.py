"""ミニトマトの鉢植えと SO-101 の場面を作る。"""
import numpy as np
import mujoco

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "so101_twin"))
from sim_so101 import SO101_SCENE  # noqa: E402

FRUIT_R = 0.013  # ミニトマトの半径 [m](直径 2.6 cm)
POT = (0.345, 0.0)
# (名前, 位置 x, y, z, 熟し具合 0=緑 〜 1=真っ赤)
FRUITS = [
    ("t1", 0.280, 0.050, 0.110, 1.00),
    ("t2", 0.270, -0.030, 0.140, 0.95),
    ("t3", 0.285, 0.010, 0.085, 0.15),
    ("t4", 0.280, -0.065, 0.095, 0.90),
    ("t5", 0.275, 0.075, 0.150, 0.55),
    ("t6", 0.290, 0.035, 0.170, 0.10),
]
BASKET = (0.05, -0.22)
BASKET_HALF, BASKET_H = 0.07, 0.04


def ripeness_rgba(r):
    """熟し具合を色にする: 緑 → オレンジ → 赤。"""
    green, orange, red = np.array([0.35, 0.65, 0.2]), np.array([0.95, 0.55, 0.12]), np.array([0.85, 0.12, 0.08])
    c = green + (orange - green) * (r / 0.5) if r < 0.5 else orange + (red - orange) * ((r - 0.5) / 0.5)
    return [*c, 1.0]


def build(fruits=FRUITS):
    spec = mujoco.MjSpec.from_file(str(SO101_SCENE))
    spec.material("groundplane").reflectance = 0.0
    tex = spec.texture("groundplane")
    tex.rgb1, tex.rgb2, tex.markrgb = [0.80, 0.78, 0.72], [0.72, 0.70, 0.64], [0.6, 0.6, 0.55]
    # 鉢・土・茎・葉(葉は見た目だけ)
    plant = spec.worldbody.add_body(name="plant", pos=[*POT, 0])
    plant.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.05, 0.03, 0], pos=[0, 0, 0.03], rgba=[0.7, 0.4, 0.25, 1])
    plant.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.046, 0.002, 0], pos=[0, 0, 0.061], rgba=[0.3, 0.22, 0.15, 1])
    plant.add_geom(name="stem", type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.006, 0.11, 0], pos=[0, 0, 0.17], rgba=[0.25, 0.5, 0.18, 1])
    rng = np.random.default_rng(3)
    for k in range(14):
        a, z = rng.uniform(0, 2 * np.pi), rng.uniform(0.10, 0.27)
        if np.cos(a) < -0.2 and z < 0.2:  # ロボット側の手前は葉を少なくして、実が見えるようにする
            a += np.pi
        r = 0.035
        plant.add_geom(type=mujoco.mjtGeom.mjGEOM_ELLIPSOID, size=[0.03, 0.016, 0.003],
                       pos=[r * np.cos(a), r * np.sin(a), z], euler=[0.3, -0.4, a],
                       rgba=[0.22, 0.55, 0.2, 1], contype=0, conaffinity=0)
    # 実: へた(果柄)で茎につながっている。つながりは「溶接」の拘束で表し、切ると外れる
    for name, x, y, z, r in fruits:
        b = spec.worldbody.add_body(name=name, pos=[x, y, z])
        b.add_freejoint()
        b.add_geom(name=f"{name}_g", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[FRUIT_R, 0, 0], rgba=ripeness_rgba(r),
                   mass=0.008, friction=[1.2, 0.02, 0.001], condim=4)
        b.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.004, 0.002, 0], pos=[0, 0, FRUIT_R], rgba=[0.2, 0.45, 0.15, 1],
                   contype=0, conaffinity=0)
        stem_x = POT[0] - x
        b.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.0015, 0, 0], fromto=[0, 0, FRUIT_R, stem_x, POT[1] - y, FRUIT_R + 0.02],
                   rgba=[0.25, 0.5, 0.18, 1], contype=0, conaffinity=0)
        eq = spec.add_equality(name=f"{name}_stem", type=mujoco.mjtEq.mjEQ_WELD)
        eq.name1, eq.name2 = name, "plant"
        eq.objtype = mujoco.mjtObj.mjOBJ_BODY
        # 基準点(アンカー)を実の位置に置く。鉢の根元のままだと、実が根元を中心に振り子のように回って落ちる
        eq.data[:11] = [x - POT[0], y - POT[1], z, 0, 0, 0, 0, 0, 0, 0, 1]
    # かご
    basket = spec.worldbody.add_body(name="basket", pos=[*BASKET, 0])
    basket.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, pos=[0, 0, 0.002], size=[BASKET_HALF, BASKET_HALF, 0.002], rgba=[0.75, 0.6, 0.35, 1])
    h = BASKET_H / 2
    for dx, dy, sx, sy in [(BASKET_HALF, 0, 0.003, BASKET_HALF), (-BASKET_HALF, 0, 0.003, BASKET_HALF),
                           (0, BASKET_HALF, BASKET_HALF, 0.003), (0, -BASKET_HALF, BASKET_HALF, 0.003)]:
        basket.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, pos=[dx, dy, h], size=[sx, sy, h], rgba=[0.75, 0.6, 0.35, 1])
    # カメラ: 実を見る RGB-D カメラ(ロボットの斜め後ろ上)と、全体を撮るカメラ
    spec.worldbody.add_body(name="plant_center", pos=[0.285, 0.0, 0.13])
    cam = spec.worldbody.add_camera(name="eye", pos=[0.06, -0.20, 0.26])
    cam.mode = mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY
    cam.targetbody = "plant_center"
    cam.fovy = 55
    spec.worldbody.add_body(name="look_at", pos=[0.18, -0.05, 0.10])
    cam2 = spec.worldbody.add_camera(name="front", pos=[0.62, -0.48, 0.42])
    cam2.mode = mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY
    cam2.targetbody = "look_at"
    return spec.compile()
