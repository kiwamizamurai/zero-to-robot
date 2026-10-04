"""Rerunでの可視化: 01_kinematicsの2リンク平面アーム(FK)を、05_planningの
5次時間スケーリング(quintic_time_scaling)で始点から終点まで動かし、
関節位置・手先軌跡をアニメーションとして、関節角・角速度を時系列として
同じビューア上に記録する。

matplotlibは1枚の静止画(または個別のアニメーションGIF)しか作れないが、
Rerunは3D/2Dのアニメーションと時系列プロットを同じタイムラインで同期して
記録・再生できる。ここではそれを、これまでのフェーズ(01_kinematicsのFK、
05_planningの時間スケーリング)の組み合わせで示す。

出力(output/arm_trajectory.rrd)は `rerun output/arm_trajectory.rrd` で
ビューアが開ける(GUIが使える環境で)。
"""
import sys
from importlib import import_module
from pathlib import Path

import numpy as np
import rerun as rr

sys.path.insert(0, str(Path(__file__).parent.parent / "01_kinematics"))
sys.path.insert(0, str(Path(__file__).parent.parent / "05_planning"))
_ik = import_module("02_ik_gradient_descent")
_poly = import_module("01_polynomial_trajectories")

L1, L2 = _ik.l1, _ik.l2
OUTPUT_DIR = Path(__file__).parent / "output"


def fk_joints(q):
    """原点・肘・手先の3点を返す(01_kinematicsのfk(q)を関節位置まで分解したもの)。"""
    origin = np.array([0.0, 0.0])
    elbow = L1 * np.array([np.cos(q[0]), np.sin(q[0])])
    ee = elbow + L2 * np.array([np.cos(q[0] + q[1]), np.sin(q[0] + q[1])])
    return origin, elbow, ee


def main():
    OUTPUT_DIR.mkdir(exist_ok=True)
    out_path = OUTPUT_DIR / "arm_trajectory.rrd"

    rr.init("zero-to-robot-06-arm-trajectory")
    rr.save(str(out_path))

    q_start = np.array([-2.0, 1.6])
    q_end = np.array([1.0, -1.2])
    T = 3.0
    dt = 1.0 / 50.0
    n_frames = int(T / dt) + 1

    ee_trace = []
    for i in range(n_frames):
        t = i * dt
        q, qdot, _ = _poly.joint_trajectory(q_start, q_end, t, T, method="quintic")
        origin, elbow, ee = fk_joints(q)
        ee_trace.append(ee.tolist())

        rr.set_time("time", duration=t)
        rr.log("arm/link1", rr.LineStrips2D([[origin.tolist(), elbow.tolist()]], radii=0.02))
        rr.log("arm/link2", rr.LineStrips2D([[elbow.tolist(), ee.tolist()]], radii=0.02))
        rr.log(
            "arm/joints",
            rr.Points2D([origin.tolist(), elbow.tolist(), ee.tolist()], radii=0.05),
        )
        rr.log("arm/ee_trace", rr.LineStrips2D([ee_trace], radii=0.005))
        rr.log("joints/q1", rr.Scalars([q[0]]))
        rr.log("joints/q2", rr.Scalars([q[1]]))
        rr.log("joints/q1_dot", rr.Scalars([qdot[0]]))
        rr.log("joints/q2_dot", rr.Scalars([qdot[1]]))

    rr.get_global_data_recording().flush(timeout_sec=5.0)
    assert out_path.exists() and out_path.stat().st_size > 0
    print(f"Rerunの記録を書き出した: {out_path} ({out_path.stat().st_size} bytes)")
    print(f"`rerun {out_path}` でビューアを開いて確認できる。")


if __name__ == "__main__":
    main()
