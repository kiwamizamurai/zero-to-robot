"""3次多項式による経由点(via point)軌道: 複数の経由点を指定時刻・指定速度で通過する
区分3次多項式軌道の実装。各セグメントは位置・速度を両端点で指定した3次多項式で、
時間スケーリングs(t)をあらかじめ決めずに、関節の時間履歴theta(t)を直接求める。

Lynch and Park, *Modern Robotics*, 2017, 9.3節の式(9.25)-(9.29)に基づく。
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def cubic_via_segment_coeffs(p0, v0, p1, v1, dT):
    """1セグメント分の3次多項式係数(式9.26-9.29)。
    beta(dt) = a0+a1 dt+a2 dt^2+a3 dt^3, 0<=dt<=dT。"""
    a0 = p0
    a1 = v0
    a2 = (3 * p1 - 3 * p0 - 2 * v0 * dT - v1 * dT) / dT**2
    a3 = (2 * p0 + (v0 + v1) * dT - 2 * p1) / dT**3
    return a0, a1, a2, a3


def eval_cubic(coeffs, dt):
    a0, a1, a2, a3 = coeffs
    pos = a0 + a1 * dt + a2 * dt**2 + a3 * dt**3
    vel = a1 + 2 * a2 * dt + 3 * a3 * dt**2
    acc = 2 * a2 + 6 * a3 * dt
    return pos, vel, acc


def via_point_trajectory(times, positions, velocities, t):
    """複数の経由点(times[i], positions[i], velocities[i])を通る区分3次多項式軌道。
    tはスカラーまたは配列。各tがどのセグメントに属するかを判定して評価する。"""
    times = np.asarray(times, dtype=float)
    positions = np.asarray(positions, dtype=float)
    velocities = np.asarray(velocities, dtype=float)
    t = np.atleast_1d(np.asarray(t, dtype=float))
    n_seg = len(times) - 1

    pos = np.zeros_like(t)
    vel = np.zeros_like(t)
    acc = np.zeros_like(t)
    for j in range(n_seg):
        dT = times[j + 1] - times[j]
        coeffs = cubic_via_segment_coeffs(positions[j], velocities[j], positions[j + 1], velocities[j + 1], dT)
        if j == n_seg - 1:
            mask = (t >= times[j] - 1e-12) & (t <= times[j + 1] + 1e-9)
        else:
            mask = (t >= times[j] - 1e-12) & (t < times[j + 1] - 1e-12)
        dt = t[mask] - times[j]
        pos[mask], vel[mask], acc[mask] = eval_cubic(coeffs, dt)
    return pos, vel, acc


def main():
    OUTPUT_DIR = Path(__file__).parent / "output"
    OUTPUT_DIR.mkdir(exist_ok=True)

    # Modern Robotics Figure 9.7(a)/9.8 と同じ設定: (x,y)平面上の4つの経由点
    times = [0.0, 1.0, 2.0, 3.0]
    x_pos = [0.0, 0.0, 1.0, 1.0]
    x_vel = [0.0, 1.0, 0.0, 0.0]
    y_pos = [0.0, 1.0, 1.0, 0.0]
    y_vel = [0.0, 0.0, -1.0, 0.0]

    # (1) 各経由点で位置・速度が指定通りになっているか
    t_via = np.array(times)
    x_at_via, xdot_at_via, _ = via_point_trajectory(times, x_pos, x_vel, t_via)
    y_at_via, ydot_at_via, _ = via_point_trajectory(times, y_pos, y_vel, t_via)
    pos_err = max(np.max(np.abs(x_at_via - x_pos)), np.max(np.abs(y_at_via - y_pos)))
    vel_err = max(np.max(np.abs(xdot_at_via - x_vel)), np.max(np.abs(ydot_at_via - y_vel)))
    print(f"経由点での位置誤差(最大)={pos_err:.2e}, 速度誤差(最大)={vel_err:.2e}")
    assert pos_err < 1e-10 and vel_err < 1e-10

    # (2) 内部経由点(t=1,2)で速度は連続、加速度は一般に不連続であることを確認
    eps = 1e-6
    for t_mid in [1.0, 2.0]:
        _, xv_l, xa_l = via_point_trajectory(times, x_pos, x_vel, np.array([t_mid - eps]))
        _, xv_r, xa_r = via_point_trajectory(times, x_pos, x_vel, np.array([t_mid + eps]))
        vel_jump = abs(xv_l[0] - xv_r[0])
        acc_jump = abs(xa_l[0] - xa_r[0])
        print(f"t={t_mid}: x方向の速度の不連続量={vel_jump:.2e}, 加速度の不連続量={acc_jump:.4f}")
        assert vel_jump < 1e-4
    # t=1では加速度が実際にジャンプすることを具体的に確認(退化的に一致しないことを保証)
    _, _, xa_l1 = via_point_trajectory(times, x_pos, x_vel, np.array([1.0 - eps]))
    _, _, xa_r1 = via_point_trajectory(times, x_pos, x_vel, np.array([1.0 + eps]))
    assert abs(xa_l1[0] - xa_r1[0]) > 0.1, "この例では加速度が不連続であるはず"

    # (3) 退化ケース: 経由点が2つ(始点・終点)で両端の速度が0のとき、
    #     05_planning/01_polynomial_trajectories.py の3次時間スケーリングに一致する
    from importlib import import_module
    import sys

    sys.path.insert(0, str(Path(__file__).parent))
    _poly = import_module("01_polynomial_trajectories")

    T_test = 2.0
    beta_start, beta_end = 0.3, 1.7
    t_fine = np.linspace(0, T_test, 200)
    pos_via, vel_via, acc_via = via_point_trajectory([0.0, T_test], [beta_start, beta_end], [0.0, 0.0], t_fine)

    s_cubic, sdot_cubic, sddot_cubic = _poly.cubic_time_scaling(t_fine, T_test)
    pos_cubic = beta_start + s_cubic * (beta_end - beta_start)
    vel_cubic = sdot_cubic * (beta_end - beta_start)
    acc_cubic = sddot_cubic * (beta_end - beta_start)

    max_diff = max(np.max(np.abs(pos_via - pos_cubic)), np.max(np.abs(vel_via - vel_cubic)), np.max(np.abs(acc_via - acc_cubic)))
    print(f"\n2経由点(両端速度0)の場合、3次時間スケーリングとの最大差={max_diff:.2e}")
    assert max_diff < 1e-10

    print("\nすべてのチェックに合格しました。")

    fig, axes = plt.subplots(1, 2, figsize=(11, 5))
    t_axis = np.linspace(0, 3, 500)
    x_axis, xdot_axis, xddot_axis = via_point_trajectory(times, x_pos, x_vel, t_axis)
    y_axis, ydot_axis, yddot_axis = via_point_trajectory(times, y_pos, y_vel, t_axis)

    axes[0].plot(x_axis, y_axis)
    axes[0].plot(x_pos, y_pos, "o", color="black")
    for i, (xi, yi) in enumerate(zip(x_pos, y_pos)):
        axes[0].annotate(f"via {i+1}", (xi, yi), textcoords="offset points", xytext=(8, 8))
    axes[0].set_xlabel("x")
    axes[0].set_ylabel("y")
    axes[0].set_title("Path in (x,y) space (cf. Modern Robotics Fig. 9.7(a))")

    axes[1].plot(t_axis, x_axis, label="x(t)")
    axes[1].plot(t_axis, y_axis, label="y(t)")
    for tv in times:
        axes[1].axvline(tv, color="gray", linestyle="--", linewidth=0.6)
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("position")
    axes[1].set_title("Coordinate time histories (cf. Fig. 9.8)")
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "05_via_point_trajectory.png", dpi=150)
    print(f"\nグラフを保存しました: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
