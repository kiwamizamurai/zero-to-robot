"""関節空間の多項式軌道: 3次/5次時間スケーリングと、5次多項式が躍度(jerk)を
最小化すること(Flash and Hogan, 1985 の最小躍度モデル)の数値的検算。

時間スケーリング s(t): [0,T] -> [0,1] を使い、theta(t) = theta_start + s(t)(theta_end-theta_start)
として関節空間の軌道を作る(Lynch and Park, *Modern Robotics*, 2017, 9.2節に基づく)。
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def cubic_time_scaling(t, T):
    """3次多項式時間スケーリング: s(0)=0,s_dot(0)=0,s(T)=1,s_dot(T)=0。
    端点で加速度が不連続(躍度が無限大)になる。"""
    a2, a3 = 3 / T**2, -2 / T**3
    s = a2 * t**2 + a3 * t**3
    s_dot = 2 * a2 * t + 3 * a3 * t**2
    s_ddot = 2 * a2 + 6 * a3 * t
    return s, s_dot, s_ddot


def quintic_time_scaling(t, T):
    """5次多項式時間スケーリング: 位置・速度に加え加速度も両端点で0。
    s(t) = 10(t/T)^3 - 15(t/T)^4 + 6(t/T)^5 (最小躍度軌道, Flash and Hogan, 1985)。"""
    a3, a4, a5 = 10 / T**3, -15 / T**4, 6 / T**5
    s = a3 * t**3 + a4 * t**4 + a5 * t**5
    s_dot = 3 * a3 * t**2 + 4 * a4 * t**3 + 5 * a5 * t**4
    s_ddot = 6 * a3 * t + 12 * a4 * t**2 + 20 * a5 * t**3
    s_dddot = 6 * a3 + 24 * a4 * t + 60 * a5 * t**2  # 躍度(jerk)
    return s, s_dot, s_ddot, s_dddot


def joint_trajectory(theta_start, theta_end, t, T, method="quintic"):
    if method == "cubic":
        s, s_dot, s_ddot = cubic_time_scaling(t, T)
    else:
        s, s_dot, s_ddot, _ = quintic_time_scaling(t, T)
    delta = theta_end - theta_start
    return theta_start + s * delta, s_dot * delta, s_ddot * delta


def solve_quintic_coeffs_numerically(T):
    """6つの境界条件を6x6線形連立方程式として数値的に解き、閉じた式(10,-15,6)/T^3,T^4,T^5 と比較する。"""
    # s(t) = a0+a1 t+a2 t^2+a3 t^3+a4 t^4+a5 t^5 の係数を、
    # [s(0),sdot(0),sddot(0),s(T),sdot(T),sddot(T)] = [0,0,0,1,0,0] から求める。
    A = np.array(
        [
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 2, 0, 0, 0],
            [1, T, T**2, T**3, T**4, T**5],
            [0, 1, 2 * T, 3 * T**2, 4 * T**3, 5 * T**4],
            [0, 0, 2, 6 * T, 12 * T**2, 20 * T**3],
        ]
    )
    b = np.array([0, 0, 0, 1, 0, 0])
    return np.linalg.solve(A, b)


def jerk_integral(a3, a4, a5, T, n=20000):
    """s(t)=a3t^3+a4t^4+a5t^5 の躍度 s'''(t)=6a3+24a4 t+60a5 t^2 の2乗を[0,T]で積分する。"""
    t = np.linspace(0, T, n)
    jerk = 6 * a3 + 24 * a4 * t + 60 * a5 * t**2
    return np.trapezoid(jerk**2, t)


def main():
    OUTPUT_DIR = Path(__file__).parent / "output"
    OUTPUT_DIR.mkdir(exist_ok=True)
    T = 2.0
    theta_start, theta_end = 0.2, 1.5

    # (1) 境界条件を満たすか(位置・速度、5次はさらに加速度も両端点で正しい値)
    t_test = np.array([0.0, T])
    for method in ["cubic", "quintic"]:
        theta, theta_dot, theta_ddot = joint_trajectory(theta_start, theta_end, t_test, T, method)
        pos_err = max(abs(theta[0] - theta_start), abs(theta[1] - theta_end))
        vel_err = max(abs(theta_dot[0]), abs(theta_dot[1]))
        print(f"{method}: 位置の境界条件誤差={pos_err:.2e}, 速度の境界条件誤差={vel_err:.2e}")
        assert pos_err < 1e-10 and vel_err < 1e-10
    _, _, acc0_5, _ = quintic_time_scaling(np.array([0.0, T]), T)
    print(f"quintic: 加速度の境界条件誤差={np.max(np.abs(acc0_5)):.2e}")
    assert np.max(np.abs(acc0_5)) < 1e-10

    # (2) 3次多項式は端点で加速度が不連続(静止状態の加速度0からの跳躍)、5次は連続
    _, _, acc_cubic_at_0 = cubic_time_scaling(np.array([1e-9]), T)
    _, _, acc_quintic_at_0, _ = quintic_time_scaling(np.array([1e-9]), T)
    print(f"\nt->0でのs_ddot: 3次={acc_cubic_at_0[0]:.4f}(静止時の0から跳躍), 5次={acc_quintic_at_0[0]:.6f}(0から連続)")
    assert abs(acc_cubic_at_0[0]) > 1.0
    assert abs(acc_quintic_at_0[0]) < 1e-6

    # (3) 数値的に解いた5次多項式の係数が閉じた式(10,-15,6)/T^3,T^4,T^5 と一致するか
    coeffs_numeric = solve_quintic_coeffs_numerically(T)
    coeffs_closed = np.array([0, 0, 0, 10 / T**3, -15 / T**4, 6 / T**5])
    coeff_err = np.max(np.abs(coeffs_numeric - coeffs_closed))
    print(f"\n5次多項式係数: 数値解 vs 閉じた式(10,-15,6): 最大誤差 = {coeff_err:.2e}")
    assert coeff_err < 1e-8

    # (4) 5次時間スケーリングが躍度積分を最小化するか(変分法の数値検算)
    #     境界条件を満たす摂動 phi(t) = t^3 (T-t)^3 (両端点で phi,phi',phi''が0) を加えて、
    #     epsilon=0 (元の5次多項式)で躍度積分が最小になっているかを確認する。
    a3_0, a4_0, a5_0 = 10 / T**3, -15 / T**4, 6 / T**5
    J0 = jerk_integral(a3_0, a4_0, a5_0, T)

    def perturbed_jerk_integral(eps, T, n=20000):
        t = np.linspace(0, T, n)
        # phi(t) = t^3 (T-t)^3, phi'''(t) を解析的に計算
        # phi(t) = t^3(T-t)^3 を展開: -t^6+3T t^5-3T^2t^4+T^3t^3
        # phi'''(t) = -120 t^3 + 180 T t^2 - 72 T^2 t + 6T^3
        phi_dddot = -120 * t**3 + 180 * T * t**2 - 72 * T**2 * t + 6 * T**3
        jerk = (6 * a3_0 + 24 * a4_0 * t + 60 * a5_0 * t**2) + eps * phi_dddot
        return np.trapezoid(jerk**2, t)

    epsilons = np.linspace(-0.05, 0.05, 21)
    J_values = np.array([perturbed_jerk_integral(eps, T) for eps in epsilons])
    print(f"\n躍度積分 J(epsilon=0) = {J0:.6f} (境界条件を満たす摂動を加えたときの最小値か確認)")
    assert np.isclose(J_values[len(J_values) // 2], J0, rtol=1e-6)
    assert np.all(J_values >= J0 - 1e-9), "epsilon=0で最小になっていない"
    assert np.all(J_values[epsilons != 0] > J0), "epsilon=0が厳密な最小点になっていない"
    print("摂動を加えるとどの方向でも躍度積分が増加する(epsilon=0が最小点): OK")

    print("\nすべてのチェックに合格しました。")

    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    t_axis = np.linspace(0, T, 500)
    theta_c, theta_dot_c, theta_ddot_c = joint_trajectory(theta_start, theta_end, t_axis, T, "cubic")
    theta_q, theta_dot_q, theta_ddot_q = joint_trajectory(theta_start, theta_end, t_axis, T, "quintic")
    for ax, (yc, yq, label) in zip(
        axes, [(theta_c, theta_q, "position"), (theta_dot_c, theta_dot_q, "velocity"), (theta_ddot_c, theta_ddot_q, "acceleration")]
    ):
        ax.plot(t_axis, yc, label="cubic")
        ax.plot(t_axis, yq, label="quintic")
        ax.set_ylabel(label)
        ax.legend()
    axes[-1].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "01_polynomial_trajectories.png", dpi=150)

    fig2, ax2 = plt.subplots()
    ax2.plot(epsilons, J_values, "o-")
    ax2.axvline(0, color="gray", linestyle="--")
    ax2.set_xlabel("epsilon (perturbation amount)")
    ax2.set_ylabel("jerk integral J")
    ax2.set_title("Quintic minimizes the jerk integral")
    fig2.savefig(OUTPUT_DIR / "01_jerk_minimization.png", dpi=150)
    print(f"\nグラフを保存しました: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
