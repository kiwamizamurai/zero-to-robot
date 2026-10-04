"""2関節平面アームの計算トルク制御(computed torque control / feedback linearization)。

モデル M̃,c̃,g̃ を使って非線形な動力学を打ち消し、閉ループの誤差ダイナミクスを
線形の(臨界減衰の)2階系に帰着させる。Lynch and Park, *Modern Robotics*
(Cambridge University Press, 2017) 11.2.2.3節の式(11.19)-(11.21)に基づく。
動力学モデル(質量行列・遠心力/コリオリ力項・重力項)は 02_dynamics/02_two_link_dynamics.py
と同じ2関節平面アーム。
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

l1, l2 = 1.0, 0.8
lc1, lc2 = 0.5, 0.4
m1, m2 = 1.0, 0.8
I1, I2 = 0.05, 0.03
g_acc = 9.81
OUTPUT_DIR = Path(__file__).parent / "output"


def mass_matrix(q, m1_=m1, m2_=m2):
    theta2 = q[1]
    H11 = m1_ * lc1**2 + I1 + m2_ * (l1**2 + lc2**2 + 2 * l1 * lc2 * np.cos(theta2)) + I2
    H22 = m2_ * lc2**2 + I2
    H12 = m2_ * (lc2**2 + l1 * lc2 * np.cos(theta2)) + I2
    return np.array([[H11, H12], [H12, H22]])


def coriolis_centrifugal(q, qdot, m2_=m2):
    theta2 = q[1]
    theta1_dot, theta2_dot = qdot
    h = m2_ * l1 * lc2 * np.sin(theta2)
    return np.array([-h * theta2_dot**2 - 2 * h * theta1_dot * theta2_dot, h * theta1_dot**2])


def gravity(q, m1_=m1, m2_=m2):
    theta1, theta2 = q
    G1 = m1_ * g_acc * lc1 * np.cos(theta1) + m2_ * g_acc * (lc2 * np.cos(theta1 + theta2) + l1 * np.cos(theta1))
    G2 = m2_ * g_acc * lc2 * np.cos(theta1 + theta2)
    return np.array([G1, G2])


def true_dynamics_rhs(q, qdot, tau):
    """真のプラント(パラメータはm1,m2,l1,l2,...そのもの)の順動力学。"""
    M = mass_matrix(q)
    rhs = tau - coriolis_centrifugal(q, qdot) - gravity(q)
    return np.linalg.solve(M, rhs)


def desired_trajectory(t):
    """目標軌道(滑らかで解析的に既知の theta_d, theta_d_dot, theta_d_ddot)。
    05_planning で本来生成すべき軌道の代わりに、ここでは単純な正弦波を使う。"""
    theta_d = np.array([0.5 + 0.3 * np.sin(t), -0.3 + 0.4 * np.cos(0.7 * t)])
    theta_d_dot = np.array([0.3 * np.cos(t), -0.4 * 0.7 * np.sin(0.7 * t)])
    theta_d_ddot = np.array([-0.3 * np.sin(t), -0.4 * 0.7**2 * np.cos(0.7 * t)])
    return theta_d, theta_d_dot, theta_d_ddot


def critically_damped_gains(kp_diag):
    """各関節を臨界減衰(zeta=1)にするKdをKpから決める: Kd_i = 2*sqrt(Kp_i)。"""
    return np.diag(2 * np.sqrt(np.diag(kp_diag)))


Kp = np.diag([100.0, 100.0])
Kd = critically_damped_gains(Kp)


def computed_torque_control(theta, theta_dot, t, model_scale=1.0):
    """model_scale: モデル化誤差を模すため、コントローラ内部で使うm2だけをスケールする。"""
    theta_d, theta_d_dot, theta_d_ddot = desired_trajectory(t)
    theta_e = theta_d - theta
    theta_e_dot = theta_d_dot - theta_dot
    theta_ddot_com = theta_d_ddot + Kp @ theta_e + Kd @ theta_e_dot
    m2_hat = m2 * model_scale
    tau = mass_matrix(theta, m2_=m2_hat) @ theta_ddot_com + coriolis_centrifugal(theta, theta_dot, m2_=m2_hat) + gravity(theta, m2_=m2_hat)
    return tau, theta_e, theta_e_dot


def simulate(model_scale, dt, n_steps, theta0, theta0_dot):
    def deriv(state, t):
        q, qdot = state[:2], state[2:]
        tau, _, _ = computed_torque_control(q, qdot, t, model_scale)
        qddot = true_dynamics_rhs(q, qdot, tau)
        return np.concatenate([qdot, qddot])

    state = np.concatenate([theta0, theta0_dot])
    errors = np.zeros((n_steps, 2))
    for k in range(n_steps):
        t = k * dt
        theta_d, theta_d_dot, _ = desired_trajectory(t)
        errors[k] = theta_d - state[:2]
        k1 = deriv(state, t)
        k2 = deriv(state + dt / 2 * k1, t + dt / 2)
        k3 = deriv(state + dt / 2 * k2, t + dt / 2)
        k4 = deriv(state + dt * k3, t + dt)
        state = state + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return errors


def ideal_error_response(theta_e0, theta_e0_dot, t_axis, kp_diag, kd_diag):
    """モデルが完全に一致する場合の理論解: 各関節独立に
    theta_e'' + Kd theta_e' + Kp theta_e = 0 (臨界減衰: Kd=2 sqrt(Kp)) の閉形式解
    theta_e(t) = (c1 + c2 t) exp(-omega_n t), omega_n = sqrt(Kp)。"""
    n = len(theta_e0)
    result = np.zeros((len(t_axis), n))
    for i in range(n):
        omega_n = np.sqrt(kp_diag[i, i])
        c1 = theta_e0[i]
        c2 = theta_e0_dot[i] + omega_n * c1
        result[:, i] = (c1 + c2 * t_axis) * np.exp(-omega_n * t_axis)
    return result


def main():
    OUTPUT_DIR.mkdir(exist_ok=True)

    dt, n_steps = 0.001, 5000
    t_axis = np.arange(n_steps) * dt
    theta_d0, theta_d0_dot, _ = desired_trajectory(0.0)
    theta0 = theta_d0 + np.array([0.2, -0.15])  # 初期誤差を与える
    theta0_dot = theta_d0_dot.copy()
    theta_e0 = theta_d0 - theta0
    theta_e0_dot = np.zeros(2)  # theta0_dot = theta_d0_dot としたので初速の誤差は0

    # (1) 完全なモデル(model_scale=1)での追従誤差 vs 理論の臨界減衰応答
    errors_perfect = simulate(1.0, dt, n_steps, theta0, theta0_dot)
    ideal = ideal_error_response(theta_e0, theta_e0_dot, t_axis, Kp, Kd)
    max_err_vs_ideal = np.max(np.abs(errors_perfect - ideal))
    print(f"完全なモデルでの追従誤差 vs 理論(臨界減衰)解: 最大誤差 = {max_err_vs_ideal:.2e}")
    assert max_err_vs_ideal < 1e-3

    # (2) 誤差が実際に0へ収束するか
    final_err = np.max(np.abs(errors_perfect[-1]))
    print(f"最終追従誤差(t={n_steps*dt:.0f}s): {final_err:.2e}")
    assert final_err < 1e-6

    # (3) モデル化誤差(m2を20%過大評価)がある場合、理論(臨界減衰)解からのズレが
    #     完全なモデルの場合より明確に大きくなることを確認
    errors_mismatched = simulate(1.2, dt, n_steps, theta0, theta0_dot)
    max_err_mismatched_vs_ideal = np.max(np.abs(errors_mismatched - ideal))
    print(f"\nモデル化誤差(m2を20%過大評価)がある場合の理論解からのズレ: 最大 = {max_err_mismatched_vs_ideal:.2e}")
    print(f"(完全なモデルの場合の {max_err_vs_ideal:.2e} と比較して、ズレが大きい)")
    assert max_err_mismatched_vs_ideal > 10 * max_err_vs_ideal

    print("\nすべてのチェックに合格しました。")

    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for i, name in enumerate(["theta1", "theta2"]):
        axes[i].plot(t_axis, errors_perfect[:, i], label="simulated (perfect model)")
        axes[i].plot(t_axis, ideal[:, i], "--", label="ideal (critically damped)")
        axes[i].plot(t_axis, errors_mismatched[:, i], label="simulated (m2 +20% model error)", alpha=0.7)
        axes[i].set_ylabel(f"error {name}")
        axes[i].legend()
    axes[1].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "02_computed_torque_error.png", dpi=150)
    print(f"\nグラフを保存しました: {OUTPUT_DIR / '02_computed_torque_error.png'}")


if __name__ == "__main__":
    main()
