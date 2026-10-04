"""2関節平面アームの運動方程式(ラグランジュ形式): M(q)q̈ + c(q,q̇) + g(q) = τ

01_kinematics で使ってきたのと同じ2リンク平面アーム(a=q1, b=q1+q2 の順運動学)に
質量・慣性を持たせたモデル。閉形式の運動方程式は
Asada, *Introduction to Robotics* (MIT 2.12, Fall 2005, OCW) 第7章 "Dynamics" の
Example 7.1 (Newton-Euler法) / 7.2.2節 (ラグランジュ法, 同じ結果を与える) に基づく。
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

l1, l2 = 1.0, 0.8
lc1, lc2 = 0.5, 0.4  # 各リンクの重心位置(関節からの距離)
m1, m2 = 1.0, 0.8
I1, I2 = 0.05, 0.03  # 重心まわりの慣性モーメント
g = 9.81
OUTPUT_DIR = Path(__file__).parent / "output"


def mass_matrix(q):
    """慣性行列 M(q) = [[H11,H12],[H12,H22]] (Asada eq.7.1.12)"""
    theta2 = q[1]
    H11 = m1 * lc1**2 + I1 + m2 * (l1**2 + lc2**2 + 2 * l1 * lc2 * np.cos(theta2)) + I2
    H22 = m2 * lc2**2 + I2
    H12 = m2 * (lc2**2 + l1 * lc2 * np.cos(theta2)) + I2
    return np.array([[H11, H12], [H12, H22]])


def coriolis_centrifugal(q, qdot):
    """遠心力・コリオリ力項 c(q,q̇) (Asada eq.7.1.11, hの定義はeq.7.1.12-d)"""
    theta2 = q[1]
    theta1_dot, theta2_dot = qdot
    h = m2 * l1 * lc2 * np.sin(theta2)
    c1 = -h * theta2_dot**2 - 2 * h * theta1_dot * theta2_dot
    c2 = h * theta1_dot**2
    return np.array([c1, c2])


def gravity(q):
    """重力項 g(q) (Asada eq.7.1.12-e,f)"""
    theta1, theta2 = q
    G1 = m1 * g * lc1 * np.cos(theta1) + m2 * g * (lc2 * np.cos(theta1 + theta2) + l1 * np.cos(theta1))
    G2 = m2 * g * lc2 * np.cos(theta1 + theta2)
    return np.array([G1, G2])


def inverse_dynamics(q, qdot, qddot):
    """逆動力学: (q,q̇,q̈) から必要なトルク τ を求める。"""
    return mass_matrix(q) @ qddot + coriolis_centrifugal(q, qdot) + gravity(q)


def forward_dynamics(q, qdot, tau):
    """順動力学: (q,q̇,τ) から q̈ を求める。"""
    M = mass_matrix(q)
    rhs = tau - coriolis_centrifugal(q, qdot) - gravity(q)
    return np.linalg.solve(M, rhs)


def kinetic_energy(q, qdot):
    return 0.5 * qdot @ mass_matrix(q) @ qdot


def potential_energy(q):
    theta1, theta2 = q
    return m1 * g * lc1 * np.sin(theta1) + m2 * g * (l1 * np.sin(theta1) + lc2 * np.sin(theta1 + theta2))


def lagrangian(q, qdot):
    return kinetic_energy(q, qdot) - potential_energy(q)


def euler_lagrange_finite_diff(q, qdot, qddot, eps=1e-5):
    """L=T-U だけから有限差分でオイラー・ラグランジュ方程式の左辺
    d/dt(dL/dq̇) - dL/dq を計算する(モデルに依らない検算用)。
    これは inverse_dynamics(q,qdot,qddot) と一致するはず。"""
    n = len(q)

    def dLdq_at(qq, qqdot):
        val = np.zeros(n)
        for i in range(n):
            dq = np.zeros(n)
            dq[i] = eps
            val[i] = (lagrangian(qq + dq, qqdot) - lagrangian(qq - dq, qqdot)) / (2 * eps)
        return val

    def dLdqdot_at(t):
        qt = q + qdot * t + 0.5 * qddot * t**2
        qdot_t = qdot + qddot * t
        val = np.zeros(n)
        for i in range(n):
            dq = np.zeros(n)
            dq[i] = eps
            val[i] = (lagrangian(qt, qdot_t + dq) - lagrangian(qt, qdot_t - dq)) / (2 * eps)
        return val

    ddt_dLdqdot = (dLdqdot_at(eps) - dLdqdot_at(-eps)) / (2 * eps)
    return ddt_dLdqdot - dLdq_at(q, qdot)


def simulate_rk4(q0, qdot0, dt, n_steps, tau=np.zeros(2)):
    """RK4によるシミュレーション(自由運動のエネルギー保存検算に十分な精度が必要なため)。"""
    def deriv(state):
        q, qdot = state[:2], state[2:]
        qddot = forward_dynamics(q, qdot, tau)
        return np.concatenate([qdot, qddot])

    state = np.concatenate([q0, qdot0])
    states = np.zeros((n_steps, 4))
    for t in range(n_steps):
        k1 = deriv(state)
        k2 = deriv(state + dt / 2 * k1)
        k3 = deriv(state + dt / 2 * k2)
        k4 = deriv(state + dt * k3)
        state = state + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        states[t] = state
    return states


def main():
    OUTPUT_DIR.mkdir(exist_ok=True)
    rng = np.random.default_rng(0)

    # (1) 有限差分によるオイラー・ラグランジュ方程式との一致(モデルに依らない検算)
    max_err = 0.0
    for _ in range(50):
        q = rng.uniform(-np.pi, np.pi, size=2)
        qdot = rng.uniform(-2, 2, size=2)
        qddot = rng.uniform(-2, 2, size=2)
        tau_analytic = inverse_dynamics(q, qdot, qddot)
        tau_fd = euler_lagrange_finite_diff(q, qdot, qddot)
        max_err = max(max_err, np.max(np.abs(tau_analytic - tau_fd)))
    print(f"逆動力学(M,c,gの式) vs 有限差分によるオイラー・ラグランジュ方程式: 最大誤差 = {max_err:.2e}")
    assert max_err < 1e-3

    # (2) エネルギー保存(自由運動、摩擦・外力なし)
    q0 = np.array([0.3, 0.5])
    qdot0 = np.array([0.0, 0.0])
    dt, n_steps = 0.001, 10000
    states = simulate_rk4(q0, qdot0, dt, n_steps)

    energies = np.array([kinetic_energy(s[:2], s[2:]) + potential_energy(s[:2]) for s in states])
    drift = energies[-1] - energies[0]
    print(f"\nエネルギー保存(自由運動, {n_steps*dt:.0f}秒): 初期値={energies[0]:.4f}, 変化={drift:.2e}")
    assert abs(drift) < 1e-3

    # (3) 順動力学と逆動力学が互いに逆であることの確認
    q, qdot = rng.uniform(-np.pi, np.pi, size=2), rng.uniform(-2, 2, size=2)
    tau_test = rng.uniform(-5, 5, size=2)
    qddot_fwd = forward_dynamics(q, qdot, tau_test)
    tau_back = inverse_dynamics(q, qdot, qddot_fwd)
    round_trip_err = np.max(np.abs(tau_test - tau_back))
    print(f"順動力学→逆動力学の往復誤差: {round_trip_err:.2e}")
    assert round_trip_err < 1e-8

    print("\nすべてのチェックに合格しました。")

    t_axis = np.arange(n_steps) * dt
    fig, axes = plt.subplots(2, 1, figsize=(8, 6))
    axes[0].plot(t_axis, states[:, 0], label="theta1")
    axes[0].plot(t_axis, states[:, 1], label="theta2")
    axes[0].set_ylabel("angle [rad]")
    axes[0].legend()
    axes[1].plot(t_axis, energies)
    axes[1].set_ylabel("total energy")
    axes[1].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "02_two_link_energy.png", dpi=150)
    print(f"\nグラフを保存しました: {OUTPUT_DIR / '02_two_link_energy.png'}")


if __name__ == "__main__":
    main()
