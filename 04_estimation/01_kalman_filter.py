"""線形カルマンフィルタ: 線形化した振り子の角度・角速度を、角度のみのノイズ付き観測から推定する。"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

g, l, dt = 9.81, 1.0, 0.001
N = 5000  # 5秒分
OUTPUT_DIR = Path(__file__).parent / "output"

# 状態 x = [theta, omega]^T。02_dynamics と同じ半陰的オイラー法を、
# 小角近似 sin(theta) ~= theta のもとで線形化した状態遷移行列。
#   omega_{t+1} = omega_t - (g/l) theta_t dt
#   theta_{t+1} = theta_t + omega_{t+1} dt
A = np.array(
    [
        [1 - (g / l) * dt**2, dt],
        [-(g / l) * dt, 1],
    ]
)
H = np.array([[1.0, 0.0]])  # 角度のみ観測(角速度センサはない)

Q = np.diag([1e-6, 1e-4])  # プロセスノイズ(線形化誤差・モデル化誤差を吸収)
R = np.array([[0.05**2]])  # 観測ノイズ(角度センサの標準偏差0.05 rad)


def true_pendulum_step(theta, omega, rng):
    """真の(非線形)振り子を1ステップ進める。02_dynamics の simulate_semi_implicit_euler と同じ式。"""
    process_noise = rng.multivariate_normal(np.zeros(2), Q)
    omega_new = omega - g / l * np.sin(theta) * dt + process_noise[1]
    theta_new = theta + omega_new * dt + process_noise[0]
    return theta_new, omega_new


def kalman_filter(z, x0, P0):
    """離散時間の線形カルマンフィルタ。z: 観測列, shape (N, 1)"""
    n = len(z)
    x_est = np.zeros((n, 2))
    P_est = np.zeros((n, 2, 2))
    x, P = x0, P0
    for t in range(n):
        x_pred = A @ x
        P_pred = A @ P @ A.T + Q

        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)
        x = x_pred + (K @ (z[t] - H @ x_pred))
        P = (np.eye(2) - K @ H) @ P_pred

        x_est[t] = x
        P_est[t] = P
    return x_est, P_est


def chi2_quantile_k2(p):
    """自由度2のカイ二乗分布の分位点。CDF(x) = 1 - exp(-x/2) の逆関数(閉形式)。"""
    return -2 * np.log(1 - p)


def main():
    OUTPUT_DIR.mkdir(exist_ok=True)
    rng = np.random.default_rng(0)

    theta, omega = 0.3, 0.0  # 小角近似の前提に合わせた初期角度
    true_states = np.zeros((N, 2))
    z = np.zeros((N, 1))
    for t in range(N):
        theta, omega = true_pendulum_step(theta, omega, rng)
        true_states[t] = [theta, omega]
        z[t, 0] = theta + rng.normal(0, np.sqrt(R[0, 0]))

    x0 = np.zeros(2)
    P0 = np.diag([0.1, 0.1])
    x_est, P_est = kalman_filter(z, x0, P0)

    # NEES (正規化推定誤差二乗): e^T P^-1 e は理論上、自由度2のカイ二乗分布に従う
    errors = true_states - x_est
    nees = np.array([e @ np.linalg.solve(P, e) for e, P in zip(errors, P_est)])

    lower, upper = chi2_quantile_k2(0.025), chi2_quantile_k2(0.975)
    within = np.mean((nees >= lower) & (nees <= upper))
    print(f"NEES: 平均 = {nees.mean():.3f} (理論値: 2.0)")
    print(f"NEESが95%区間[{lower:.3f}, {upper:.3f}]に収まる割合: {within:.1%} (理論値: 95.0%)")

    rmse_filtered = np.sqrt(np.mean(errors[:, 0] ** 2))
    rmse_raw = np.sqrt(np.mean((z.flatten() - true_states[:, 0]) ** 2))
    print(f"\n角度のRMSE: フィルタ後 = {rmse_filtered:.4f} rad, 生の観測 = {rmse_raw:.4f} rad")

    assert 0.85 < within < 1.0, "NEESが95%区間に収まる割合が理論値から大きく外れている"
    assert rmse_filtered < rmse_raw, "フィルタはノイズ低減できているはず"
    print("\nフィルタは一貫性(NEES)・ノイズ低減の両方でOK")

    t_axis = np.arange(N) * dt
    fig, axes = plt.subplots(3, 1, figsize=(8, 9))

    axes[0].plot(t_axis, z.flatten(), ".", label="measured (noisy)", markersize=1, alpha=0.3)
    axes[0].plot(t_axis, true_states[:, 0], label="true theta", linewidth=1)
    axes[0].plot(t_axis, x_est[:, 0], label="KF estimate", linewidth=1)
    axes[0].set_ylabel("theta [rad]")
    axes[0].legend(markerscale=5)

    axes[1].plot(t_axis, true_states[:, 1], label="true omega", linewidth=1)
    axes[1].plot(t_axis, x_est[:, 1], label="KF estimate", linewidth=1)
    axes[1].set_ylabel("omega [rad/s]")
    axes[1].legend()

    axes[2].plot(t_axis, nees, linewidth=0.5)
    axes[2].axhline(lower, color="r", linestyle="--", label="95% chi2(2) bounds")
    axes[2].axhline(upper, color="r", linestyle="--")
    axes[2].axhline(2.0, color="k", linestyle=":", label="theoretical mean = 2")
    axes[2].set_ylabel("NEES")
    axes[2].set_xlabel("time [s]")
    axes[2].set_ylim(0, 15)
    axes[2].legend()

    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "01_kalman_filter.png", dpi=150)
    print(f"\nグラフを保存しました: {OUTPUT_DIR / '01_kalman_filter.png'}")


if __name__ == "__main__":
    main()
