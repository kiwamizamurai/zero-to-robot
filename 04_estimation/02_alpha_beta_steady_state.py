"""定速度モデルのカルマンフィルタは、定常状態でalpha-betaフィルタ(固定ゲイン)に一致することを確認する。"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

dt = 0.1
N = 400
OUTPUT_DIR = Path(__file__).parent / "output"

# 状態 x = [position, velocity]^T の定速度モデル(等速直線運動 + プロセスノイズ)
A = np.array([[1, dt], [0, 1]])
H = np.array([[1.0, 0.0]])
Q = np.diag([1e-4, 1e-3])
R = np.array([[0.5**2]])


def kalman_filter(z, x0, P0):
    """時変ゲインの通常のカルマンフィルタ。"""
    n = len(z)
    x_est, K_hist = np.zeros((n, 2)), np.zeros((n, 2))
    x, P = x0, P0
    for t in range(n):
        x_pred = A @ x
        P_pred = A @ P @ A.T + Q
        S = H @ P_pred @ H.T + R
        K = (P_pred @ H.T @ np.linalg.inv(S)).flatten()
        x = x_pred + K * (z[t, 0] - H @ x_pred)
        P = (np.eye(2) - np.outer(K, H)) @ P_pred
        x_est[t] = x
        K_hist[t] = K
    return x_est, K_hist


def fixed_gain_filter(z, x0, K):
    """定常ゲイン K を最初から固定で使うフィルタ(= alpha-beta フィルタ)。"""
    n = len(z)
    x_est = np.zeros((n, 2))
    x = x0
    for t in range(n):
        x_pred = A @ x
        x = x_pred + K * (z[t, 0] - H @ x_pred)
        x_est[t] = x
    return x_est


def main():
    OUTPUT_DIR.mkdir(exist_ok=True)
    rng = np.random.default_rng(0)

    true_pos0, true_vel = 0.0, 1.0  # 等速直線運動(真の系にはプロセスノイズなし)
    true_states = np.array([[true_pos0 + true_vel * t * dt, true_vel] for t in range(N)])
    z = true_states[:, [0]] + rng.normal(0, np.sqrt(R[0, 0]), size=(N, 1))

    x0 = np.zeros(2)
    P0 = np.diag([1.0, 1.0])
    x_kf, K_hist = kalman_filter(z, x0, P0)

    K_ss = K_hist[-1]
    alpha, beta = K_ss[0], K_ss[1] * dt
    print(f"収束後の定常ゲイン: K_ss = {K_ss}")
    print(f"alpha-betaパラメータ: alpha = {alpha:.4f}, beta = {beta:.4f}")

    x_ab = fixed_gain_filter(z, x0, K_ss)

    # 誤差の収束レート |eig((I-K_ss H)A)| ~= 0.94 と遅いため、
    # 過渡応答が十分に減衰する区間まで burn-in を長く取って比較する。
    burn_in = 250
    diff = np.abs(x_kf[burn_in:] - x_ab[burn_in:])
    print(f"\n過渡応答後(t>{burn_in*dt:.0f}s)のKFとalpha-betaフィルタの最大差: {diff.max():.2e}")

    assert diff.max() < 1e-3, "定常状態ではKFとalpha-betaフィルタはほぼ一致するはず"
    print("定常状態でKF = alpha-betaフィルタであることを確認: OK")

    t_axis = np.arange(N) * dt
    fig, axes = plt.subplots(2, 1, figsize=(8, 7))

    axes[0].plot(t_axis, z.flatten(), ".", label="measured (noisy)", markersize=3, alpha=0.4)
    axes[0].plot(t_axis, true_states[:, 0], label="true position", linewidth=1.5)
    axes[0].plot(t_axis, x_kf[:, 0], label="Kalman filter (time-varying gain)", linewidth=1)
    axes[0].plot(t_axis, x_ab[:, 0], "--", label="alpha-beta filter (fixed gain)", linewidth=1)
    axes[0].set_ylabel("position")
    axes[0].legend()

    axes[1].plot(t_axis, K_hist[:, 0], label="K_t[0] (position gain, -> alpha)")
    axes[1].plot(t_axis, K_hist[:, 1], label="K_t[1] (velocity gain, -> beta/dt)")
    axes[1].axhline(K_ss[0], color="C0", linestyle=":", linewidth=1)
    axes[1].axhline(K_ss[1], color="C1", linestyle=":", linewidth=1)
    axes[1].set_ylabel("Kalman gain")
    axes[1].set_xlabel("time [s]")
    axes[1].legend()

    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "02_alpha_beta_steady_state.png", dpi=150)
    print(f"\nグラフを保存しました: {OUTPUT_DIR / '02_alpha_beta_steady_state.png'}")


if __name__ == "__main__":
    main()
