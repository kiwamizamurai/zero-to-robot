"""振り子のシミュレーション: エネルギー保存で積分の正しさを確かめる。"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

g, l, dt = 9.81, 1.0, 0.001
N = 10000  # 10秒分
OUTPUT_DIR = Path(__file__).parent / "output"


def energy(th, om):
    return 0.5 * l**2 * om**2 - g * l * np.cos(th)


def simulate_naive_euler(th0=1.0, om0=0.0):
    """両方の更新を同じ古い状態(th, om)から計算する素朴な(真の)オイラー法。エネルギーが増え続ける。"""
    th, om = th0, om0
    E = []
    for _ in range(N):
        th_new = th + om * dt
        om_new = om + (-g / l * np.sin(th)) * dt
        th, om = th_new, om_new
        E.append(energy(th, om))
    return np.array(E)


def simulate_semi_implicit_euler(th0=1.0, om0=0.0):
    """先に速度を更新してから位置を更新する(半陰的/シンプレクティック オイラー法)。エネルギーがほぼ一定。"""
    th, om = th0, om0
    E = []
    for _ in range(N):
        om += -g / l * np.sin(th) * dt
        th += om * dt
        E.append(energy(th, om))
    return np.array(E)


def main():
    OUTPUT_DIR.mkdir(exist_ok=True)

    E_naive = simulate_naive_euler()
    E_semi = simulate_semi_implicit_euler()

    drift_naive = E_naive[-1] - E_naive[0]
    drift_semi = E_semi[-1] - E_semi[0]

    print(f"素朴なオイラー法: エネルギーの変化 = {drift_naive:.4f} (初期値 {E_naive[0]:.4f})")
    print(f"半陰的オイラー法: エネルギーの変化 = {drift_semi:.4f} (初期値 {E_semi[0]:.4f})")
    print(
        "\n両方の更新式を同じ古い状態から計算する(素朴なオイラー法)とエネルギーが増え続けますが、"
        "片方の更新に「更新済みの新しい値」を使う(半陰的/シンプレクティック オイラー法)だけで"
        "エネルギーはほぼ一定に保たれます。"
    )

    assert abs(drift_semi) < 0.01, "半陰的オイラー法のエネルギーはほぼ一定であるべき"
    assert abs(drift_naive) > 0.1, "素朴なオイラー法のエネルギーは大きく増加するはず"

    fig, axes = plt.subplots(1, 2, figsize=(10, 4), sharey=True)
    axes[0].plot(E_naive)
    axes[0].set_title("naive Euler (both from old state)")
    axes[0].set_xlabel("step")
    axes[0].set_ylabel("energy")
    axes[1].plot(E_semi)
    axes[1].set_title("semi-implicit Euler (om then th, reusing new th)")
    axes[1].set_xlabel("step")
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "01_pendulum_energy.png", dpi=150)
    print(f"\nエネルギーのグラフを保存しました: {OUTPUT_DIR / '01_pendulum_energy.png'}")


if __name__ == "__main__":
    main()
