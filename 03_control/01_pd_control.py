"""PD制御で振り子を目標角度(真横, theta*=0)に止める。重力補償の有無で収束先が変わることを確かめる。"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

g, l, dt = 9.81, 1.0, 0.001
Kp, Kd, target = 30.0, 5.0, 0.0
N = 5000  # 5秒分
OUTPUT_DIR = Path(__file__).parent / "output"


def simulate_pd(th0=-1.5, gravity_compensation=False):
    """theta は真横(水平)から測る。真下付近から始めて水平で止めたい。"""
    th, om = th0, 0.0
    errs = []
    for _ in range(N):
        tau = Kp * (target - th) - Kd * om
        if gravity_compensation:
            tau += g * l * np.cos(th)
        om += (-g / l * np.cos(th) + tau) * dt
        th += om * dt
        errs.append(target - th)
    return np.array(errs)


def main():
    OUTPUT_DIR.mkdir(exist_ok=True)

    errs_no_comp = simulate_pd(gravity_compensation=False)
    errs_comp = simulate_pd(gravity_compensation=True)

    print(f"重力補償なし: 最終誤差 = {errs_no_comp[-1]:.4f} rad (0には収束しない)")
    print(f"重力補償あり: 最終誤差 = {errs_comp[-1]:.4f} rad")

    assert abs(errs_no_comp[-1]) > 0.05, "補償なしは定常偏差が残るはず"
    assert abs(errs_comp[-1]) < 1e-3, "補償ありは誤差がほぼ0に収束するはず"
    print("\n重力補償ありの場合のみ誤差が0に収束: OK")

    fig, ax = plt.subplots()
    ax.plot(errs_no_comp, label="no gravity compensation")
    ax.plot(errs_comp, label="with gravity compensation")
    ax.axhline(0, color="gray", linewidth=0.8, linestyle="--")
    ax.set_xlabel("step")
    ax.set_ylabel("error (target - theta)")
    ax.set_title("PD control error")
    ax.legend()
    fig.savefig(OUTPUT_DIR / "01_pd_error.png", dpi=150)
    print(f"\n誤差のグラフを保存しました: {OUTPUT_DIR / '01_pd_error.png'}")


if __name__ == "__main__":
    main()
