"""2関節アーム: 勾配降下法によるIK。損失曲線・ヤコビアンの検算・公式解との比較を行う。"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

l1, l2 = 1.0, 0.8
OUTPUT_DIR = Path(__file__).parent / "output"


def fk(q):
    a, b = q[0], q[0] + q[1]
    return np.array([l1 * np.cos(a) + l2 * np.cos(b), l1 * np.sin(a) + l2 * np.sin(b)])


def jac(q):
    a, b = q[0], q[0] + q[1]
    return np.array(
        [
            [-l1 * np.sin(a) - l2 * np.sin(b), -l2 * np.sin(b)],
            [l1 * np.cos(a) + l2 * np.cos(b), l2 * np.cos(b)],
        ]
    )


def numerical_jac(f, q, eps=1e-6):
    """有限差分によるヤコビアン。解析的な jac() の検算に使う。"""
    J = np.zeros((len(f(q)), len(q)))
    for i in range(len(q)):
        dq = np.zeros_like(q)
        dq[i] = eps
        J[:, i] = (f(q + dq) - f(q - dq)) / (2 * eps)
    return J


def closed_form_ik(target, elbow_up=True):
    """余弦定理による2関節IKの公式解（肘が上/下の2解がある）"""
    x, y = target
    c2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    c2 = np.clip(c2, -1.0, 1.0)
    q2 = np.arccos(c2) * (1 if elbow_up else -1)
    q1 = np.arctan2(y, x) - np.arctan2(l2 * np.sin(q2), l1 + l2 * np.cos(q2))
    return np.array([q1, q2])


def gradient_check():
    rng = np.random.default_rng(0)
    max_err = 0.0
    for _ in range(50):
        q = rng.uniform(-np.pi, np.pi, size=2)
        max_err = max(max_err, np.max(np.abs(jac(q) - numerical_jac(fk, q))))
    print(f"ヤコビアンの有限差分チェック: 最大誤差 = {max_err:.2e}")
    assert max_err < 1e-6
    print("  -> OK: 解析的ヤコビアンは有限差分と一致")


def gradient_descent_ik(target, q0, lr=0.1, n_iter=500):
    q = q0.copy()
    losses = []
    for _ in range(n_iter):
        e = target - fk(q)
        losses.append(0.5 * e @ e)
        q -= lr * (-jac(q).T @ e)
    return q, np.array(losses)


def main():
    OUTPUT_DIR.mkdir(exist_ok=True)

    gradient_check()

    target = np.array([1.2, 0.9])
    q0 = np.array([0.3, 0.3])
    q_gd, losses = gradient_descent_ik(target, q0)

    print(f"\n勾配降下法の最終損失: {losses[-1]:.2e}")
    print(f"勾配降下法の解: q = {q_gd}, fk(q) = {fk(q_gd)}")

    q_closed = closed_form_ik(target, elbow_up=True)
    print(f"公式解(肘上): q = {q_closed}, fk(q) = {fk(q_closed)}")

    assert np.allclose(fk(q_gd), target, atol=1e-4)
    assert np.allclose(fk(q_closed), target, atol=1e-9)
    print("\n両方とも目標位置 [1.2 0.9] に到達: OK")

    same_solution = np.allclose(np.sin(q_gd), np.sin(q_closed), atol=1e-3) and np.allclose(
        np.cos(q_gd), np.cos(q_closed), atol=1e-3
    )
    print(
        f"勾配降下法と公式解の角度は一致{'する' if same_solution else 'しない（別の解＝肘の向きが逆に収束した）'}"
    )

    fig, ax = plt.subplots()
    ax.semilogy(losses)
    ax.set_xlabel("iteration")
    ax.set_ylabel("loss")
    ax.set_title("IK via gradient descent")
    fig.savefig(OUTPUT_DIR / "02_ik_loss.png", dpi=150)
    print(f"\n損失曲線を保存しました: {OUTPUT_DIR / '02_ik_loss.png'}")


if __name__ == "__main__":
    main()
