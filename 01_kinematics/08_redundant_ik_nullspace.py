"""冗長マニピュレータ(3関節平面アームで2次元位置のみを制御する、自由度1つ分の冗長性を持つ例)の
擬似逆行列IKと、零空間を使った副次タスク(可操作度の最大化)の実装。

擬似逆行列 J^+ = J^T (J J^T)^{-1} と、零空間への射影 (I - J^+ J) の性質
(Buss, "Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse
and Damped Least Squares methods", CMU, 2009, 式(7)(9)。零空間法自体の初出はLiegeois, 1977)は
notes.typ 12節で自前で証明する。
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def fk_3link_planar(q, l1=1.0, l2=1.0, l3=0.8):
    a, b, c = q[0], q[0] + q[1], q[0] + q[1] + q[2]
    return np.array([l1 * np.cos(a) + l2 * np.cos(b) + l3 * np.cos(c), l1 * np.sin(a) + l2 * np.sin(b) + l3 * np.sin(c)])


def jacobian_3link_planar(q, l1=1.0, l2=1.0, l3=0.8):
    """2x3の位置ヤコビアン(解析式)。"""
    a, b, c = q[0], q[0] + q[1], q[0] + q[1] + q[2]
    J = np.zeros((2, 3))
    J[:, 0] = [-l1 * np.sin(a) - l2 * np.sin(b) - l3 * np.sin(c), l1 * np.cos(a) + l2 * np.cos(b) + l3 * np.cos(c)]
    J[:, 1] = [-l2 * np.sin(b) - l3 * np.sin(c), l2 * np.cos(b) + l3 * np.cos(c)]
    J[:, 2] = [-l3 * np.sin(c), l3 * np.cos(c)]
    return J


def numerical_jacobian(fk_fn, q, eps=1e-6):
    q = np.asarray(q, dtype=float)
    n = len(q)
    J = np.zeros((2, n))
    for i in range(n):
        dq = np.zeros(n)
        dq[i] = eps
        J[:, i] = (fk_fn(q + dq) - fk_fn(q - dq)) / (2 * eps)
    return J


def right_pseudo_inverse(J):
    """J: m x n (m<n, フルランクm)の右擬似逆行列 J^+ = J^T (J J^T)^{-1}。"""
    return J.T @ np.linalg.inv(J @ J.T)


def manipulability(J):
    """05_dh_parameters.py系と同じYoshikawaの可操作度 w(J)=sqrt(det(J J^T))(06_differential_kinematics_3d.pyと同じ定義)。"""
    return np.sqrt(max(0.0, np.linalg.det(J @ J.T)))


def manipulability_gradient(q, l1=1.0, l2=1.0, l3=0.8, eps=1e-6):
    """可操作度w(theta)のtheta勾配を有限差分で求める(副次タスクの目的関数の勾配)。"""
    n = len(q)
    grad = np.zeros(n)
    for i in range(n):
        dq = np.zeros(n)
        dq[i] = eps
        w_plus = manipulability(jacobian_3link_planar(np.asarray(q) + dq, l1, l2, l3))
        w_minus = manipulability(jacobian_3link_planar(np.asarray(q) - dq, l1, l2, l3))
        grad[i] = (w_plus - w_minus) / (2 * eps)
    return grad


def redundant_ik(
    q0, target, l1=1.0, l2=1.0, l3=0.8, n_iters=300, alpha_primary=0.5, alpha_secondary=0.0, use_manipulability=True
):
    """theta_dot = J^+ e + (I - J^+ J) phi の反復積分によるIK。
    alpha_secondary=0なら最小ノルム解(純粋な擬似逆行列法)、>0なら零空間で副次タスクも追う。"""
    q = np.array(q0, dtype=float)
    history = {"q": [q.copy()], "pos_err": [], "manipulability": []}
    for _ in range(n_iters):
        J = jacobian_3link_planar(q, l1, l2, l3)
        e = target - fk_3link_planar(q, l1, l2, l3)
        J_pinv = right_pseudo_inverse(J)
        if alpha_secondary > 0:
            phi = manipulability_gradient(q, l1, l2, l3) if use_manipulability else np.zeros(3)
        else:
            phi = np.zeros(3)
        q_dot = alpha_primary * (J_pinv @ e) + alpha_secondary * (np.eye(3) - J_pinv @ J) @ phi
        q = q + q_dot
        history["q"].append(q.copy())
        history["pos_err"].append(np.linalg.norm(e))
        history["manipulability"].append(manipulability(J))
    history["q"] = np.array(history["q"])
    return q, history


def main():
    OUTPUT_DIR = Path(__file__).parent / "output"
    OUTPUT_DIR.mkdir(exist_ok=True)
    rng = np.random.default_rng(0)
    l1, l2, l3 = 1.0, 1.0, 0.8

    # (1) 解析的ヤコビアン vs 有限差分
    max_jac_err = 0.0
    for _ in range(100):
        q = rng.uniform(-np.pi, np.pi, size=3)
        J_analytic = jacobian_3link_planar(q, l1, l2, l3)
        J_numeric = numerical_jacobian(lambda qq: fk_3link_planar(qq, l1, l2, l3), q)
        max_jac_err = max(max_jac_err, np.max(np.abs(J_analytic - J_numeric)))
    print(f"解析的ヤコビアン vs 有限差分: 最大誤差 = {max_jac_err:.2e}")
    assert max_jac_err < 1e-6

    # (2) J @ J^+ = I_2 (フルランクなら右逆行列になっている)
    max_right_inverse_err = 0.0
    for _ in range(100):
        q = rng.uniform(-np.pi, np.pi, size=3)
        J = jacobian_3link_planar(q, l1, l2, l3)
        if manipulability(J) < 1e-3:
            continue
        J_pinv = right_pseudo_inverse(J)
        max_right_inverse_err = max(max_right_inverse_err, np.max(np.abs(J @ J_pinv - np.eye(2))))
    print(f"J @ J^+ = I の最大誤差: {max_right_inverse_err:.2e}")
    assert max_right_inverse_err < 1e-8

    # (3) P:=I-J^+J が射影行列であること(冪等性 P@P=P)と、J@P=0(零空間への射影)を確認
    q_test = np.array([0.3, 0.5, -0.4])
    J = jacobian_3link_planar(q_test, l1, l2, l3)
    J_pinv = right_pseudo_inverse(J)
    P = np.eye(3) - J_pinv @ J
    idempotent_err = np.max(np.abs(P @ P - P))
    jp_zero_err = np.max(np.abs(J @ P))
    print(f"\nP=I-J^+J の冪等性(P@P=P)誤差: {idempotent_err:.2e}")
    print(f"J@P=0(零空間への射影)の誤差: {jp_zero_err:.2e}")
    assert idempotent_err < 1e-10 and jp_zero_err < 1e-10

    # (4) 最小ノルム性: J theta_dot = e を満たす解の中で、J^+ e が最小ノルムであることを確認
    #     (J^+ e に、零空間の適当なベクトル P@phi を足した「別の解」と比較する)
    e_test = np.array([0.1, -0.2])
    theta_dot_min = J_pinv @ e_test
    for _ in range(20):
        phi = rng.normal(size=3)
        theta_dot_other = theta_dot_min + P @ phi
        # 両方とも J theta_dot = e を満たす解であることを確認
        assert np.max(np.abs(J @ theta_dot_other - e_test)) < 1e-8
        assert np.linalg.norm(theta_dot_other) >= np.linalg.norm(theta_dot_min) - 1e-10
    print(f"J^+e が最小ノルム解であること(20個のランダムな別解と比較): OK")

    # (5) 零空間法によるIK: 目標位置には到達しつつ、副次タスク(可操作度の最大化)を追えるか
    target = np.array([1.5, 0.8])
    q0 = np.array([0.2, 0.3, 0.1])

    q_final_plain, hist_plain = redundant_ik(q0, target, l1, l2, l3, alpha_secondary=0.0)
    q_final_manip, hist_manip = redundant_ik(q0, target, l1, l2, l3, alpha_secondary=0.3, use_manipulability=True)

    err_plain = np.linalg.norm(fk_3link_planar(q_final_plain, l1, l2, l3) - target)
    err_manip = np.linalg.norm(fk_3link_planar(q_final_manip, l1, l2, l3) - target)
    print(f"\n最小ノルムIK: 最終位置誤差={err_plain:.2e}, 最終可操作度={hist_plain['manipulability'][-1]:.4f}")
    print(f"零空間で可操作度最大化: 最終位置誤差={err_manip:.2e}, 最終可操作度={hist_manip['manipulability'][-1]:.4f}")
    assert err_plain < 1e-6 and err_manip < 1e-6
    assert hist_manip["manipulability"][-1] > hist_plain["manipulability"][-1], (
        "副次タスクで可操作度が改善しているはず"
    )
    # 同じ目標位置に到達しているが、姿勢(関節角)は異なる(=自己運動、redundancyの直接的な現れ)
    posture_diff = np.linalg.norm(q_final_plain - q_final_manip)
    print(f"同じ目標位置に到達した2つの姿勢の差(自己運動) ||q_plain - q_manip|| = {posture_diff:.4f}")
    assert posture_diff > 0.05

    # (6) 自己運動(self-motion)多様体: 同じ目標位置を保ったまま冗長自由度(q1)を動かせることを確認
    #     連続法(continuation)でq1を少しずつ動かし、直前の解を初期値として(q2,q3)だけを
    #     IK(2x2の正方ヤコビアンをそのまま逆行列で解くニュートン法)で再調整して同じtargetに到達させる
    def solve_q23(q1, q23_init, n_iters=50, damping=0.8):
        q = np.array([q1, q23_init[0], q23_init[1]])
        for _ in range(n_iters):
            J23 = jacobian_3link_planar(q, l1, l2, l3)[:, 1:3]
            e = target - fk_3link_planar(q, l1, l2, l3)
            if np.linalg.cond(J23) > 1e8:
                break
            q[1:3] += damping * np.linalg.solve(J23, e)
        err = np.linalg.norm(fk_3link_planar(q, l1, l2, l3) - target)
        return q[1:3], err

    def trace_direction(q1_center, sign, q23_start, max_offset=0.5, n_steps=8):
        """連続法でq1をsignの向きに動かし、追跡できた(q1,q2,q3)のリストを返す
        (elbow(l1関節)がtargetに届く範囲を超えると2リンク部分の逆運動学が解けなくなるので、
        その時点で追跡を打ち切る)。"""
        traced = []
        q23 = q23_start
        for dq1 in np.linspace(0, max_offset, n_steps)[1:]:
            q1 = q1_center + sign * dq1
            q23_new, err = solve_q23(q1, q23)
            if err > 1e-6:
                break
            q23 = q23_new
            traced.append((q1, q23[0], q23[1]))
        return traced

    q1_center = q_final_plain[0]
    forward = trace_direction(q1_center, +1, q_final_plain[1:3])
    backward = trace_direction(q1_center, -1, q_final_plain[1:3])
    self_motion_q = np.array(backward[::-1] + [tuple(q_final_plain)] + forward)
    print(f"自己運動を追跡できた範囲: q1 in [{self_motion_q[0,0]:.3f}, {self_motion_q[-1,0]:.3f}] ({len(self_motion_q)}点)")
    self_motion_errs = [np.linalg.norm(fk_3link_planar(q, l1, l2, l3) - target) for q in self_motion_q]
    print(f"自己運動多様体上の{len(self_motion_q)}点での目標到達誤差(最大): {max(self_motion_errs):.2e}")
    assert len(self_motion_q) >= 5, "自己運動を追跡できた点が少なすぎる"
    assert max(self_motion_errs) < 1e-4

    print("\nすべてのチェックに合格しました。")

    # 可視化(1): 最小ノルムIK vs 零空間可操作度最大化IKの収束の様子
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    axes[0].plot(hist_plain["pos_err"], label="min-norm")
    axes[0].plot(hist_manip["pos_err"], label="+ manipulability in null space")
    axes[0].set_xlabel("iteration")
    axes[0].set_ylabel("position error")
    axes[0].set_yscale("log")
    axes[0].legend()
    axes[0].set_title("Primary task convergence (unaffected by null-space term)")

    axes[1].plot(hist_plain["manipulability"], label="min-norm")
    axes[1].plot(hist_manip["manipulability"], label="+ manipulability in null space")
    axes[1].set_xlabel("iteration")
    axes[1].set_ylabel("manipulability w(J)")
    axes[1].legend()
    axes[1].set_title("Secondary objective improves only with null-space term")
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "08_nullspace_convergence.png", dpi=150)

    # 可視化(2): 自己運動多様体(同じ手先位置に到達する複数の姿勢)
    fig2, ax2 = plt.subplots(figsize=(6, 6))

    def joint_positions(q):
        a, b, c = q[0], q[0] + q[1], q[0] + q[1] + q[2]
        p0 = np.array([0.0, 0.0])
        p1 = p0 + l1 * np.array([np.cos(a), np.sin(a)])
        p2 = p1 + l2 * np.array([np.cos(b), np.sin(b)])
        p3 = p2 + l3 * np.array([np.cos(c), np.sin(c)])
        return np.array([p0, p1, p2, p3])

    cmap = plt.get_cmap("viridis")
    for i, q in enumerate(self_motion_q):
        pts = joint_positions(q)
        ax2.plot(pts[:, 0], pts[:, 1], "-o", color=cmap(i / len(self_motion_q)), alpha=0.6, markersize=3)
    ax2.plot(*target, "r*", markersize=20, label="target (fixed)")
    ax2.set_aspect("equal")
    ax2.set_title("Self-motion: same end-effector position, different postures")
    ax2.legend()
    fig2.savefig(OUTPUT_DIR / "08_self_motion.png", dpi=150)
    print(f"\nグラフを保存しました: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
