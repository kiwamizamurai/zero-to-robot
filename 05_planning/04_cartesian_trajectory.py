"""カルテシアン(タスク)空間軌道: スクリューモーション経路と分離型(位置直線+姿勢測地線)
経路の実装、時間スケーリングとの組み合わせ、複数経由点(waypoint)への拡張、
そして数値IK(ヤコビアン転置/減衰最小二乗法)によるカルテシアン軌道の関節空間への変換。

設定は Lynch and Park, *Modern Robotics*, 2017, 9.1節の式(9.6)(スクリュー経路)・
式(9.7)(9.8)(分離型経路)に基づく。SO(3)の対数写像・SE(3)の指数写像(G(theta)行列)の
閉じた式は notes_cartesian.typ で自前で導出する(01_kinematics/notes_3d.typ で証明済みの
Rodriguesの回転公式・skew行列の性質を再利用する)。
"""
import sys
from importlib import import_module
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent.parent / "01_kinematics"))
_poly = import_module("01_polynomial_trajectories")
_diffkin = import_module("06_differential_kinematics_3d")


def skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def so3_exp(omega_hat, theta):
    """Rodriguesの回転公式(01_kinematics/notes_3d.typ 定理2.1と同じ)。"""
    K = skew(omega_hat)
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


def so3_log(R):
    """SO(3)の対数写像: R から (axis, theta) を復元する(notes_cartesian.typ 補題1.1)。
    theta away from 0, pi を仮定(01_kinematics/03_rotations_3d.py の axis_angle_from_rotation と同じ)。"""
    theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))
    if np.isclose(theta, 0):
        return np.array([1.0, 0.0, 0.0]), 0.0
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(theta))
    return axis, theta


def G_matrix(omega_hat, theta):
    """SE(3)指数写像の並進部分に現れる行列 G(theta)=theta I+(1-cos theta)[w]+(theta-sin theta)[w]^2
    (notes_cartesian.typ 定理1.2)。"""
    K = skew(omega_hat)
    return theta * np.eye(3) + (1 - np.cos(theta)) * K + (theta - np.sin(theta)) * (K @ K)


def se3_exp(omega_hat, v, theta):
    """SE(3)の指数写像: ツイスト(omega_hat,v)とtheta から同次変換行列を復元する。"""
    R = so3_exp(omega_hat, theta)
    p = G_matrix(omega_hat, theta) @ v
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def se3_log(X):
    """SE(3)の対数写像: X=(R,p) からツイスト(omega_hat,v)とthetaを復元する。
    G(theta)v=p を解いてvを求める(G(theta)の可逆性は notes_cartesian.typ 定理1.3で証明)。"""
    R, p = X[:3, :3], X[:3, 3]
    omega_hat, theta = so3_log(R)
    if np.isclose(theta, 0):
        return omega_hat, p.copy(), 0.0  # 純平行移動: v = p/1 (G(0)=0 なので特別扱い、方向はp自体)
    v = np.linalg.solve(G_matrix(omega_hat, theta), p)
    return omega_hat, v, theta


def screw_trajectory(X_start, X_end, s):
    """スクリュー経路(Modern Robotics式9.6): X(s)=X_start exp(log(X_start^-1 X_end) s)。"""
    X_rel = np.linalg.inv(X_start) @ X_end
    omega_hat, v, theta = se3_log(X_rel)
    return X_start @ se3_exp(omega_hat, v, theta * s)


def decoupled_trajectory(X_start, X_end, s):
    """分離型経路(Modern Robotics式9.7,9.8): 位置は直線、姿勢はbody frameでの一定回転軸まわりの回転。"""
    R_start, p_start = X_start[:3, :3], X_start[:3, 3]
    R_end, p_end = X_end[:3, :3], X_end[:3, 3]
    p = p_start + s * (p_end - p_start)
    R_rel = R_start.T @ R_end
    omega_hat, theta = so3_log(R_rel)
    R = R_start @ so3_exp(omega_hat, theta * s)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def cartesian_trajectory(X_start, X_end, t, T, kind="decoupled", method="quintic"):
    """時間スケーリングs(t)と組み合わせたカルテシアン軌道 X(t)。"""
    if method == "quintic":
        s, _, _, _ = _poly.quintic_time_scaling(np.atleast_1d(t), T)
    else:
        s, _ = _poly.cubic_time_scaling(np.atleast_1d(t), T)
    s = float(s[0]) if np.ndim(t) == 0 else s
    path_fn = screw_trajectory if kind == "screw" else decoupled_trajectory
    if np.ndim(s) == 0:
        return path_fn(X_start, X_end, s)
    return [path_fn(X_start, X_end, si) for si in s]


def waypoint_trajectory(waypoints, segment_time, kind="decoupled"):
    """複数経由点(waypoint)を、各区間ごとに静止状態から静止状態への5次時間スケーリングで
    順に結ぶ(各waypointで一旦速度0になる、stop-and-go方式)。"""
    n_seg = len(waypoints) - 1
    total_T = n_seg * segment_time

    def X_of_t(t):
        seg = min(int(t // segment_time), n_seg - 1)
        t_local = t - seg * segment_time
        return cartesian_trajectory(waypoints[seg], waypoints[seg + 1], t_local, segment_time, kind=kind)

    return X_of_t, total_T


def damped_least_squares_ik(dh_table, theta_init, p_target, max_iters=200, damping=1e-3, tol=1e-8):
    """位置のみを目標とする減衰最小二乗法IK(01_kinematics/06_differential_kinematics_3d.py の
    geometric_jacobian を再利用)。"""
    theta = np.array(theta_init, dtype=float)
    for _ in range(max_iters):
        T = _diffkin.forward_kinematics_dh(dh_table, theta)
        p_current = T[:3, 3]
        err = p_target - p_current
        if np.linalg.norm(err) < tol:
            break
        J = _diffkin.geometric_jacobian(dh_table, theta)[:3]
        JJt = J @ J.T + damping**2 * np.eye(3)
        dtheta = J.T @ np.linalg.solve(JJt, err)
        theta = theta + dtheta
    return theta


def main():
    OUTPUT_DIR = Path(__file__).parent / "output"
    OUTPUT_DIR.mkdir(exist_ok=True)
    rng = np.random.default_rng(0)

    # (1) SO(3)対数写像の検算: ランダムな回転で exp(log(R))=R, log(exp(axis,theta))=(axis,theta)
    max_err_log_exp = 0.0
    max_err_exp_log = 0.0
    for _ in range(200):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        theta_true = rng.uniform(0.05, np.pi - 0.05)
        R = so3_exp(axis, theta_true)
        axis_rec, theta_rec = so3_log(R)
        # 軸の符号は theta->-theta, axis->-axis の不定性があるので、正しい方を選んで比較
        if np.dot(axis_rec, axis) < 0:
            axis_rec, theta_rec = -axis_rec, -theta_rec
        max_err_exp_log = max(max_err_exp_log, abs(theta_rec - theta_true) + np.linalg.norm(axis_rec - axis))
        R_rec = so3_exp(axis_rec, theta_rec)
        max_err_log_exp = max(max_err_log_exp, np.max(np.abs(R_rec - R)))
    print(f"SO(3)対数写像: exp(log(R))=R の誤差(最大)={max_err_log_exp:.2e}")
    print(f"SO(3)対数写像: log(exp(axis,theta))=(axis,theta) の誤差(最大)={max_err_exp_log:.2e}")
    assert max_err_log_exp < 1e-10 and max_err_exp_log < 1e-8

    # (2) SE(3)指数写像・対数写像の往復検算
    max_err_se3 = 0.0
    for _ in range(200):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        v = rng.normal(size=3)
        theta_true = rng.uniform(0.05, np.pi - 0.05)
        X = se3_exp(axis, v, theta_true)
        axis_rec, v_rec, theta_rec = se3_log(X)
        if np.dot(axis_rec, axis) < 0:
            axis_rec, v_rec, theta_rec = -axis_rec, -v_rec, -theta_rec
        X_rec = se3_exp(axis_rec, v_rec, theta_rec)
        max_err_se3 = max(max_err_se3, np.max(np.abs(X_rec - X)))
    print(f"\nSE(3)指数写像/対数写像の往復誤差(最大)={max_err_se3:.2e}")
    assert max_err_se3 < 1e-8

    # (3) スクリュー経路 vs 分離型経路: 境界条件 X(0)=X_start, X(1)=X_end
    R_start = so3_exp(np.array([0.0, 0.0, 1.0]), 0.3)
    p_start = np.array([0.3, 0.0, 0.2])
    X_start = np.eye(4)
    X_start[:3, :3], X_start[:3, 3] = R_start, p_start

    R_end = so3_exp(np.array([1.0, 0.0, 0.0]), 1.2)
    p_end = np.array([0.1, 0.4, 0.6])
    X_end = np.eye(4)
    X_end[:3, :3], X_end[:3, 3] = R_end, p_end

    for name, fn in [("screw", screw_trajectory), ("decoupled", decoupled_trajectory)]:
        X0 = fn(X_start, X_end, 0.0)
        X1 = fn(X_start, X_end, 1.0)
        err0 = np.max(np.abs(X0 - X_start))
        err1 = np.max(np.abs(X1 - X_end))
        print(f"\n{name}経路: X(0)誤差={err0:.2e}, X(1)誤差={err1:.2e}")
        assert err0 < 1e-9 and err1 < 1e-9

    # (4) 分離型経路の位置は厳密に直線、スクリュー経路の位置は直線からずれる
    s_values = np.linspace(0, 1, 50)
    positions_decoupled = np.array([decoupled_trajectory(X_start, X_end, s)[:3, 3] for s in s_values])
    positions_screw = np.array([screw_trajectory(X_start, X_end, s)[:3, 3] for s in s_values])
    straight_line = p_start[None, :] + s_values[:, None] * (p_end - p_start)[None, :]

    dev_decoupled = np.max(np.abs(positions_decoupled - straight_line))
    dev_screw = np.max(np.abs(positions_screw - straight_line))
    print(f"\n分離型経路の位置が直線からずれる量(最大)={dev_decoupled:.2e} (厳密に0であるべき)")
    print(f"スクリュー経路の位置が直線からずれる量(最大)={dev_screw:.4f} (0ではないはず)")
    assert dev_decoupled < 1e-9
    assert dev_screw > 1e-3

    # (5) 時間スケーリングとの組み合わせ: X(0)=X_start, X(T)=X_end, 各t で回転行列がSO(3)
    T_total = 3.0
    t_values = np.linspace(0, T_total, 20)
    Xs = cartesian_trajectory(X_start, X_end, t_values, T_total, kind="decoupled")
    max_orth_err = max(np.max(np.abs(X[:3, :3].T @ X[:3, :3] - np.eye(3))) for X in Xs)
    print(f"\n時間スケーリングと組み合わせた軌道: 各tでR^T R=Iの誤差(最大)={max_orth_err:.2e}")
    assert max_orth_err < 1e-9
    assert np.max(np.abs(Xs[0] - X_start)) < 1e-9
    assert np.max(np.abs(Xs[-1] - X_end)) < 1e-6

    # (6) 複数waypoint(stop-and-go)軌道: 各経由点で厳密に通過することを確認
    waypoints = [X_start, X_end]
    R_mid = so3_exp(np.array([0.0, 1.0, 0.0]), 0.8)
    p_mid = np.array([0.5, 0.3, 0.4])
    X_mid = np.eye(4)
    X_mid[:3, :3], X_mid[:3, 3] = R_mid, p_mid
    waypoints = [X_start, X_mid, X_end]
    X_of_t, T_wp = waypoint_trajectory(waypoints, segment_time=2.0, kind="decoupled")
    checkpoints = [0.0, 2.0, 4.0]
    max_wp_err = 0.0
    for tc, Xw in zip(checkpoints, waypoints):
        t_check = tc if tc < T_wp else T_wp - 1e-6
        X_actual = X_of_t(t_check)
        max_wp_err = max(max_wp_err, np.max(np.abs(X_actual - Xw)))
    print(f"\n複数waypoint軌道: 各経由点通過時の誤差(最大)={max_wp_err:.2e}")
    assert max_wp_err < 1e-4

    # (7) 数値IK: 3自由度アームでカルテシアン軌道の位置を追従できるか
    l2, l3 = 1.0, 0.8
    dh_table = [(0.0, np.pi / 2, 0.0), (l2, 0.0, 0.0), (l3, 0.0, 0.0)]

    p_start_ik = np.array([0.9, 0.3, 0.5])
    p_end_ik = np.array([0.5, -0.5, 1.0])
    theta_guess = np.array([0.1, 0.5, 0.5])
    theta_start_ik = damped_least_squares_ik(dh_table, theta_guess, p_start_ik)
    p_check = _diffkin.forward_kinematics_dh(dh_table, theta_start_ik)[:3, 3]
    print(f"\nIK検算(開始姿勢): 目標位置との誤差={np.linalg.norm(p_check - p_start_ik):.2e}")
    assert np.linalg.norm(p_check - p_start_ik) < 1e-6

    T_ik = 2.0
    t_ik = np.linspace(0, T_ik, 30)
    s_ik, _, _, _ = _poly.quintic_time_scaling(t_ik, T_ik)
    positions_ik = p_start_ik[None, :] + s_ik[:, None] * (p_end_ik - p_start_ik)[None, :]

    theta_prev = theta_start_ik
    max_tracking_err = 0.0
    thetas_along_path = []
    for p_target in positions_ik:
        theta_sol = damped_least_squares_ik(dh_table, theta_prev, p_target)
        p_actual = _diffkin.forward_kinematics_dh(dh_table, theta_sol)[:3, 3]
        max_tracking_err = max(max_tracking_err, np.linalg.norm(p_actual - p_target))
        thetas_along_path.append(theta_sol)
        theta_prev = theta_sol
    print(f"カルテシアン軌道に沿ったIK追従誤差(最大)={max_tracking_err:.2e}")
    assert max_tracking_err < 1e-6

    print("\nすべてのチェックに合格しました。")

    fig = plt.figure(figsize=(10, 5))
    ax1 = fig.add_subplot(121, projection="3d")
    ax1.plot(*positions_decoupled.T, label="decoupled")
    ax1.plot(*positions_screw.T, label="screw")
    ax1.plot(*straight_line.T, "k--", linewidth=0.8, label="straight line")
    ax1.set_title("Screw vs decoupled Cartesian path")
    ax1.legend()

    ax2 = fig.add_subplot(122, projection="3d")
    thetas_along_path = np.array(thetas_along_path)
    fk_positions = np.array([_diffkin.forward_kinematics_dh(dh_table, th)[:3, 3] for th in thetas_along_path])
    ax2.plot(*positions_ik.T, label="desired (Cartesian)")
    ax2.plot(*fk_positions.T, "--", label="FK(IK(t)) actual")
    ax2.set_title("Cartesian trajectory tracked via numerical IK")
    ax2.legend()
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "04_cartesian_trajectory.png", dpi=150)
    print(f"\nグラフを保存しました: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
