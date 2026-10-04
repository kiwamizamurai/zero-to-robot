"""3D微分運動学: 一般化された幾何ヤコビ行列(z_{i-1} x (o_n - o_{i-1}))と、特異点の検出。

ヤコビ行列の列を「他の関節を固定してi番目の関節だけを単位角速度で動かしたときの
手先速度」として幾何学的に導出する考え方、および可操作度による特異点の解析は、
Asada, *Introduction to Robotics* (MIT 2.12, Fall 2005, OCW) の第5章 "Differential
Motion" に基づく。同章の2関節平面アームの特異配置(肘が伸びきる/折れきる: theta_2=0,pi)は、
本スクリプトの3自由度アームでq3(肘)について見つかる特異配置と同じ構造をしている。
"""
import numpy as np


def dh_transform(a, alpha, d, theta):
    """標準DH変換(05_dh_parameters.py と同じ定義)"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1],
        ]
    )


def forward_kinematics_dh(dh_table, thetas):
    T = np.eye(4)
    for (a, alpha, d), theta in zip(dh_table, thetas):
        T = T @ dh_transform(a, alpha, d, theta)
    return T


def forward_kinematics_all_frames(dh_table, thetas):
    """各関節までの累積変換 T_0^0=I, T_0^1, ..., T_0^n を全て返す(ヤコビアン計算に必要)。"""
    transforms = [np.eye(4)]
    T = np.eye(4)
    for (a, alpha, d), theta in zip(dh_table, thetas):
        T = T @ dh_transform(a, alpha, d, theta)
        transforms.append(T)
    return transforms


def geometric_jacobian(dh_table, thetas):
    """全関節が回転関節であるときの幾何ヤコビ行列(6 x n)。
    01_kinematics/notes.typ の Remark(J_i = z_{i-1} x (o_n - o_{i-1}))の3D・一般関節数への拡張。"""
    transforms = forward_kinematics_all_frames(dh_table, thetas)
    o_n = transforms[-1][:3, 3]
    n = len(thetas)
    J = np.zeros((6, n))
    for i in range(n):
        z_prev = transforms[i][:3, 2]  # frame {i-1} の z軸(回転関節iの軸)
        o_prev = transforms[i][:3, 3]
        J[:3, i] = np.cross(z_prev, o_n - o_prev)
        J[3:, i] = z_prev
    return J


def numerical_jacobian_position(dh_table, thetas, eps=1e-6):
    """有限差分による位置のヤコビアン(3 x n)。検算用。"""
    thetas = np.asarray(thetas, dtype=float)
    n = len(thetas)
    J_num = np.zeros((3, n))
    for i in range(n):
        dq = np.zeros(n)
        dq[i] = eps
        p_plus = forward_kinematics_dh(dh_table, thetas + dq)[:3, 3]
        p_minus = forward_kinematics_dh(dh_table, thetas - dq)[:3, 3]
        J_num[:, i] = (p_plus - p_minus) / (2 * eps)
    return J_num


def manipulability(J):
    """可操作度 sqrt(det(J J^T))。特異点に近づくと0に近づく(Yoshikawa, 1985)。"""
    return np.sqrt(max(0.0, np.linalg.det(J @ J.T)))


def main():
    rng = np.random.default_rng(0)
    l2, l3 = 1.0, 0.8
    dh_table = [(0.0, np.pi / 2, 0.0), (l2, 0.0, 0.0), (l3, 0.0, 0.0)]

    # (1) 解析的ヤコビアン(位置部分)と有限差分の比較
    max_err = 0.0
    for _ in range(100):
        q = rng.uniform(-np.pi, np.pi, size=3)
        J_analytic = geometric_jacobian(dh_table, q)[:3]
        J_num = numerical_jacobian_position(dh_table, q)
        max_err = max(max_err, np.max(np.abs(J_analytic - J_num)))
    print(f"解析的ヤコビアン(線速度部分) vs 有限差分: 最大誤差 = {max_err:.2e}")
    assert max_err < 1e-6

    # (2) 可操作度を q2, q3 それぞれについて走査し、特異点(可操作度が0に近づく配置)を探す
    print("\nq2(肩ピッチ)を走査(q1=0.2, q3=0.5 固定):")
    for q2 in [0.0, 0.3, np.pi / 2, np.pi - 0.01]:
        J = geometric_jacobian(dh_table, [0.2, q2, 0.5])
        print(f"  q2={q2:.3f} rad -> 可操作度 = {manipulability(J[:3]):.4f}")

    print("\nq3(肘ピッチ)を走査(q1=0.2, q2=0.3 固定):")
    for q3 in [0.0, 0.3, np.pi / 2, np.pi - 0.01, -np.pi + 0.01]:
        J = geometric_jacobian(dh_table, [0.2, 0.3, q3])
        print(f"  q3={q3:.3f} rad -> 可操作度 = {manipulability(J[:3]):.4f}")

    # q3=0(腕が伸びきる)・q3=±pi(肘が折れきる)で可操作度がほぼ0になることを確認。
    # 01_kinematics/notes.typ の「肘上/肘下の2解・損失関数の非凸性」と対応する特異配置。
    m_extended = manipulability(geometric_jacobian(dh_table, [0.2, 0.3, 0.0])[:3])
    m_folded = manipulability(geometric_jacobian(dh_table, [0.2, 0.3, np.pi])[:3])
    print(
        f"\nq3=0(伸びきり)の可操作度={m_extended:.2e}, q3=pi(折れきり)の可操作度={m_folded:.2e}"
        " -> ともに特異配置(肘の自由度が手先の動きに寄与しない)"
    )
    assert m_extended < 1e-6 and m_folded < 1e-6


if __name__ == "__main__":
    main()
