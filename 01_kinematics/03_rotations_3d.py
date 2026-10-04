"""3D回転表現: 回転行列(SO(3))・軸角度(Rodrigues)・クォータニオンの相互変換と検算。

クォータニオンの定義・変換式は Lynch and Park, *Modern Robotics: Mechanics, Planning,
and Control* (Cambridge University Press, 2017), Appendix B.3 の式(B.9)(B.12)(B.15)と
一致するように実装している(quat_from_axis_angle が式B.9、rotation_from_quat が式B.12、
quat_multiply が式B.15に対応)。
"""
import numpy as np


def skew(v):
    """3次元ベクトル v の歪対称行列 [v]_x (v x u = [v]_x u を満たす)"""
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def rotation_from_axis_angle(axis, theta):
    """Rodriguesの回転公式: R = I + sin(theta) K + (1-cos(theta)) K^2"""
    axis = axis / np.linalg.norm(axis)
    K = skew(axis)
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


def axis_angle_from_rotation(R):
    """回転行列から軸角度表現を復元する(theta away from 0, pi を仮定)。"""
    theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))
    if np.isclose(theta, 0):
        return np.array([1.0, 0.0, 0.0]), 0.0
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(theta))
    return axis, theta


def quat_from_axis_angle(axis, theta):
    """q = [cos(theta/2), sin(theta/2) axis] (Hamilton, [w,x,y,z]順)"""
    axis = axis / np.linalg.norm(axis)
    return np.concatenate([[np.cos(theta / 2)], np.sin(theta / 2) * axis])


def quat_multiply(q1, q2):
    """ハミルトン積 q1 ⊗ q2。この積は、対応する回転の合成 R1 R2 に対応する。"""
    w1, v1 = q1[0], q1[1:]
    w2, v2 = q2[0], q2[1:]
    w = w1 * w2 - v1 @ v2
    v = w1 * v2 + w2 * v1 + np.cross(v1, v2)
    return np.concatenate([[w], v])


def quat_conjugate(q):
    return np.concatenate([[q[0]], -q[1:]])


def rotation_from_quat(q):
    """単位クォータニオン q=[w,x,y,z] から回転行列を作る。"""
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x**2 + z**2), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x**2 + y**2)],
        ]
    )


def quat_from_rotation(R):
    """回転行列からクォータニオンを作る(トレースベースの標準的な方法)。w>=0の代表元を返す。"""
    tr = np.trace(R)
    w = np.sqrt(max(0, 1 + tr)) / 2
    x = np.sign(R[2, 1] - R[1, 2] + 1e-30) * np.sqrt(max(0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sign(R[0, 2] - R[2, 0] + 1e-30) * np.sqrt(max(0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sign(R[1, 0] - R[0, 1] + 1e-30) * np.sqrt(max(0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    return np.array([w, x, y, z])


def main():
    rng = np.random.default_rng(0)

    # (1) 回転行列は直交行列で行列式1(SO(3)の定義そのものの検算)
    max_orth_err, max_det_err = 0.0, 0.0
    for _ in range(200):
        axis = rng.normal(size=3)
        theta = rng.uniform(-np.pi, np.pi)
        R = rotation_from_axis_angle(axis, theta)
        max_orth_err = max(max_orth_err, np.max(np.abs(R.T @ R - np.eye(3))))
        max_det_err = max(max_det_err, abs(np.linalg.det(R) - 1))
    print(f"R^T R = I の最大誤差: {max_orth_err:.2e}")
    print(f"det(R) = 1 の最大誤差: {max_det_err:.2e}")
    assert max_orth_err < 1e-10 and max_det_err < 1e-10

    # (2) 軸角度 -> R -> 軸角度 の往復(theta, axisともに復元できるか)
    max_round_trip_err = 0.0
    for _ in range(200):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        theta = rng.uniform(0.1, np.pi - 0.1)  # 0, pi近傍(退化)は除く
        R = rotation_from_axis_angle(axis, theta)
        axis2, theta2 = axis_angle_from_rotation(R)
        R2 = rotation_from_axis_angle(axis2, theta2)
        max_round_trip_err = max(max_round_trip_err, np.max(np.abs(R - R2)))
    print(f"\n軸角度->R->軸角度->R の往復誤差: {max_round_trip_err:.2e}")
    assert max_round_trip_err < 1e-8

    # (3) クォータニオンの積が回転の合成に対応するか: R(q1*q2) == R(q1) R(q2)
    max_compose_err = 0.0
    for _ in range(200):
        axis1, theta1 = rng.normal(size=3), rng.uniform(-np.pi, np.pi)
        axis2, theta2 = rng.normal(size=3), rng.uniform(-np.pi, np.pi)
        q1, q2 = quat_from_axis_angle(axis1, theta1), quat_from_axis_angle(axis2, theta2)
        R1, R2 = rotation_from_axis_angle(axis1, theta1), rotation_from_axis_angle(axis2, theta2)
        R_from_quat_mult = rotation_from_quat(quat_multiply(q1, q2))
        max_compose_err = max(max_compose_err, np.max(np.abs(R_from_quat_mult - R1 @ R2)))
    print(f"クォータニオン積 R(q1*q2) と行列積 R(q1)R(q2) の最大誤差: {max_compose_err:.2e}")
    assert max_compose_err < 1e-8

    # (4) 二重被覆: q と -q は同じ回転を表す
    axis, theta = rng.normal(size=3), rng.uniform(-np.pi, np.pi)
    q = quat_from_axis_angle(axis, theta)
    double_cover_err = np.max(np.abs(rotation_from_quat(q) - rotation_from_quat(-q)))
    print(f"\n二重被覆 R(q) == R(-q) の誤差: {double_cover_err:.2e}")
    assert double_cover_err < 1e-10

    # (5) 共役 = 逆回転: R(q^*) == R(q)^T
    conj_err = np.max(np.abs(rotation_from_quat(quat_conjugate(q)) - rotation_from_quat(q).T))
    print(f"共役クォータニオン R(q*) == R(q)^T の誤差: {conj_err:.2e}")
    assert conj_err < 1e-10

    # (6) R -> q -> R の往復
    max_r2q2r_err = 0.0
    for _ in range(200):
        axis, theta = rng.normal(size=3), rng.uniform(0.1, np.pi - 0.1)
        R = rotation_from_axis_angle(axis, theta)
        R_round = rotation_from_quat(quat_from_rotation(R))
        max_r2q2r_err = max(max_r2q2r_err, np.max(np.abs(R - R_round)))
    print(f"\nR->クォータニオン->R の往復誤差: {max_r2q2r_err:.2e}")
    assert max_r2q2r_err < 1e-8

    print("\nすべてのチェックに合格しました。")


if __name__ == "__main__":
    main()
