"""同次変換行列(SE(3)): 回転Rと並進pをまとめて4x4行列で扱い、座標変換の合成・逆変換を検算する。"""
import numpy as np


def skew(v):
    """3次元ベクトル v の歪対称行列 [v]_x (03_rotations_3d.py と同じ定義)"""
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def rotation_from_axis_angle(axis, theta):
    """Rodriguesの回転公式(03_rotations_3d.py と同じ)"""
    axis = axis / np.linalg.norm(axis)
    K = skew(axis)
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


def make_transform(R, p):
    """回転R, 並進pから同次変換行列 T = [[R, p], [0, 1]] を作る。"""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def transform_inverse(T):
    """T^-1 = [[R^T, -R^T p], [0, 1]] (直交行列の逆行列が転置であることを利用)"""
    R, p = T[:3, :3], T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ p
    return T_inv


def transform_point(T, point):
    """3次元の点に同次変換を作用させる。"""
    return (T @ np.append(point, 1.0))[:3]


def main():
    rng = np.random.default_rng(0)

    # (1) T @ T^-1 = I
    max_inv_err = 0.0
    for _ in range(200):
        axis, theta = rng.normal(size=3), rng.uniform(-np.pi, np.pi)
        R = rotation_from_axis_angle(axis, theta)
        p = rng.normal(size=3)
        T = make_transform(R, p)
        max_inv_err = max(max_inv_err, np.max(np.abs(T @ transform_inverse(T) - np.eye(4))))
    print(f"T @ T^-1 = I の最大誤差: {max_inv_err:.2e}")
    assert max_inv_err < 1e-10

    # (2) 合成: T1 を作用させてからT2を作用させることは T2 @ T1 を1回作用させることと同じ
    #     (T2 @ T1) @ x = T2 @ (T1 @ x))
    max_compose_err = 0.0
    for _ in range(200):
        T1 = make_transform(rotation_from_axis_angle(rng.normal(size=3), rng.uniform(-np.pi, np.pi)), rng.normal(size=3))
        T2 = make_transform(rotation_from_axis_angle(rng.normal(size=3), rng.uniform(-np.pi, np.pi)), rng.normal(size=3))
        point = rng.normal(size=3)
        direct = transform_point(T2 @ T1, point)
        sequential = transform_point(T2, transform_point(T1, point))
        max_compose_err = max(max_compose_err, np.max(np.abs(direct - sequential)))
    print(f"\n合成 T2@T1 を1回適用 == T1適用後にT2適用、の最大誤差: {max_compose_err:.2e}")
    assert max_compose_err < 1e-10

    # (3) 合成の逆行列は逆順の積: (T2 @ T1)^-1 == T1^-1 @ T2^-1
    max_inv_compose_err = 0.0
    for _ in range(200):
        T1 = make_transform(rotation_from_axis_angle(rng.normal(size=3), rng.uniform(-np.pi, np.pi)), rng.normal(size=3))
        T2 = make_transform(rotation_from_axis_angle(rng.normal(size=3), rng.uniform(-np.pi, np.pi)), rng.normal(size=3))
        lhs = transform_inverse(T2 @ T1)
        rhs = transform_inverse(T1) @ transform_inverse(T2)
        max_inv_compose_err = max(max_inv_compose_err, np.max(np.abs(lhs - rhs)))
    print(f"(T2@T1)^-1 == T1^-1 @ T2^-1 の最大誤差: {max_inv_compose_err:.2e}")
    assert max_inv_compose_err < 1e-10

    print("\nすべてのチェックに合格しました。")


if __name__ == "__main__":
    main()
