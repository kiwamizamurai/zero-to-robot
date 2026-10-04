"""1関節アーム: 順運動学(FK)と逆運動学(IK)の往復チェック"""
import numpy as np

l = 1.0

fk = lambda th: np.array([l * np.cos(th), l * np.sin(th)])
ik = lambda p: np.arctan2(p[1], p[0])


def main():
    rng = np.random.default_rng(0)
    thetas = rng.uniform(-np.pi, np.pi, size=200)

    # 角度 -> 位置 -> 角度 の往復
    recovered = np.array([ik(fk(th)) for th in thetas])
    ok_theta_roundtrip = np.allclose(np.sin(thetas), np.sin(recovered)) and np.allclose(
        np.cos(thetas), np.cos(recovered)
    )

    # 位置 -> 角度 -> 位置 の往復（単位円上の点）
    points = np.array([fk(th) for th in rng.uniform(-np.pi, np.pi, size=200)])
    recovered_points = np.array([fk(ik(p)) for p in points])
    ok_point_roundtrip = np.allclose(points, recovered_points, atol=1e-9)

    p = np.array([0.6, 0.8])
    print(f"fk(ik([0.6, 0.8])) = {fk(ik(p))}")
    print(f"角度往復(200サンプル): {'OK' if ok_theta_roundtrip else 'NG'}")
    print(f"位置往復(200サンプル): {'OK' if ok_point_roundtrip else 'NG'}")

    assert ok_theta_roundtrip and ok_point_roundtrip
    print("すべてのチェックに合格しました。")


if __name__ == "__main__":
    main()
