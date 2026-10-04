"""DHパラメータ(Denavit-Hartenberg記法)による、任意の関節数を持つ直列マニピュレータの順運動学。

ここでは「標準(古典)DH」規約 A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i) を採用する
(Spong, Vidyasagar and Hutchinson, *Robot Modeling and Control*, 2005 と同じ規約)。
これは、Craigの教科書で使われる「修正DH」規約
T_(i-1,i) = Rx(alpha_(i-1)) Tx(a_(i-1)) Tz(d_i) Rz(theta_i)
(Lynch and Park, *Modern Robotics*, 2017, Appendix C.1 にも同じ形で記載)とは
積の順序・添字の付け方が異なるので注意。両者は同じロボットに対して異なる
パラメータ値を与えるため、単純に読み替えることはできない。
"""
import numpy as np


def rotz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def rotx(alpha):
    c, s = np.cos(alpha), np.sin(alpha)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])


def transz(d):
    T = np.eye(4)
    T[2, 3] = d
    return T


def transx(a):
    T = np.eye(4)
    T[0, 3] = a
    return T


def dh_transform(a, alpha, d, theta):
    """標準DH変換 A_i = Rz(theta) Tz(d) Tx(a) Rx(alpha) の閉じた式。"""
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


def dh_transform_by_composition(a, alpha, d, theta):
    """dh_transform の定義通り、4つの初等変換の積として組み立てたもの(検算用)。"""
    return rotz(theta) @ transz(d) @ transx(a) @ rotx(alpha)


def forward_kinematics_dh(dh_table, thetas):
    """dh_table: [(a_1,alpha_1,d_1), ...] のリスト、thetas: 関節角のリスト。
    戻り値: 手先の同次変換行列 T_0^n = A_1 A_2 ... A_n"""
    T = np.eye(4)
    for (a, alpha, d), theta in zip(dh_table, thetas):
        T = T @ dh_transform(a, alpha, d, theta)
    return T


def fk_2link_planar(q, l1=1.0, l2=0.8):
    """01_kinematics/02_ik_gradient_descent.py の fk(q) と同じ定義(検算用に再掲)。"""
    a, b = q[0], q[0] + q[1]
    return np.array([l1 * np.cos(a) + l2 * np.cos(b), l1 * np.sin(a) + l2 * np.sin(b)])


def main():
    rng = np.random.default_rng(0)

    # (1) dh_transform(閉じた式) と dh_transform_by_composition(定義通りの積)が一致するか
    max_def_err = 0.0
    for _ in range(200):
        a, alpha, d, theta = rng.uniform(-2, 2, size=4)
        max_def_err = max(
            max_def_err,
            np.max(np.abs(dh_transform(a, alpha, d, theta) - dh_transform_by_composition(a, alpha, d, theta))),
        )
    print(f"DH変換の閉じた式 == 4つの初等変換の積、の最大誤差: {max_def_err:.2e}")
    assert max_def_err < 1e-10

    # (2) 2関節平面アーム(alpha=d=0)として使うと、02_ik_gradient_descent.py の fk(q) と一致するか
    l1, l2 = 1.0, 0.8
    dh_table = [(l1, 0.0, 0.0), (l2, 0.0, 0.0)]
    max_planar_err = 0.0
    for _ in range(200):
        q = rng.uniform(-np.pi, np.pi, size=2)
        T = forward_kinematics_dh(dh_table, q)
        p_dh = T[:2, 3]  # z成分は0のはず(平面内の運動)
        p_2d = fk_2link_planar(q, l1, l2)
        max_planar_err = max(max_planar_err, np.max(np.abs(p_dh - p_2d)), abs(T[2, 3]))
    print(f"\nDH(alpha=d=0)によるFK == 2関節平面アームのfk(q)、の最大誤差: {max_planar_err:.2e}")
    assert max_planar_err < 1e-10

    # (3) 3次元の例: 肩ヨー・肩ピッチ・肘ピッチを持つ3自由度アーム
    #     肩ヨー(z軸周り)の後、肩ピッチ・肘ピッチをx軸周りの回転(alpha=90度で軸を振る)として構成
    l1, l2, l3 = 0.5, 1.0, 0.8
    dh_table_3d = [
        (0.0, np.pi / 2, 0.0),  # 肩ヨー: 次のリンクの回転軸をx軸周りに90度振る
        (l2, 0.0, 0.0),  # 肩ピッチ
        (l3, 0.0, 0.0),  # 肘ピッチ
    ]
    q3 = np.array([0.3, 0.5, -0.4])
    T3 = forward_kinematics_dh(dh_table_3d, q3)
    print(f"\n3自由度アームの例: 手先位置 = {T3[:3, 3]}")
    # 全関節角=0なら、腕はx軸方向に伸びきった姿勢になるはず
    # (関節1のalpha=pi/2は「次のリンクの回転軸をx軸周りに90度振る」だけで、
    #  並進(a=0)は起こさないため、theta=0では関節2・3の並進Tx(l2), Tx(l3)が
    #  そのままx軸方向に積み重なる)
    T3_zero = forward_kinematics_dh(dh_table_3d, [0.0, 0.0, 0.0])
    expected_zero = np.array([l2 + l3, 0.0, 0.0])
    zero_pose_err = np.max(np.abs(T3_zero[:3, 3] - expected_zero))
    print(f"全関節角0での手先位置 = {T3_zero[:3,3]} (期待値 [{l2+l3},0,0], 誤差 {zero_pose_err:.2e})")
    assert zero_pose_err < 1e-10

    # 肩ヨー(q1)だけを動かすと、腕全体がz軸周りに回転するはず(q1がz軸周りのヨー関節であることの確認)
    max_yaw_err = 0.0
    for q1 in rng.uniform(-np.pi, np.pi, size=50):
        T = forward_kinematics_dh(dh_table_3d, [q1, 0.0, 0.0])
        expected = np.array([np.cos(q1) * (l2 + l3), np.sin(q1) * (l2 + l3), 0.0])
        max_yaw_err = max(max_yaw_err, np.max(np.abs(T[:3, 3] - expected)))
    print(f"肩ヨーq1のみ動かした時、手先がz軸周りに回転するか: 最大誤差 {max_yaw_err:.2e}")
    assert max_yaw_err < 1e-10

    print("\nすべてのチェックに合格しました。")


if __name__ == "__main__":
    main()
