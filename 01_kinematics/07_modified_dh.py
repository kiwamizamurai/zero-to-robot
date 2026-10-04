"""修正DH(Craig)規約: T_(i-1,i) = Rx(alpha_(i-1)) Tx(a_(i-1)) Tz(d_i) Rz(theta_i) の実装と、
05_dh_parameters.py の標準(古典)DH規約との関係の検算。

両規約は同じロボットに異なるパラメータ値を与える(単純な読み替えはできない)が、
標準DHのパラメータ表 {(a_i,alpha_i,d_i,theta_i)} を1つずらして
(a_i^mod, alpha_i^mod) := (a_(i-1), alpha_(i-1))(a_0=alpha_0=0)とした修正DH表は、
「最後のリンクのTx(a_n)Rx(alpha_n)を除いた」同じ手先位置を与える、という厳密な関係がある
(notes.typ 11節で証明)。
"""
import sys
from importlib import import_module
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
_std = import_module("05_dh_parameters")


def modified_dh_transform(a_prev, alpha_prev, d, theta):
    """修正DH変換 T_(i-1,i) = Rx(alpha_(i-1)) Tx(a_(i-1)) Tz(d_i) Rz(theta_i) の閉じた式。"""
    ca, sa = np.cos(alpha_prev), np.sin(alpha_prev)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array(
        [
            [ct, -st, 0, a_prev],
            [st * ca, ct * ca, -sa, -d * sa],
            [st * sa, ct * sa, ca, d * ca],
            [0, 0, 0, 1],
        ]
    )


def modified_dh_transform_by_composition(a_prev, alpha_prev, d, theta):
    """modified_dh_transform の定義通り、4つの初等変換の積として組み立てたもの(検算用)。"""
    return _std.rotx(alpha_prev) @ _std.transx(a_prev) @ _std.transz(d) @ _std.rotz(theta)


def forward_kinematics_modified_dh(mod_dh_table, thetas):
    """mod_dh_table: [(a_(i-1), alpha_(i-1), d_i), ...] のリスト(添字は標準DH表と揃えて0始まりの
    最初のリンクパラメータはa_0=alpha_0=0を渡す)、thetas: 関節角のリスト。
    戻り値: T_0^n = A_1^mod A_2^mod ... A_n^mod"""
    T = np.eye(4)
    for (a_prev, alpha_prev, d), theta in zip(mod_dh_table, thetas):
        T = T @ modified_dh_transform(a_prev, alpha_prev, d, theta)
    return T


def standard_table_to_modified_table(std_dh_table):
    """標準DH表 [(a_1,alpha_1,d_1), ..., (a_n,alpha_n,d_n)] から、
    notes.typ 定理11.1の対応 (a_i^mod,alpha_i^mod):=(a_(i-1),alpha_(i-1)) (a_0=alpha_0=0) で
    修正DH表 [(a_0,alpha_0,d_1), (a_1,alpha_1,d_2), ..., (a_(n-1),alpha_(n-1),d_n)] を作る。
    末尾の(a_n,alpha_n)は使われない(定理11.1よりT_0^n(modified)はT_0^n(standard)から
    最後のTx(a_n)Rx(alpha_n)を除いたものに一致するため)。"""
    mod_table = []
    a_prev, alpha_prev = 0.0, 0.0
    for a, alpha, d in std_dh_table:
        mod_table.append((a_prev, alpha_prev, d))
        a_prev, alpha_prev = a, alpha
    return mod_table


def main():
    rng = np.random.default_rng(0)

    # (1) modified_dh_transform(閉じた式) と modified_dh_transform_by_composition(定義通りの積)が一致するか
    max_def_err = 0.0
    for _ in range(200):
        a_prev, alpha_prev, d, theta = rng.uniform(-2, 2, size=4)
        max_def_err = max(
            max_def_err,
            np.max(
                np.abs(
                    modified_dh_transform(a_prev, alpha_prev, d, theta)
                    - modified_dh_transform_by_composition(a_prev, alpha_prev, d, theta)
                )
            ),
        )
    print(f"修正DH変換の閉じた式 == 4つの初等変換の積、の最大誤差: {max_def_err:.2e}")
    assert max_def_err < 1e-10

    # (2) notes.typ 定理11.1: T_0^n(modified) = T_0^(n-1)(standard) @ Rz(theta_n) @ Tz(d_n)
    #     を、ランダムなDHパラメータ・関節角で数値的に確認する
    n_links = 4
    std_table_full = [tuple(rng.uniform(-1, 1, size=2)) + (rng.uniform(-1, 1),) for _ in range(n_links)]
    thetas = rng.uniform(-np.pi, np.pi, size=n_links)

    max_theorem_err = 0.0
    for k in range(1, n_links + 1):
        std_table_k = std_table_full[:k]
        mod_table_k = standard_table_to_modified_table(std_table_k)
        T_mod_k = forward_kinematics_modified_dh(mod_table_k, thetas[:k])

        T_std_km1 = _std.forward_kinematics_dh(std_table_full[: k - 1], thetas[: k - 1])
        a_k, alpha_k, d_k = std_table_full[k - 1]
        theta_k = thetas[k - 1]
        T_rhs = T_std_km1 @ _std.rotz(theta_k) @ _std.transz(d_k)

        max_theorem_err = max(max_theorem_err, np.max(np.abs(T_mod_k - T_rhs)))
    print(f"\n定理11.1 T_0^k(modified) = T_0^(k-1)(standard) Rz(theta_k) Tz(d_k) の最大誤差(k=1..{n_links}): {max_theorem_err:.2e}")
    assert max_theorem_err < 1e-10

    # (3) 系11.1: T_0^n(standard) = T_0^n(modified) @ Tx(a_n) @ Rx(alpha_n)
    std_table = std_table_full
    mod_table = standard_table_to_modified_table(std_table)
    T_std = _std.forward_kinematics_dh(std_table, thetas)
    T_mod = forward_kinematics_modified_dh(mod_table, thetas)
    a_n, alpha_n, _ = std_table[-1]
    T_mod_plus_tail = T_mod @ _std.transx(a_n) @ _std.rotx(alpha_n)
    corollary_err = np.max(np.abs(T_std - T_mod_plus_tail))
    print(f"系11.1 T_0^n(standard) = T_0^n(modified) Tx(a_n) Rx(alpha_n) の誤差: {corollary_err:.2e}")
    assert corollary_err < 1e-10

    # 系の裏付け: 末尾の補正なしでは(一般に)T_0^n(standard)と一致しないことも確認
    mismatch = np.max(np.abs(T_std - T_mod))
    print(f"(参考) 補正なしT_0^n(modified)とT_0^n(standard)の食い違い: {mismatch:.4f} (一致しないのが正常)")
    assert mismatch > 1e-3

    # (4) 05_dh_parameters.py の2関節平面アーム例(alpha=d=0)でも一致するか
    l1, l2 = 1.0, 0.8
    std_planar = [(l1, 0.0, 0.0), (l2, 0.0, 0.0)]
    mod_planar = standard_table_to_modified_table(std_planar)
    q = np.array([0.4, -0.6])
    T_std_planar = _std.forward_kinematics_dh(std_planar, q)
    T_mod_planar = forward_kinematics_modified_dh(mod_planar, q)
    a_n, alpha_n, _ = std_planar[-1]
    planar_err = np.max(np.abs(T_std_planar - T_mod_planar @ _std.transx(a_n) @ _std.rotx(alpha_n)))
    print(f"\n2関節平面アーム例での系11.1の誤差: {planar_err:.2e}")
    assert planar_err < 1e-10

    print("\nすべてのチェックに合格しました。")


if __name__ == "__main__":
    main()
