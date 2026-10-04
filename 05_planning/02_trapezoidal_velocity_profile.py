"""関節空間の台形速度プロファイル(trapezoidal velocity profile, LSPB)と、
速度上限v・加速度上限aが与えられたときの時間最適性の数値検算。

Lynch and Park, *Modern Robotics*, 2017, 9.2.2.2節の設定・式(9.14)-(9.24)に基づく。
同節が演習問題(Exercise 8, 10, 11, 12)に委ねている証明は notes.typ 4-5節で
自前で導出・証明し、ここではその数値的な裏付けを取る。
"""
import sys
from importlib import import_module
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
_poly = import_module("01_polynomial_trajectories")


def trapezoidal_time_scaling(t, T, v, a):
    """速度上限v・加速度上限aを使った3段階(加速・等速・減速)の時間スケーリング。
    v^2/a <= 1 が3段階(coast区間が存在)になるための条件(notes.typ 定理4.3)。"""
    assert v**2 / a <= 1 + 1e-9, "v^2/a > 1: 3段階の台形プロファイルにならない(bang-bangになる)"
    t = np.atleast_1d(np.asarray(t, dtype=float))
    ta = v / a
    s = np.zeros_like(t)
    s_dot = np.zeros_like(t)
    s_ddot = np.zeros_like(t)

    m1 = t <= ta
    s_ddot[m1] = a
    s_dot[m1] = a * t[m1]
    s[m1] = 0.5 * a * t[m1] ** 2

    m2 = (t > ta) & (t <= T - ta)
    s_ddot[m2] = 0.0
    s_dot[m2] = v
    s[m2] = v * t[m2] - v**2 / (2 * a)

    m3 = t > T - ta
    s_ddot[m3] = -a
    s_dot[m3] = a * (T - t[m3])
    s[m3] = (2 * a * v * T - 2 * v**2 - a**2 * (t[m3] - T) ** 2) / (2 * a)

    return s, s_dot, s_ddot


def bang_bang_time_scaling(t, T, a):
    """v制約が効かない場合(v^2/a>1)の2段階bang-bangプロファイル。
    中間点T/2で加速から減速へ切り替わる(notes.typ 定理4.5のRemarkに対応)。"""
    t = np.atleast_1d(np.asarray(t, dtype=float))
    s = np.zeros_like(t)
    s_dot = np.zeros_like(t)
    s_ddot = np.zeros_like(t)

    m1 = t <= T / 2
    s_ddot[m1] = a
    s_dot[m1] = a * t[m1]
    s[m1] = 0.5 * a * t[m1] ** 2

    m2 = t > T / 2
    s_ddot[m2] = -a
    s_dot[m2] = a * (T - t[m2])
    s[m2] = (2 * a * (T / 2) * T - 2 * (a * T / 2) ** 2 - a**2 * (t[m2] - T) ** 2) / (2 * a)

    return s, s_dot, s_ddot


def min_time_given_v_a(v, a):
    """v,aを固定して s(T)=1 を満たす最小時間 T = (a+v^2)/(va)(notes.typ 系4.2 (a)。"""
    assert v**2 / a <= 1 + 1e-9
    return (a + v**2) / (v * a)


def accel_given_v_T(v, T):
    """v,Tを固定して s(T)=1 を満たす a = v^2/(vT-1)(notes.typ 系4.2 (b))。vT>1が必要。"""
    assert v * T > 1, "vT > 1 が a>0 の必要条件"
    return v**2 / (v * T - 1)


def velocity_given_a_T(a, T):
    """a,Tを固定して s(T)=1 を満たす v(notes.typ 系4.2 (c))。aT^2>=4が必要。
    v^2 - aT v + a = 0 の2解のうち、3段階条件 v<=aT/2 を満たす小さい方の根を取る。"""
    disc = a * (a * T**2 - 4)
    assert disc >= -1e-9, "aT^2 < 4: この加速度では時間T内に到達不可能"
    disc = max(disc, 0.0)
    return 0.5 * (a * T - np.sqrt(disc))


def main():
    OUTPUT_DIR = Path(__file__).parent / "output"
    OUTPUT_DIR.mkdir(exist_ok=True)

    v, a = 0.8, 1.0
    assert v**2 / a <= 1.0
    T = min_time_given_v_a(v, a)
    print(f"v={v}, a={a} -> 最小時間 T={T:.6f}")

    # (1) 境界条件: s(0)=0, s_dot(0)=0, s(T)=1, s_dot(T)=0
    s_bc, sdot_bc, _ = trapezoidal_time_scaling(np.array([0.0, T]), T, v, a)
    pos_err = max(abs(s_bc[0] - 0.0), abs(s_bc[1] - 1.0))
    vel_err = max(abs(sdot_bc[0]), abs(sdot_bc[1]))
    print(f"境界条件誤差: 位置={pos_err:.2e}, 速度={vel_err:.2e}")
    assert pos_err < 1e-10 and vel_err < 1e-10

    # (2) s, s_dot は連続、s_ddot は加速・等速・減速の切替点で不連続にジャンプする
    ta = v / a
    eps = 1e-7
    s_dot_left, s_dot_right = [], []
    for tp in [ta, T - ta]:
        _, sd_l, _ = trapezoidal_time_scaling(np.array([tp - eps]), T, v, a)
        _, sd_r, _ = trapezoidal_time_scaling(np.array([tp + eps]), T, v, a)
        s_dot_left.append(sd_l[0])
        s_dot_right.append(sd_r[0])
    vel_continuity_err = max(abs(s_dot_left[0] - s_dot_right[0]), abs(s_dot_left[1] - s_dot_right[1]))
    print(f"速度の連続性誤差(切替点): {vel_continuity_err:.2e}")
    assert vel_continuity_err < 1e-5

    _, _, sddot_left1 = trapezoidal_time_scaling(np.array([ta - eps]), T, v, a)
    _, _, sddot_right1 = trapezoidal_time_scaling(np.array([ta + eps]), T, v, a)
    accel_jump = abs(sddot_left1[0] - sddot_right1[0])
    print(f"t=ta での加速度のジャンプ: {accel_jump:.4f} (=a={a}であるべき、5次多項式と違い不連続)")
    assert abs(accel_jump - a) < 1e-4

    # (3) 閉じた式 v*T - v^2/a = 1 の数値検算(積分によるs(T)の再計算)
    t_fine = np.linspace(0, T, 200_000)
    _, sdot_fine, _ = trapezoidal_time_scaling(t_fine, T, v, a)
    s_T_numeric = np.trapezoid(sdot_fine, t_fine)
    closure_err = abs(s_T_numeric - 1.0)
    print(f"\n閉じた式 vT-v^2/a=1 の数値積分による検算: s(T)(積分)={s_T_numeric:.8f}, 誤差={closure_err:.2e}")
    assert closure_err < 1e-6

    # (4) 3つのパラメータ化(v,a -> T / v,T -> a / a,T -> v)の往復一致
    a_recovered = accel_given_v_T(v, T)
    v_recovered = velocity_given_a_T(a, T)
    print(f"\n(v,T)からa逆算: a={a_recovered:.8f} (真値{a}との誤差={abs(a_recovered - a):.2e})")
    print(f"(a,T)からv逆算: v={v_recovered:.8f} (真値{v}との誤差={abs(v_recovered - v):.2e})")
    assert abs(a_recovered - a) < 1e-8
    assert abs(v_recovered - v) < 1e-8

    # (5) 実行可能性条件の検算
    #     (a) v^2/a <= 1 <=> T >= 2*ta (3段階になる)
    for vv, aa in [(0.8, 1.0), (1.2, 1.0)]:
        three_stage = vv**2 / aa <= 1.0
        T_would_be = (aa + vv**2) / (vv * aa)
        has_coast = T_would_be >= 2 * (vv / aa) - 1e-9
        assert three_stage == has_coast, f"v^2/a<=1 と T>=2ta の同値性が崩れた(v={vv},a={aa})"
    print("\nv^2/a<=1 <=> T>=2ta (3段階になる条件)の同値性を確認: OK")

    #     (b) (v,T)を固定したとき、vT>1 が a>0 の必要条件、vT<=2 が3段階の必要条件
    T_fixed = 2.0
    for vv in [0.3, 0.6, 0.9, 1.1]:
        vT = vv * T_fixed
        if vT > 1:
            aa = accel_given_v_T(vv, T_fixed)
            assert aa > 0
            ta_check = vv / aa
            three_stage_ok = ta_check <= T_fixed / 2 + 1e-9
            assert three_stage_ok == (vT <= 2 + 1e-9), f"vT<=2 <-> 3段階 の同値性が崩れた(v={vv})"
        else:
            try:
                accel_given_v_T(vv, T_fixed)
                raise AssertionError("vT<=1 なのに a>0 の解が求まってしまった")
            except AssertionError as e:
                assert "vT > 1" in str(e) or "vT>1" in str(e)
    print("vT>1 (a>0の必要条件) と 1<vT<=2 (3段階の必要十分条件)を確認: OK")

    #     (c) (a,T)を固定したとき、aT^2>=4 が実行可能性の必要条件
    for aa in [0.5, 1.0, 2.0]:
        aT2 = aa * T_fixed**2
        if aT2 >= 4:
            vv = velocity_given_a_T(aa, T_fixed)
            assert vv <= aa * T_fixed / 2 + 1e-9, "3段階条件 v<=aT/2 を満たす根を選べていない"
            s_check, sdot_check, _ = trapezoidal_time_scaling(np.array([T_fixed]), T_fixed, vv, aa) if vv**2/aa <= 1+1e-9 else (None, None, None)
        else:
            try:
                velocity_given_a_T(aa, T_fixed)
                raise AssertionError("aT^2<4 なのに解が求まってしまった")
            except AssertionError as e:
                assert "aT^2" in str(e)
    print("aT^2>=4 (実行可能性の必要条件)を確認: OK")

    # (6) bang-bang(2段階)極限: v^2/a>1 のとき、境界 aT^2=4 で v=aT/2 に一致する
    a_bb = 1.0
    T_bb = 2 / np.sqrt(a_bb)  # aT^2=4 を満たす T
    v_limit = velocity_given_a_T(a_bb, T_bb)
    v_expected = a_bb * T_bb / 2
    print(f"\nbang-bang極限(aT^2=4): v(a,T)から逆算={v_limit:.8f}, aT/2={v_expected:.8f}")
    assert abs(v_limit - v_expected) < 1e-8
    s_bb, sdot_bb, _ = bang_bang_time_scaling(np.array([0.0, T_bb / 2, T_bb]), T_bb, a_bb)
    print(f"bang-bang: s(0)={s_bb[0]:.2e}, s(T)={s_bb[-1]:.6f}, 中間点速度={sdot_bb[1]:.6f} (=a*T/2={a_bb*T_bb/2:.6f}であるべき)")
    assert abs(s_bb[-1] - 1.0) < 1e-6
    assert abs(sdot_bb[1] - a_bb * T_bb / 2) < 1e-6

    # (7) 時間最適性の数値的illustration: 同じ(v,a)制約のもとで、
    #     5次多項式(quintic)時間スケーリングが同じ制約を守るには台形よりも長い時間が必要になることを確認する。
    #     quintic: s(tau)=10tau^3-15tau^4+6tau^5, tau=t/T。ピーク速度・加速度はtau=0.5付近で数値的に評価する。
    tau = np.linspace(0, 1, 200_001)
    s_q, sdot_q, sddot_q, _ = _poly.quintic_time_scaling(tau, 1.0)  # T=1 で規格化(s,sdot,sddotはtau微分に相当)
    peak_sdot_normalized = sdot_q.max()  # = ds/dtau の最大値
    peak_sddot_normalized = sddot_q.max()  # = d^2s/dtau^2 の最大値

    # theta(t)=s(t/T_q)とおくと、d s/dt = peak_sdot_normalized/T_q <= v, d^2s/dt^2 = peak_sddot_normalized/T_q^2 <= a
    # を両方満たす最小の T_q を求める(速度制約と加速度制約それぞれから下限を出し、大きい方を取る)
    T_q_from_v = peak_sdot_normalized / v
    T_q_from_a = np.sqrt(peak_sddot_normalized / a)
    T_q = max(T_q_from_v, T_q_from_a)
    print(f"\n同じ(v={v},a={a})制約下で: 台形の最小時間 T*={T:.6f}, quinticが同じ制約を守るのに必要な時間 T_q={T_q:.6f}")
    assert T_q > T, "quinticが台形より速いという矛盾した結果になった(時間最適性の反例)"
    print(f"T_q - T* = {T_q - T:.6f} > 0: 台形(bang-bang型)がquinticより速いことを確認(時間最適性の数値的裏付け)")

    print("\nすべてのチェックに合格しました。")

    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    t_axis = np.linspace(0, T, 1000)
    s_ax, sdot_ax, sddot_ax = trapezoidal_time_scaling(t_axis, T, v, a)
    for ax, (y, label) in zip(axes, [(s_ax, "s(t)"), (sdot_ax, "s_dot(t)"), (sddot_ax, "s_ddot(t)")]):
        ax.plot(t_axis, y)
        ax.set_ylabel(label)
        ax.axvline(ta, color="gray", linestyle="--", linewidth=0.8)
        ax.axvline(T - ta, color="gray", linestyle="--", linewidth=0.8)
    axes[-1].set_xlabel("time [s]")
    axes[0].set_title(f"Trapezoidal velocity profile (v={v}, a={a}, T={T:.3f})")
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "02_trapezoidal_velocity_profile.png", dpi=150)

    fig2, ax2 = plt.subplots()
    a_sweep = np.linspace(0.3, 3.0, 100)
    T_sweep = [min_time_given_v_a(v, aa) if v**2 / aa <= 1 else 2 / np.sqrt(aa) for aa in a_sweep]
    ax2.plot(a_sweep, T_sweep)
    ax2.set_xlabel("acceleration limit a")
    ax2.set_ylabel("minimum time T")
    ax2.set_title(f"Minimum time vs acceleration limit (v={v} fixed)")
    fig2.savefig(OUTPUT_DIR / "02_min_time_vs_accel.png", dpi=150)
    print(f"\nグラフを保存しました: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
