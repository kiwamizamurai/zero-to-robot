"""S字(ジャーク制限)速度プロファイル: ジャーク上限Jのもとで、台形プロファイルでは
不連続だった加速度を連続にする7段階の時間スケーリング。

設定はLynch and Park, *Modern Robotics*, 2017, Figure 9.6の定性的な7段階の記述
(1:定ジャーク+J, 2:定加速度, 3:定ジャーク-J, 4:定速度, 5:定ジャーク-J, 6:定加速度,
7:定ジャーク+J)に基づく。同書はこの構成の閉じた式を与えていないため、BYU ME 537
"S-Curve Equations for a Trajectory Generator" (Ch5, Ezekiel Rediet, et al.) の
構成(§5.5, S-curve with linear period)を参考に、notes.typ 5節で自前で導出した式
(t_j=a/J, T_r=v/a+a/J, S_ramp=v^2/(2a)+av/(2J), T=a/J+v/a+1/v)を実装・検算する。
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def _integrate_constant_jerk(s0, v0, a0, jerk, tau):
    """初期状態(s0,v0,a0)から、一定ジャークjerkのもとで経過時間tauだけ積分した(s,v,a)。"""
    s = s0 + v0 * tau + 0.5 * a0 * tau**2 + jerk * tau**3 / 6
    v = v0 + a0 * tau + 0.5 * jerk * tau**2
    a = a0 + jerk * tau
    return s, v, a


def scurve_phase_boundaries(v, a, J):
    """7段階の各フェーズの所要時間と、フェーズ境界での(s,v,a)を求める。
    a^2<=vJ(加速度プラトーに到達できる)、かつcoast区間が存在する(2*S_ramp<=1)ことを要求する。"""
    assert a**2 <= v * J + 1e-9, "a^2 > vJ: 加速度プラトーaに到達できない(5段階に退化)"
    t_j = a / J
    T_r = v / a + a / J
    S_ramp = v**2 / (2 * a) + a * v / (2 * J)
    assert 2 * S_ramp <= 1 + 1e-9, "2*S_ramp > 1: coast区間が存在しない(7段階にならない)"
    T_c = (1 - 2 * S_ramp) / v
    T = 2 * T_r + T_c
    t_lin = v / a - a / J

    durations = [t_j, t_lin, t_j, T_c, t_j, t_lin, t_j]
    jerks = [J, 0.0, -J, 0.0, -J, 0.0, J]

    boundaries = [(0.0, 0.0, 0.0)]  # (s, v, a) at start of each phase, plus final
    s, v_cur, a_cur = 0.0, 0.0, 0.0
    for dur, jerk in zip(durations, jerks):
        s, v_cur, a_cur = _integrate_constant_jerk(s, v_cur, a_cur, jerk, dur)
        boundaries.append((s, v_cur, a_cur))

    return T, t_j, t_lin, T_c, durations, jerks, boundaries


def scurve_time_scaling(t, v, a, J):
    """7段階S字プロファイル s(t), s_dot(t), s_ddot(t), s_dddot(t) を返す(T=総時間も返す)。"""
    T, t_j, t_lin, T_c, durations, jerks, boundaries = scurve_phase_boundaries(v, a, J)
    t = np.atleast_1d(np.asarray(t, dtype=float))
    s = np.zeros_like(t)
    s_dot = np.zeros_like(t)
    s_ddot = np.zeros_like(t)
    s_dddot = np.zeros_like(t)

    edges = np.cumsum([0.0] + durations)
    for i, (dur, jerk) in enumerate(zip(durations, jerks)):
        t_start, t_end = edges[i], edges[i + 1]
        if i == len(durations) - 1:
            mask = (t >= t_start - 1e-12) & (t <= t_end + 1e-9)
        else:
            mask = (t >= t_start - 1e-12) & (t < t_end - 1e-12)
        tau = t[mask] - t_start
        s0, v0, a0 = boundaries[i]
        s[mask], s_dot[mask], s_ddot[mask] = _integrate_constant_jerk(s0, v0, a0, jerk, tau)
        s_dddot[mask] = jerk

    return s, s_dot, s_ddot, s_dddot, T


def degenerate_peak_accel(v, J):
    """a^2>vJ(プラトーに到達できない)場合の、実際に到達する最大加速度 a_s=sqrt(vJ)。"""
    return np.sqrt(v * J)


def main():
    OUTPUT_DIR = Path(__file__).parent / "output"
    OUTPUT_DIR.mkdir(exist_ok=True)

    v, a, J = 0.8, 1.0, 5.0
    T, t_j, t_lin, T_c, durations, jerks, boundaries = scurve_phase_boundaries(v, a, J)
    print(f"v={v}, a={a}, J={J} -> t_j={t_j:.4f}, t_lin={t_lin:.4f}, T_c={T_c:.4f}, T={T:.6f}")
    assert t_lin > 0 and T_c > 0, "この例はt_lin>0, T_c>0(真の7段階)になるよう選んだはず"

    # (1) 境界条件: s(0)=0,s_dot(0)=0,s_ddot(0)=0, s(T)=1,s_dot(T)=0,s_ddot(T)=0
    s_bc, sdot_bc, sddot_bc, _, _ = scurve_time_scaling(np.array([0.0, T]), v, a, J)
    pos_err = max(abs(s_bc[0] - 0.0), abs(s_bc[1] - 1.0))
    vel_err = max(abs(sdot_bc[0]), abs(sdot_bc[1]))
    acc_err = max(abs(sddot_bc[0]), abs(sddot_bc[1]))
    print(f"境界条件誤差: 位置={pos_err:.2e}, 速度={vel_err:.2e}, 加速度={acc_err:.2e}")
    assert pos_err < 1e-9 and vel_err < 1e-9 and acc_err < 1e-9

    # (2) 加速度は全ての切替点で連続(台形プロファイルと違い、ジャンプしない)
    edges = np.cumsum([0.0] + durations)[1:-1]  # 6つの内部切替点
    eps = 1e-7
    max_accel_jump = 0.0
    for e in edges:
        _, _, a_l, _, _ = scurve_time_scaling(np.array([e - eps]), v, a, J)
        _, _, a_r, _, _ = scurve_time_scaling(np.array([e + eps]), v, a, J)
        max_accel_jump = max(max_accel_jump, abs(a_l[0] - a_r[0]))
    print(f"6つの内部切替点での加速度の不連続量(最大): {max_accel_jump:.2e} (台形と違い連続であるべき)")
    assert max_accel_jump < 1e-4

    # (3) ジャークは切替点で不連続にジャンプする(+J,0,-J の間を飛ぶ)
    jerk_jump_01 = abs(jerks[0] - jerks[1])
    print(f"ジャークのジャンプ(phase1->2): {jerk_jump_01:.4f} (=J={J}であるべき)")
    assert abs(jerk_jump_01 - J) < 1e-9

    # (4) 点対称性の補題: 立ち上がり区間(0<=t<=T_r)で s_dot(T_r - t) = v - s_dot(t)
    T_r = v / a + a / J
    t_test = np.linspace(0, T_r, 200)
    _, sdot_t, _, _, _ = scurve_time_scaling(t_test, v, a, J)
    _, sdot_mirror, _, _, _ = scurve_time_scaling(T_r - t_test, v, a, J)
    symmetry_err = np.max(np.abs(sdot_mirror - (v - sdot_t)))
    print(f"\n点対称性 s_dot(T_r-t)=v-s_dot(t) の誤差(最大): {symmetry_err:.2e}")
    assert symmetry_err < 1e-9

    # (5) 閉じた式 T=a/J+v/a+1/v の数値積分による検算
    t_fine = np.linspace(0, T, 400_000)
    _, sdot_fine, _, _, _ = scurve_time_scaling(t_fine, v, a, J)
    s_T_numeric = np.trapezoid(sdot_fine, t_fine)
    print(f"閉じた式 T={T:.8f} での数値積分 s(T)={s_T_numeric:.8f} (誤差={abs(s_T_numeric - 1.0):.2e})")
    assert abs(s_T_numeric - 1.0) < 1e-6

    # (6) 台形プロファイルとの比較: T_scurve = T*_trapezoidal + a/J (ちょうどa/J長い)
    from importlib import import_module
    import sys

    sys.path.insert(0, str(Path(__file__).parent))
    _trap = import_module("02_trapezoidal_velocity_profile")
    T_trap = _trap.min_time_given_v_a(v, a)
    print(f"\n台形プロファイルの最小時間 T*={T_trap:.6f}, S字プロファイルの時間 T={T:.6f}")
    print(f"T - T* = {T - T_trap:.6f} (理論値 a/J = {a / J:.6f})")
    assert abs((T - T_trap) - a / J) < 1e-9

    # (7) J->infinityの極限で台形に一致することを確認(Jを大きくするとT->T*)
    J_values = np.array([3.0, 5.0, 20.0, 100.0, 1000.0])
    T_values = []
    for Jv in J_values:
        if a**2 <= v * Jv:
            Tv, *_ = scurve_phase_boundaries(v, a, Jv)
            T_values.append(Tv)
        else:
            T_values.append(np.nan)
    T_values = np.array(T_values)
    print(f"\nJを大きくしたときのT: {T_values}")
    print(f"台形の最小時間 T*={T_trap:.6f} に収束していくことを確認")
    assert T_values[-1] - T_trap < (T_values[0] - T_trap)  # 単調にT*へ近づく

    # (8) 退化ケース(a^2>vJ, プラトーに到達できない)での実際のピーク加速度
    J_small = 0.5  # a^2=1.0 > v*J_small=0.4 なので退化する
    assert a**2 > v * J_small
    a_s = degenerate_peak_accel(v, J_small)
    print(f"\n退化ケース(a={a}, J={J_small}, a^2={a**2:.2f} > vJ={v * J_small:.2f}): 実際のピーク加速度 a_s={a_s:.4f}")
    # a_sは要求したaより小さいはず、かつ a_s^2 = v*J_small を満たす
    assert a_s < a
    assert abs(a_s**2 - v * J_small) < 1e-9

    print("\nすべてのチェックに合格しました。")

    fig, axes = plt.subplots(4, 1, figsize=(8, 10), sharex=True)
    t_axis = np.linspace(0, T, 2000)
    s_ax, sdot_ax, sddot_ax, sdddot_ax, _ = scurve_time_scaling(t_axis, v, a, J)
    for ax, (y, label) in zip(
        axes, [(s_ax, "s(t)"), (sdot_ax, "s_dot(t)"), (sddot_ax, "s_ddot(t)"), (sdddot_ax, "s_dddot(t) [jerk]")]
    ):
        ax.plot(t_axis, y)
        ax.set_ylabel(label)
        for e in edges:
            ax.axvline(e, color="gray", linestyle="--", linewidth=0.6)
    axes[-1].set_xlabel("time [s]")
    axes[0].set_title(f"S-curve (jerk-limited) velocity profile (v={v}, a={a}, J={J}, T={T:.3f})")
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "03_scurve_velocity_profile.png", dpi=150)

    fig2, ax2 = plt.subplots()
    J_sweep = np.linspace(1.0, 30.0, 60)

    def _feasible_T(Jv):
        if a**2 > v * Jv:
            return np.nan
        S_ramp = v**2 / (2 * a) + a * v / (2 * Jv)
        if 2 * S_ramp > 1:
            return np.nan
        return scurve_phase_boundaries(v, a, Jv)[0]

    T_sweep = [_feasible_T(Jv) for Jv in J_sweep]
    ax2.plot(J_sweep, T_sweep, label="S-curve T(J)")
    ax2.axhline(T_trap, color="gray", linestyle="--", label="trapezoidal T*")
    ax2.set_xlabel("jerk limit J")
    ax2.set_ylabel("total time T")
    ax2.set_title(f"S-curve time -> trapezoidal time as J increases (v={v}, a={a})")
    ax2.legend()
    fig2.savefig(OUTPUT_DIR / "03_scurve_vs_trapezoidal.png", dpi=150)
    print(f"\nグラフを保存しました: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
