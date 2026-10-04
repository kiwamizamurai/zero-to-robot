"""力を加減してなぞる(Panda × MuJoCo × インピーダンス制御)。

これまでのアームは「関節をこの角度にしろ」という位置の指令で動かしていた。位置の指令は、何かに触れる作業が苦手。
机の高さの見込みが 1 cm ずれているだけで、ペン先が届かずに宙に浮くか、机に強く押しつけすぎてしまう。

そこで、手先を「ばねとダンパーでつながれた点」のように振る舞わせる(インピーダンス制御)。
目標の位置より少し奥(机の中)を目標にしておくと、ばねが縮んだ分だけの力で机を押す。
ばねが柔らかければ、机の高さが少しずれても、押す力はあまり変わらない。

やること: ペン先で机に半径 8 cm の円を描く。押す力の目標は 10 N。
比べるもの:
- 硬い制御(位置制御に近い): 上下のばね定数 8000 N/m。目標は机の見込みの 1.25 mm 下
- 柔らかい制御(インピーダンス制御): 上下のばね定数 400 N/m。目標は机の見込みの 2.5 cm 下
それぞれ、机が見込みどおり / 1 cm 低い / 1 cm 高い の 3 通りで試し、押す力と描けた線を比べる。

しくみ: Panda のモーターを「トルクを直接指令する」方式に切り替え、
  τ = Jᵀ F + (重力・コリオリ力の打ち消し) + (余った自由度で姿勢を保つ分)
  F = K (目標の位置 − 手先の位置) − D (手先の速度)
を毎ステップ計算する。

実行:
    python apps/panda_impedance/impedance.py
"""
import argparse
import os
import sys
from pathlib import Path

if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "panda_pick_place"))
import pick_place as base  # noqa: E402

OUTPUT_DIR = Path(__file__).parent / "output"
TABLE_Z = 0.20       # 机の上面の高さの見込み [m]
CENTER, RADIUS = np.array([0.50, 0.0]), 0.08
PEN_R = 0.012
MODES = {"stiff": dict(kz=8000.0, depth=0.00125), "soft": dict(kz=400.0, depth=0.025)}
FORCE_GOAL = 10.0


def build(table_offset):
    spec = mujoco.MjSpec.from_file(str(base.SCENE))
    # モーターを「トルク指令」に切り替える(位置サーボのばね・ダンパーを外す)
    for i in range(1, 8):
        a = spec.actuator(f"actuator{i}")
        a.gainprm[0] = 1.0
        a.biastype = mujoco.mjtBias.mjBIAS_NONE
        a.biasprm[:3] = 0.0
        lim = 12.0 if i >= 5 else 87.0
        a.ctrlrange = [-lim, lim]
    hand = spec.body("hand")
    hand.add_geom(name="pen", type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[PEN_R, 0.04, 0], pos=[0, 0, 0.1034 + 0.04],
                  rgba=[0.15, 0.15, 0.2, 1], friction=[0.05, 0.005, 0.0001], mass=0.05)
    hand.add_site(name="pen_tip", pos=[0, 0, 0.1034 + 0.08 + PEN_R])
    top = TABLE_Z + table_offset
    table = spec.worldbody.add_body(name="table", pos=[0.5, 0, top / 2])
    table.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.25, 0.3, top / 2], rgba=[0.93, 0.93, 0.9, 1],
                   friction=[0.05, 0.005, 0.0001])  # ホワイトボードとマーカーくらいの、すべりやすい面
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.worldbody.add_body(name="look_at", pos=[0.45, 0.0, 0.25])
    cam = spec.worldbody.add_camera(name="side", pos=[1.25, -0.75, 0.75])
    cam.mode = mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY
    cam.targetbody = "look_at"
    spec.material("groundplane").reflectance = 0.0
    return spec.compile()


class Impedance:
    def __init__(self, model, data, kz):
        self.m, self.d = model, data
        self.site = model.site("pen_tip").id
        self.K = np.diag([1500.0, 1500.0, kz])
        self.D = np.diag([2 * np.sqrt(1500.0 * 2), 2 * np.sqrt(1500.0 * 2), 2 * np.sqrt(kz * 2)])
        self.Kr, self.Dr = 60.0, 4.0
        self.q_home = model.key("home").qpos[:7].copy()
        self.Rd = None

    def __call__(self, x_goal):
        m, d = self.m, self.d
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        mujoco.mj_jacSite(m, d, jp, jr, self.site)
        jp, jr = jp[:, :7], jr[:, :7]
        x = d.site_xpos[self.site]
        R = d.site_xmat[self.site].reshape(3, 3)
        if self.Rd is None:
            self.Rd = R.copy()
        qd = d.qvel[:7]
        v, w = jp @ qd, jr @ qd
        F = self.K @ (x_goal - x) - self.D @ v                       # 並進のばねとダンパー
        E = 0.5 * (self.Rd.T @ R - R.T @ self.Rd)
        eR = R @ np.array([E[2, 1], E[0, 2], E[1, 0]])
        Mrot = -self.Kr * eR - self.Dr * w                          # ペンを真下に向け続ける
        J = np.vstack([jp, jr])
        tau = J.T @ np.concatenate([F, Mrot])
        Jpinv = np.linalg.pinv(J)
        null = (np.eye(7) - J.T @ Jpinv.T) @ (10.0 * (self.q_home - d.qpos[:7]) - 2.0 * qd)  # 余った自由度
        return tau + null + d.qfrc_bias[:7]                         # 重力・コリオリ力を打ち消す


def pen_force(model, data):
    """ペンと机の接触力(机を押す向きの成分)と、触れている点を返す。"""
    pen, table = model.geom("pen").id, model.geom("table").id
    f6, total, point = np.zeros(6), 0.0, None
    for i, c in enumerate(data.contact[:data.ncon]):
        if {c.geom1, c.geom2} == {pen, table}:
            mujoco.mj_contactForce(model, data, i, f6)
            total += f6[0]
            point = c.pos.copy()
    return total, point


def run(mode, table_offset, record=None):
    model = build(table_offset)
    data = mujoco.MjData(model)
    data.qpos[:9] = model.key("home").qpos[:9]
    mujoco.mj_forward(model, data)
    ctl = Impedance(model, data, MODES[mode]["kz"])
    depth = MODES[mode]["depth"]
    start = data.site("pen_tip").xpos.copy()
    above = np.array([*(CENTER + [RADIUS, 0]), TABLE_Z + 0.05])
    press = np.array([*(CENTER + [RADIUS, 0]), TABLE_Z - depth])
    T_move, T_down, T_circle, T_hold = 2.0, 1.5, 6.0, 0.5
    log, ink, recent = [], [], []
    t_end = T_move + T_down + T_circle + T_hold
    while data.time < t_end:
        t = data.time
        if t < T_move:
            goal = start + (above - start) * base.smooth(t / T_move)
        elif t < T_move + T_down:
            goal = above + (press - above) * base.smooth((t - T_move) / T_down)
        elif t < T_move + T_down + T_circle:
            a = 2 * np.pi * base.smooth((t - T_move - T_down) / T_circle)
            goal = np.array([*(CENTER + RADIUS * np.array([np.cos(a), np.sin(a)])), TABLE_Z - depth])
        else:
            goal = press
        data.ctrl[:7] = np.clip(ctl(goal), model.actuator_ctrlrange[:7, 0], model.actuator_ctrlrange[:7, 1])
        data.ctrl[7] = 0
        mujoco.mj_step(model, data)
        f, p = pen_force(model, data)
        drawing = T_move + T_down <= t < T_move + T_down + T_circle
        if p is not None and f > 0.5:
            ink.append(p[:2].copy())
        log.append(dict(t=t, f=f, drawing=drawing, z=data.site("pen_tip").xpos[2]))
        recent = (recent + [f])[-10:]
        if record and len(log) % 10 == 0:
            record(model, data, ink, float(np.mean(recent)), t)
    return log, np.array(ink) if ink else np.zeros((0, 2))


def smooth_force(log, n=10):
    """接触力は 1 ステップごとに細かく振動するので、力センサーと同じように 20 ms(10 ステップ)の平均で見る。"""
    f = np.array([r["f"] for r in log])
    return np.convolve(f, np.ones(n) / n, mode="same")


def summarize(log, ink):
    fs = smooth_force(log)
    f = fs[np.array([r["drawing"] for r in log])]
    touching = (f > 0.5).mean() * 100
    # 円周のうち、線が引けた割合(1° 刻みで、近くに触れた点があるか)
    ang = np.linspace(0, 2 * np.pi, 360, endpoint=False)
    circle = CENTER + RADIUS * np.column_stack([np.cos(ang), np.sin(ang)])
    covered = 0.0 if len(ink) == 0 else (np.min(np.linalg.norm(circle[:, None] - ink[None], axis=2), axis=1) < 0.006).mean() * 100
    return f.mean(), f.max(), touching, covered


def plot(results, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    offsets = [-0.01, 0.0, 0.01]
    names = {-0.01: "table 1 cm LOWER", 0.0: "table as expected", 0.01: "table 1 cm HIGHER"}
    fig, axes = plt.subplots(2, 3, figsize=(12, 6.6))
    for j, off in enumerate(offsets):
        ax = axes[0, j]
        for mode, color in (("stiff", "#d9542f"), ("soft", "#1f6fb2")):
            log, _ = results[(mode, off)]
            t = [r["t"] for r in log]
            ax.plot(t, smooth_force(log), color=color, lw=1.2,
                    label=f"{'stiff (8000 N/m)' if mode == 'stiff' else 'soft impedance (400 N/m)'}")
        ax.axhline(FORCE_GOAL, color="k", ls=":", lw=1)
        ax.set_title(names[off], fontsize=10)
        ax.set_xlabel("time [s]")
        ax.set_ylim(0, 100)
        ax.grid(alpha=0.3)
        if j == 0:
            ax.set_ylabel("pressing force [N] (20 ms average)")
            ax.legend(fontsize=8)
        ax = axes[1, j]
        ang = np.linspace(0, 2 * np.pi, 200)
        ax.plot(CENTER[0] + RADIUS * np.cos(ang), CENTER[1] + RADIUS * np.sin(ang), color="gray", ls="--", lw=1)
        for mode, color, ms in (("stiff", "#d9542f", 6), ("soft", "#1f6fb2", 2)):
            _, ink = results[(mode, off)]
            if len(ink):
                ax.plot(ink[:, 0], ink[:, 1], ".", ms=ms, color=color, alpha=0.6 if mode == "stiff" else 1.0)
        ax.set_aspect("equal")
        ax.set_title("ink on the table (thick red: stiff, thin blue: soft, dashed: goal)", fontsize=8)
        ax.set_xticks([])
        ax.set_yticks([])
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def make_video(path_mp4, clips):
    import subprocess

    import cv2
    import imageio.v2 as imageio
    import imageio_ffmpeg

    frames = []
    for mode, off, title in clips:
        model = build(off)
        r = mujoco.Renderer(model, height=360, width=480)

        def rec(m, d, ink, f, t):
            r.update_scene(d, camera="side")
            for p in ink[-1500::3]:  # 描けた線を小さな円盤で描く
                if r.scene.ngeom >= r.scene.maxgeom:
                    break
                g = r.scene.geoms[r.scene.ngeom]
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CYLINDER, np.array([0.004, 0.004, 0.0005]),
                                    np.array([p[0], p[1], TABLE_Z + off + 0.0006]), np.eye(3).flatten(),
                                    np.array([0.1, 0.2, 0.8, 1.0], np.float32))
                r.scene.ngeom += 1
            img = np.ascontiguousarray(r.render())
            for i, s in enumerate([title, f"t {t:4.1f} s   pressing force {f:5.1f} N  (goal 10 N)"]):
                for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    cv2.putText(img, s, (10 + dx, 24 + 22 * i + dy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
                cv2.putText(img, s, (10, 24 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            frames.append(img)

        run(mode, off, rec)
        r.close()
    imageio.mimsave(path_mp4, frames, fps=20)
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-loglevel", "error", "-y", "-i", str(path_mp4), "-vf",
                    "fps=10,scale=440:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=48[p];[b][p]paletteuse=dither=bayer",
                    str(path_mp4.with_suffix(".gif"))], check=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-video", action="store_true")
    args = ap.parse_args()
    OUTPUT_DIR.mkdir(exist_ok=True)
    results = {}
    print(f"{'制御':28s} {'机の高さ':10s} {'平均の力':>8s} {'最大の力':>8s} {'触れていた時間':>10s} {'描けた円周':>8s}")
    for mode in ("stiff", "soft"):
        for off in (-0.01, 0.0, 0.01):
            log, ink = run(mode, off)
            results[(mode, off)] = (log, ink)
            mean, mx, touch, cov = summarize(log, ink)
            label = "硬い(8000 N/m)" if mode == "stiff" else "柔らかい(400 N/m)"
            where = {-0.01: "1 cm 低い", 0.0: "見込みどおり", 0.01: "1 cm 高い"}[off]
            print(f"{label:24s} {where:10s} {mean:7.1f} N {mx:7.1f} N {touch:9.0f} % {cov:8.0f} %")
    plot(results, OUTPUT_DIR / "force_compare.png")
    print(f"図を保存: {OUTPUT_DIR / 'force_compare.png'}")
    if not args.no_video:
        make_video(OUTPUT_DIR / "impedance.mp4", [
            ("stiff", -0.01, "STIFF (position-like), table 1 cm lower"),
            ("soft", -0.01, "SOFT impedance, table 1 cm lower"),
            ("stiff", 0.01, "STIFF (position-like), table 1 cm higher"),
            ("soft", 0.01, "SOFT impedance, table 1 cm higher")])
        print(f"動画を保存: {OUTPUT_DIR / 'impedance.mp4'}")
    for off in (-0.01, 0.0, 0.01):
        mean, _, _, cov = summarize(*results[("soft", off)])
        assert 5 < mean < 15 and cov > 90, "柔らかい制御で、狙った力で描けなかった"


if __name__ == "__main__":
    main()
