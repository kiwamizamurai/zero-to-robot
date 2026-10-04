"""障害物をよけて腕を動かす(Panda × MuJoCo の当たり判定 × RRT-Connect)。

これまでのアームは「真上へ上がる → 横へ動く → 下りる」という、ぶつからないと分かっている道を人が決めていた。
ここでは、腕の前に壁があって、壁の向こう側へ手を移したい。どう動けばぶつからないかを、ロボット自身に探させる。

1. 始まりと終わりの姿勢: 手先の位置から逆運動学(mink)で 7 つの関節角度を出す
2. まっすぐ動かすと?: 7 つの関節角度を直線で補間すると、腕が壁を突き抜けてしまう(MuJoCo の当たり判定で分かる)
3. RRT-Connect で探す: 7 次元の「関節角度の空間(構成空間)」に、始まりと終わりの両方から木を伸ばしていき、
   つながったら道が見つかる。木を伸ばすたびに、その姿勢で腕が壁や床にぶつからないかを MuJoCo に聞く
4. 道をなめらかに: 見つかった道はジグザグなので、まっすぐつなげる所は飛ばして短くする(ショートカット)
5. 実行: 道に沿って 5 次多項式でなめらかに動かし、実際にぶつからなかったかを確かめる

実行:
    python apps/panda_rrt/rrt_plan.py              # 図と動画を output/ に書き出す
    python apps/panda_rrt/rrt_plan.py --seed 3     # 乱数を変えると、見つかる道も変わる
"""
import argparse
import os
import sys
import time
from pathlib import Path

if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")

import mink
import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "panda_pick_place"))
import pick_place as base  # noqa: E402  (Menagerie の Panda の場面を使う)

OUTPUT_DIR = Path(__file__).parent / "output"
WALL = dict(pos=(0.48, 0.0, 0.30), half=(0.12, 0.015, 0.30))  # 高さ 60 cm の壁
START_TCP, GOAL_TCP = (0.50, -0.28, 0.18), (0.50, 0.28, 0.18)
ROBOT_BODIES = ["link1", "link2", "link3", "link4", "link5", "link6", "link7", "hand", "left_finger", "right_finger"]


def build():
    spec = mujoco.MjSpec.from_file(str(base.SCENE))
    spec.body("hand").add_site(name="tcp", pos=[0, 0, 0.1034])
    wall = spec.worldbody.add_body(name="wall", pos=list(WALL["pos"]))
    wall.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=list(WALL["half"]), rgba=[0.75, 0.45, 0.3, 1])
    for name, p, rgba in [("start", START_TCP, [0.2, 0.6, 1, 0.9]), ("goal", GOAL_TCP, [0.2, 0.8, 0.3, 0.9])]:
        spec.worldbody.add_geom(name=f"mark_{name}", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.025, 0, 0], pos=list(p),
                                rgba=rgba, contype=0, conaffinity=0)
    spec.worldbody.add_body(name="look_at", pos=[0.35, 0.0, 0.35])
    cam = spec.worldbody.add_camera(name="side", pos=[1.55, -1.05, 1.0])
    cam.mode = mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY
    cam.targetbody = "look_at"
    spec.material("groundplane").reflectance = 0.0
    return spec.compile()


class Checker:
    """関節角度 q のとき、腕が壁や床にぶつかっていないかを MuJoCo の当たり判定で調べる。"""

    def __init__(self, model, margin=0.03):
        # 計画用には壁と床を margin [m] だけ大きく見る(実行時のわずかなずれで、かすらないように)
        self.m = model
        for g in range(model.ngeom):
            if model.geom_bodyid[g] not in {model.body(b).id for b in ROBOT_BODIES}:
                model.geom_margin[g] = margin
        self.d = mujoco.MjData(model)
        self.robot = {model.body(b).id for b in ROBOT_BODIES}
        self.lo, self.hi = model.jnt_range[:7].T
        self.calls = 0

    def free(self, q):
        self.calls += 1
        if np.any(q < self.lo) or np.any(q > self.hi):
            return False
        self.d.qpos[:7] = q
        self.d.qpos[7:9] = 0.04
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_collision(self.m, self.d)
        for c in self.d.contact[:self.d.ncon]:
            b1, b2 = self.m.geom_bodyid[c.geom1], self.m.geom_bodyid[c.geom2]
            if (b1 in self.robot) != (b2 in self.robot):  # 腕と、腕でないもの(壁・床)の接触
                return False
        return True

    def edge_free(self, a, b, step=0.03):
        n = max(2, int(np.ceil(np.max(np.abs(b - a)) / step)))
        return all(self.free(a + (b - a) * k / n) for k in range(1, n + 1))


def ik(model, tcp, q0):
    cfg = mink.Configuration(model)
    q = model.key("home").qpos.copy()
    q[:7] = q0
    cfg.update(q)
    task = mink.FrameTask("tcp", "site", position_cost=1.0, orientation_cost=1.0)
    down = cfg.get_transform_frame_to_world("tcp", "site").rotation()
    task.set_target(mink.SE3.from_rotation_and_translation(down, np.array(tcp)))
    post = mink.PostureTask(model, cost=1e-2)
    post.set_target_from_configuration(cfg)
    for _ in range(300):
        v = mink.solve_ik(cfg, [task, post], 0.02, solver="daqp", damping=1e-3, limits=[mink.ConfigurationLimit(model)])
        cfg.integrate_inplace(v, 0.02)
    return cfg.q[:7].copy()


def rrt_connect(chk, qs, qg, rng, step=0.15, max_iter=20000):
    """始まりと終わりの両方から木を伸ばし、つながったら道を返す。"""
    trees = [{"q": [qs], "parent": [-1]}, {"q": [qg], "parent": [-1]}]

    def nearest(tree, q):
        return int(np.argmin(np.linalg.norm(np.array(tree["q"]) - q, axis=1)))

    def extend(tree, q):
        i = nearest(tree, q)
        qn = tree["q"][i]
        d = q - qn
        dist = np.linalg.norm(d)
        new = q if dist <= step else qn + d / dist * step
        if not chk.edge_free(qn, new):
            return None, False
        tree["q"].append(new)
        tree["parent"].append(i)
        return len(tree["q"]) - 1, np.allclose(new, q)

    def path_to_root(tree, i):
        out = []
        while i != -1:
            out.append(tree["q"][i])
            i = tree["parent"][i]
        return out

    a, b = 0, 1
    for it in range(max_iter):
        q_rand = rng.uniform(chk.lo, chk.hi)
        ia, _ = extend(trees[a], q_rand)
        if ia is not None:
            target = trees[a]["q"][ia]
            while True:  # もう一方の木を、いま伸ばした点に向かって伸ばせるだけ伸ばす(connect)
                ib, reached = extend(trees[b], target)
                if ib is None:
                    break
                if reached:
                    pa, pb = path_to_root(trees[a], ia), path_to_root(trees[b], ib)
                    path = pa[::-1] + pb if a == 0 else pb[::-1] + pa
                    return path, it + 1, sum(len(t["q"]) for t in trees)
        a, b = b, a
    return None, max_iter, sum(len(t["q"]) for t in trees)


def shortcut(chk, path, rng, tries=200):
    """道の途中の 2 点をまっすぐつなげられるなら、その間を飛ばす。"""
    path = list(path)
    for _ in range(tries):
        if len(path) < 3:
            break
        i, j = sorted(rng.choice(len(path), 2, replace=False))
        if j - i < 2:
            continue
        if chk.edge_free(path[i], path[j]):
            path = path[:i + 1] + path[j:]
    return path


def path_length(path):
    return float(sum(np.linalg.norm(b - a) for a, b in zip(path, path[1:])))


def execute(model, path, duration, record=None):
    """道を弧長でたどり、5 次多項式でなめらかに動かす。実行中に腕が壁・床に触れた回数を数える。"""
    data = mujoco.MjData(model)
    data.qpos[:9] = model.key("home").qpos[:9]
    data.qpos[:7] = path[0]
    data.ctrl[:7] = path[0]
    data.ctrl[7] = 255
    mujoco.mj_forward(model, data)
    seg = np.array([0.0] + list(np.cumsum([np.linalg.norm(b - a) for a, b in zip(path, path[1:])])))
    robot = {model.body(b).id for b in ROBOT_BODIES}
    hits, tcp = 0, []
    n = int(duration / model.opt.timestep)
    for k in range(n + int(0.8 / model.opt.timestep)):
        s = base.smooth(k / n) * seg[-1]
        i = min(np.searchsorted(seg, s, side="right") - 1, len(path) - 2)
        u = (s - seg[i]) / max(seg[i + 1] - seg[i], 1e-9)
        data.ctrl[:7] = path[i] + (path[i + 1] - path[i]) * u
        mujoco.mj_step(model, data)
        for c in data.contact[:data.ncon]:
            b1, b2 = model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]
            if (b1 in robot) != (b2 in robot):
                hits += 1
                break
        if k % 10 == 0:
            tcp.append(data.site("tcp").xpos.copy())
        if record and k % 10 == 0:
            record(data)
    return hits, np.array(tcp), data.site("tcp").xpos.copy()


def plot(straight_tcp, rrt_tcp, raw_len, short_len, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(10.5, 4.2), gridspec_kw={"width_ratios": [1.3, 1]})
    wx, wy, wz = WALL["pos"]
    hx, hy, hz = WALL["half"]
    ax.add_patch(plt.Rectangle((wy - hy, 0), 2 * hy, 2 * hz, color="#c07250", label="wall (60 cm)"))
    ax.plot(straight_tcp[:, 1], straight_tcp[:, 2], "r--", lw=1.6, label="straight in joint space (hits the wall)")
    ax.plot(rrt_tcp[:, 1], rrt_tcp[:, 2], "b", lw=2, label="RRT-Connect + shortcut")
    ax.plot(START_TCP[1], START_TCP[2], "o", color="#3399ff", ms=9, label="start")
    ax.plot(GOAL_TCP[1], GOAL_TCP[2], "o", color="#33cc55", ms=9, label="goal")
    ax.set_xlabel("y [m]  (side view, seen from the front)")
    ax.set_ylabel("height z [m]")
    ax.set_title("hand (tcp) path", fontsize=10)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, loc="upper left")
    q = np.array(path)
    ax2.plot(q, marker="o", ms=3)
    ax2.set_title(f"joint angles along the path ({len(path)} waypoints)", fontsize=10)
    ax2.set_xlabel("waypoint")
    ax2.set_ylabel("rad")
    ax2.legend([f"j{i + 1}" for i in range(7)], fontsize=7, ncol=4)
    ax2.grid(alpha=0.3)
    fig.suptitle(f"joint-space path length: raw RRT {raw_len:.1f} rad -> after shortcut {short_len:.1f} rad", fontsize=10)
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "rrt_paths.png", dpi=105)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-video", action="store_true")
    args = ap.parse_args()
    OUTPUT_DIR.mkdir(exist_ok=True)
    rng = np.random.default_rng(args.seed)
    model = build()
    chk = Checker(build())  # 計画用は別のモデル(壁を少し大きく見る)。実行は model で行う

    home = model.key("home").qpos[:7]
    qs = ik(model, START_TCP, home)
    qg = ik(model, GOAL_TCP, qs)
    print(f"1. 始まり・終わりの姿勢: ぶつかっていないか {chk.free(qs)} / {chk.free(qg)}")

    straight = [qs, qg]
    print(f"2. 関節角度をまっすぐ補間すると: {'ぶつからない' if chk.edge_free(qs, qg) else '壁にぶつかる'}")

    t0, chk.calls = time.perf_counter(), 0
    path, iters, nodes = rrt_connect(chk, qs, qg, rng)
    assert path is not None, "道が見つからなかった"
    t_plan, calls = time.perf_counter() - t0, chk.calls
    short = shortcut(chk, path, rng)
    raw_len, short_len = path_length(path), path_length(short)
    print(f"3. RRT-Connect: {iters} 回で道が見つかった({t_plan:.2f} 秒、木の点 {nodes} 個、当たり判定 {calls} 回)")
    print(f"4. ショートカット: 経由点 {len(path)} → {len(short)} 個、関節空間での道のり {raw_len:.1f} → {short_len:.1f} rad")

    frames = []
    renderer = None if args.no_video else mujoco.Renderer(model, height=360, width=480)

    def recorder(title):
        def rec(d):
            renderer.update_scene(d, camera="side")
            frames.append(caption(renderer.render(), title))
        return rec

    hits_s, tcp_s, _ = execute(model, straight, 3.0, recorder("straight in joint space: hits the wall") if renderer else None)
    hits_r, tcp_r, end = execute(model, short, 4.0, recorder("RRT-Connect path: goes over the wall") if renderer else None)
    err = np.linalg.norm(end - np.array(GOAL_TCP)) * 100
    print(f"5. 実行: まっすぐ = 壁や床との接触 {hits_s} ステップ / RRT の道 = 接触 {hits_r} ステップ、着いた位置のずれ {err:.1f} cm")
    plot(tcp_s, tcp_r, raw_len, short_len, short)
    print(f"図を保存: {OUTPUT_DIR / 'rrt_paths.png'}")
    if renderer:
        renderer.close()
        import subprocess

        import imageio.v2 as imageio
        import imageio_ffmpeg
        mp4 = OUTPUT_DIR / "rrt.mp4"
        imageio.mimsave(mp4, frames, fps=20)
        subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-loglevel", "error", "-y", "-i", str(mp4), "-vf",
                        "fps=10,scale=440:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=48[p];[b][p]paletteuse=dither=bayer",
                        str(OUTPUT_DIR / "rrt.gif")], check=True)
        print(f"動画を保存: {mp4}")
    assert hits_s > 0 and hits_r == 0 and err < 3


def caption(img, text):
    import cv2

    img = np.ascontiguousarray(img)
    for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):  # 黒いふちどり
        cv2.putText(img, text, (10 + dx, 24 + dy), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(img, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return img


if __name__ == "__main__":
    main()
