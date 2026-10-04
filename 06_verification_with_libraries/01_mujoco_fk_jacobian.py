"""自作の順運動学・ヤコビ行列(01_kinematics/02_ik_gradient_descent.py)を、
MuJoCo(mj_forward, mj_jacSite)の出力と突き合わせて検算する。

ロボットは01_kinematicsと同じ2リンク平面アーム(l1=1.0, l2=0.8)を、
MJCF(MuJoCoのXML形式)でそのまま再現する: 関節はz軸まわりのhinge、
リンクはローカルx軸方向に伸びるcapsule。関節角の符号・回転の向きは
z軸まわりの右手則で、01_kinematicsの平面回転行列R(theta)の定義と一致する。
"""
import sys
from importlib import import_module
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "01_kinematics"))
_ik = import_module("02_ik_gradient_descent")

L1, L2 = _ik.l1, _ik.l2

MJCF = f"""
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="link1" pos="0 0 0">
      <joint name="joint1" type="hinge" axis="0 0 1"/>
      <geom type="capsule" fromto="0 0 0  {L1} 0 0" size="0.02" mass="1.0"/>
      <body name="link2" pos="{L1} 0 0">
        <joint name="joint2" type="hinge" axis="0 0 1"/>
        <geom type="capsule" fromto="0 0 0  {L2} 0 0" size="0.02" mass="0.8"/>
        <site name="ee" pos="{L2} 0 0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def mujoco_fk_and_jac(model, data, q):
    """MuJoCoでFK(手先位置)とヤコビ行列(線速度部分)を計算する。"""
    data.qpos[:] = q
    mujoco.mj_forward(model, data)
    ee_site = model.site("ee").id
    pos = data.site_xpos[ee_site][:2].copy()
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, jacr, ee_site)
    return pos, jacp[:2, :]


def main():
    model = mujoco.MjModel.from_xml_string(MJCF)
    data = mujoco.MjData(model)

    rng = np.random.default_rng(0)
    max_pos_err = 0.0
    max_jac_err = 0.0
    for _ in range(200):
        q = rng.uniform(-np.pi, np.pi, size=2)

        pos_mj, jac_mj = mujoco_fk_and_jac(model, data, q)
        pos_ours = _ik.fk(q)
        jac_ours = _ik.jac(q)

        max_pos_err = max(max_pos_err, np.max(np.abs(pos_mj - pos_ours)))
        max_jac_err = max(max_jac_err, np.max(np.abs(jac_mj - jac_ours)))

    assert max_pos_err < 1e-9, f"FK mismatch: {max_pos_err}"
    assert max_jac_err < 1e-9, f"Jacobian mismatch: {max_jac_err}"
    print(f"FK position: max error over 200 random q = {max_pos_err:.2e}")
    print(f"Jacobian: max error over 200 random q = {max_jac_err:.2e}")
    print("自作のfk(q)・jac(q)はMuJoCoのmj_forward・mj_jacSiteと一致した。")


if __name__ == "__main__":
    main()
