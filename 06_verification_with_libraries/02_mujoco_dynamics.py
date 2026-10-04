"""自作の運動方程式(02_dynamics/02_two_link_dynamics.py: 質量行列・コリオリ/遠心力項・
重力項・逆動力学)を、MuJoCo(mj_fullM, mj_rne)の出力と突き合わせて検算する。

ロボットは02_dynamicsと全く同じ物理パラメータ(質量・重心位置・慣性モーメント)の
2リンク平面アームをMJCFで再現する。重力ベクトルは02_dynamicsの g_vec=(0,-g) に
合わせて (0, -9.81, 0) とする(01_kinematicsの平面はxy平面、関節軸はz)。

mj_rne(flg_acc=0) は M(q)*0 + C(q,qdot)、すなわち加速度0のときに関節を
その場に留めるために必要なトルク = コリオリ/遠心力項+重力項の和を返す
(MuJoCoのドキュメント上の"C"は、Modern Roboticsの記法の c(q,qdot)+g(q) に対応する)。
"""
import sys
from importlib import import_module
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "02_dynamics"))
_dyn = import_module("02_two_link_dynamics")

L1, L2 = _dyn.l1, _dyn.l2
LC1, LC2 = _dyn.lc1, _dyn.lc2
M1, M2 = _dyn.m1, _dyn.m2
I1, I2 = _dyn.I1, _dyn.I2
G = _dyn.g

MJCF = f"""
<mujoco>
  <option gravity="0 -{G} 0"/>
  <worldbody>
    <body name="link1" pos="0 0 0">
      <joint name="joint1" type="hinge" axis="0 0 1"/>
      <geom type="capsule" fromto="0 0 0  {L1} 0 0" size="0.02" contype="0" conaffinity="0"/>
      <inertial pos="{LC1} 0 0" mass="{M1}" diaginertia="{I1} {I1} {I1}"/>
      <body name="link2" pos="{L1} 0 0">
        <joint name="joint2" type="hinge" axis="0 0 1"/>
        <geom type="capsule" fromto="0 0 0  {L2} 0 0" size="0.02" contype="0" conaffinity="0"/>
        <inertial pos="{LC2} 0 0" mass="{M2}" diaginertia="{I2} {I2} {I2}"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def mujoco_mass_matrix(model, data, q):
    data.qpos[:] = q
    mujoco.mj_forward(model, data)
    M = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, data, M)
    return M


def mujoco_bias(model, data, q, qdot):
    """C(q,qdot) + g(q): 加速度0で釣り合わせるのに必要なトルク。"""
    data.qpos[:] = q
    data.qvel[:] = qdot
    mujoco.mj_forward(model, data)
    result = np.zeros(model.nv)
    mujoco.mj_rne(model, data, 0, result)
    return result


def mujoco_inverse_dynamics(model, data, q, qdot, qddot):
    """mj_forwardは順動力学(qfrc_applied=0のもとでのqacc)を計算してd.qaccを
    上書きしてしまうので、d.qaccへの代入はmj_forwardの後、mj_rneの直前に行う
    (mj_rne自体は前進動力学を呼ばず、現在のd.qpos/qvel/qaccからRNEを行うのみ)。"""
    data.qpos[:] = q
    data.qvel[:] = qdot
    mujoco.mj_forward(model, data)
    data.qacc[:] = qddot
    result = np.zeros(model.nv)
    mujoco.mj_rne(model, data, 1, result)
    return result


def main():
    model = mujoco.MjModel.from_xml_string(MJCF)
    data = mujoco.MjData(model)

    rng = np.random.default_rng(0)
    max_M_err = max_bias_err = max_id_err = 0.0
    for _ in range(200):
        q = rng.uniform(-np.pi, np.pi, size=2)
        qdot = rng.uniform(-2.0, 2.0, size=2)
        qddot = rng.uniform(-2.0, 2.0, size=2)

        M_mj = mujoco_mass_matrix(model, data, q)
        M_ours = _dyn.mass_matrix(q)
        max_M_err = max(max_M_err, np.max(np.abs(M_mj - M_ours)))

        bias_mj = mujoco_bias(model, data, q, qdot)
        bias_ours = _dyn.coriolis_centrifugal(q, qdot) + _dyn.gravity(q)
        max_bias_err = max(max_bias_err, np.max(np.abs(bias_mj - bias_ours)))

        tau_mj = mujoco_inverse_dynamics(model, data, q, qdot, qddot)
        tau_ours = _dyn.inverse_dynamics(q, qdot, qddot)
        max_id_err = max(max_id_err, np.max(np.abs(tau_mj - tau_ours)))

    assert max_M_err < 1e-8, f"mass matrix mismatch: {max_M_err}"
    assert max_bias_err < 1e-6, f"coriolis+gravity mismatch: {max_bias_err}"
    assert max_id_err < 1e-6, f"inverse dynamics mismatch: {max_id_err}"
    print(f"質量行列 M(q): max error over 200 random q = {max_M_err:.2e}")
    print(f"コリオリ/遠心力項+重力項 c(q,qdot)+g(q): max error = {max_bias_err:.2e}")
    print(f"逆動力学 M(q)qddot+c(q,qdot)+g(q): max error = {max_id_err:.2e}")
    print("自作の運動方程式はMuJoCoのmj_fullM・mj_rneと一致した。")


if __name__ == "__main__":
    main()
