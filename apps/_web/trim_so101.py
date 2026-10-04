"""Menagerie の robotstudio_so101 から、ブラウザ用に軽量化した so101.xml とメッシュを作る。

描画用(visual)メッシュは合計 17MB あるので捨て、衝突用メッシュだけ(数十 KB)を使う。
物理は元のモデルと同じ(関節・サーボの PD ゲイン・衝突形状)。見た目は、衝突用の箱と指のメッシュで描く。
 - 衝突用の形状に色をつける(元は rgba が既定色か赤)
 - 手首カメラの台(camera_mount)は、見た目のメッシュが質量 0.012 kg を持っているので、同じ質量の箱に置き換える

使い方: python _web/trim_so101.py <mujoco_menagerie/robotstudio_so101>
"""
import re
import shutil
import sys
from pathlib import Path

src = Path(sys.argv[1])
dst = Path(__file__).parent / "assets" / "so101"
(dst / "assets").mkdir(parents=True, exist_ok=True)

xml = (src / "so101.xml").read_text()
# カメラ台の見た目メッシュ → 同じ質量の箱(衝突はしない)
xml = re.sub(r'<geom type="mesh" mass="0.012" class="visual"[^>]*?mesh="wrist_roll_follower_so101_camera_mount"[^>]*?/>',
             '<geom type="box" mass="0.012" size="0.02 0.03 0.012" pos="0 0.03 -0.03" class="visual" rgba="0.1 0.1 0.1 1"/>',
             xml, flags=re.S)
# 見た目用の geom(class="visual" のメッシュ)を消す
xml = re.sub(r'\s*<geom type="mesh" class="visual"[^>]*?/>', "", xml, flags=re.S)
# 見た目用のメッシュ・マテリアルの定義を消し、衝突で使う 3 つだけ残す
keep = ["wrist_roll_follower_so101_gripper_part0_v1.stl", "moving_jaw_so101_gripper_part0_v1.stl", "moving_jaw_so101_gripper_part1_v1.stl"]
xml = re.sub(r'\s*<mesh file="[^"]+"/>', "", xml)
xml = re.sub(r'\s*<material name="[^"]+"[^>]*/>', "", xml)
xml = xml.replace('material="sts3215_03a_v1_material"', 'rgba="0.1 0.1 0.1 1"')  # カメラの基板・レンズ
# 色: 関節の外装は黄色、サーボのまわりの箱も黄色、指は濃いグレー
xml = xml.replace('<geom group="3" condim="3"/>', '<geom group="3" condim="3" rgba="1 0.82 0.12 1"/>')
xml = xml.replace('priority="1" rgba="1.0 0 0 1.0" mass="0"/>', 'priority="1" rgba="0.18 0.18 0.2 1" mass="0"/>')
(dst / "so101.xml").write_text(xml)

used = set(re.findall(r'file="([^"]+)"', xml))
assert used == set(keep), used
for name in sorted(used):
    shutil.copy(src / "assets" / name, dst / "assets" / name)
shutil.copy(src / "LICENSE", dst / "LICENSE")
print(f"{len(used)} meshes, so101.xml {len(xml)} bytes")
