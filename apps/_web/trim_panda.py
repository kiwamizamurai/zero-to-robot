"""Menagerie の franka_emika_panda から、ブラウザ用に軽量化した panda.xml とメッシュを作る。

描画用(visual)メッシュは合計 33MB あるので捨て、衝突用メッシュだけ(約 0.4MB)を使う。
物理は元のモデルと同じ。IK 用の手先の目印 tcp も hand に足す(pick_place.py:build_model と同じ位置)。

使い方: python _web/trim_panda.py <mujoco_menagerie/franka_emika_panda>
"""
import re
import shutil
import sys
from pathlib import Path

src = Path(sys.argv[1])
dst = Path(__file__).parent / "assets" / "panda"
(dst / "assets").mkdir(parents=True, exist_ok=True)

xml = (src / "panda.xml").read_text()
xml = re.sub(r'^\s*<geom [^>]*class="visual"[^>]*/>\n', "", xml, flags=re.M)
xml = re.sub(r"<!-- Visual meshes -->.*?(?=\n  </asset>)", "", xml, flags=re.S)
xml = xml.replace('<mesh name="hand_c" file="hand.stl"/>',
                  '<mesh name="hand_c" file="hand.stl"/>\n    <mesh file="finger_0.obj"/>')  # 指の衝突にも使う
xml = xml.replace('<geom mesh="hand_c" class="collision"/>',
                  '<geom mesh="hand_c" class="collision"/>\n                      <site name="tcp" pos="0 0 0.1034"/>')
xml = xml.replace('<light name="top" pos="0 0 2" mode="trackcom"/>\n    ', "")
(dst / "panda.xml").write_text(xml)

used = set(re.findall(r'file="([^"]+)"', xml))
for name in sorted(used):
    shutil.copy(src / "assets" / name, dst / "assets" / name)
shutil.copy(src / "LICENSE", dst / "LICENSE")
print(f"{len(used)} meshes, panda.xml {len(xml)} bytes")
