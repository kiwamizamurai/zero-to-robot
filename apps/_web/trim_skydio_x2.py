"""Menagerie の skydio_x2 から、ブラウザ用に軽量化した x2.xml とメッシュを作る。

ブラウザの描画は rgba の色だけを使い、テクスチャ(PNG、約 0.3MB)は読まないので捨てる。
機体の見た目のメッシュ(X2_lowpoly.obj)と物理(慣性・推力・ミキサー用のサイト)は元のモデルのまま。
質量を持つ見えない楕円体は、描かれないように group 3(衝突用)にしておく。

使い方: python _web/trim_skydio_x2.py <mujoco_menagerie/skydio_x2>
"""
import re
import shutil
import sys
from pathlib import Path

src = Path(sys.argv[1])
dst = Path(__file__).parent / "assets" / "skydio_x2"
(dst / "assets").mkdir(parents=True, exist_ok=True)

xml = (src / "x2.xml").read_text()
xml = re.sub(r'\s*<texture [^>]*/>', "", xml)
xml = re.sub(r'\s*<material name="phong3SG"[^>]*/>', "", xml)
xml = xml.replace('<geom material="phong3SG" mesh="X2_lowpoly"', '<geom rgba="0.16 0.17 0.19 1" mesh="X2_lowpoly"')
xml = xml.replace('class="visual" material="invisible"/>', 'class="visual" group="3"/>')
xml = re.sub(r'\s*<material name="invisible"[^>]*/>', "", xml)
(dst / "x2.xml").write_text(xml)
shutil.copy(src / "assets" / "X2_lowpoly.obj", dst / "assets" / "X2_lowpoly.obj")
shutil.copy(src / "LICENSE", dst / "LICENSE")
print(f"x2.xml {len(xml)} bytes")
