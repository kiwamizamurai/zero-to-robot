# 軌道計画 — 多項式・台形・S字速度プロファイル・経由点軌道の証明

`01_polynomial_trajectories.py` / `02_trapezoidal_velocity_profile.py` / `03_scurve_velocity_profile.py` / `05_via_point_trajectory.py` の数式的裏付け。ソースは [`notes.typ`](notes.typ)（[`notes.pdf`](notes.pdf) にコンパイル済み）。このページはGitHub上でTypstをインストールせずに数式を読めるように、コンパイル結果をページ画像として埋め込んだものです。

3次・5次時間スケーリングを境界条件から導出し、5次時間スケーリング(Flash and Hogan, 1985 の最小躍度モデル)が躍度(jerk)の2乗積分を変分法の意味で最小化する唯一の関数であることを証明します(Lynch and Park, *Modern Robotics* 9.2節に基づく)。台形速度プロファイル(LSPB)を境界条件から導出し、3つのパラメータ化(v,a→T / v,T→a / a,T→v)の式、3段階(coast区間あり)になるための条件、そして台形プロファイルが速度・加速度の上限のもとで時間最適であることを、初等的な(Pontryaginの最大値原理を使わない)議論で完全に証明します(Modern Roboticsが演習問題Exercise 8, 10, 11, 12に委ねている内容を自前で解いたもの)。S字(ジャーク制限)速度プロファイルを7段階のジャーク関数から構成し、立ち上がり時間・変位の閉じた式、そして台形プロファイルよりちょうど`a/J`だけ長い時間を要するという関係を、対称性を使った証明で導出します(BYU ME 537コース資料 "S-Curve Equations for a Trajectory Generator" の構成を自前の記法に翻訳)。最後に、経路と時間スケーリングを分離せず関節の時間履歴を直接補間する多項式経由点軌道(Modern Robotics 9.3節)の係数を導出し、複数の経由点を通過する軌道が速度連続(だが一般に加速度不連続)であること、そして2経由点・零速度の特別な場合には3次時間スケーリングに一致することを証明します。

---

![page 1](notes_pages/page-1.png)

---

![page 2](notes_pages/page-2.png)

---

![page 3](notes_pages/page-3.png)

---

![page 4](notes_pages/page-4.png)

---

![page 5](notes_pages/page-5.png)

---

![page 6](notes_pages/page-6.png)

---

![page 7](notes_pages/page-7.png)

---

![page 8](notes_pages/page-8.png)

---

![page 9](notes_pages/page-9.png)

---

![page 10](notes_pages/page-10.png)

---

![page 11](notes_pages/page-11.png)

---

![page 12](notes_pages/page-12.png)

---

![page 13](notes_pages/page-13.png)

---

![page 14](notes_pages/page-14.png)

---

![page 15](notes_pages/page-15.png)

---

![page 16](notes_pages/page-16.png)

---

![page 17](notes_pages/page-17.png)
