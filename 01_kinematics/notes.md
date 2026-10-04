# 運動学 — 2関節平面アームの逆運動学から3次元・冗長マニピュレータまで

`01_forward_kinematics.py` 〜 `08_redundant_ik_nullspace.py` の数式的裏付け。ソースは [`notes.typ`](notes.typ)（[`notes.pdf`](notes.pdf) にコンパイル済み）。このページはGitHub上でTypstをインストールせずに数式を読めるように、コンパイル結果をページ画像として埋め込んだものです。

前半(1-5節)は、回転行列の性質(加法性・直交性・微分・偏角の保存)から出発し、2関節平面アームのヤコビ行列・勾配降下法の勾配・公式解を、添字を使わない行列形式で行間を省略せずに証明します。中盤(6-10節)は3次元へ一般化し、skew行列の性質、Rodriguesの回転公式、クォータニオンとの対応、`n`関節・3次元への一般化された幾何ヤコビ行列(`z_(i-1) × (o_n - o_(i-1))`)、特異点と可操作度を証明します(Lynch and Park, *Modern Robotics*; Craig, *Introduction to Robotics*; Asada, MIT 2.12 に基づく)。後半(11-12節)は、標準DH規約と修正DH(Craig)規約の厳密な対応関係(帰納法による証明)、そして冗長マニピュレータの擬似逆行列IKと零空間射影の性質(射影行列であること・最小ノルム性・零空間法)を証明します(Buss, CMU, 2009に基づく)。

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

---

![page 18](notes_pages/page-18.png)

---

![page 19](notes_pages/page-19.png)
