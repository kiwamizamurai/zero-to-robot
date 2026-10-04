#import "../common/theorems.typ": *

#show: setup-theorems

#set page(paper: "a4", margin: 2.5cm)
#set text(font: "New Computer Modern", size: 11pt, lang: "ja")
#set heading(numbering: "1.1")
#set math.equation(numbering: "(1)", supplement: "式")

#align(center)[
  #text(17pt, weight: "bold")[動力学 --- 2関節平面アームの運動方程式(ラグランジュ形式)]

  #text(11pt, fill: gray)[`02_two_link_dynamics.py` の数式的裏付け]
]

#v(1cm)

記法・証明技法は `01_kinematics/notes.typ` を踏襲する。$dot(theta)$ は $theta$ の時間微分、$dot.double(theta)$ は2階時間微分を表す。ラグランジュ方程式そのものの導出(Hamiltonの原理・d'Alembertの原理から)は解析力学の標準的な結果としてここでは証明せず、既知として用いる(例えば Goldstein, _Classical Mechanics_、または Asada, _Introduction to Robotics_ (MIT 2.12, Fall 2005, OCW) 7.2.1節を参照)。本稿の目的は、2関節平面アームに対して運動エネルギー・位置エネルギーを具体的に計算し、そこから運動方程式を導出する部分を省略なく行うことである。導出全体は Asada, 前掲書, Example 7.1 および 7.2.2節に基づく。

= 設定

運動方程式 $M(bm(theta))dot.double(bm(theta))+bm(c)(bm(theta),dot(bm(theta)))+bm(g)(bm(theta))=bm(tau)$ をラグランジュ形式で導出するには、まずロボットを「質量がどこにどれだけ乗っているか」で記述しなければならない。そこで `01_kinematics` と同じ順運動学($a=theta_1,b=theta_1+theta_2$)を持つ2リンク平面アームに、各リンクの質量 $m_1,m_2$・重心位置(関節から測った距離)$l_(c 1),l_(c 2)$・重心まわりの慣性モーメント $I_1,I_2$ を持たせる。関節角ベクトルを $bm(theta)=(theta_1,theta_2)$ とする。

#definition[
  関節1の位置を原点とし、リンク1の重心位置 $bm(c)_1$、関節2(肘)の位置 $bm(p)_1$、リンク2の重心位置 $bm(c)_2$ を
  $ bm(c)_1 (bm(theta)) := l_(c 1) vec(cos theta_1, sin theta_1), quad bm(p)_1 (bm(theta)) := l_1 vec(cos theta_1, sin theta_1), quad bm(c)_2 (bm(theta)) := bm(p)_1 (bm(theta)) + l_(c 2) vec(cos(theta_1+theta_2), sin(theta_1+theta_2)) $
  で定める。重力加速度ベクトルを $bm(g) := vec(0,-g)$ とする($g>0$は重力加速度の大きさ)。
]

= 運動エネルギーと位置エネルギー

3節でラグランジュ方程式を適用するには、ラグランジアン $L=T-U$ を $bm(theta),dot(bm(theta))$ の具体的な式として持っている必要がある。本節では、各リンクの重心速度(補題2.1)を経由して、運動エネルギー $T$ と位置エネルギー $U$ を閉じた式で求める(コードの `kinetic_energy`, `potential_energy` に対応)。

#lemma("重心速度")[
  $ bm(v)_(c 1) := (dif bm(c)_1)/(dif t) = l_(c 1) dot(theta)_1 vec(-sin theta_1, cos theta_1) $ <vc1>
  $ bm(v)_(c 2) := (dif bm(c)_2)/(dif t) = l_1 dot(theta)_1 vec(-sin theta_1, cos theta_1) + l_(c 2)(dot(theta)_1+dot(theta)_2) vec(-sin(theta_1+theta_2), cos(theta_1+theta_2)) $ <vc2>
]

#proof[
  合成関数の微分則を $bm(c)_1 (bm(theta)(t))$ に適用すると

  $ (dif bm(c)_1)/(dif t) = (partial bm(c)_1)/(partial theta_1) dot(theta)_1 = l_(c 1) vec(-sin theta_1, cos theta_1) dot(theta)_1 $

  これが#ref(<vc1>)である。$bm(c)_2 = bm(p)_1 + l_(c 2) vec(cos(theta_1+theta_2), sin(theta_1+theta_2))$ は $theta_1$ と $theta_1+theta_2$ の両方を通じて時間に依存するので、和の微分則と連鎖律より

  $ (dif bm(c)_2)/(dif t) = (dif bm(p)_1)/(dif t) + (dif)/(dif t) [l_(c 2) vec(cos(theta_1+theta_2), sin(theta_1+theta_2))] \
    = l_1 dot(theta)_1 vec(-sin theta_1, cos theta_1) + l_(c 2)(dot(theta)_1+dot(theta)_2) vec(-sin(theta_1+theta_2), cos(theta_1+theta_2)) $

  （最後の項は $theta_1+theta_2$ の時間微分が $dot(theta)_1+dot(theta)_2$ であることを用いた）。これが#ref(<vc2>)である。
]

各リンクの重心速度が求まったので、あとは「速さの2乗×質量/2」を足し合わせれば運動エネルギーに、「高さ×質量×重力加速度」を足し合わせれば位置エネルギーになる。

#theorem("運動エネルギー・位置エネルギー")[
  $ T(bm(theta),dot(bm(theta))) = 1/2 H_11 dot(theta)_1^2 + H_12 dot(theta)_1 dot(theta)_2 + 1/2 H_22 dot(theta)_2^2 $ <T-eq>
  $ H_11 := m_1 l_(c 1)^2 + I_1 + m_2(l_1^2+l_(c 2)^2+2 l_1 l_(c 2) cos theta_2) + I_2 $ <H-eq>
  $ H_22 := m_2 l_(c 2)^2 + I_2, quad H_12 := m_2(l_(c 2)^2+l_1 l_(c 2) cos theta_2) + I_2 $
  $ U(bm(theta)) = m_1 g l_(c 1) sin theta_1 + m_2 g (l_1 sin theta_1 + l_(c 2) sin(theta_1+theta_2)) $ <U-eq>
]

#proof[
  *(i) 運動エネルギー。* 各リンクの重心の並進運動エネルギーと、自身の重心まわりの回転運動エネルギーの和として

  $ T = 1/2 m_1 |bm(v)_(c 1)|^2 + 1/2 I_1 dot(theta)_1^2 + 1/2 m_2 |bm(v)_(c 2)|^2 + 1/2 I_2 (dot(theta)_1+dot(theta)_2)^2 $ <T-def>

  （リンク1の角速度は $dot(theta)_1$、リンク2の角速度は $dot(theta)_1+dot(theta)_2$ である）。#ref(<vc1>)より

  $ |bm(v)_(c 1)|^2 = l_(c 1)^2 dot(theta)_1^2 (sin^2 theta_1 + cos^2 theta_1) = l_(c 1)^2 dot(theta)_1^2 $ <vc1-norm>

  #ref(<vc2>)を成分ごとに書き下すと

  $ bm(v)_(c 2) = vec(
    -l_1 dot(theta)_1 sin theta_1 - l_(c 2)(dot(theta)_1+dot(theta)_2) sin(theta_1+theta_2),
    l_1 dot(theta)_1 cos theta_1 + l_(c 2)(dot(theta)_1+dot(theta)_2) cos(theta_1+theta_2)
  ) $

  各成分を2乗して加えると

  $ |bm(v)_(c 2)|^2 &= l_1^2 dot(theta)_1^2 sin^2 theta_1 + 2l_1l_(c 2)dot(theta)_1(dot(theta)_1+dot(theta)_2) sin theta_1 sin(theta_1+theta_2) + l_(c 2)^2(dot(theta)_1+dot(theta)_2)^2 sin^2 (theta_1+theta_2) \
    &quad + l_1^2 dot(theta)_1^2 cos^2 theta_1 + 2l_1l_(c 2)dot(theta)_1(dot(theta)_1+dot(theta)_2) cos theta_1 cos(theta_1+theta_2) + l_(c 2)^2(dot(theta)_1+dot(theta)_2)^2 cos^2 (theta_1+theta_2) $

  $sin^2+cos^2=1$ を使ってまとめると

  $ |bm(v)_(c 2)|^2 &= l_1^2 dot(theta)_1^2 + l_(c 2)^2(dot(theta)_1+dot(theta)_2)^2 \
    &quad + 2l_1l_(c 2)dot(theta)_1(dot(theta)_1+dot(theta)_2)[sin theta_1 sin(theta_1+theta_2)+cos theta_1 cos(theta_1+theta_2)] $

  加法定理 $cos(A-B)=cos A cos B + sin A sin B$ を $A=theta_1+theta_2, B=theta_1$ に用いると、角括弧の中は $cos((theta_1+theta_2)-theta_1) = cos theta_2$ に等しいので

  $ |bm(v)_(c 2)|^2 = l_1^2 dot(theta)_1^2 + l_(c 2)^2(dot(theta)_1+dot(theta)_2)^2 + 2l_1l_(c 2)cos theta_2 dot(theta)_1(dot(theta)_1+dot(theta)_2) $ <vc2-norm>

  $(dot(theta)_1+dot(theta)_2)^2 = dot(theta)_1^2+2dot(theta)_1dot(theta)_2+dot(theta)_2^2$ および $dot(theta)_1(dot(theta)_1+dot(theta)_2)=dot(theta)_1^2+dot(theta)_1dot(theta)_2$ を用いて#ref(<vc2-norm>)を展開し、$dot(theta)_1^2,dot(theta)_1dot(theta)_2,dot(theta)_2^2$ について整理すると

  $ |bm(v)_(c 2)|^2 = (l_1^2+l_(c 2)^2+2l_1l_(c 2)cos theta_2)dot(theta)_1^2 + (2l_(c 2)^2+2l_1l_(c 2)cos theta_2)dot(theta)_1dot(theta)_2 + l_(c 2)^2dot(theta)_2^2 $ <vc2-expand>

  同様に $1/2I_2(dot(theta)_1+dot(theta)_2)^2 = 1/2I_2dot(theta)_1^2+I_2dot(theta)_1dot(theta)_2+1/2I_2dot(theta)_2^2$ である。#ref(<vc1-norm>)、#ref(<vc2-expand>)、およびこの展開を#ref(<T-def>)に代入し、$dot(theta)_1^2,dot(theta)_1dot(theta)_2,dot(theta)_2^2$ の係数ごとにまとめる。

  $dot(theta)_1^2$ の係数:
  $ 1/2 m_1 l_(c 1)^2 + 1/2 I_1 + 1/2 m_2(l_1^2+l_(c 2)^2+2l_1l_(c 2)cos theta_2) + 1/2I_2 = 1/2 H_11 $

  $dot(theta)_1dot(theta)_2$ の係数:
  $ m_2(l_(c 2)^2+l_1l_(c 2)cos theta_2) + I_2 = H_12 $

  $dot(theta)_2^2$ の係数:
  $ 1/2 m_2 l_(c 2)^2 + 1/2 I_2 = 1/2 H_22 $

  これらは#ref(<H-eq>)の $H_11,H_12,H_22$ の定義に一致する。よって#ref(<T-eq>)を得る。

  *(ii) 位置エネルギー。* 質点系の重力位置エネルギーは $U=-sum_i m_i bm(g) dot bm(c)_i$ で与えられるので

  $ U = -m_1 bm(g) dot bm(c)_1 - m_2 bm(g) dot bm(c)_2 $

  $bm(g)=(0,-g)$ なので $bm(g) dot bm(c)_1 = -g (bm(c)_1)_y = -g l_(c 1) sin theta_1$、よって第1項は $-m_1 bm(g) dot bm(c)_1 = m_1 g l_(c 1) sin theta_1$。同様に $bm(g) dot bm(c)_2 = -g(bm(c)_2)_y = -g[l_1 sin theta_1 + l_(c 2) sin(theta_1+theta_2)]$ なので

  $ -m_2 bm(g) dot bm(c)_2 = m_2 g [l_1 sin theta_1 + l_(c 2) sin(theta_1+theta_2)] $

  両者を足すと#ref(<U-eq>)を得る。
]

= 運動方程式の導出

前節で求めた $T,U$ から、運動方程式 $M(bm(theta))dot.double(bm(theta))+bm(c)(bm(theta),dot(bm(theta)))+bm(g)(bm(theta))=bm(tau)$ の具体的な成分(質量行列の$H_11,H_12,H_22$、コリオリ・遠心力項$h$、重力項$G_1,G_2$)を求める。これらはそれぞれコードの `mass_matrix`・`coriolis_centrifugal`・`gravity` が計算する値である。導出にはラグランジアン $L(bm(theta),dot(bm(theta))) := T(bm(theta),dot(bm(theta))) - U(bm(theta))$ を作り、ラグランジュ方程式

$ (dif)/(dif t) (partial L)/(partial dot(theta)_i) - (partial L)/(partial theta_i) = tau_i, quad i=1,2 $ <lagrange-eq>

を用いる。

#theorem("運動方程式")[
  $ tau_1 = H_11 dot.double(theta)_1 + H_12 dot.double(theta)_2 - h dot(theta)_2^2 - 2h dot(theta)_1 dot(theta)_2 + G_1 $ <eom1>
  $ tau_2 = H_12 dot.double(theta)_1 + H_22 dot.double(theta)_2 + h dot(theta)_1^2 + G_2 $ <eom2>
  ここで
  $ h := m_2 l_1 l_(c 2) sin theta_2 $
  $ G_1 := m_1 g l_(c 1) cos theta_1 + m_2 g(l_(c 2) cos(theta_1+theta_2) + l_1 cos theta_1), quad G_2 := m_2 g l_(c 2) cos(theta_1+theta_2) $
  である(コードの `mass_matrix`, `coriolis_centrifugal`, `gravity` に対応)。
]

#proof[
  $H_11,H_12$ は $theta_2$ のみに依存し($theta_1$には依存しない)、$H_22$ は定数であることに注意する。

  *(i) $theta_1$ に関する方程式。* $U$ は $dot(theta)_1$ に依存しないので

  $ (partial L)/(partial dot(theta)_1) = (partial T)/(partial dot(theta)_1) = H_11 dot(theta)_1 + H_12 dot(theta)_2 $

  積の微分則と連鎖律($H_11,H_12$は$theta_2(t)$を通じてのみ時間に依存)より

  $ (dif)/(dif t) (partial L)/(partial dot(theta)_1) = H_11 dot.double(theta)_1 + H_12 dot.double(theta)_2 + (dif H_11)/(dif t) dot(theta)_1 + (dif H_12)/(dif t) dot(theta)_2 $

  $ (dif H_11)/(dif t) = (partial H_11)/(partial theta_2) dot(theta)_2 = -2 m_2 l_1 l_(c 2) sin theta_2 dot dot(theta)_2 = -2h dot(theta)_2 $
  $ (dif H_12)/(dif t) = (partial H_12)/(partial theta_2) dot(theta)_2 = -m_2 l_1 l_(c 2) sin theta_2 dot dot(theta)_2 = -h dot(theta)_2 $

  なので

  $ (dif)/(dif t) (partial L)/(partial dot(theta)_1) = H_11 dot.double(theta)_1 + H_12 dot.double(theta)_2 - 2h dot(theta)_1 dot(theta)_2 - h dot(theta)_2^2 $ <lhs1>

  一方、$T$は$H_11,H_12,H_22$のみを通じて$bm(theta)$に依存するが、これらはいずれも$theta_1$を含まないので $(partial T)/(partial theta_1)=0$。ゆえに

  $ (partial L)/(partial theta_1) = -(partial U)/(partial theta_1) = -[m_1 g l_(c 1) cos theta_1 + m_2 g (l_(c 2) cos(theta_1+theta_2) + l_1 cos theta_1)] = -G_1 $ <rhs1>

  #ref(<lagrange-eq>)に#ref(<lhs1>)と#ref(<rhs1>)を代入すると

  $ tau_1 = [H_11 dot.double(theta)_1 + H_12 dot.double(theta)_2 - 2h dot(theta)_1 dot(theta)_2 - h dot(theta)_2^2] - (-G_1) $

  であり、これは#ref(<eom1>)に等しい。

  *(ii) $theta_2$ に関する方程式。* $H_22$は定数なので

  $ (partial L)/(partial dot(theta)_2) = H_12 dot(theta)_1 + H_22 dot(theta)_2 $

  $ (dif)/(dif t) (partial L)/(partial dot(theta)_2) = H_12 dot.double(theta)_1 + (dif H_12)/(dif t) dot(theta)_1 + H_22 dot.double(theta)_2 = H_12 dot.double(theta)_1 + H_22 dot.double(theta)_2 - h dot(theta)_1 dot(theta)_2 $ <lhs2>

  次に $(partial T)/(partial theta_2)$ を計算する。$T$の$theta_2$への依存は$H_11,H_12$を通じてのみ($H_22$は定数)なので、#ref(<T-eq>)より

  $ (partial T)/(partial theta_2) = 1/2 (partial H_11)/(partial theta_2) dot(theta)_1^2 + (partial H_12)/(partial theta_2) dot(theta)_1 dot(theta)_2 = 1/2(-2h) dot(theta)_1^2 + (-h) dot(theta)_1 dot(theta)_2 = -h dot(theta)_1^2 - h dot(theta)_1 dot(theta)_2 $

  また、$U$の$theta_2$依存は$m_2 g l_(c 2) sin(theta_1+theta_2)$の項のみなので

  $ (partial U)/(partial theta_2) = m_2 g l_(c 2) cos(theta_1+theta_2) = G_2 $

  よって

  $ (partial L)/(partial theta_2) = (partial T)/(partial theta_2) - (partial U)/(partial theta_2) = -h dot(theta)_1^2 - h dot(theta)_1 dot(theta)_2 - G_2 $ <rhs2>

  #ref(<lagrange-eq>)に#ref(<lhs2>)と#ref(<rhs2>)を代入すると

  $ tau_2 = [H_12 dot.double(theta)_1 + H_22 dot.double(theta)_2 - h dot(theta)_1 dot(theta)_2] - [-h dot(theta)_1^2 - h dot(theta)_1 dot(theta)_2 - G_2] $

  右辺の $-h dot(theta)_1 dot(theta)_2$ と $+h dot(theta)_1 dot(theta)_2$ が打ち消し合い

  $ tau_2 = H_12 dot.double(theta)_1 + H_22 dot.double(theta)_2 + h dot(theta)_1^2 + G_2 $

  であり、これは#ref(<eom2>)に等しい。
]

#remark[
  この結果が言っているのは、2関節アームには単振り子にはない2つの効果があるということだ。1つは慣性そのものが姿勢によって変わること——質量行列の成分 $H_11$ が $theta_2$ に依存するのは、肘の曲げ方によって腕全体の「振り回しにくさ」が変わるからである。もう1つは2つの関節が互いに引きずり合う結合項で、$h dot(theta)_2^2$(遠心力項)や $2h dot(theta)_1 dot(theta)_2$(コリオリ力項)がそれに当たり、どちらも $h=m_2 l_1 l_(c 2) sin theta_2$ という同じ量から生じている。$G_1,G_2$ は単に各関節が支えるべき重力トルクである。

  同じ運動方程式は、各リンクに対するNewton-Euler方程式(並進運動に対するNewtonの運動方程式・重心まわりの回転運動に対するEulerの運動方程式)を個別に立て、リンク間の拘束力を消去することによっても得られる(Asada, 前掲書, Example 7.1)。ラグランジュ法はエネルギーの微分だけで運動方程式が体系的に得られるのに対し、Newton-Euler法は各リンクに働く力・トルクの物理的な意味が明確という特徴がある。本稿ではラグランジュ法による導出のみを完全に行い、Newton-Euler法との一致はAsadaの原著の計算に委ねる。
]

#remark[
  `02_two_link_dynamics.py` では、上記の $M(bm(theta)) dot.double(bm(theta)) + bm(c)(bm(theta),dot(bm(theta))) + bm(g)(bm(theta)) = bm(tau)$ という形の式(コードの `inverse_dynamics`。$M(bm(theta))=mat(H_11,H_12;H_12,H_22)$）を、$L=T-U$ のみから計算した有限差分によるラグランジュ方程式の左辺と比較しており(モデルに依らない検算)、さらに外力なしでの自由運動でエネルギー $T+U$ が保存されることも確認している。
]
