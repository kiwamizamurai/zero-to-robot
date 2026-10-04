#import "../common/theorems.typ": *

#show: setup-theorems

#set page(paper: "a4", margin: 2.5cm)
#set text(font: "New Computer Modern", size: 11pt, lang: "ja")
#set heading(numbering: "1.1")
#set math.equation(numbering: "(1)", supplement: "式")

#align(center)[
  #text(17pt, weight: "bold")[制御 --- 計算トルク制御(フィードバック線形化)]

  #text(11pt, fill: gray)[`02_computed_torque_control.py` の数式的裏付け]
]

#v(1cm)

記法・証明技法は `01_kinematics/notes.typ` を踏襲する。$dot(theta)$は$theta$の時間微分、$dot.double(theta)$は2階時間微分。動力学モデル $M(bm(theta)) dot.double(bm(theta)) + bm(c)(bm(theta),dot(bm(theta))) + bm(g)(bm(theta)) = bm(tau)$ は `02_dynamics/notes.typ` で証明済みの結果をそのまま使う。導出は Lynch and Park, _Modern Robotics: Mechanics, Planning, and Control_ (Cambridge University Press, 2017), 11.2.2.3節(式11.19-11.21)に基づく。

= 計算トルク制御則と閉ループ誤差ダイナミクス

`01_pd_control.py`のPD制御は目標角度が一定(setpoint)の場合しか扱えない。時間変化する目標軌道 $bm(theta)_d (t)$ に追従させるには、アームの非線形な動力学($M,bm(c),bm(g)$)を打ち消す項が要る。ここでは計算トルク制御(フィードバック線形化)を導入し、モデルが正確であれば閉ループの誤差ダイナミクスが、非線形力学に一切依存しないゲイン $bm(K)_p,bm(K)_d$ だけの線形方程式に帰着することを示す。これがコードの`computed_torque_control`が実装する制御則である。

目標軌道を $bm(theta)_d (t)$(2回微分可能で、$dot(bm(theta))_d, dot.double(bm(theta))_d$ が既知)とし、追従誤差を $bm(theta)_e (t) := bm(theta)_d (t) - bm(theta)(t)$ と定める。コントローラが使う動力学モデルを $tilde(M), tilde(bm(c)), tilde(bm(g))$ とする(実際のプラントの $M,bm(c),bm(g)$ と一致するとは限らない)。

#definition[
  ゲイン行列 $bm(K)_p, bm(K)_d in RR^(2 times 2)$(対称正定値、本稿では対角行列)を用いて、計算トルク制御則を
  $ bm(tau) := tilde(M)(bm(theta)) [dot.double(bm(theta))_d + bm(K)_p bm(theta)_e + bm(K)_d dot(bm(theta))_e] + tilde(bm(c))(bm(theta),dot(bm(theta))) + tilde(bm(g))(bm(theta)) $ <ctc-law>
  で定める(コードの `computed_torque_control` に対応)。
]

この制御則を実際にプラントの運動方程式に代入すると、追従誤差 $bm(theta)_e$ がどんな方程式に従うかが分かる。

#theorem[
  モデルが正確( $tilde(M)=M, tilde(bm(c))=bm(c), tilde(bm(g))=bm(g)$ )であれば、#ref(<ctc-law>)を適用したときの閉ループの誤差ダイナミクスは
  $ dot.double(bm(theta))_e + bm(K)_d dot(bm(theta))_e + bm(K)_p bm(theta)_e = bm(0) $ <error-dynamics>
  である。
]

#proof[
  プラントの運動方程式 $M(bm(theta)) dot.double(bm(theta)) + bm(c)(bm(theta),dot(bm(theta))) + bm(g)(bm(theta)) = bm(tau)$ に#ref(<ctc-law>)を代入し、$tilde(M)=M,tilde(bm(c))=bm(c),tilde(bm(g))=bm(g)$ を用いると

  $ M(bm(theta)) dot.double(bm(theta)) + bm(c)(bm(theta),dot(bm(theta))) + bm(g)(bm(theta)) = M(bm(theta))[dot.double(bm(theta))_d + bm(K)_p bm(theta)_e + bm(K)_d dot(bm(theta))_e] + bm(c)(bm(theta),dot(bm(theta))) + bm(g)(bm(theta)) $

  両辺から $bm(c)(bm(theta),dot(bm(theta))) + bm(g)(bm(theta))$ を引くと

  $ M(bm(theta)) dot.double(bm(theta)) = M(bm(theta))[dot.double(bm(theta))_d + bm(K)_p bm(theta)_e + bm(K)_d dot(bm(theta))_e] $

  $M(bm(theta))$ は正定値(`02_dynamics/notes.typ` の運動エネルギーが $T=1/2 dot(bm(theta))^T M dot(bm(theta)) >= 0$ で、$dot(bm(theta)) eq.not bm(0)$ なら$T>0$であることから従う)ゆえ正則なので、左から $M(bm(theta))^(-1)$ を掛けると

  $ dot.double(bm(theta)) = dot.double(bm(theta))_d + bm(K)_p bm(theta)_e + bm(K)_d dot(bm(theta))_e $ <actual-accel>

  $bm(theta)_e = bm(theta)_d - bm(theta)$ の定義より $dot.double(bm(theta))_e = dot.double(bm(theta))_d - dot.double(bm(theta))$ である。#ref(<actual-accel>)を代入すると

  $ dot.double(bm(theta))_e = dot.double(bm(theta))_d - [dot.double(bm(theta))_d + bm(K)_p bm(theta)_e + bm(K)_d dot(bm(theta))_e] = -bm(K)_p bm(theta)_e - bm(K)_d dot(bm(theta))_e $

  移項すると#ref(<error-dynamics>)を得る。
]

#remark[
  #ref(<error-dynamics>)は、実際のロボットの非線形力学 $M,bm(c),bm(g)$ に一切依存しない、$bm(K)_p,bm(K)_d$ だけで決まる線形の誤差ダイナミクスである。これが「フィードバック線形化」と呼ばれる所以である。$bm(K)_p,bm(K)_d$ が対角行列なら、2つの関節の誤差ダイナミクスは互いに独立な2階線形常微分方程式に分離する。
]

= 各関節の誤差応答(臨界減衰)

1節の誤差ダイナミクスはゲイン $bm(K)_p,bm(K)_d$ をどう選ぶかによって、振動しながら収束したり、逆に収束が遅くなったりする。ここでは振動せず最速で収束するゲイン比(臨界減衰)を選び、そのときの誤差応答を閉じた式で求める。これがコードの`critically_damped_gains`・`ideal_error_response`が計算する理論解であり、`02_computed_torque_control.py`ではこの理論解とシミュレーション結果を突き合わせて検算する。

$bm(K)_p = "diag"(k_(p,1),k_(p,2))$, $bm(K)_d = "diag"(k_(d,1),k_(d,2))$ のとき、#ref(<error-dynamics>)は関節ごとに独立な方程式

$ dot.double(theta)_(e,i) + k_(d,i) dot(theta)_(e,i) + k_(p,i) theta_(e,i) = 0, quad i=1,2 $ <scalar-ode>

に分離する。

#theorem[
  $k_(d,i) = 2 sqrt(k_(p,i))$(臨界減衰)のとき、#ref(<scalar-ode>)の解は $omega_n := sqrt(k_(p,i))$ とおくと
  $ theta_(e,i)(t) = (c_1 + c_2 t) e^(-omega_n t), quad c_1 = theta_(e,i)(0), quad c_2 = dot(theta)_(e,i)(0) + omega_n c_1 $ <critically-damped-sol>
  で与えられる(コードの `ideal_error_response` に対応)。
]

#proof[
  #ref(<scalar-ode>)の特性方程式は $s^2+k_(d,i)s+k_(p,i)=0$。判別式は $k_(d,i)^2-4k_(p,i) = 4k_(p,i)-4k_(p,i)=0$($k_(d,i)=2sqrt(k_(p,i))$を代入)なので、重解 $s=-k_(d,i)/2 = -sqrt(k_(p,i)) = -omega_n$ を持つ。

  重解を持つ2階線形常微分方程式 $dot.double(x)-2s_0 dot(x)+s_0^2 x=0$(本稿では $s_0=-omega_n$)の一般解が $x(t)=(c_1+c_2 t)e^(s_0 t)$ で与えられることを確認する。$x(t)=(c_1+c_2t)e^(s_0t)$ に対して

  $ dot(x)(t) = c_2 e^(s_0 t) + s_0(c_1+c_2t)e^(s_0t) = [c_2+s_0(c_1+c_2t)] e^(s_0 t) $
  $ dot.double(x)(t) = s_0 c_2 e^(s_0t) + s_0[c_2+s_0(c_1+c_2t)]e^(s_0t) = [2s_0c_2 + s_0^2(c_1+c_2t)] e^(s_0t) $

  なので

  $ dot.double(x) - 2s_0 dot(x) + s_0^2 x = [2s_0c_2+s_0^2(c_1+c_2t) - 2s_0(c_2+s_0(c_1+c_2t)) + s_0^2(c_1+c_2t)] e^(s_0t) \
    = [2s_0c_2 - 2s_0c_2 + (s_0^2-2s_0^2+s_0^2)(c_1+c_2t)] e^(s_0t) = 0 $

  よって$x(t)=(c_1+c_2t)e^(s_0t)$は任意の$c_1,c_2$に対して解であり、2階線形常微分方程式の解空間は2次元なので、これが一般解である。$s_0=-omega_n$を代入すると#ref(<scalar-ode>)の一般解 $theta_(e,i)(t)=(c_1+c_2t)e^(-omega_n t)$ を得る。

  初期条件を課す。$theta_(e,i)(0)=c_1$ よりただちに $c_1=theta_(e,i)(0)$。また

  $ dot(theta)_(e,i)(t) = c_2 e^(-omega_n t) - omega_n (c_1+c_2 t) e^(-omega_n t) $

  に $t=0$ を代入すると $dot(theta)_(e,i)(0) = c_2 - omega_n c_1$、すなわち $c_2 = dot(theta)_(e,i)(0)+omega_n c_1$。これが#ref(<critically-damped-sol>)である。
]

#remark[
  `02_computed_torque_control.py` では、モデルが正確な場合(`model_scale=1.0`)にシミュレーションした追従誤差が、#ref(<critically-damped-sol>)による理論解と最大誤差 $10^(-11)$ 程度で一致することを確認している。一方、コントローラ内部のモデルで $m_2$ を20%過大評価した場合(`model_scale=1.2`)は、#ref(<error-dynamics>)の前提($tilde(M)=M$等)が崩れるため理論解から明確にズレる(最大ズレは完全なモデルの場合の10倍以上)。これは Lynch and Park, 前掲書, Figure 11.10 が示す「不正確なモデルによるフィードフォワード制御はズレを生む」という現象の、フィードバック線形化バージョンである。

  `01_pd_control.py`(振り子へのPD制御+重力補償)との対応: あちらは1自由度・一定目標角度(setpoint)に対して重力項のみを打ち消していたのに対し、本稿は2自由度・時間変化する目標軌道(トラッキング)に対して質量行列・遠心力/コリオリ力項・重力項の全てを打ち消す、その一般化になっている。
]
