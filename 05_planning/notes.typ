#import "../common/theorems.typ": *

#show: setup-theorems

#set page(paper: "a4", margin: 2.5cm)
#set text(font: "New Computer Modern", size: 11pt, lang: "ja")
#set heading(numbering: "1.1")
#set math.equation(numbering: "(1)", supplement: "式")

#align(center)[
  #text(17pt, weight: "bold")[軌道計画 --- 多項式時間スケーリングと最小躍度軌道]

  #text(11pt, fill: gray)[`01_polynomial_trajectories.py` の数式的裏付け]
]

#v(1cm)

記法・証明技法は `01_kinematics/notes.typ` を踏襲する。$dot(s)$は$s$の時間微分。本稿では、微分積分学の基本的な結果として部分積分(積の微分則の積分形)と、次の変分法の基本補題を断りなく用いる: 区間$[0,T]$上の連続関数$f$が、$(0,T)$の内部で何回でも微分可能かつ端点で高階微分も含めて0になる任意の関数$phi$に対して $integral_0^T f(t)phi(t) dif t = 0$ を満たすならば、$f equiv 0$ である(標準的な変分法の教科書に載っている事実)。時間スケーリング $s(t): [0,T] arrow [0,1]$ を用いた関節空間軌道の構成は Lynch and Park, _Modern Robotics: Mechanics, Planning, and Control_ (Cambridge University Press, 2017), 9.2節に基づく。

= 設定

本稿の目標は、関節を始点から終点まで滑らかに動かす軌道 $theta(t)$ を、速度・加速度・躍度に対する異なる要求(多項式・台形・S字・複数経由点)ごとに構成し、それぞれが何を達成し何を犠牲にしているかを証明することである。共通の骨格として、経路を$[0,1]$のパラメータ$s$で表し、時間スケーリング $s(t)$ を掛け合わせて軌道を作るという構成(2-6節)を先に固定する。

#definition[
  始点 $theta_(s)$、終点 $theta_(e)$、所要時間 $T>0$ に対し、時間スケーリング $s: [0,T] arrow [0,1]$ を用いて関節軌道を
  $ theta(t) := theta_(s) + s(t)(theta_(e) - theta_(s)) $
  で定める(コードの `joint_trajectory` に対応)。$dot(theta)(t)=dot(s)(t)(theta_(e)-theta_(s))$、$dot.double(theta)(t)=dot.double(s)(t)(theta_(e)-theta_(s))$ である。
]

= 3次・5次多項式時間スケーリング

時間スケーリング $s(t)$ の最も単純な作り方は、境界条件(両端点での位置・速度、必要なら加速度)だけを課して多項式の係数を決めることである。ここでは3次(位置・速度のみ指定)と5次(加速度まで指定)の2通りを構成し、両者の違い(加速度の連続性)を明らかにする。この違いが、後の台形・S字プロファイルとの比較の基準になる。3次はコードの`cubic_time_scaling`、5次は`quintic_time_scaling`に対応する。

#theorem("3次時間スケーリング")[
  $s(0)=dot(s)(0)=0$, $s(T)=1,dot(s)(T)=0$ を満たす3次多項式 $s(t)=a_0+a_1t+a_2t^2+a_3t^3$ は
  $ s(t) = 3(t/T)^2 - 2(t/T)^3 $ <cubic>
  で与えられる(コードの `cubic_time_scaling` に対応)。
]

#proof[
  $s(0)=a_0=0$。$dot(s)(t)=a_1+2a_2t+3a_3t^2$ より $dot(s)(0)=a_1=0$。残る条件は

  $ s(T) = a_2T^2+a_3T^3 = 1, quad dot(s)(T) = 2a_2T+3a_3T^2=0 $

  第2式より $a_2 = -3a_3T/2$。第1式に代入すると $-3a_3T^3/2+a_3T^3=1$、すなわち $-a_3T^3/2=1$ より $a_3=-2/T^3$。ゆえに $a_2=-3(-2/T^3)T/2=3/T^2$。よって

  $ s(t) = 3/T^2 t^2 - 2/T^3 t^3 = 3(t/T)^2-2(t/T)^3 $

  であり、これは#ref(<cubic>)に等しい。
]

#remark[
  #ref(<cubic>)の2階微分は $dot.double(s)(t) = 6/T^2-12t/T^3$ であり、$dot.double(s)(0)=6/T^2 eq.not 0$ である。したがって、静止状態($dot.double(theta)=0$)からこの軌道を開始すると、$t=0$の瞬間に加速度が0から$6/T^2 (theta_e-theta_s)$へ不連続に跳躍する。これは無限大の躍度(jerkの)を意味し、実機では振動の原因になりうる(`01_polynomial_trajectories.py`で数値的に確認)。
]

この3次多項式の欠点(加速度の不連続)を解消するには、加速度についても両端で境界条件を課せばよい。

#theorem("5次時間スケーリング")[
  $s(0)=dot(s)(0)=dot.double(s)(0)=0$, $s(T)=1,dot(s)(T)=0,dot.double(s)(T)=0$ を満たす5次多項式 $s(t)=a_0+dots+a_5t^5$ は
  $ s(t) = 10(t/T)^3 - 15(t/T)^4 + 6(t/T)^5 $ <quintic>
  で与えられる(コードの `quintic_time_scaling` に対応)。
]

#proof[
  $s(0)=a_0=0$。$dot(s)(0)=a_1=0$。$dot.double(s)(t)=2a_2+6a_3t+12a_4t^2+20a_5t^3$ より $dot.double(s)(0)=2a_2=0$、すなわち $a_2=0$。

  よって $s(t)=a_3t^3+a_4t^4+a_5t^5$。無次元の係数 $A,B,C$ を $a_3=A/T^3,a_4=B/T^4,a_5=C/T^5$ とおくと、残る3条件は

  $ s(T)=A+B+C=1 $ <c1>
  $ dot(s)(T)=3A+4B+5C=0 $ <c2>
  $ dot.double(s)(T)=6A+12B+20C=0 $ <c3>

  #ref(<c3>)を2で割ると $3A+6B+10C=0$。これから#ref(<c2>)を引くと $2B+5C=0$、すなわち
  $ B=-5C/2 $ <b-eq>

  #ref(<c1>)より $A=1-B-C$。#ref(<b-eq>)を代入すると
  $ A=1-(-5C/2)-C=1+3C/2 $ <a-eq>

  #ref(<a-eq>)と#ref(<b-eq>)を#ref(<c2>)に代入すると

  $ 3(1+3C/2)+4(-5C/2)+5C=0 $
  $ 3+9C/2-10C+5C=0 $
  $ 3 + (9/2-10+5)C=0 $
  $ 3 - C/2=0 $

  よって $C=6$。#ref(<b-eq>)より $B=-15$。#ref(<a-eq>)より $A=1+9=10$。したがって $a_3=10/T^3,a_4=-15/T^4,a_5=6/T^5$ であり

  $ s(t) = 10/T^3 t^3 - 15/T^4 t^4 + 6/T^5 t^5 = 10(t/T)^3-15(t/T)^4+6(t/T)^5 $

  これは#ref(<quintic>)に等しい。
]

= 5次時間スケーリングは躍度を最小化する(最小躍度軌道)

前節では5次多項式を「加速度まで境界条件を課す」という設計上の要請から導いたが、実はこの5次多項式には設計者の意図を超えた意味がある: 躍度(jerk, 加速度の時間微分 $dot.triple(s)$)の2乗を積分した量を、同じ境界条件を満たす関数の中で最小にする、ただ一つの関数になっているのである。これは Flash and Hogan (1985) の最小躍度モデル(人間の腕の到達運動のモデルとしても知られる)の数学的な内容であり、本節では変分法を使ってこれを証明する。

#definition[
  6つの境界条件 $s(0)=0,dot(s)(0)=0,dot.double(s)(0)=0,s(T)=1,dot(s)(T)=0,dot.double(s)(T)=0$ を満たす($[0,T]$上3回連続微分可能な)関数全体の集合を $cal(A)$ とし、$cal(A)$上の汎関数(躍度の2乗の積分)を
  $ J[s] := integral_0^T dot.triple(s)(t)^2 dif t $
  で定める。
]

汎関数 $J$ を最小化する関数を、変分法の標準的な手順(停留条件を求め、部分積分でオイラー・ラグランジュ方程式に帰着させる)で特定する。

#theorem[
  #ref(<quintic>)の5次時間スケーリング $s^*$ は $cal(A)$上で $J$ を最小化する唯一の関数である。
]

#proof[
  *(i) 停留条件。* $s^* in cal(A)$ とし、$phi$を、$(0,T)$上で何回でも微分可能かつ $phi(0)=phi(T)=0,dot(phi)(0)=dot(phi)(T)=0,dot.double(phi)(0)=dot.double(phi)(T)=0$(斉次境界条件)を満たす任意の関数とする。任意の$epsilon in RR$に対して $s^*+epsilon phi in cal(A)$ である(境界条件は線形で、$phi$は斉次境界条件を満たすため)。

  $ J(epsilon) := J[s^*+epsilon phi] = integral_0^T (dot.triple(s^*)(t)+epsilon dot.triple(phi)(t))^2 dif t $

  を$epsilon$の関数と見ると

  $ (dif J)/(dif epsilon) = integral_0^T 2(dot.triple(s^*)+epsilon dot.triple(phi)) dot.triple(phi) dif t $

  $epsilon=0$ での値は

  $ (dif J)/(dif epsilon) |_(epsilon=0) = 2 integral_0^T dot.triple(s^*)(t) dot.triple(phi)(t) dif t $ <first-variation>

  部分積分を3回行う。1回目($dot.double(phi)(0)=dot.double(phi)(T)=0$により境界項が消える):

  $ integral_0^T dot.triple(s^*) dot.triple(phi) dif t = [dot.triple(s^*) dot.double(phi)]_0^T - integral_0^T s^*_((4)) dot.double(phi) dif t = -integral_0^T s^*_((4)) dot.double(phi) dif t $

  （$s^*_((4))$は$s^*$の4階微分）。2回目($dot(phi)(0)=dot(phi)(T)=0$により境界項が消える):

  $ -integral_0^T s^*_((4)) dot.double(phi) dif t = -[s^*_((4)) dot(phi)]_0^T + integral_0^T s^*_((5)) dot(phi) dif t = integral_0^T s^*_((5)) dot(phi) dif t $

  3回目($phi(0)=phi(T)=0$により境界項が消える):

  $ integral_0^T s^*_((5)) dot(phi) dif t = [s^*_((5)) phi]_0^T - integral_0^T s^*_((6)) phi dif t = -integral_0^T s^*_((6)) phi dif t $

  まとめると

  $ integral_0^T dot.triple(s^*) dot.triple(phi) dif t = -integral_0^T s^*_((6))(t) phi(t) dif t $ <after-ibp>

  *(ii) 6階微分がゼロであること。* $s^*$が$J$の最小点であれば、任意の$phi$に対して#ref(<first-variation>)はゼロでなければならない(そうでなければ、$dif J/dif epsilon|_0 eq.not 0$ となる方向に$epsilon$を動かして$J$をさらに小さくできてしまう)。#ref(<after-ibp>)よりこれは

  $ integral_0^T s^*_((6))(t) phi(t) dif t = 0 $

  が任意の(斉次境界条件を満たす)$phi$について成り立つことを意味する。変分法の基本補題より $s^*_((6))(t) equiv 0$。すなわち$s^*$は高々5次の多項式である。

  *(iii) 一意性。* $cal(A)$の6つの境界条件は、5次多項式 $a_0+dots+a_5t^5$ の6個の係数を一意に定める線形連立方程式(定理2.2の証明で解いたもの)なので、高々5次の多項式で$cal(A)$に属するものは#ref(<quintic>)の$s^*$ただ1つである。

  *(iv) 大域最小性。* $s in cal(A)$を任意に取り、$phi := s-s^*$ とおくと、$phi$は斉次境界条件を満たす($cal(A)$の境界条件は線形で、$s,s^*$がともに満たすため差は斉次条件を満たす)。#ref(<after-ibp>)と(ii)で示した $s^*_((6))equiv 0$ より

  $ integral_0^T dot.triple(s^*) dot.triple(phi) dif t = -integral_0^T s^*_((6)) phi dif t = 0 $

  であるから

  $ J[s] = J[s^*+phi] = integral_0^T (dot.triple(s^*)+dot.triple(phi))^2 dif t = integral_0^T dot.triple(s^*)^2 dif t + 2integral_0^T dot.triple(s^*)dot.triple(phi) dif t + integral_0^T dot.triple(phi)^2 dif t \
    = J[s^*] + 0 + integral_0^T dot.triple(phi)^2 dif t >= J[s^*] $

  等号は $integral_0^T dot.triple(phi)^2 dif t=0$、すなわち $dot.triple(phi) equiv 0$(連続関数の2乗の積分が0なら被積分関数は恒等的に0)のときに限る。このとき$phi$は高々2次の多項式 $phi(t)=c_0+c_1t+c_2t^2$ だが、$phi(0)=c_0=0$、$dot(phi)(0)=c_1=0$、$dot.double(phi)(0)=2c_2=0$ より $phi equiv 0$、すなわち $s=s^*$。よって$s^*$は$cal(A)$上で$J$を最小化する唯一の関数である。
]

#remark[
  この定理が言っているのは、5次多項式は「加速度まで境界条件を課したらたまたま出てきた式」ではなく、躍度(加速度が変化する速さ。電車が急に加速・減速を切り替えると体に感じる「ガクッ」という衝撃の原因になる量)を最小にするという意味で数学的に特別な存在だ、ということである。Flash and Hoganはこれを、人間が腕を伸ばして物を取る動作が実際にほぼ最小躍度軌道になっていることの根拠として使っている。
]

#remark[
  `01_polynomial_trajectories.py` では、斉次境界条件を満たす具体的な摂動 $phi(t)=t^3(T-t)^3$(定義より$phi(0)=phi(T)=0$、$phi(t)=t^3(T-t)^3$の展開から$dot(phi),dot.double(phi)$も両端点で0になることが確かめられる)を用いて、$J[s^*+epsilon phi]$を$epsilon$の関数として数値的に評価し、$epsilon=0$(すなわち$s^*$自身)で最小になることを確認している。
]

= 台形速度プロファイル

車の運転を思い浮かべてほしい。発進のときはアクセルを踏んで一定の加速度で速度を上げ、道が空いていれば一定速度で巡航し、目的地の手前でブレーキを踏んで一定の減速度で止まる。この「加速・巡航・減速」の3段階が、関節の速度のグラフに台形の形を描くので台形速度プロファイル(trapezoidal motion profile, LSPB)と呼ばれる。多項式時間スケーリング(2, 3節)は式としては滑らかだが、関節の速度・加速度がどこまで許されるか(モーターが出せる最大の速さ・力)という現実の制約を直接には扱えない。台形速度プロファイルは、速度上限 $v$・加速度上限 $a$ をそのままパラメータとして指定でき、かつ(後で示すように)それらの上限のもとで*時間最適*であるという利点を持つ。設定・式番号は Lynch and Park, _Modern Robotics_, 2017, 9.2.2.2節(式(9.14)-(9.24))に対応するが、同節が演習問題(Exercise 8, 10, 11, 12)に委ねている証明は、以下ですべて自前で導出する。

#definition[
  加速度の一定値 $a>0$、速度の一定値 $v>0$、所要時間 $T>0$ に対して、切替時刻 $t_a := v/a$ を用いて
  $ s(t) := cases(
    display(1/2 a t^2) & quad 0 <= t <= t_a,
    v t - display(v^2/(2a)) & quad t_a < t <= T-t_a,
    display((2a v T-2v^2-a^2(t-T)^2)/(2a)) & quad quad T-t_a < t <= T,
  ) $ <trapezoid-def>
  で定める(コードの `trapezoidal_time_scaling` に対応)。
]

#theorem("台形プロファイルの導出")[
  #ref(<trapezoid-def>)の$s(t)$は、$s(0)=0,dot(s)(0)=0,dot(s)(T)=0$、および3段階
  $ dot.double(s)(t) &= a & quad & (0<=t<=t_a) \
    dot.double(s)(t) &= 0 & quad & (t_a<t<=T-t_a) \
    dot.double(s)(t) &= -a & quad & (T-t_a<t<=T) $ <trapezoid-accel>
  を満たす、$dot(s)$が連続な(区分的に2回微分可能な)唯一の関数である。
]

#proof[
  *第1段階($0<=t<=t_a$)。* $dot.double(s)(t)=a$ を$s(0)=0,dot(s)(0)=0$のもとで2回積分すると $dot(s)(t)=a t$, $s(t)=1/2 a t^2$。$t=t_a=v/a$ で $dot(s)(t_a)=a dot v/a = v$、$s(t_a)=1/2 a (v/a)^2 = v^2/(2a)$。

  *第2段階($t_a<t<=T-t_a$)。* $dot.double(s)(t)=0$ より $dot(s)(t)$は定数。$dot(s)$の連続性(第1段階の終値$v$)から $dot(s)(t)=v$。積分定数を第1段階の終値 $s(t_a)=v^2/(2a)$ に合わせると
  $ s(t) = s(t_a) + v(t-t_a) = v^2/(2a) + v t - v^2/a = v t - v^2/(2a) $
  であり、$t=T-t_a$ での値は $s(T-t_a) = v(T-t_a)-v^2/(2a) = v T - v^2/a - v^2/(2a) = v T - 3v^2/(2a)$。

  *第3段階($T-t_a<t<=T$)。* $dot.double(s)(t)=-a$ を積分すると $dot(s)(t) = C_1 - a t$。$dot(s)$の連続性より $dot(s)(T-t_a) = v$ なので $C_1 - a(T-t_a) = v$、すなわち $C_1 = v+a(T-t_a) = v+a T-v = a T$。よって $dot(s)(t) = a T-a t = a(T-t)$。特に $dot(s)(T)=0$(境界条件を満たす)。

  さらに積分すると $s(t) = a T t - 1/2 a t^2 + C_2$。$s$の連続性より $s(T-t_a)=v T-3v^2/(2a)$ に一致させると
  $ C_2 = v T - 3v^2/(2a) - a T(T-t_a) + 1/2 a (T-t_a)^2 $
  ここで $T-t_a=T-v/a$ を代入して整理すると(あるいは直接 $s(t)$ が $t=T$ で $s(T)$ となるように$C_2$を決めても同じ式が得られる。以下、整理した結果のみ示す)
  $ s(t) = a T t - 1/2 a t^2 + C_2 = (2a v T-2v^2-a^2(t-T)^2)/(2a) $
  であり、これは#ref(<trapezoid-def>)の第3段階の式に一致する(両辺を展開して $t^2$ の係数・$t$ の係数・定数項を比較すれば検証できる: 右辺 $=(2a v T-2v^2-a^2t^2+2a^2T t-a^2T^2)/(2a) = (2a v T-2v^2-a^2T^2)/(2a) + a T t - 1/2 a t^2$ となり、$t$の1次・2次の係数が左辺の式と一致し、定数項の一致は$t=T-t_a$での連続性から保証される)。

  各段階の式は#ref(<trapezoid-accel>)の加速度と、隣接段階との$s,dot(s)$の連続性のみから一意に定まるので、条件を満たす区分的に2回微分可能な関数は#ref(<trapezoid-def>)の$s$ただ1つである。
]

#remark[
  #ref(<trapezoid-accel>)より、$dot.double(s)$は$t=t_a$と$t=T-t_a$で$a$から$0$へ、$0$から$-a$へ不連続にジャンプする(`02_trapezoidal_velocity_profile.py`で$abs(dot.double(s)(t_a^-)-dot.double(s)(t_a^+))=a$を数値的に確認)。5次時間スケーリング(定理2.2)は加速度まで連続だが、台形プロファイルは速度までしか連続でない。これは後の定理4.6で示す時間最適性との引き換えである。
]

== パラメータ間の関係と実行可能性条件

台形プロファイルの3段階のうち、真ん中の一定速度で移動する区間をcoast区間(巡航区間)と呼ぶ。$v,a,T$ は独立に選べるパラメータではなく、$s(T)=1$という拘束が1本あるため、3つのうち独立に選べるのは2つだけである。次の補題がその拘束の具体形を与える。

#lemma("閉じた式")[
  #ref(<trapezoid-def>)の$s$が$s(T)=1$を満たすための必要十分条件は
  $ v T - v^2/a = 1 $ <closure>
  である。
]

#proof[
  $dot(s)(t)$のグラフ(Figure 9.5)は、高さ$v$、上底$T-2t_a$(coast区間の長さ)、下底$T$の台形である。$s(T)-s(0)=integral_0^T dot(s)(t) dif t$ は、この台形の面積(台形の面積公式 $=$ (上底+下底)$times$高さ$slash 2$)に等しいので
  $ s(T) = 1/2 (T + (T-2t_a)) dot v = (T-t_a) v = (T-v/a)v = v T - v^2/a $
  ($s(0)=0$を用いた)。ゆえに $s(T)=1 arrow.l.r.double v T-v^2/a=1$。
]

この式を使えば、$v,a,T$のうち好きな2つを指定したとき、残り1つがいくつになるかが具体的に解ける。

#corollary("3通りのパラメータ化")[
  #ref(<closure>)を用いて、$v,a,T$のうち2つを指定して残り1つを解くと
  + *$v,a$を指定($v^2/a<=1$のとき)*: $T = (a+v^2)/(v a)$
  + *$v,T$を指定($v T>1$のとき)*: $a = v^2/(v T-1)$
  + *$a,T$を指定($a T^2>=4$のとき)*: $v = 1/2 (a T - sqrt(a(a T^2-4)))$
  (コードの`min_time_given_v_a`, `accel_given_v_T`, `velocity_given_a_T`にそれぞれ対応)。
]

#proof[
  *(1)* #ref(<closure>)を$T$について解く: $v T=1+v^2/a$ より $T=1/v+v/a=(a+v^2)/(v a)$。

  *(2)* #ref(<closure>)を$a$について解く: $v^2/a=v T-1$ より($v T-1 eq.not 0$のとき)$a=v^2/(v T-1)$。$a>0$であるためには$v T-1>0$、すなわち$v T>1$が必要。

  *(3)* #ref(<closure>)の両辺に$a$を掛けて整理すると、$v$についての2次方程式
  $ v^2 - a T v + a = 0 $
  が得られる。解の公式より
  $ v = (a T plus.minus sqrt((a T)^2-4a))/2 = 1/2 (a T plus.minus sqrt(a(a T^2-4))) $
  実数解を持つためには判別式が非負、すなわち $a(a T^2-4)>=0$、$a>0$より $a T^2>=4$ が必要。2つの根のうち、3段階になるための条件(次の定理4.4で示す$v<=a T/2$)を満たすのは小さい方の根なので、$v=1/2(a T-sqrt(a(a T^2-4)))$を採用する。
]

ただし、これらの式が意味を持つのは実際に加速・巡航・減速の3段階になる場合だけである(巡航区間の長さが負にはなれない)。まず$v,a$を指定する場合に、それがいつ成り立つかを確認する。

#theorem("3段階になるための条件(v,aを指定する場合)")[
  $v,a$を指定したとき(#ref(<closure>)より$T=(a+v^2)/(v a)$)、coast区間が実際に存在する(3段階になる、すなわち$T>=2t_a$)ための必要十分条件は
  $ v^2/a <= 1 $
]

#proof[
  $t_a=v/a$なので、$T>=2t_a$は$T>=2v/a$と同値。$T=(a+v^2)/(v a)$を代入すると
  $ (a+v^2)/(v a) >= 2v/a $
  両辺に$a>0$を掛けて $(a+v^2)/v >= 2v$、さらに両辺に$v>0$を掛けて $a+v^2>=2v^2$、すなわち $a>=v^2$、これは$v^2/a<=1$と同値である(各変形は$a,v>0$のもとで両辺を正の量倍しているだけなので同値変形である)。
]

同様の条件を、今度は$v,T$を指定する場合について求める。

#theorem("3段階になるための条件(v,Tを指定する場合)")[
  $v,T$を指定したとき、$s(T)=1$を満たす$a>0$が存在し、かつ3段階になるための必要十分条件は
  $ 1 < v T <= 2 $
]

#proof[
  *$v T>1$が$a>0$の必要十分条件であること*は系4.2(2)の証明で示した。

  *$v T<=2$が3段階の条件であること*: 3段階になる条件は$T>=2t_a=2v/a$、すなわち$a>=2v/T$。系4.2(2)の$a=v^2/(v T-1)$を代入すると($v T>1$より$v T-1>0$なので、両辺に$T(v T-1)>0$を掛けても不等号の向きは変わらない)
  $ v^2/(v T-1) >= 2v/T
    &arrow.l.r.double v^2 T >= 2v(v T-1) \
    &arrow.l.r.double v^2T >= 2v^2T-2v \
    &arrow.l.r.double 0>=v^2T-2v=v(v T-2) $
  $v>0$なので、これは$v T-2<=0$、すなわち$v T<=2$と同値である。
]

最後に、$a,T$を指定する場合の実行可能性条件を確認する。

#theorem("実行可能性の条件(a,Tを指定する場合)")[
  $a,T$を指定したとき、$s(T)=1$を満たす(実数の)$v$が存在するための必要十分条件は
  $ a T^2 >= 4 $
  であり、等号$a T^2=4$は、coast区間の長さが0になる退化した場合(2段階のbang-bang運動、$v=a T/2$)に対応する。
]

#proof[
  系4.2(3)の証明で見たとおり、$v$は2次方程式$v^2-a T v+a=0$の解であり、実数解を持つ条件は判別式 $(a T)^2-4a>=0$、すなわち$a(a T^2-4)>=0$、$a>0$より$a T^2>=4$。

  等号$a T^2=4$のとき判別式は0なので重解 $v=a T/2$を持ち、このとき$t_a=v/a=T/2$となってcoast区間の長さ$T-2t_a=0$になる。これは、加速度$a$のみで前半$[0,T/2]$を加速し後半$[T/2,T]$を減速する2段階のbang-bang運動に一致する(`bang_bang_time_scaling`に対応)。実際、この運動の変位は$dot(s)(t)$のグラフ(頂点$a T/2$の三角形)の面積 $1/2 dot T dot (a T/2)=a T^2/4$ であり、$a T^2=4$のとき丁度$1$に一致する。
]

== 台形プロファイルの時間最適性

台形プロファイルの実用上の主な利点は、速度・加速度の上限を守るという制約のもとで、点から点への移動を*最短時間*で行えることである。これは Modern Robotics の Exercise 8 で証明抜きに述べられている事実だが、ここでは初等的な(Pontryaginの最大値原理などを使わない)議論で完全に証明する。

#definition[
  $v,a>0$を固定する。時刻$T'>0$上で定義された関数 $tilde(s):[0,T'] arrow [0,1]$ が、$tilde(s)(0)=0,dot(tilde(s))(0)=0,tilde(s)(T')=1,dot(tilde(s))(T')=0$ と、任意の$t in [0,T']$で $abs(dot.double(tilde(s))(t))<=a$ かつ $abs(dot(tilde(s))(t))<=v$ を満たすとき、$tilde(s)$を(この$v,a$のもとでの)*実行可能な時間スケーリング*と呼ぶ。
]

台形プロファイルだけでなく、同じ速度・加速度の上限を守る動かし方は他にも無数にある(例えば途中で速度を上げ下げする波打つような動き方も条件だけなら満たせる)。それらすべてと比べて、台形プロファイルが本当に最短時間かどうかを確認する。

#theorem("時間最適性")[
  $s^*$を#ref(<trapezoid-def>)の台形プロファイル(パラメータ$v,a$、所要時間$T^*=(a+v^2)/(v a)$、$v^2/a<=1$)とする。同じ$v,a$のもとで実行可能な任意の時間スケーリング $tilde(s):[0,T'] arrow [0,1]$ に対して $T'>=T^*$ が成り立つ。すなわち$s^*$は所要時間を最小化する。
]

#proof[
  *(i) 速度の上界。* $tilde(s)$を実行可能な時間スケーリングとする。$dot(tilde(s))(0)=0$と$abs(dot.double(tilde(s)))<=a$より、任意の$t in [0,T']$で
  $ dot(tilde(s))(t) = integral_0^t dot.double(tilde(s))(tau) dif tau <= integral_0^t a dif tau = a t $ <bound1>
  同様に$dot(tilde(s))(T')=0$より、任意の$t in[0,T']$で
  $ dot(tilde(s))(t) = dot(tilde(s))(T') - integral_t^(T') dot.double(tilde(s))(tau) dif tau = -integral_t^(T') dot.double(tilde(s))(tau) dif tau <= integral_t^(T') a dif tau = a(T'-t) $ <bound2>
  さらに実行可能性の定義より
  $ dot(tilde(s))(t) <= v $ <bound3>
  #ref(<bound1>)、#ref(<bound2>)、#ref(<bound3>)をあわせると、任意の$t in [0,T']$で
  $ dot(tilde(s))(t) <= min(a t, v, a(T'-t)) =: phi.alt_(T')(t) $ <envelope>

  *(ii) 変位の上界。* #ref(<envelope>)を$[0,T']$上で積分すると
  $ 1 = tilde(s)(T')-tilde(s)(0) = integral_0^(T') dot(tilde(s))(t) dif t <= integral_0^(T') phi.alt_(T')(t) dif t =: A(T') $ <area-bound>
  $A(T')$は、高さ$min(v,a T'/2)$の"台形または三角形"(関数$phi.alt_(T')$のグラフ)の面積である。$a T'^2/4<=v^2$のとき(頂点の高さ$a T'/2$が$v$に届かない場合)、$phi.alt_(T')$のグラフは底辺$T'$・高さ$a T'/2$の二等辺三角形で $A(T')=1/2 T' dot (a T'/2) = a T'^2/4$。$a T'^2/4>v^2$のとき、台形の面積公式(補題4.1の証明と同様)より $A(T') = v T'-v^2/a$。いずれの場合も$A$は$T'$の連続関数であり、
  $ (dif A)/(dif T') = cases(a T'/2 & (a T'^2<=4v^2 "のとき"), v & (a T'^2>4v^2 "のとき")) > 0 $
  なので$A$は$(0,infinity)$上で狭義単調増加である(2つの場合の境界$a T'^2=4v^2$、すなわち$T'=2v/a$で両側からの値は$A(2v/a)=a(2v/a)^2/4=v^2/a=v dot(2v/a)-v^2/a$となり連続に接続している)。

  *(iii) 結論。* 台形プロファイル自身は#ref(<envelope>)の各不等号を等号で満たすように構成されている(#ref(<trapezoid-accel>)の3段階はまさに$dot.double(s^*)=plus.minus a$または$0$、$dot(s^*)(t)=min(a t,v,a(T^*-t))$)ので、$A(T^*)=integral_0^(T^*)dot(s^*)(t)dif t=s^*(T^*)-s^*(0)=1$。

  一方、#ref(<area-bound>)より、任意の実行可能な$tilde(s)$に対して $A(T')>=1=A(T^*)$。(ii)で示した$A$の狭義単調増加性より、これは$T'>=T^*$を意味する。したがって$s^*$は実行可能な時間スケーリングの中で所要時間を最小化する。
]

#remark[
  `02_trapezoidal_velocity_profile.py`では、この定理の内容を、同じ$(v,a)$制約のもとで5次時間スケーリング(定理2.2)が(ピーク速度・ピーク加速度を$v,a$に合わせて時間軸をスケールしたとき)台形プロファイルより長い時間を要することを数値的に確認することで裏付けている(台形: $T^*approx 2.05$、5次: $T_q approx 2.40$、$v=0.8,a=1.0$の場合)。台形プロファイルは速度までしか連続でない(定理4.1のRemark)という代償と引き換えに、この時間最適性を得ている。
]

= S字(ジャーク制限)速度プロファイル

エレベーターに乗っているとき、動き出す瞬間に「ガクッ」と体が揺れることがあるが、高級なエレベーターほどそれが滑らかで気づきにくい。この違いを生むのが躍度(ジャーク、加速度が変化する速さ)である。台形プロファイルは加速度の値そのものは有限(上限$a$)に抑えているが、その加速度が$0$から$a$へ一瞬で切り替わる(定理4.1のRemark)。これはモーターへの負荷や機械の振動の原因になりうる。S字プロファイル(ジャーク制限プロファイル、速度のグラフがアルファベットのSに似た滑らかな曲線を描くことに由来)は、躍度そのものに上限$J$を課すことで、加速度の変化を滑らかにする。設定はLynch and Park, _Modern Robotics_, Figure 9.6の定性的な7段階の記述(定ジャーク→定加速度→定ジャーク→定速度→定ジャーク→定加速度→定ジャーク)に基づくが、同書はこの構成の閉じた式を与えていない。以下の構成は、BYU ME 537コース資料 "S-Curve Equations for a Trajectory Generator"(Ch. 5, 5.5節 "S-Curve with Linear Period")の構成を、これまでの時間スケーリング$s(t)$の記法に翻訳し、自前で導出・証明したものである。

#definition("7段階ジャーク")[
  加速度プラトー$a>0$、速度プラトー$v>0$、ジャーク上限$J>0$が
  $ a^2 <= v J $ <plateau-feasible>
  を満たすとする。$t_j := a/J$ とおく。$dot.triple(s)(t)$(躍度)を次の7段階の区分定数関数として定める(各段階の所要時間は下の定理5.1で決まる$t_j,t_"lin",T_c$を用いる):
  + $[0,t_j)$: $dot.triple(s)=J$ (concave, 加速の立ち上がり)
  + $[t_j,t_j+t_"lin")$: $dot.triple(s)=0$, $dot.double(s)=a$ (定加速度)
  + $[t_j+t_"lin",T_r)$: $dot.triple(s)=-J$ (convex, 加速度が0へ)
  + $[T_r,T_r+T_c)$: $dot.triple(s)=0$, $dot(s)=v$ (coast, 定速度)
  + $[T_r+T_c,T_r+T_c+t_j)$: $dot.triple(s)=-J$ (concave, 減速の立ち上がり)
  + 続く$t_"lin"$の間: $dot.triple(s)=0$, $dot.double(s)=-a$ (定加速度)
  + 最後の$t_j$の間: $dot.triple(s)=J$ (convex, 加速度が0へ)
  ここで$T_r:=2t_j+t_"lin"$(立ち上がりに要する時間)。初期条件は$s(0)=0,dot(s)(0)=0,dot.double(s)(0)=0$(静止状態からの開始)とする(コードの`scurve_time_scaling`に対応。実装は各段階を一定ジャークのもとでの逐次積分として構成している)。
]

まず、加速度が$0$から加速度プラトー$a$まで立ち上がるのにどれだけ時間がかかるかを求める。

#lemma("立ち上がり時間")[
  #ref(<plateau-feasible>)のもとで、加速度が$0$から$a$を経て再び$0$に戻る立ち上がり区間の所要時間は
  $ T_r = v/a + a/J $ <ramp-time>
  であり、このうち定加速度(段階2)の長さは $t_"lin" = v/a - a/J >= 0$ である。
]

#proof[
  段階1($[0,t_j]$、$dot.triple(s)=J$、$dot.double(s)(0)=0$)を積分すると $dot.double(s)(t)=J t$。$t=t_j=a/J$ で $dot.double(s)(t_j)=J(a/J)=a$ となり、これが加速度プラトー$a$に一致する($t_j$の定義と整合)。

  段階2は$dot.double(s)=a$(定数)、段階3は段階1と対称に$a$から$0$へ線形に減少する(同様の積分で確認できる)。したがって、立ち上がり区間全体で$dot.double(s)(t)$のグラフは、底辺$T_r$、上底$t_"lin"=T_r-2t_j$、高さ$a$の台形である(台形プロファイルの$dot(s)(t)$のグラフが台形だったのと同様の構造)。ゆえに、速度の変化量は台形の面積公式(補題4.1と同じ技法)より
  $ v = dot(s)(T_r)-dot(s)(0) = integral_0^(T_r) dot.double(s)(t) dif t
    &= 1/2 (T_r+t_"lin") dot a \
    &= 1/2(T_r + (T_r-2t_j)) dot a = (T_r - t_j) a $
  よって $T_r = v/a + t_j = v/a+a/J$、これは#ref(<ramp-time>)に等しい。また $t_"lin" = T_r-2t_j = v/a+a/J-2a/J=v/a-a/J$。$t_"lin">=0$は#ref(<plateau-feasible>) ($a^2<=v J$、すなわち$a/J<=v/a$)と同値である。
]

立ち上がり区間の加速のパターン(ジャーク→定加速度→ジャーク)は、前半と後半で綺麗な対称性を持つ。この対称性が、次に立ち上がり区間全体でどれだけ進むか(変位)を求める鍵になる。

#lemma("立ち上がり区間の点対称性")[
  立ち上がり区間$[0,T_r]$上で、$dot.double(s)(T_r-t) = dot.double(s)(t)$ かつ $dot(s)(T_r-t) = v - dot(s)(t)$ が任意の$t in [0,T_r]$で成り立つ。
]

#proof[
  *加速度の対称性。* 段階1($t in [0,t_j]$)で$dot.double(s)(t)=J t$。このとき$T_r-t in [T_r-t_j,T_r]$は段階3の範囲にあり、段階3は$dot.triple(s)=-J$のもとで$dot.double(s)(T_r-t_j)=a$から$dot.double(s)(T_r)=0$へ線形に減少するので、局所時間$sigma:=(T_r-t)-(T_r-t_j)=t_j-t$を使って $dot.double(s)(T_r-t) = a - J sigma = a-J(t_j-t) = a-J t_j+J t = a-a+J t = J t = dot.double(s)(t)$(ここで$J t_j=a$を用いた)。段階2($t in [t_j,T_r-t_j]$)では$dot.double(s)=a$定数であり、この区間は$T_r-t$についても同じ区間に写る($T_r-[t_j,T_r-t_j]=[t_j,T_r-t_j]$)ので自明に成り立つ。ゆえに$dot.double(s)(T_r-t)=dot.double(s)(t)$が$[0,T_r]$全体で成り立つ。

  *速度の対称性。* $psi(t) := dot(s)(T_r-t)+dot(s)(t)$ とおくと
  $ dot(psi)(t) = -dot.double(s)(T_r-t) + dot.double(s)(t) = 0 $
  (加速度の対称性による)。よって$psi$は定数であり、$psi(0)=dot(s)(T_r)+dot(s)(0)=v+0=v$。ゆえに$psi(t)=v$、すなわち$dot(s)(T_r-t)=v-dot(s)(t)$が任意の$t$で成り立つ。
]

この対称性を使うと、立ち上がり区間で実際にどれだけ進む(変位する)かが、積分を直接計算せずに求まる。

#corollary("立ち上がり区間の変位")[
  立ち上がり区間の変位は
  $ S_"ramp" := s(T_r)-s(0) = v^2/(2a) + (a v)/(2J) $ <s-ramp>
  である。
]

#proof[
  補題5.2の速度の対称性において、$u:=T_r-t$と置換すると
  $ integral_0^(T_r) dot(s)(t) dif t = integral_0^(T_r) dot(s)(T_r-u) dif u = integral_0^(T_r) (v-dot(s)(u)) dif u = v T_r - integral_0^(T_r) dot(s)(u) dif u $
  したがって $2 integral_0^(T_r) dot(s)(t) dif t = v T_r$、すなわち $S_"ramp" = integral_0^(T_r) dot(s)(t) dif t = v T_r\/2$。#ref(<ramp-time>)を代入すると
  $ S_"ramp" = v/2 (v/a+a/J) = v^2/(2a) + (a v)/(2J) $
  であり、これは#ref(<s-ramp>)に等しい。
]

立ち上がり・立ち下がりそれぞれの変位が分かったので、残るcoast区間(定速度で進む区間)の長さを埋め合わせれば、全体の所要時間が求まる。

#theorem("S字プロファイルの所要時間")[
  #ref(<plateau-feasible>)に加えて $2 S_"ramp" <= 1$ (coast区間が存在する)を仮定する。このとき、定義5.1の7段階プロファイルが$s(T)=1,dot(s)(T)=0,dot.double(s)(T)=0$を満たすための所要時間は
  $ T = a/J + v/a + 1/v $ <scurve-time>
  である。
]

#proof[
  減速区間(段階5-7)は、立ち上がり区間(段階1-3)を時間反転してジャークの符号を逆にしたものなので(定義5.1)、加速度・速度の変化量の大きさは立ち上がり区間と同一である。特に、減速区間の変位も$S_"ramp"$(系5.1)に等しい。

  coast区間(段階4)の変位は$v T_c$である($dot(s)=v$定数)。全区間の変位が1になるという条件から
  $ 1 = S_"ramp" + v T_c + S_"ramp" = 2S_"ramp" + v T_c $
  ゆえに $T_c = (1-2S_"ramp")/v$($2S_"ramp"<=1$より$T_c>=0$)。#ref(<s-ramp>)を代入すると
  $ T_c = 1/v - S_"ramp"/v dot 2 = 1/v - a/J - v/a $

  全所要時間は、立ち上がり・coast・立ち下がりの合計
  $ T = T_r + T_c + T_r = 2 T_r + T_c = 2(v/a+a/J) + (1/v-a/J-v/a) = a/J+v/a+1/v $
  であり、これは#ref(<scurve-time>)に等しい。境界条件$s(T)=1$は変位の合計が1になることから、$dot(s)(T)=0,dot.double(s)(T)=0$は減速区間の終端が立ち上がり区間の始端($dot(s)(0)=0,dot.double(s)(0)=0$)を時間反転したものであることから従う。
]

#remark[
  定義5.1の構成より、$dot.double(s)$は各段階の境界で連続である(段階の終値と次段階の初期値が一致するように積分定数を選んでいるため)。台形プロファイルの加速度は不連続にジャンプした(定理4.1のRemark)のに対し、S字プロファイルは加速度まで連続である。ただし躍度$dot.triple(s)$自身は$plus.minus J,0$の間を不連続にジャンプする。`03_scurve_velocity_profile.py`では、6つの内部切替点すべてで加速度の不連続量が$10^(-7)$未満(数値誤差の範囲)であることを確認し、対照的にジャークのジャンプ量がちょうど$J$であることを確認している。
]

加速度を滑らかにする代わりに、S字プロファイルは台形プロファイルより長い時間がかかるはずである。その差を具体的に計算する。

#corollary("台形プロファイルとの時間差")[
  同じ$v,a$のもとで、台形プロファイルの最小時間を$T^*=(a+v^2)/(v a)$(定理4.2.1)とすると
  $ T - T^* = a/J $
  すなわち、S字プロファイルは台形プロファイルよりちょうど$a/J$だけ長い時間を要する。特に$J arrow infinity$の極限で$T arrow T^*$となる。
]

#proof[
  $T^* = (a+v^2)/(v a) = a/(v a) + v^2/(v a) = 1/v + v/a$。#ref(<scurve-time>)との差を取ると
  $ T - T^* = (a/J+v/a+1/v) - (1/v+v/a) = a/J $
]

#remark[
  `03_scurve_velocity_profile.py`では、この系を$v=0.8,a=1.0,J=5.0$で数値的に確認している($T-T^*=0.2=a/J$)。さらに、$J$を大きくしていくと$T$が単調に$T^*$へ近づくことも確認している(#ref(<plateau-feasible>)を満たす$J>=a^2/v=1.25$の範囲で)。$a^2>v J$(#ref(<plateau-feasible>)が破れる場合)は、加速度がプラトー$a$に到達できず、実際のピーク加速度は$a_s=sqrt(v J)<a$に退化する(5段階プロファイル、BYU資料5.1節の"Ideal S-Curve"に対応。`degenerate_peak_accel`で検算)。
]

= 多項式経由点軌道

これまでの節(2-5)は、始点から終点まで一直線に動く場合だけを扱ってきた。しかし実際の作業では、途中の特定の位置・時刻を必ず通過させたいことがある——例えば棚の荷物を取ってから別の棚へ置く途中で、障害物を避けるために決まった高さを経由する、といった場合である。このような「途中で必ず通る点」を経由点(via point)と呼ぶ。これまでの節(2-5)は、幾何学的な経路 $theta(s)$($s in [0,1]$)をまず定め、次に時間スケーリング $s(t)$ を掛け合わせて $theta(t)=theta(s(t))$ を作るという2段階の構成だった。本節では、複数の経由点を指定された時刻に通過させたい場合の、より直接的な構成(経路と時間スケーリングを分離せず、関節の時間履歴 $beta(t)$ を直接補間する)を扱う。設定・式番号は Lynch and Park, _Modern Robotics_, 2017, 9.3節(式(9.25)-(9.29))に基づく。

#definition[
  $k$個の経由点が、時刻 $T_1=0<T_2<dots<T_k=T$ において、位置 $beta_1,dots,beta_k$ と速度 $dot(beta)_1,dots,dot(beta)_k$ で指定されているとする。区間 $j in {1,dots,k-1}$(所要時間 $Delta T_j := T_(j+1)-T_j$)における軌道を、その区間内で経過した時間 $Delta t in [0,Delta T_j]$ の3次多項式
  $ beta(T_j+Delta t) = a_0^j + a_1^j Delta t + a_2^j Delta t^2 + a_3^j Delta t^3 $
  で表し、境界条件
  $ beta(T_j)=beta_j, quad dot(beta)(T_j)=dot(beta)_j, quad beta(T_j+Delta T_j)=beta_(j+1), quad dot(beta)(T_j+Delta T_j)=dot(beta)_(j+1) $
  を課す(コードの`cubic_via_segment_coeffs`に対応)。
]

この4つの境界条件から、区間ごとの3次多項式の係数 $a_0,a_1,a_2,a_3$ が実際にどう決まるかを求める。

#theorem("経由点セグメントの係数")[
  上の境界条件を満たす係数は(添字$j$を省略して)
  $ a_0 = p_0, quad a_1 = v_0 $ <via-a01>
  $ a_2 = (3p_1-3p_0-2v_0 Delta T-v_1 Delta T)/(Delta T^2) $ <via-a2>
  $ a_3 = (2p_0+(v_0+v_1)Delta T-2p_1)/(Delta T^3) $ <via-a3>
  で与えられる。ここで $p_0:=beta_j,v_0:=dot(beta)_j,p_1:=beta_(j+1),v_1:=dot(beta)_(j+1)$、$Delta T:=Delta T_j$ とおいた。
]

#proof[
  $beta(0)=p_0$より$a_0=p_0$。$dot(beta)(Delta t)=a_1+2a_2 Delta t+3a_3 Delta t^2$、$dot(beta)(0)=v_0$より$a_1=v_0$。これで#ref(<via-a01>)が示された。

  残る2条件 $beta(Delta T)=p_1$、$dot(beta)(Delta T)=v_1$ に $a_0=p_0,a_1=v_0$ を代入すると
  $ a_2 Delta T^2+a_3 Delta T^3 = p_1-p_0-v_0 Delta T $ <via-c1>
  $ 2a_2 Delta T+3a_3 Delta T^2 = v_1-v_0 $ <via-c2>

  #ref(<via-c1>)より $a_2 = (p_1-p_0-v_0 Delta T-a_3 Delta T^3)/(Delta T^2)$。これを#ref(<via-c2>)に代入すると
  $ 2(p_1-p_0-v_0 Delta T)/(Delta T) - 2a_3 Delta T^2+3a_3 Delta T^2 = v_1-v_0 $
  $ a_3 Delta T^2 = v_1-v_0-2(p_1-p_0-v_0 Delta T)/(Delta T) = v_1+v_0-2(p_1-p_0)/(Delta T) $
  両辺に$Delta T$を掛けて整理すると
  $ a_3 = ((v_0+v_1)Delta T - 2(p_1-p_0))/(Delta T^3) = (2p_0+(v_0+v_1)Delta T-2p_1)/(Delta T^3) $
  これは#ref(<via-a3>)に等しい。

  #ref(<via-c2>)より $a_2 = (v_1-v_0)/(2Delta T) - 3/2 a_3 Delta T$。上で求めた$a_3$を代入し、通分して整理すると(分母を$Delta T^2$に揃える)
  $ a_2 &= (v_1-v_0)/(2Delta T) - 3/2 dot (2p_0+(v_0+v_1)Delta T-2p_1)/(Delta T^2) \
    &= ((v_1-v_0)Delta T slash 2 - 3p_0 - 3/2(v_0+v_1)Delta T+3p_1)/(Delta T^2) \
    &= (-v_1 Delta T - 2v_0 Delta T + 3p_1-3p_0)/(Delta T^2) = (3p_1-3p_0-2v_0 Delta T-v_1 Delta T)/(Delta T^2) $
  (3行目は、$v_1,v_0$の係数をそれぞれ集めると $(1/2-3/2)v_1 Delta T=-v_1 Delta T$、$(-1/2-3/2)v_0 Delta T=-2v_0 Delta T$ となることから従う)。これは#ref(<via-a2>)に等しい。
]

#remark[
  隣接するセグメント$j,j+1$は、共有する経由点$T_(j+1)$で境界条件により位置・速度が一致するように構成されているので、複数セグメントをつなげた軌道全体は位置・速度について連続である($C^1$級)。しかし、各セグメントには4つの境界条件(両端の位置・速度)しか課していないため、加速度 $dot.double(beta)(T_(j+1)^-) = 2a_2^j+6a_3^j Delta T_j$(セグメント$j$の終端)と $dot.double(beta)(T_(j+1)^+)=2a_2^(j+1)$(セグメント$j+1$の始端)が一致する保証はない。`05_via_point_trajectory.py`では、Modern Robotics Figure 9.7(a)/9.8と同じ4経由点の例($x$座標: 位置$(0,0,1,1)$・速度$(0,1,0,0)$、時刻$(0,1,2,3)$)で、内部経由点$t=1,2$において加速度がそれぞれ約$2.0$、$4.0$だけ不連続にジャンプすることを数値的に確認している(速度は連続、誤差$<10^(-4)$)。
]

この経由点補間は一般化になっているはずなので、経由点が始点・終点の2つだけの特別な場合には、2節の3次時間スケーリングと一致していなければならない。それを確認する。

#corollary("2経由点の場合は3次時間スケーリングに一致する")[
  $k=2$(始点・終点のみ)、両端の速度がともに0($v_0=v_1=0$)のとき、#ref(<via-a2>)・#ref(<via-a3>)の軌道は、定理2.1の3次時間スケーリングによる軌道 $beta(t)=p_0+s(t)(p_1-p_0)$(ただし$s$は#ref(<cubic>))に一致する。
]

#proof[
  $v_0=v_1=0$を#ref(<via-a2>)・#ref(<via-a3>)に代入すると(区間全体を$Delta T=T$として)
  $ a_2 = (3p_1-3p_0)/T^2 = 3(p_1-p_0)/T^2, quad a_3 = (2p_0-2p_1)/T^3 = -2(p_1-p_0)/T^3 $
  よって $beta(t) = p_0 + 3(p_1-p_0)/T^2 t^2 - 2(p_1-p_0)/T^3 t^3 = p_0 + (p_1-p_0)(3(t/T)^2-2(t/T)^3)$。#ref(<cubic>)より$3(t/T)^2-2(t/T)^3=s(t)$なので、これは$beta(t)=p_0+s(t)(p_1-p_0)$に等しい。
]

#remark[
  `05_via_point_trajectory.py`では、$p_0=0.3,p_1=1.7,T=2.0$の場合に、この一致を数値的に確認している(位置・速度・加速度すべての最大差 $<10^(-10)$)。すなわち、経由点補間は3次時間スケーリングの真の一般化になっている。5次多項式(定理2.2)を使えば、各経由点で加速度まで指定でき、Modern Robotics 9.3節はこの拡張にも触れているが、係数の導出が煩雑になるため本稿では割愛する。
]
