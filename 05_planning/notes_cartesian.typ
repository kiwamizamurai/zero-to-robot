#import "../common/theorems.typ": *

#show: setup-theorems

#set page(paper: "a4", margin: 2.5cm)
#set text(font: "New Computer Modern", size: 11pt, lang: "ja")
#set heading(numbering: "1.1")
#set math.equation(numbering: "(1)", supplement: "式")

#align(center)[
  #text(17pt, weight: "bold")[軌道計画 --- カルテシアン空間軌道: スクリューモーションと分離型経路]

  #text(11pt, fill: gray)[`04_cartesian_trajectory.py` の数式的裏付け]
]

#v(1cm)

記法・証明技法は `01_kinematics/notes.typ`(6-7節)を踏襲する。特に、Rodriguesの回転公式 $bm(R)(hat(bm(omega)),theta) = bm(I)+sin theta [hat(bm(omega))]_times+(1-cos theta)[hat(bm(omega))]_times^2$、skew行列の性質 $[hat(bm(omega))]_times^2=hat(bm(omega))hat(bm(omega))^T-bm(I)$、$[hat(bm(omega))]_times^3=-[hat(bm(omega))]_times$(同notesの補題6.1・定理7.1)は証明済みとしてそのまま用いる。カルテシアン空間の経路の構成は Lynch and Park, _Modern Robotics_, 2017, 9.1節の式(9.6)(スクリュー経路)・式(9.7)(9.8)(分離型経路)に基づくが、これらの式に現れる$"SO"(3)$の対数写像・$"SE"(3)$の指数写像の閉じた式は、同書では別の章(第3章)で扱われているため、以下で自前で導出・証明する。

= 準備: $"SO"(3)$の対数写像と$"SE"(3)$の指数写像

2節でカルテシアン空間の経路を構成するには、始点から終点への「差分」を回転量・並進量として取り出す道具が要る。回転だけならRodriguesの回転公式の逆演算(対数写像)で済むが、並進も同時に扱うスクリュー経路(コードの`screw_trajectory`)には、剛体運動全体を指数写像で表す$bm(G)(theta)$行列が必要になる。Lynch and Parkの9.1節はこれらの写像が既知であることを前提にしているが、同書の該当章(第3章)を読まずに済むよう、Rodriguesの回転公式(すでに`01_kinematics/notes.typ`で証明済み)だけから自前で導出する。

Rodriguesの回転公式は、単位軸$hat(bm(omega))$と角度$theta$から回転行列$bm(R)$を作る(いわば「指数写像」)。逆に、$bm(R)$から$(hat(bm(omega)),theta)$を復元する「対数写像」を導出する。

#lemma("SO(3)の対数写像")[
  $bm(R) in "SO"(3)$、$bm(R) eq.not bm(I)$ とする。$bm(R)=bm(R)(hat(bm(omega)),theta)$($theta in (0,pi]$)を満たす$(hat(bm(omega)),theta)$は
  $ theta = arccos((op("tr")(bm(R))-1)/2) $ <log-theta>
  $ hat(bm(omega)) = 1/(2 sin theta) vec(R_(32)-R_(23), R_(13)-R_(31), R_(21)-R_(12)) quad (theta eq.not 0,pi "のとき") $ <log-axis>
  で与えられる(コードの`so3_log`に対応)。
]

#proof[
  *(i) $theta$の式。* Rodriguesの公式の両辺のトレースを取る。$op("tr")(bm(I))=3$、交代行列のトレースは0なので $op("tr")([hat(bm(omega))]_times)=0$。$op("tr")([hat(bm(omega))]_times^2) = op("tr")(hat(bm(omega))hat(bm(omega))^T-bm(I)) = op("tr")(hat(bm(omega))hat(bm(omega))^T)-3 = hat(bm(omega))^T hat(bm(omega)) - 3 = 1-3=-2$(単位ベクトルなので$hat(bm(omega))^T hat(bm(omega))=1$、かつ$op("tr")(bm(a)bm(a)^T)=bm(a)^T bm(a)$)。よって
  $ op("tr")(bm(R)) = 3 + sin theta dot 0 + (1-cos theta)(-2) = 3-2+2cos theta = 1+2cos theta $
  ゆえに $cos theta = (op("tr")(bm(R))-1)/2$。$theta in (0,pi]$の範囲では$arccos$の値域とちょうど一致するので、#ref(<log-theta>)が唯一の解を与える。

  *(ii) 軸の式。* Rodriguesの公式の転置を取ると、$bm(I)^T=bm(I)$、$[hat(bm(omega))]_times^T=-[hat(bm(omega))]_times$(交代行列)、$([hat(bm(omega))]_times^2)^T = (hat(bm(omega))hat(bm(omega))^T-bm(I))^T = hat(bm(omega))hat(bm(omega))^T-bm(I)=[hat(bm(omega))]_times^2$(対称)より
  $ bm(R)^T = bm(I) - sin theta [hat(bm(omega))]_times + (1-cos theta)[hat(bm(omega))]_times^2 $
  Rodriguesの公式から辺々引くと、対称な項($bm(I)$と$[hat(bm(omega))]_times^2$の項)が消えて
  $ bm(R)-bm(R)^T = 2 sin theta [hat(bm(omega))]_times $ <r-minus-rt>
  $theta eq.not 0,pi$ のとき$sin theta eq.not 0$なので、$[hat(bm(omega))]_times = (bm(R)-bm(R)^T)/(2 sin theta)$。交代行列の定義 $[hat(bm(omega))]_times = mat(0,-omega_3,omega_2;omega_3,0,-omega_1;-omega_2,omega_1,0)$ と#ref(<r-minus-rt>)の$(3,2),(1,3),(2,1)$成分を比較すると、$omega_1=(R_(32)-R_(23))\/(2sin theta)$等が得られ、これは#ref(<log-axis>)に等しい。
]

次に、回転だけでなく並進も含む剛体運動(スクリューモーション、ねじを締めるときのように回転と並進が同時に起きる動き)の指数写像を導出する。角速度$hat(bm(omega))$(単位ベクトル)と線速度$bm(v) in RR^3$の組をツイスト(twist、その瞬間の「回転+並進」の速度をまとめて表したもの)と呼び、$cal(S)=(hat(bm(omega)),bm(v))$と書く。これを4x4行列
$ [cal(S)] := mat([hat(bm(omega))]_times, bm(v); bm(0)^T, 0) $
として表す。

#theorem("SE(3)の指数写像")[
  $ exp([cal(S)]theta) = mat(bm(R)(hat(bm(omega)),theta), bm(G)(theta)bm(v); bm(0)^T, 1) $ <se3-exp>
  ここで
  $ bm(G)(theta) := theta bm(I) + (1-cos theta)[hat(bm(omega))]_times + (theta-sin theta)[hat(bm(omega))]_times^2 $ <g-matrix>
  である(コードの`se3_exp`, `G_matrix`に対応)。
]

#proof[
  $[cal(S)]$はブロック上三角(右下ブロックが0)なので、$k>=1$に対して
  $ [cal(S)]^k = mat([hat(bm(omega))]_times^k, [hat(bm(omega))]_times^(k-1)bm(v); bm(0)^T,0) $
  (帰納法: $k=1$は定義より成立。$[cal(S)]^(k+1)=[cal(S)]^k [cal(S)]$を計算すると、左上ブロックは$[hat(bm(omega))]_times^k [hat(bm(omega))]_times=[hat(bm(omega))]_times^(k+1)$、右上ブロックは$[hat(bm(omega))]_times^k bm(v) + [hat(bm(omega))]_times^(k-1)bm(v) dot 0 = [hat(bm(omega))]_times^k bm(v)$となり主張が従う)。

  行列指数の定義($sum_(k=0)^infinity X^k\/k!$、成分ごとに絶対収束するため項別に和を取れる)より
  $ exp([cal(S)]theta) = mat(bm(I)+sum_(k=1)^infinity [hat(bm(omega))]_times^k theta^k\/k!, sum_(k=1)^infinity [hat(bm(omega))]_times^(k-1)bm(v)theta^k\/k!; bm(0)^T,1) $

  左上ブロックは、定義よりそのまま$exp([hat(bm(omega))]_times theta)=bm(R)(hat(bm(omega)),theta)$である(Rodriguesの回転公式は、この行列指数の閉じた式に他ならない)。

  右上ブロックを計算する。$j:=k-1$とおくと
  $ sum_(k=1)^infinity [hat(bm(omega))]_times^(k-1) theta^k\/k! = sum_(j=0)^infinity [hat(bm(omega))]_times^j theta^(j+1)\/(j+1)! =: bm(G)(theta) $
  $[hat(bm(omega))]_times^3=-[hat(bm(omega))]_times$(01_kinematics/notes.typ 補題6.1(c))より、$j$を3で割った余りで分類すると、$[hat(bm(omega))]_times^0=bm(I)$の項は$j=0$のみ、$[hat(bm(omega))]_times$の項は$j=1,3,5,dots$($[hat(bm(omega))]_times^(2m+1)=(-1)^m [hat(bm(omega))]_times$)、$[hat(bm(omega))]_times^2$の項は$j=2,4,6,dots$($[hat(bm(omega))]_times^(2m)=(-1)^(m-1) [hat(bm(omega))]_times^2$、$m>=1$)に現れる。それぞれの係数を集めると
  $ bm(I)"の係数": quad theta $
  $ [hat(bm(omega))]_times"の係数": quad sum_(m=0)^infinity (-1)^m theta^(2m+2)/(2m+2)! = 1-cos theta $
  $ [hat(bm(omega))]_times^2"の係数": quad sum_(m=0)^infinity (-1)^m theta^(2m+3)/(2m+3)! = theta - sin theta $
  (それぞれ$cos theta = sum_m (-1)^m theta^(2m)\/(2m)!$、$sin theta=sum_m (-1)^m theta^(2m+1)\/(2m+1)!$のTaylor級数から直接確認できる)。ゆえに
  $ bm(G)(theta) = theta bm(I) + (1-cos theta)[hat(bm(omega))]_times + (theta-sin theta)[hat(bm(omega))]_times^2 $
  であり、これは#ref(<g-matrix>)に等しい。右上ブロックは$bm(G)(theta)bm(v)$であり、#ref(<se3-exp>)が従う。
]

後で対数写像(指数写像の逆演算)を定義する際、$bm(G)(theta)bm(v)=bm(p)$を$bm(v)$について解く場面が出てくる。そのために、$bm(G)(theta)$が常に逆行列を持つ(方程式が一意に解ける)ことを確認しておく。

#theorem("G(θ)の可逆性")[
  $theta in (0,2pi)$ のとき、#ref(<g-matrix>)の$bm(G)(theta)$は正則である。
]

#proof[
  $bm(v)=bm(v)_parallel+bm(v)_perp$(01_kinematics/notes.typ 定理7.1のRemarkと同じ、$hat(bm(omega))$方向への平行成分と直交成分の分解)とする。$[hat(bm(omega))]_times bm(v)_parallel=bm(0)$、$[hat(bm(omega))]_times^2 bm(v)_perp = -bm(v)_perp$(BAC-CAB則、$hat(bm(omega))dot bm(v)_perp=0$より)なので
  $ bm(G)(theta)bm(v) = theta bm(v)_parallel + theta bm(v)_perp + (1-cos theta)[hat(bm(omega))]_times bm(v)_perp - (theta-sin theta)bm(v)_perp \
    = theta bm(v)_parallel + sin theta med bm(v)_perp + (1-cos theta)[hat(bm(omega))]_times bm(v)_perp $ <g-decomposed>
  したがって、$hat(bm(omega))$方向(1次元)への$bm(G)(theta)$の制限は $theta$倍(スカラー)、$hat(bm(omega))^perp$平面(2次元)への制限は、その平面内で$[hat(bm(omega))]_times$が90度回転に相当する線形写像であることを使うと、複素数の言葉で $sin theta + (1-cos theta)i$ 倍(回転+スケーリング)に対応する。

  $theta in (0,2pi)$ のとき $theta eq.not 0$ なので平行成分は可逆。直交成分についても
  $ |sin theta + (1-cos theta)i|^2 = sin^2 theta + (1-cos theta)^2 = 2-2cos theta $
  であり、$theta in (0,2pi)$ では $cos theta < 1$ なので $2-2cos theta > 0$、すなわちこの複素数は0でなく可逆である。平行・直交どちらの成分についても可逆なので、$bm(G)(theta)$は($RR^3$を$hat(bm(omega))$方向の1次元部分空間とその直交補空間$hat(bm(omega))^perp$の直和に分解した上で)正則である。
]

#remark[
  $"SE"(3)$の対数写像は、#ref(<se3-exp>)の逆演算として、$bm(X)=(bm(R),bm(p))$から、まず補題1.1で$(hat(bm(omega)),theta)$を復元し、次に $bm(G)(theta)bm(v)=bm(p)$ を(定理1.3の可逆性により一意に解が存在する線形方程式として)解いて$bm(v)$を求めることで得られる(コードの`se3_log`。閉じた式$bm(G)(theta)^(-1)$を陽に書き下す代わりに、数値的に線形方程式を解く)。
]

= カルテシアン空間の経路

始点・終点の姿勢(位置+向き)だけが与えられたとき、その間をどう補間するかには自由度がある。ねじを回しながら壁にねじ込む動きのように、回転と並進を一体の「ひねり」として同時に進める方法(スクリュー経路、`screw_trajectory`)と、ドローンが向きを変えながらまっすぐ前進するときのように、位置は位置で直線的に、向きは向きで別個に補間する方法(分離型経路、`decoupled_trajectory`)の2通りが考えられる。ここではこの2通りの構成を与え、どちらを選ぶべきかの判断材料として、位置成分がらせんを描くか直線を描くかという決定的な違いを証明する。

始点 $bm(X)_"start"$、終点 $bm(X)_"end" in "SE"(3)$ に対して、パラメータ $s in [0,1]$ の経路 $bm(X)(s)$ を構成する2つの方法を考える。

#definition("スクリュー経路")[
  $(hat(bm(omega)),bm(v),theta) := $ ($"SE"(3)$の対数写像で得られる)$bm(X)_"start"^(-1) bm(X)_"end"$ のツイストとする。
  $ bm(X)_"screw"(s) := bm(X)_"start" thin exp([cal(S)]theta s) $ <screw-path>
  で定める(コードの`screw_trajectory`に対応)。
]

対照的に、位置と姿勢を最初から別々に補間するのが分離型経路である。

#definition("分離型経路")[
  $bm(X)_"start"=(bm(R)_"start",bm(p)_"start")$、$bm(X)_"end"=(bm(R)_"end",bm(p)_"end")$ とし、$(hat(bm(omega)),theta):=$ ($"SO"(3)$の対数写像で得られる)$bm(R)_"start"^T bm(R)_"end"$ の軸角度とする。
  $ bm(p)_"dec"(s) := bm(p)_"start" + s(bm(p)_"end"-bm(p)_"start") $ <dec-pos>
  $ bm(R)_"dec"(s) := bm(R)_"start" thin bm(R)(hat(bm(omega)),theta s) $ <dec-rot>
  で定める(コードの`decoupled_trajectory`に対応)。
]

どちらの構成も、まず経路として満たすべき最低限の要件——始点から出発し終点に到着すること——を満たしているかを確認する。

#theorem("境界条件")[
  スクリュー経路・分離型経路のいずれも $bm(X)(0)=bm(X)_"start"$、$bm(X)(1)=bm(X)_"end"$ を満たす。
]

#proof[
  *スクリュー経路。* $s=0$: $exp([cal(S)]dot 0)=exp(bm(0))=bm(I)$(4x4)なので$bm(X)_"screw"(0)=bm(X)_"start"$。$s=1$: $exp([cal(S)]theta)=bm(X)_"start"^(-1)bm(X)_"end"$(定義より、これは$"SE"(3)$の指数写像と対数写像が互いに逆演算であることによる)なので $bm(X)_"screw"(1)=bm(X)_"start" (bm(X)_"start"^(-1)bm(X)_"end")=bm(X)_"end"$。

  *分離型経路。* $s=0$: #ref(<dec-pos>)より$bm(p)_"dec"(0)=bm(p)_"start"$。#ref(<dec-rot>)より$bm(R)_"dec"(0)=bm(R)_"start"bm(R)(hat(bm(omega)),0)=bm(R)_"start"bm(I)=bm(R)_"start"$。$s=1$: $bm(p)_"dec"(1)=bm(p)_"end"$は明らか。$bm(R)(hat(bm(omega)),theta)=bm(R)_"start"^T bm(R)_"end"$(補題1.1と定義より)なので $bm(R)_"dec"(1)=bm(R)_"start" (bm(R)_"start"^T bm(R)_"end")=bm(R)_"end"$。
]

次に、両者の決定的な違い(Figure 9.2, Lynch and Park)を証明する。

#theorem("スクリュー経路はらせんを描く")[
  スクリュー経路の位置成分を、$bm(X)_"start"$の枠内で見た相対位置 $tilde(bm(p))(s) := bm(G)(theta s)bm(v)$ とする(すなわち、定理1.2より$exp([cal(S)]theta s)$の回転部分は$bm(R)(hat(bm(omega)),theta s)$、並進部分は$tilde(bm(p))(s)$である)。$bm(v)=bm(v)_parallel+bm(v)_perp$($hat(bm(omega))$方向の平行成分・直交成分の分解)とすると
  $ tilde(bm(p))(s) = theta s med bm(v)_parallel + sin(theta s) med bm(v)_perp + (1-cos(theta s))[hat(bm(omega))]_times bm(v)_perp $ <helix>
  が成り立つ。特に、$bm(v)_perp=bm(0)$(すなわち$bm(v)parallel hat(bm(omega))$)でない限り、$tilde(bm(p))(s)$は$s$の直線ではなく、$hat(bm(omega))$方向への一様な並進と、$hat(bm(omega))^perp$平面内での半径$|bm(v)_perp|$の等速円運動を合成した、らせん(helix)を描く。
]

#proof[
  #ref(<g-decomposed>) ($theta$を$theta s$に置き換えたもの)より、$tilde(bm(p))(s) = bm(G)(theta s)bm(v) = theta s med bm(v)_parallel + sin(theta s)bm(v)_perp + (1-cos(theta s))[hat(bm(omega))]_times bm(v)_perp$、これは#ref(<helix>)に等しい。

  $bm(v)_perp eq.not bm(0)$ とする。$hat(bm(omega))^perp$平面内で$bm(e)_2:=hat(bm(v)_perp)$(単位ベクトル)、$bm(e)_3:=hat(bm(omega))times bm(e)_2$ を取ると(`01_kinematics/notes.typ` 定理7.1(iii)の証明と同じ構成)、$[hat(bm(omega))]_times bm(v)_perp = |bm(v)_perp|(hat(bm(omega))times bm(e)_2) = |bm(v)_perp|bm(e)_3$ なので、直交成分は
  $ sin(theta s)|bm(v)_perp|bm(e)_2 + (1-cos(theta s))|bm(v)_perp|bm(e)_3 $
  であり、これは中心 $|bm(v)_perp|bm(e)_3$、半径$|bm(v)_perp|$の円周上の点を、パラメータ$theta s$(円弧の角度)で表したものである($(sin u)^2+(1-cos u -1)^2=sin^2 u+cos^2 u=1$より確認できる、$u:=theta s$)。この円運動の成分は$s$について線形ではない(非自明な三角関数)ので、$bm(v)_perp eq.not bm(0)$のとき$tilde(bm(p))(s)$は直線ではない。
]

#remark[
  対照的に、分離型経路の位置#ref(<dec-pos>)は定義よりそのまま$s$の1次式であり、常に直線である。`04_cartesian_trajectory.py`では、スクリューモーション経路(位置の直線からのずれ最大0.0666)と分離型経路(ずれ厳密に0)を、同じ始点・終点で数値的に比較して確認している。
]

位置についてはこの通り違いが明確だが、姿勢(向き)の変化のし方についても、分離型経路には次のようなきれいな性質がある。

#theorem("分離型経路の回転は一定の角速度を持つ")[
  分離型経路#ref(<dec-rot>)の姿勢は、body frame(物体固定座標系)から見て、$s$によらず一定方向・一定角速度で回転する。すなわち
  $ bm(R)_"dec"(s)^T (dif bm(R)_"dec")/(dif s)(s) = theta [hat(bm(omega))]_times $
  は$s$に依存しない定数行列である。
]

#proof[
  #ref(<dec-rot>)を$s$で微分すると(Rodriguesの公式の行列指数としての表示 $bm(R)(hat(bm(omega)),theta s)=exp([hat(bm(omega))]_times theta s)$ を使うと、行列指数の微分公式 $dif/(dif s) exp(bm(A)s) = bm(A)exp(bm(A)s)=exp(bm(A)s)bm(A)$ より)
  $ (dif bm(R)_"dec")/(dif s)(s) = bm(R)_"start" exp([hat(bm(omega))]_times theta s) [hat(bm(omega))]_times theta = bm(R)_"dec"(s) thin theta[hat(bm(omega))]_times $
  両辺に左から$bm(R)_"dec"(s)^T$を掛けると($bm(R)_"dec"(s)^T bm(R)_"dec"(s)=bm(I)$より)
  $ bm(R)_"dec"(s)^T (dif bm(R)_"dec")/(dif s)(s) = theta[hat(bm(omega))]_times $
  であり、右辺は$s$に依存しない。
]

= 時間スケーリングとの組み合わせ、複数経由点への拡張

2節までで作った経路$bm(X)(s)$は幾何学的な形を決めるだけで、いつどの速さで通過するかは決めていない。ここではこれまでの時間スケーリング$s(t)$をそのまま流用してカルテシアン軌道$bm(X)(t)$を作り(コードの`cartesian_trajectory`)、複数の経由点をつなぐ拡張(`waypoint_trajectory`)、さらに数値IK(`damped_least_squares_ik`)で関節角に変換するところまで実装する。

`01_polynomial_trajectories.py`等で構成した時間スケーリング $s(t)$ を使い、$bm(X)(t):=bm(X)_"path"(s(t))$ とすれば、本フォルダのこれまでの成果(境界条件・滑らかさ)がそのままカルテシアン空間の軌道に引き継がれる(コードの`cartesian_trajectory`)。

#remark[
  複数の経由点(waypoint) $bm(X)_0,bm(X)_1,dots,bm(X)_n$ が与えられたとき、最も単純な拡張は、各区間$[bm(X)_(i-1),bm(X)_i]$を独立に(定理2.1の境界条件により)静止状態から静止状態への時間スケーリングで結ぶことである(コードの`waypoint_trajectory`、stop-and-go方式)。この方式は各経由点で位置・姿勢は連続だが速度は0になり滑らかに通過しない。連続な速度で経由点を通過する軌道(Modern Robotics 9.3節、"Polynomial Via Point Trajectories")の構成は今後の課題とする。
]

#remark[
  カルテシアン空間の軌道 $bm(X)(t)$ の位置成分 $bm(p)(t)$ が定まれば、`01_kinematics/06_differential_kinematics_3d.py`の幾何ヤコビアンを用いた減衰最小二乗法IK(コードの`damped_least_squares_ik`)により、各時刻での関節角 $bm(theta)(t)$ を数値的に求めることができる。`04_cartesian_trajectory.py`では、3自由度アームでカルテシアン直線軌道を追従させ、$"FK"("IK"(bm(p)(t)))approx bm(p)(t)$ (誤差 $<10^(-6)$)であることを確認している。これは`01_kinematics`(FK/IK)と`05_planning`(軌道生成)の内容を統合する具体例である。
]
