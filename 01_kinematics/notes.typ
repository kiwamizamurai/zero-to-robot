#import "../common/theorems.typ": *

#show: setup-theorems

#set page(paper: "a4", margin: 2.5cm)
#set text(font: "New Computer Modern", size: 11pt, lang: "ja")
#set heading(numbering: "1.1")
#set math.equation(numbering: "(1)", supplement: "式")

#let atan2 = math.op("atan2")

#align(center)[
  #text(17pt, weight: "bold")[運動学 --- 2関節平面アームの逆運動学と3次元への一般化]

  #text(11pt, fill: gray)[`01_forward_kinematics.py` 〜 `06_differential_kinematics_3d.py` の数式的裏付け]
]

#v(1cm)

記法: ベクトル・行列は太字 $bm(q), bm(f), bm(J)$ で、スカラーは非太字 $q_1, l_1$ で表す。本稿全体を通じて、実数の四則演算や基本的な微分則(積の微分則・合成関数の微分則)は既知として用いる。3次元を扱う6節以降ではさらに、次の事実も断りなく用いる: 実3次正方行列に対する行列式の性質 $det(bm(A)bm(B))=det(bm(A))det(bm(B))$、スカラー三重積 $(bm(a)times bm(b))dot bm(c) = det[bm(a) med bm(b) med bm(c)]$(列に$bm(a),bm(b),bm(c)$を並べた行列の行列式)、ベクトル三重積 $bm(a)times(bm(b)times bm(c))=bm(b)(bm(a)dot bm(c))-bm(c)(bm(a)dot bm(b))$(BAC-CAB則)。これらは標準的な線形代数・ベクトル解析の教科書に載っている事実であり、ここでは証明しない。

= 準備: 線形代数の基本事実と平面回転行列の性質

これから2-4節で、2関節平面アームの順運動学・ヤコビ行列を作り、逆運動学を勾配降下法と公式解の両方で解く。そのどちらの証明でも繰り返し使う道具が2つある。転置や線形性といった行列の基本演算規則と、関節の回転を表す平面回転行列 $bm(R)(theta)$ の性質(加法性・直交性・微分・偏角の保存)である。まずこの2つを先に用意しておく。

#lemma("基本事実")[
  (a) (転置と積) 任意の $bm(A) in RR^(m times n)$, $bm(B) in RR^(n times p)$ に対して $(bm(A)bm(B))^T = bm(B)^T bm(A)^T$。

  (b) (線形性) 任意の $bm(A) in RR^(m times n)$, $bm(u), bm(v) in RR^n$, $c in RR$ に対して $bm(A)(bm(u)+bm(v)) = bm(A)bm(u) + bm(A)bm(v)$, $bm(A)(c bm(u)) = c bm(A) bm(u)$。
]

#proof[
  (a) 両辺の $(i,j)$ 成分を比較する。$((bm(A)bm(B))^T)_(i,j) = (bm(A)bm(B))_(j,i) = sum_k A_(j,k) B_(k,i)$。一方 $(bm(B)^T bm(A)^T)_(i,j) = sum_k (bm(B)^T)_(i,k) (bm(A)^T)_(k,j) = sum_k B_(k,i) A_(j,k)$。これらは和の順序を除いて同じ項の和なので一致する。

  (b) 行列とベクトルの積の定義 $(bm(A) bm(u))_i = sum_j A_(i,j) u_j$ と、実数の分配法則 $sum_j A_(i,j)(u_j+v_j) = sum_j A_(i,j) u_j + sum_j A_(i,j) v_j$ から直ちに従う。スカラー倍についても同様。
]

この2つの基本事実を手に、いよいよ本題である「回転」の表現に入る。関節を角度 $theta$ だけ回すという操作を、行列の掛け算として書き表したい。

#definition[
  $theta in RR$ に対し、平面回転行列 $bm(R)(theta) in RR^(2 times 2)$ を
  $ bm(R)(theta) := mat(cos theta, -sin theta; sin theta, cos theta) $
  で定める。また $bm(Omega) := mat(0,-1;1,0)$ とおく。
]

この行列 $bm(R)(theta)$ が実際に「回転」と呼ぶにふさわしい性質(角度を足し合わせられる、長さを変えない、逆回転は転置で求まる)を持つことを、次に確認する。

#lemma("回転行列の性質")[
  任意の $alpha, beta, theta in RR$ に対して次が成り立つ。
  + (加法性) $ bm(R)(alpha) bm(R)(beta) = bm(R)(alpha+beta) $
  + (直交性) $ bm(R)(theta)^T bm(R)(theta) = bm(I), quad bm(R)(theta)^T = bm(R)(-theta) $
  + (微分) $ dif / (dif theta) bm(R)(theta) = bm(Omega) bm(R)(theta) $
  + (偏角の保存) $bm(v) in RR^2$, $bm(v) eq.not bm(0)$ に対して
    $ atan2((bm(R)(theta) bm(v))_2, thin (bm(R)(theta) bm(v))_1) = atan2(v_2, v_1) + theta $
]

#proof[
  *(1)* 行列の積の定義から直接計算すると

  $ bm(R)(alpha) bm(R)(beta) &= mat(cos alpha,-sin alpha;sin alpha,cos alpha) mat(cos beta,-sin beta;sin beta,cos beta) \
    &= mat(
      cos alpha cos beta - sin alpha sin beta, -cos alpha sin beta - sin alpha cos beta;
      sin alpha cos beta + cos alpha sin beta, -sin alpha sin beta + cos alpha cos beta
    ) $

  加法定理 $cos(alpha+beta) = cos alpha cos beta - sin alpha sin beta$、$sin(alpha+beta) = sin alpha cos beta + cos alpha sin beta$ を用いてこの行列の4成分を書き換えると

  $ = mat(cos(alpha+beta), -sin(alpha+beta); sin(alpha+beta), cos(alpha+beta)) = bm(R)(alpha+beta) $

  *(2)* (1) で $beta = -theta$ とおくと $bm(R)(theta) bm(R)(-theta) = bm(R)(0) = bm(I)$。一方 $bm(R)(-theta) = mat(cos theta, sin theta; -sin theta, cos theta) = bm(R)(theta)^T$ は定義から直接読み取れるので、$bm(R)(theta)^T bm(R)(theta) = bm(R)(-theta)bm(R)(theta) = bm(R)(0) = bm(I)$ も (1) より従う。

  *(3)* 左辺を成分ごとに微分すると
  $ dif/(dif theta) bm(R)(theta) = mat(-sin theta, -cos theta; cos theta, -sin theta) $
  右辺は
  $ bm(Omega) bm(R)(theta) = mat(0,-1;1,0) mat(cos theta,-sin theta;sin theta,cos theta) = mat(-sin theta, -cos theta; cos theta, -sin theta) $
  両者は成分ごとに一致する。

  *(4)* $r := sqrt(v_1^2+v_2^2) > 0$、$phi := atan2(v_2,v_1)$ とおくと、$atan2$ の定義より $bm(v) = r vec(cos phi, sin phi)$ である。ゆえに
  $ bm(R)(theta) bm(v) &= r mat(cos theta,-sin theta;sin theta,cos theta) vec(cos phi,sin phi) = r vec(cos theta cos phi - sin theta sin phi, sin theta cos phi + cos theta sin phi) \
                       &= r vec(cos(phi+theta), sin(phi+theta)) $
  (最後の等号は加法定理)。$r>0$ なので、この式は $bm(R)(theta)bm(v)$ の長さが $r$、偏角が $phi+theta$ であることを意味し、$atan2((bm(R)(theta)bm(v))_2,(bm(R)(theta)bm(v))_1) = phi + theta = atan2(v_2,v_1)+theta$ を得る。
]

#remark[
  4つの性質はどれも、時計の針を思い浮かべると当たり前に見える。(1)は「$alpha$度動かしてから$beta$度動かす」のと「$alpha+beta$度を一度に動かす」が同じ結果になるということ、(4)はその言い換えで、針の向きは元の向きからちょうど$theta$だけずれる(長さは変わらない)ということに過ぎない。(2)は回転が長さを変えない操作である($bm(R)(theta)^T bm(R)(theta)=bm(I)$)ことの表明で、逆回転が転置を取るだけで求まるという実用上便利な事実も込みになっている。(3)は少し毛色が違い、$theta$を時刻$t$の関数$theta(t)$と思うと連鎖律から $dif/(dif t) bm(R)(theta(t)) = dot(theta)(t) bm(Omega) bm(R)(theta(t))$ となる——$bm(Omega)$は「角速度1で回転させる」ことに対応する行列であり、これは6節以降で3次元の角速度を扱うskew行列 $[bm(v)]_times$ の2次元版になっている。
]

= 順運動学とヤコビ行列

2関節の腕を伸ばして目の前の物を掴むところを想像してほしい。肩と肘をそれぞれどれだけ曲げれば手が目的の位置に来るか——これが順運動学 $bm(f)(bm(q))$ である。さらに、手先をほんの少しだけ動かしたいとき、肩と肘をどちらにどれだけ動かせばよいかを知りたければ、$bm(f)$ の「傾き」であるヤコビ行列 $bm(J)(bm(q))=partial bm(f)\/partial bm(q)$ が要る。高校数学の微分は1変数の関数の傾きを求める道具だったが、ヤコビ行列はそれを「2変数入力・2変数出力」に拡張しただけのものである。3節の勾配降下法はこのヤコビ行列を使って手先を目標に近づける向きを求め、4節の公式解は $bm(f)$ の式そのものを逆算する。この節ではまずこの2つを2関節平面アームについて用意する。

#definition[
  長さ $l_1, l_2 > 0$ の2リンク平面アームを考える。関節角ベクトルを $bm(q) = vec(q_1,q_2)$ とし、
  $ bm(p)_0 := vec(0,0), quad bm(p)_1 (bm(q)) := bm(R)(q_1) vec(l_1,0), quad bm(d)(bm(q)) := bm(R)(q_1+q_2) vec(l_2,0) $
  とおく（$bm(p)_0$: 第1関節の位置、$bm(p)_1(bm(q))$: 第2関節(肘)の位置、$bm(d)(bm(q))$: 第2リンクの変位ベクトル）。順運動学 $bm(f): RR^2 arrow RR^2$ を
  $ bm(f)(bm(q)) := bm(p)_1(bm(q)) + bm(d)(bm(q)) $
  で定義する（コードの `fk(q)` に対応。$a=q_1, b=q_1+q_2$ として展開すれば $bm(f)(bm(q)) = vec(l_1 cos a + l_2 cos b, l_1 sin a + l_2 sin b)$ となり、コードの実装と一致する）。
]

$bm(f)$ の式ができたので、次はその「傾き」であるヤコビ行列を実際に求める。

#theorem[
  $bm(f)$ のヤコビ行列 $bm(J)(bm(q)) := (partial bm(f))/(partial bm(q)) in RR^(2 times 2)$ は
  $ bm(J)(bm(q)) = mat(
    bm(Omega)(bm(f)(bm(q)) - bm(p)_0) & bm(Omega)(bm(f)(bm(q)) - bm(p)_1(bm(q)))
  ) $
  （すなわち第 $i$ 列は「$90 degree$ 回転行列 $bm(Omega)$ を、手先位置から第 $i$ 関節位置を引いたベクトルに作用させたもの」）で与えられる。
]

#proof[
  $bm(p)_1(bm(q))$ は $q_1$ のみに、$bm(d)(bm(q))$ は $q_1+q_2$ を通して $q_1,q_2$ の両方に依存する。回転行列の性質(3)と合成関数の微分則、および $(partial (q_1+q_2))/(partial q_1) = (partial(q_1+q_2))/(partial q_2) = 1$ に注意すると

  $ (partial bm(f))/(partial q_1) &= (partial bm(p)_1)/(partial q_1) + (partial bm(d))/(partial q_1) = bm(Omega) bm(R)(q_1) vec(l_1,0) + bm(Omega) bm(R)(q_1+q_2) vec(l_2,0) dot 1 \
                                   &= bm(Omega)(bm(p)_1(bm(q)) + bm(d)(bm(q))) = bm(Omega) bm(f)(bm(q)) $

  $ (partial bm(f))/(partial q_2) = (partial bm(p)_1)/(partial q_2) + (partial bm(d))/(partial q_2) = bm(0) + bm(Omega) bm(R)(q_1+q_2) vec(l_2,0) dot 1 = bm(Omega) bm(d)(bm(q)) $

  $bm(p)_0 = bm(0)$ かつ $bm(f)(bm(q)) - bm(p)_1(bm(q)) = bm(d)(bm(q))$ であるから、以上2式は主張の行列の第1列・第2列にそれぞれ一致する。
]

#remark[
  この結果が言っているのは、「ヤコビ行列の各列は、対応する関節だけを動かしたときに手先が動く向きと速さ」だということだ。第1列は肩だけを動かしたとき、第2列は肘だけを動かしたときの手先の速度ベクトルに他ならない。この形は、回転関節 $i$ の列が $bm(z)_(i-1) times (bm(o)_n - bm(o)_(i-1))$ で与えられるというマニピュレータ・ヤコビ行列の一般公式（$bm(z)_(i-1)$: 関節軸、$bm(o)_(i-1)$: 関節位置、$bm(o)_n$: 手先位置）の平面版である（Craig, _Introduction to Robotics: Mechanics and Control_, 3rd ed., 2003; Spong, Vidyasagar and Hutchinson, _Robot Modeling and Control_, 2005。Stanford大学 CS223A などの講義でも採用される標準的な導出法）。平面上では関節軸は紙面に垂直なので、外積 $bm(z)_(i-1) times (dot)$ は $90 degree$ 回転 $bm(Omega)$ の作用に帰着する。この一般公式は、6節以降で$n$関節・3次元の場合に完全に証明する。
]

この一般公式の$2 times 2$版が、具体的にどんな成分になるかを書き下しておく。

#corollary[
  $a := q_1, b := q_1+q_2$ とおくと
  $ bm(J)(bm(q)) = mat(-l_1 sin a - l_2 sin b, -l_2 sin b; l_1 cos a + l_2 cos b, l_2 cos b) $
  であり、これはコードの `jac(q)` と一致する。
]

#proof[
  定理の $bm(Omega) bm(f)(bm(q))$ を成分で計算すると
  $ bm(Omega) bm(f)(bm(q)) = mat(0,-1;1,0) vec(l_1 cos a + l_2 cos b, l_1 sin a + l_2 sin b) = vec(-l_1 sin a - l_2 sin b, l_1 cos a + l_2 cos b) $
  同様に
  $ bm(Omega) bm(d)(bm(q)) = mat(0,-1;1,0) vec(l_2 cos b, l_2 sin b) = vec(-l_2 sin b, l_2 cos b) $
  この2つを列として並べると主張の行列を得る。
]

= 勾配降下法の勾配

逆運動学を「目標位置との誤差を最小化する」問題として解くアプローチが勾配降下法である(コードの`gradient_descent_ik`)。これは機械学習の線形回帰を勾配降下法で解くのと発想は同じで、線形回帰では損失 $1/2 ||bm(y)-bm(X)bm(w)||^2$ の勾配が「計画行列 $bm(X)$ の転置 $times$ 誤差」という形になったのに対し、ここでは $bm(f)$ が非線形なので計画行列の代わりにヤコビ行列が現れる、という違いだけである。そのために必要なのは、誤差の2乗和である損失関数 $cal(L)(bm(q))$ の勾配 $nabla cal(L)(bm(q))$ であり、これを2節のヤコビ行列から求めるのが本節の目標である。

#lemma[
  可微分な写像 $bm(e): RR^n arrow RR^m$ のヤコビ行列を $bm(J)_e (bm(q)) := (partial bm(e))/(partial bm(q)) in RR^(m times n)$ とする。このとき、スカラー値関数
  $ F(bm(q)) := 1/2 bm(e)(bm(q))^T bm(e)(bm(q)) $
  の勾配は
  $ nabla F(bm(q)) = bm(J)_e (bm(q))^T bm(e)(bm(q)) $
  で与えられる。
]

#proof[
  積の微分則より $dif (bm(e)^T bm(e)) = (dif bm(e))^T bm(e) + bm(e)^T (dif bm(e))$。右辺第1項はスカラー $(dif bm(e))^T bm(e)$ であり、スカラーは自分自身の転置に等しいことと基本事実(a)より

  $ (dif bm(e))^T bm(e) = ((dif bm(e))^T bm(e))^T = bm(e)^T ((dif bm(e))^T)^T = bm(e)^T dif bm(e) $

  であるから $dif(bm(e)^T bm(e)) = 2 bm(e)^T dif bm(e)$、したがって

  $ dif F = 1/2 dif(bm(e)^T bm(e)) = bm(e)^T dif bm(e) $

  全微分を取ると $dif bm(e) = bm(J)_e dif bm(q)$ なので、上式に代入して

  $ dif F = bm(e)^T bm(J)_e dif bm(q) $

  一方、勾配の定義より任意の $dif bm(q)$ に対して $dif F = nabla F(bm(q))^T dif bm(q)$ が成り立つ。両辺は任意の $dif bm(q) in RR^n$ について等しくなければならないので、係数を比較すると

  $ nabla F(bm(q))^T = bm(e)(bm(q))^T bm(J)_e (bm(q)) $

  両辺を転置し、基本事実(a)を右辺に適用すると $nabla F(bm(q)) = (bm(e)(bm(q))^T bm(J)_e (bm(q)))^T = bm(J)_e (bm(q))^T bm(e)(bm(q))$ を得る。
]

この補題を、2節で作った順運動学の誤差 $bm(e)(bm(q)) := bm(x)^* - bm(f)(bm(q))$ に当てはめれば、目当ての勾配が求まる。

#theorem[
  誤差 $bm(e)(bm(q)) := bm(x)^* - bm(f)(bm(q))$（$bm(x)^*$ は目標位置で $bm(q)$ に依存しない定数）、損失関数
  $ cal(L)(bm(q)) := 1/2 bm(e)(bm(q))^T bm(e)(bm(q)) $
  とすると
  $ nabla cal(L)(bm(q)) = -bm(J)(bm(q))^T bm(e)(bm(q)) $
  である（コードの `gradient_descent_ik` に対応）。
]

#proof[
  $bm(x)^*$ は定数なので $bm(J)_e (bm(q)) = (partial bm(e))/(partial bm(q)) = -(partial bm(f))/(partial bm(q)) = -bm(J)(bm(q))$。上の補題を $bm(e)$ に適用すると

  $ nabla cal(L)(bm(q)) = bm(J)_e (bm(q))^T bm(e)(bm(q)) = (-bm(J)(bm(q)))^T bm(e)(bm(q)) = -bm(J)(bm(q))^T bm(e)(bm(q)) $
]

#remark[
  この結果が言っているのは、損失を減らす向きは「目標との誤差ベクトル $bm(e)$ を、ヤコビ行列の転置で関節空間に引き戻したもの」だということだ。ニューラルネットの誤差逆伝播が出力側の誤差を$bm(W)^T$で入力側に伝えるのと構造的に同じで、ここでは$bm(W)^T$の役にヤコビ行列の転置$bm(J)^T$が立っている。コード中の更新式 `q -= lr * (-jac(q).T @ e)` は $bm(q) arrow.l bm(q) - eta (-bm(J)(bm(q))^T bm(e)) = bm(q) - eta nabla cal(L)(bm(q))$ であり、勾配降下法の更新式そのものである。
]

= 複素数を使わない公式解の導出

勾配降下法(3節)は反復計算で近似解に近づけるが、2関節平面アームの逆運動学には閉じた式(公式解)が存在する。ドアノブに手を伸ばすところを思い浮かべてほしい——手の位置を固定したまま、肘を上に張り出す姿勢と下げた姿勢の2通りが可能なはずだ(肘上・肘下)。この2つの姿勢は「肩から目標までの距離」と「2つのリンクの長さ」だけで決まる三角形の頂点角度であり、中学校で習う余弦定理そのもので求まる。ここでは、複素数表示に頼らず、2節の順運動学 $bm(f)(bm(q))$ と1節の回転行列の性質だけから、この余弦定理に相当する式を導く(コードの`closed_form_ik`に対応)。

#theorem[
  目標位置 $bm(x)^* = vec(x_1^*, x_2^*) eq.not bm(0)$ に対し $r^2 := (x_1^*)^2 + (x_2^*)^2$ とおく。$|l_1-l_2| <= r <= l_1+l_2$（到達可能条件）のとき、
  $ q_2 = plus.minus arccos( (r^2 - l_1^2 - l_2^2) / (2 l_1 l_2) ) $
  $ q_1 = atan2(x_2^*, x_1^*) - atan2(w_2 (q_2), w_1 (q_2)), quad bm(w)(q_2) := vec(l_1 + l_2 cos q_2, l_2 sin q_2) $
  は $bm(f)(bm(q)) = bm(x)^*$ を満たす（コードの `closed_form_ik` に対応。複号 $plus.minus$ が肘上/肘下の2つの解に対応する）。
]

#proof[
  まず $bm(f)(bm(q))$ を書き換える。回転行列の性質(1)より $bm(R)(q_1+q_2) = bm(R)(q_1) bm(R)(q_2)$ であるから

  $ bm(f)(bm(q)) = bm(R)(q_1) vec(l_1,0) + bm(R)(q_1) bm(R)(q_2) vec(l_2,0) $

  さらに基本事実(b)（線形性）より $bm(R)(q_1) vec(l_1,0) + bm(R)(q_1) [bm(R)(q_2) vec(l_2,0)] = bm(R)(q_1) [vec(l_1,0) + bm(R)(q_2) vec(l_2,0)]$ なので

  $ bm(f)(bm(q)) = bm(R)(q_1) [ vec(l_1,0) + bm(R)(q_2) vec(l_2,0) ] = bm(R)(q_1) bm(w)(q_2) $ <fq-eq>

  *(i) $q_2$ の導出。* $bm(f)(bm(q)) = bm(x)^*$ と @fq-eq、および基本事実(a)（転置と積）と回転行列の性質(2)（$bm(R)(q_1)^T bm(R)(q_1) = bm(I)$）を順に用いると

  $ r^2 &= (bm(x)^*)^T bm(x)^* = (bm(f)(bm(q)))^T bm(f)(bm(q)) = (bm(R)(q_1) bm(w)(q_2))^T (bm(R)(q_1) bm(w)(q_2)) \
       &= bm(w)(q_2)^T bm(R)(q_1)^T bm(R)(q_1) bm(w)(q_2) = bm(w)(q_2)^T bm(w)(q_2) $ <r-eq>

  ここで

  $ bm(w)(q_2)^T bm(w)(q_2) &= (l_1 + l_2 cos q_2)^2 + (l_2 sin q_2)^2 \
                            &= l_1^2 + 2 l_1 l_2 cos q_2 + l_2^2 (cos^2 q_2 + sin^2 q_2) \
                            &= l_1^2 + l_2^2 + 2 l_1 l_2 cos q_2 $

  であるから $r^2 = l_1^2 + l_2^2 + 2 l_1 l_2 cos q_2$ を得る。これを $cos q_2$ について解けば

  $ cos q_2 = (r^2 - l_1^2 - l_2^2) / (2 l_1 l_2) $ <cosq2-eq>

  この値が $arccos$ の定義域 $[-1,1]$ に入ることは、仮定した到達可能条件と同値である。実際 $2 l_1 l_2 > 0$ なので

  $ -1 <= (r^2-l_1^2-l_2^2)/(2 l_1 l_2) <= 1
    &arrow.l.r.double.long -2 l_1 l_2 <= r^2-l_1^2-l_2^2 <= 2 l_1 l_2 \
    &arrow.l.r.double.long (l_1-l_2)^2 <= r^2 <= (l_1+l_2)^2 \
    &arrow.l.r.double.long |l_1-l_2| <= r <= l_1+l_2 $

  （最後の同値は $r, l_1, l_2 > 0$ に対して $t |-> t^2$ が単調増加であることによる）。よって到達可能条件のもとで @cosq2-eq の右辺は $arccos$ の定義域に入り、$q_2$ は well-defined である。$arccos$ の値域は $[0,pi]$ であり、かつ $cos$ は偶関数（$cos(-q_2) = cos q_2$）なので、$q_2 = arccos(dot)$ と $q_2 = -arccos(dot)$ はいずれも @cosq2-eq を満たす2つの解を与える。

  また、$bm(x)^* eq.not bm(0)$ より $r^2 > 0$ であり、@r-eq で示した $bm(w)(q_2)^T bm(w)(q_2) = r^2$ と合わせると $bm(w)(q_2) eq.not bm(0)$ が従う。

  *(ii) $q_1$ の導出。* (i) で得た $q_2$ を固定すると $bm(w)(q_2)$ は既知の非零ベクトルになる。@fq-eq と $bm(f)(bm(q)) = bm(x)^*$、および回転行列の性質(4)（$bm(v) = bm(w)(q_2) eq.not bm(0)$, $theta = q_1$ として適用可能）より

  $ atan2(x_2^*, x_1^*) &= atan2((bm(R)(q_1) bm(w)(q_2))_2, (bm(R)(q_1) bm(w)(q_2))_1) \
                        &= atan2(w_2(q_2), w_1(q_2)) + q_1 $

  これを $q_1$ について解けば主張の式を得る。
]

#remark[
  この結果が言っているのは、逆運動学の計算は実質2ステップで終わるということだ。まず目標までの距離 $r$ だけから余弦定理で肘の角度 $q_2$ を決め(肘上・肘下の2通り)、次に「目標の方向」から「肘の位置を考慮したリンクの向き」のズレを引き算して肩の角度 $q_1$ を決める。ヤコビ行列も勾配降下法も要らず、三角関数の逆関数だけで済むのは、2関節平面アームという特に単純な構造だからこそである。
]

= 補足: 損失関数の非凸性

4節で解が2つ(肘上・肘下)あることを見た。3節の勾配降下法がどちらの解に落ち着くかは初期姿勢次第であり、この節ではそれが偶然ではなく損失関数 $cal(L)$ の形そのものに起因することを確認する。

#remark[
  上の定理が示すように、到達可能な目標に対して $q_2 = +arccos(c)$ と $q_2 = -arccos(c)$（$c := (r^2-l_1^2-l_2^2)/(2l_1l_2)$）の2つの解（肘上・肘下）が存在し、いずれも $bm(f)(bm(q)) = bm(x)^*$、すなわち $cal(L)(bm(q)) = 0$ を達成する。凸関数の最小点は（存在すれば）唯一つなので、$cal(L)$ は $bm(q)$ について凸ではない。勾配降下法がどちらの解に収束するかは初期値 $bm(q)^((0))$ に依存する。
]

= 3次元への一般化: 交代行列(skew行列)の性質

ここからは3次元回転(`03_rotations_3d.py`)・同次変換(`04_homogeneous_transforms.py`)・DHパラメータ(`05_dh_parameters.py`)・一般化ヤコビ行列(`06_differential_kinematics_3d.py`)を扱う。最終目標は、任意軸まわりの回転公式(7節)と、$n$関節・3次元へ一般化されたヤコビ行列(9節)を証明することである。両方の証明で繰り返し使うのが、外積 $bm(v) times bm(u)$ を行列の掛け算として書き直す道具、skew行列(交代行列)である。たとえば角速度 $bm(omega)$ で回転する剛体上の点 $bm(r)$ の速度は $bm(v) = bm(omega) times bm(r)$ で与えられるが、これを $bm(v) = [bm(omega)]_times bm(r)$ と行列×ベクトルの形に書けると、1節で扱ったような行列の性質(転置・積・微分)がそのまま使えるようになり、扱いが格段に楽になる。先にここで性質をまとめておく。記法・証明技法は上の2D部分(1-5節)の補題(基本事実)を踏襲する(全微分を取って係数比較する、転置と積 $(bm(A)bm(B))^T=bm(B)^T bm(A)^T$、線形性 $bm(A)(bm(u)+bm(v))=bm(A)bm(u)+bm(A)bm(v)$ は既に証明済みとしてそのまま使う)。

#definition[
  $bm(v)=(v_1,v_2,v_3)^T in RR^3$ に対して、交代行列(skew行列) $[bm(v)]_times in RR^(3 times 3)$ を、任意の $bm(u)$ に対して $[bm(v)]_times bm(u) = bm(v) times bm(u)$ を満たすものとして定める。具体的には
  $ [bm(v)]_times := mat(0,-v_3,v_2; v_3,0,-v_1; -v_2,v_1,0) $
]

この定義から、skew行列は転置・回転との交換・べき乗について次の性質を持つ。

#lemma("skew行列の性質")[
  (a) $[bm(v)]_times^T = -[bm(v)]_times$(交代性)

  (b) 任意の $bm(R) in "SO"(3)$ と $bm(v) in RR^3$ に対して $bm(R) [bm(v)]_times bm(R)^T = [bm(R)bm(v)]_times$

  (c) 単位ベクトル $hat(bm(n))$ に対して $[hat(bm(n))]_times^2 = hat(bm(n)) hat(bm(n))^T - bm(I)$, $quad [hat(bm(n))]_times^3 = -[hat(bm(n))]_times$
]

#proof[
  *(a)* 定義の行列を直接見れば、転置を取ると符号が反転した同じ行列になることが分かる。

  *(b)* まず、任意の $bm(a),bm(b) in RR^3$ と可逆行列 $bm(M) in RR^(3 times 3)$ に対して
  $ (bm(M)bm(a)) times (bm(M)bm(b)) = det(bm(M)) med bm(M)^(-T) (bm(a)times bm(b)) $ <cross-under-M>
  が成り立つことを示す。任意の $bm(c) in RR^3$ に対して、スカラー三重積と $det(bm(A)bm(B))=det(bm(A))det(bm(B))$ より
  $ [(bm(M)bm(a))times(bm(M)bm(b))] dot bm(c) = det[bm(M)bm(a) med bm(M)bm(b) med bm(c)] = det[bm(M)bm(a) med bm(M)bm(b) med bm(M)(bm(M)^(-1)bm(c))] \
    = det(bm(M)) det[bm(a) med bm(b) med bm(M)^(-1)bm(c)] = det(bm(M)) (bm(a)times bm(b)) dot (bm(M)^(-1)bm(c)) = det(bm(M)) (bm(M)^(-T)(bm(a)times bm(b))) dot bm(c) $
  （最後の等号は転置の定義 $bm(x) dot (bm(N) bm(y))=(bm(N)^T bm(x)) dot bm(y)$ を $bm(N)=bm(M)^(-1)$ に適用したもの）。これが任意の $bm(c)$ について成り立つので#ref(<cross-under-M>)を得る。

  $bm(M)=bm(R) in "SO"(3)$ とすると、$det(bm(R))=1$、$bm(R)^(-T)=(bm(R)^(-1))^T=(bm(R)^T)^T=bm(R)$(直交性 $bm(R)^(-1)=bm(R)^T$ より)なので、#ref(<cross-under-M>) は $(bm(R)bm(a))times(bm(R)bm(b)) = bm(R)(bm(a)times bm(b))$ となる。

  さて、任意の $bm(u)$ に対して $bm(w):=bm(R)^T bm(u)$ とおくと($bm(R)$は直交行列なので $bm(u)=bm(R)bm(w)$)、
  $ bm(R)[bm(v)]_times bm(R)^T bm(u) = bm(R)[bm(v)]_times bm(w) = bm(R)(bm(v)times bm(w)) = (bm(R)bm(v))times(bm(R)bm(w)) = (bm(R)bm(v)) times bm(u) = [bm(R)bm(v)]_times bm(u) $
  任意の $bm(u)$ でこれが成り立つので $bm(R)[bm(v)]_times bm(R)^T = [bm(R)bm(v)]_times$ を得る。

  *(c)* BAC-CAB則より $hat(bm(n))times(hat(bm(n))times bm(v)) = hat(bm(n))(hat(bm(n))dot bm(v)) - bm(v)(hat(bm(n))dot hat(bm(n))) = hat(bm(n))hat(bm(n))^T bm(v) - bm(v)$(単位ベクトルなので $hat(bm(n))dot hat(bm(n))=1$)。これが任意の $bm(v)$ で成り立つので $[hat(bm(n))]_times^2 = hat(bm(n))hat(bm(n))^T - bm(I)$。

  ゆえに $[hat(bm(n))]_times^3 = [hat(bm(n))]_times(hat(bm(n))hat(bm(n))^T-bm(I)) = ([hat(bm(n))]_times hat(bm(n)))hat(bm(n))^T - [hat(bm(n))]_times = (hat(bm(n))times hat(bm(n)))hat(bm(n))^T - [hat(bm(n))]_times = bm(0) - [hat(bm(n))]_times = -[hat(bm(n))]_times$($hat(bm(n))times hat(bm(n))=bm(0)$ より)。
]

#remark[
  (a)は外積の反交換性 $bm(a)times bm(b) = -bm(b)times bm(a)$ の行列版に過ぎない。(b)は「回転させてから外積を取る」のと「外積を取ってから回転させる」が入れ替えられるということで、次節でRodriguesの回転公式が実際に「回転」になっていることを確かめる際、鍵となる。(c)は skew行列の高次べきを低次の行列(せいぜい2次)に落とせるという恒等式で、これがあるおかげで、回転行列を表す無限級数 $e^([hat(bm(n))]_times theta) = sum_(k=0)^infinity ([hat(bm(n))]_times theta)^k \/ k!$ が有限個の項(Rodriguesの回転公式)に畳み込める。
]

= Rodriguesの回転公式

3次元の回転行列は9個の成分を持つが、実際の自由度は3つ(回転軸2つ+角度1つ)しかない。Rodriguesの回転公式は、回転を「軸 $hat(bm(n))$」と「角度 $theta$」という3つのパラメータから直接組み立てる式であり、`03_rotations_3d.py`の`rotation_from_axis_angle`に対応する。ここでは、この式が実際に軸 $hat(bm(n))$ まわりの回転を表し、回転行列の集合 $"SO"(3)$ に属することを証明する。

#definition[
  単位ベクトル $hat(bm(n))$ と角度 $theta$ に対して
  $ bm(R)(hat(bm(n)),theta) := bm(I) + sin theta [hat(bm(n))]_times + (1-cos theta) [hat(bm(n))]_times^2 $
  と定める(コードの `rotation_from_axis_angle` に対応)。
]

この式が本当に「軸 $hat(bm(n))$ まわりの回転」になっているかは、定義しただけではまだ分からない。次の定理で、その幾何学的な意味と $"SO"(3)$ への所属を示す。

#theorem[
  任意の $bm(v) in RR^3$ に対して
  $ bm(R)(hat(bm(n)),theta) bm(v) = cos theta med bm(v) + (1-cos theta)(hat(bm(n))dot bm(v)) hat(bm(n)) + sin theta (hat(bm(n)) times bm(v)) $ <rodrigues-geo>
  が成り立つ。すなわち $bm(R)(hat(bm(n)),theta)$ は、軸 $hat(bm(n))$ のまわりに右手則で角度 $theta$ だけ回転させる操作を表す。さらに $bm(R)(hat(bm(n)),theta) in "SO"(3)$ である。
]

#proof[
  *(i) #ref(<rodrigues-geo>) の導出。* 定義とskew行列の性質(c)より
  $ bm(R)(hat(bm(n)),theta)bm(v) = bm(v) + sin theta (hat(bm(n))times bm(v)) + (1-cos theta)[(hat(bm(n))dot bm(v))hat(bm(n)) - bm(v)] \
    = bm(v)(1-(1-cos theta)) + (1-cos theta)(hat(bm(n))dot bm(v))hat(bm(n)) + sin theta(hat(bm(n))times bm(v)) $
  これは#ref(<rodrigues-geo>)に一致する。

  *(ii) 幾何学的な意味。* $bm(v)=(hat(bm(n))dot bm(v))hat(bm(n)) + bm(v)_perp$(平行成分と直交成分の分解、$bm(v)_perp := bm(v)-(hat(bm(n))dot bm(v))hat(bm(n))$)とおくと、$hat(bm(n))times bm(v) = hat(bm(n))times bm(v)_perp$(平行成分との外積は$bm(0)$)なので、#ref(<rodrigues-geo>) は
  $ bm(R)(hat(bm(n)),theta)bm(v) = (hat(bm(n))dot bm(v))hat(bm(n)) + cos theta med bm(v)_perp + sin theta(hat(bm(n))times bm(v)_perp) $
  平行成分 $(hat(bm(n))dot bm(v))hat(bm(n))$ は変化せず、直交成分 $bm(v)_perp$ は、それ自身と $hat(bm(n))times bm(v)_perp$(同じ長さで直交する)が張る平面内で角度 $theta$ だけ回転する。これは軸 $hat(bm(n))$ まわりの回転そのものである。

  *(iii) $bm(R)(hat(bm(n)),theta) in "SO"(3)$ の証明。* $hat(bm(n))$ に直交する単位ベクトル $bm(e)_2$ を1つ取り、$bm(e)_3 := hat(bm(n))times bm(e)_2$ とおくと、${hat(bm(n)),bm(e)_2,bm(e)_3}$ は $RR^3$ の正規直交基底をなす($hat(bm(n))dot bm(e)_2=0$、$|bm(e)_3|=|hat(bm(n))||bm(e)_2|=1$、$hat(bm(n)) dot bm(e)_3 = hat(bm(n)) dot (hat(bm(n))times bm(e)_2)=0$ はスカラー三重積の性質)。

  #ref(<rodrigues-geo>) に $bm(v)=hat(bm(n)),bm(e)_2,bm(e)_3$ を代入すると
  $ bm(R)(hat(bm(n)),theta)hat(bm(n)) = hat(bm(n)) $
  $ bm(R)(hat(bm(n)),theta)bm(e)_2 = cos theta med bm(e)_2 + sin theta(hat(bm(n))times bm(e)_2) = cos theta med bm(e)_2 + sin theta med bm(e)_3 $
  $ bm(R)(hat(bm(n)),theta)bm(e)_3 = cos theta med bm(e)_3 + sin theta(hat(bm(n))times bm(e)_3) $
  ここで BAC-CAB則より $hat(bm(n))times bm(e)_3 = hat(bm(n))times(hat(bm(n))times bm(e)_2) = hat(bm(n))(hat(bm(n))dot bm(e)_2) - bm(e)_2(hat(bm(n))dot hat(bm(n))) = -bm(e)_2$ なので
  $ bm(R)(hat(bm(n)),theta)bm(e)_3 = -sin theta med bm(e)_2 + cos theta med bm(e)_3 $

  よって、正規直交基底 $bm(P):=[hat(bm(n)) med bm(e)_2 med bm(e)_3]$(各列を基底ベクトルとする直交行列、$bm(P)^T bm(P)=bm(I)$)を用いると
  $ bm(P)^T bm(R)(hat(bm(n)),theta) bm(P) = mat(1,0,0; 0,cos theta,-sin theta; 0,sin theta,cos theta) =: bm(D) $
  $bm(D)$ は明らかに直交行列で $det(bm(D))=1$(2次元回転行列との直和)。$bm(R)(hat(bm(n)),theta) = bm(P)bm(D)bm(P)^T$ と書けるので

  $ bm(R)(hat(bm(n)),theta)^T bm(R)(hat(bm(n)),theta) = bm(P)bm(D)^T bm(P)^T bm(P) bm(D) bm(P)^T = bm(P)bm(D)^T bm(D) bm(P)^T = bm(P)bm(I)bm(P)^T = bm(P)bm(P)^T = bm(I) $

  （$bm(P)^T bm(P)=bm(I)$ かつ $bm(D)^T bm(D)=bm(I)$ を使った。$bm(P)bm(P)^T=bm(I)$ は $bm(P)$ が正方直交行列であることから従う）。また $det(bm(R)(hat(bm(n)),theta)) = det(bm(P))det(bm(D))det(bm(P)^T) = det(bm(D)) = 1$($det(bm(P))det(bm(P)^T)=det(bm(P))^2$ ではなく、$det(bm(P)^T)=det(bm(P))$ と $bm(P)$が直交行列であることから $det(bm(P))=plus.minus 1$、よって $det(bm(P))det(bm(P)^T)=det(bm(P))^2=1$ となるので消える)。ゆえに $bm(R)(hat(bm(n)),theta) in "SO"(3)$。
]

#remark[
  この定理が言っているのは、3次元の回転は結局のところ「軸方向はそのまま、軸に垂直な平面内だけを1節の2次元回転行列 $bm(R)(theta)$ で回す」操作に他ならない、ということだ。証明(iii)で作った基底 $bm(P)=[hat(bm(n)) med bm(e)_2 med bm(e)_3]$ に関して回転行列を書き直すと、対角ブロックの片方が $1$(軸方向不変)、もう片方がちょうど1節の $bm(R)(theta)$(垂直平面内の回転)になっているのはそのためである。
]

= クォータニオン

回転行列(9成分、6個の直交条件)や軸角度(角度0付近で軸の向きが不定になる)には、それぞれ扱いにくさがある。3DCGやゲームエンジンでキャラクターの向きを滑らかに補間する(slerp)場合や、姿勢制御・VRのヘッドトラッキングで数値誤差の蓄積による直交性の崩れを避けたい場合、実務では回転行列そのものではなくクォータニオンが使われることが多い。クォータニオンは、たった4つの数で回転を表しつつこうした問題を避けられる表現であり、`03_rotations_3d.py`の`quat_from_axis_angle`/`rotation_from_quat`に対応する。ここでは、クォータニオンから作った回転行列が、7節のRodriguesの回転公式と同じ回転を表すことを証明する。定義・変換公式は Lynch and Park, _Modern Robotics: Mechanics, Planning, and Control_ (Cambridge University Press, 2017), Appendix B.3 の記法(式B.9, B.12, B.15)に合わせている。

#definition[
  単位ベクトル $hat(bm(n))$ と角度 $theta$ に対し、クォータニオン $bm(q)=(q_0,q_1,q_2,q_3) in RR^4$ を
  $ bm(q) := vec(cos(theta/2), hat(bm(n)) sin(theta/2)) $
  で定める(コードの `quat_from_axis_angle` に対応)。逆に、単位クォータニオン $bm(q)$ から回転行列を
  $ bm(R)(bm(q)) := mat(
    q_0^2+q_1^2-q_2^2-q_3^2, 2(q_1q_2-q_0q_3), 2(q_0q_2+q_1q_3);
    2(q_0q_3+q_1q_2), q_0^2-q_1^2+q_2^2-q_3^2, 2(q_2q_3-q_0q_1);
    2(q_1q_3-q_0q_2), 2(q_0q_1+q_2q_3), q_0^2-q_1^2-q_2^2+q_3^2
  ) $
  で定める(コードの `rotation_from_quat` に対応)。
]

軸角度からクォータニオンを作り、それを回転行列に戻す——この2つの変換を合成すると、元のRodriguesの回転公式に一致するはずである。それを確かめる。

#theorem[
  $bm(R)(bm(q)) = bm(R)(hat(bm(n)),theta)$(クォータニオンはRodriguesの回転公式と同じ回転を表す)。
]

#proof[
  $s:=sin(theta/2)$ とおくと $q_0=cos(theta/2)$, $(q_1,q_2,q_3)=s hat(bm(n))$ である。半角の公式より
  $ q_0^2 - s^2 = cos^2(theta/2)-sin^2(theta/2) = cos theta, quad 2 q_0 s = 2sin(theta/2)cos(theta/2) = sin theta $ <half-angle-1>
  $ 2s^2 = 1-cos theta $ <half-angle>

  $bm(R)(hat(bm(n)),theta)$ の対角成分は、skew行列の性質(c)より $[hat(bm(n))]_times^2 = hat(bm(n))hat(bm(n))^T-bm(I)$ なので
  $ R(hat(bm(n)),theta)_(1,1) = 1+(1-cos theta)(n_1^2-1) = cos theta + (1-cos theta) n_1^2 $
  一方 $bm(R)(bm(q))$ の対角成分は、$q_1^2+q_2^2+q_3^2=s^2$ に注意すると
  $ R(bm(q))_(1,1) &= q_0^2+q_1^2-q_2^2-q_3^2 = q_0^2 + 2q_1^2 - s^2 = (q_0^2-s^2) + 2q_1^2 \
                  &= cos theta + 2s^2 n_1^2 = cos theta + (1-cos theta)n_1^2 $
  （最後の等号は#ref(<half-angle>)の$2s^2=1-cos theta$と$q_1=s n_1$より）。両者は一致する。対角成分 $(2,2),(3,3)$ も同様。

  非対角成分は、例えば $(1,2)$ 成分について、$[hat(bm(n))]_times$ の定義より $(K)_(1,2)=-n_3$、$(hat(bm(n))hat(bm(n))^T)_(1,2)=n_1n_2$ なので
  $ R(hat(bm(n)),theta)_(1,2) = sin theta dot (-n_3) + (1-cos theta)n_1n_2 = -sin theta med n_3 + (1-cos theta)n_1n_2 $
  一方
  $ R(bm(q))_(1,2) = 2(q_1q_2-q_0q_3) = 2s^2n_1n_2 - 2q_0 s med n_3 = (1-cos theta)n_1n_2 - sin theta med n_3 $
  （#ref(<half-angle>)の$2s^2=1-cos theta$と#ref(<half-angle-1>)の$2q_0s=sin theta$を使った）。両者は一致する。他の非対角成分も同様の計算で一致することが確かめられる。
]

#remark[
  この定理が言っているのは、クォータニオンは回転行列の「圧縮表現」に過ぎないということだ。9成分・6個の拘束条件を持つ回転行列の代わりに、4成分・1個の拘束条件($||bm(q)||=1$)で同じ回転を表せる。

  $bm(R)(bm(q))$ の各成分は $bm(q)$ の2次同次多項式なので、$bm(R)(-bm(q))=(-1)^2 bm(R)(bm(q))=bm(R)(bm(q))$ が定義から直ちに従う。すなわち $bm(q)$ と $-bm(q)$ は同じ回転を表す(二重被覆)。これは `03_rotations_3d.py` で数値的にも確認済み。

  クォータニオンの積(ハミルトン積、コードの`quat_multiply`)が回転の合成 $bm(R)(bm(q)_1 dot bm(q)_2) = bm(R)(bm(q)_1)bm(R)(bm(q)_2)$ に対応することは、$bm(q)_1,bm(q)_2$ を$2 times 2$の複素行列として表現し積を取ることで示せる(Lynch and Park, 前掲書, 式B.13〜B.15)。ここでは繰り返さず、`03_rotations_3d.py` での数値的な検算(200サンプルのランダムな回転で最大誤差 $10^(-8)$ 以下)に委ねる。
]

= 一般化された幾何ヤコビアン(多関節・3次元)

「順運動学とヤコビ行列」の節(2節)では2関節平面アームについて、ヤコビ行列の列が $bm(Omega)(bm(f)(bm(q))-bm(p)_(i-1))$ の形で書けることを示し、これが一般公式 $bm(z)_(i-1) times (bm(o)_n - bm(o)_(i-1))$ の平面版であるとRemarkで述べるに留めていた。実際の産業用アーム(6軸垂直多関節アームなど)は関節軸が3次元空間で任意の向きを持ちうるので、2節の「$90 degree$回転」を「関節軸 $bm(z)_(i-1)$ まわりの回転の微小変化」に一般化する必要がある。ここでは、$n$個の回転関節からなる任意の直列マニピュレータについて、この一般公式を完全に証明する(`06_differential_kinematics_3d.py` の `geometric_jacobian` に対応)。

#definition[
  DH変換の列 $bm(A)_1(theta_1),dots,bm(A)_n(theta_n)$ からなる直列マニピュレータを考える(各 $bm(A)_j$ は4x4の同次変換行列)。回転関節の定義より、各 $bm(A)_j(theta_j) = bm(R)_z(theta_j) bm(B)_j$ と書ける。ここで $bm(R)_z(theta)$ は $z$軸まわりの回転を表す4x4行列(3x3回転行列を並進0で埋め込んだもの)、$bm(B)_j$ は $theta_j$ に依存しない定数の4x4同次変換行列である。

  $bm(T)_0^i (theta_1,dots,theta_i) := bm(A)_1 bm(A)_2 dots bm(A)_i$(ただし $bm(T)_0^0:=bm(I)$)とし、$bm(R)_i, bm(o)_i$ をそれぞれ $bm(T)_0^i$ の回転部分・並進部分(手先位置)とする。$bm(z)_i := bm(R)_i bm(e)_3$(フレーム$i$の$z$軸を基準フレームで表したもの、$bm(e)_3=(0,0,1)^T$)とおく。
]

証明の鍵は、回転関節による変換 $bm(A)_j(theta_j)$ を $theta_j$ で微分する計算である。まずその土台になる、$z$軸回転の微分公式を示す。

#lemma[
  $bm(Omega)_z := mat(0,-1,0;1,0,0;0,0,0)$(3x3、$z$軸方向の交代行列)とし、$hat(bm(Omega))_z := mat(bm(Omega)_z,bm(0);bm(0)^T,0)$(4x4 embedding)とすると
  $ (partial bm(R)_z(theta))/(partial theta) = bm(Omega)_z bm(R)_z (theta) $
  である。ここで両辺は4x4行列として見てよい。
]

#proof[
  $bm(R)_z(theta)$ の3x3回転部分は、$z$軸を回転軸とするRodriguesの回転公式 $bm(R)(bm(e)_3,theta) = bm(I)+sin theta [bm(e)_3]_times + (1-cos theta)[bm(e)_3]_times^2$ に他ならず、$[bm(e)_3]_times = bm(Omega)_z$ である。よって
  $ (partial)/(partial theta) bm(R)(bm(e)_3,theta) = cos theta med bm(Omega)_z + sin theta med bm(Omega)_z^2 = bm(Omega)_z (cos theta med bm(I) + sin theta med bm(Omega)_z) $
  一方 skew行列の性質(c)より $bm(Omega)_z^3 = -bm(Omega)_z$ なので
  $ bm(Omega)_z bm(R)(bm(e)_3,theta) = bm(Omega)_z + sin theta med bm(Omega)_z^2 + (1-cos theta)bm(Omega)_z^3 = bm(Omega)_z + sin theta med bm(Omega)_z^2 - (1-cos theta)bm(Omega)_z \
    = cos theta med bm(Omega)_z + sin theta med bm(Omega)_z^2 $
  両者は一致する。並進部分・4行4列目は両辺とも $bm(0)$ なので、4x4行列としても等式が成り立つ。
]

この補題を使って、手先位置・姿勢を各関節角で微分する。

#theorem[
  $bm(o)_n$(手先位置)の各関節角に関する偏微分、および角速度に対応する回転部分の偏微分は
  $ (partial bm(o)_n)/(partial theta_j) = bm(z)_(j-1) times (bm(o)_n - bm(o)_(j-1)), quad (partial bm(R)_n)/(partial theta_j) = [bm(z)_(j-1)]_times bm(R)_n $
  で与えられる。すなわち幾何ヤコビ行列 $bm(J) in RR^(6 times n)$(上3行が線速度、下3行が角速度)の第$j$列は
  $ bm(J)_(: comma j) = vec(bm(z)_(j-1) times (bm(o)_n - bm(o)_(j-1)), bm(z)_(j-1)) $
  である(コードの `geometric_jacobian` に対応)。
]

#proof[
  $bm(A)_j(theta_j) = bm(R)_z(theta_j)bm(B)_j$ なので、上の補題より
  $ (partial bm(A)_j)/(partial theta_j) = hat(bm(Omega))_z bm(R)_z(theta_j) bm(B)_j = hat(bm(Omega))_z bm(A)_j (theta_j) $ <dAj>

  $bm(T)_0^n = bm(T)_0^(j-1) bm(A)_j bm(A)_(j+1) dots bm(A)_n$ であり、$bm(A)_j$ のみが $theta_j$ に依存するので、積の微分則と#ref(<dAj>)より
  $ (partial bm(T)_0^n)/(partial theta_j) = bm(T)_0^(j-1) hat(bm(Omega))_z bm(A)_j bm(A)_(j+1) dots bm(A)_n = bm(T)_0^(j-1) hat(bm(Omega))_z (bm(T)_0^(j-1))^(-1) bm(T)_0^n $ <dT0n>

  （$bm(A)_j dots bm(A)_n = (bm(T)_0^(j-1))^(-1)bm(T)_0^n$ を使った）。ここで $bm(T)_0^(j-1) hat(bm(Omega))_z (bm(T)_0^(j-1))^(-1)$ を計算する。$bm(T)_0^(j-1) = mat(bm(R)_(j-1),bm(o)_(j-1);bm(0)^T,1)$、$(bm(T)_0^(j-1))^(-1) = mat(bm(R)_(j-1)^T,-bm(R)_(j-1)^T bm(o)_(j-1);bm(0)^T,1)$(`04_homogeneous_transforms.py` で検算済みの同次変換の逆行列の公式)なので、ブロック行列の積を計算すると

  $ hat(bm(Omega))_z (bm(T)_0^(j-1))^(-1) = mat(bm(Omega)_z bm(R)_(j-1)^T, -bm(Omega)_z bm(R)_(j-1)^T bm(o)_(j-1); bm(0)^T,0) $

  $ bm(T)_0^(j-1) hat(bm(Omega))_z (bm(T)_0^(j-1))^(-1) = mat(bm(R)_(j-1)bm(Omega)_z bm(R)_(j-1)^T, -bm(R)_(j-1)bm(Omega)_z bm(R)_(j-1)^T bm(o)_(j-1); bm(0)^T,0) $

  skew行列の性質(b)より $bm(R)_(j-1)bm(Omega)_z bm(R)_(j-1)^T = bm(R)_(j-1)[bm(e)_3]_times bm(R)_(j-1)^T = [bm(R)_(j-1)bm(e)_3]_times = [bm(z)_(j-1)]_times$ なので

  $ bm(T)_0^(j-1) hat(bm(Omega))_z (bm(T)_0^(j-1))^(-1) = mat([bm(z)_(j-1)]_times, -[bm(z)_(j-1)]_times bm(o)_(j-1); bm(0)^T,0) =: hat(bm(xi))_j $ <twist>

  #ref(<dT0n>) と#ref(<twist>)を合わせ、$bm(T)_0^n = mat(bm(R)_n,bm(o)_n;bm(0)^T,1)$ とブロック行列の積を計算すると

  $ (partial bm(T)_0^n)/(partial theta_j) &= hat(bm(xi))_j bm(T)_0^n = mat([bm(z)_(j-1)]_times bm(R)_n, [bm(z)_(j-1)]_times bm(o)_n - [bm(z)_(j-1)]_times bm(o)_(j-1); bm(0)^T,0) \
    &= mat([bm(z)_(j-1)]_times bm(R)_n, [bm(z)_(j-1)]_times (bm(o)_n-bm(o)_(j-1)); bm(0)^T,0) $

  左上ブロック(回転部分)より $(partial bm(R)_n)/(partial theta_j) = [bm(z)_(j-1)]_times bm(R)_n$。右上ブロック(並進部分)より $(partial bm(o)_n)/(partial theta_j) = [bm(z)_(j-1)]_times (bm(o)_n-bm(o)_(j-1)) = bm(z)_(j-1) times (bm(o)_n-bm(o)_(j-1))$、これが主張の式である。
]

#remark[
  この結果は2節の平面版の完全な一般化になっている。2節では関節軸が常に紙面に垂直だったので、外積 $bm(z)_(i-1) times (dot)$ は $90 degree$ 回転 $bm(Omega)$ の作用に退化していたが、ここでは関節軸 $bm(z)_(j-1)$ が任意の向きを取りうる3次元の場合を、外積という形のまま正確に扱えている。言い換えると、「関節$j$を単位速度で動かしたときの手先の速度は、関節軸と(関節から手先までのベクトル)の外積」というのは、2次元・3次元によらず成り立つ1つの事実である。
]

= 特異点と可操作度

腕を真っ直ぐ伸ばしきった状態で、肘を伸ばす方向にさらに手先を押しても手先は動かせない——これが特異点である。9節で一般化したヤコビ行列 $bm(J)$ を使うと、この「ある方向に手先を動かせなくなる」状態を、可操作度という1つの数値で定量的に検出できる(コードの`manipulability`に対応)。

#definition[
  正方(または一般)行列 $bm(J)$ に対して、可操作度(manipulability measure)を $w(bm(J)) := sqrt(det(bm(J)bm(J)^T))$ で定める(Yoshikawa, 1985)。$bm(J)$ が正方のとき $w(bm(J))=|det(bm(J))|$ である(コードの `manipulability` に対応)。
]

#remark[
  MIT 2.12(Asada, _Introduction to Robotics_, Fall 2005, OCW)第5章では、2関節平面アームの特異配置(肘が伸びきる/折れきる: $theta_2=0,pi$)において、ヤコビ行列の2つの列 $bm(J)_1,bm(J)_2$ が同じ方向を向いてしまい $det bm(J)=0$ になることが示されている。`06_differential_kinematics_3d.py` の3自由度アームでも、肘関節 $q_3=0$(伸びきり)・$q_3=plus.minus pi$(折れきり)で可操作度が数値的に0に近づくことを確認しており($10^(-6)$ 以下)、同じ現象の3次元版になっている。特異点では、ヤコビ行列の列空間(到達可能な手先速度の空間)の次元が落ち、少なくとも1つの方向に手先を動かせなくなる。
]

= 修正DH(Craig)規約との対応

`05_dh_parameters.py` では標準(古典)DH規約 $bm(A)_i = bm(R)_z(theta_i)bm(T)_z(d_i)bm(T)_x(a_i)bm(R)_x(alpha_i)$ を使ったが、Craigの教科書やLynch and Parkの付録では、添字の付け方が異なる修正DH規約 $bm(T)_(i-1,i) = bm(R)_x(alpha_(i-1))bm(T)_x(a_(i-1))bm(T)_z(d_i)bm(R)_z(theta_i)$ が使われる。同じ物理的なロボットに対して、この2つの規約は異なる数値のパラメータ表を要求するので、単純な「読み替え」はできない。ここでは、標準DH表から修正DH表を機械的に作る具体的な変換規則を与え、両者が生成する順運動学が(手先の1つ手前のリンクまでは)厳密に一致し、最後のリンクだけ補正項が必要になることを証明する(`07_modified_dh.py` に対応)。

#definition[
  単位ベクトル $hat(bm(x))$ 方向の並進 $bm(T)_x(a)$・回転 $bm(R)_x(alpha)$、$hat(bm(z))$ 方向の並進 $bm(T)_z(d)$・回転 $bm(R)_z(theta)$ を用いて、修正DH変換を
  $ bm(A)_i^"mod" := bm(R)_x(alpha_(i-1)) bm(T)_x(a_(i-1)) bm(T)_z(d_i) bm(R)_z(theta_i) $
  と定める(コードの `modified_dh_transform` に対応)。
]

この修正DH変換を標準DH変換とつなげるために、まず「同じ軸方向の並進と回転は可換」という補題を用意する。

#lemma("同軸の並進と回転の可換性")[
  $ bm(T)_x(a) bm(R)_x(alpha) = bm(R)_x(alpha) bm(T)_x(a), quad bm(T)_z(d) bm(R)_z(theta) = bm(R)_z(theta) bm(T)_z(d) $
]

#proof[
  $x$軸方向の場合を示す($z$軸方向も同様)。$bm(T)_x(a) = mat(bm(I),a hat(bm(x));bm(0)^T,1)$、$bm(R)_x(alpha) = mat(bm(R),bm(0);bm(0)^T,1)$(ここで$bm(R):=bm(R)(hat(bm(x)),alpha)$は7節のRodriguesの回転公式)とブロック行列で書くと
  $ bm(T)_x(a) bm(R)_x(alpha) = mat(bm(R), a hat(bm(x));bm(0)^T,1), quad bm(R)_x(alpha)bm(T)_x(a) = mat(bm(R), a bm(R)hat(bm(x));bm(0)^T,1) $
  $hat(bm(x))$は回転軸そのものなので、7節の定理7.1(または直接#ref(<rodrigues-geo>))より$bm(R)hat(bm(x))=hat(bm(x))$(回転軸方向の成分は回転で変化しない)。よって両辺の右上ブロックは一致し、$bm(T)_x(a)bm(R)_x(alpha)=bm(R)_x(alpha)bm(T)_x(a)$。
]

この可換性を使うと、標準DH表から機械的に作った修正DH表が、標準DH規約の順運動学とどう関係するかが示せる。

#theorem[
  標準DH表 ${(a_i,alpha_i,d_i)}_(i=1)^n$ から、$a_i^"mod":=a_(i-1)$, $alpha_i^"mod":=alpha_(i-1)$(ただし$a_0:=0,alpha_0:=0$)、$d_i^"mod":=d_i$、$theta_i^"mod":=theta_i$ として修正DH表を作ると、任意の$k=1,dots,n$について
  $ bm(T)_0^k ("modified") = bm(T)_0^(k-1) ("standard") thin bm(R)_z(theta_k) bm(T)_z(d_k) $ <mod-dh-thm>
  が成り立つ(コードの `standard_table_to_modified_table` に対応)。
]

#proof[
  $k$についての帰納法で示す。

  *base case ($k=1$)。* $a_1^"mod"=a_0=0$, $alpha_1^"mod"=alpha_0=0$ なので $bm(A)_1^"mod" = bm(R)_x(0)bm(T)_x(0)bm(T)_z(d_1)bm(R)_z(theta_1) = bm(T)_z(d_1)bm(R)_z(theta_1) = bm(R)_z(theta_1)bm(T)_z(d_1)$(上の補題)。$bm(T)_0^0("standard")=bm(I)$ なので、これは#ref(<mod-dh-thm>)の$k=1$の場合に一致する。

  *帰納段階。* $k$で#ref(<mod-dh-thm>)が成り立つと仮定し、$k+1$を示す。定義より $bm(T)_0^(k+1)("modified") = bm(T)_0^k("modified") bm(A)_(k+1)^"mod"$。帰納法の仮定と、$a_(k+1)^"mod"=a_k$, $alpha_(k+1)^"mod"=alpha_k$ (標準DH表の値)より
  $ bm(T)_0^(k+1)("modified") &= bm(T)_0^(k-1)("standard") bm(R)_z(theta_k)bm(T)_z(d_k) dot bm(R)_x(alpha_k)bm(T)_x(a_k)bm(T)_z(d_(k+1))bm(R)_z(theta_(k+1)) $
  上の補題より$bm(R)_x(alpha_k)bm(T)_x(a_k) = bm(T)_x(a_k)bm(R)_x(alpha_k)$なので
  $ &= bm(T)_0^(k-1)("standard") underbrace(bm(R)_z(theta_k)bm(T)_z(d_k)bm(T)_x(a_k)bm(R)_x(alpha_k), = bm(A)_k^"standard") bm(T)_z(d_(k+1))bm(R)_z(theta_(k+1)) \
    &= bm(T)_0^k ("standard") bm(R)_z(theta_(k+1)) bm(T)_z(d_(k+1)) $
  (最後の等号は補題を再度使って$bm(T)_z(d_(k+1))bm(R)_z(theta_(k+1))=bm(R)_z(theta_(k+1))bm(T)_z(d_(k+1))$と書き換えた)。これは#ref(<mod-dh-thm>)の$k+1$の場合である。
]

最後のリンク($k=n$)まで進めると、標準規約と修正規約それぞれの全体の同次変換行列がどうずれるかが分かる。

#corollary[
  $ bm(T)_0^n ("standard") = bm(T)_0^n ("modified") thin bm(T)_x(a_n) bm(R)_x(alpha_n) $ <mod-dh-cor>
]

#proof[
  標準DH変換の定義と#ref(<mod-dh-thm>)の$k=n$の場合より
  $ bm(T)_0^n("standard") &= bm(T)_0^(n-1)("standard") bm(A)_n^"standard" \
    &= bm(T)_0^(n-1)("standard")bm(R)_z(theta_n)bm(T)_z(d_n) dot bm(T)_x(a_n)bm(R)_x(alpha_n) \
    &= bm(T)_0^n("modified")bm(T)_x(a_n)bm(R)_x(alpha_n) $
]

#remark[
  `07_modified_dh.py`では、ランダムな4関節のDHパラメータ表について#ref(<mod-dh-thm>)を$k=1,dots,4$それぞれで数値的に確認し(最大誤差 $2.22 times 10^(-16)$)、#ref(<mod-dh-cor>)も同様に確認している(誤差 $2.22 times 10^(-16)$)。補正項$bm(T)_x(a_n)bm(R)_x(alpha_n)$を付けない場合は一般に一致しないことも確認しており(食い違いの大きさ$0.52$)、`05_dh_parameters.py`の2関節平面アーム例でも#ref(<mod-dh-cor>)が成り立つことを確認している。直感的には、修正DH規約のフレーム$i$は標準DH規約のフレーム$i$より「1リンク手前」の位置に置かれるため、最後のリンクの並進・回転だけがずれる。
]

= 冗長マニピュレータの擬似逆行列IKと零空間

ここまでの逆運動学は、関節数とタスク空間の次元が一致する場合(2関節アームで2次元位置)を扱ってきた。テーブルの上のコップを掴む場面を考えてほしい——手の位置と向きだけなら肩・肘・手首の数自由度で決まるはずなのに、人間の腕は肩から先だけでも冗長な自由度を持ち、肘の高さを自由に変えてもコップを掴む手の位置は変えられる。関節数がタスクの自由度より多い冗長マニピュレータでは、まさにこの状況が起き、目標を達成する関節速度 $dot(bm(theta))$ が一意に決まらず、残った自由度を可操作度の最大化や関節可動域の回避といった副次的な目的に使える。この節では、擬似逆行列 $bm(J)^+$ と、それが誘導する零空間への射影の性質を証明し、`08_redundant_ik_nullspace.py`(2次元位置のみを制御する3関節平面アーム、冗長度1)の実装を裏付ける。設定はBuss, "Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares methods" (CMU, 2009)の式(7)(9)に基づくが、同資料が証明を省略している性質はここで自前で証明する(零空間法自体の初出はLiegeois, 1977)。

#definition[
  $bm(J) in RR^(m times n)$($m<n$、階数$m$のフルランク)に対して、右擬似逆行列を
  $ bm(J)^+ := bm(J)^T (bm(J)bm(J)^T)^(-1) $
  と定める(コードの `right_pseudo_inverse` に対応。$bm(J)bm(J)^T in RR^(m times m)$は$bm(J)$がフルランクなので正則)。
]

この擬似逆行列が、実際に「逆行列」と呼ぶにふさわしい性質(右逆行列であること、それが誘導する射影の性質)を持つことを確認する。

#theorem("擬似逆行列の基本性質")[
  $bm(P) := bm(I)_n - bm(J)^+ bm(J)$ とおくと、次が成り立つ。
  + $bm(J)bm(J)^+ = bm(I)_m$(右逆行列になっている)
  + $bm(P)^2 = bm(P)$($bm(P)$は射影行列)
  + $bm(J)bm(P) = bm(0)$($bm(P)$の像は$bm(J)$の零空間に含まれる)
  + $bm(P)$の像は、ちょうど$bm(J)$の零空間に等しい: $"image"(bm(P)) = "null"(bm(J))$
]

#proof[
  *(1)* $bm(J)bm(J)^+ = bm(J)bm(J)^T(bm(J)bm(J)^T)^(-1) = bm(I)_m$。

  *(2)* まず $bm(J)^+ bm(J) bm(J)^+ = bm(J)^+(bm(J)bm(J)^+) = bm(J)^+ bm(I)_m = bm(J)^+$((1)を使った)。これを使うと
  $ bm(P)^2 &= (bm(I)-bm(J)^+bm(J))(bm(I)-bm(J)^+bm(J)) = bm(I) - 2bm(J)^+bm(J) + bm(J)^+bm(J)bm(J)^+bm(J) \
    &= bm(I)-2bm(J)^+bm(J)+bm(J)^+bm(J) = bm(I)-bm(J)^+bm(J) = bm(P) $

  *(3)* $bm(J)bm(P) = bm(J)-bm(J)bm(J)^+bm(J) = bm(J)-(bm(J)bm(J)^+)bm(J) = bm(J)-bm(I)_m bm(J) = bm(0)$((1)を使った)。

  *(4)* (3)より、任意の$bm(phi.alt) in RR^n$に対して$bm(J)(bm(P)bm(phi.alt))=(bm(J)bm(P))bm(phi.alt)=bm(0)$なので$bm(P)bm(phi.alt) in "null"(bm(J))$、すなわち$"image"(bm(P)) subset.eq "null"(bm(J))$。

  逆に $bm(n) in "null"(bm(J))$(すなわち$bm(J)bm(n)=bm(0)$)とすると、$bm(P)bm(n) = bm(n)-bm(J)^+(bm(J)bm(n)) = bm(n)-bm(J)^+ bm(0) = bm(n)$。ゆえに$bm(n)=bm(P)bm(n) in "image"(bm(P))$、すなわち$"null"(bm(J)) subset.eq "image"(bm(P))$。両方向の包含から$"image"(bm(P))="null"(bm(J))$。
]

擬似逆行列 $bm(J)^+ bm(e)$ が $bm(J)dot(bm(theta))=bm(e)$ の1つの解であることは(1)から分かるが、数ある解の中でなぜこれを基準に選ぶのか——実は、最もノルムが小さい特別な解になっている。

#theorem("最小ノルム性")[
  $bm(e) in RR^m$ が $bm(J)dot(bm(theta))=bm(e)$ を満たす解を持つ(すなわち$bm(e) in "range"(bm(J))$、$bm(J)$がフルランク$m$のときは常に成り立つ)とする。この方程式の解の中で、$dot(bm(theta))=bm(J)^+ bm(e)$ が最小の2-ノルムを持つ唯一の解である。
]

#proof[
  *(i) 解の集合。* $dot(bm(theta))$を任意の解とし、$bm(n):=dot(bm(theta))-bm(J)^+bm(e)$とおく。定理の(1)より $bm(J)bm(n) = bm(J)dot(bm(theta))-bm(J)bm(J)^+bm(e) = bm(e)-bm(e)=bm(0)$なので$bm(n) in "null"(bm(J))$。すなわち任意の解は $dot(bm(theta))=bm(J)^+bm(e)+bm(n)$($bm(n) in "null"(bm(J))$)の形に書ける。

  *(ii) 直交性。* $bm(J)^+bm(e)$と$bm(n)$が直交することを示す。$(bm(J)bm(J)^T)^(-1)$は対称行列の逆行列なので対称であることに注意すると
  $ (bm(J)^+ bm(e)) dot bm(n) &= bm(e)^T (bm(J)^+)^T bm(n) = bm(e)^T (bm(J)^T(bm(J)bm(J)^T)^(-1))^T bm(n) \
    &= bm(e)^T (bm(J)bm(J)^T)^(-1) bm(J) bm(n) = bm(e)^T (bm(J)bm(J)^T)^(-1) bm(0) = 0 $
  ($bm(n) in "null"(bm(J))$より$bm(J)bm(n)=bm(0)$を使った)。

  *(iii) ピタゴラスの定理。* (ii)より
  $ ||dot(bm(theta))||^2 = ||bm(J)^+bm(e)+bm(n)||^2 = ||bm(J)^+bm(e)||^2 + 2(bm(J)^+bm(e))dot bm(n) + ||bm(n)||^2 = ||bm(J)^+bm(e)||^2+||bm(n)||^2 >= ||bm(J)^+bm(e)||^2 $
  等号は$bm(n)=bm(0)$のとき、かつそのときに限る。ゆえに$bm(J)^+bm(e)$が唯一の最小ノルム解である。
]

最小ノルム解 $bm(J)^+ bm(e)$ だけでは、冗長関節が持つ余った自由度を活かせない。零空間 $"null"(bm(J))$ の成分を足しても主タスクを崩さないことを最後に確認し、それを副次目的に使う。

#corollary("零空間法")[
  任意の$bm(phi.alt) in RR^n$に対して、$dot(bm(theta)) := bm(J)^+ bm(e) + bm(P)bm(phi.alt)$ は $bm(J)dot(bm(theta))=bm(e)$ を満たす。逆に、$bm(J)dot(bm(theta))=bm(e)$を満たす任意の$dot(bm(theta))$は、この形($bm(phi.alt):=dot(bm(theta))$と取れば)で表せる(コードの`redundant_ik`の更新式に対応)。
]

#proof[
  *前半。* 定理(1)(3)より $bm(J)dot(bm(theta)) = bm(J)bm(J)^+bm(e)+bm(J)bm(P)bm(phi.alt) = bm(e)+bm(0)=bm(e)$。

  *後半。* $bm(J)dot(bm(theta))=bm(e)$ とする。上の定理の証明(i)と同様に $bm(J)^+ bm(J) dot(bm(theta)) = bm(J)^+ bm(e)$ なので、$bm(P)dot(bm(theta)) = dot(bm(theta)) - bm(J)^+ bm(J)dot(bm(theta)) = dot(bm(theta))-bm(J)^+bm(e)$。ゆえに $bm(phi.alt):=dot(bm(theta))$ とおくと $bm(J)^+bm(e)+bm(P)bm(phi.alt) = bm(J)^+bm(e)+dot(bm(theta))-bm(J)^+bm(e) = dot(bm(theta))$。
]

#remark[
  この結果が言っているのは、冗長関節が持つ「余った自由度」は主タスク(手先位置)を一切邪魔しない範囲で自由に使ってよく、零空間法 $dot(bm(theta)) = bm(J)^+ bm(e) + bm(P)bm(phi.alt)$ はその使い方——好きな副次目的 $bm(phi.alt)$ を選べば、それがそのまま主タスクを崩さない関節速度に変換される——を数式として保証する、ということだ。コップを掴む例で言えば、$bm(J)^+ bm(e)$ が「手をコップへ向かわせる最小限の動き」、$bm(P)bm(phi.alt)$ が「手の位置を変えずに肘の高さだけ変える自己運動」に対応する。

  `08_redundant_ik_nullspace.py`では、$bm(phi.alt)$として可操作度 $w(bm(J))=sqrt(det(bm(J)bm(J)^T))$(定義10.1と同じYoshikawaの尺度)の勾配 $nabla w(bm(theta))$ を有限差分で計算して採用している。数値的には、$bm(J)bm(J)^+=bm(I)$の誤差 $10^(-14)$ 以下、$bm(P)$の冪等性・零空間性の誤差 $10^(-14)$ 以下、最小ノルム解であることをランダムな20個の別解との比較で確認している。零空間法を使うと、使わない場合(最小ノルム解のみ)より可操作度が改善し(例: $2.058 arrow 2.115$)、かつ手先位置の収束(主タスク)には影響しないことを確認している。さらに、目標位置を固定したまま冗長関節 $theta_1$ だけを連続的に動かし、残り2関節を逆運動学で追従させることで、同じ手先位置に到達する複数の姿勢の連続族(自己運動、self-motion)を追跡できることも確認している。
]
