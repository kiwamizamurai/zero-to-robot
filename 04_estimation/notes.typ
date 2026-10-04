#import "../common/theorems.typ": *

#show: setup-theorems

#set page(paper: "a4", margin: 2.5cm)
#set text(font: "New Computer Modern", size: 11pt, lang: "ja")
#set heading(numbering: "1.1")
#set math.equation(numbering: "(1)", supplement: "式")

#align(center)[
  #text(17pt, weight: "bold")[状態推定 --- 線形カルマンフィルタ]

  #text(11pt, fill: gray)[`01_kalman_filter.py` / `02_alpha_beta_steady_state.py` の数式的裏付け]
]

#v(1cm)

記法: ベクトル・行列は太字、スカラーは非太字で表す。本稿は `01_kinematics/notes.typ` で確立した「全微分を取って係数比較する」証明技法をそのまま流用する。行列に関する基本事実(転置と積・線形性)は `01_kinematics/notes.typ` の Lemma(基本事実) を参照し、ここでは繰り返さない。

= 設定

カルマンフィルタは、ノイズを含む観測列から状態を逐次的に推定する手法である。以下では、まず対象とする状態空間モデルを定義し、2-4節でこのモデルに対する予測ステップ・更新ステップの式を行間なく導出する。これが `01_kalman_filter.py` の `kalman_filter` 関数全体が実装する対象である。

#definition[
  離散時間の線形・ガウス状態空間モデルを次のように定める。
  $ bm(x)_t = bm(A) bm(x)_(t-1) + bm(w)_t, quad bm(w)_t tilde cal(N)(bm(0), bm(Q)) $
  $ bm(z)_t = bm(H) bm(x)_t + bm(v)_t, quad bm(v)_t tilde cal(N)(bm(0), bm(R)) $
  ここで $bm(x)_t in RR^n$ は状態、$bm(z)_t in RR^m$ は観測、$bm(A) in RR^(n times n)$ は状態遷移行列、$bm(H) in RR^(m times n)$ は観測行列、$bm(w)_t$(プロセスノイズ)と $bm(v)_t$(観測ノイズ)は互いに独立、時刻ごとにも独立とする（コードの `A`, `H`, `Q`, `R` に対応）。
]

#remark[
  文献によっては(例えば移動ロボットの教科書であるThrun, Burgard and Fox, *Probabilistic Robotics*, 2005の慣習に従う和文講義資料では)プロセスノイズの共分散を $bm(R)$、観測ノイズの共分散を $bm(Q)$ と、本稿とは逆の記号で書く場合がある。本稿ではコード(`Q`=プロセスノイズ, `R`=観測ノイズ)に合わせた記法を採用する。
]

= 期待値・共分散の基本性質

予測ステップ・更新ステップの導出(3, 4節)は、期待値・共分散を含む式の変形を繰り返し使う。先にその基本性質を証明し、以降はこれらを断りなく用いる。

#lemma("期待値・共分散")[
  (a) (線形性) 定数行列 $bm(A)$、定数ベクトル $bm(b)$、確率ベクトル $bm(y)$ に対して $EE[bm(A)bm(y)+bm(b)] = bm(A)EE[bm(y)]+bm(b)$。

  (b) (線形変換の共分散) 定数行列 $bm(A)$、確率ベクトル $bm(y)$ に対して $"Cov"(bm(A)bm(y)) = bm(A) "Cov"(bm(y)) bm(A)^T$。

  (c) (独立な確率ベクトルの和の共分散) 独立な確率ベクトル $bm(y), bm(z)$ に対して $"Cov"(bm(y)+bm(z)) = "Cov"(bm(y)) + "Cov"(bm(z))$。
]

#proof[
  (a) 期待値の定義(積分・和の線形性)から直ちに従う。

  (b) 共分散の定義 $"Cov"(bm(y)) := EE[(bm(y)-EE[bm(y)])(bm(y)-EE[bm(y)])^T]$ と (a) を用いると

  $ "Cov"(bm(A)bm(y)) &= EE[(bm(A)bm(y)-EE[bm(A)bm(y)])(bm(A)bm(y)-EE[bm(A)bm(y)])^T] \
                       &= EE[bm(A)(bm(y)-EE[bm(y)])(bm(y)-EE[bm(y)])^T bm(A)^T] \
                       &= bm(A) EE[(bm(y)-EE[bm(y)])(bm(y)-EE[bm(y)])^T] bm(A)^T = bm(A) "Cov"(bm(y)) bm(A)^T $

  (c) $bm(mu)_y := EE[bm(y)]$, $bm(mu)_z := EE[bm(z)]$ とおく。

  $ "Cov"(bm(y)+bm(z)) &= EE[((bm(y)-bm(mu)_y)+(bm(z)-bm(mu)_z))((bm(y)-bm(mu)_y)+(bm(z)-bm(mu)_z))^T] \
    &= "Cov"(bm(y)) + EE[(bm(y)-bm(mu)_y)(bm(z)-bm(mu)_z)^T] + EE[(bm(z)-bm(mu)_z)(bm(y)-bm(mu)_y)^T] + "Cov"(bm(z)) $

  $bm(y), bm(z)$ は独立なので、任意の可測関数 $f,g$ に対し $EE[f(bm(y))g(bm(z))^T] = EE[f(bm(y))]EE[g(bm(z))]^T$ が成り立つ(独立性の定義)。これを交差項に適用すると

  $ EE[(bm(y)-bm(mu)_y)(bm(z)-bm(mu)_z)^T] = EE[bm(y)-bm(mu)_y] EE[(bm(z)-bm(mu)_z)]^T = bm(0) dot bm(0)^T = bm(0) $

  （$EE[bm(y)-bm(mu)_y]=bm(0)$ は期待値の定義そのもの）。もう一方の交差項も同様に $bm(0)$。ゆえに $"Cov"(bm(y)+bm(z)) = "Cov"(bm(y))+"Cov"(bm(z))$。
]

= 予測ステップ

新しい観測が届く前に、現在の推定値と運動モデルだけから次の時刻の状態の予測分布を求める。これがカルマンフィルタの2段階(予測→更新)のうち前半である。時刻 $t-1$ の観測列を所与とした事後分布を $bm(x)_(t-1) tilde cal(N)(bm(mu)_(t-1), bm(Sigma)_(t-1))$ とする。

#theorem[
  予測分布 $bm(x)_t | bm(z)_(1:t-1) tilde cal(N)(overline(bm(mu))_t, overline(bm(Sigma))_t)$ の平均・共分散は
  $ overline(bm(mu))_t = bm(A) bm(mu)_(t-1), quad overline(bm(Sigma))_t = bm(A) bm(Sigma)_(t-1) bm(A)^T + bm(Q) $
  で与えられる（コードの `x_pred = A @ x`, `P_pred = A @ P @ A.T + Q` に対応）。
]

#proof[
  $bm(x)_t = bm(A)bm(x)_(t-1) + bm(w)_t$ であり、$bm(w)_t$ は $bm(x)_(t-1)$(過去の観測のみに依存)と独立である。期待値・共分散の基本事実の(a)より

  $ overline(bm(mu))_t = EE[bm(A)bm(x)_(t-1)+bm(w)_t] = bm(A)EE[bm(x)_(t-1)] + EE[bm(w)_t] = bm(A)bm(mu)_(t-1) + bm(0) = bm(A)bm(mu)_(t-1) $

  共分散については、$bm(A)bm(x)_(t-1)$ と $bm(w)_t$ が独立であることと基本事実の(b),(c)より

  $ overline(bm(Sigma))_t = "Cov"(bm(A)bm(x)_(t-1) + bm(w)_t) = "Cov"(bm(A)bm(x)_(t-1)) + "Cov"(bm(w)_t) = bm(A)bm(Sigma)_(t-1)bm(A)^T + bm(Q) $
]

#remark[
  この結果が言っているのは、「何も観測しないまま時間だけ進めると、不確かさは運動モデルを通した分($bm(A)bm(Sigma)_(t-1)bm(A)^T$)に加えてプロセスノイズの分($bm(Q)$)だけ必ず増える」ということだ。共分散が足し算(引き算ではない)で増えるのは、$bm(A)bm(x)_(t-1)$と$bm(w)_t$が独立な不確かさの源だから、というのが期待値・共分散の基本性質(c)の意味するところである。
]

= 更新ステップ

予測ステップで求めた分布を、新しい観測値で補正し、事後分布を求める。ここで導出するカルマンゲイン $bm(K)_t$ が、予測と観測のどちらをどれだけ信頼するかを決める重みである。観測 $bm(z)_t$ が得られたときの事後分布 $bm(x)_t | bm(z)_(1:t) tilde cal(N)(bm(mu)_t, bm(Sigma)_t)$ を、次のコスト関数を最小化する問題として定める(事前分布からの逸脱と観測との食い違いを、それぞれの不確かさの逆行列で重み付けた二次形式の和になっている点で、`01_kinematics` の損失関数 $cal(L)(bm(q))=1/2 bm(e)^T bm(e)$ と同じ発想である)。

$ J(bm(x)) := 1/2 (bm(x) - overline(bm(mu))_t)^T overline(bm(Sigma))_t^(-1) (bm(x) - overline(bm(mu))_t) + 1/2 (bm(z)_t - bm(H)bm(x))^T bm(R)^(-1) (bm(z)_t - bm(H)bm(x)) $

#lemma("二次形式の勾配")[
  対称行列 $bm(M) in RR^(n times n)$ と定数 $bm(a) in RR^n$ に対して、$F(bm(x)) := 1/2(bm(x)-bm(a))^T bm(M) (bm(x)-bm(a))$ の勾配は $nabla F(bm(x)) = bm(M)(bm(x)-bm(a))$ である。
]

#proof[
  `01_kinematics` の Lemma(勾配の行列表現)と同じ議論を用いる。$bm(y) := bm(x)-bm(a)$ とおくと、全微分は

  $ dif F = 1/2 dif (bm(y)^T bm(M) bm(y)) $

  積の微分則より $dif(bm(y)^T bm(M) bm(y)) = (dif bm(y))^T bm(M) bm(y) + bm(y)^T bm(M) (dif bm(y))$。右辺第1項はスカラーであり、基本事実(a)と $bm(M)^T=bm(M)$(対称性)より

  $ (dif bm(y))^T bm(M) bm(y) = ((dif bm(y))^T bm(M) bm(y))^T = bm(y)^T bm(M)^T dif bm(y) = bm(y)^T bm(M) dif bm(y) $

  であるから $dif(bm(y)^T bm(M) bm(y)) = 2 bm(y)^T bm(M) dif bm(y)$、よって $dif F = bm(y)^T bm(M) dif bm(y) = bm(y)^T bm(M) dif bm(x)$($bm(a)$は定数なので $dif bm(y)=dif bm(x)$)。勾配の定義 $dif F = nabla F(bm(x))^T dif bm(x)$ と係数比較して、転置を取ると $nabla F(bm(x)) = bm(M)^T bm(y) = bm(M)(bm(x)-bm(a))$。
]

この補題を $J$ の2つの二次形式それぞれに適用し、$nabla J(bm(x))=bm(0)$ を解けば、事前分布と観測を最適に統合する $bm(x)$ が求まる。

#theorem[
  $J$ を最小化する $bm(x)$ は
  $ bm(mu)_t = overline(bm(mu))_t + bm(K)_t (bm(z)_t - bm(H) overline(bm(mu))_t), quad bm(K)_t := overline(bm(Sigma))_t bm(H)^T (bm(H) overline(bm(Sigma))_t bm(H)^T + bm(R))^(-1) $
  であり、このときの共分散は $bm(Sigma)_t = (bm(I) - bm(K)_t bm(H)) overline(bm(Sigma))_t$ である（コードの `kalman_filter` 内の `K = P_pred @ H.T @ np.linalg.inv(S)`, `x = x_pred + K@(z-H@x_pred)`, `P = (I-K@H)@P_pred` に対応）。
]

#proof[
  *(i) 最小点の導出。* $bm(H)bm(x)$ の全微分は $dif(bm(H)bm(x)) = bm(H) dif bm(x)$ なので、二次形式の勾配の補題を2回(第1項に $bm(M)=overline(bm(Sigma))_t^(-1)$, $bm(a)=overline(bm(mu))_t$ として、第2項に対しては連鎖律を介して)適用すると

  $ nabla J(bm(x)) = overline(bm(Sigma))_t^(-1)(bm(x)-overline(bm(mu))_t) - bm(H)^T bm(R)^(-1) (bm(z)_t - bm(H)bm(x)) $

  （第2項の符号は、$bm(y):=bm(H)bm(x)-bm(z)_t$ とおいて補題を適用し $dif(1/2 bm(y)^T bm(R)^(-1) bm(y)) = bm(y)^T bm(R)^(-1) bm(H) dif bm(x)$ となることから従う）。$J$ は正定値行列 $overline(bm(Sigma))_t^(-1)$ に関する二次形式と半正定値の二次形式の和であり凸なので、最小点は $nabla J(bm(x))=bm(0)$ で特徴づけられる。

  $ overline(bm(Sigma))_t^(-1)(bm(x)-overline(bm(mu))_t) = bm(H)^T bm(R)^(-1) (bm(z)_t - bm(H)bm(x)) $

  右辺の $bm(z)_t - bm(H)bm(x) = (bm(z)_t - bm(H)overline(bm(mu))_t) - bm(H)(bm(x)-overline(bm(mu))_t)$ と書き換えて整理すると

  $ (overline(bm(Sigma))_t^(-1) + bm(H)^T bm(R)^(-1) bm(H))(bm(x)-overline(bm(mu))_t) = bm(H)^T bm(R)^(-1) (bm(z)_t - bm(H)overline(bm(mu))_t) $ <normal-eq>

  したがって

  $ bm(x) - overline(bm(mu))_t = (overline(bm(Sigma))_t^(-1) + bm(H)^T bm(R)^(-1) bm(H))^(-1) bm(H)^T bm(R)^(-1) (bm(z)_t - bm(H)overline(bm(mu))_t) $ <pre-gain>

  （左辺の逆行列は、$overline(bm(Sigma))_t^(-1)$ が正定値、$bm(H)^T bm(R)^(-1)bm(H)$ が半正定値であり、正定値行列と半正定値行列の和は正定値だから存在する）。

  *(ii) ゲイン行列の書き換え。* @pre-gain の係数行列が $bm(K)_t$ に一致することを示す。すなわち

  $ (overline(bm(Sigma))_t^(-1) + bm(H)^T bm(R)^(-1) bm(H))^(-1) bm(H)^T bm(R)^(-1) = overline(bm(Sigma))_t bm(H)^T (bm(H) overline(bm(Sigma))_t bm(H)^T + bm(R))^(-1) $ <gain-identity>

  を示せばよい。両辺に左から $(overline(bm(Sigma))_t^(-1) + bm(H)^T bm(R)^(-1) bm(H))$ を掛けたものが等しいことを確かめる。

  $ (overline(bm(Sigma))_t^(-1) + bm(H)^T bm(R)^(-1) bm(H)) overline(bm(Sigma))_t bm(H)^T (bm(H)overline(bm(Sigma))_t bm(H)^T + bm(R))^(-1) $
  $ = [bm(H)^T + bm(H)^T bm(R)^(-1) bm(H) overline(bm(Sigma))_t bm(H)^T] (bm(H)overline(bm(Sigma))_t bm(H)^T + bm(R))^(-1) $

  $bm(H)^T = bm(H)^T bm(R)^(-1) bm(R)$ と書けることに注意して括弧の中を $bm(H)^T bm(R)^(-1)$ で括ると

  $ bm(H)^T + bm(H)^T bm(R)^(-1) bm(H) overline(bm(Sigma))_t bm(H)^T = bm(H)^T bm(R)^(-1) (bm(R) + bm(H)overline(bm(Sigma))_t bm(H)^T) $

  ゆえに

  $ (overline(bm(Sigma))_t^(-1) + bm(H)^T bm(R)^(-1) bm(H)) overline(bm(Sigma))_t bm(H)^T (bm(H)overline(bm(Sigma))_t bm(H)^T + bm(R))^(-1) \
    = bm(H)^T bm(R)^(-1) (bm(R)+bm(H)overline(bm(Sigma))_t bm(H)^T)(bm(H)overline(bm(Sigma))_t bm(H)^T+bm(R))^(-1) = bm(H)^T bm(R)^(-1) $

  これは @gain-identity の右辺に左から同じ行列を掛けたもの、すなわち恒等的に成り立つ式 $(overline(bm(Sigma))_t^(-1)+bm(H)^T bm(R)^(-1)bm(H))(overline(bm(Sigma))_t^(-1)+bm(H)^T bm(R)^(-1)bm(H))^(-1)bm(H)^T bm(R)^(-1) = bm(H)^T bm(R)^(-1)$ と一致する。両辺に左から同じ正則行列を掛けて等しいことが分かったので、@gain-identity が成り立つ。@pre-gain と @gain-identity を合わせると $bm(mu)_t = overline(bm(mu))_t + bm(K)_t (bm(z)_t-bm(H)overline(bm(mu))_t)$ を得る。

  *(iii) 共分散の書き換え。* $bm(Sigma)_t := (overline(bm(Sigma))_t^(-1)+bm(H)^T bm(R)^(-1)bm(H))^(-1)$(@normal-eq の係数行列の逆行列。ガウス分布の正規方程式の係数行列の逆行列が事後共分散に等しいことは、線形回帰の正規方程式 $(bm(J)^T bm(J))^(-1)$ が誤差共分散を与えることの類似である)が $(bm(I)-bm(K)_t bm(H))overline(bm(Sigma))_t$ に等しいことを、右から $(overline(bm(Sigma))_t^(-1)+bm(H)^T bm(R)^(-1)bm(H))$ を掛けて確認する。

  $ (bm(I)-bm(K)_t bm(H)) overline(bm(Sigma))_t (overline(bm(Sigma))_t^(-1) + bm(H)^T bm(R)^(-1) bm(H)) \
    = bm(I) + overline(bm(Sigma))_t bm(H)^T bm(R)^(-1) bm(H) - bm(K)_t bm(H) - bm(K)_t bm(H) overline(bm(Sigma))_t bm(H)^T bm(R)^(-1) bm(H) $

  最後の2項を $bm(K)_t$ でくくり、$bm(H) + bm(H)overline(bm(Sigma))_t bm(H)^T bm(R)^(-1)bm(H) = (bm(I)+bm(H)overline(bm(Sigma))_t bm(H)^T bm(R)^(-1))bm(H) = (bm(R)+bm(H)overline(bm(Sigma))_t bm(H)^T)bm(R)^(-1)bm(H)$ と書き換え、さらに $bm(K)_t$ の定義を代入すると

  $ bm(K)_t [bm(H) + bm(H)overline(bm(Sigma))_t bm(H)^T bm(R)^(-1) bm(H)] \
    = overline(bm(Sigma))_t bm(H)^T (bm(H)overline(bm(Sigma))_t bm(H)^T+bm(R))^(-1)(bm(R)+bm(H)overline(bm(Sigma))_t bm(H)^T) bm(R)^(-1) bm(H) = overline(bm(Sigma))_t bm(H)^T bm(R)^(-1) bm(H) $

  （$(bm(H)overline(bm(Sigma))_t bm(H)^T+bm(R))^(-1)(bm(R)+bm(H)overline(bm(Sigma))_t bm(H)^T)=bm(I)$ が打ち消し合う）。これを最初の式に戻すと

  $ (bm(I)-bm(K)_t bm(H)) overline(bm(Sigma))_t (overline(bm(Sigma))_t^(-1) + bm(H)^T bm(R)^(-1) bm(H)) = bm(I) + overline(bm(Sigma))_t bm(H)^T bm(R)^(-1) bm(H) - overline(bm(Sigma))_t bm(H)^T bm(R)^(-1) bm(H) = bm(I) $

  すなわち $(bm(I)-bm(K)_t bm(H))overline(bm(Sigma))_t$ は $(overline(bm(Sigma))_t^(-1)+bm(H)^T bm(R)^(-1)bm(H))$ の逆行列であり、$bm(Sigma)_t = (bm(I)-bm(K)_t bm(H))overline(bm(Sigma))_t$ を得る。
]

#remark[
  カルマンゲイン $bm(K)_t = overline(bm(Sigma))_t bm(H)^T (bm(H) overline(bm(Sigma))_t bm(H)^T + bm(R))^(-1)$ が言っているのは、「予測と観測のどちらをどれだけ信じるか」という重み付けそのものだ。予測の不確かさ $overline(bm(Sigma))_t$ が観測ノイズ $bm(R)$ に比べて大きいほど $bm(K)_t$ は大きくなり、新しい観測を強く信じて事後平均を観測側へ引き寄せる。逆に $overline(bm(Sigma))_t$ が小さければ($bm(K)_t approx bm(0)$)、予測をほぼそのまま信じて観測をあまり反映しない。GPSの測位とスマホの慣性センサーを融合するときも、まさにこの「どちらのセンサーがその瞬間より信頼できるか」を自動で決める重みとして$bm(K)_t$が働く。
]

= 拡張カルマンフィルタ(EKF)について

ここまでは状態遷移・観測が線形の場合を扱った。実際のロボットでは運動モデルや観測モデルが非線形なことが多いので、その拡張(EKF)の考え方を示す。状態遷移・観測が非線形関数 $bm(g), bm(h)$ で与えられる場合(EKF)は、現在の推定平均のまわりで1次のテイラー展開により線形化し、そのヤコビ行列を $bm(A), bm(H)$ の代わりに用いる。すなわち

$ overline(bm(mu))_t = bm(g)(bm(mu)_(t-1)), quad bm(A)_t := (partial bm(g))/(partial bm(x)) bar_(bm(x)=bm(mu)_(t-1)), quad bm(mu)_t = overline(bm(mu))_t + bm(K)_t (bm(z)_t - bm(h)(overline(bm(mu))_t)), quad bm(H)_t := (partial bm(h))/(partial bm(x)) bar_(bm(x)=overline(bm(mu))_t) $

とし、予測・更新ステップの式はそのまま $bm(A) arrow bm(A)_t$, $bm(H) arrow bm(H)_t$ と置き換えて使う。ヤコビ行列による線形化の議論そのものは `01_kinematics` でヤコビ行列を扱った議論と同じであり、EKFの導出は本質的に上の線形カルマンフィルタの証明と同じ手順を毎ステップ異なる線形化点で繰り返すだけなので、ここでは繰り返さない(この省略は和文の参考資料である拡張カルマンフィルタの解説(Thrun, Burgard and Fox, *Probabilistic Robotics*, 2005, 3.3節に基づく)でも同様になされている)。

= 補足: alpha-betaフィルタとの関係

最後に、以前MLの学習で実装した「Kalman Filter alpha-beta」が、ここまで証明した線形カルマンフィルタの特別な場合(時不変な系の定常状態)に過ぎないことを示す。

#remark[
  状態遷移・観測行列 $bm(A), bm(H)$ とノイズ共分散 $bm(Q), bm(R)$ が時刻に依らず一定(時不変)であれば、リカッチ方程式の解 $bm(Sigma)_t$ は時刻とともにある定常値 $bm(Sigma)_infinity$ に収束し、ゲイン $bm(K)_t$ も定常値 $bm(K)_infinity$ に収束する(`02_alpha_beta_steady_state.py` で数値的に確認)。定常状態でこの一定ゲイン $bm(K)_infinity$ をそのまま(毎回リカッチ方程式を解き直さずに)使うフィルタが、等速直線運動モデルにおける#strong[alpha-betaフィルタ] (位置のゲインが $alpha$、速度のゲインが $beta \/ Delta t$)に他ならない。すなわちalpha-betaフィルタは、カルマンフィルタを時不変な線形系の定常状態に限定した特別な場合である。
]
