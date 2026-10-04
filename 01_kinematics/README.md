# 01. 運動学 (Kinematics)

> 産業用ロボットアームからVRのハンドトラッキング、3DCGのキャラクターアニメーション(IK)まで — 「関節角と手先位置を相互に求める」計算は、動くものすべての土台。

## 目標

- 順運動学(FK)・逆運動学(IK)をフロムスクラッチ(numpyのみ)で実装する
- IKを勾配降下法で解き、MLの回帰・最急降下法との対応を体感する
- 実装したヤコビ行列・勾配・公式解を、行列形式の数式証明(Typst)で裏付ける
- 標準DH規約と修正DH(Craig)規約の対応関係、冗長マニピュレータの擬似逆行列IK・零空間法を証明する

## 内容

| ファイル | 内容 |
|---|---|
| `01_forward_kinematics.py` | 1関節アームのFK/IK。角度→位置→角度、位置→角度→位置の往復が一致するかを確認 |
| `02_ik_gradient_descent.py` | 2関節アームのIKを勾配降下法で解く。ヤコビアンの有限差分チェック、余弦定理による公式解との比較も実施 |
| `03_rotations_3d.py` | 3D回転表現: 回転行列(SO(3))・軸角度(Rodrigues)・クォータニオンの相互変換。$R^\top R=I$・$\det R=1$、クォータニオン積と回転の合成の対応、二重被覆($q$と$-q$が同じ回転)を検算 |
| `04_homogeneous_transforms.py` | 同次変換行列(SE(3)): 回転+並進の合成・逆変換を検算 |
| `05_dh_parameters.py` | DHパラメータ(標準/古典DH規約)による一般化された順運動学。2関節平面アームの結果と一致することを確認し、3自由度の空間アーム例も実装 |
| `06_differential_kinematics_3d.py` | 一般化された幾何ヤコビ行列($z_{i-1}\times(o_n-o_{i-1})$)を有限差分と照合。肘が伸びきる/折れきる配置での特異点(可操作度が0に近づく)を確認 |
| `07_modified_dh.py` | 修正DH(Craig)規約の実装。標準DH表から修正DH表を作る変換規則(添字を1つずらす)を実装し、両者が生成する順運動学が最後のリンクの補正項$T_x(a_n)R_x(\alpha_n)$を除いて厳密に一致することを検算 |
| `08_redundant_ik_nullspace.py` | 冗長マニピュレータ(2次元位置のみを制御する3関節平面アーム、冗長度1)の擬似逆行列IKと零空間法。$JJ^+=I$・零空間への射影・最小ノルム性を検算し、零空間で可操作度を最大化しつつ主タスク(手先位置)を維持できることを確認。同じ目標位置に到達する複数姿勢の連続族(自己運動)も追跡・可視化 |
| `notes.typ` / [`notes.md`](notes.md) / `notes.pdf` | `01`〜`08`すべての数式的裏付け(全19ページ)。前半(1-5節): 平面回転行列 $R(\theta)$ の性質から出発し、2関節平面アームのヤコビ行列・勾配降下法の勾配・公式解を行列形式で証明する。中盤(6-10節): skew行列の性質→Rodriguesの回転公式→クォータニオン→一般化された幾何ヤコビ行列($n$関節・3次元)→特異点、まで証明する。後半(11-12節): 標準DH規約と修正DH規約の厳密な対応関係(帰納法による証明)、擬似逆行列$J^+$の性質(射影行列・最小ノルム性・零空間法)を証明する（`typst compile --root . notes.typ` でPDF生成、`common/theorems.typ` の定理環境を使用。`notes.md` はPDFのページ画像を埋め込んだGitHub閲覧用） |

## 確認方法

- `01_forward_kinematics.py`: 200サンプルのランダムな角度・位置で往復チェックが全てOKになるか(assert)
- `02_ik_gradient_descent.py`:
  - ヤコビアンの解析式が有限差分と一致するか(勾配チェック)
  - 損失曲線が単調に減少するか(`output/02_ik_loss.png`)
  - 勾配降下法の解と公式解(余弦定理)が同じ目標位置に到達するか
- `03_rotations_3d.py` / `04_homogeneous_transforms.py` / `05_dh_parameters.py` / `06_differential_kinematics_3d.py`: 各スクリプトのassertを参照(往復変換・群の性質・有限差分との一致など)
- `07_modified_dh.py`: 標準DH↔修正DHの変換公式の誤差が$10^{-16}$程度、補正項なしでは一致しない(食い違い$0.52$)ことを確認
- `08_redundant_ik_nullspace.py`: $JJ^+=I$・$P$の冪等性/零空間性の誤差が$10^{-14}$以下、最小ノルム性をランダムな別解と比較、零空間法で可操作度が改善($2.058\to2.115$)しつつ主タスクの収束に影響しないこと、自己運動多様体を9点以上追跡できることを確認
- `notes.typ`: 各定理の証明がコード中のどの関数に対応するかをコメントで明記。証明と実装が食い違っていないかを突き合わせる

## 参考文献

- Lynch and Park, *Modern Robotics: Mechanics, Planning, and Control* (Cambridge University Press, 2017) — クォータニオンの定義・変換式(Appendix B.3)、DHパラメータ(Appendix C.1、Craigの修正DH規約と同じ形)
- Asada, *Introduction to Robotics* (MIT 2.12, Fall 2005, OCW) 第5章 "Differential Motion" — ヤコビ行列の幾何学的解釈(各列は他の関節を固定したときの手先速度)、特異点の解析(肘が伸びきる/折れきる配置)
- Buss, "Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares methods" (CMU, 2009) — 擬似逆行列$J^+=J^T(JJ^T)^{-1}$と零空間法$\dot\theta=J^+e+(I-J^+J)\phi$の式(7)(9)。同資料が証明を省略している性質(Pの冪等性・零空間性、最小ノルム性)は自前で証明した。零空間法自体の初出はLiegeois (1977)
