# 06. 外部ライブラリとの照合・可視化

> MuJoCo(DeepMindが強化学習研究で標準採用)やRerun(自動運転・ロボティクス企業が可視化に使用)で検算するのは、研究・実務で「信頼できる結果」を出すために誰もが通る道。

## 目標

これまでのスクラッチ実装(01_kinematics・02_dynamics)を、確立されたライブラリと突き合わせて検算する。MLでいう「sklearnの結果と照合する」に対応するフェーズ。01〜05とは異なり、ここではスクラッチ実装ではなくライブラリ(MuJoCo, Rerun)を使う側に回る。

## 内容

| ファイル | 内容 |
|---|---|
| `01_mujoco_fk_jacobian.py` | `01_kinematics`と同じ2リンク平面アーム(`l1=1.0, l2=0.8`)をMJCF(MuJoCoのXML形式)で再現し、自作の`fk(q)`・`jac(q)`を、MuJoCoの`mj_forward`・`mj_jacSite`の出力と突き合わせる |
| `02_mujoco_dynamics.py` | `02_dynamics`と全く同じ物理パラメータ(質量・重心位置・慣性モーメント)のアームをMJCFで再現し、自作の質量行列・コリオリ/遠心力項・重力項・逆動力学を、MuJoCoの`mj_fullM`・`mj_rne`の出力と突き合わせる |
| `03_rerun_visualization.py` | `01_kinematics`のFKと`05_planning`の5次時間スケーリングを組み合わせ、2リンクアームが始点から終点まで動く様子と、関節角・角速度の時系列を、Rerunで同じタイムライン上にアニメーションとして記録する(`output/arm_trajectory.rrd`、`rerun output/arm_trajectory.rrd`で再生) |
| [`notes.md`](notes.md) | 01〜05のTypst証明とは違い、このフェーズは数式証明ではなくライブラリ活用が主題なので、証明ではなく実務ガイドとしてまとめている。MJCFの読み方、MuJoCoの主要API(`mj_forward`・`mj_jacSite`・`mj_fullM`・`mj_rne`)の役割、実装中にハマった落とし穴(`qacc`が`mj_forward`で上書きされる、バージョンによる属性名の違いなど)、Rerunがmatplotlibとどう違うかを解説する |

## 確認方法

- `01_mujoco_fk_jacobian.py`: 200個のランダムな関節角で、自作FK・ヤコビ行列とMuJoCoの出力の誤差が`1e-9`未満であることを確認
- `02_mujoco_dynamics.py`: 200個のランダムな関節角・角速度・角加速度で、質量行列(`1e-8`未満)・コリオリ/遠心力項+重力項(`1e-6`未満)・逆動力学(`1e-6`未満)の誤差を確認。`mj_rne`は前進動力学(`mj_forward`)とは別の計算(現在の`qpos`・`qvel`・`qacc`からRNE逆動力学を行うだけ)であることに注意し、`qacc`の設定は`mj_forward`の後に行う
- `03_rerun_visualization.py`: 出力される`.rrd`ファイルが空でないこと、Rerunビューアでアームのアニメーションと関節角・角速度のグラフが同期して表示されること(目視確認)

## 参考文献

- MuJoCo公式ドキュメント([mujoco.readthedocs.io](https://mujoco.readthedocs.io/)) — MJCFのXMLスキーマ、`mj_forward`・`mj_jacSite`・`mj_fullM`・`mj_rne`のAPI
- Rerun公式ドキュメント([rerun.io/docs](https://rerun.io/docs)) — `LineStrips2D`・`Points2D`・`Scalars`アーキタイプ、タイムラインの扱い
