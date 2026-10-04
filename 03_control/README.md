# 03. 制御 (Control)

> ドローンの姿勢制御、エアコンの温度調整、自動運転のステアリングまで — PID/PD制御は世界で最も実際に動いている制御理論。

## 目標

- フィードバック制御(PD制御)で振り子を目標角度に収束させる
- モデル(物理の式)を知っていることが制御の質を上げることを体感する(重力補償)
- 多関節アーム・時間変化する目標軌道へ一般化する(計算トルク制御/フィードバック線形化)

## 内容

| ファイル | 内容 |
|---|---|
| `01_pd_control.py` | PD制御で振り子を目標角度(真横, $\theta^*=0$)に収束させる。重力補償の有無で収束先がどう変わるかを比較 |
| `02_computed_torque_control.py` | `02_dynamics`と同じ2関節平面アームで、計算トルク制御(フィードバック線形化)により時間変化する目標軌道への追従を実現。閉ループ誤差が理論(臨界減衰)解と一致するか、モデル化誤差がある場合にズレが生じるかを検算 |
| `notes.typ` / [`notes.md`](notes.md) / `notes.pdf` | `02_computed_torque_control.py` の数式的裏付け。計算トルク制御則→線形な閉ループ誤差ダイナミクス→臨界減衰の閉形式解、まで行間なく証明する(Lynch and Park, *Modern Robotics* 11.2.2.3節に基づく) |

## 確認方法

- `01_pd_control.py`: 誤差(`target - theta`)のグラフが0に近づくか(`output/01_pd_error.png`)。重力補償なしでは定常偏差が残り、補償ありでは誤差がほぼ0に収束することを確認
- `02_computed_torque_control.py`: モデルが正確な場合、シミュレーションした追従誤差が理論(臨界減衰)解と一致するか(`output/02_computed_torque_error.png`)。モデル化誤差がある場合、理論解からのズレが明確に大きくなるか
- `notes.typ`: 各定理の証明がコード中のどの関数に対応するかをコメントで明記

## 扱っていないこと(発展課題)

- LQR(線形二次レギュレータ)入門(参考: Slotine and Li, *Applied Nonlinear Control*, 1991、または Åström and Murray, *Feedback Systems*)
- 軌道生成(`05_planning`)と組み合わせた、より実践的な軌道追従

## 参考文献

- Lynch and Park, *Modern Robotics: Mechanics, Planning, and Control* (Cambridge University Press, 2017) 第11章 "Robot Control" — 計算トルク制御(フィードフォワード+フィードバック線形化)の導出(11.2.2.3節)、モデル化誤差による性能劣化の議論(Figure 11.10)
