# zero-to-robot

ゼロからロボットを作れるようになるまでの学習リポジトリ。**フロムスクラッチ実装 → 数値検算 → 可視化 → 数式証明** のサイクルを、MLを学ぶときと同じやり方でロボット工学に適用する。

## 進め方

MLで「スクラッチ実装 → 数値で検算 → 可視化」のサイクルを回すのと同じ発想で進める。

| ML | ロボット |
|---|---|
| 線形回帰の正規方程式(逆行列) | 逆運動学を公式(余弦定理)で解く |
| 勾配降下法 | 逆運動学を勾配降下法で解く |
| 勾配チェック(有限差分) | ヤコビアンの有限差分チェック |
| （なし） | 振り子のシミュレーション(エネルギー保存で検算) |
| （なし） | PD制御(誤差が0に収束するか) |
| sklearnの結果と照合 | 物理エンジン(MuJoCo)の結果と照合 |
| matplotlibで損失曲線 | Rerun / Foxgloveで3D+時系列 |
| テストデータで汎化を確認 | 実機で確認(sim2real) |

各フェーズでは、実装(`.py`)・検算(assert/グラフ)に加えて、その裏付けとなる数式証明を **Typst** で書く(`notes.typ` → `notes.pdf`)。証明は添字を使った成分ごとの計算ではなく、なるべく行列・ベクトル形式で行間を省略せずに書く(参考: Craig, *Introduction to Robotics*; Spong et al., *Robot Modeling and Control*; JSKロボティクス研究室の資料の記法)。

## スクラッチ実装とライブラリ実践の境界線

**1〜5(運動学・動力学・制御・状態推定・軌道計画)はフロムスクラッチ、6以降(外部ライブラリとの照合・実機)はライブラリを使った実践**、という切り分けにしている。

これはMLを学んだときの経験に合わせたもの。ML学習時は回帰・PCA・カーネル法・EM/変分ベイズ・バンディット・**Kalman Filter alpha-beta**・NNまで一通りフルスクラッチ(numpyのみ)で実装してから、実務ではsklearn/PyTorchなどのライブラリに移った。ロボットでも同じように、**「アルゴリズムの核」を理解するまではスクラッチ、実機やシミュレーターが絡む「システムとしての実践」からはライブラリ**、という境界線を引く。

- `04_estimation`(カルマンフィルタ)は、ML時代の「Kalman Filter alpha-beta」のスクラッチ実装をそのまま多次元・非線形(EKF)に拡張する位置づけ
- `05_planning`(軌道生成)までは数式で閉じたアルゴリズムなのでスクラッチを継続
- `06_verification_with_libraries`(MuJoCo照合・Rerun可視化)からは、確立された物理エンジン・可視化ツールを実践的に使う側に回る
- `07_hardware_sim2real`(実機)はそもそも自前実装のしようがない領域なので、当然ライブラリ・既存ツール(ROS等)を使う

## ロードマップ / 目次

| # | フォルダ | 内容 | 確認方法 |
|---|---|---|---|
| 1 | [`01_kinematics/`](01_kinematics/) | 順運動学・逆運動学(勾配降下法・公式解)、修正DH規約、冗長マニピュレータの零空間 | FK/IK往復・ヤコビアン有限差分チェック・零空間の射影性質・自己運動の追跡 |
| 2 | [`02_dynamics/`](02_dynamics/) | 振り子シミュレーション、2関節アームの運動方程式(ラグランジュ形式) | エネルギー保存、有限差分によるラグランジュ方程式との一致 |
| 3 | [`03_control/`](03_control/) | PD制御、計算トルク制御(フィードバック線形化) | 誤差の収束、理論(臨界減衰)解との一致 |
| 4 | [`04_estimation/`](04_estimation/) | カルマンフィルタ、alpha-betaフィルタとの関係 | NEES(推定誤差の一貫性)、KF↔alpha-betaフィルタの一致 |
| 5 | [`05_planning/`](05_planning/) | 3次/5次多項式・台形・S字速度プロファイル、カルテシアン空間軌道(スクリュー/分離型)とIK | 境界条件充足、閉じた式との一致、対称性による最小性の検算、IK追従誤差 |
| 6 | [`06_verification_with_libraries/`](06_verification_with_libraries/) | MuJoCoとの照合、Rerunでの可視化 | 自作実装とライブラリの数値一致 |
| 7 | [`07_hardware_sim2real/`](07_hardware_sim2real/) | 実機、AprilTag、キャリブレーション(README に計画だけ書いてある) | シミュレーションと実機の差分 |

各フォルダの `README.md` に、そのフェーズの目標・内容・確認方法・参考文献をまとめている。実務との繋がりだけ一言ずつ:

- **1. 運動学** — 産業用ロボットアームからVRのハンドトラッキング、3DCGのキャラクターアニメーション(IK)まで、「関節角と手先位置を相互に求める」計算は動くものすべての土台 → [`01_kinematics/`](01_kinematics/)
- **2. 動力学** — ヒューマノイドの二足歩行、ゲームエンジンの物理演算まで、「力と運動の関係」のモデルなしにまともな制御もシミュレーションもできない → [`02_dynamics/`](02_dynamics/)
- **3. 制御** — ドローンの姿勢制御、エアコンの温度調整まで、PID/PD制御は世界で最も実際に動いている制御理論 → [`03_control/`](03_control/)
- **4. 状態推定** — GPS・スマホの位置推定、自動運転のセンサーフュージョンまで、カルマンフィルタは「ノイズだらけの観測から真の状態を知る」ための60年選手 → [`04_estimation/`](04_estimation/)
- **5. 軌道計画** — Amazon倉庫ロボットの棚運び、自動運転の車線変更まで、「目標に到達する」ではなく「どう動くか」を決める技術 → [`05_planning/`](05_planning/)
- **6. 外部ライブラリとの照合・可視化** — MuJoCo/Rerunで検算するのは、研究・実務で「信頼できる結果」を出すために誰もが通る道 → [`06_verification_with_libraries/`](06_verification_with_libraries/)
- **7. 実機・sim2real** — シミュレーションで完璧でも実世界では動かない「sim2realギャップ」 → [`07_hardware_sim2real/`](07_hardware_sim2real/)(計画だけ)

## まず触ってみる

証明より先に「ロボットが何をしているか」をつかみたいときは、[`apps/`](apps/) の 17 個のシミュレーターから始める。**全部、インストール不要でブラウザからそのまま動かせる**: [https://kiwamizamurai.github.io/zero-to-robot/](https://kiwamizamurai.github.io/zero-to-robot/)(一覧と進め方は [`apps/README.md`](apps/README.md))。実務のライブラリ(MuJoCo・mink・OpenCV・PyTorch など)を使った Python スクリプトも各フォルダに残してあり、ブラウザ版はそれを MuJoCo の WebAssembly 版などに移したもの。学習系の 2 つ(`panda_imitation`、`rl_balance`)は、事前に学習した結果をブラウザで再生する。

**仕組みをつかむ(ブラウザ)**

- [`apps/tidy_arm/`](apps/tidy_arm/): ブロックを箱に片付けるアームで、逆運動学(01)と軌道計画(05)がどこで使われているかを目で見られる
- [`apps/balance_bot/`](apps/balance_bot/): 車輪 2 つで立つロボットが、倒れないように 1 秒に 100 回車輪を動かす。動力学(02)と制御(03)、センサーの遅れで倒れる sim2real の問題(07)を試せる
- [`apps/delivery_robot/`](apps/delivery_robot/): レストランの配膳ロボットが、自分の位置を推定しながら道を探して走る。状態推定(04)・経路計画(05)・経路追従(03)がそろった自律移動の基本形で、本物では ROS 2 の Nav2 が担う部分

**本物のロボットのモデルとライブラリで(Python)**

- [`apps/panda_pick_place/`](apps/panda_pick_place/): `tidy_arm` と同じ片付けを、実在するアーム Franka Panda のモデルで行う。逆運動学は mink、物理とモーターは MuJoCo に任せる、ライブラリ実践の入り口
- [`apps/panda_rrt/`](apps/panda_rrt/): 壁をよけて腕を動かす。7 次元の関節角度の空間で、ぶつからない道を RRT-Connect で探す
- [`apps/panda_impedance/`](apps/panda_impedance/): ペン先を机に押しつけて円を描く。位置で指令すると机の高さのずれで力が 0 か過大になり、インピーダンス制御(トルク制御)なら力が一定に近く保たれる
- [`apps/drone_flight/`](apps/drone_flight/): 横向きの力を出せないドローンを、機体を傾けて進ませる 3 段のカスケード制御。突風に耐える
- [`apps/panda_vision_pick/`](apps/panda_vision_pick/): 同じ片付けを、天井カメラの画像だけから OpenCV でブロックを見つけて行う
- [`apps/panda_calibration/`](apps/panda_calibration/): カメラの取り付けが数 cm ずれていたらどうなるかを試し、ハンドアイ・キャリブレーションで測り直す
- [`apps/panda_imitation/`](apps/panda_imitation/): 手順書を先生にしてお手本を集め、ネットに真似させる模倣学習
- [`apps/rl_balance/`](apps/rl_balance/): お手本もモデルもなしに、強化学習(PPO)で倒立振子を立たせる
- [`apps/so101_twin/`](apps/so101_twin/): これから買う SO-101 を MuJoCo で動かすデジタルツイン。LeRobot の実機クラスと同じ口を持つので、シミュレーターで書いたコードを実機でそのまま動かせる

**作る前に、全体をシミュレーターで通す(家庭菜園・家の中)**

- [`apps/garden_watering/`](apps/garden_watering/): 土の湿り気を測って、乾いた株にだけ水をやる門型ロボット
- [`apps/tomato_harvest/`](apps/tomato_harvest/): SO-101 がカメラで赤い実だけを見分けて収穫する
- [`apps/robot_vacuum/`](apps/robot_vacuum/): ぶつかったら曲がる・渦巻き・LiDAR で地図を作って往復、の 3 つのやり方を比べる
- [`apps/cat_teaser/`](apps/cat_teaser/): カメラで測った猫の反応から、遊び方を多腕バンディットで学ぶ
- [`apps/cat_laser/`](apps/cat_laser/): レーザーの光が猫の頭の近くを通らないよう、光の通り道を 3 次元で計算する

この 5 つの README には、実物を作るときの部品表(型番・価格の目安・買える店)も書いている。

## 環境構築

### Python

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

各スクリプトはフォルダ単体で実行できる(例: `python 01_kinematics/02_ik_gradient_descent.py`)。出力グラフは各フォルダの `output/` に保存される。

### ブラウザ版のサンプルアプリ(`apps/`)をローカルで動かす

インストールは要らない。リポジトリのルートで次を実行し、`http://localhost:8000/` を開く。

```bash
python3 -m http.server -d apps
```

公開用のサイトを組み立てるときは `bash scripts/build_site.sh`(`site/` に出力、リンク切れも検査する)。ブラウザなしの動作確認は、Node 22 以降で `node apps/<app>/test_sim.mjs`。main に push すると、GitHub Actions が [GitHub Pages](https://kiwamizamurai.github.io/zero-to-robot/) に公開し、CI が各アプリのテスト・サイトのビルド・数式ノート(Typst)のコンパイルを確かめる。

### Typst

数式証明は [Typst](https://typst.app/) で書く(`notes.typ` → `notes.pdf` / `notes.md`)。このマシンには mise 経由でインストール済み。コンパイル手順・執筆ルールは [`CLAUDE.md`](CLAUDE.md) を参照。

## 参考

- PRML(Bishop)をやっていれば、第13章の線形動的システムは**カルマンフィルタそのもの**で、13.3.4節の粒子フィルタもロボットの自己位置推定の定番手法。`04_estimation/` はこの対応を実装で確かめるフェーズ

各フェーズが実際に準拠した文献(Craig, Spong, Lynch and Park, Asada, Thrun, Flash and Hogan, BYU ME 537コース資料など)は、それぞれの `README.md` の「参考文献」節にまとめている。
