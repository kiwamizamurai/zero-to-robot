# 本物のロボットアームでお片付け(Franka Panda × MuJoCo × mink)

[ブラウザでシミュレーションを動かす](https://kiwamizamurai.github.io/zero-to-robot/panda_pick_place/)

[`../tidy_arm/`](../tidy_arm/) のブラウザ版と同じ「ブロックを箱に片付ける」作業を、実在するロボットアーム **Franka Panda** のモデルと、実務で使うライブラリだけで組み立てたもの。自作したのは作業の段取りだけで、逆運動学・物理・モーター制御はすべてライブラリに任せている。

## このアプリで学べること

- **実在するロボットのモデルを使う**: MuJoCo Menagerie の Franka Panda(7 関節 + 2 本指)を 1 行で読み込む
- **ライブラリで逆運動学を解く**: mink に「手先をこの位置・この向きへ」を渡すだけで、7 つの関節角度が決まる。余った 1 自由度の使い方も指定できる
- **物理エンジンでつかむ**: 指とブロックの摩擦だけで持ち上げる。条件が悪いと本当にすべり落ちる
- **位置サーボの使い方**: 関節に目標角度を渡すだけで、モーター制御はシミュレーターの中のサーボがやる

**キーワード**: MuJoCo、MuJoCo Menagerie、MJCF、MjSpec、mink、微分 IK、冗長マニピュレータ、零空間、位置サーボ、把持、摩擦、物理シミュレーション、robot_descriptions

## 動かし方

**ブラウザ(おすすめ)**: [https://kiwamizamurai.github.io/zero-to-robot/panda_pick_place/](https://kiwamizamurai.github.io/zero-to-robot/panda_pick_place/)。手元で動かすときは、リポジトリのルートで `python3 -m http.server -d apps` を実行して、`http://localhost:8000/panda_pick_place/` を開く(ES モジュールを読み込むので、ファイルを直接開く方法では動かない)。

**Python 版(元の実装)**: 次のとおり。

```bash
pip install -r apps/panda_pick_place/requirements.txt
python apps/panda_pick_place/pick_place.py            # output/pick_place.mp4 と .gif を書き出す
python apps/panda_pick_place/pick_place.py --viewer   # 3D ビューアでその場で見る(macOS は mjpython で実行)
```

初回だけ、`robot_descriptions` が MuJoCo Menagerie(ロボットモデル集)を `~/.cache/robot_descriptions/` にダウンロードする。実行すると、各ブロックを持ち上げられたか、最後に箱の中にあるかを表示する(1 つでも箱に入らなければ `assert` で止まる)。

## 動画の要所(Python 版が書き出すもの)

動画と図は、Python 版のスクリプトを動かすと `output/` に書き出される(リポジトリが重くならないよう、コミットしていない)。ブラウザ版では、同じ様子を画面でそのまま見られる。

| コマ | 何をしているか | ブラウザ版のステップ |
|---|---|---|
| 左上 | 赤いブロックの真上へ手を運ぶ。7 つの関節を mink が同時に動かしている | ② 角度を計算 → ③ 動かす |
| 右上 | 真下に降りて、指を閉じる。ブロックは指の**摩擦だけ**で持ち上がる | ④ つかむ |
| 左下 | 持ち上げて、箱の上へ運ぶ | ⑤ 運ぶ |
| 右下 | 3 つとも箱に入ったところ。はなしたブロックは物理計算で箱に落ちる | ⑥ はなす |

ブラウザ版との一番の違いは右上のコマ。ブラウザ版は「指の間に入ったらくっつける」だけだったが、こちらは MuJoCo が指とブロックの接触・摩擦を計算するので、指の位置がずれたり力が弱かったりすると、本当にすべり落ちる。実行すると、ブロックごとに「持ち上げ成功 / 失敗」と最後に箱の中にあるかを表示する。

## ブラウザ版との対応

| ステップ | ブラウザ版(tidy_arm)| このスクリプト |
|---|---|---|
| ロボット | 2 関節の平面アーム(自作) | Franka Panda(7 関節 + 2 本指)。`robot_descriptions` で Menagerie のモデルを読む |
| 場面 | 自作の描画 | `mujoco.MjSpec` で Menagerie の場面にブロック・箱・カメラを足して `compile()` |
| 1 見つける | 位置を最初から知っている | 同じ(`data.body(...).xpos` をシミュレーターから読む) |
| 2 角度を計算 | 余弦定理で解く(01) | `mink.FrameTask` に「手先をこの位置・この向きへ」を渡し、`mink.solve_ik` が関節速度を返す |
| 3 動かす | 5 次多項式(05) | 同じ 5 次多項式で手先の目標を動かし、IK の結果を `data.ctrl` に入れる。関節のモーター制御(03)は MuJoCo の位置サーボがやる |
| 4 つかむ | 指の間に入ったらくっつける | 指を閉じて**摩擦だけ**で持つ。`mujoco.mj_step` が接触と摩擦を計算するので、条件が悪いとすべり落ちる |
| 5・6 運ぶ・はなす | 同じ | 同じ。はなしたブロックは物理で箱に落ちる |

## ライブラリの使い方の要点

- **mink は「手先の目標」を並べて解く**: `FrameTask`(手先の位置・向き)と `PostureTask`(ホーム姿勢に近く)を同時に渡している。Panda は 7 関節で、手先の位置・向き(6 つ)より 1 つ多い。余った 1 つの自由度をどう使うかを `PostureTask` が決めている。01 で扱った冗長マニピュレータの零空間を、ライブラリではこう書く
- **`solve_ik` は角度ではなく速度を返す**: 返ってきた関節速度を `configuration.integrate_inplace(vel, dt)` で積分して、次の関節角度にする。1 回で解ききらず、制御周期ごとに少しずつ目標へ寄せる「微分 IK」
- **Panda のモーターは位置サーボ**: Menagerie の Panda は関節ごとに PD 制御の位置アクチュエータを持つので、`data.ctrl[:7]` に目標角度を入れるだけで動く。指(`ctrl[7]`)は 0(閉じる)〜 255(開く)
- **手先の目印は自分で足す**: Menagerie の Panda には指先の目印がないので、`hand` の 0.1034 m 先に `tcp` という site を足して、IK の対象にしている
- **キーフレームはロボット分だけ使う**: Menagerie の `home` キーフレームはロボットの 9 関節分しかない。ブロックを足したモデルでそのまま `mj_resetDataKeyframe` を使うとブロックの位置まで上書きされるので、先頭 9 個だけコピーしている

## 次の一歩

- ブロックの位置をシミュレーターから読むのをやめ、カメラ画像から見つける(`mujoco.Renderer` で撮った画像を OpenCV で処理する)
- 同じことを実機やほかのロボットでやる。Menagerie には UR5e・Kinova・xArm などもあり、`robot_descriptions` の名前を変えるだけで読める

## ブラウザ版と Python 版の違い

- 物理(MuJoCo)・場面・手順・5 次多項式は Python 版と同じ。逆運動学だけ、mink ではなく自前の減衰つき最小二乗法(手先の位置と向きを目標へ、余った自由度はホーム姿勢へ弱く引く)に置き換えた。
- Panda のモデルは、衝突用のメッシュだけに絞った軽量版(約 0.24 MB)を使う。見た目は元より角ばる。
- `node apps/panda_pick_place/test_sim.mjs` で、ブラウザなしでも「3 つのブロックが箱に入る」ことを確かめられる(CI でも走る)。
