# Typstワークフロー

数式証明は [Typst](https://typst.app/) で書く。このマシンには mise 経由でインストール済み(`typst --version`で確認)。

```bash
# リポジトリのルートを起点にコンパイル(common/ への相対importのため --root が必要)
typst compile --root . 01_kinematics/notes.typ 01_kinematics/notes.pdf
```

定理・証明環境(`theorem`, `lemma`, `proof`, `definition`, `corollary`, `remark`)は [`common/theorems.typ`](common/theorems.typ) で共通定義しており、`ctheorems` パッケージ(Typst Universe)を使用している。各フェーズの `notes.typ` から次のように読み込む。

```typst
#import "../common/theorems.typ": *
#show: setup-theorems
```

`notes.pdf` はTypstが無いと読めないので、GitHub上でそのまま数式を読めるように `notes.md` も併置する。これは `notes.pdf` の各ページをPNG画像に書き出し、順番に埋め込んだだけのファイルで、内容は `notes.pdf` と同じ(ソースは常に `notes.typ`)。`notes.pdf` 自体はPNG生成用のローカル中間ファイルに過ぎず、`notes.md` と内容が重複するので git 管理しない(`.gitignore` 参照)。新しいフェーズで証明を書いたら、次の手順で `notes.md` も生成する。

```bash
# ページ画像を notes_pages/ に書き出す(--ppi は解像度、150〜180程度で十分読める)
mkdir -p 01_kinematics/notes_pages
typst compile --root . 01_kinematics/notes.typ 01_kinematics/notes_pages/page-{p}.png --format png --ppi 180
```

生成した `page-1.png, page-2.png, ...` を `notes.md` に `![page N](notes_pages/page-N.png)` の形でページ順に埋め込む。

# notes.typの書き方

定理集(Definition/Theorem/Lemma/Proof/Corollary)の形式は維持しつつ、各定理の前後の地の文を厚くする。

- 各Definition/Theoremの直前: 何をこれから定義・証明するのかを、具体例やアナロジーで先に示す(いきなり数式から入らない)
- 各Theorem/Proofの直後: 結果が実際に何を意味するかを平文で解釈する(「この結果が言っているのは〜ということだ」)
- 同じ節の中でLemma→Theorem→Corollaryのように定理が連続する場合も、ボックス同士の間に橋渡しの一文を入れる(前の結果があるから次に何を示すのか)。ボックスを地の文なしに連続させない

絵文字の状態マーカー(✅ 完了など)や「フェーズを完結させる」のようなメタ的な進捗コメントは書かない。
