// 全トピック共通の定理・証明環境。
// 各フェーズの notes.typ から `#import "../common/theorems.typ": *` で読み込む。
#import "@preview/ctheorems:1.1.3": *

#let setup-theorems(body) = {
  show: thmrules
  body
}

#let definition = thmbox("definition", "Definition", fill: rgb("#e8f4fd"))
#let theorem = thmbox("theorem", "Theorem", fill: rgb("#fdf2e8"))
#let lemma = thmbox("lemma", "Lemma", fill: rgb("#fdf2e8"))
#let corollary = thmbox("corollary", "Corollary", fill: rgb("#fdf2e8"))
#let remark = thmplain("remark", "Remark").with(numbering: none)
#let proof = thmproof("proof", "Proof")

// ベクトル・行列を太字で表す略記 (JSKロボティクス資料・Craig "Introduction to
// Robotics" などの記法に合わせ、太字 = ベクトル/行列、非太字 = スカラー)
#let bm(x) = math.bold(x)
