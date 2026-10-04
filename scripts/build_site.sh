#!/usr/bin/env bash
# GitHub Pages に載せる静的サイトを site/ に組み立てる。
# apps/<app>/index.html を持つアプリを全部集め、共通部品 apps/_web/ も一緒に置く。
# ソースの配置(apps/<app>/ と apps/_web/)と配信物(site/<app>/ と site/_web/)で相対パスが同じなので、
# どちらからでも同じように開ける(サブパス /zero-to-robot/ の配下でも動く)。
set -euo pipefail
cd "$(dirname "$0")/.."

rm -rf site
mkdir -p site
cp apps/index.html site/
rsync -a --exclude '*.py' --exclude '*.md' --exclude 'requirements.txt' --exclude '__pycache__' --exclude '*.mjs' \
  --exclude 'node_modules' apps/_web/ site/_web/
for dir in apps/*/; do
  app=$(basename "$dir")
  [ "$app" = "_web" ] && continue
  [ -f "$dir/index.html" ] || continue
  rsync -a --exclude '*.py' --exclude '*.md' --exclude 'requirements.txt' --exclude '__pycache__' --exclude '*.mjs' \
    --exclude 'output' --exclude 'images' --exclude 'tools' "$dir" "site/$app/"
done
touch site/.nojekyll

# 相対リンク(href/src)と import の行き先が存在することを確認する
fail=0
while IFS= read -r file; do
  dir=$(dirname "$file")
  while IFS= read -r ref; do
    path="${ref%%[#?]*}"
    [ -z "$path" ] && continue
    if [ ! -e "$dir/$path" ]; then
      echo "リンク切れ: $file -> $ref" >&2
      fail=1
    fi
  done < <(
    { grep -oE '(href|src)="[^"]+"' "$file" | sed -E 's/^(href|src)="//; s/"$//'
      grep -oE "from \"\.{1,2}/[^\"]+\"|from '\.{1,2}/[^']+'|import\(\"\.{1,2}/[^\"]+\"\)" "$file" | grep -oE '\.{1,2}/[^"'"'"']+'
      grep -oE '"(three|three/addons/)": "[^"]+"' "$file" | sed -E 's/.*: "//; s/"$//'
    } | grep -vE '^(https?:|//|data:|mailto:|#|\$|\{)' || true)
done < <(find site \( -name '*.html' -o -name '*.js' \) -not -path 'site/_web/vendor/*')
exit $fail
