#!/usr/bin/env bash
# CI 用: 変更ファイルがすべて docs/** または *.md なら "docs_only=true"、それ以外は "docs_only=false" を標準出力に出す。
# 使い方: tools/ci_docs_only.sh <base-sha> <head-sha>
# 安全側: 引数が空、merge base が 1 つでない、差分の取得に失敗、変更ファイルが 0 件のときは false(= 全部実行)。
# ファイル名は NUL 区切りで読むので、空白や改行を含んでも誤判定しない。リネームは旧名・新名の両方を見る。
set -u
base="${1:-}"
head="${2:-}"
if [[ -z "$base" || -z "$head" ]]; then
  echo "docs_only=false"; exit 0
fi
# merge base がちょうど 1 つでなければ(複数・0 個・取得失敗)三点差分が不定なので false。
mb="$(git merge-base --all "$base" "$head" 2>/dev/null)" || { echo "docs_only=false"; exit 0; }
if [[ -z "$mb" || "$(printf '%s\n' "$mb" | wc -l | tr -d ' ')" != "1" ]]; then
  echo "docs_only=false"; exit 0
fi
tmp="$(mktemp)" || { echo "docs_only=false"; exit 0; }
trap 'rm -f "$tmp"' EXIT
if ! git diff --name-only --no-renames -z "$base...$head" > "$tmp" 2>/dev/null; then
  echo "docs_only=false"; exit 0
fi
n=0
while IFS= read -r -d '' f; do
  n=$((n + 1))
  case "$f" in
    docs/* | *.md) ;;
    *) echo "docs_only=false"; exit 0 ;;
  esac
done < "$tmp"
if [[ $n -eq 0 ]]; then
  echo "docs_only=false"
else
  echo "docs_only=true"
fi
