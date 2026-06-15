#!/usr/bin/env bash
# Compile the 10-page report (EN + FR) with XeLaTeX. Includes clean.
#   bash compile_rapport10p.sh         # builds both rapport10p_EN and rapport10p_FR
#   bash compile_rapport10p.sh EN      # builds only one
set -uo pipefail
cd "$(dirname "$0")"

clean() {
  rm -f ./*.aux ./*.log ./*.out ./*.toc ./*.lof ./*.lot ./*.nav ./*.snm \
        ./*.vrb ./*.bbl ./*.bcf ./*.blg ./*.run.xml ./*.fls ./*.fdb_latexmk 2>/dev/null || true
}

build() {
  local doc="$1"
  [ -f "$doc.tex" ] || { echo "[skip] $doc.tex absent"; return 0; }
  echo "=== build $doc ==="
  # two passes for ToC / \cite cross-references
  xelatex -interaction=nonstopmode -halt-on-error "$doc.tex" >/tmp/${doc}_pass1.log 2>&1
  local rc=$?
  xelatex -interaction=nonstopmode -halt-on-error "$doc.tex" >/tmp/${doc}_pass2.log 2>&1
  rc=$?
  if [ "$rc" -ne 0 ] || [ ! -f "$doc.pdf" ]; then
    echo "[FAIL] $doc — voir /tmp/${doc}_pass2.log"
    grep -nE "^!|Error|Undefined|Missing" "/tmp/${doc}_pass2.log" | head -20
    return 1
  fi
  local pages
  pages=$(pdfinfo "$doc.pdf" 2>/dev/null | awk '/^Pages:/{print $2}')
  echo "[OK] $doc.pdf — $pages pages"
}

clean
status=0
case "${1:-ALL}" in
  EN) build rapport10p_EN || status=1 ;;
  FR) build rapport10p_FR || status=1 ;;
  *)  build rapport10p_EN || status=1; build rapport10p_FR || status=1 ;;
esac
clean
exit "$status"
