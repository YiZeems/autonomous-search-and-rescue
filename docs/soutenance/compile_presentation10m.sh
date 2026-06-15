#!/usr/bin/env bash
# Compile the 10-minute Beamer presentation (EN + FR) with XeLaTeX. Includes clean.
#   bash compile_presentation10m.sh        # builds both presentation10m and presentation10m_FR
#   bash compile_presentation10m.sh EN     # builds only one
set -uo pipefail
cd "$(dirname "$0")"

clean() {
  rm -f ./*.aux ./*.log ./*.out ./*.toc ./*.nav ./*.snm ./*.vrb \
        ./*.bbl ./*.bcf ./*.blg ./*.run.xml ./*.fls ./*.fdb_latexmk 2>/dev/null || true
}

build() {
  local doc="$1"
  [ -f "$doc.tex" ] || { echo "[skip] $doc.tex absent"; return 0; }
  echo "=== build $doc ==="
  xelatex -interaction=nonstopmode -halt-on-error "$doc.tex" >/tmp/${doc}_pass1.log 2>&1
  xelatex -interaction=nonstopmode -halt-on-error "$doc.tex" >/tmp/${doc}_pass2.log 2>&1
  local rc=$?
  if [ "$rc" -ne 0 ] || [ ! -f "$doc.pdf" ]; then
    echo "[FAIL] $doc — voir /tmp/${doc}_pass2.log"
    grep -nE "^!|Error|Undefined|Missing" "/tmp/${doc}_pass2.log" | head -20
    return 1
  fi
  local pages
  pages=$(pdfinfo "$doc.pdf" 2>/dev/null | awk '/^Pages:/{print $2}')
  echo "[OK] $doc.pdf — $pages slides"
}

clean
status=0
case "${1:-ALL}" in
  EN) build presentation10m || status=1 ;;
  FR) build presentation10m_FR || status=1 ;;
  *)  build presentation10m || status=1; build presentation10m_FR || status=1 ;;
esac
clean
exit "$status"
