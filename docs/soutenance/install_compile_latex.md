# Installing the LaTeX toolchain to compile the report & presentation (Ubuntu 22.04)

The four documents in this folder are compiled with **XeLaTeX**:

- `rapport10p_EN.tex` / `rapport10p_FR.tex` — 10-page report (EN / FR)
- `presentation10m.tex` / `presentation10m_FR.tex` — Beamer presentation (EN / FR)

They use `fontspec` (XeLaTeX), `tikz`/`pgf` (all figures), `beamer` (the slides),
`amsmath`, `graphicx`, `subcaption`, `caption`, `booktabs`, `enumitem`, `titlesec`,
`hyperref`, `geometry`, and the **Latin Modern** fonts (the default for `fontspec`, so
no extra font install is needed).

## 1. Required apt packages

```bash
sudo apt update
sudo apt install -y \
  texlive-xetex \
  texlive-latex-base \
  texlive-latex-recommended \
  texlive-latex-extra \
  texlive-pictures \
  texlive-fonts-recommended \
  texlive-fonts-extra \
  texlive-science \
  lmodern \
  fontconfig
```

What each package provides (so the set is minimal but complete):

| Package | Provides (used here) |
|---|---|
| `texlive-xetex` | the **`xelatex`** engine + `fontspec` |
| `texlive-latex-base` | `article`, `amsmath`, `graphicx`, `geometry` (core) |
| `texlive-latex-recommended` | **`beamer`**, `caption`, `hyperref`, `booktabs` |
| `texlive-latex-extra` | `tikz` extras, `subcaption`, `enumitem`, `titlesec` |
| `texlive-pictures` | **`pgf` / `tikz`** (all the diagrams) |
| `texlive-fonts-recommended` + `lmodern` | **Latin Modern** fonts (`fontspec` default) |
| `texlive-fonts-extra` | extra fonts (safety; optional) |
| `texlive-science` | maths support (safety; optional) |
| `fontconfig` | font lookup for XeLaTeX/`fontspec` |

> Alternative (simplest but ~5 GB): `sudo apt install -y texlive-full` installs everything.
> The targeted list above is ~1–1.5 GB and sufficient.

## 2. Optional — only to *preview/verify* the PDFs (not needed to compile)

```bash
sudo apt install -y poppler-utils imagemagick
# poppler-utils: pdfinfo (page count), pdftoppm (PDF -> PNG)
# imagemagick:   montage (assemble page thumbnails)
```

## 3. Compile

The two scripts run XeLaTeX twice (for the table of contents / `\cite` references) and
**clean** the auxiliary files automatically.

```bash
cd docs/soutenance

# report — builds both EN and FR (or pass EN / FR for one)
bash compile_rapport10p.sh          # -> rapport10p_EN.pdf, rapport10p_FR.pdf
bash compile_rapport10p.sh EN       # -> rapport10p_EN.pdf only

# presentation — builds both EN and FR
bash compile_presentation10m.sh     # -> presentation10m.pdf, presentation10m_FR.pdf
```

Each script prints `[OK] <doc>.pdf — N pages` on success, or `[FAIL]` with the first
errors (full log under `/tmp/<doc>_pass2.log`).

## 4. Manual compile (if you prefer)

```bash
xelatex -interaction=nonstopmode rapport10p_EN.tex
xelatex -interaction=nonstopmode rapport10p_EN.tex   # 2nd pass for references
```

## 5. Notes

- **XeLaTeX is required** (not pdfLaTeX): the documents load `fontspec`.
- Figures live in `img/` (RViz key-moment frames + result figures); they are referenced
  with relative paths, so always compile from inside `docs/soutenance/`.
- No internet or `biber` is needed: references use a plain `thebibliography`.
- Tested on Ubuntu 22.04 with TeX Live 2022.
