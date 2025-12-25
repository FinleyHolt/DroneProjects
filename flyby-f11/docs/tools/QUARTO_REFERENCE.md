# Quarto Documentation Reference

## Overview

Quarto is an open-source scientific and technical publishing system built on Pandoc. It enables creation of dynamic content with Python, R, Julia, and Observable, producing high-quality articles, reports, presentations, websites, and books.

**Official Website**: https://quarto.org/

## Installation

### Linux Installation

```bash
# Download latest release (check https://github.com/quarto-dev/quarto-cli/releases)
wget https://github.com/quarto-dev/quarto-cli/releases/download/v1.5.40/quarto-1.5.40-linux-amd64.deb
sudo dpkg -i quarto-1.5.40-linux-amd64.deb

# Verify installation
quarto --version
```

### Dependencies for PDF Output

```bash
# Install TinyTeX (lightweight LaTeX)
quarto install tinytex

# Or use system LaTeX (full installation)
sudo apt install texlive-full

# For Ubuntu/Debian minimal setup
sudo apt install texlive-latex-base texlive-latex-extra texlive-fonts-recommended
```

## Basic Usage

### Creating a Quarto Document

Create a `.qmd` file with YAML frontmatter and content:

```markdown
---
title: "My Document"
author: "Your Name"
format: pdf
---

# Introduction

This is a Quarto document.

## Code Example

```{python}
print("Hello from Python!")
```
```

### Rendering Documents

```bash
# Render to PDF
quarto render document.qmd

# Render to HTML
quarto render document.qmd --to html

# Render to multiple formats
quarto render document.qmd --to pdf,html,docx

# Render entire directory
quarto render .

# Watch mode (auto-rebuild on save)
quarto preview document.qmd
```

## YAML Frontmatter Configuration

### PDF Format Options

```yaml
---
title: "Document Title"
author: "Author Name"
date: today
format:
  pdf:
    # Document class
    documentclass: article  # article, report, book, scrartcl, etc.

    # Page geometry
    geometry:
      - margin=1in
      - paperwidth=8.5in
      - paperheight=11in

    # Font settings
    fontsize: 11pt
    mainfont: "Latin Modern Roman"
    monofont: "Courier New"

    # Table of contents
    toc: true
    toc-depth: 3
    toc-title: "Contents"
    lof: true  # List of figures
    lot: true  # List of tables

    # Section numbering
    number-sections: true
    number-depth: 3

    # Code blocks
    highlight-style: github
    code-line-numbers: true

    # Bibliography
    bibliography: references.bib
    csl: ieee.csl

    # Cross-references
    crossref:
      fig-prefix: "Figure"
      tbl-prefix: "Table"
      eq-prefix: "Equation"

    # PDF engine
    pdf-engine: pdflatex  # or xelatex, lualatex

    # Keep intermediate files
    keep-tex: true
---
```

### Custom LaTeX Headers

```yaml
---
title: "Document with Custom Headers"
format:
  pdf:
    include-in-header:
      text: |
        \usepackage{fancyhdr}
        \pagestyle{fancy}
        \fancyhf{}
        \fancyhead[L]{\textit{Document Title}}
        \fancyhead[R]{\thepage}
        \renewcommand{\headrulewidth}{0.4pt}
        \usepackage{graphicx}
        \usepackage{amsmath}
        \usepackage{hyperref}
---
```

## flyby-f11 Documentation Template

### Standard Template Used in Project

This is the template used in `APPROACH.qmd`, `ONTOLOGY_FOUNDATION.qmd`, etc.:

```yaml
---
title: "Document Title"
author: "Finley Holt"
date: today
format:
  pdf:
    documentclass: article
    geometry:
      - margin=1in
    fontsize: 11pt
    number-sections: true
    toc: true
    include-in-header:
      text: |
        \usepackage{fancyhdr}
        \pagestyle{fancy}
        \fancyhf{}
        \fancyhead[L]{\textit{Short Document Title}}
        \fancyhead[R]{\thepage}
        \renewcommand{\headrulewidth}{0.4pt}
---

# Introduction

Document content here...

## Subsection

More content...
```

### Rendering Project Documentation

```bash
# Render single document
quarto render /home/finley/Github/DroneProjects/flyby-f11/APPROACH.qmd

# Render all .qmd files in directory
cd /home/finley/Github/DroneProjects/flyby-f11
quarto render *.qmd

# Render literature review documents
cd /home/finley/Github/DroneProjects/flyby-f11/literature_review
quarto render .
```

## Advanced Features

### Code Execution

```markdown
## Python Code

```{python}
#| label: fig-plot
#| fig-cap: "Sample Plot"

import matplotlib.pyplot as plt
import numpy as np

x = np.linspace(0, 10, 100)
y = np.sin(x)
plt.plot(x, y)
plt.show()
```

## R Code

```{r}
#| echo: false
summary(cars)
```
```

### Cell Options (Code Chunk Options)

```markdown
```{python}
#| label: fig-example
#| fig-cap: "Figure caption"
#| echo: true
#| warning: false
#| message: false
#| eval: true
#| output: true

# Code here
```
```

Common options:
- `label`: Unique identifier for cross-references
- `echo`: Show code (true/false)
- `eval`: Execute code (true/false)
- `warning`: Show warnings (true/false)
- `message`: Show messages (true/false)
- `output`: Show output (true/false)
- `fig-cap`: Figure caption
- `fig-width`: Figure width in inches
- `fig-height`: Figure height in inches

### Cross-References

```markdown
See @fig-example for details.

![Caption](image.png){#fig-example}

As shown in @tbl-results, the accuracy improved.

| Col1 | Col2 |
|------|------|
| A    | B    |

: Results {#tbl-results}
```

### Citations and Bibliography

```yaml
---
bibliography: references.bib
csl: ieee.csl  # Citation style
---
```

In text:
```markdown
According to @smith2020, autonomous systems require...

Multiple citations [@smith2020; @jones2021].
```

BibTeX file (`references.bib`):
```bibtex
@article{smith2020,
  title={Autonomous Navigation},
  author={Smith, John},
  journal={Robotics Journal},
  year={2020}
}
```

### Math Equations

```markdown
Inline math: $E = mc^2$

Display math:
$$
\nabla \times \mathbf{E} = -\frac{\partial \mathbf{B}}{\partial t}
$$ {#eq-maxwell}

Reference equation: See @eq-maxwell.
```

### Callout Blocks

```markdown
::: {.callout-note}
This is a note callout.
:::

::: {.callout-warning}
This is a warning.
:::

::: {.callout-important}
This is important information.
:::

::: {.callout-tip}
This is a helpful tip.
:::

::: {.callout-caution}
Proceed with caution.
:::
```

### Tabbed Content

```markdown
::: {.panel-tabset}

## Tab 1
Content for tab 1.

## Tab 2
Content for tab 2.

:::
```

### Conditional Content

```markdown
::: {.content-visible when-format="pdf"}
This content only appears in PDF output.
:::

::: {.content-visible when-format="html"}
This content only appears in HTML output.
:::
```

## Multiple Output Formats

```yaml
---
title: "Multi-format Document"
format:
  html:
    toc: true
    code-fold: true
    theme: cosmo
  pdf:
    documentclass: article
    toc: true
  docx:
    toc: true
---
```

Render all formats:
```bash
quarto render document.qmd
```

Render specific format:
```bash
quarto render document.qmd --to pdf
```

## Project Configuration

Create `_quarto.yml` for project-wide settings:

```yaml
project:
  type: default
  output-dir: _output

format:
  pdf:
    documentclass: article
    fontsize: 11pt
    geometry:
      - margin=1in
    toc: true
    number-sections: true
    include-in-header:
      text: |
        \usepackage{fancyhdr}
        \pagestyle{fancy}

execute:
  echo: true
  warning: false
  message: false
```

Place in project root:
```
flyby-f11/
├── _quarto.yml
├── APPROACH.qmd
├── ONTOLOGY_FOUNDATION.qmd
└── literature_review/
    ├── paper1.qmd
    └── paper2.qmd
```

## PDF-Specific Customizations

### Custom LaTeX Preamble

```yaml
---
format:
  pdf:
    include-in-header:
      file: preamble.tex
---
```

`preamble.tex`:
```latex
\usepackage{fancyhdr}
\usepackage{graphicx}
\usepackage{amsmath}
\usepackage{booktabs}
\usepackage{longtable}

% Custom commands
\newcommand{\todo}[1]{\textcolor{red}{TODO: #1}}
```

### Page Breaks

```markdown
# Section 1

Content here.

\newpage

# Section 2

More content on new page.
```

### Landscape Pages

```markdown
\begin{landscape}

![Wide figure](wide_figure.png)

\end{landscape}
```

### Custom Theorem Environments

```yaml
---
format:
  pdf:
    include-in-header:
      text: |
        \usepackage{amsthm}
        \newtheorem{theorem}{Theorem}
        \newtheorem{lemma}{Lemma}
---
```

In document:
```markdown
::: {.theorem}
For all autonomous systems...
:::
```

## Parameterized Documents

```yaml
---
title: "Report for {{< meta params.robot >}}"
params:
  robot: "flyby-f11"
  mission: "search"
---
```

Render with parameters:
```bash
quarto render report.qmd -P robot:project-drone -P mission:mapping
```

## Publishing and Deployment

### GitHub Pages

```bash
# Render for web
quarto publish gh-pages

# Or manually
quarto render --to html
# Push _site/ to gh-pages branch
```

### Quarto Pub

```bash
quarto publish quarto-pub
```

## Troubleshooting

### LaTeX Errors

```bash
# Check LaTeX log
quarto render document.qmd --verbose

# Keep intermediate files for debugging
quarto render document.qmd --keep-tex
```

### Missing Packages

```bash
# TinyTeX: Install missing packages
quarto install tinytex
tlmgr install <package-name>

# System LaTeX
sudo apt install texlive-<package>
```

### Font Issues

```yaml
---
format:
  pdf:
    mainfont: "DejaVu Serif"
    sansfont: "DejaVu Sans"
    monofont: "DejaVu Sans Mono"
---
```

## Best Practices

1. **Use Consistent Templates**: Standardize YAML frontmatter across project
2. **Version Control**: Commit `.qmd` files, not generated PDFs (unless needed)
3. **Code Chunks**: Keep code chunks small and focused
4. **Cross-References**: Use labels for figures, tables, equations
5. **Citations**: Maintain central bibliography file
6. **Project Structure**: Use `_quarto.yml` for shared settings
7. **Rendering**: Test renders frequently during writing
8. **Documentation**: Comment complex LaTeX customizations

## Integration with Version Control

`.gitignore` for Quarto projects:
```
# Quarto outputs
/_site/
/_output/
*.pdf
*.html
*.docx

# LaTeX intermediate files
*.aux
*.log
*.out
*.toc
*.tex
/.quarto/
```

## Command Line Reference

```bash
# Render document
quarto render document.qmd

# Preview with live reload
quarto preview document.qmd

# Check Quarto installation
quarto check

# List installed tools
quarto tools list

# Install/update TinyTeX
quarto install tinytex
quarto update tinytex

# Convert Jupyter notebook to Quarto
quarto convert notebook.ipynb

# Create new project
quarto create project default myproject
```

## Resources

- **Official Documentation**: https://quarto.org/docs/guide/
- **Gallery**: https://quarto.org/docs/gallery/
- **PDF Format Guide**: https://quarto.org/docs/output-formats/pdf-basics.html
- **Reference**: https://quarto.org/docs/reference/
- **GitHub**: https://github.com/quarto-dev/quarto-cli
- **Community**: https://github.com/quarto-dev/quarto-cli/discussions

## Example Workflow for flyby-f11

### Writing New Documentation

```bash
# Navigate to project
cd /home/finley/Github/DroneProjects/flyby-f11

# Create new document from template
cat > NEW_DOCUMENT.qmd << 'EOF'
---
title: "Document Title"
author: "Finley Holt"
date: today
format:
  pdf:
    documentclass: article
    geometry:
      - margin=1in
    fontsize: 11pt
    number-sections: true
    toc: true
    include-in-header:
      text: |
        \usepackage{fancyhdr}
        \pagestyle{fancy}
        \fancyhf{}
        \fancyhead[L]{\textit{Short Title}}
        \fancyhead[R]{\thepage}
        \renewcommand{\headrulewidth}{0.4pt}
---

# Introduction

Content here...
EOF

# Edit document
vim NEW_DOCUMENT.qmd

# Preview with live reload
quarto preview NEW_DOCUMENT.qmd

# Render final PDF
quarto render NEW_DOCUMENT.qmd

# Output: NEW_DOCUMENT.pdf
```

### Batch Rendering

```bash
# Render all project documents
cd /home/finley/Github/DroneProjects/flyby-f11
for file in *.qmd; do
    echo "Rendering $file..."
    quarto render "$file"
done

# Render literature review
cd literature_review
quarto render .
```

## Notes for flyby-f11 Project

- All technical documentation uses `.qmd` format per CLAUDE.md
- Standard template includes custom fancyhdr headers
- PDFs generated alongside `.qmd` files (not committed to git unless needed)
- Use `quarto render` to regenerate PDFs after editing
- Consistent formatting across all project documentation
- TinyTeX recommended for lightweight LaTeX installation
