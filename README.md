# Pyomo Conic Engine

A Python extension for **Pyomo** that adds support for **Semidefinite Programming (SDP)** and **Second-Order Cone Programming (SOCP)** constraints.

The library automatically decomposes affine expressions into constant and variable coefficient matrices, exporting synchronized problem files in **MOSEK Conic Benchmark Format (CBF)** alongside AMPL `.nl` bytecode and column index mappings.

[![Status](https://img.shields.io/badge/status-stable-green?style=flat-square)](https://github.com/iago-fernandez/pyomo-conic/releases)
[![Python](https://img.shields.io/badge/python-3.12%2B-blue?style=flat-square)](requirements.txt)
[![Modeling](https://img.shields.io/badge/modeling-Pyomo%206.10%2B-yellow?style=flat-square)](https://www.pyomo.org/)
[![Format](https://img.shields.io/badge/format-CBF%20%7C%20AMPL-lightgrey?style=flat-square)](https://docs.mosek.com/latest/cbf/index.html)
[![License](https://img.shields.io/badge/license-MIT-orange?style=flat-square)](LICENSE)

## Overview

Standard `pyomo.environ` does not provide native abstractions for conic constraints such as Linear Matrix Inequalities (LMIs) or second-order Lorentz cones. Handling these problems typically requires reformulating them as quadratic inequalities (which breaks conic structure) or switching to convex-only tools like CVXPY (which cannot handle non-convex polynomial terms).

`pyomo-conic` allows defining conic constraints directly on `pyo.ConcreteModel` instances:
* **Conic modeling:** Add SDP matrix inequalities and SOCP cones to standard Pyomo models.
* **Affine decomposition:** Extracts constant matrices and variable coefficients in a single pass without auxiliary slack variables.
* **Solver export:** Writes synchronized AMPL bytecode (`.nl`), column order listings (`.col`), and Conic Benchmark Format (`.cbf`) files for solvers like RAPOSa, MOSEK, and Clarabel.

## Quickstart

### 1. Second-Order Cone Programming (SOCP)

Affine expressions can be constrained to a Lorentz cone using either norm-bound component form or vector form:

```python
import pyomo.environ as pyo
from pyomo_conic import add_socp_constraint, add_socp_cone, export_cbf_files

model = pyo.ConcreteModel()
model.x = pyo.Var([0, 1, 2], bounds=(0, None), initialize=1.0)
model.obj = pyo.Objective(expr=model.x[0] + model.x[1] + model.x[2], sense=pyo.minimize)

# Component form: 2.5 * x[0] >= sqrt( (3.0 * x[1])^2 + (1.5 * x[2])^2 )
add_socp_constraint(
    model,
    y0_expr=2.5 * model.x[0],
    y_rest_exprs=[3.0 * model.x[1], 1.5 * model.x[2]],
    name="cone_1"
)

# Vector form: [y0, y1, y2] in Q^3
add_socp_cone(
    model,
    expr_list=[2.5 * model.x[0], 3.0 * model.x[1], 1.5 * model.x[2]],
    name="cone_vector"
)

# Export solver files (.nl, .col, .cbf)
export_cbf_files(model, "output/socp_problem")
```

### 2. Semidefinite Programming (SDP / LMIs)

Linear Matrix Inequalities ($M(x) \succeq 0$) are defined as symmetric 2D lists of affine Pyomo expressions:

```python
import pyomo.environ as pyo
from pyomo_conic import add_sdp_constraint, export_cbf_files

model = pyo.ConcreteModel()
model.x1 = pyo.Var(bounds=(0, 5), initialize=1.0)
model.x2 = pyo.Var(bounds=(0, 5), initialize=1.0)

# Define 2x2 LMI: [[ 3*x2 - 1,   x1 + x2 ],
#                 [  x1 + x2,  3*x1 - 1 ]] >= 0
matrix = [
    [3.0 * model.x2 - 1.0, model.x1 + model.x2],
    [model.x1 + model.x2,  3.0 * model.x1 - 1.0]
]

add_sdp_constraint(model, matrix=matrix, name="lmi_1")

# Export solver files (.nl, .col, .cbf)
export_cbf_files(model, "output/sdp_problem")
```

## Mathematical Definition

The library supports two conic geometries:

### 1. Second-Order (Lorentz) Cone

A vector $y \in \mathbb{R}^d$ belongs to the Lorentz cone $\mathcal{Q}^d$ if and only if:

$$
\mathcal{Q}^d = \left\lbrace y \in \mathbb{R}^d \mid y_0 \ge \sqrt{\sum_{i=1}^{d-1} y_i^2}, \quad y_0 \ge 0 \right\rbrace
$$

Given affine expressions $y = b + A x$, the engine extracts constant vector $b \in \mathbb{R}^d$ and coefficient matrix $A \in \mathbb{R}^{d \times n}$.

### 2. Semidefinite Programming (LMI)

Given a symmetric affine matrix expression $M(x) \in \mathcal{S}^n$, the Linear Matrix Inequality enforces positive semidefiniteness:

$$
M(x) = D + \sum_{t=1}^k x_t H_t \succeq 0, \quad D, H_t \in \mathcal{S}^n
$$

The matrix $M(x)$ has non-negative eigenvalues for all feasible decision vectors $x$.

## Format Alignment (.cbf, .col, .nl)

Calling `export_cbf_files(model, "prefix")` generates three synchronized files:

1. **`prefix.nl`**: AMPL bytecode containing objective and algebraic constraints. Pyomo's `export_nonlinear_variables` directive ensures variables used only in conic domains are indexed without adding artificial constraints.
2. **`prefix.col`**: Ordered list of variable names matching the 0-indexed column IDs in `.nl`.
3. **`prefix.cbf`**: Conic Benchmark Format file following the MOSEK CBF v4 specification:
   * `PSDCON`: Defines positive semidefinite matrix cones with lower-triangular coordinates (`HCOORD`, `DCOORD`).
   * `CON`: Defines quadratic cones (`Q`) and affine coordinates (`ACOORD`, `BCOORD`).
   * `SCALARCORR`: Explicit mapping from CBF scalar variable indices to `.nl` column names.

## Implementation Details

* **Single-pass traversal:** Uses Pyomo's `generate_standard_repn` to extract affine terms in $O(N)$ time without numerical differentiation.
* **Sparse lower-triangular storage:** Symmetric matrices are stored in lower-triangular coordinates ($r \ge c$) with small-value thresholding ($10^{-12}$) to keep files sparse.
* **Direct domain registration:** Embeds conic bounds into the `.cbf` file, avoiding intermediate slack equality constraints.
* **Variable alignment:** Matches variable names in the `.col` file with the column order in the `.nl` and `.cbf` files.

## Project Structure

```text
pyomo-conic/
├── pyomo_conic/
│   ├── __init__.py           # Package exports
│   ├── constraints.py        # Conic container, SDP/SOCP models, affine extraction
│   └── exporter.py           # CBF, NL, and COL problem exporter
├── tests/
│   ├── test_constraints.py   # Unit tests for affine extraction and validations
│   ├── test_exporter.py      # Unit tests for file generation and column ordering
│   └── test_benchmarks.py    # Integration benchmarks (SDP, SOCP, Hybrid)
├── pyproject.toml            # Build configuration and dependencies
├── requirements.txt          # Runtime requirements
├── CONTRIBUTING.md           # Contribution guidelines
├── LICENSE                   # MIT License
└── README.md                 # Project documentation
```

## Installation & Setup

### Prerequisites

* **Python 3.12+**
* **Pyomo 6.10+**
* **NumPy 2.5+**

### Quick Start

```bash
# Clone the repository
git clone git@github.com:iago-fernandez/pyomo-conic.git
cd pyomo-conic

# Create and activate virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install in editable mode with development dependencies
pip install -e ".[dev]"
```

## Testing

The test suite validates affine matrix decomposition, coordinate mapping, boundary conditions, and canonical benchmarks:

```bash
pytest
```

## Research & Attribution

This library was developed as part of research engineering on [RAPOSa](https://raposa.usc.es), a global optimization solver for continuous polynomial programming.

## Contributing

Please review the [CONTRIBUTING](CONTRIBUTING.md) guidelines for code style and development workflows.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
