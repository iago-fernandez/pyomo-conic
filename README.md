# Pyomo Conic Engine

A lightweight, high-performance extension for **Pyomo** providing first-class modeling support for **Semidefinite Programming (SDP)** and **Second-Order Cone Programming (SOCP)** constraints.

The engine automates symbolic affine decomposition directly from standard Pyomo expressions, producing clean problem definitions without auxiliary variables and exporting synchronized **MOSEK Conic Benchmark Format (CBF)** files alongside AMPL `.nl` bytecode and column index mappings.

[![Status](https://img.shields.io/badge/status-dev-green?style=flat-square)](https://github.com/iago-fernandez/pyomo-conic)
[![Python](https://img.shields.io/badge/python-3.12%2B-blue?style=flat-square)](requirements.txt)
[![Modeling](https://img.shields.io/badge/modeling-Pyomo%206.10%2B-yellow?style=flat-square)](https://www.pyomo.org/)
[![Format](https://img.shields.io/badge/format-CBF%20%7C%20AMPL-lightgrey?style=flat-square)](https://docs.mosek.com/latest/cbf/index.html)
[![License](https://img.shields.io/badge/license-MIT-orange?style=flat-square)](LICENSE)

## Mathematical Scope

The engine introduces native representations for two fundamental conic geometries:

### 1. Second-Order (Lorentz) Cone

A vector $y \in \mathbb{R}^d$ belongs to the Lorentz cone $\mathcal{Q}^d$ if and only if:

$$\mathcal{Q}^d = \left\{ y \in \mathbb{R}^d \;\middle|\; y_0 \ge \sqrt{\sum_{i=1}^{d-1} y_i^2}, \; y_0 \ge 0 \right\}$$

Given affine combinations $y = b + A x$, the engine extracts constant shifts $b \in \mathbb{R}^d$ and coefficient matrices $A \in \mathbb{R}^{d \times n}$ for direct conic embedding.

### 2. Semidefinite Programming (LMI)

Given a symmetric affine matrix expression $M(x) \in \mathcal{S}^n$, the Linear Matrix Inequality enforces positive semidefiniteness:

$$M(x) = D + \sum_{t=1}^k x_t H_t \succeq 0, \quad D, H_t \in \mathcal{S}^n$$

The matrix $M(x)$ has non-negative eigenvalues for all feasible decision vectors $x$.

## Contributing

Please review [CONTRIBUTING.md](CONTRIBUTING.md) for engineering standards, commit conventions, and the development workflow.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
