# Contributing

Thank you for your interest in improving the Pyomo Conic Engine. This repository strictly adheres to high-performance scientific computing and software engineering standards, prioritizing symbolic efficiency, mathematical rigor, and architectural cleanliness.

## Engineering Standards

To maintain computational efficiency and seamless solver integration, all contributions must adhere to the following:

* **Clean Code Philosophy:** Code must be self-documenting through expressive variable naming, strict type hinting (`typing` module), and modularity. Avoid redundant comments that merely explain *what* the code does; reserve comments exclusively to clarify *why* a specific mathematical or architectural decision was made.
* **Symbolic Efficiency:** Traversal of Pyomo expression trees must operate in linear time without generating intermediate symbolic duplicates. Rely on standard Pyomo representation routines (`generate_standard_repn`).
* **Coordinate Fidelity:** All variable mappings between AMPL `.nl` column representations and Conic Benchmark Format (`.cbf`) coordinates must remain strictly synchronized and zero-indexed.
* **Minimal Abstraction Overhead:** Avoid unnecessary numerical callbacks, matrix inversions, or artificial auxiliary equality constraints when decomposing affine conic components.
* **Code Style:** Strict adherence to **PEP 8**. Ensure code passes standard linting and static analysis without warnings.

## Development Workflow

1. **Fork and Clone** the repository.

2. **Branching Strategy:**

   * `feat/`: New conic abstractions, solver exporters, or core algorithms.
   * `fix/`: Resolution of coordinate misalignments, parsing edge cases, or numerical precision bugs.
   * `refactor/`: Symbolic traversal optimization or memory footprint reductions.
   * `docs/`: Technical documentation and mathematical specification updates.
   * `chore/`: Dependency, packaging, and tooling maintenance.
   
3. **Validation:** Ensure the test suite passes before pushing code:

   ```bash
   pytest
   ```

## Commit Guidelines

We use **Conventional Commits** to maintain a clean, semantic, and easily traceable history:

* `feat(core): implement automatic affine decomposition for SDP`
* `fix(exporter): correct coordinate offset in scalar variable mapping`
* `refactor(constraints): optimize lower-triangular normalization`
* `docs(readme): clarify Lorentz cone mathematical definition`
* `chore(deps): update Pyomo dependency baseline`

## Pull Request Process

* Provide a clear, bulleted description of modifications using the infinitive tense (e.g., `- Add support for...`, `- Fix indexing error...`).
* Ensure changes do not introduce regressions, deprecation warnings, or test failures.
* Squash intermediate commits to keep the repository history linear and atomic.
