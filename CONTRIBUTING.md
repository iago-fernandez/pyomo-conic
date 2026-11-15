# Contributing

Thank you for your interest in improving the Pyomo Conic Engine. This repository adheres to high-performance scientific computing and software engineering standards, prioritizing symbolic efficiency, mathematical rigor, and architectural cleanliness.

## Engineering Standards

To maintain computational efficiency and reliable solver integration, all contributions must adhere to the following:

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
   * `ci/`: Continuous integration, deployment pipelines, and release workflows.
   * `test/`: Test suite additions, benchmarks, and regression suites.
   * `chore/`: Dependency, packaging, and tooling maintenance.

3. **Validation:** Ensure the test suite passes before submitting changes:

   ```bash
   pytest
   ```

## Commit Guidelines

We use **Conventional Commits** without scopes to maintain a clean, semantic, and easily traceable history:

* `feat: implement automatic affine decomposition for SDP`
* `fix: correct coordinate offset in scalar variable mapping`
* `refactor: optimize lower-triangular normalization`
* `docs: clarify Lorentz cone mathematical definition`
* `ci: add automated multi-version test suite with GitHub Actions`
* `test: add benchmark integration tests for hybrid conic models`
* `chore: update Pyomo dependency baseline`

## Pull Request Process

* Provide a clear, bulleted description of modifications using the infinitive tense (e.g., `- Add support for...`, `- Fix indexing error...`).
* Ensure changes do not introduce regressions, deprecation warnings, or test failures.
* Squash intermediate commits upon merge to keep the repository history linear and atomic.

## Release Workflow

Releases follow Semantic Versioning (`vMAJOR.MINOR.PATCH`):

1. Verify that all tests pass across all supported Python versions in CI.
2. Publish a GitHub Release with the appropriate version tag (e.g., `v1.0.0`) on the `main` branch.
3. The automated publication workflow builds the package and deploys distribution artifacts to PyPI via Trusted Publishing.
