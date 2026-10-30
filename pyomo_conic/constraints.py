"""
Conic constraint representations and modeling extensions for Pyomo.

Provides mathematical abstractions and symbolic affine decomposition routines
for Semidefinite Programming (SDP) and Second-Order Cone Programming (SOCP).
"""

from __future__ import annotations

from typing import Any, Optional
import pyomo.environ as pyo
from pyomo.common.collections import ComponentMap, ComponentSet
from pyomo.repn.standard_repn import generate_standard_repn


class SDPConstraint:
    """
    Symmetric positive semidefinite matrix constraint M(x) >= 0.

    Stores affine matrix terms M(x) = D + sum(x_t * H_t) in lower-triangular
    coordinate representation to enforce positive semidefiniteness.
    """

    def __init__(self, dim: int, name: Optional[str] = None):
        self.dim = int(dim)
        self.name = name or f"sdp_{id(self)}"
        self.D_entries: dict[tuple[int, int], float] = {}
        self.H_entries: ComponentMap = ComponentMap()

    def set_d_entry(self, row: int, col: int, val: float):
        """Sets a constant coefficient normalized to the lower triangle."""
        if col > row:
            row, col = col, row
        if abs(val) > 1e-12:
            self.D_entries[(row, col)] = val
        elif (row, col) in self.D_entries:
            del self.D_entries[(row, col)]

    def set_h_entry(self, var_obj: Any, row: int, col: int, val: float):
        """Sets a variable coefficient normalized to the lower triangle."""
        if col > row:
            row, col = col, row
        if var_obj not in self.H_entries:
            self.H_entries[var_obj] = {}
        if abs(val) > 1e-12:
            self.H_entries[var_obj][(row, col)] = val
        elif (row, col) in self.H_entries[var_obj]:
            del self.H_entries[var_obj][(row, col)]


class SOCPConstraint:
    """
    Second-order cone constraint defining affine vector membership in Q^dim.

    Stores linear coefficients A and affine shift b such that b + Ax in Q^dim,
    representing the Euclidean norm upper-bound constraint y_0 >= ||y_{1, ..., dim-1}||_2.
    """

    def __init__(self, dim: int, name: Optional[str] = None):
        self.dim = int(dim)
        self.name = name or f"socp_{id(self)}"
        self.b_entries: dict[int, float] = {}
        self.A_entries: list[tuple[int, Any, float]] = []

    def set_b_entry(self, row: int, val: float):
        """Sets an affine constant shift in a designated coordinate row."""
        if abs(val) > 1e-12:
            self.b_entries[row] = val
        elif row in self.b_entries:
            del self.b_entries[row]

    def add_a_entry(self, row: int, var_obj: Any, val: float):
        """Appends a linear variable coefficient to a designated coordinate row."""
        if abs(val) > 1e-12:
            self.A_entries.append((row, var_obj, val))


class ConicContainer:
    """Stores all conic constraints associated with a Pyomo ConcreteModel."""

    def __init__(self):
        self.sdp_constraints: list[SDPConstraint] = []
        self.socp_constraints: list[SOCPConstraint] = []

    def get_all_variables(self) -> ComponentSet:
        """Retrieves the complete set of Pyomo variables referenced in conic domains."""
        vars_set = ComponentSet()
        for sdp in self.sdp_constraints:
            for v in sdp.H_entries.keys():
                vars_set.add(v)
        for socp in self.socp_constraints:
            for (_, v, _) in socp.A_entries:
                vars_set.add(v)
        return vars_set


def get_conic_container(model: pyo.ConcreteModel) -> ConicContainer:
    """Retrieves or initializes the conic constraint container on a Pyomo model."""
    container = getattr(model, "_pyomo_conic", None)
    if isinstance(container, ConicContainer):
        return container

    raposa_container = getattr(model, "_raposa_conic", None)
    if isinstance(raposa_container, ConicContainer):
        return raposa_container

    new_container = ConicContainer()
    setattr(model, "_pyomo_conic", new_container)
    return new_container


def add_sdp_constraint(
    model: pyo.ConcreteModel,
    matrix: list[list[Any]],
    name: Optional[str] = None
) -> SDPConstraint:
    """
    Attaches a Linear Matrix Inequality (LMI) constraint to the model.

    Extracts affine constant matrices D and variable coefficient matrices H_t
    from the lower-triangular entries of the symmetric expression matrix.
    """
    dim = len(matrix)
    if dim == 0:
        raise ValueError("SDP matrix dimension must be greater than zero.")

    for row in matrix:
        if len(row) != dim:
            raise ValueError(f"SDP matrix must be square, got {dim}x{len(row)}")

    sdp = SDPConstraint(dim=dim, name=name)

    for r in range(dim):
        for c in range(r + 1):
            expr = matrix[r][c]
            repn = generate_standard_repn(expr)
            if not repn.is_linear():
                raise ValueError(
                    f"Entry ({r}, {c}) in SDP matrix must be affine or linear, got: {expr}"
                )

            if repn.constant is not None and abs(repn.constant) > 1e-12:
                sdp.set_d_entry(r, c, float(repn.constant))

            if repn.linear_vars:
                for var, coef in zip(repn.linear_vars, repn.linear_coefs):
                    if abs(coef) > 1e-12:
                        sdp.set_h_entry(var, r, c, float(coef))

    container = get_conic_container(model)
    container.sdp_constraints.append(sdp)
    return sdp


def add_socp_cone(
    model: pyo.ConcreteModel,
    expr_list: list[Any],
    name: Optional[str] = None
) -> SOCPConstraint:
    """
    Attaches a Second-Order Cone constraint in vector format: expr_list in Q^dim.

    The first entry represents the upper-bound coordinate y_0 >= ||y_{1, ..., dim-1}||_2.
    """
    dim = len(expr_list)
    if dim < 2:
        raise ValueError("Second-Order Cone must have dimension of at least 2.")

    socp = SOCPConstraint(dim=dim, name=name)

    for i, expr in enumerate(expr_list):
        repn = generate_standard_repn(expr)
        if not repn.is_linear():
            raise ValueError(
                f"SOCP component {i} must be affine or linear, got: {expr}"
            )

        if repn.constant is not None and abs(repn.constant) > 1e-12:
            socp.set_b_entry(i, float(repn.constant))

        if repn.linear_vars:
            for var, coef in zip(repn.linear_vars, repn.linear_coefs):
                if abs(coef) > 1e-12:
                    socp.add_a_entry(i, var, float(coef))

    container = get_conic_container(model)
    container.socp_constraints.append(socp)
    return socp


def add_socp_constraint(
    model: pyo.ConcreteModel,
    y0_expr: Any,
    y_rest_exprs: list[Any],
    name: Optional[str] = None
) -> SOCPConstraint:
    """
    Attaches a Second-Order Cone constraint specified by norm components.

    Enforces y0_expr >= sqrt(sum(y_rest_exprs^2)) via vector cone representation.
    """
    full_list = [y0_expr] + list(y_rest_exprs)
    return add_socp_cone(model, full_list, name=name)
