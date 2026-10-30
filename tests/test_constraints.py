"""
Unit tests for conic constraint representations and affine decomposition.
"""

import pytest
import pyomo.environ as pyo
from pyomo_conic.constraints import (
    add_sdp_constraint,
    add_socp_cone,
    add_socp_constraint,
    get_conic_container,
)


def test_sdp_affine_decomposition():
    model = pyo.ConcreteModel()
    model.x1 = pyo.Var()
    model.x2 = pyo.Var()

    matrix = [
        [3.0 * model.x1 + 2.0, model.x2 - 1.0],
        [model.x2 - 1.0, 5.0 * model.x2 + 4.0],
    ]
    sdp = add_sdp_constraint(model, matrix, name="test_sdp")

    assert sdp.dim == 2
    assert sdp.name == "test_sdp"
    assert sdp.D_entries[(0, 0)] == pytest.approx(2.0)
    assert sdp.D_entries[(1, 0)] == pytest.approx(-1.0)
    assert sdp.D_entries[(1, 1)] == pytest.approx(4.0)

    assert sdp.H_entries[model.x1][(0, 0)] == pytest.approx(3.0)
    assert sdp.H_entries[model.x2][(1, 0)] == pytest.approx(1.0)
    assert sdp.H_entries[model.x2][(1, 1)] == pytest.approx(5.0)


def test_sdp_non_square_matrix():
    model = pyo.ConcreteModel()
    model.x = pyo.Var()

    matrix = [
        [model.x, 1.0, 2.0],
        [1.0, model.x],
    ]
    with pytest.raises(ValueError, match="square"):
        add_sdp_constraint(model, matrix)


def test_sdp_zero_dimension():
    model = pyo.ConcreteModel()
    with pytest.raises(ValueError, match="greater than zero"):
        add_sdp_constraint(model, [])


def test_sdp_nonlinear_expression():
    model = pyo.ConcreteModel()
    model.x1 = pyo.Var()
    model.x2 = pyo.Var()

    matrix = [
        [model.x1 * model.x2, 0.0],
        [0.0, 1.0],
    ]
    with pytest.raises(ValueError, match="affine or linear"):
        add_sdp_constraint(model, matrix)


def test_socp_cone_decomposition():
    model = pyo.ConcreteModel()
    model.x1 = pyo.Var()
    model.x2 = pyo.Var()

    expr_list = [2.0 * model.x1 + 3.0, -1.5 * model.x2 + 0.5, 4.0]
    socp = add_socp_cone(model, expr_list, name="test_socp")

    assert socp.dim == 3
    assert socp.name == "test_socp"
    assert socp.b_entries[0] == pytest.approx(3.0)
    assert socp.b_entries[1] == pytest.approx(0.5)
    assert socp.b_entries[2] == pytest.approx(4.0)

    var_entries = [(row, v, coef) for (row, v, coef) in socp.A_entries]
    assert (0, model.x1, 2.0) in var_entries
    assert (1, model.x2, -1.5) in var_entries


def test_socp_constraint_wrapper():
    model = pyo.ConcreteModel()
    model.x1 = pyo.Var()
    model.x2 = pyo.Var()

    socp = add_socp_constraint(
        model,
        y0_expr=model.x1 + 1.0,
        y_rest_exprs=[model.x2, 2.0],
        name="norm_cone",
    )
    assert socp.dim == 3
    assert socp.name == "norm_cone"
    assert socp.b_entries[0] == pytest.approx(1.0)
    assert socp.b_entries[2] == pytest.approx(2.0)


def test_socp_dimension_under_two():
    model = pyo.ConcreteModel()
    model.x = pyo.Var()

    with pytest.raises(ValueError, match="at least 2"):
        add_socp_cone(model, [model.x])


def test_socp_nonlinear_expression():
    model = pyo.ConcreteModel()
    model.x = pyo.Var()

    with pytest.raises(ValueError, match="affine or linear"):
        add_socp_cone(model, [model.x, model.x**2])


def test_conic_container_variable_extraction():
    model = pyo.ConcreteModel()
    model.x1 = pyo.Var()
    model.x2 = pyo.Var()
    model.x3 = pyo.Var()

    add_sdp_constraint(model, [[model.x1, 0.0], [0.0, model.x2]])
    add_socp_cone(model, [model.x3, model.x1])

    container = get_conic_container(model)
    vars_set = container.get_all_variables()

    assert len(vars_set) == 3
    assert model.x1 in vars_set
    assert model.x2 in vars_set
    assert model.x3 in vars_set
