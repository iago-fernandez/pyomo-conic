"""
Integration benchmark tests validating canonical conic optimization problems.
"""

import os
import pyomo.environ as pyo
from pyomo_conic.constraints import add_sdp_constraint, add_socp_constraint
from pyomo_conic.exporter import export_cbf_files


def test_sdp_canonical_benchmark(tmp_path):
    prefix = str(tmp_path / "sdp_benchmark")
    model = pyo.ConcreteModel(name="sdp_benchmark")
    model.x1 = pyo.Var(bounds=(0.0, 4.0), initialize=1.0)
    model.x2 = pyo.Var(bounds=(0.0, 5.0), initialize=1.0)
    model.x3 = pyo.Var(bounds=(0.0, 6.0), initialize=1.0)
    model.x4 = pyo.Var(bounds=(0.0, 7.0), initialize=1.0)

    model.obj = pyo.Objective(
        expr=6.0 * model.x1 - 1.5 * model.x2 + model.x4 + 1.0,
        sense=pyo.minimize
    )
    model.c1 = pyo.Constraint(expr=2.0 * model.x3 - model.x1 - model.x2 >= 0.0)
    model.c2 = pyo.Constraint(expr=model.x1 * (model.x2**2) + 5.0 >= 3.0)

    add_sdp_constraint(
        model,
        matrix=[
            [3.0 * model.x2 - 1.0, model.x1 + model.x2],
            [model.x1 + model.x2, 3.0 * model.x1 - 1.0],
        ],
        name="lmi_1",
    )
    add_sdp_constraint(
        model,
        matrix=[
            [model.x1, model.x3],
            [model.x3, model.x4],
        ],
        name="lmi_2",
    )

    nl_file, col_file, cbf_file = export_cbf_files(model, prefix)

    assert os.path.isfile(nl_file)
    assert os.path.isfile(col_file)
    assert os.path.isfile(cbf_file)

    with open(cbf_file) as f:
        content = f.read()

    assert "VER\n4" in content
    assert "VAR\n4 1\nF 4" in content
    assert "PSDCON\n2\n2\n2" in content
    assert "HCOORD\n7" in content
    assert "DCOORD\n2" in content
    assert "SCALARCORR" in content


def test_socp_canonical_benchmark(tmp_path):
    prefix = str(tmp_path / "socp_benchmark")
    model = pyo.ConcreteModel(name="socp_benchmark")
    model.x1 = pyo.Var(bounds=(0.0, 10.0), initialize=1.0)
    model.x2 = pyo.Var(bounds=(0.0, 20.0), initialize=1.0)
    model.x3 = pyo.Var(bounds=(0.0, 30.0), initialize=1.0)
    model.x4 = pyo.Var(bounds=(0.0, 40.0), initialize=1.0)
    model.x5 = pyo.Var(bounds=(0.0, 50.0), initialize=1.0)
    model.x6 = pyo.Var(bounds=(0.0, 60.0), initialize=1.0)

    model.obj = pyo.Objective(
        expr=6.0 * model.x1 - 1.5 * model.x2 + model.x4 + 1.0,
        sense=pyo.minimize
    )
    model.c1 = pyo.Constraint(expr=model.x1 * (model.x4**2) >= 2.0)
    model.c2 = pyo.Constraint(expr=model.x3 + model.x5 + model.x6 >= 0.0)

    add_socp_constraint(
        model,
        y0_expr=2.5 * model.x3 - 1.5,
        y_rest_exprs=[3.1 * model.x1, -0.8 * model.x2],
        name="socp_1",
    )
    add_socp_constraint(
        model,
        y0_expr=model.x6,
        y_rest_exprs=[model.x4, model.x5],
        name="socp_2",
    )

    nl_file, col_file, cbf_file = export_cbf_files(model, prefix)

    assert os.path.isfile(nl_file)
    assert os.path.isfile(col_file)
    assert os.path.isfile(cbf_file)

    with open(cbf_file) as f:
        content = f.read()

    assert "VER\n4" in content
    assert "VAR\n6 1\nF 6" in content
    assert "CON\n6 2\nQ 3\nQ 3" in content
    assert "ACOORD\n6" in content
    assert "BCOORD\n1" in content
    assert "SCALARCORR" in content


def test_hybrid_canonical_benchmark(tmp_path):
    prefix = str(tmp_path / "hybrid_benchmark")
    model = pyo.ConcreteModel(name="hybrid_benchmark")
    model.x1 = pyo.Var(bounds=(0.0, 10.0), initialize=1.0)
    model.x2 = pyo.Var(bounds=(0.0, 20.0), initialize=1.0)
    model.x3 = pyo.Var(bounds=(0.0, 30.0), initialize=1.0)
    model.x4 = pyo.Var(bounds=(0.0, 40.0), initialize=1.0)
    model.x5 = pyo.Var(bounds=(0.0, 50.0), initialize=1.0)
    model.x6 = pyo.Var(bounds=(0.0, 60.0), initialize=1.0)

    model.obj = pyo.Objective(
        expr=6.0 * model.x1 - 1.5 * model.x2 + model.x4 + 1.0,
        sense=pyo.minimize
    )
    model.c1 = pyo.Constraint(expr=model.x1 * (model.x4**2) >= 2.0)
    model.c2 = pyo.Constraint(expr=model.x3 + model.x5 + model.x6 >= 0.0)

    add_sdp_constraint(
        model,
        matrix=[
            [model.x1, model.x3],
            [model.x3, model.x4],
        ],
        name="sdp_hybrid",
    )
    add_socp_constraint(
        model,
        y0_expr=model.x6,
        y_rest_exprs=[model.x4, model.x5],
        name="socp_hybrid",
    )

    nl_file, col_file, cbf_file = export_cbf_files(model, prefix)

    assert os.path.isfile(nl_file)
    assert os.path.isfile(col_file)
    assert os.path.isfile(cbf_file)

    with open(cbf_file) as f:
        content = f.read()

    assert "VER\n4" in content
    assert "PSDCON\n1\n2" in content
    assert "CON\n3 1\nQ 3" in content
    assert "SCALARCORR" in content
