"""
Unit tests for conic problem exporter and format synchronization.
"""

import os
import pyomo.environ as pyo
from pyomo_conic.constraints import add_sdp_constraint, add_socp_constraint
from pyomo_conic.exporter import export_cbf_files


def test_export_file_triplet_generation(tmp_path):
    prefix = str(tmp_path / "simple_problem")
    model = pyo.ConcreteModel()
    model.x = pyo.Var(initialize=1.0)
    model.obj = pyo.Objective(expr=model.x)
    add_socp_constraint(model, y0_expr=model.x, y_rest_exprs=[1.0])

    nl_path, col_path, cbf_path = export_cbf_files(model, prefix)

    assert os.path.isfile(nl_path)
    assert os.path.isfile(col_path)
    assert os.path.isfile(cbf_path)
    assert not os.path.isfile(f"{prefix}.row")


def test_export_variables_only_in_conic_domains(tmp_path):
    prefix = str(tmp_path / "conic_only_vars")
    model = pyo.ConcreteModel()
    model.x_obj = pyo.Var()
    model.x_conic_sdp = pyo.Var()
    model.x_conic_socp = pyo.Var()

    model.obj = pyo.Objective(expr=model.x_obj)
    add_sdp_constraint(model, [[model.x_conic_sdp, 0.0], [0.0, 1.0]])
    add_socp_constraint(model, y0_expr=10.0, y_rest_exprs=[model.x_conic_socp])

    _, col_path, cbf_path = export_cbf_files(model, prefix)

    with open(col_path) as f:
        col_names = [line.strip() for line in f if line.strip()]

    assert "x_obj" in col_names
    assert "x_conic_sdp" in col_names
    assert "x_conic_socp" in col_names

    with open(cbf_path) as f:
        cbf_content = f.read()

    assert "VAR\n3 1\nF 3" in cbf_content
    assert "SCALARCORR" in cbf_content


def test_export_column_ordering_matches_cbf(tmp_path):
    prefix = str(tmp_path / "order_check")
    model = pyo.ConcreteModel()
    model.a = pyo.Var()
    model.b = pyo.Var()
    model.c = pyo.Var()
    model.obj = pyo.Objective(expr=model.c + model.b + model.a)
    add_sdp_constraint(model, [[model.b, model.a], [model.a, model.c]])

    _, col_path, cbf_path = export_cbf_files(model, prefix)

    with open(col_path) as f:
        col_names = [line.strip() for line in f if line.strip()]

    with open(cbf_path) as f:
        cbf_lines = [line.strip() for line in f]

    corr_idx = cbf_lines.index("SCALARCORR")
    cbf_mappings = cbf_lines[corr_idx + 1: corr_idx + 1 + len(col_names)]

    for idx, (col_name, mapping) in enumerate(zip(col_names, cbf_mappings)):
        cbf_id, cbf_var = mapping.split()
        assert int(cbf_id) == idx
        assert cbf_var == col_name
