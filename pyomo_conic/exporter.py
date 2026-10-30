"""
Conic benchmark file exporter for Pyomo models.

Generates synchronized problem files (.nl, .col, .cbf) for conic solvers.
"""

from __future__ import annotations

import os
import weakref
from typing import Any, Optional
import pyomo.environ as pyo
from pyomo.common.collections import ComponentMap
from .constraints import get_conic_container


def export_cbf_files(
    model: pyo.ConcreteModel,
    prefix_path: str,
    io_options: Optional[dict[str, Any]] = None
) -> tuple[str, str, str]:
    """
    Exports a Pyomo model and its attached conic constraints into solver files.

    Writes the synchronized file triplet:
    prefix_path.nl contains algebraic objectives and constraints.
    prefix_path.col contains ordered variable labels matching the column indices.
    prefix_path.cbf contains conic domains and affine coordinate mappings.
    """
    prefix_path = os.path.abspath(prefix_path)
    os.makedirs(os.path.dirname(prefix_path), exist_ok=True)

    nl_path = f"{prefix_path}.nl"
    col_path = f"{prefix_path}.col"
    cbf_path = f"{prefix_path}.cbf"

    container = get_conic_container(model)
    conic_vars = container.get_all_variables()

    writer_options: dict[str, Any] = {
        "symbolic_solver_labels": True,
        "export_nonlinear_variables": list(conic_vars),
    }
    if io_options:
        writer_options.update(io_options)

    fname, smap_id = model.write(nl_path, io_options=writer_options)
    smap = model.solutions.symbol_map[smap_id]

    row_path = f"{prefix_path}.row"
    if os.path.isfile(row_path):
        try:
            os.remove(row_path)
        except OSError:
            pass

    var_dict: dict[int, str] = {}
    obj_to_idx = ComponentMap()
    name_to_idx: dict[str, int] = {}

    for sym, raw_obj in smap.bySymbol.items():
        if sym.startswith("v") and sym[1:].isdigit():
            idx = int(sym[1:])
            obj = raw_obj() if isinstance(raw_obj, weakref.ReferenceType) else raw_obj
            var_name = obj.name
            var_dict[idx] = var_name
            obj_to_idx[obj] = idx
            name_to_idx[var_name] = idx

    num_vars = len(var_dict)
    col_vars = [var_dict[i] for i in range(num_vars)]

    with open(col_path, "w", encoding="utf-8") as f_col:
        for vname in col_vars:
            f_col.write(f"{vname}\n")

    def get_var_index(v: Any) -> int:
        actual_v = v() if isinstance(v, weakref.ReferenceType) else v
        if actual_v in obj_to_idx:
            return obj_to_idx[actual_v]
        if hasattr(actual_v, "name") and actual_v.name in name_to_idx:
            return name_to_idx[actual_v.name]
        if str(actual_v) in name_to_idx:
            return name_to_idx[str(actual_v)]
        raise KeyError(f"Variable {actual_v} not found in exported problem variables.")

    with open(cbf_path, "w", encoding="utf-8") as f_cbf:
        f_cbf.write("# Generated CBF file by pyomo-conic\n")
        f_cbf.write("VER\n4\n\n")

        f_cbf.write("VAR\n")
        f_cbf.write(f"{num_vars} 1\nF {num_vars}\n\n")

        if container.sdp_constraints:
            num_sdp = len(container.sdp_constraints)
            f_cbf.write("PSDCON\n")
            f_cbf.write(f"{num_sdp}\n")
            for sdp in container.sdp_constraints:
                f_cbf.write(f"{sdp.dim}\n")
            f_cbf.write("\n")

            h_entries = []
            for sdp_idx, sdp in enumerate(container.sdp_constraints):
                for var_obj, entries in sdp.H_entries.items():
                    var_idx = get_var_index(var_obj)
                    for (r, c), val in entries.items():
                        h_entries.append((sdp_idx, var_idx, r, c, val))

            if h_entries:
                f_cbf.write("HCOORD\n")
                f_cbf.write(f"{len(h_entries)}\n")
                for sdp_idx, var_idx, r, c, val in h_entries:
                    f_cbf.write(f"{sdp_idx} {var_idx} {r} {c} {val:.10g}\n")
                f_cbf.write("\n")

            d_entries = []
            for sdp_idx, sdp in enumerate(container.sdp_constraints):
                for (r, c), val in sdp.D_entries.items():
                    d_entries.append((sdp_idx, r, c, val))

            if d_entries:
                f_cbf.write("DCOORD\n")
                f_cbf.write(f"{len(d_entries)}\n")
                for sdp_idx, r, c, val in d_entries:
                    f_cbf.write(f"{sdp_idx} {r} {c} {val:.10g}\n")
                f_cbf.write("\n")

        if container.socp_constraints:
            total_rows = sum(socp.dim for socp in container.socp_constraints)
            num_domains = len(container.socp_constraints)

            f_cbf.write("CON\n")
            f_cbf.write(f"{total_rows} {num_domains}\n")
            for socp in container.socp_constraints:
                f_cbf.write(f"Q {socp.dim}\n")
            f_cbf.write("\n")

            a_entries = []
            b_entries = []
            current_row_start = 0

            for socp in container.socp_constraints:
                for (local_row, var_obj, val) in socp.A_entries:
                    var_idx = get_var_index(var_obj)
                    a_entries.append((current_row_start + local_row, var_idx, val))

                for local_row, val in socp.b_entries.items():
                    b_entries.append((current_row_start + local_row, val))

                current_row_start += socp.dim

            if a_entries:
                f_cbf.write("ACOORD\n")
                f_cbf.write(f"{len(a_entries)}\n")
                for grow, var_idx, val in a_entries:
                    f_cbf.write(f"{grow} {var_idx} {val:.10g}\n")
                f_cbf.write("\n")

            if b_entries:
                f_cbf.write("BCOORD\n")
                f_cbf.write(f"{len(b_entries)}\n")
                for grow, val in b_entries:
                    f_cbf.write(f"{grow} {val:.10g}\n")
                f_cbf.write("\n")

        f_cbf.write("SCALARCORR\n")
        for i, vname in enumerate(col_vars):
            f_cbf.write(f"{i} {vname}\n")
        f_cbf.write("\n")

    return nl_path, col_path, cbf_path


export_raposa_files = export_cbf_files
