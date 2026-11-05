"""
Conic benchmark file exporter for Pyomo models.

Generates synchronized problem files (.nl, .col, .cbf) for conic solvers.
"""

from __future__ import annotations

import os
import weakref
from typing import Any, NamedTuple, Optional
import pyomo.environ as pyo
from pyomo.common.collections import ComponentMap
from .constraints import get_conic_container, get_conic_summary


class ExportResult(NamedTuple):
    """Container for paths to generated solver problem files."""
    nl: Optional[str]
    col: Optional[str]
    cbf: str

    def as_dict(self) -> dict[str, Optional[str]]:
        """Returns generated file paths as a dictionary."""
        return {"nl": self.nl, "col": self.col, "cbf": self.cbf}


def export_cbf_files(
    model: pyo.ConcreteModel,
    prefix_path: str,
    *,
    export_nl: bool = True,
    export_col: bool = True,
    tolerance: float = 1e-12,
    verbose: bool = False,
    io_options: Optional[dict[str, Any]] = None,
    **pyomo_options: Any,
) -> ExportResult:
    """
    Exports a Pyomo model and its attached conic constraints into solver files.

    Writes the synchronized file triplet:
    prefix_path.nl contains algebraic objectives and constraints (if export_nl=True).
    prefix_path.col contains ordered variable labels matching column indices (if export_col=True).
    prefix_path.cbf contains conic domains and affine coordinate mappings.

    Keyword arguments not specifically intercepted by this function are passed
    directly to Pyomo's underlying model.write call as io_options.
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
    if pyomo_options:
        writer_options.update(pyomo_options)

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
            if obj is None:
                continue
            var_name = str(getattr(obj, "name", f"v{idx}"))
            var_dict[idx] = var_name
            obj_to_idx[obj] = idx
            name_to_idx[var_name] = idx

    num_vars = len(var_dict)
    col_vars = [var_dict[i] for i in range(num_vars)]

    if export_col:
        with open(col_path, "w", encoding="utf-8") as f_col:
            for vname in col_vars:
                f_col.write(f"{vname}\n")
        final_col_path: Optional[str] = col_path
    else:
        if os.path.isfile(col_path):
            try:
                os.remove(col_path)
            except OSError:
                pass
        final_col_path = None

    if not export_nl:
        if os.path.isfile(nl_path):
            try:
                os.remove(nl_path)
            except OSError:
                pass
        final_nl_path: Optional[str] = None
    else:
        final_nl_path = nl_path

    def get_var_index(v: Any) -> int:
        actual_v = v() if isinstance(v, weakref.ReferenceType) else v
        if actual_v in obj_to_idx:
            return obj_to_idx[actual_v]
        v_name = getattr(actual_v, "name", None)
        if isinstance(v_name, str) and v_name in name_to_idx:
            return name_to_idx[v_name]
        str_v = str(actual_v)
        if str_v in name_to_idx:
            return name_to_idx[str_v]
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
                        if abs(val) > tolerance:
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
                    if abs(val) > tolerance:
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
                    if abs(val) > tolerance:
                        var_idx = get_var_index(var_obj)
                        a_entries.append((current_row_start + local_row, var_idx, val))

                for local_row, val in socp.b_entries.items():
                    if abs(val) > tolerance:
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

    if verbose:
        summary = get_conic_summary(model)
        files_written = []
        if final_nl_path:
            files_written.append(os.path.basename(final_nl_path))
        if final_col_path:
            files_written.append(os.path.basename(final_col_path))
        files_written.append(os.path.basename(cbf_path))
        print(f"[pyomo-conic] Exported problem '{os.path.basename(prefix_path)}':")
        print(f"  Total variables: {num_vars} (conic domain variables: {summary['num_variables']})")
        print(f"  SDP constraints: {summary['num_sdp_constraints']} (dims: {summary['sdp_dimensions']})")
        print(f"  SOCP constraints: {summary['num_socp_constraints']} (dims: {summary['socp_dimensions']})")
        print(f"  Generated files: {', '.join(files_written)}")

    return ExportResult(nl=final_nl_path, col=final_col_path, cbf=cbf_path)


export_raposa_files = export_cbf_files
