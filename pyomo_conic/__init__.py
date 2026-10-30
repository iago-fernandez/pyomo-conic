"""
pyomo-conic: Conic optimization modeling and CBF export for Pyomo.

Provides first-class Semidefinite Programming (SDP) and Second-Order Cone
Programming (SOCP) modeling extensions with automated affine decomposition
and Conic Benchmark Format (CBF) export.
"""

from .constraints import (
    ConicContainer,
    SDPConstraint,
    SOCPConstraint,
    add_sdp_constraint,
    add_socp_cone,
    add_socp_constraint,
    get_conic_container,
)
from .exporter import export_cbf_files, export_raposa_files

__version__ = "0.1.0"

__all__ = [
    "ConicContainer",
    "SDPConstraint",
    "SOCPConstraint",
    "add_sdp_constraint",
    "add_socp_cone",
    "add_socp_constraint",
    "export_cbf_files",
    "export_raposa_files",
    "get_conic_container",
]
