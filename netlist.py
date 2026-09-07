"""
netlist.py: load a yosys-normalized bit-level netlist from Verilog
produced by decompiler.py's own render_verilog() -- the shared
foundation backtrace.py (fan-in visualization) and cluster_solve.py
(per-cluster combinational solving) are both built on.

Why yosys instead of a hand-written Verilog parser: render_verilog()'s
own output is restricted but still real Verilog (operator precedence,
ternaries, concatenation, bit-selects, arithmetic operators) -- reusing
a mature, correct frontend is both less code and less risk than
re-deriving that parsing logic by hand. yosys's own JSON backend
(write_json) resolves every net down to a flat list of global bit IDs
and every cell's own connections to bit-ID lists with port directions --
exactly the graph structure both downstream consumers need, with zero
Verilog-syntax awareness required past this module.

Deliberately does NOT flatten the design: keeping module boundaries
intact is exactly the granularity both consumers want (cluster_type_N
instances as the natural node/unit, matching the RTL structure
chip_manipulation.py itself recovered), not raw gates.

Runs yosys via WSL (see HANDOFF.md / sim/run_*.sh's own account: a
native Windows/MSYS2 toolchain hits a real environment bug where nested
process spawning silently drops TMP/TEMP; WSL sidesteps the whole class
of bug) -- a local, no-root install under ~/yosys_local, the same
apt-get-download-then-dpkg-x approach sim/'s own verilator setup uses,
for the same reason (interactive sudo isn't available non-interactively
here).
"""

import json
import subprocess
import tempfile
from pathlib import Path

YOSYS_BIN_DIR = "$HOME/yosys_local/extracted/usr/bin"


def _to_wsl_path(windows_path):
    """C:\\Users\\x\\y.v -> /mnt/c/Users/x/y.v"""
    p = str(Path(windows_path).resolve())
    drive, rest = p.split(":", 1)
    return f"/mnt/{drive.lower()}{rest.replace(chr(92), '/')}"


def _run_wsl(command, timeout=120):
    """Run one shell command inside WSL Ubuntu, with the local yosys
    install on PATH. Raises RuntimeError (with yosys's own stdout/stderr
    attached) on a nonzero exit rather than letting a silent failure
    produce an empty/stale JSON file downstream."""
    full = f'export PATH={YOSYS_BIN_DIR}:$PATH; {command}'
    result = subprocess.run(
        ["wsl.exe", "-d", "Ubuntu", "-e", "bash", "-c", full],
        capture_output=True, text=True, timeout=timeout,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"yosys (via WSL) failed, exit {result.returncode}:\n{result.stdout}\n{result.stderr}"
        )
    return result.stdout


def load_netlist(verilog_path, top_module, out_dir=None):
    """Read `verilog_path` with yosys (hierarchy + proc only -- no
    flatten, no optimization passes, so the resulting netlist stays as
    close to decompiler.py's own rendered structure as yosys's frontend
    allows) and return its write_json output as a plain dict:
    {"modules": {module_name: {"ports": {...}, "cells": {...},
    "netnames": {...}}}} -- see yosys's own JSON backend documentation
    for the full schema; Module (below) wraps the fields this project
    actually reads.

    `out_dir` (default: the system temp dir, which is Windows-side and
    therefore visible to WSL too via /mnt/c/...) is where the
    intermediate JSON file is written; harmless to leave around, but
    cleaned up automatically for the default case.
    """
    wsl_v = _to_wsl_path(verilog_path)
    tmp_ctx = tempfile.TemporaryDirectory(dir=out_dir) if out_dir is None else None
    out_dir_path = Path(tmp_ctx.name) if tmp_ctx else Path(out_dir)
    out_dir_path.mkdir(parents=True, exist_ok=True)
    out_path = out_dir_path / f"{top_module}.netlist.json"
    try:
        wsl_out = _to_wsl_path(str(out_path))
        script = f"read_verilog {wsl_v}; hierarchy -top {top_module}; proc; write_json {wsl_out}"
        _run_wsl(f"yosys -p '{script}'")
        with open(out_path) as f:
            return json.load(f)
    finally:
        if tmp_ctx:
            tmp_ctx.cleanup()


class Module:
    """Thin accessors over one yosys JSON module record.

    Attributes:
        name: the module's own name (e.g. "cluster_type_5", or the top
            module's own name).
        ports: {port_name: {"direction": "input"|"output", "bits": [...]}}
        cells: {inst_name: {"type": cell_type, "port_directions": {...},
            "connections": {port_name: [bit_ids]}}} -- for a
            cluster_type_N instantiation, `type` is the OTHER module's
            own name (e.g. "cluster_type_5"); for a primitive gate,
            `type` is one of yosys's own internal cell types (e.g.
            "$and", "$add" -- see cell_solve.py's own YOSYS_CELL_OPS for
            the ones this project actually interprets).
        netnames: {net_name: {"bits": [bit_ids]}} -- every net-name
            alias for a bit-id group (a single physical net can have
            more than one name, e.g. a port name AND an internal bus
            name if they happen to coincide).
    """

    def __init__(self, name, data):
        self.name = name
        self.ports = data.get("ports", {})
        self.cells = data.get("cells", {})
        self.netnames = data.get("netnames", {})
        self._name_by_bit = {}
        for net_name, info in self.netnames.items():
            for i, bit in enumerate(info["bits"]):
                label = net_name if len(info["bits"]) == 1 else f"{net_name}[{i}]"
                # prefer a PORT's own name over an internal alias when a
                # bit has both (ports are this module's own public
                # interface, the more meaningful label for a human).
                if bit not in self._name_by_bit or net_name in self.ports:
                    self._name_by_bit[bit] = label

    def bit_name(self, bit):
        """Human-readable label for one bit ID (e.g. "A_10_bus[3]"), or
        the bit ID itself (as a string) if yosys recorded no name for it
        (an internal, unnamed intermediate signal)."""
        return self._name_by_bit.get(bit, f"<bit {bit}>")

    def port_bits(self, port_name):
        return self.ports[port_name]["bits"]

    def output_ports(self):
        return [n for n, p in self.ports.items() if p["direction"] == "output"]

    def input_ports(self):
        return [n for n, p in self.ports.items() if p["direction"] == "input"]


def modules(netlist_json):
    """{module_name: Module} for every module in a load_netlist() result."""
    return {name: Module(name, data) for name, data in netlist_json["modules"].items()}
