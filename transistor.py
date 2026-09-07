"""
Find and build Transistor objects from a GDS cell's poly/diffusion layers.

GDSII has no native "transistor" object -- a MOSFET is inferred from where
the gate (poly) layer crosses the active/diffusion layer. This follows the
SkyWater sky130 GDS layer map:

    nwell : 64/20
    diff  : 65/20   (the *same* layer's 65/44 datatype is "tap" -- a
                     well/substrate strap, not a transistor channel, and
                     is deliberately excluded)
    poly  : 66/20

Each disjoint polygon in (poly AND diff) is counted as one transistor gate.
For a folded multi-finger device this means each finger is counted
separately (N fingers -> N transistors), which matches how most
extraction/LVS tools report individual channel regions.

`count_transistors()` only locates the gate polygons and tallies nmos/pmos
counts. `build_transistors()` takes those gate polygons plus a cell's
labeled source/drain diffusion regions (gds_utils.label_diffusion_regions)
and matches each gate to the regions it touches, producing a Transistor
per gate.

Assumes a Manhattan (axis-aligned) layout, which covers standard planar
CMOS -- each polygon edge is either purely horizontal or purely vertical.
A gate polygon is matched to the labeled diffusion regions it touches by
checking, for every edge of the gate against every edge of every labeled
region, whether the two edges are collinear and their overlapping span has
positive length (geometry._touching_regions).

`labeled_regions` labels are expected to follow the "DN_i" / "DP_i"
convention (D = diffusion, N/P = NMOS/PMOS, i = index) -- the transistor's
type is inferred from the N/P of whichever regions it touches. Gate labels
are generated with the mirrored convention "GN_i" / "GP_i", where `i` is
the gate's position in the `gates` list (not tied to the diffusion index).

Source vs. drain is geometrically arbitrary (real source/drain roles
depend on circuit biasing, not layout), so the two touching regions are
just ordered deterministically by centroid (x, then y) -- the lower one
is called "source", the higher one "drain". Swap them if your convention
runs the other way.
"""

from dataclasses import dataclass, field
from enum import Enum

import gdstk

from geometry import _touching_regions
from gds_utils import SKY130


class TransistorType(Enum):
    Invalid = 0
    NMOS = 1
    PMOS = 2


@dataclass
class Transistor:
    kind: TransistorType
    gate_label: str
    source_label: str = None
    drain_label: str = None
    gate: gdstk.Polygon = None
    extra_region_labels: list = field(default_factory=list)  # matches beyond source/drain


def count_transistors(
    gds_path,
    cell_name,
    layers=SKY130,
    precision=1e-3,
    min_area=0.0,
):
    """Count transistors in `cell_name` inside `gds_path`.

    Args:
        gds_path: path to the .gds/.gds.gz/.oas file.
        cell_name: name of the cell to inspect. The cell's full reference
            hierarchy (all sub-cells it instantiates, at any depth) is
            included -- you do not need to flatten the GDS first.
        layers: gds_utils.Layers instance. Defaults to sky130's layer
            numbers; override this if your PDK differs.
        precision: precision (in the library's user units) passed to
            gdstk.boolean for the poly/diff intersection.
        min_area: discard intersection regions smaller than this area
            (same units as the GDS, usually um^2) -- useful for dropping
            slivers produced by rounding at abutting-shape edges.

    Returns:
        dict with keys:
            "total": total transistor (gate/finger) count
            "nmos":  gates whose area is NOT substantially inside an nwell
            "pmos":  gates whose area IS substantially inside an nwell
            "gates": list of gdstk.Polygon for the raw poly/diff overlaps
                     (useful for debugging / visualizing what was counted)
    """
    library = gdstk.read_gds(gds_path)
    cell = next((c for c in library.cells if c.name == cell_name), None)
    if cell is None:
        available = ", ".join(sorted(c.name for c in library.cells))
        raise ValueError(
            f"Cell {cell_name!r} not found in {gds_path!r}. "
            f"Available cells: {available}"
        )

    # depth=None recurses through every reference in the cell's hierarchy
    # and applies each reference's transform, so this also catches
    # transistors defined inside instantiated sub-cells.
    poly = cell.get_polygons(depth=None, layer=layers.poly[0], datatype=layers.poly[1])
    diff = cell.get_polygons(depth=None, layer=layers.diff[0], datatype=layers.diff[1])
    nwell = cell.get_polygons(depth=None, layer=layers.nwell[0], datatype=layers.nwell[1])

    gates = gdstk.boolean(poly, diff, "and", precision=precision)
    if min_area > 0:
        gates = [g for g in gates if g.area() >= min_area]

    nmos = pmos = 0
    for gate in gates:
        overlap = gdstk.boolean(gate, nwell, "and", precision=precision)
        overlap_area = sum(p.area() for p in overlap)
        if overlap_area == gate.area():
            pmos += 1
        else:
            nmos += 1

    return {
        "total": len(gates),
        "nmos": nmos,
        "pmos": pmos,
        "gates": gates,
    }


def build_transistors(gates, labeled_regions):
    """Build a Transistor per gate polygon.

    Args:
        gates: list of gdstk.Polygon, each one transistor gate/channel
            (e.g. the poly/diff overlap regions from count_transistors).
        labeled_regions: list of (gdstk.Polygon, label) tuples for the
            source/drain diffusion regions, labels following "DN_i" /
            "DP_i".

    Returns:
        list of Transistor, one per entry in `gates`, in the same order.
    """
    transistors = []
    for i, gate in enumerate(gates):
        matches = _touching_regions(gate, labeled_regions)

        kind_letters = {label[1] for label, _ in matches}
        if len(kind_letters) > 1:
            raise ValueError(
                f"gate {i} touches regions of conflicting type: "
                f"{[label for label, _ in matches]}"
            )
        kind_letter = next(iter(kind_letters), None)
        kind = {"N": TransistorType.NMOS, "P": TransistorType.PMOS}.get(kind_letter)

        labels = [label for label, _ in matches]
        # arbitrarily pick a source/drain since that does not matter much
        source_label = labels[0] if len(labels) >= 1 else None
        drain_label = labels[1] if len(labels) >= 2 else None
        extra = labels[2:]

        if len(labels) != 2:
            print(f"warning: gate {i} touches {len(labels)} labeled region(s): {labels}")

        transistors.append(
            Transistor(
                kind=kind,
                gate_label=f"G{kind_letter or '?'}_{i}",
                source_label=source_label,
                drain_label=drain_label,
                gate=gate,
                extra_region_labels=extra,
            )
        )
    return transistors


# GDS coordinates for sky130 layouts are in micrometers; PySpice's w/l
# parameters want meters.
_GDS_UNIT_TO_METERS = 1e-6


def _gate_dimensions(transistor):
    """Derive (length, width) in meters from the gate/diff overlap
    polygon's bounding box -- its two edge lengths ARE the physical
    channel length and width, in whichever order the gate happens to be
    drawn. The shorter edge is assumed to be the channel length (typically
    the process minimum) and the longer edge the channel width (drive
    strength), matching how standard-cell gates are actually drawn.

    Returns None if `transistor.gate` is unavailable.
    """
    if transistor.gate is None:
        return None
    (x0, y0), (x1, y1) = transistor.gate.bounding_box()
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    length, width = sorted((dx, dy))
    return length * _GDS_UNIT_TO_METERS, width * _GDS_UNIT_TO_METERS


def transistor_to_pyspice(circuit, transistor, gate_net, source_net, drain_net,
                           bulk_net=None, nmos_model="NMOS", pmos_model="PMOS",
                           length=150e-9, width=420e-9, name=None):
    """Add one MOSFET element to a PySpice `circuit` for `transistor`.

    A Transistor's own gate_label/source_label/drain_label are internal
    layout labels, not usable as circuit net names on their own -- two
    transistors sharing a diffusion region need to land on the *same* net,
    which requires net_trace.NetTracer's electrical-equivalence resolution
    first. So the caller supplies the already-resolved net name for each
    terminal (see cell.Cell._build_circuit for how those are derived from
    a full cell's transistor list).

    Args:
        circuit: a PySpice Circuit to add the MOSFET to.
        transistor: a Transistor with a resolved NMOS/PMOS kind.
        gate_net, source_net, drain_net: net names (strings, or any value
            PySpice accepts as a node) for the three terminals.
        bulk_net: net for the body/substrate terminal. Defaults to "VSS"
            for NMOS / "VDD" for PMOS -- the standard single-well
            assumption for a self-contained standard cell (this project
            doesn't track a separate substrate/nwell-tap net per
            transistor, so there's no finer-grained connection to make).
        nmos_model, pmos_model: names of the .model cards `circuit`
            already has defined for the two device types.
        length, width: fallback channel length/width in meters, used only
            when `transistor.gate` is unavailable; otherwise both are
            derived from the gate polygon's own bounding box.
        name: element name PySpice registers this MOSFET under within
            `circuit`. Defaults to transistor.gate_label, which is fine
            for a single cell's own circuit (cell.Cell._build_circuit),
            but transistor.gate_label is only unique WITHIN one leaf
            cell's own transistor list (e.g. "GP_0") -- a circuit
            combining transistors from several placed instances of the
            same leaf cell type (clustering.py's
            build_cluster_pyspice_circuit) needs the caller to pass an
            instance-qualified name instead, or PySpice raises
            "Element name ... is already defined" the moment a second
            instance of that cell type adds its own "GP_0".

    Returns:
        The PySpice Mosfet element that was added to `circuit`.
    """
    if transistor.kind == TransistorType.NMOS:
        model, default_bulk = nmos_model, "VSS"
    elif transistor.kind == TransistorType.PMOS:
        model, default_bulk = pmos_model, "VDD"
    else:
        raise ValueError(f"transistor {transistor.gate_label} has no resolved NMOS/PMOS kind")

    dims = _gate_dimensions(transistor)
    if dims is not None:
        length, width = dims

    return circuit.MOSFET(
        name if name is not None else transistor.gate_label,
        drain_net, gate_net, source_net, bulk_net or default_bulk,
        model=model, l=length, w=width,
    )


def transistor_to_z3(transistor, gate_var, source_var, drain_var):
    """Model `transistor` as an ideal digital switch and return the single
    z3 constraint for it: an NMOS conducts (shorts source to drain) while
    its gate is high, a PMOS conducts while its gate is low, and while
    "off" a transistor asserts nothing (the two terminals are left free to
    differ, exactly like an open switch).

    This is a switch-level abstraction, not an analog model -- it doesn't
    capture threshold voltages, drive strength, or timing the way
    transistor_to_pyspice()'s SPICE element does. It's meant for asking
    purely logical questions about a cell (what does it compute, which
    inputs make the output 1) with z3 rather than simulating it, exactly
    like cell.Cell.simulate_z3()/find_inputs_for_output() do.

    Args:
        transistor: a Transistor with a resolved NMOS/PMOS kind.
        gate_var, source_var, drain_var: z3 BoolRef for the three
            terminals' nets (see cell.Cell._z3_net_var for how a whole
            cell's transistors share one variable per electrical net).

    Returns:
        A z3 BoolRef: Implies(gate_var, source_var == drain_var) for
        NMOS, Implies(Not(gate_var), source_var == drain_var) for PMOS.
    """
    import z3

    if transistor.kind == TransistorType.NMOS:
        condition = gate_var
    elif transistor.kind == TransistorType.PMOS:
        condition = z3.Not(gate_var)
    else:
        raise ValueError(f"transistor {transistor.gate_label} has no resolved NMOS/PMOS kind")

    return z3.Implies(condition, source_var == drain_var)


# python ./transistor.py ./warmup/04_final.gds sky130_fd_sc_hd__and3_2
if __name__ == "__main__":
    import sys
    from gds_utils import label_diffusion_regions, load_cell

    gds_file = sys.argv[1] if len(sys.argv) > 1 else "./warmup/04_final.gds"
    # sky130_fd_sc_hd__and4bb_2
    cell_name = sys.argv[2] if len(sys.argv) > 2 else "sky130_fd_sc_hd__and3_2"

    result = count_transistors(gds_file, cell_name)
    cell = load_cell(gds_file, cell_name)
    labeled_regions = label_diffusion_regions(cell, result["gates"])

    for t in build_transistors(result["gates"], labeled_regions):
        print(t.kind, t.gate_label, t.source_label, t.drain_label)
