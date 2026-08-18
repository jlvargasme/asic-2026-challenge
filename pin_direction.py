"""
Classify each of a leaf standard cell's own pins as an input or an output,
purely from geometry -- no "A/B/C are inputs, X is the output" naming
convention assumed -- then apply that to the per-instance pins found by
instance_pins.find_instance_pins(), renaming them "in_<name>" / "out_<name>"
(or "unknown_<name>" / "ambiguous_<name>" if a pin's net couldn't be
resolved cleanly).

The idea: trace a pin's net down through li1/licon into the leaf cell's own
transistors (reusing count_transistors.py, build_transistor.py and
trace_net.py exactly as tracer.py already does for one cell in isolation).
In static CMOS a net terminates on one of two kinds of transistor
terminal, and they mean opposite things:

  - a transistor GATE (poly) draws no DC current -- it's a capacitive
    control input, so the only reason a net would end there is that
    something *drives* it from outside. That's an input.
  - a transistor DRAIN/SOURCE diffusion node (one that isn't already tied
    to VPWR/VGND) is the low-impedance node a pull-up/pull-down stage
    pushes current onto to drive the net. That's an output.

This is genuinely derived from the instance's own layout connectivity, not
borrowed from the GDS pin-name convention -- the pin name is still used
afterwards (as the human-readable part of "in_A"/"out_X"), but not to
decide direction.

Caveat inherited from find_instance_pins(): a leaf cell's own transistor
graph never depends on *where* it's placed, so it's built once per
(gds_path, leaf_cell_name) and cached -- reused across every instance of
that cell type in the design instead of being recomputed per instance.
"""

import gdstk

from build_transistor import build_transistors
from cell_utils import (
    LI1_LAYER,
    LICON_LAYER,
    MCON_LAYER,
    MET1_LAYER,
    NWELL_LAYER,
    POLY_LAYER,
    label_diffusion_regions,
    load_cell,
)
from count_transistors import count_transistors
from instance_pins import find_instance_pins
from trace_net import NetTracer

_LEAF_CACHE = {}


def _build_leaf_graph(gds_path, leaf_cell_name):
    """Build (once, cached) the transistor-level connectivity graph for one
    leaf cell type, plus lookup tables of its own local li1/met1 polygons
    so a pin label's XY point can be mapped to a union-find node."""
    key = (gds_path, leaf_cell_name)
    if key in _LEAF_CACHE:
        return _LEAF_CACHE[key]

    cell = load_cell(gds_path, leaf_cell_name)
    gates = count_transistors(gds_path, leaf_cell_name)["gates"]
    labeled_regions = label_diffusion_regions(cell, gates)
    transistors = build_transistors(gates, labeled_regions)

    poly = cell.get_polygons(depth=None, layer=POLY_LAYER[0], datatype=POLY_LAYER[1])
    licon = cell.get_polygons(depth=None, layer=LICON_LAYER[0], datatype=LICON_LAYER[1])
    li1 = cell.get_polygons(depth=None, layer=LI1_LAYER[0], datatype=LI1_LAYER[1])
    mcon = cell.get_polygons(depth=None, layer=MCON_LAYER[0], datatype=MCON_LAYER[1])
    met1 = cell.get_polygons(depth=None, layer=MET1_LAYER[0], datatype=MET1_LAYER[1])
    nwell = cell.get_polygons(depth=None, layer=NWELL_LAYER[0], datatype=NWELL_LAYER[1])
    labels = cell.get_labels(depth=None)

    tracer = NetTracer(labeled_regions, transistors, poly, licon, li1, mcon, met1, nwell, labels=labels)

    graph = {
        "tracer": tracer,
        "labels": labels,
        "li1": list(zip(li1, [f"li1_{i}" for i in range(len(li1))])),
        "met1": list(zip(met1, [f"met1_{i}" for i in range(len(met1))])),
    }
    _LEAF_CACHE[key] = graph
    return graph


def classify_pin_direction(gds_path, leaf_cell_name, pin_name, texttype=5):
    """Return "input", "output", "ambiguous", or "unknown" for one named
    pin of a leaf cell, by tracing its net down to whatever it terminates
    on transistor-wise (see module docstring)."""
    graph = _build_leaf_graph(gds_path, leaf_cell_name)
    tracer = graph["tracer"]

    # a pin can be stamped more than once (e.g. one label per finger), so
    # gather every matching label and combine what all of them trace to,
    # rather than betting on the first one landing on a live polygon.
    matching_labels = [
        lbl for lbl in graph["labels"]
        if lbl.text == pin_name and lbl.texttype == texttype
    ]
    if not matching_labels:
        return "unknown"

    equivalents = set()
    matched_any = False
    for lbl in matching_labels:
        node_key = None
        # sky130 stamps pin labels on li1, not met1 -- but check met1 too
        # as a fallback in case a particular library/cell differs.
        for layer_name in ("li1", "met1"):
            for poly, key in graph[layer_name]:
                if gdstk.inside([lbl.origin], [poly])[0]:
                    node_key = key
                    break
            if node_key:
                break
        if node_key is None:
            continue
        matched_any = True
        equivalents.update(tracer.trace(node_key))

    if not matched_any:
        return "unknown"

    touches_gate = any(e.startswith("G") for e in equivalents)  # GN_i / GP_i
    touches_diff = any(e.startswith("D") for e in equivalents)  # DN_i / DP_i

    if touches_gate and touches_diff:
        return "ambiguous"  # touches both a gate and a drain -- rare, needs a human look
    if touches_gate:
        return "input"
    if touches_diff:
        return "output"
    return "unknown"  # isolated from the transistor graph entirely (e.g. VDD/VSS-only net)


def pin_name_for_net(gds_path, leaf_cell_name, real_label, texttype=5,
                      exclude=("VPWR", "VDD", "VGND", "VSS", "GND", "VPB", "VNB")):
    """Reverse of classify_pin_direction(): given one of NetTracer's own
    labels for a leaf cell (a transistor gate label like "GP_3", or a
    (post-find_root) diffusion label like "DN_2"/"VDD"), find the
    human-assigned pin name (e.g. "A") stamped on that same net in the
    cell's own GDS, if any.

    Returns (pin_name, direction) -- direction is whatever
    classify_pin_direction() would say for that pin name -- or (None,
    None) if this particular net isn't one of the cell's externally-named
    pins (e.g. it's an internal-only node between two logic stages, or a
    bare VDD/VSS net with no A/B/C/X-style label of its own).
    """
    graph = _build_leaf_graph(gds_path, leaf_cell_name)
    tracer = graph["tracer"]

    root = tracer._uf.find(real_label)
    net_polys = [
        poly for layer_name in ("li1", "met1")
        for poly, key in graph[layer_name]
        if tracer._uf.find(key) == root
    ]
    if not net_polys:
        return None, None

    for lbl in graph["labels"]:
        if lbl.texttype != texttype or lbl.text in exclude:
            continue
        if gdstk.inside([lbl.origin], net_polys)[0]:
            return lbl.text, classify_pin_direction(gds_path, leaf_cell_name, lbl.text, texttype=texttype)

    return None, None


def label_instance_pins(gds_path, parent_cell, reference, **find_pins_kwargs):
    """find_instance_pins() + input/output classification.

    Returns the same list of dicts find_instance_pins() does, with two
    keys added: "direction" ("input"/"output"/"ambiguous"/"unknown"/None)
    and "final_label" ("in_A", "out_X", "ambiguous_<name>",
    "unknown_<name>", or None if the pin itself had no matched GDS label
    to begin with).
    """
    pins = find_instance_pins(parent_cell, reference, **find_pins_kwargs)
    leaf_cell_name = reference.cell.name

    labeled = []
    for pin in pins:
        name = pin["label"]
        if name is None:
            labeled.append({**pin, "direction": None, "final_label": None})
            continue

        direction = classify_pin_direction(gds_path, leaf_cell_name, name)
        prefix = {"input": "in", "output": "out"}.get(direction, direction)
        labeled.append({**pin, "direction": direction, "final_label": f"{prefix}_{name}"})

    return labeled


if __name__ == "__main__":
    import sys

    gds_path = sys.argv[1] if len(sys.argv) > 1 else "warmup/04_final.gds"
    parent_name = sys.argv[2] if len(sys.argv) > 2 else "adder_demo"
    instance_cell_name = sys.argv[3] if len(sys.argv) > 3 else "sky130_fd_sc_hd__and3_2"
    instance_index = int(sys.argv[4]) if len(sys.argv) > 4 else 0

    library = gdstk.read_gds(gds_path)
    parent = next(c for c in library.cells if c.name == parent_name)
    labels = parent.get_labels(depth=None)

    matches = [
        r for r in parent.references
        if hasattr(r.cell, "name") and r.cell.name == instance_cell_name
    ]
    if not matches:
        raise SystemExit(f"no instance of {instance_cell_name!r} found inside {parent_name!r}")
    if instance_index >= len(matches):
        raise SystemExit(
            f"only {len(matches)} instance(s) of {instance_cell_name!r} "
            f"(asked for index {instance_index})"
        )
    ref = matches[instance_index]

    print(f"{len(matches)} instance(s) of {instance_cell_name!r} found; using index {instance_index}")
    print(f"instance: {ref.cell.name} @ {ref.origin}\n")

    for pin in label_instance_pins(gds_path, parent, ref, labels=labels):
        print(
            f"  {pin['final_label'] or '(unlabeled)':12s} "
            f"raw={pin['label']!r:6}  direction={pin['direction']}  "
            f"shapes={len(pin['polygons'])}"
        )