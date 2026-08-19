"""
Everything about one placed cell instance's I/O pins: finding them, and
classifying each as an input or an output.

Finding pins (find_instance_pins): a net that only exists to serve an
instance's own internal logic never has to leave the instance's own
footprint, but a real pin has to be reachable from outside -- so its
met1/via1/met2/via2 shape necessarily pokes out past the instance's
placement rectangle. Two things make this work reliably rather than by
luck:

1. The "footprint" has to come from an authoritative source, not from the
   very shapes you're testing -- Reference.bounding_box() gives the
   instance's placed extent (the referenced cell's own bounding box,
   transformed by this particular placement's origin/rotation), which is
   independent of whatever routing happens to reach it. Computing the
   footprint from the routing shapes themselves would be circular: it
   would auto-expand to contain every pin and nothing would ever look
   like it "crosses" it.

2. Not everything that crosses an instance's footprint is a signal I/O --
   VPWR/VGND rails cross every instance's left/right edges by design,
   since abutting standard cells share a continuous power rail. Those are
   filtered out by name using the same "VPWR"/"VGND" GDS labels used
   elsewhere in this project, not by shape, since shape-based heuristics
   for "is this a rail" are much less reliable than the ground truth
   already sitting in the file.

Classifying direction (LeafCellAnalyzer, label_instance_pins): purely from
geometry -- no "A/B/C are inputs, X is the output" naming convention
assumed. The idea: trace a pin's net down through li1/licon into the leaf
cell's own transistors (reusing transistor.py and net_trace.py). In static
CMOS a net terminates on one of two kinds of transistor terminal, and they
mean opposite things:

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
graph never depends on *where* it's placed, so LeafCellAnalyzer builds it
once per (gds_path, leaf_cell_name) and caches the instance -- reused
across every instance of that cell type in the design instead of being
recomputed per instance.
"""

import gdstk

from geometry import _extends_beyond, _overlaps
from gds_utils import SKY130, label_diffusion_regions, load_cell
from net_trace import NetTracer
from transistor import build_transistors, count_transistors
from union_find import UnionFind

DEFAULT_LAYERS = {
    # li1/mcon are included even though li1 essentially never itself pokes
    # past an instance's footprint -- sky130 stamps every pin's net label
    # on the li1 layer (not met1), so without li1 in the graph a label can
    # sit right on top of the correct net and still never get matched,
    # because the *polygon* the label falls inside (li1) was never part of
    # the union-find group in the first place. Pulling li1/mcon in lets
    # that li1 shape merge into the same group as the met1 track it
    # connects to (the one that actually extends beyond the footprint),
    # so the label becomes reachable without changing which groups count
    # as "a pin" -- li1-only groups still get dropped by the
    # extends_beyond check below, same as before.
    "li1": SKY130.li1,
    "mcon": SKY130.mcon,
    "met1": SKY130.met1,
    "via1": (68, 44),
    "met2": (69, 20),
    "via2": (69, 44),
}
# only physically-adjacent layer pairs can belong to the same net
ADJACENT_LAYER_PAIRS = {
    ("li1", "mcon"),
    ("mcon", "met1"),
    ("met1", "via1"),
    ("via1", "met2"),
    ("met2", "via2"),
}


def find_instance_pins(
    parent_cell,
    reference,
    layers=DEFAULT_LAYERS,
    labels=None,
    label_texttypes=(5,), #TODO: checl if this type is global or need to be removed
    vdd_names=("VPWR", "VDD", "VPB"),
    vss_names=("VGND", "VSS", "GND", "VNB"),
):
    """Detect and label the I/O pins of one placed instance.

    Args:
        parent_cell: the gdstk.Cell that contains `reference` (routing
            metal is pulled from here, flattened, since a routed design's
            met1/met2 typically isn't drawn inside the leaf cell itself).
        reference: the gdstk.Reference (instance) to find pins for --
            must be one of parent_cell.references.
        layers: dict of layer name -> (layer, datatype) for the routing
            stack to search. Defaults to sky130's met1/via1/met2/via2.
        labels: labels to use for naming pins (e.g. parent_cell.get_labels
            (depth=None)). If omitted, pins are returned unnamed.
        label_texttypes: only labels with one of these texttypes are used
            for naming. Default (5,) matches sky130's net/pin label
            convention; texttype 44 is typically the *instance name*
            label (e.g. "and3_2" at the instance origin), not a net name,
            and is excluded by default.
        vdd_names, vss_names: label text values that mean "this is a
            power rail, not a signal pin" -- these are dropped from the
            result even though they do cross the footprint by design.

    Returns:
        list of dicts, one per detected pin:
            {"label": str or None, "polygons": [gdstk.Polygon, ...]}
        "label" is the matched GDS pin/net label if one was found inside
        any of the pin's polygons, else None (a synthetic "pin_N" name is
        NOT invented here -- that's a judgment call left to the caller,
        since an unlabeled result is meaningfully different from a named
        one and callers may want to handle it differently).
    """
    footprint_box = reference.bounding_box()
    if footprint_box is None:
        raise ValueError("reference has no bounding box (unresolved/string cell reference?)")
    (fx0, fy0), (fx1, fy1) = footprint_box
    footprint = gdstk.rectangle((fx0, fy0), (fx1, fy1))

    # pull each layer's shapes once, flattened, from the parent
    layer_polys = {
        name: parent_cell.get_polygons(depth=None, layer=lay[0], datatype=lay[1])
        for name, lay in layers.items()
    }

    # seeds: every shape on these layers that touches this instance's
    # footprint at all -- candidates for "this instance's own pins"
    seeds = []  # (layer_name, polygon)
    for name, polys in layer_polys.items():
        for p in polys:
            if _overlaps(p, footprint):
                seeds.append((name, p))

    if not seeds:
        return []

    # union-find same-net grouping, but only across physically adjacent
    # layers (met1 can touch via1, via1 can touch met2, etc. -- met1
    # touching met2 directly would be a different, unrelated coincidence,
    # not a real via connection)
    uf = UnionFind()
    keys = [f"{name}_{i}" for i, (name, _) in enumerate(seeds)]
    for k in keys:
        uf.find(k)

    for a in range(len(seeds)):
        layer_a, poly_a = seeds[a]
        for b in range(a + 1, len(seeds)):
            layer_b, poly_b = seeds[b]
            if (layer_a, layer_b) not in ADJACENT_LAYER_PAIRS and (layer_b, layer_a) not in ADJACENT_LAYER_PAIRS:
                continue
            if _overlaps(poly_a, poly_b):
                uf.union(keys[a], keys[b])

    groups = {}
    for key, (_, poly) in zip(keys, seeds):
        groups.setdefault(uf.find(key), []).append(poly)

    # Restrict candidates to labels actually placed AT this
    # instance (origin inside its own footprint) before matching
    # by polygon -- sky130 stamps a local copy of every pin's
    # label (including VPWR/VGND) at each instance, so this is
    # enough to disambiguate. Without this restriction, a label
    # from some other, unrelated instance sharing the same
    # physically-continuous rail elsewhere in the row can also
    # satisfy the polygon containment test and give a nonsense
    # name (e.g. borrowing a neighboring cell's "A" pin label).
    local_labels = [
        lbl for lbl in labels
        if (not label_texttypes or lbl.texttype in label_texttypes)
        and fx0 <= lbl.origin[0] <= fx1 and fy0 <= lbl.origin[1] <= fy1
    ]

    pins = []
    for polys in groups.values():
        # a real pin has to be reachable from outside this instance --
        # if every shape in the group is fully contained, it's not an
        # I/O of THIS instance (most likely routing that merely passes
        # over it, or, per how these leaf cells are drawn, shouldn't
        # happen for a genuine signal net)
        if not any(_extends_beyond(p, footprint) for p in polys):
            continue

        name = None
        if labels:
            matched_names = [
                lbl.text for lbl in local_labels
                if gdstk.inside([lbl.origin], polys)[0]
            ]
            distinct = set(matched_names)
            name = matched_names[0] if matched_names else None
            if len(distinct) > 1 and name not in vdd_names and name not in vss_names:
                print(
                    f"warning: pin group {[p.bounding_box() for p in polys]} matched "
                    f"multiple differently-named local labels {sorted(distinct)!r} -- "
                    f"using {name!r}. This usually means two unrelated nets' shapes "
                    f"overlap in XY at different layers (gdstk.inside only checks 2D "
                    f"containment, not layer); verify this pin by hand."
                )

        if name in vdd_names or name in vss_names:
            continue  # power rail crossing the footprint by design, not a signal pin

        pins.append({"label": name, "polygons": polys})

    return pins


class LeafCellAnalyzer:
    """Transistor-level connectivity graph for one leaf cell type, plus
    lookup tables of its own local li1/met1 polygons so a pin label's XY
    point can be mapped to a union-find node.

    Cached per (gds_path, leaf_cell_name) via `for_cell()` -- a leaf cell's
    own layout never depends on where it's placed, so every instance of the
    same cell type reuses one analyzer instead of rebuilding the graph.
    """

    _cache = {}

    def __init__(self, gds_path, leaf_cell_name):
        self.gds_path = gds_path
        self.leaf_cell_name = leaf_cell_name

        cell = load_cell(gds_path, leaf_cell_name)
        gates = count_transistors(gds_path, leaf_cell_name)["gates"]
        labeled_regions = label_diffusion_regions(cell, gates)
        transistors = build_transistors(gates, labeled_regions)
        self.transistors = transistors

        poly = cell.get_polygons(depth=None, layer=SKY130.poly[0], datatype=SKY130.poly[1])
        licon = cell.get_polygons(depth=None, layer=SKY130.licon[0], datatype=SKY130.licon[1])
        li1 = cell.get_polygons(depth=None, layer=SKY130.li1[0], datatype=SKY130.li1[1])
        mcon = cell.get_polygons(depth=None, layer=SKY130.mcon[0], datatype=SKY130.mcon[1])
        met1 = cell.get_polygons(depth=None, layer=SKY130.met1[0], datatype=SKY130.met1[1])
        nwell = cell.get_polygons(depth=None, layer=SKY130.nwell[0], datatype=SKY130.nwell[1])
        self.labels = cell.get_labels(depth=None)

        self.tracer = NetTracer(labeled_regions, transistors, poly, licon, li1, mcon, met1, nwell, labels=self.labels)
        self._li1 = list(zip(li1, [f"li1_{i}" for i in range(len(li1))]))
        self._met1 = list(zip(met1, [f"met1_{i}" for i in range(len(met1))]))

    @classmethod
    def for_cell(cls, gds_path, leaf_cell_name):
        key = (gds_path, leaf_cell_name)
        if key not in cls._cache:
            cls._cache[key] = cls(gds_path, leaf_cell_name)
        return cls._cache[key]

    def classify_pin_direction(self, pin_name, texttype=5):
        """Return "input", "output", "ambiguous", or "unknown" for one
        named pin of this leaf cell, by tracing its net down to whatever
        it terminates on transistor-wise (see module docstring)."""
        # a pin can be stamped more than once (e.g. one label per finger),
        # so gather every matching label and combine what all of them
        # trace to, rather than betting on the first one landing on a
        # live polygon.
        matching_labels = [
            lbl for lbl in self.labels
            if lbl.text == pin_name and lbl.texttype == texttype
        ]
        if not matching_labels:
            return "unknown"

        equivalents = set()
        matched_any = False
        for lbl in matching_labels:
            node_key = None
            # sky130 stamps pin labels on li1, not met1 -- but check met1
            # too as a fallback in case a particular library/cell differs.
            for layer_polys in (self._li1, self._met1):
                for poly, key in layer_polys:
                    if gdstk.inside([lbl.origin], [poly])[0]:
                        node_key = key
                        break
                if node_key:
                    break
            if node_key is None:
                continue
            matched_any = True
            equivalents.update(self.tracer.trace(node_key))

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

    def pin_name_for_net(self, real_label, texttype=5,
                          exclude=("VPWR", "VDD", "VGND", "VSS", "GND", "VPB", "VNB")):
        """Reverse of classify_pin_direction(): given one of NetTracer's own
        labels for this leaf cell (a transistor gate label like "GP_3", or a
        (post-find_root) diffusion label like "DN_2"/"VDD"), find the
        human-assigned pin name (e.g. "A") stamped on that same net in the
        cell's own GDS, if any.

        Returns (pin_name, direction) -- direction is whatever
        classify_pin_direction() would say for that pin name -- or (None,
        None) if this particular net isn't one of the cell's externally-
        named pins (e.g. it's an internal-only node between two logic
        stages, or a bare VDD/VSS net with no A/B/C/X-style label of its
        own).
        """
        root = self.tracer._uf.find(real_label)
        net_polys = [
            poly for layer_polys in (self._li1, self._met1)
            for poly, key in layer_polys
            if self.tracer._uf.find(key) == root
        ]
        if not net_polys:
            return None, None

        for lbl in self.labels:
            if lbl.texttype != texttype or lbl.text in exclude:
                continue
            if gdstk.inside([lbl.origin], net_polys)[0]:
                return lbl.text, self.classify_pin_direction(lbl.text, texttype=texttype)

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
    analyzer = LeafCellAnalyzer.for_cell(gds_path, leaf_cell_name)

    labeled = []
    for pin in pins:
        name = pin["label"]
        if name is None:
            labeled.append({**pin, "direction": None, "final_label": None})
            continue

        direction = analyzer.classify_pin_direction(name)
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
