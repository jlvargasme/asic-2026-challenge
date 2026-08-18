"""
Detect and label the I/O pins of one placed cell *instance* inside a
routed parent cell (e.g. one standard-cell Reference inside a top-level
design), using the same idea you noticed by inspection: a net that only
exists to serve that instance's own internal logic never has to leave the
instance's own footprint, but a real pin has to be reachable from outside
-- so its met1/via1/met2/via2 shape necessarily pokes out past the
instance's placement rectangle.

Two things make this work reliably rather than by luck:

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
"""

import gdstk

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
    "li1": (67, 20),
    "mcon": (67, 44),
    "met1": (68, 20),
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


def _overlaps(poly_a, poly_b, precision=1e-3):
    return len(gdstk.boolean(poly_a, poly_b, "and", precision=precision)) > 0


def _extends_beyond(poly, footprint, precision=1e-3):
    """True if `poly` has area sticking out past `footprint`."""
    outside = gdstk.boolean(poly, footprint, "not", precision=precision)
    return sum(p.area() for p in outside) > 0


class _UnionFind:
    def __init__(self):
        self.parent = {}

    def find(self, x):
        self.parent.setdefault(x, x)
        while self.parent[x] != x:
            self.parent[x] = self.parent[self.parent[x]]
            x = self.parent[x]
        return x

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[ra] = rb


def find_instance_pins(
    parent_cell,
    reference,
    layers=DEFAULT_LAYERS,
    labels=None,
    label_texttypes=(5,),
    vdd_names=("VPWR", "VDD"),
    vss_names=("VGND", "VSS", "GND"),
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
    uf = _UnionFind()
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
            for lbl in local_labels:
                # gdstk.inside(points, polygons) checks each point against
                # the *set* of polygons and returns one bool per point --
                # passing all of `polys` at once (rather than looping and
                # wrapping each call in any()) both avoids re-testing the
                # same point per polygon and sidesteps a real footgun:
                # `any(gdstk.inside(...) for p in polys)` would iterate
                # over the returned *tuples* themselves (e.g. `(False,)`),
                # which are always truthy as non-empty containers -- that
                # silently matched the first label checked every time,
                # regardless of its actual position.
                if gdstk.inside([lbl.origin], polys)[0]:
                    name = lbl.text
                    break

        if name in vdd_names or name in vss_names:
            continue  # power rail crossing the footprint by design, not a signal pin

        pins.append({"label": name, "polygons": polys})

    return pins


if __name__ == "__main__":
    import sys

    # Usage:
    #   python3 instance_pins.py [gds_path] [parent_name] [instance_cell_name] [instance_index]
    #
    # `instance_cell_name` restricts the demo to instances of one specific
    # leaf cell type (e.g. "sky130_fd_sc_hd__and3_2") instead of just
    # grabbing whatever the first non-tap/decap reference happens to be.
    # `parent_cell` still has to be a cell that PLACES that leaf (e.g. the
    # routed top level, "adder_demo") -- find_instance_pins needs routing
    # that reaches in from outside the instance's own footprint, which a
    # leaf cell's own standalone definition doesn't have (its own met1 pin
    # shapes stop right at its own boundary, they don't stick out, since
    # nothing has been routed to them yet).
    gds_path = sys.argv[1] if len(sys.argv) > 1 else "warmup/04_final.gds"
    parent_name = sys.argv[2] if len(sys.argv) > 2 else "adder_demo"
    instance_cell_name = sys.argv[3] if len(sys.argv) > 3 else "sky130_fd_sc_hd__and3_2"
    instance_index = int(sys.argv[4]) if len(sys.argv) > 4 else 0

    library = gdstk.read_gds(gds_path)
    parent = next(c for c in library.cells if c.name == parent_name)
    labels = parent.get_labels(depth=None)

    if instance_cell_name:
        matches = [
            r for r in parent.references
            if hasattr(r.cell, "name") and r.cell.name == instance_cell_name
        ]
        if not matches:
            raise SystemExit(
                f"no instance of {instance_cell_name!r} found inside {parent_name!r}"
            )
        if instance_index >= len(matches):
            raise SystemExit(
                f"only {len(matches)} instance(s) of {instance_cell_name!r} in "
                f"{parent_name!r} (asked for index {instance_index})"
            )
        ref = matches[instance_index]
        print(f"{len(matches)} instance(s) of {instance_cell_name!r} found; using index {instance_index}")
    else:
        # demo: find pins for the first non-tap/decap/via standard-cell instance
        skip_prefixes = ("sky130_fd_sc_hd__decap", "sky130_fd_sc_hd__tapvpwrvgnd", "VIA_")
        ref = next(
            r for r in parent.references
            if hasattr(r.cell, "name") and not r.cell.name.startswith(skip_prefixes)
        )

    print(f"instance: {ref.cell.name} @ {ref.origin}  bbox={ref.bounding_box()}")

    pins = find_instance_pins(parent, ref, labels=labels)
    print(f"found {len(pins)} pin(s):")
    for pin in pins:
        bboxes = [p.bounding_box() for p in pin["polygons"]]
        print(f"  {pin['label'] or '(unlabeled)'}: {len(pin['polygons'])} shape(s), bboxes={bboxes}")