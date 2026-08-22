"""
Chip: a full routed top-level design -- every placed standard-cell logic
instance, wired together into one chip-wide netlist by tracing the
routing fabric (li1 through met5, the full metal stack a real routed
design like this project's adder_demo.gds actually uses) that connects
them, instead of just each instance's own local pin names in isolation.

Why this needs its own net-tracing pass instead of reusing pin.py's
find_instance_pins() per instance: find_instance_pins() builds a *fresh*
union-find graph from scratch, scoped to whatever crosses ONE instance's
footprint, every time it's called -- there's deliberately no shared state
between calls, so it can't tell you whether instance A's output and
instance B's input are the same physical net (only that each one, on its
own, looks like a pin). A chip-wide netlist needs the opposite: ONE
union-find graph over every routing shape in the whole top cell, built
once, so two different instances' pins that are actually the same wire
resolve to the same global net name.

Scale: a real routed design has on the order of 10,000+ routing-layer
polygons (adder_demo.gds: ~10,300). The brute-force "test every pair for
overlap" approach net_trace.py/pin.py use elsewhere is fine for one leaf
cell's few dozen shapes, but is O(n^2) there -- roughly 1e8 geometric
checks here, impractically slow. _SpatialGrid below buckets shapes into a
uniform grid sized so each bucket holds a handful of shapes, so only
shapes that are actually near each other ever get a real (and much more
expensive) gdstk.boolean overlap test.

Non-logic references (VIA_* metal-stitching cells, decap, welltap cells)
are skipped when building `instances` -- not by name, but because
_is_logic_cell() finds, purely from their own geometry, that none of
their pins resolve to anything but a power rail (see its docstring).
They're just part of the routing fabric that carries nets between real
logic instances, and still contribute their own shapes to the net graph
either way.

Once every instance's pins are resolved to global nets, `Chip._levelize()`
walks the design breadth-first from its own primary inputs -- the same
net graph, just traversed instance-to-instance instead of queried by
name: start from instances with no internally-driven input net, then
whatever instances their outputs reach, and so on -- grouping instances
into `levels` by how many hops of logic separate them from an input.
Anything that never becomes reachable this way (typically a register on a
combinational feedback loop -- see `unresolved`'s own docs) is reported
separately rather than stalling the traversal forever.
"""

from collections import defaultdict

import gdstk

from cell import Cell, _is_logic_cell
from gds_utils import SKY130, load_cell
from geometry import ADJACENT_LAYER_PAIRS, _overlaps, _touching
from union_find import UnionFind

PIN_TEXTTYPE = 5

# The full metal stack this project's example routed design actually
# uses (li1 through met5) -- pin.py's DEFAULT_LAYERS only goes up to
# met2/via2, which is enough for one leaf cell's own internal routing but
# would silently treat chip-level nets that run on met3+ as disconnected.
ROUTING_LAYERS = {
    "li1": SKY130.li1,
    "mcon": SKY130.mcon,
    "met1": SKY130.met1,
    "via1": (68, 44),
    "met2": (69, 20),
    "via2": (69, 44),
    "met3": (70, 20),
    "via3": (70, 44),
    "met4": (71, 20),
    "via4": (71, 44),
    "met5": (72, 20),
}

class _SpatialGrid:
    """Uniform grid index over a list of (key, layer_name, poly) shapes.

    Both jobs Chip needs -- "which pairs of shapes might overlap" and
    "which shape contains this point" -- would otherwise mean scanning
    every shape. Bucketing by bounding box turns each into "scan the
    handful of shapes near here" instead, which is what actually makes
    tracing a whole chip's routing tractable.
    """

    def __init__(self, shapes, target_occupancy=4):
        self.shapes = shapes

        boxes = []
        x_min = y_min = float("inf")
        x_max = y_max = float("-inf")
        for _, _, poly in shapes:
            (x0, y0), (x1, y1) = poly.bounding_box()
            boxes.append((x0, y0, x1, y1))
            x_min, y_min = min(x_min, x0), min(y_min, y0)
            x_max, y_max = max(x_max, x1), max(y_max, y1)

        self.x_min, self.y_min = x_min, y_min
        width, height = max(x_max - x_min, 1e-6), max(y_max - y_min, 1e-6)
        # pitch chosen so the grid has roughly len(shapes)/target_occupancy
        # cells spread over the design's bounding box -- i.e. ~target_occupancy
        # shapes registered per cell on average.
        n_cells = max(len(shapes) / target_occupancy, 1)
        self.pitch = max((width * height / n_cells) ** 0.5, 1e-6)

        self.buckets = defaultdict(list)
        for i, (x0, y0, x1, y1) in enumerate(boxes):
            for cell in self._cell_range(x0, y0, x1, y1):
                self.buckets[cell].append(i)

    def _cell(self, x, y):
        return int((x - self.x_min) // self.pitch), int((y - self.y_min) // self.pitch)

    def _cell_range(self, x0, y0, x1, y1):
        ix0, iy0 = self._cell(x0, y0)
        ix1, iy1 = self._cell(x1, y1)
        for ix in range(ix0, ix1 + 1):
            for iy in range(iy0, iy1 + 1):
                yield ix, iy

    def candidate_pairs(self):
        """Yield index pairs into `shapes` that share a grid cell -- a
        superset of the pairs that could actually overlap."""
        seen = set()
        for indices in self.buckets.values():
            for a in range(len(indices)):
                for b in range(a + 1, len(indices)):
                    i, j = indices[a], indices[b]
                    pair = (i, j) if i < j else (j, i)
                    if pair not in seen:
                        seen.add(pair)
                        yield pair

    def near(self, x, y):
        """Indices of shapes registered in the same grid cell as (x, y)."""
        return self.buckets.get(self._cell(x, y), [])

    def near_box(self, x0, y0, x1, y1):
        """Indices of shapes registered in any grid cell (x0, y0)-(x1, y1)
        touches -- the bounding-box analogue of near(), for matching a
        whole polygon (e.g. one pin's transformed routing shape) instead
        of a single point."""
        seen = set()
        result = []
        for cell in self._cell_range(x0, y0, x1, y1):
            for i in self.buckets.get(cell, []):
                if i not in seen:
                    seen.add(i)
                    result.append(i)
        return result


class Instance:
    """One placed standard-cell logic instance inside a Chip.

    Attributes:
        reference: the gdstk.Reference this instance is placed by.
        cell: the cell.Cell electrical model for this instance's leaf
            type (shared/cached across every instance of the same leaf
            type -- see cell.Cell/pin.LeafCellAnalyzer's own caching).
        index: this instance's position among placements of the same leaf
            cell type (0-based, in the order references were found) --
            the same role instance_index plays elsewhere in this project.
        global_pins: {local_pin_name: global_net_name}, for every one of
            this instance's own named pins (cell.input_labels +
            cell.output_labels) Chip could actually resolve to a net.
    """

    def __init__(self, reference, cell, index, global_pins):
        self.reference = reference
        self.cell = cell
        self.index = index
        self.global_pins = global_pins

    def __repr__(self):
        return f"Instance({self.cell.cell_name}#{self.index} @ {self.reference.origin}, pins={self.global_pins})"


class Chip:
    """A full routed top-level design, wired into one chip-wide netlist.

    Args:
        gds_file: path to the .gds file.
        top_cell_name: name of the routed top cell (e.g. "adder_demo").
            If omitted, the library's single top-level cell is used (see
            gds_utils.load_cell()).

    Attributes:
        instances: list of Instance, one per placed logic-cell reference
            (VIA_*/decap/tap references are skipped -- see module
            docstring). Each Instance.global_pins value is a real GDS
            net/pin label if the chip-wide net it's on has one anywhere
            along its routing, else a synthetic "net_<n>".
        levels: list of lists of Instance, grouped by a breadth-first
            traversal from the chip's own primary inputs -- level 0 is
            every instance with no input net driven by another instance
            in this chip (so each of its input nets is either a true
            top-level port, or one Chip couldn't resolve a driver for);
            level N is every instance whose input nets are all covered by
            levels 0..N-1's outputs. See _levelize().
        unresolved: instances that never became "ready" in that
            traversal -- typically ones on a sequential feedback loop
            (e.g. a register's D input fed, through combinational logic,
            by that same register's own Q output), which breaks the
            "wait until every input net has a resolved driver" rule no
            matter how far the traversal runs. Rather than looping
            forever or silently dropping them, they're reported here
            instead of being placed in `levels`.
    """

    def __init__(self, gds_file, top_cell_name=None):
        self.gds_file = gds_file
        self.top_cell = load_cell(gds_file, top_cell_name)
        self.labels = self.top_cell.get_labels(depth=None)

        self._uf, self._shapes, self._grid = self._build_net_graph()
        self._root_labels = self._index_labels()
        self._net_names = {}
        self._used_names = set()

        self.instances = self._build_instances()
        self.levels, self.unresolved = self._levelize()

    def _build_net_graph(self):
        """Union-find over every routing shape in the top cell.

        A same-layer pair is unioned on overlap OR edge-touch; a
        cross-layer pair only if it's one of geometry.ADJACENT_LAYER_PAIRS,
        and only on overlap (a via/contact is a real 2D square that must
        overlap both the layer above and below it -- two layers merely
        touching edge-to-edge isn't a via). Same-layer self-connection is
        not an optional extra: sky130 routinely draws one physical wire as
        several abutting or overlapping same-layer polygons (e.g. multiple
        rectangles stitched together at a bend, or a routing tool emitting
        overlapping segments for one net), and nothing in a purely
        cross-layer adjacency chain ever reunites two same-layer records
        with each other. Without this, such a net silently splits into
        multiple disconnected union-find groups -- found in practice on
        warmup/04_final.gds, where a DFF's Q output (labeled net) and an
        AND gate's A input (also labeled) turned out to be the same
        physical met1 wire, split into 3 separate groups by exactly this
        gap, leaving the AND gate's input looking like an undriven,
        neighborless net. net_trace.py's NetTracer already had to solve
        this identical problem for one leaf cell's own poly/li1/met1 (see
        its module docstring and connect_self()) -- this is the same fix,
        applied to the chip-wide routing graph instead.
        """
        layer_polys = {
            name: self.top_cell.get_polygons(depth=None, layer=lay[0], datatype=lay[1])
            for name, lay in ROUTING_LAYERS.items()
        }
        shapes = [
            (f"{name}_{i}", name, poly)
            for name, polys in layer_polys.items()
            for i, poly in enumerate(polys)
        ]

        grid = _SpatialGrid(shapes)
        uf = UnionFind()
        for key, _, _ in shapes:
            uf.find(key)

        for i, j in grid.candidate_pairs():
            key_a, layer_a, poly_a = shapes[i]
            key_b, layer_b, poly_b = shapes[j]

            if layer_a == layer_b:
                connected = _overlaps(poly_a, poly_b) or _touching(poly_a, poly_b)
            elif (layer_a, layer_b) in ADJACENT_LAYER_PAIRS or (layer_b, layer_a) in ADJACENT_LAYER_PAIRS:
                connected = _overlaps(poly_a, poly_b)
            else:
                continue

            if connected:
                uf.union(key_a, key_b)

        return uf, shapes, grid

    def _index_labels(self):
        """Precompute, once via the spatial grid, {root: {label texts}} --
        which chip-wide net (union-find root) each texttype-5 label sits
        on, used only to name a net from whatever label(s) it has (see
        _net_name) if one exists. Purely cosmetic: _net_name() already
        falls back to a synthetic "net"/"net#1"/... name when a root has
        no label at all, so a design with few or no top-level labels (the
        actual reverse-engineering case -- see chip.Instance.global_pins'
        docstring) still gets a distinct, stable name per net, just not a
        human-meaningful one. This is NOT used to determine which
        instances share a net -- that's Chip._net_root_for_polygon(),
        purely geometric -- only to decide what to print for one once it's
        already been found.
        """
        root_labels = defaultdict(set)
        for lbl in self.labels:
            if lbl.texttype != PIN_TEXTTYPE:
                continue
            x, y = lbl.origin
            for i in self._grid.near(x, y):
                key, _, poly = self._shapes[i]
                if gdstk.inside([lbl.origin], [poly])[0]:
                    root_labels[self._uf.find(key)].add(lbl.text)
                    break
        return root_labels

    def _net_name(self, root):
        """Resolve a net-graph union-find root to a global net name: a
        real GDS label if this net has one anywhere along its routing,
        else "net". Sky130 stamps each instance's own *local* pin name
        (e.g. "A") as a label at that instance -- since practically every
        cell type has its own "A" input, that same text shows up on many
        electrically unrelated nets across a whole chip, so a label alone
        is not a unique identifier. This is still guaranteed to return a
        different string for a different root: if the chosen name is
        already in use by some other net, it's suffixed "#1", "#2", ...
        until it isn't -- a real 1:1 mapping from root to name, which
        matters because callers (_levelize() in particular) use these
        names as dict keys to mean "the same net"."""
        if root in self._net_names:
            return self._net_names[root]

        matches = self._root_labels.get(root, set())
        if matches:
            base = sorted(matches)[0]
            if len(matches) > 1:
                print(f"warning: net resolves to {len(matches)} distinct labels {sorted(matches)!r}; using {base!r}")
        else:
            base = "net"

        name, suffix = base, 0
        while name in self._used_names:
            suffix += 1
            name = f"{base}#{suffix}"
        self._used_names.add(name)

        self._net_names[root] = name
        return name

    def _net_root_for_polygon(self, poly):
        """Chip-wide union-find root for whatever routing shape `poly`
        (already transformed into chip/global coordinates) overlaps, or
        None if it doesn't overlap anything registered in the chip-wide
        routing graph. Purely geometric -- the replacement for looking up
        a per-instance net label at this same spot."""
        (x0, y0), (x1, y1) = poly.bounding_box()
        for i in self._grid.near_box(x0, y0, x1, y1):
            key, _, shape = self._shapes[i]
            if _overlaps(poly, shape):
                return self._uf.find(key)
        return None

    def _instance_global_pins(self, reference, cell):
        """{local_pin_name: global_net_name} for every one of `cell`'s own
        named pins this instance's *placement* actually connects to a
        chip-wide net.

        Deliberately does NOT look for a per-instance net/pin label
        stamped at this placement (the way an earlier version of this
        method did, and the way pin.py's find_instance_pins() still does)
        -- that label is the routed DESIGN's own naming for this specific
        net, exactly the kind of internal name a "final manufacturable"
        GDS is meant to have stripped (see this project's own README on
        warmup/04_final.gds vs. the earlier flow stages), and exactly what
        a reverse engineer isn't supposed to get to read off directly.

        Instead: `cell._analyzer.pin_local_polygons(pin_name)` gives this
        pin's own routing shape(s) in the LEAF CELL's local coordinate
        system -- derived from the *standard cell library's* own internal
        labels (see LeafCellAnalyzer's docstring for why that's a
        different, non-secret category of label: PDK/library data true
        for every design built from this cell, not this specific circuit's
        own net naming). Each candidate polygon is transformed through
        this instance's own placement (origin/rotation/reflection --
        exactly what turns a leaf cell's local geometry into this
        instance's actual position on the die) and matched directly
        against the chip-wide routing graph via _net_root_for_polygon --
        so two instances' pins land on the same global net because their
        *routing shapes actually geometrically connect*, never because
        they happen to share a label.
        """
        global_pins = {}
        # sorted(), not a bare set union: iteration order here feeds
        # _net_name()'s "first root seen claims the plain base name, later
        # ones get #1/#2/..." suffix assignment, and Python's set iteration
        # order for strings is randomized per-process (PYTHONHASHSEED) --
        # without this, which arbitrary suffix a given net ends up with
        # would vary between runs of the exact same GDS.
        for pin_name in sorted(set(cell.input_labels) | set(cell.output_labels)):
            root = None
            for local_poly in cell._analyzer.pin_local_polygons(pin_name):
                # .copy() first: .transform() mutates in place, and
                # local_poly is a shape owned by the (cached, shared-
                # across-every-instance-of-this-cell-type) LeafCellAnalyzer
                # -- transforming it directly would corrupt every other
                # instance's own view of this same leaf cell.
                global_poly = local_poly.copy().transform(
                    magnification=reference.magnification,
                    x_reflection=reference.x_reflection,
                    rotation=reference.rotation,
                    translation=reference.origin,
                )
                root = self._net_root_for_polygon(global_poly)
                if root is not None:
                    break
            if root is not None:
                global_pins[pin_name] = self._net_name(root)
        return global_pins

    def _build_instances(self):
        counts = defaultdict(int)
        instances = []
        for reference in self.top_cell.references:
            if not hasattr(reference.cell, "name"):
                continue
            leaf_cell_name = reference.cell.name

            try:
                cell = Cell(self.gds_file, leaf_cell_name)
            except Exception as e:
                print(f"warning: skipping {leaf_cell_name} -- couldn't build its Cell model ({e!r})")
                continue

            if not _is_logic_cell(cell):
                print(f"skipping non-logic cell {leaf_cell_name}")
                continue

            index = counts[leaf_cell_name]
            counts[leaf_cell_name] += 1

            global_pins = self._instance_global_pins(reference, cell)
            instances.append(Instance(reference, cell, index, global_pins))
        return instances

    def _levelize(self):
        """Breadth-first traversal from the chip's own primary inputs --
        Kahn's algorithm for topological sort, applied to instances
        rather than individual nets. See the `levels`/`unresolved`
        attribute docs for what each result means.
        """
        drivers = defaultdict(list)   # net -> instances with an output pin on it
        consumers = defaultdict(list)  # net -> instances with an input pin on it
        for inst in self.instances:
            for pin in inst.cell.output_labels:
                net = inst.global_pins.get(pin)
                if net is not None:
                    drivers[net].append(inst)
            for pin in inst.cell.input_labels:
                net = inst.global_pins.get(pin)
                if net is not None:
                    consumers[net].append(inst)

        def input_nets(inst):
            return {inst.global_pins[p] for p in inst.cell.input_labels if p in inst.global_pins}

        # pending[inst]: this instance's input nets that DO have a driver
        # somewhere in the chip but that driver hasn't been resolved yet.
        # A net with no driver at all inside the chip is a true primary
        # input (or a pin Chip couldn't resolve) and never blocks
        # readiness -- an instance is "ready" once pending[inst] is empty.
        pending = {inst: {net for net in input_nets(inst) if drivers.get(net)} for inst in self.instances}

        levels = []
        resolved = set()
        frontier = [inst for inst in self.instances if not pending[inst]]
        while frontier:
            levels.append(frontier)
            resolved.update(frontier)

            newly_ready = []
            for inst in frontier:
                for pin in inst.cell.output_labels:
                    net = inst.global_pins.get(pin)
                    if net is None:
                        continue
                    for consumer in consumers.get(net, []):
                        if net in pending[consumer]:
                            pending[consumer].discard(net)
                            if not pending[consumer]:
                                newly_ready.append(consumer)

            seen = set()
            frontier = [inst for inst in newly_ready if not (inst in seen or seen.add(inst))]

        unresolved = [inst for inst in self.instances if inst not in resolved]
        return levels, unresolved


if __name__ == "__main__":
    import sys

    gds_file = sys.argv[1] if len(sys.argv) > 1 else "./warmup/04_final.gds"
    top_cell_name = sys.argv[2] if len(sys.argv) > 2 else "adder_demo"

    chip = Chip(gds_file, top_cell_name)
    print(f"{top_cell_name}: {len(chip.instances)} logic instance(s)")

    for level_num, level in enumerate(chip.levels):
        print(f"level {level_num} ({len(level)} instance(s)):")
        for inst in level:
            print(f"  {inst!r}")

    if chip.unresolved:
        print(f"unresolved ({len(chip.unresolved)} instance(s), likely sequential feedback):")
        for inst in chip.unresolved:
            print(f"  {inst!r}")
