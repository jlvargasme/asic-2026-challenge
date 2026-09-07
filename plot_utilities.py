"""
Visualize gdstk polygons, detected instance pins, and transistor-level
schematics with matplotlib.

gdstk itself has no plotting/rendering functions -- it only manipulates
geometry. Each gdstk.Polygon exposes its vertices as a numpy array via
`.points`, so the standard way to "see" them from Python is to wrap those
vertices in matplotlib patches and let matplotlib do the drawing.

Five layers of visualization live here:

  - plot_cell/plot_polygon/plot_labels: raw GDS layout -- draw a cell's
    polygons by layer, optionally overlaying the transistor gates found by
    transistor.count_transistors() (the poly/diff intersection regions) as
    black-hatched outlines, so you can visually confirm what geometry was
    counted.
  - plot_net: one cell's own named input/output pins, highlighted and
    labeled -- either a leaf standard cell's own layout (cell.Cell), or
    (chip_io_only=True) just a routed top cell's primary I/O nets
    (chip.Chip) without redrawing its entire routing fabric.
  - plot_instance_pins: what instance_pins.py (and pin_direction.py)
    actually find for one placed instance -- the instance's footprint, and
    every detected pin group drawn in its own color and labeled with its
    resolved name.
  - plot_clusters_optimized: clustering.plot_clusters()'s own chip-floorplan
    picture, but after chip_manipulation.py's buffer-collapsing pass --
    what the recovered RTL structure looks like once clock-tree-synthesis
    noise is stripped back out, rather than the raw physically-placed
    instance set.
  - draw_schematic: a transistor-level electrical schematic from the same
    gate/kind/source/drain data pipeline.py's "transistors renamed" step
    prints -- one row of PMOS symbols under a VDD rail, one row of NMOS
    symbols above a VSS rail, the usual way a static CMOS gate is drawn.
"""

import gdstk
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Patch, Rectangle
from matplotlib.patches import Polygon as MplPolygon

from cell import Cell
from chip import Chip
from clustering import plot_clusters
from chip_manipulation import DecompileError, _collapse_transparent_buffers, _stub_leaf, decompile_leaf
from pin import find_instance_pins, label_instance_pins
from transistor import count_transistors, TransistorType

# ---------------------------------------------------------------------------
# GDS layout plotting
# ---------------------------------------------------------------------------

# Layer -> drawing style. Colors loosely follow common EDA-tool conventions
# (poly=red, diffusion=green, nwell=grey) rather than a generic chart
# palette, since that's what a layout viewer's audience expects.
LAYER_STYLE = {
    (64, 20): dict(color="#999999", alpha=0.20, label="nwell (64/20)"),
    (65, 20): dict(color="#2ca02c", alpha=0.55, label="diff (65/20)"),
    (65, 44): dict(color="#98df8a", alpha=0.55, label="tap (65/44)"),
    (66, 20): dict(color="#d62728", alpha=0.55, label="poly (66/20)"),
}


def plot_cell(gds_path, cell_name, out_path="layout.png", highlight_gates=True, gate_labels=None):
    library = gdstk.read_gds(gds_path)
    cell = next((c for c in library.cells if c.name == cell_name), None)
    if cell is None:
        raise ValueError(f"Cell {cell_name!r} not found in {gds_path!r}")

    fig, ax = plt.subplots(figsize=(9, 7))
    legend_handles = []

    for (layer, datatype), style in LAYER_STYLE.items():
        polys = cell.get_polygons(depth=None, layer=layer, datatype=datatype)
        if not polys:
            continue
        patches = [MplPolygon(p.points, closed=True) for p in polys]
        ax.add_collection(
            PatchCollection(
                patches,
                facecolor=style["color"],
                edgecolor=style["color"],
                alpha=style["alpha"],
            )
        )
        legend_handles.append(Patch(facecolor=style["color"], alpha=style["alpha"], label=style["label"]))

    if highlight_gates:
        result = count_transistors(gds_path, cell_name)
        patches = [MplPolygon(g.points, closed=True) for g in result["gates"]]
        ax.add_collection(
            PatchCollection(
                patches,
                facecolor="none",
                edgecolor="black",
                linewidth=1.4,
                hatch="//",
            )
        )
        legend_handles.append(
            Patch(facecolor="none", edgecolor="black", hatch="//", label=f"detected gates ({result['total']})")
        )

    if gate_labels:
        # accept either Transistor objects (.gate / .gate_label) or plain
        # (polygon, label) tuples, same convention as plot_labels().
        items = [
            (t.gate, t.gate_label) if hasattr(t, "gate_label") else t
            for t in gate_labels
        ]
        for polygon, label in items:
            x_min, _ = polygon.points.min(axis=0)
            x_max, y_max = polygon.points.max(axis=0)
            ax.annotate(
                label,
                ((x_min + x_max) / 2, y_max),
                xytext=(0, 2),
                textcoords="offset points",
                ha="center",
                va="bottom",
                fontsize=7,
                fontweight="bold",
            )

    ax.set_aspect("equal")
    ax.autoscale_view()
    ax.set_xlabel("x (um)")
    ax.set_ylabel("y (um)")
    ax.set_title(f"{cell_name}  ({gds_path})")
    ax.legend(handles=legend_handles, loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    print(f"wrote {out_path}")


PIN_STYLE = {"input": "#1f77b4", "output": "#d62728"}


def plot_net(gds_path, cell_name, out_path=None, chip_io_only=False):
    """Render one cell's input/output pins, highlighted and labeled.

    Args:
        gds_path: path to the GDS file.
        cell_name: name of the cell to plot -- a leaf standard cell (e.g.
            "sky130_fd_sc_hd__and3_2") by default, or a routed TOP cell
            (e.g. "adder_demo") if chip_io_only=True.
        out_path: PNG path to write; defaults to "<cell_name>_net.png".
        chip_io_only: False (default) treats `cell_name` as one leaf
            standard cell -- draws its full poly/diff/nwell layout (same
            LAYER_STYLE as plot_cell()) with every one of its own named
            pins highlighted, via cell.Cell's own pin-direction
            classification. This does NOT scale to a whole routed chip:
            depth=None flattens every placed instance's own poly/diff/
            nwell geometry too (tens of thousands of polygons for a real
            design), and cell.Cell tries to run leaf-cell transistor
            extraction on `cell_name` itself, which a top cell isn't.

            Pass True when `cell_name` is a routed top cell instead: this
            skips the whole-chip layout draw and the leaf-cell model
            entirely, and instead uses chip.Chip's own (spatial-grid-
            based, built for this exact scale) net tracing to find just
            that chip's own primary_inputs/primary_outputs -- its true
            top-level ports, not its internal wiring -- and draws only
            those nets' own routing shapes (chip.Chip.net_polygons()),
            each labeled with its resolved name. See _plot_chip_io_nets().
    """
    if chip_io_only:
        return _plot_chip_io_nets(gds_path, cell_name, out_path)

    cell = Cell(gds_path, cell_name)

    library = gdstk.read_gds(gds_path)
    raw_cell = next((c for c in library.cells if c.name == cell_name), None)
    if raw_cell is None:
        raise ValueError(f"Cell {cell_name!r} not found in {gds_path!r}")

    fig, ax = plt.subplots(figsize=(9, 7))
    legend_handles = []

    for (layer, datatype), style in LAYER_STYLE.items():
        polys = raw_cell.get_polygons(depth=None, layer=layer, datatype=datatype)
        if not polys:
            continue
        patches = [MplPolygon(p.points, closed=True) for p in polys]
        ax.add_collection(
            PatchCollection(patches, facecolor=style["color"], edgecolor=style["color"], alpha=style["alpha"])
        )
        legend_handles.append(Patch(facecolor=style["color"], alpha=style["alpha"], label=style["label"]))

    for direction, pin_names in (("input", cell.input_labels), ("output", cell.output_labels)):
        color = PIN_STYLE[direction]
        for pin_name in pin_names:
            polys = cell._analyzer.pin_local_polygons(pin_name)
            if not polys:
                print(f"warning: no routing geometry found for pin {pin_name!r}, skipping")
                continue

            patches = [MplPolygon(p.points, closed=True) for p in polys]
            ax.add_collection(
                PatchCollection(patches, facecolor=color, edgecolor="black", alpha=0.65, linewidth=0.8, zorder=3)
            )

            # label at the centroid of the largest polygon in the group,
            # same reasoning as plot_instance_pins() -- a pin can be
            # stamped on more than one disjoint piece (e.g. one per
            # finger), and an average over all of them can land in a gap.
            biggest = max(polys, key=lambda p: p.area())
            cx, cy = biggest.points.mean(axis=0)
            ax.annotate(
                pin_name, (cx, cy), ha="center", va="center", fontsize=8, fontweight="bold", zorder=4,
                bbox=dict(boxstyle="round,pad=0.15", facecolor="white", alpha=0.8, edgecolor="none"),
            )
        legend_handles.append(Patch(facecolor=color, alpha=0.65, label=f"{direction} pin"))

    ax.set_aspect("equal")
    ax.autoscale_view()
    ax.set_xlabel("x (um)")
    ax.set_ylabel("y (um)")
    ax.set_title(f"{cell_name} pins  ({gds_path})")
    ax.legend(handles=legend_handles, loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=8)

    fig.tight_layout()
    out_path = out_path or f"{cell_name}_net.png"
    fig.savefig(out_path, dpi=180)
    plt.close(fig)
    print(f"wrote {out_path}  ({len(cell.input_labels)} input(s), {len(cell.output_labels)} output(s))")
    return out_path


def _plot_chip_io_nets(gds_path, top_cell_name, out_path=None):
    """plot_net(chip_io_only=True)'s actual implementation -- see that
    function's docstring for why this exists as a separate path instead
    of just calling plot_net's default leaf-cell code on a top cell.

    Draws a dashed chip-footprint outline (from placed instances' own
    bounding boxes, same trick clustering.py's top-level port scan uses --
    top_cell.bounding_box() itself walks the same tens-of-thousands of
    shapes this function exists to avoid touching) for orientation, then
    only chip.primary_inputs/primary_outputs' own routing shapes
    (chip.Chip.net_polygons()) -- not the chip's entire routing fabric.
    """
    chip = Chip(gds_path, top_cell_name)

    fig, ax = plt.subplots(figsize=(9, 7))
    legend_handles = []

    boxes = [r.bounding_box() for r in chip.top_cell.references if r.bounding_box() is not None]
    if boxes:
        fx0 = min(b[0][0] for b in boxes)
        fy0 = min(b[0][1] for b in boxes)
        fx1 = max(b[1][0] for b in boxes)
        fy1 = max(b[1][1] for b in boxes)
        ax.add_patch(
            Rectangle((fx0, fy0), fx1 - fx0, fy1 - fy0, fill=False,
                      edgecolor="black", linestyle="--", linewidth=1.0, zorder=1)
        )
        legend_handles.append(Patch(fill=False, edgecolor="black", linestyle="--", label="chip footprint"))

    for direction, net_names in (("input", chip.primary_inputs), ("output", chip.primary_outputs)):
        color = PIN_STYLE[direction]
        for net_name in net_names:
            polys = chip.net_polygons(net_name)
            if not polys:
                print(f"warning: no routing geometry found for net {net_name!r}, skipping")
                continue

            patches = [MplPolygon(p.points, closed=True) for p in polys]
            ax.add_collection(
                PatchCollection(patches, facecolor=color, edgecolor="black", alpha=0.7, linewidth=0.8, zorder=3)
            )

            biggest = max(polys, key=lambda p: p.area())
            cx, cy = biggest.points.mean(axis=0)
            ax.annotate(
                net_name, (cx, cy), ha="center", va="center", fontsize=8, fontweight="bold", zorder=4,
                bbox=dict(boxstyle="round,pad=0.15", facecolor="white", alpha=0.8, edgecolor="none"),
            )
        legend_handles.append(Patch(facecolor=color, alpha=0.7, label=f"{direction} pin"))

    ax.set_aspect("equal")
    ax.autoscale_view()
    ax.set_xlabel("x (um)")
    ax.set_ylabel("y (um)")
    ax.set_title(f"{top_cell_name} primary I/O nets  ({gds_path})")
    ax.legend(handles=legend_handles, loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=8)

    fig.tight_layout()
    out_path = out_path or f"{top_cell_name}_io_net.png"
    fig.savefig(out_path, dpi=180)
    plt.close(fig)
    print(f"wrote {out_path}  ({len(chip.primary_inputs)} input(s), {len(chip.primary_outputs)} output(s))")
    return out_path


def plot_polygon(polygon, filename, ax=None):
    """Plot a gdstk.Polygon, or a list of gdstk.Polygon."""
    if ax is None:
        _, ax = plt.subplots()

    polygons = polygon if isinstance(polygon, (list, tuple)) else [polygon]
    for p in polygons:
        ax.add_patch(MplPolygon(p.points, closed=True, facecolor="steelblue", edgecolor="black"))

    ax.autoscale_view()
    ax.set_aspect("equal")
    return ax


def plot_labels(items, filename, ax=None):
    """Plot (polygon, label) tuples, with each label above its polygon.

    items: list of (gdstk.Polygon, str) tuples.
    """
    ax = plot_polygon([p for p, _ in items], filename, ax=ax)

    for polygon, label in items:
        x_min, y_min = polygon.points.min(axis=0)
        x_max, y_max = polygon.points.max(axis=0)
        ax.text((x_min + x_max) / 2, y_max, label, ha="center", va="bottom")

    plt.savefig(filename, dpi=150)
    return ax


# ---------------------------------------------------------------------------
# Instance pin plotting
# ---------------------------------------------------------------------------

_PALETTE = plt.get_cmap("tab20").colors


def _find_reference(parent, instance_cell_name, instance_index=0):
    matches = [
        r for r in parent.references
        if hasattr(r.cell, "name") and r.cell.name == instance_cell_name
    ]
    if not matches:
        raise ValueError(f"no instance of {instance_cell_name!r} found inside {parent.name!r}")
    if instance_index >= len(matches):
        raise ValueError(
            f"only {len(matches)} instance(s) of {instance_cell_name!r} in "
            f"{parent.name!r} (asked for index {instance_index})"
        )
    return matches[instance_index], len(matches)


def plot_instance_pins(gds_path, parent_name, instance_cell_name, instance_index=0,
                        out_path=None, show_direction=True):
    """Render, as a PNG, every pin instance_pins.py detects for one placed
    instance -- footprint outline (dashed), each detected pin group in
    its own color, labeled with its resolved name.

    Drawing the footprint as a dashed outline (rather than clipping to it)
    is deliberate: a real pin's shape is EXPECTED to extend past the
    footprint -- that's the entire detection signal find_instance_pins()
    relies on -- so seeing a colored shape stick out past the dashed box is
    exactly the "why this counted as a pin" explanation, not a rendering
    bug.

    Args:
        gds_path: path to the GDS file.
        parent_name: the routed cell that PLACES the instance (e.g.
            "adder_demo") -- pin detection needs routing that reaches in
            from outside the instance's own footprint, so this has to be
            the routed top level, not the leaf cell itself (see
            instance_pins.py's module docstring).
        instance_cell_name: the leaf cell type to look for (e.g.
            "sky130_fd_sc_hd__and3_2").
        instance_index: which placed instance of that type, if there's
            more than one (0 = first).
        out_path: PNG path to write; defaults to
            "<instance_cell_name>_pins.png".
        show_direction: if True (default), use pin_direction.py's
            input/output classification and label pins "in_A"/"out_X";
            if False, just use instance_pins.py's raw pin names.
    """
    library = gdstk.read_gds(gds_path)
    parent = next((c for c in library.cells if c.name == parent_name), None)
    if parent is None:
        raise ValueError(f"cell {parent_name!r} not found in {gds_path!r}")

    ref, n_instances = _find_reference(parent, instance_cell_name, instance_index)
    labels = parent.get_labels(depth=None)

    if show_direction:
        pins = label_instance_pins(gds_path, parent, ref, labels=labels)
    else:
        pins = [
            {**p, "final_label": p["label"]}
            for p in find_instance_pins(parent, ref, labels=labels)
        ]

    (fx0, fy0), (fx1, fy1) = ref.bounding_box()

    # auto-fit the view to the footprint AND every pin polygon -- a real
    # pin's routing can run far past the footprint (see the clkbuf_16
    # example from earlier), so fitting to the footprint alone would
    # clip exactly the part of the picture that explains the detection.
    xs, ys = [fx0, fx1], [fy0, fy1]
    for pin in pins:
        for p in pin["polygons"]:
            (x0, y0), (x1, y1) = p.bounding_box()
            xs += [x0, x1]
            ys += [y0, y1]
    xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)
    data_w, data_h = max(xmax - xmin, 1e-3), max(ymax - ymin, 1e-3)
    pad_x, pad_y = max(data_w * 0.06, 0.5), max(data_h * 0.06, 0.5)

    fig_w = 10
    fig_h = min(max(fig_w * (data_h + 2 * pad_y) / (data_w + 2 * pad_x), 4), 22)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))

    ax.add_patch(
        Rectangle((fx0, fy0), fx1 - fx0, fy1 - fy0, fill=False,
                  edgecolor="black", linestyle="--", linewidth=1.2, zorder=3)
    )

    legend_handles = [Patch(fill=False, edgecolor="black", linestyle="--", label="instance footprint")]
    for i, pin in enumerate(pins):
        color = _PALETTE[i % len(_PALETTE)]
        name = pin.get("final_label") or pin.get("label") or "(unlabeled)"

        patches = [MplPolygon(p.points, closed=True) for p in pin["polygons"]]
        ax.add_collection(PatchCollection(patches, facecolor=color, edgecolor=color, alpha=0.6, zorder=2))
        legend_handles.append(Patch(facecolor=color, alpha=0.6, label=name))

        # label at the centroid of whichever polygon in the group is
        # largest, so it lands solidly inside the shape rather than at an
        # average point that might fall in a gap between disjoint pieces
        biggest = max(pin["polygons"], key=lambda p: p.area())
        cx, cy = biggest.points.mean(axis=0)
        ax.annotate(
            name, (cx, cy), ha="center", va="center", fontsize=8, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.15", facecolor="white", alpha=0.75, edgecolor="none"),
        )

    ax.set_xlim(xmin - pad_x, xmax + pad_x)
    ax.set_ylim(ymin - pad_y, ymax + pad_y)
    ax.set_aspect("equal")
    ax.set_xlabel("x (um)")
    ax.set_ylabel("y (um)")
    suffix = f" [instance {instance_index + 1}/{n_instances}]" if n_instances > 1 else ""
    ax.set_title(f"{instance_cell_name}{suffix} pins  ({parent_name}, {gds_path})")
    ax.legend(handles=legend_handles, loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=8)

    fig.tight_layout()
    out_path = out_path or f"{instance_cell_name}_pins.png"
    fig.savefig(out_path, dpi=180)
    plt.close(fig)
    print(f"wrote {out_path}  ({len(pins)} pin group(s))")
    return out_path


# ---------------------------------------------------------------------------
# Cluster floorplan plotting
# ---------------------------------------------------------------------------


def plot_clusters_optimized(chip, clusters, out_path="clusters_optimized.png", title=None, force_tier2=False):
    """Render the same chip-floorplan cluster plot clustering.plot_clusters()
    does, but AFTER running the one chip_manipulation.py pass that actually
    changes cluster membership: chip_manipulation._collapse_transparent_
    buffers() (see its own docstring) splices out every clock-buffer-tree
    instance and re-aliases its consumers straight to its source --
    shrinking, and for a cluster that was ENTIRELY buffers (e.g. a
    leftover clock-buffer "cluster" from cluster_chip()'s own
    small-cluster merging), sometimes completely emptying out, every
    cluster it touches.

    plot_clusters() itself doesn't want this done first: main.py's own
    pipeline deliberately calls it BEFORE build_ast(), so the plot still
    shows every physically-placed instance (including the buffers about
    to be collapsed) -- that's what its actual job (an epsilon_scale
    sanity check: does a cluster's footprint look like one real module on
    the die) needs. This function is for the opposite question -- what
    does the RECOVERED RTL structure look like once the buffer-tree noise
    clock-tree synthesis introduced during physical implementation (see
    _collapse_transparent_buffers's own docstring) is stripped back out --
    an emptied-out cluster here means one fewer real module, not a bug.

    Mutates `chip` and every Cluster in `clusters` IN PLACE, via
    _collapse_transparent_buffers -- same as build_ast(). Call
    clustering.plot_clusters() FIRST if you also want the pre-collapse
    picture; there's no going back afterward on this chip/clusters pair.
    (This deliberately doesn't just call build_ast() itself and plot the
    result -- build_ast() also runs cluster-type dedup/bus-grouping,
    neither of which chip.instances/Cluster.instances (what plot_clusters
    actually reads) reflect at all, so running the full pipeline here
    would just be slower for no visual difference.)

    Args:
        chip: the chip.Chip clusters was computed from.
        clusters: list of Cluster, e.g. from cluster_chip().
        out_path: PNG file to write.
        title: optional title text; defaults to naming chip.top_cell.name,
            the post-collapse cluster count, and how many instances were
            collapsed out.
        force_tier2: passed through to decompile_leaf() for every leaf
            type -- see its own docstring. Only affects which combinational
            cells get correctly recognized as transparent buffers, not
            what gets plotted.

    Returns:
        out_path.
    """
    leaf_modules = {}
    for inst in chip.instances:
        if inst.cell.cell_name not in leaf_modules:
            try:
                leaf_modules[inst.cell.cell_name] = decompile_leaf(inst.cell, force_tier2=force_tier2)
            except DecompileError as e:
                leaf_modules[inst.cell.cell_name] = _stub_leaf(inst.cell, str(e))

    before = len(chip.instances)
    _collapse_transparent_buffers(chip, clusters, leaf_modules)
    collapsed = before - len(chip.instances)

    surviving = sum(1 for c in clusters if c.instances)
    emptied = len(clusters) - surviving
    default_title = (
        f"{chip.top_cell.name} -- {surviving} cluster(s) after collapsing {collapsed} "
        f"buffer instance(s)" + (f" ({emptied} cluster(s) emptied out)" if emptied else "")
    )
    return plot_clusters(chip, clusters, out_path=out_path, title=title or default_title)


# ---------------------------------------------------------------------------
# Transistor-level schematic plotting
# ---------------------------------------------------------------------------

RAIL_LINEWIDTH = 1.8
LEAD_LINEWIDTH = 1.2
PLATE_LINEWIDTH = 1.6


def _field(t, *names):
    """Read the first present attribute/key out of `names` from `t`,
    whether `t` is a dict (e.g. {"kind": ..., "gate": ...}) or an object
    with attributes (e.g. transistor.py's Transistor dataclass, where the
    fields are named gate_label/source_label/drain_label)."""
    if isinstance(t, dict):
        for name in names:
            if name in t:
                return t[name]
        return None
    for name in names:
        if hasattr(t, name):
            return getattr(t, name)
    return None


def _transistor_fields(t):
    return (
        _field(t, "kind"),
        _field(t, "gate", "gate_label"),
        _field(t, "source", "source_label"),
        _field(t, "drain", "drain_label"),
    )


def _draw_mosfet(ax, cx, cy, kind, gate_label, top_label, bot_label,
                  height=0.8, width=0.5, gate_len=0.55, gate_gap=0.12,
                  bubble_r=0.05, fontsize=8):
    """Draw one simplified MOSFET symbol centered at (cx, cy): a vertical
    channel line (split by a small gap -- the two "plates" -- to mark
    where the gate couples in), a gate lead coming in from the left with
    a bubble on it for PMOS (active-low gate, the usual convention) and
    no bubble for NMOS, and text labels for whichever of top_label/
    bot_label/gate_label are given (pass None to omit a label -- used
    when that terminal is about to get a real rail wire drawn instead).

    Returns (top_lead_y, bottom_lead_y) so the caller can extend a rail
    wire from the exact point the symbol's lead ends.
    """
    half_h, half_w, half_gap = height / 2, width / 2, gate_gap / 2
    top_lead_y, bot_lead_y = cy + half_h, cy - half_h

    ax.add_line(Line2D([cx, cx], [top_lead_y, cy + half_gap], color="black", linewidth=LEAD_LINEWIDTH))
    ax.add_line(Line2D([cx, cx], [cy - half_gap, bot_lead_y], color="black", linewidth=LEAD_LINEWIDTH))
    ax.add_line(Line2D([cx - half_w, cx + half_w], [cy + half_gap, cy + half_gap], color="black", linewidth=PLATE_LINEWIDTH))
    ax.add_line(Line2D([cx - half_w, cx + half_w], [cy - half_gap, cy - half_gap], color="black", linewidth=PLATE_LINEWIDTH))

    gate_x0 = cx - half_w - gate_len
    if kind == "PMOS":
        bubble_x = cx - half_w - bubble_r
        ax.add_line(Line2D([gate_x0, bubble_x - bubble_r], [cy, cy], color="black", linewidth=LEAD_LINEWIDTH))
        ax.add_patch(Circle((bubble_x, cy), bubble_r, fill=False, linewidth=LEAD_LINEWIDTH))
    else:
        ax.add_line(Line2D([gate_x0, cx - half_w], [cy, cy], color="black", linewidth=LEAD_LINEWIDTH))

    if top_label is not None:
        ax.text(cx, top_lead_y + 0.08, top_label, ha="center", va="bottom", fontsize=fontsize)
    if bot_label is not None:
        ax.text(cx, bot_lead_y - 0.08, bot_label, ha="center", va="top", fontsize=fontsize)
    ax.text(gate_x0 - 0.05, cy, gate_label or "?", ha="right", va="center", fontsize=fontsize)

    return top_lead_y, bot_lead_y


def _draw_row(ax, records, kind, cy, rail_y, rail_names, n_cols, dx, x0):
    """Draw one row (all PMOS, or all NMOS) of symbols, centered within
    n_cols so a shorter row (e.g. 3 PMOS vs. 5 NMOS) lines up visually."""
    offset = (n_cols - len(records)) * dx / 2
    for i, r in enumerate(records):
        cx = x0 + offset + i * dx

        # orient the symbol so the rail-connected terminal (if any) is
        # drawn on the side nearer that rail -- purely cosmetic.
        drain_is_rail = r["drain"] in rail_names
        source_is_rail = r["source"] in rail_names
        near_rail_first = (kind == "PMOS")  # PMOS row sits above the VDD rail -> rail terminal on top
        if drain_is_rail or source_is_rail:
            rail_net = r["drain"] if drain_is_rail else r["source"]
            other_net = r["source"] if drain_is_rail else r["drain"]
            top_net, bot_net = (rail_net, other_net) if near_rail_first else (other_net, rail_net)
        else:
            top_net, bot_net = r["source"], r["drain"]

        top_is_rail = top_net in rail_names
        bot_is_rail = bot_net in rail_names
        top_y, bot_y = _draw_mosfet(
            ax, cx, cy, kind, r["gate"],
            None if top_is_rail else top_net,
            None if bot_is_rail else bot_net,
        )
        if top_is_rail:
            ax.add_line(Line2D([cx, cx], [top_y, rail_y], color="black", linewidth=1.0))
        if bot_is_rail:
            ax.add_line(Line2D([cx, cx], [bot_y, rail_y], color="black", linewidth=1.0))


def draw_schematic(transistors, out_path="schematic.png", title=None,
                    vdd_names=("VDD",), vss_names=("VSS",)):
    """Render a transistor-level schematic and save it as a PNG.

    Args:
        transistors: list of Transistor objects (transistor.py) or plain
            dicts with keys "kind"/"gate"/"source"/"drain" -- the same
            shape pipeline.py's "transistors renamed" step prints, where
            source/drain/gate are the *display* labels (e.g. "in_A",
            "out_X", "VDD", or a raw internal label like "DN_0" when a net
            couldn't be resolved to a named pin).
        out_path: PNG file to write.
        title: optional title text drawn above the diagram.
        vdd_names, vss_names: which terminal values count as "this is the
            supply rail, draw a real wire" rather than a net-label stub.
    """
    records = []
    for t in transistors:
        kind, gate, source, drain = _transistor_fields(t)
        records.append({"kind": kind, "gate": gate, "source": source, "drain": drain})

    pmos = [r for r in records if r["kind"] == TransistorType.PMOS]
    nmos = [r for r in records if r["kind"] == TransistorType.NMOS]
    other = [r for r in records if r["kind"] not in (TransistorType.PMOS, TransistorType.NMOS)]

    n_cols = max(len(pmos), len(nmos), 1)
    dx = 1.6
    x0 = 1.0
    width = x0 + n_cols * dx + 1.0

    y_vdd, y_pmos = 3.8, 2.7
    y_nmos, y_vss = 1.0, -0.1
    y_other = y_vss - 1.6 if other else None

    fig_h = 5.4 + (1.8 if other else 0)
    fig, ax = plt.subplots(figsize=(max(width, 4.0), fig_h))

    rail_x1 = x0 + n_cols * dx
    ax.add_line(Line2D([0.2, rail_x1], [y_vdd, y_vdd], color="black", linewidth=RAIL_LINEWIDTH))
    ax.text(0.1, y_vdd, "VDD", ha="right", va="center", fontsize=10, fontweight="bold")
    ax.add_line(Line2D([0.2, rail_x1], [y_vss, y_vss], color="black", linewidth=RAIL_LINEWIDTH))
    ax.text(0.1, y_vss, "VSS", ha="right", va="center", fontsize=10, fontweight="bold")

    _draw_row(ax, pmos, "PMOS", y_pmos, y_vdd, vdd_names, n_cols, dx, x0)
    _draw_row(ax, nmos, "NMOS", y_nmos, y_vss, vss_names, n_cols, dx, x0)

    if other:
        offset = (n_cols - len(other)) * dx / 2
        for i, r in enumerate(other):
            cx = x0 + offset + i * dx
            _draw_mosfet(ax, cx, y_other, "NMOS", r["gate"], r["source"], r["drain"], fontsize=8)
            ax.text(cx, y_other - 0.55, "kind unresolved", ha="center", va="top", fontsize=6.5, color="crimson")

    ax.set_xlim(-0.3, rail_x1 + 0.4)
    y_bottom = (y_other - 1.0) if other else (y_vss - 0.6)
    ax.set_ylim(y_bottom, y_vdd + 0.6)
    ax.set_aspect("equal")
    ax.axis("off")
    if title:
        ax.set_title(title, fontsize=11)

    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)
    print(f"wrote {out_path}")
    return out_path


if __name__ == "__main__":
    import sys

    gds_path = sys.argv[1] if len(sys.argv) > 1 else "warmup/04_final.gds"
    parent_name = sys.argv[2] if len(sys.argv) > 2 else "adder_demo"
    instance_cell_name = sys.argv[3] if len(sys.argv) > 3 else "sky130_fd_sc_hd__and3_2"
    instance_index = int(sys.argv[4]) if len(sys.argv) > 4 else 0

    plot_instance_pins(gds_path, parent_name, instance_cell_name, instance_index)
