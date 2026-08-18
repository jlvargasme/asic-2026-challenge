"""
Visualize gdstk polygons with matplotlib.

gdstk itself has no plotting/rendering functions -- it only manipulates
geometry. Each gdstk.Polygon exposes its vertices as a numpy array via
`.points`, so the standard way to "see" them from Python is to wrap those
vertices in matplotlib patches and let matplotlib do the drawing.

This also overlays the transistor gates found by count_transistors.py
(the poly/diff intersection regions) as black-hatched outlines, so you can
visually confirm what geometry was counted.
"""

import gdstk
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Patch
from matplotlib.patches import Polygon as MplPolygon

from count_transistors import count_transistors

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

def plot_polygon(polygon, filename, ax=None):
    """Plot a gdstk.Polygon, or a list of gdstk.Polygon."""
    if ax is None:
        _, ax = plt.subplots()
 
    polygons = polygon if isinstance(polygon, (list, tuple)) else [polygon]
    for p in polygons:
        ax.add_patch(MplPolygon(p.points, closed=True, facecolor="steelblue", edgecolor="black"))
 
    ax.autoscale_view()
    ax.set_aspect("equal")
    # plt.savefig(filename, dpi=150)
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

if __name__ == "__main__":
    import sys

    gds_path = sys.argv[1] if len(sys.argv) > 1 else "./warmup/04_final.gds"
    cell_name = sys.argv[2] if len(sys.argv) > 2 else "sky130_fd_sc_hd__and3_2"

    # demonstrate gate_labels: build the Transistor list the same way
    # tracer.py does, and plot each one's gate_label on top of its gate.
    from build_transistor import build_transistors
    from tracer import label_diffusion_regions, load_cell

    cell = load_cell(gds_path, cell_name)
    gates = count_transistors(gds_path, cell_name)["gates"]
    labeled_regions = label_diffusion_regions(cell, gates)
    transistors = build_transistors(gates, labeled_regions)

    plot_cell(gds_path, cell_name, gate_labels=transistors)