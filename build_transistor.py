
from count_transistors import count_transistors
from label_regions import label_regions

"""
Build Transistor objects from gate polygons + labeled source/drain regions.

Assumes a Manhattan (axis-aligned) layout, which covers standard planar
CMOS -- each polygon edge is either purely horizontal or purely vertical.
A gate polygon (one entry of `gates`, e.g. a poly/diff overlap region) is
matched to the labeled diffusion regions it touches by checking, for every
edge of the gate against every edge of every labeled region, whether the
two edges are collinear and their overlapping span has positive length.

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

import gdstk

TOL = 1e-6


@dataclass
class Transistor:
    kind: str  # "NMOS" or "PMOS", or None if it couldn't be resolved
    gate_label: str
    source_label: str = None
    drain_label: str = None
    gate: gdstk.Polygon = None
    extra_region_labels: list = field(default_factory=list)  # matches beyond source/drain


def _edges(polygon):
    pts = polygon.points
    n = len(pts)
    return [(tuple(pts[i]), tuple(pts[(i + 1) % n])) for i in range(n)]


def _segments_overlap(seg_a, seg_b, tol=TOL):
    (ax0, ay0), (ax1, ay1) = seg_a
    (bx0, by0), (bx1, by1) = seg_b

    a_vertical = abs(ax0 - ax1) < tol
    b_vertical = abs(bx0 - bx1) < tol
    if a_vertical and b_vertical and abs(ax0 - bx0) < tol:
        a_lo, a_hi = sorted((ay0, ay1))
        b_lo, b_hi = sorted((by0, by1))
        return min(a_hi, b_hi) - max(a_lo, b_lo) > tol

    a_horizontal = abs(ay0 - ay1) < tol
    b_horizontal = abs(by0 - by1) < tol
    if a_horizontal and b_horizontal and abs(ay0 - by0) < tol:
        a_lo, a_hi = sorted((ax0, ax1))
        b_lo, b_hi = sorted((bx0, bx1))
        return min(a_hi, b_hi) - max(a_lo, b_lo) > tol

    return False

def _touching_regions(gate, labeled_regions):
    gate_edges = _edges(gate)
    matches = []
    for region, label in labeled_regions:
        region_edges = _edges(region)
        if any(_segments_overlap(ge, re) for ge in gate_edges for re in region_edges):
            centroid = region.points.mean(axis=0)
            matches.append((label, centroid))
    matches.sort(key=lambda m: (m[1][0], m[1][1]))
    return matches


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
        kind = {"N": "NMOS", "P": "PMOS"}.get(kind_letter)

        labels = [label for label, _ in matches]
        source_label = labels[0] if len(labels) >= 1 else None
        drain_label = labels[1] if len(labels) >= 2 else None
        extra = labels[2:]

        if len(labels) < 2:
            print(f"warning: gate {i} only touches {len(labels)} labeled region(s): {labels}")

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

# python ./count_transistors.py ./warmup/04_final.gds sky130_fd_sc_hd__and3_2
if __name__ == "__main__":
    import sys

    # if len(sys.argv) != 3:
    #     print(f"usage: {sys.argv[0]} <gds_path> <cell_name>")
    #     sys.exit(1)

    # result = count_transistors(sys.argv[1], sys.argv[2])
    gds_file = "./warmup/04_final.gds"
    cell_name = "sky130_fd_sc_hd__and3_2"
    result = count_transistors(gds_file, cell_name)
    labeled_regions = label_regions(gds_file, cell_name)
    # transistors = build_transistors(result["gates"], labeled_regions)

    for t in build_transistors(result["gates"], labeled_regions):
        print(t.kind, t.gate_label, t.source_label, t.drain_label)
