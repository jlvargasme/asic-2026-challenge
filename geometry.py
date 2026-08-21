"""
Generic gdstk.Polygon geometry predicates shared across the pipeline: edge
extraction, collinear-segment overlap (used to match transistor gates to
the diffusion regions they touch), and area-based containment/overlap
checks (used for via/contact connectivity and instance-footprint tests).
"""

import gdstk

TOL = 1e-6

# sky130's routing stack, contact/via by contact/via: two shapes on one of
# these ADJACENT (name_a, name_b) layer pairs can be electrically connected
# by a real via/contact if their footprints overlap. This is the single
# canonical definition of that adjacency -- net_trace.py (one leaf cell's
# poly/li1/met1/licon/mcon graph), pin.py (one placed instance's own
# li1..met2 pin-detection graph), and chip.py (the full chip's li1..met5
# routing graph) all import it rather than keeping their own copies, so a
# layer added or renamed here doesn't need to be kept in sync by hand in
# three places. A caller that only uses a subset of these layers (e.g.
# pin.py never builds met3+ shapes) simply never generates candidate pairs
# for the unused entries -- carrying the full stack's pairs is harmless.
#
# This is NOT sufficient on its own to reconstruct a net graph: two shapes
# on the SAME layer that overlap or abut are just as electrically connected
# as two shapes on adjacent layers linked by a via (see _touching's
# docstring for why -- sky130 routinely draws one physical wire as several
# same-layer polygons). Same-layer self-connection has to be handled
# separately by each caller, since whether it's warranted depends on that
# caller's own layer set.
ADJACENT_LAYER_PAIRS = {
    ("li1", "mcon"),
    ("mcon", "met1"),
    ("met1", "via1"),
    ("via1", "met2"),
    ("met2", "via2"),
    ("via2", "met3"),
    ("met3", "via3"),
    ("via3", "met4"),
    ("met4", "via4"),
    ("via4", "met5"),
}


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


def _touching(poly_a, poly_b):
    """True if `poly_a` and `poly_b` share a collinear, overlapping edge
    segment -- i.e. they abut, even if their areas don't overlap at all.
    Same technique transistor.py's build_transistors() uses to match a
    gate to the diffusion it touches (via _touching_regions above).

    Needed alongside _overlaps() wherever same-layer shapes are unioned
    together: sky130 sometimes draws one physical net as more than one
    polygon record on the same layer, and the two records can meet either
    by area-overlapping OR by abutting edge-to-edge with zero area overlap
    (e.g. a poly gate shared by a PMOS row and an NMOS row is drawn as one
    rectangle per row, meeting exactly where the rows meet). Either
    failure mode -- a missed edge-touch, or a same-layer overlap nobody
    checked -- leaves part of a net with no way back to the rest of it, so
    callers doing same-layer self-connection (net_trace.py's
    connect_self(), chip.py's _build_net_graph(), pin.py's
    find_instance_pins()) all check both _overlaps() OR _touching()."""
    edges_a, edges_b = _edges(poly_a), _edges(poly_b)
    return any(_segments_overlap(ea, eb) for ea in edges_a for eb in edges_b)


def _overlaps(poly_a, poly_b, precision=1e-3):
    return len(gdstk.boolean(poly_a, poly_b, "and", precision=precision)) > 0


def _extends_beyond(poly, footprint, precision=1e-3):
    """True if `poly` has area sticking out past `footprint`."""
    outside = gdstk.boolean(poly, footprint, "not", precision=precision)
    return sum(p.area() for p in outside) > 0


def _split_pinched(polygons, epsilon=1e-4, precision=1e-6):
    """Split any polygon in `polygons` that is really several disjoint
    pieces joined by zero-width "pinch" bridges into its true separate
    pieces.

    gdstk.boolean's underlying clipping engine can represent what is
    electrically several disjoint regions as a SINGLE polygon record --
    e.g. subtracting two nearby gate cuts from one diffusion strip can
    come back as one comb-shaped polygon whose "teeth" meet the opposite
    edge exactly, joined only by a zero-width bridge, rather than as
    separate polygons for each tooth. This isn't a precision artifact
    (confirmed empirically: unaffected by the boolean's own `precision`
    argument, across a 1e-3 to 1e-9 sweep) -- it's just how the underlying
    engine chooses to report a multiply-connected result.

    A zero-width bridge can't survive being eroded at all, no matter how
    small the erosion -- so shrinking each polygon by `epsilon` (a
    negative offset) severs any such bridge and leaves the true disjoint
    pieces behind, each still (approximately) its own shape. Growing them
    back by the same `epsilon` restores each piece's original size --
    exactly, for the axis-aligned/Manhattan geometry this project already
    assumes elsewhere (see transistor.py's module docstring), since a
    miter join preserves right-angle corners exactly.

    A polygon that was never pinched eroded-then-dilated only produces
    that one polygon back, so this is safe to apply broadly.
    """
    result = []
    for p in polygons:
        eroded = gdstk.offset([p], -epsilon, join="miter", precision=precision)
        if len(eroded) <= 1:
            result.append(p)
            continue
        result.extend(gdstk.offset(eroded, epsilon, join="miter", precision=precision))
    return result
