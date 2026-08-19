"""
Generic gdstk.Polygon geometry predicates shared across the pipeline: edge
extraction, collinear-segment overlap (used to match transistor gates to
the diffusion regions they touch), and area-based containment/overlap
checks (used for via/contact connectivity and instance-footprint tests).
"""

import gdstk

TOL = 1e-6


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


def _overlaps(poly_a, poly_b, precision=1e-3):
    return len(gdstk.boolean(poly_a, poly_b, "and", precision=precision)) > 0


def _extends_beyond(poly, footprint, precision=1e-3):
    """True if `poly` has area sticking out past `footprint`."""
    outside = gdstk.boolean(poly, footprint, "not", precision=precision)
    return sum(p.area() for p in outside) > 0
