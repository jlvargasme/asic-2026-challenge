"""
Trace which labeled diffusion regions (and power rails) are electrically
equivalent, by following the sky130-style contact/via stack:

    diff (labeled_regions) --licon--> li1 --mcon--> met1 (VDD or VSS)

Two shapes on adjacent layers in that chain are considered connected if
their XY footprints actually overlap (not just touch at an edge, the way
transistor terminals did in transistor.py -- vias/contacts are real 2D
squares that must overlap both the layer above and below them).

poly, li1, and met1 are the exception: sky130 sometimes draws a single
physical net as more than one polygon record on the same layer, and nothing
in a strictly layer-to-layer chain (diff->licon->li1->mcon->met1) ever
reunites two same-layer records with each other. Two concrete cases found
in this project's own example cell (sky130_fd_sc_hd__dfrtp_2):

  - poly: a gate shared by a PMOS row and an NMOS row is drawn as one
    rectangle per row, meeting exactly where the rows meet with ZERO area
    overlap between them (they abut edge-to-edge, not overlap).
  - met1: an internal signal's routing is drawn as two separate met1
    polygons (met1[1] and met1[3] in dfrtp_2) that DO overlap in area --
    but since NetTracer never checked met1-against-itself at all, that
    overlap was simply never looked at. met1[1] alone reached four
    transistor gates and nothing else; met1[3] alone reached the real
    diffusion driver (DP_2/DN_2) and nothing else -- so the whole net was
    silently split in two, and the gate side had no path to anything,
    which SPICE reports as a singular matrix.

Either failure mode (missed edge-touch, or a same-layer overlap nobody
checked) leaves part of a net with no way back to the rest of it, so
poly/li1/met1 are each connected against themselves for BOTH conditions
(_touching() OR _overlaps()) -- see connect_self(). This is always safe:
two shapes on the same layer that touch or overlap are, by construction,
electrically the same net, since anything else would be a physical short.
licon/mcon (small discrete contacts) and the labeled diffusion regions
(already correctly split apart by label_diffusion_regions) aren't given
this treatment -- there's no equivalent reason to expect either of those
to legitimately need self-merging.

met1 is a general-purpose routing layer, not exclusively power -- a real
routed design (e.g. the "adder_demo" top cell, as opposed to a single
standard cell) has hundreds of met1 shapes that are ordinary signal wires
with nothing to do with VDD/VSS. Force-classifying every met1 polygon as
either "VDD" or "VSS" (the original approach here, by nwell proximity)
was wrong: it silently folded unrelated signal routing into the power
nets, which is exactly what made unrelated diffusion regions appear
"equivalent" through met1 when they shouldn't be.

The fix is to use the ground truth that's already in the GDS: sky130
places an actual "VPWR"/"VGND" text label directly on each power rail
shape (see cell.get_labels()). A met1 polygon is only ever classified as
VDD/VSS if one of those labels falls inside it, OR it's electrically
connected (by the same touching/overlap test connect_self() uses) to one
that already is -- e.g. sky130_fd_sc_hd__dfrtp_2 has two small met1 stubs
right at its right edge (met1[10]/met1[11]) that are clearly just the
VSS/VDD rails continuing slightly past the main cell body for abutment
with the next cell in a row, sharing the rails' exact y-range and meeting
them edge-to-edge -- but neither carries its own copy of the VPWR/VGND
label, so a label-only test would (and used to) call them unidentified.
Every OTHER met1 polygon -- ordinary routing, not connected to a proven
rail either directly or transitively -- still gets its own private node,
just like licon/li1/mcon, so it can still participate in the connectivity
graph without ever being mistaken for a power connection. The old
proximity heuristic is kept only as a fallback for GDS files with no
usable labels, and even then only for the unambiguous case of exactly one
rail on each side, once propagation has already resolved everything it
can prove.

The whole connectivity graph is a set of union-find merges, so it's built
once in NetTracer.__init__ and trace() queries are then just a cheap
lookup -- rebuilding the graph on every trace() call would be wasteful
for anything but a one-off query.
"""

import gdstk

from geometry import _overlaps, _touching
from union_find import UnionFind


def _label_met1_nets(met1, nwell, labels=None, vdd_names=("VPWR", "VDD"), vss_names=("VGND", "VSS", "GND")):
    """Classify met1 polygons as "VDD" / "VSS" -- only where we can prove it.

    Returns a dict {index: "VDD"|"VSS"}; any met1 index NOT present in the
    returned dict simply wasn't identified as a power rail and should be
    treated as ordinary routing (the caller gives it its own node instead
    of forcing it into a power net).
    """
    if not met1:
        return {}

    result = {}

    # Preferred: a VPWR/VGND (etc.) text label sitting inside the polygon
    # -- this is ground truth straight from the GDS.
    if labels:
        vdd_pts = [lbl.origin for lbl in labels if lbl.text in vdd_names]
        vss_pts = [lbl.origin for lbl in labels if lbl.text in vss_names]
        for i, poly in enumerate(met1):
            if vdd_pts and any(gdstk.inside(vdd_pts, [poly])):
                result[i] = "VDD"
            elif vss_pts and any(gdstk.inside(vss_pts, [poly])):
                result[i] = "VSS"

    # Propagate: a met1 shape with no label of its own but electrically
    # connected (touching or overlapping -- same test connect_self() uses
    # for the real connectivity graph) to one that's already proven is
    # just as provably that same net -- e.g. a small rail-extension stub
    # at a cell's edge, meant for abutment with the next cell in a row,
    # that never got its own copy of the VPWR/VGND label. Iterated to a
    # fixed point since a chain of several unlabeled stubs should all
    # inherit the same classification, not just the one directly
    # touching a labeled shape.
    changed = True
    while changed:
        changed = False
        for i in range(len(met1)):
            if i in result:
                continue
            for j in list(result):
                if _overlaps(met1[i], met1[j]) or _touching(met1[i], met1[j]):
                    result[i] = result[j]
                    changed = True
                    break

    unresolved = [i for i in range(len(met1)) if i not in result]
    if not unresolved:
        return result

    # Fallback proximity heuristic (closer to nwell => VDD) -- only trusted
    # for the unambiguous single-rail-each-side case, and only for the
    # polygons a label didn't already resolve. Guessing across many met1
    # shapes (a real routed top level) is exactly the bug this replaced,
    # so we deliberately refuse to guess once there's more than one
    # candidate per side.
    if len(unresolved) == 2 and not result and nwell:
        nwell_centroids = [w.points.mean(axis=0) for w in nwell]

        def dist_to_nwell(poly):
            cx, cy = poly.points.mean(axis=0)
            return min(((cx - nx) ** 2 + (cy - ny) ** 2) ** 0.5 for nx, ny in nwell_centroids)

        i, j = unresolved
        if dist_to_nwell(met1[i]) <= dist_to_nwell(met1[j]):
            result[i], result[j] = "VDD", "VSS"
        else:
            result[i], result[j] = "VSS", "VDD"
        print(f"warning: no VPWR/VGND labels found; guessed met1[{i}]/met1[{j}] as VDD/VSS by nwell proximity")
    elif unresolved:
        print(
            f"warning: {len(unresolved)} met1 polygon(s) could not be identified as VDD/VSS "
            f"(no matching label, or too many candidates to guess safely) -- "
            f"treating them as ordinary routing, not a power net"
        )

    return result


class NetTracer:
    def __init__(self, labeled_regions, transistors, poly, licon, li1, mcon, met1, nwell, labels=None):
        self._uf = UnionFind()

        met1_net = _label_met1_nets(met1, nwell, labels=labels)

        diff_polys = [region for region, _ in labeled_regions]
        diff_keys = [label for _, label in labeled_regions]
        licon_keys = [f"licon_{i}" for i in range(len(licon))]
        li1_keys = [f"li1_{i}" for i in range(len(li1))]
        mcon_keys = [f"mcon_{i}" for i in range(len(mcon))]
        poly_keys = [f"poly_{i}" for i in range(len(poly))]
        transistor_keys = [t.gate_label for t in transistors]
        transistor_poly = [t.gate for t in transistors]
        # only polygons _label_met1_nets could actually prove are VDD/VSS
        # get that shared key; everything else gets its own private node
        # (like licon/li1/mcon) so it can't be mistaken for a power net.
        met1_keys = [met1_net.get(i, f"met1_{i}") for i in range(len(met1))]

        for key in diff_keys + licon_keys + li1_keys + mcon_keys + met1_keys + poly_keys + transistor_keys:
            self._uf.find(key)  # register every node, even isolated ones

        def connect(polys_a, keys_a, polys_b, keys_b):
            for pa, ka in zip(polys_a, keys_a):
                for pb, kb in zip(polys_b, keys_b):
                    if _overlaps(pa, pb):
                        self._uf.union(ka, kb)

        def connect_self(polys, keys):
            """Union every pair of same-layer `polys` that either area-
            overlap or abut edge-to-edge -- see module docstring for why
            both checks are needed and why this is always safe."""
            for i in range(len(polys)):
                for j in range(i + 1, len(polys)):
                    if _overlaps(polys[i], polys[j]) or _touching(polys[i], polys[j]):
                        self._uf.union(keys[i], keys[j])

        connect(diff_polys, diff_keys, licon, licon_keys)
        connect(licon, licon_keys, li1, li1_keys)
        connect_self(li1, li1_keys)
        connect(li1, li1_keys, mcon, mcon_keys)
        connect(mcon, mcon_keys, met1, met1_keys)
        connect_self(met1, met1_keys)
        connect(licon, licon_keys, poly, poly_keys)
        connect_self(poly, poly_keys)
        connect(poly, poly_keys, transistor_poly, transistor_keys)

        # "labels" in the trace() sense are the diffusion labels you passed
        # in plus the two power nets -- licon/li1/mcon/unresolved-met1 are
        # just internal plumbing, not something callers referred to by name.
        self._real_labels = set(diff_keys) | {k for k in met1_keys if k in ("VDD", "VSS")} | set(transistor_keys)

    def trace(self, label):
        root = self._uf.find(label)
        return sorted(
            other for other in self._real_labels
            if other != label and self._uf.find(other) == root
        )

    def find_root(self, label):
        others = self.trace(label)
        if len(others) == 0:
            return label
        elif "VSS" in others:
           return "VSS"
        elif "VDD" in others:
            return "VDD"
        else:
            return others[0]
