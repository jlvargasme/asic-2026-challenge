"""
Trace which labeled diffusion regions (and power rails) are electrically
equivalent, by following the sky130-style contact/via stack:

    diff (labeled_regions) --licon--> li1 --mcon--> met1 (VDD or VSS)

Two shapes on adjacent layers in that chain are considered connected if
their XY footprints actually overlap (not just touch at an edge, the way
transistor terminals did in build_transistors.py -- vias/contacts are
real 2D squares that must overlap both the layer above and below them).

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
VDD/VSS if one of those labels falls inside it; every other met1 polygon
-- ordinary routing -- gets its own private node, just like licon/li1/
mcon, so it can still participate in the connectivity graph without ever
being mistaken for a power connection. The old proximity heuristic is
kept only as a fallback for GDS files with no usable labels, and even
then only for the unambiguous case of exactly one rail on each side.

The whole connectivity graph is a set of union-find merges, so it's built
once in NetTracer.__init__ and trace() queries are then just a cheap
lookup -- rebuilding the graph on every trace() call would be wasteful
for anything but a one-off query.
"""

import gdstk


def _overlaps(poly_a, poly_b, precision=1e-3):
    return len(gdstk.boolean(poly_a, poly_b, "and", precision=precision)) > 0


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
        self._uf = _UnionFind()

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

        connect(diff_polys, diff_keys, licon, licon_keys)
        connect(licon, licon_keys, li1, li1_keys)
        connect(li1, li1_keys, mcon, mcon_keys)
        connect(mcon, mcon_keys, met1, met1_keys)
        connect(licon, licon_keys, poly, poly_keys)
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

# if __name__ == "__main__":
#     nwell = [gdstk.rectangle((0, 40), (40, 50))]

#     met1_vdd = gdstk.rectangle((0, 38), (40, 40))   # close to nwell
#     met1_vss = gdstk.rectangle((0, 0), (40, 2))     # far from nwell

#     # a met1 shape that is NOT a power rail -- ordinary signal routing
#     # that happens to run near DN_1's contact stack. If met1 were still
#     # being force-classified as VDD/VSS, this would (wrongly) short DN_1
#     # to a power net; it must come back unconnected to VDD/VSS instead.
#     met1_signal = gdstk.rectangle((19, 3.6), (23, 3.9))
#     met1 = [met1_vdd, met1_vss, met1_signal]

#     # sky130-style pin labels -- ground truth for which met1 shape is which
#     labels = [
#         gdstk.Label("VPWR", (1, 39)),
#         gdstk.Label("VGND", (1, 1)),
#     ]

#     # PMOS-side nwell tap, tied up to the VDD rail
#     dp0 = gdstk.rectangle((5, 41), (7, 43))
#     licon_dp0 = gdstk.rectangle((5.5, 41.5), (6.5, 42.5))
#     li1_dp0 = gdstk.rectangle((5.5, 39.5), (6.5, 42.5))
#     mcon_dp0 = gdstk.rectangle((5.5, 39.6), (6.5, 39.9))

#     # two separate NMOS-side substrate taps, both tied down to the VSS rail
#     dn0 = gdstk.rectangle((5, 3), (7, 5))
#     licon_dn0 = gdstk.rectangle((5.5, 3.5), (6.5, 4.5))
#     li1_dn0 = gdstk.rectangle((5.5, 1.5), (6.5, 4.5))
#     mcon_dn0 = gdstk.rectangle((5.5, 1.6), (6.5, 1.9))

#     # DN_1 routes sideways on li1 to a signal wire on met1 (NOT power)
#     dn1 = gdstk.rectangle((20, 3), (22, 5))
#     licon_dn1 = gdstk.rectangle((20.5, 3.5), (21.5, 4.5))
#     li1_dn1 = gdstk.rectangle((20.5, 3.6), (21.5, 4.5))
#     mcon_dn1 = gdstk.rectangle((20.5, 3.65), (21.5, 3.85))

#     labeled_regions = [(dp0, "DP_0"), (dn0, "DN_0"), (dn1, "DN_1")]
#     licon = [licon_dp0, licon_dn0, licon_dn1]
#     li1 = [li1_dp0, li1_dn0, li1_dn1]
#     mcon = [mcon_dp0, mcon_dn0, mcon_dn1]

#     tracer = NetTracer(labeled_regions, licon, li1, mcon, met1, nwell, labels=labels)
#     print("trace(DN_0):", tracer.trace("DN_0"))  # -> ['VSS']
#     print("trace(DN_1):", tracer.trace("DN_1"))  # -> [] (on signal routing, not power)
#     print("trace(DP_0):", tracer.trace("DP_0"))  # -> ['VDD']