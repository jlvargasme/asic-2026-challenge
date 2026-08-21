"""
Recover approximate RTL module boundaries from a routed, flattened chip.

GDSII has no notion of "module" -- by the time a design reaches GDS, place
and route has flattened the original Verilog hierarchy into one big list of
standard-cell instances wired together by routing (confirmed empirically on
this project's own puzzle.gds: its texttype-44 labels are just each
instance's leaf cell type, e.g. "and2_2", not a per-instance hierarchical
name -- there is no ground truth to read back out of the GDS itself).

This recovers it from physical layout: two instances merge into the same
cluster if their footprints are close enough, transitively (single-linkage
-- "cell A merges with B, B merges with C, so A/B/C are one cluster even
if A and C aren't directly close"), then any resulting cluster too small
to trust on its own gets folded into whichever other cluster its nearest
neighbor belongs to.

"Close enough" took two real corrections to get right, both worth keeping
in mind before trusting this on a different design:

1. Distance has to be measured edge-to-edge (gap between footprints), not
   center-to-center. Center distance is confounded by cell size: two
   instances sitting flush against each other (zero real gap) still show a
   nonzero center-to-center distance, roughly equal to the sum of their
   half-widths -- so a cell that's simply *wide* looks "far" from its
   immediate neighbor even with no space between them. Confirmed directly
   on warmup/04_final.gds: 7 of 8 sampled instances' nearest neighbor by
   center distance (2.7-5.5 apart) had a true edge-to-edge gap of exactly
   0.0.

2. Even measured correctly, the gap between two instances in the same
   module isn't always exactly 0. chip.py deliberately excludes non-logic
   references (decap, tap, via-stitching cells -- see chip.py's
   _is_logic_cell) from chip.instances, but those cells are still
   physically placed in the rows. Confirmed directly: a clkbuf_16's
   nearest logic neighbor (edge gap 2.24) turned out to have a
   sky130_fd_sc_hd__tapvpwrvgnd_1 -- a well-strap tap cell -- sitting
   exactly in the gap between them, both in the same shift register
   module. So "same module" can't mean "gap == 0"; it needs a real
   tolerance.

That tolerance is each pair's own footprint size: two instances merge if
their edge-to-edge gap is no more than the SMALLER of their two min(width,
height) values. This has a concrete physical motivation, not just a
plausible-sounding one -- a filler/tap cell sitting between two logic
cells is itself built from the same row height and similar unit-width
increments as the logic cells around it, so "about as big as the smaller
of these two cells" is a reasonable estimate of how big a gap a filler
cell could plausibly explain, without needing to know that filler cell's
exact size (chip.py doesn't keep it, by design -- see _is_logic_cell).

Both corrections were validated together on warmup/04_final.gds, whose
source (warmup/00_source.v) names exactly 4 submodules (sr_a, sr_b, add0,
cmp0): the "4 modules" result holds for EVERY edge-to-edge threshold from
2.5 to 4.96 (a single-linkage merge-distance plateau nearly 2x wider than
the equivalent plateau found earlier for center-distance, 6.9-8.21), and
each cell's own min(width, height) (~3.2 here, effectively the row height)
sits comfortably in the middle of that range rather than near either edge.
At that setting, cluster_chip() recovers all 4 modules exactly: both
shift registers as their full 16-cell core (+clkbufs), add0 as one clean
41-gate combinational block, and -- notably -- comparator496's 3 gates as
their own isolated cluster, a case that no amount of tuning a netlist-
connectivity-based approach (this file's very first implementation) could
ever separate from the adder it reads from, because comparator496's 3
gates have only 2 internal wires against 9 external ones (see this file's
git history for that whole investigation).

This still isn't free of the risk the very first version of this file
raised: physical placement doesn't guarantee module separation. A placer
optimizes wirelength/timing/congestion, not module identity, and nothing
here would notice if two genuinely different modules were ever placed
closer together than this epsilon allows -- there is no connectivity
cross-check anymore. That risk is entirely untested on puzzle.gds (roughly
20x this design's instance count, deliberately never used to calibrate any
of this) and there's no way to rule it out short of trying it and checking
plot_clusters()'s output by eye, the same way every claim in this
docstring was checked here rather than assumed.
"""

import math
from collections import Counter, defaultdict
from dataclasses import dataclass, field

import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Patch, Rectangle

from union_find import UnionFind


@dataclass
class Cluster:
    """One recovered cluster of chip.py Instances -- a candidate RTL module.

    Attributes:
        id: cluster index (arbitrary, stable only within one clustering run).
        instances: the chip.Instance objects grouped into this cluster.
    """

    id: int
    instances: list = field(default_factory=list)

    @property
    def size(self):
        return len(self.instances)

    @property
    def cell_type_counts(self):
        """Counter of leaf cell type name -> count -- a rough fingerprint of
        what this cluster is built from (e.g. mostly dfrtp_2 => likely
        holds registers; mostly full-adder/xor shapes => likely arithmetic)."""
        return Counter(inst.cell.cell_name for inst in self.instances)

    @property
    def bounding_box(self):
        """((x0, y0), (x1, y1)) covering every instance's footprint, or None
        if the cluster is empty."""
        boxes = [b for inst in self.instances if (b := inst.reference.bounding_box()) is not None]
        if not boxes:
            return None
        x0 = min(b[0][0] for b in boxes)
        y0 = min(b[0][1] for b in boxes)
        x1 = max(b[1][0] for b in boxes)
        y1 = max(b[1][1] for b in boxes)
        return (x0, y0), (x1, y1)

    def __repr__(self):
        top = self.cell_type_counts.most_common(3)
        return f"Cluster({self.id}, size={self.size}, top_cells={top})"


def _instance_center(inst):
    (x0, y0), (x1, y1) = inst.reference.bounding_box()
    return (x0 + x1) / 2, (y0 + y1) / 2


def _instance_min_dim(inst):
    """min(width, height) of this instance's own footprint -- the same
    per-cell quantity the original proximity idea started from. Used here
    as this instance's own contribution to a pairwise merge epsilon (see
    _edge_gap and cluster_chip's docstring for why a fixed epsilon isn't
    enough)."""
    (x0, y0), (x1, y1) = inst.reference.bounding_box()
    return min(x1 - x0, y1 - y0)


def _edge_gap(box_a, box_b):
    """True edge-to-edge (bounding-box) distance between two footprints --
    0 if they touch or overlap. NOT the same as center-to-center distance:
    two abutting boxes of very different sizes have a large center
    distance (roughly the sum of their half-widths) despite a real gap of
    0 -- see the module docstring for why that distinction mattered here."""
    (ax0, ay0), (ax1, ay1) = box_a
    (bx0, by0), (bx1, by1) = box_b
    dx = max(ax0 - bx1, bx0 - ax1, 0.0)
    dy = max(ay0 - by1, by0 - ay1, 0.0)
    return math.hypot(dx, dy)


def _merge_small_clusters(clusters, min_cluster_size):
    """Fold each cluster at or below min_cluster_size into whichever other
    cluster contains its physically nearest instance (by edge-to-edge
    gap), repeating until none is left that small (or only one cluster
    remains). A "floating gate" cluster_chip's epsilon left as its own
    tiny cluster almost always belongs with whatever's physically closest
    to it, not with itself."""
    inst_cluster = {inst: c.id for c in clusters for inst in c.instances}
    by_id = {c.id: c for c in clusters}
    all_insts = [inst for c in clusters for inst in c.instances]
    boxes = {inst: inst.reference.bounding_box() for inst in all_insts}

    changed = True
    while changed and len(by_id) > 1:
        changed = False
        for cid in list(by_id):
            cluster = by_id.get(cid)
            if cluster is None or cluster.size > min_cluster_size:
                continue

            best_dist, target_id = None, None
            for inst in cluster.instances:
                for other in all_insts:
                    if inst_cluster[other] == cid:
                        continue
                    d = _edge_gap(boxes[inst], boxes[other])
                    if best_dist is None or d < best_dist:
                        best_dist, target_id = d, inst_cluster[other]
            if target_id is None:
                continue

            target = by_id[target_id]
            target.instances.extend(cluster.instances)
            for inst in cluster.instances:
                inst_cluster[inst] = target_id
            del by_id[cid]
            changed = True

    return list(by_id.values())


def cluster_chip(chip, epsilon_scale=1.0, min_cluster_size=1):
    """Recover candidate RTL-module clusters from a routed chip.Chip, by
    single-linkage clustering on instance footprints.

    Args:
        chip: a chip.Chip with .instances already built.
        epsilon_scale: multiplier on each pair's merge epsilon (see the
            module docstring) -- two instances merge if their edge-to-edge
            gap is at most epsilon_scale * min(each instance's own
            min(width, height)). 1.0 (the default) is the validated
            setting on warmup/04_final.gds; expose this rather than a
            fixed distance so a design with a very different row height or
            filler-cell sizing can be rescaled without touching the
            per-cell logic.
        min_cluster_size: clusters at or below this size are merged into
            whichever neighboring cluster contains their physically
            nearest instance (see _merge_small_clusters) instead of being
            reported as their own one/two-instance "module".

    Returns:
        list of Cluster, sorted largest-first, with .id reassigned 0..N-1
        in that order.
    """
    instances = chip.instances
    if len(instances) < 2:
        return [Cluster(id=0, instances=list(instances))] if instances else []

    boxes = {inst: inst.reference.bounding_box() for inst in instances}
    min_dims = {inst: _instance_min_dim(inst) for inst in instances}

    uf = UnionFind()
    for inst in instances:
        uf.find(inst)

    for i in range(len(instances)):
        a = instances[i]
        for j in range(i + 1, len(instances)):
            b = instances[j]
            epsilon = epsilon_scale * min(min_dims[a], min_dims[b])
            if _edge_gap(boxes[a], boxes[b]) <= epsilon:
                uf.union(a, b)

    groups = defaultdict(list)
    for inst in instances:
        groups[uf.find(inst)].append(inst)
    clusters = [Cluster(id=i, instances=insts) for i, insts in enumerate(groups.values())]

    if min_cluster_size > 1:
        clusters = _merge_small_clusters(clusters, min_cluster_size)

    clusters.sort(key=lambda c: c.size, reverse=True)
    for new_id, c in enumerate(clusters):
        c.id = new_id
    return clusters


_PALETTE = plt.get_cmap("tab20").colors


def plot_clusters(chip, clusters, out_path="clusters.png", title=None):
    """Render a chip floorplan PNG with every instance colored by which
    Cluster it landed in -- both the tool epsilon_scale sweeping depends
    on (see the module docstring) and a sanity check on the result: a
    cluster whose cells actually sit together on the die looks like a real
    module at a glance; one scattered across the floorplan is a sign
    epsilon_scale is too large (merging separate modules) or too small
    (fragmenting one).

    Follows the same rectangle-per-footprint + PatchCollection + legend
    approach as plot_utilities.plot_instance_pins, just keyed by cluster
    id instead of by pin name, and covering the whole chip's instances
    rather than one placed instance's pins.

    Args:
        chip: the chip.Chip clusters was computed from (used only for
            chip.top_cell.name, in the default title).
        clusters: list of Cluster, e.g. from cluster_chip().
        out_path: PNG file to write.
        title: optional title text; defaults to naming chip.top_cell.name
            and the cluster count.

    Returns:
        out_path.
    """
    fig, ax = plt.subplots(figsize=(10, 8))

    xs, ys = [], []
    legend_handles = []
    for cluster in clusters:
        color = _PALETTE[cluster.id % len(_PALETTE)]
        patches = []
        for inst in cluster.instances:
            box = inst.reference.bounding_box()
            if box is None:
                continue
            (x0, y0), (x1, y1) = box
            patches.append(Rectangle((x0, y0), x1 - x0, y1 - y0))
            xs += [x0, x1]
            ys += [y0, y1]
        if not patches:
            continue

        ax.add_collection(
            PatchCollection(patches, facecolor=color, edgecolor=color, alpha=0.7, linewidth=0.3)
        )
        top_cells = ", ".join(f"{name}x{n}" for name, n in cluster.cell_type_counts.most_common(2))
        legend_handles.append(
            Patch(facecolor=color, alpha=0.7, label=f"cluster {cluster.id} ({cluster.size}): {top_cells}")
        )

    if xs and ys:
        pad_x = max((max(xs) - min(xs)) * 0.03, 1.0)
        pad_y = max((max(ys) - min(ys)) * 0.03, 1.0)
        ax.set_xlim(min(xs) - pad_x, max(xs) + pad_x)
        ax.set_ylim(min(ys) - pad_y, max(ys) + pad_y)
    ax.set_aspect("equal")
    ax.set_xlabel("x (um)")
    ax.set_ylabel("y (um)")
    ax.set_title(title or f"{chip.top_cell.name} -- {len(clusters)} cluster(s)")
    ax.legend(handles=legend_handles, loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=7)

    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)
    print(f"wrote {out_path}")
    return out_path


if __name__ == "__main__":
    import sys

    from chip import Chip

    gds_file = sys.argv[1] if len(sys.argv) > 1 else "./warmup/04_final.gds"
    top_cell_name = sys.argv[2] if len(sys.argv) > 2 else "adder_demo"
    epsilon_scale = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0

    chip = Chip(gds_file, top_cell_name)
    print(f"{top_cell_name}: {len(chip.instances)} logic instance(s)")

    clusters = cluster_chip(chip, epsilon_scale=epsilon_scale)
    print(f"\n{len(clusters)} cluster(s) found (epsilon_scale={epsilon_scale}):")
    for c in clusters:
        top_cells = ", ".join(f"{name}x{n}" for name, n in c.cell_type_counts.most_common(5))
        print(f"  cluster {c.id}: {c.size} instance(s) -- {top_cells}")

    plot_clusters(chip, clusters, out_path=f"{top_cell_name}_clusters.png")
