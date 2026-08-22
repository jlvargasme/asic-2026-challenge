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
import random
from collections import Counter, defaultdict
from dataclasses import dataclass, field

import gdstk
import matplotlib.pyplot as plt
import numpy as np
import z3
from matplotlib.collections import PatchCollection
from matplotlib.patches import Patch, Rectangle
from PySpice.Spice.Netlist import Circuit

from cell import GROUND, NMOS_MODEL, NMOS_PARAMS, PMOS_MODEL, PMOS_PARAMS
from chip import PIN_TEXTTYPE
from transistor import TransistorType, transistor_to_pyspice, transistor_to_z3
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


# ---------------------------------------------------------------------------
# Cluster-level functional analysis (z3)
# ---------------------------------------------------------------------------


def _cluster_net_var(inst, label, net_vars):
    """Resolve one transistor terminal's raw label (see transistor.py's
    module docstring -- "GN_3", "DP_1", positional identifiers assigned
    during gate/diffusion extraction, not GDS text) to a z3 BoolRef,
    shared correctly across every instance in a cluster:

      - VDD/VSS become concrete z3.BoolVal constants, same as
        cell.Cell._z3_net_var -- they're not free to solve for.
      - a terminal that IS one of `inst`'s own named pins (per
        inst.cell._analyzer.pin_name_for_net -- geometric net-equivalence
        traced down to a transistor gate or diffusion terminal, same
        mechanism cell.py's own pin classification uses; these are the
        leaf cell's own PDK-library-internal labels, the same category
        chip.py's own net resolution treats as fine to use -- see
        chip.Chip._instance_global_pins' docstring) AND that pin has a
        chip-wide net recorded in inst.global_pins, resolves to a
        variable keyed by that GLOBAL net name -- shared with whichever
        other instance's pin is on the same physical net, correctly
        modeling their real electrical connection.
      - anything else (a purely internal node, or a named pin chip.py
        couldn't resolve to a chip-wide net) resolves to a variable
        private to THIS instance. This is necessary, not just cautious:
        LeafCellAnalyzer -- and therefore the raw label string itself --
        is cached and shared per leaf-cell TYPE, not per instance (see
        pin.LeafCellAnalyzer's docstring), so two different placed
        instances of the same cell type carry the IDENTICAL raw label for
        their own, physically distinct internal nodes. Without this
        per-instance qualification, z3 would treat them as one variable
        (z3 interns Bool constants by name within a context) and silently
        short two unrelated instances' internal nodes together.
    """
    uf = inst.cell._analyzer.tracer._uf
    net_key = uf.find(label)
    if net_key == uf.find("VSS"):
        return z3.BoolVal(False)
    if net_key == uf.find("VDD"):
        return z3.BoolVal(True)

    pin_name, _ = inst.cell._analyzer.pin_name_for_net(label)
    if pin_name is not None:
        global_net = inst.global_pins.get(pin_name)
        if global_net is not None:
            return net_vars.setdefault(global_net, z3.Bool(str(global_net)))

    inst_key = f"{inst.cell.cell_name}#{inst.index}"
    key = f"{inst_key}:{net_key}"
    return net_vars.setdefault(key, z3.Bool(key))


def build_cluster_z3_circuit(cluster):
    """Combine every instance in `cluster` into one z3 switch-level model,
    mirroring cell.Cell.build_z3_circuit() but across multiple instances
    instead of one leaf cell (see _cluster_net_var for how their
    variables are kept correctly shared or correctly separate).

    Returns:
        (circuit, net_vars) -- `circuit` is a single z3 BoolRef (the AND
        of every transistor's constraint, across every instance in the
        cluster); `net_vars` is {net_name: z3.BoolRef} for every net that
        got a variable, keyed by chip-wide global net name for nets that
        resolved to one, or by an instance-qualified private key
        otherwise (see _cluster_net_var).
    """
    net_vars = {}
    constraints = []
    for inst in cluster.instances:
        for t in inst.cell.transistors:
            if t.kind not in (TransistorType.NMOS, TransistorType.PMOS) or None in (t.source_label, t.drain_label):
                continue
            constraints.append(transistor_to_z3(
                t,
                _cluster_net_var(inst, t.gate_label, net_vars),
                _cluster_net_var(inst, t.source_label, net_vars),
                _cluster_net_var(inst, t.drain_label, net_vars),
            ))
    return z3.And(*constraints), net_vars


def cluster_io_nets(chip, cluster):
    """Classify every net touching `cluster`'s own instances as one of
    this cluster's own external inputs or outputs, relative to the REST
    OF THE CHIP -- the same "is this net's driver/consumer inside or
    outside" reasoning chip.Chip._levelize() uses to find the whole
    chip's own primary inputs, just scoped to cluster membership instead
    of instance-in-chip membership.

    A net is an input if some instance inside the cluster reads it and no
    instance inside the cluster drives it (its driver, if any, is outside
    the cluster, or it's a true chip-level primary input). A net is an
    output if some instance inside the cluster drives it and it's read by
    something outside the cluster, or by nothing recorded at all (e.g. a
    true chip-level primary output). A net driven and consumed entirely
    within the cluster stays purely internal -- neither.

    Returns:
        (input_nets, output_nets) -- each a sorted list of global net
        names.
    """
    members = set(cluster.instances)
    drivers = defaultdict(list)
    consumers = defaultdict(list)
    for inst in chip.instances:
        for pin in inst.cell.output_labels:
            net = inst.global_pins.get(pin)
            if net is not None:
                drivers[net].append(inst)
        for pin in inst.cell.input_labels:
            net = inst.global_pins.get(pin)
            if net is not None:
                consumers[net].append(inst)

    input_nets, output_nets = set(), set()
    for inst in cluster.instances:
        for pin in inst.cell.input_labels:
            net = inst.global_pins.get(pin)
            if net is None:
                continue
            if not any(d in members for d in drivers.get(net, [])):
                input_nets.add(net)
        for pin in inst.cell.output_labels:
            net = inst.global_pins.get(pin)
            if net is None:
                continue
            net_consumers = consumers.get(net, [])
            if not net_consumers or any(c not in members for c in net_consumers):
                output_nets.add(net)
    return sorted(input_nets), sorted(output_nets)


def find_cluster_inputs_for_output(chip, cluster, target):
    """Find every input assignment for `cluster`'s own external input nets
    that produces `target` on ALL of its own external output nets
    simultaneously -- mirrors cell.Cell.find_inputs_for_output(), but for
    a whole recovered cluster (candidate RTL module) made of several
    instances instead of one leaf standard cell, and solving for the
    cluster's full output vector at once rather than one output pin at a
    time.

    Args:
        chip: the chip.Chip cluster's instances came from -- needed to
            tell which of the cluster's own nets are its external inputs/
            outputs versus purely internal (see cluster_io_nets).
        cluster: a Cluster, e.g. from cluster_chip().
        target: the output value(s) to solve for -- a single 0/1 if the
            cluster has exactly one external output net, else a sequence
            of 0/1 values, one per entry of this cluster's own
            output_nets (see cluster_io_nets), in that same order.

    Returns:
        (input_nets, output_nets, solutions) -- input_nets/output_nets are
        the ordered net-name lists `target` and each solution tuple's
        values correspond to; solutions is a list of tuples of 0/1 ints,
        one tuple per input assignment for which every output net matches
        `target`.
    """
    input_nets, output_nets = cluster_io_nets(chip, cluster)
    if not output_nets:
        raise ValueError("cluster has no net that reads as an external output -- nothing to solve for")

    targets = [target] if isinstance(target, int) else list(target)
    if len(targets) != len(output_nets):
        raise ValueError(
            f"cluster has {len(output_nets)} external output net(s) {output_nets}; "
            f"target must give exactly that many 0/1 value(s), got {len(targets)}"
        )

    circuit, net_vars = build_cluster_z3_circuit(cluster)

    solver = z3.Solver()
    solver.add(circuit)
    for net, value in zip(output_nets, targets):
        solver.add(net_vars[net] == bool(value))

    input_vars = [net_vars[net] for net in input_nets]
    solutions = []
    while solver.check() == z3.sat:
        model = solver.model()
        values = tuple(
            int(z3.is_true(model.eval(v, model_completion=True)))
            for v in input_vars
        )
        solutions.append(values)
        # block this exact input combination so the next check() either
        # finds a different one or comes back unsat -- same enumeration
        # technique as cell.Cell.find_inputs_for_output().
        solver.add(z3.Or([v != z3.BoolVal(bool(val)) for v, val in zip(input_vars, values)]))
    return input_nets, output_nets, solutions


# ---------------------------------------------------------------------------
# Cluster-level functional analysis (PySpice) -- for sequential clusters
# ---------------------------------------------------------------------------
#
# build_cluster_z3_circuit()'s ideal switch-level model is the right tool
# for combinational clusters, but not for sequential ones: a real, area-
# optimized standard-cell flip-flop layout shares transistors between its
# master and slave stages in ways that break the "is this node currently
# reachable to a supply rail through some chain of conducting transistors"
# reasoning a switch-level model needs to tell "being freshly written" apart
# from "holding its previous value" (confirmed by hand-tracing
# sky130_fd_sc_hd__dfrtp_2's actual 30-transistor netlist -- see this file's
# git history for the full account, including a bounded-model-checking
# attempt with exactly that reachability formula that still got 3 of 10
# probes wrong). PySpice's real analog transient simulation doesn't have
# this problem, because it's actual device physics rather than a boolean
# approximation of it -- cell.Cell.simulate_transient() already gets this
# exact cell right, per test_suite.py's own validated sequential test.
#
# The functions below are that same real-SPICE-simulation approach, scaled
# from one leaf cell up to a whole cluster (e.g. a full 16-cell shift
# register instead of one flip-flop) -- the intended hybrid is: let
# find_chip_inputs_for_output() search the combinational logic efficiently
# (something SPICE can't do -- it can only forward-simulate one concrete
# trace at a time, not search backward for "what inputs give this output"),
# then use simulate_cluster_transient() here to verify a candidate input
# sequence actually drives a sequential cluster to the register state that
# search asked for.


def _cluster_pyspice_net(inst, label):
    """Resolve one transistor terminal's raw label to a PySpice net name --
    the same resolution _cluster_net_var does for z3, just returning a
    plain string instead of a z3 BoolRef, since that's all PySpice needs:
    two elements that reference the identical net-name string are already
    connected, no interning trick required the way z3 needed one.

    VDD/VSS map to the same rail names cell.py's own single-instance
    circuits use (GROUND, "VDD"); a named pin with a chip-wide net uses
    that global net name, shared across instances exactly like
    _cluster_net_var's global-net case; anything else (a purely internal
    node, or a named pin chip.py couldn't resolve) gets an instance-
    qualified private name, for the identical reason _cluster_net_var
    needs one -- LeafCellAnalyzer, and therefore the raw label string
    itself, is cached and shared per leaf-cell TYPE, not per instance, so
    two instances of the same type carry the identical raw label for
    their own, physically distinct internal nodes.
    """
    uf = inst.cell._analyzer.tracer._uf
    net_key = uf.find(label)
    if net_key == uf.find("VSS"):
        return GROUND
    if net_key == uf.find("VDD"):
        return "VDD"

    pin_name, _ = inst.cell._analyzer.pin_name_for_net(label)
    if pin_name is not None:
        global_net = inst.global_pins.get(pin_name)
        if global_net is not None:
            return global_net

    inst_key = f"{inst.cell.cell_name}#{inst.index}"
    return f"{inst_key}:{net_key}"


def build_cluster_pyspice_circuit(cluster):
    """Combine every instance in `cluster` into one PySpice circuit,
    mirroring build_cluster_z3_circuit() but for a real transistor-level
    SPICE model instead of an ideal switch-level one (see the section
    docstring above for why this exists alongside the z3 path).

    Returns:
        a PySpice Circuit with every instance's transistors added and
        connected via _cluster_pyspice_net.
    """
    circuit = Circuit("cluster")
    circuit.model(NMOS_MODEL, "nmos", **NMOS_PARAMS)
    circuit.model(PMOS_MODEL, "pmos", **PMOS_PARAMS)

    # M<i>, not transistor.gate_label ("GP_0", ...): that raw label is only
    # unique within one leaf cell's own transistor list (LeafCellAnalyzer,
    # and therefore the label, is cached and shared per cell TYPE), so two
    # instances of the same type would otherwise both try to add a MOSFET
    # named "GP_0" and PySpice raises "Element name ... is already defined".
    i = 0
    for inst in cluster.instances:
        for t in inst.cell.transistors:
            if t.kind not in (TransistorType.NMOS, TransistorType.PMOS) or None in (t.source_label, t.drain_label):
                continue
            bulk_net = GROUND if t.kind == TransistorType.NMOS else "VDD"
            transistor_to_pyspice(
                circuit, t,
                gate_net=_cluster_pyspice_net(inst, t.gate_label),
                source_net=_cluster_pyspice_net(inst, t.source_label),
                drain_net=_cluster_pyspice_net(inst, t.drain_label),
                bulk_net=bulk_net,
                nmos_model=NMOS_MODEL, pmos_model=PMOS_MODEL,
                name=f"M{i}",
            )
            i += 1
    return circuit


def simulate_cluster_transient(chip, cluster, events, probe_times, vdd=1.8, logic_threshold=0.5,
                                edge_time=1e-9, step_time=None, end_time=None):
    """Run a real SPICE transient simulation across every instance in
    `cluster` at once, and sample its own external output nets at
    specific times -- the cluster-level analogue of
    cell.Cell.simulate_transient(), for exercising a sequential cluster
    (a whole shift register, not just one flip-flop) the way an ideal
    switch-level z3 model can't (see the section docstring above).

    Args:
        chip: the chip.Chip cluster's instances came from -- needed to
            classify this cluster's own external input/output nets (see
            cluster_io_nets).
        cluster: a Cluster, e.g. from cluster_chip().
        events: list of (time, {net_name: 0/1, ...}) tuples, in
            increasing time order, first time == 0 -- same shape as
            Cell.simulate_transient()'s own `events`, except keyed by
            this cluster's own external input net names (cluster_io_nets)
            instead of one leaf cell's local pin names. Each dict must
            give ALL of this cluster's own input nets' values from that
            time on.
        probe_times: times (seconds) to sample this cluster's own output
            nets at.
        vdd, logic_threshold, edge_time, step_time, end_time: see
            Cell.simulate_transient() -- same meaning and same defaults,
            EXCEPT edge_time, which defaults an order of magnitude slower
            here (1e-9 instead of 1e-10): a whole cluster's internal
            timing races (several chained flip-flops, not just one) are
            more sensitive to an edge too fast to resolve correctly, the
            same effect Cell.simulate_transient()'s own docs describe for
            a single sequential cell.

    Returns:
        list of (list of 0/1, one per this cluster's own output_nets --
        see cluster_io_nets), one entry per `probe_times`, in that order.
    """
    input_nets, output_nets = cluster_io_nets(chip, cluster)
    if not events or events[0][0] != 0:
        raise ValueError("events must be non-empty and start at time 0")
    for t, values in events:
        missing = [net for net in input_nets if net not in values]
        if missing:
            raise ValueError(f"event at time {t} doesn't specify a value for input net(s) {missing}")

    circuit = build_cluster_pyspice_circuit(cluster)
    circuit.V("dd_supply", "VDD", GROUND, vdd)

    for i, net in enumerate(input_nets):
        points = []
        prev_level = None
        for t, values in events:
            level = vdd if values[net] else 0.0
            if prev_level is not None and level != prev_level:
                points.append((max(t - edge_time, points[-1][0]), prev_level))
            points.append((t, level))
            prev_level = level
        circuit.PieceWiseLinearVoltageSource(f"in_{i}", net, GROUND, values=points)

    end_time = end_time or max(events[-1][0], max(probe_times)) + edge_time
    step_time = step_time or edge_time / 100

    simulator = circuit.simulator(temperature=25, nominal_temperature=25)
    analysis = simulator.transient(step_time=step_time, end_time=end_time)
    time = np.array(analysis.time)

    results = []
    for probe in probe_times:
        idx = int(np.abs(time - probe).argmin())
        results.append([
            int(float(analysis[net][idx]) >= vdd * logic_threshold)
            for net in output_nets
        ])
    return results


def _cluster_is_sequential(chip, cluster, trials=6):
    """True if `cluster` contains a stateful element (flip-flop, latch)
    that a pure switch-level z3 model can't represent -- detected
    structurally, the same way cell.Cell.simulate_z3() already detects it
    per output pin (see its own "isn't uniquely determined" guard): pin
    the cluster's own external input nets to several different
    assignments in turn, and check whether every external output net is
    STILL forced to one value under each of them.

    A purely combinational network says yes for any input assignment --
    that's what "combinational" means. A network with an internal
    feedback loop (the switch-level signature of a latch/flip-flop) can
    hold either state regardless of its current inputs, so it says no --
    for MOST input assignments, but not necessarily every single one: a
    flip-flop with an active reset input is a real counterexample. Pinning
    every input to 0 first looked like a safe, generic choice, but for a
    DFF whose reset happens to be active-LOW, all-zero means "reset
    asserted", which deterministically forces Q -- exactly the one
    assignment where a genuinely stateful cell stops looking stateful.
    Found on warmup/04_final.gds: both 8-DFF shift-register clusters were
    misclassified as combinational this way, because their own RESET_B
    net was one of the inputs pinned to 0 -- so this now tries several
    different assignments (all-0, all-1, and a handful of pseudo-random
    ones) and calls a cluster sequential if ANY of them exposes the
    ambiguity, since a real reset/set condition only ever "collapses" the
    state for one specific polarity, not for every pattern tried.

    No cell name or label is involved anywhere in this check -- purely a
    property of the transistor network's own topology.
    """
    input_nets, output_nets = cluster_io_nets(chip, cluster)
    if not output_nets:
        return False

    circuit, net_vars = build_cluster_z3_circuit(cluster)
    rng = random.Random(0)
    patterns = [
        [False] * len(input_nets),
        [True] * len(input_nets),
        *([rng.random() < 0.5 for _ in input_nets] for _ in range(trials)),
    ]

    solver = z3.Solver()
    solver.add(circuit)
    for pattern in patterns:
        solver.push()
        for net, value in zip(input_nets, pattern):
            solver.add(net_vars[net] == value)
        for net in output_nets:
            var = net_vars[net]
            solver.push()
            solver.add(var == True)
            can_high = solver.check() == z3.sat
            solver.pop()
            solver.push()
            solver.add(var == False)
            can_low = solver.check() == z3.sat
            solver.pop()
            if can_high and can_low:
                solver.pop()
                return True
        solver.pop()
    return False


def build_chip_z3_circuit(chip, clusters):
    """Combine every non-sequential cluster's z3 model (build_cluster_z3_
    circuit) into one chip-wide combinational model, connected across
    cluster boundaries automatically: two different clusters' pins on the
    same chip-wide net (chip.Instance.global_pins) already resolve to the
    identical z3 variable purely because z3 interns Bool constants by
    name within one context -- calling z3.Bool("some_net") in cluster A's
    build and again in cluster B's build returns the SAME term either
    way, with no extra wiring code needed here.

    Sequential clusters (see _cluster_is_sequential) are deliberately left
    OUT of the combined circuit -- their own transistor constraints have
    no well-formed combinational meaning (an ideal-switch model of a
    flip-flop has an internal feedback loop with no notion of clock edges
    or history, exactly what a real transient/clocked simulation is for;
    see cell.Cell.simulate_transient()). Excluding them, rather than
    trying to model them, is what turns their own output nets into FREE
    variables here -- which is the intended, standard way to analyze a
    sequential design's combinational logic in isolation: a register's
    output becomes a stand-in for "whatever state it currently holds",
    solvable for, rather than something this model tries to derive.

    Returns:
        (circuit, net_vars, free_nets, chip_output_nets) --
        `circuit` is the combined z3 BoolRef; `net_vars` is
        {net_name: z3.BoolRef} for every net any combinational cluster's
        transistors touch; `free_nets` is every net read by some
        combinational cluster but driven by none of them -- true chip
        primary inputs (a net no cluster drives at all -- note a purely
        timing signal like a clock never shows up here in the first
        place, since nothing combinational ever reads it) and sequential-
        cluster outputs (register state) alike, indistinguishable at this
        level and both genuinely free to solve for; `chip_output_nets` is
        every net driven by some combinational cluster but read by none
        of them -- this chip's own combinational outputs.
    """
    seq_flags = {id(c): _cluster_is_sequential(chip, c) for c in clusters}
    combinational = [c for c in clusters if not seq_flags[id(c)]]

    circuits = []
    net_vars = {}
    driven, consumed = set(), set()
    for cluster in combinational:
        circuit, cluster_vars = build_cluster_z3_circuit(cluster)
        circuits.append(circuit)
        net_vars.update(cluster_vars)
        input_nets, output_nets = cluster_io_nets(chip, cluster)
        driven.update(output_nets)
        consumed.update(input_nets)

    free_nets = sorted(consumed - driven)
    chip_output_nets = sorted(driven - consumed)
    return z3.And(*circuits), net_vars, free_nets, chip_output_nets


def find_chip_inputs_for_output(chip, clusters, target, output_net=None):
    """Find every free-variable assignment (chip primary inputs and/or
    sequential-cluster/register state, see build_chip_z3_circuit) that
    produces `target` on one of this chip's own combinational output
    nets -- the chip-wide analogue of find_cluster_inputs_for_output(),
    built by combining every recovered cluster's own z3 model instead of
    one cluster's.

    Args:
        chip: the chip.Chip clusters was computed from.
        clusters: list of Cluster covering (ideally all of) chip's own
            instances, e.g. from cluster_chip().
        target: the output value (0/1) to solve for.
        output_net: which of build_chip_z3_circuit's own chip_output_nets
            to solve for. Required if there's more than one; defaults to
            the only one otherwise.

    Returns:
        (free_nets, solutions) -- free_nets is the ordered net-name list
        each solution tuple's values correspond to; solutions is a list
        of tuples of 0/1 ints, one tuple per assignment for which
        output_net == target.
    """
    circuit, net_vars, free_nets, chip_output_nets = build_chip_z3_circuit(chip, clusters)
    if not chip_output_nets:
        raise ValueError("no net in this chip's combinational logic reads as a chip-level output -- nothing to solve for")
    if output_net is None:
        if len(chip_output_nets) != 1:
            raise ValueError(f"chip has {len(chip_output_nets)} combinational output net(s) {chip_output_nets}; pass output_net explicitly")
        output_net = chip_output_nets[0]
    elif output_net not in chip_output_nets:
        raise ValueError(f"{output_net!r} isn't one of this chip's own combinational output nets {chip_output_nets}")

    solver = z3.Solver()
    solver.add(circuit)
    solver.add(net_vars[output_net] == bool(target))

    free_vars = [net_vars[net] for net in free_nets]
    solutions = []
    while solver.check() == z3.sat:
        model = solver.model()
        values = tuple(
            int(z3.is_true(model.eval(v, model_completion=True)))
            for v in free_vars
        )
        solutions.append(values)
        solver.add(z3.Or([v != z3.BoolVal(bool(val)) for v, val in zip(free_vars, values)]))
    return free_nets, solutions


# ---------------------------------------------------------------------------
# Human-readable labels for find_chip_inputs_for_output's free nets
# ---------------------------------------------------------------------------
#
# find_chip_inputs_for_output() reports its free variables by their raw,
# synthetic net name (e.g. "A0#3") -- a real identifier (see
# chip.Chip._net_name's own 1:1 root-to-name guarantee), but not a
# meaningful one to a human reading the result. The functions below build
# a best-effort DISPLAY label instead -- e.g. "A#0".."A#7" for one
# sequential cluster's own group of free nets -- WITHOUT feeding anything
# label-derived back into the actual analysis (cluster_chip,
# build_chip_z3_circuit, etc. all stay exactly as label-free as before;
# only the printed report changes) and WITHOUT assuming any particular
# cell library's naming or topology (no cell type name appears anywhere
# below -- this has to work on a design built from a completely different
# standard-cell library, not just this project's own sky130 example):
#
#   - Which free nets belong together as one group comes from
#     _cluster_driving_net(): purely "does some cluster have an instance
#     whose own output resolves to this net" -- true regardless of what
#     kind of sequential element that cluster turns out to be.
#   - Each group's NAME comes from matching the cluster's own external
#     input nets (cluster_io_nets(), already label-free) against the
#     chip's TRUE top-level port labels (_all_top_level_port_nets()) --
#     its public external interface (e.g. what a datasheet would call
#     pin "A"), carefully distinguished from the many per-instance LOCAL
#     pins that happen to share the same text by requiring the label sit
#     outside every placed instance's own footprint. Falls back to a
#     generic "reg<cluster id>" name if no single input net matches
#     exactly one top-level port.
#   - Each net's NUMBER within a group is just a stable, deterministic
#     order (sorted net name) -- NOT a reconstruction of true bit
#     significance (which bit is the MSB/LSB). Recovering that would
#     need assuming a specific register topology (e.g. "an unlabeled pin
#     hold-vs-shift convention for a shift register built from THESE
#     specific cell types"), which is exactly the kind of design-specific
#     assumption this project's own puzzle GDS won't cooperate with.


def _all_top_level_port_nets(chip):
    """{port_name: net} for every TRUE top-level port label in the chip --
    a label sitting outside every placed instance's own footprint, so it
    can't be one of the many per-instance LOCAL pins that happen to share
    the same text (e.g. "A" is one of the most common local pin names in
    almost any standard-cell library). This is the chip's own public
    external interface, not a secret about its internal structure -- see
    this section's docstring above."""
    footprints = [
        r.bounding_box() for r in chip.top_cell.references
        if hasattr(r.cell, "name") and r.bounding_box() is not None
    ]
    ports = {}
    for lbl in chip.labels:
        if lbl.texttype != PIN_TEXTTYPE or lbl.text in ports:
            continue
        x, y = lbl.origin
        if any(fx0 <= x <= fx1 and fy0 <= y <= fy1 for (fx0, fy0), (fx1, fy1) in footprints):
            continue  # a per-instance local pin that happens to share this text, not a true top-level port
        for i in chip._grid.near(x, y):
            key, _, poly = chip._shapes[i]
            if gdstk.inside([lbl.origin], [poly])[0]:
                ports[lbl.text] = chip._net_name(chip._uf.find(key))
                break
    return ports


def _cluster_driving_net(clusters, net):
    """Whichever cluster has an instance whose own output resolves to
    `net`, or None if no cluster drives it at all (a true chip primary
    input, e.g. clk -- see build_chip_z3_circuit's own free_nets
    docstring for why both cases end up looking identical there)."""
    for cluster in clusters:
        for inst in cluster.instances:
            for pin in inst.cell.output_labels:
                if inst.global_pins.get(pin) == net:
                    return cluster
    return None


def _max_transitive_fanout(cluster, net):
    """The largest fanout (count of directly-consuming instances) found
    anywhere while following `net` forward through `cluster`: net's own
    direct consumers, then each of THEIR outputs' own consumers, and so
    on -- see label_free_nets' own inline comment for why this is what
    tells a control/clock signal apart from a genuine shift-register data
    signal, without assuming which is which by name or cell type.

    Bounded to len(cluster.instances) + 1 hops -- more than enough to
    reach every instance at least once in a cluster this size, so no
    reachable broadcast point goes unseen."""
    seen_nets = {net}
    frontier = [net]
    max_fanout = 0
    for _ in range(len(cluster.instances) + 1):
        if not frontier:
            break
        next_frontier = []
        for n in frontier:
            consumers = [
                inst for inst in cluster.instances
                for pin in inst.cell.input_labels
                if inst.global_pins.get(pin) == n
            ]
            max_fanout = max(max_fanout, len(consumers))
            for inst in consumers:
                for pin in inst.cell.output_labels:
                    out_net = inst.global_pins.get(pin)
                    if out_net is not None and out_net not in seen_nets:
                        seen_nets.add(out_net)
                        next_frontier.append(out_net)
        frontier = next_frontier
    return max_fanout


def label_free_nets(chip, clusters, free_nets):
    """Best-effort human-readable label for each of build_chip_z3_circuit's
    own free nets -- e.g. "A#0".."A#7" for one sequential cluster's own
    group of free nets (see this section's docstring above for exactly
    what each part of the label does and doesn't assume) -- falling back
    to the raw net name itself for any free net this can't confidently
    label (e.g. a design without a clean top-level port to match against).

    Returns {net_name: label} covering every entry of `free_nets` (always
    a complete mapping -- an unlabelable net just maps to itself).
    """
    labels = {net: net for net in free_nets}
    net_to_port = {net: name for name, net in _all_top_level_port_nets(chip).items()}

    groups = defaultdict(list)
    for net in free_nets:
        source = _cluster_driving_net(clusters, net)
        if source is not None:
            groups[id(source)].append(net)
        elif net in net_to_port:
            labels[net] = net_to_port[net]  # a lone primary input, not part of any group

    for cluster_id, nets in groups.items():
        cluster = next(c for c in clusters if id(c) == cluster_id)
        input_nets, _ = cluster_io_nets(chip, cluster)
        matches = [n for n in input_nets if n in net_to_port]

        # A sequential cluster's own external inputs typically include
        # SEVERAL top-level-connected signals at once -- clk, reset,
        # enable, AND whatever actually carries new data in -- so "exactly
        # one input net matches a top-level port" is normal, not a sign of
        # ambiguity. What distinguishes a genuine data input from a
        # control signal, without assuming which is which by name or cell
        # type: a control signal reaches a broadcast point SOMEWHERE
        # downstream (e.g. a clock buffer's own output reaching every
        # flip-flop's CLK pin at once), while a genuine shift-register
        # data signal never does -- pure serial propagation, no
        # branching, is what makes it a shift register in the first
        # place. Raw immediate fanout alone isn't enough to tell them
        # apart (a local clock buffer means clk's own direct fanout is
        # also 1, same as the data path), so this follows each candidate
        # forward through the cluster and takes the largest fanout seen
        # anywhere along the way.
        group_name = min(matches, key=lambda n: (_max_transitive_fanout(cluster, n), n), default=None)
        group_name = net_to_port[group_name] if group_name is not None else f"reg{cluster.id}"
        for i, net in enumerate(sorted(nets)):
            labels[net] = f"{group_name}#{i}"

    return labels


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

    # python clustering.py [gds_file] [top_cell_name] [epsilon_scale] [target]
    # target is optional -- omit it to just cluster + plot (the original
    # behavior); pass 0 or 1 to also solve build_chip_z3_circuit()'s
    # combinational logic for that chip-level output value.
    gds_file = sys.argv[1] if len(sys.argv) > 1 else "./warmup/04_final.gds"
    top_cell_name = sys.argv[2] if len(sys.argv) > 2 else "adder_demo"
    epsilon_scale = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    target = int(sys.argv[4]) if len(sys.argv) > 4 else None

    chip = Chip(gds_file, top_cell_name)
    print(f"{top_cell_name}: {len(chip.instances)} logic instance(s)")

    clusters = cluster_chip(chip, epsilon_scale=epsilon_scale)
    print(f"\n{len(clusters)} cluster(s) found (epsilon_scale={epsilon_scale}):")
    for c in clusters:
        top_cells = ", ".join(f"{name}x{n}" for name, n in c.cell_type_counts.most_common(5))
        print(f"  cluster {c.id}: {c.size} instance(s) -- {top_cells}")

    plot_clusters(chip, clusters, out_path=f"{top_cell_name}_clusters.png")

    if target is not None:
        free_nets, solutions = find_chip_inputs_for_output(chip, clusters, target)
        labels = label_free_nets(chip, clusters, free_nets)
        display_names = [labels[net] for net in free_nets]
        print(f"\n{len(free_nets)} free net(s) (chip primary inputs + sequential-cluster/register "
              f"state -- see build_chip_z3_circuit's docstring): {display_names}")
        print(f"{len(solutions)} assignment(s) give output={target}:")
        for values in solutions[:20]:
            print(f"  {dict(zip(display_names, values))}")
        if len(solutions) > 20:
            print(f"  ... and {len(solutions) - 20} more")
