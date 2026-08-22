"""
Decompiler: write a best-effort <chip_name>.v Verilog file from a chip.Chip
+ clustering.py's recovered clusters -- the last stage of this project's
bottom-up pipeline (geometry -> transistors -> cells -> chip netlist ->
clusters -> RTL text).

AST shape
---------

Two-level, matching how the rest of this project already models a design
(chip.Instance groups by cell TYPE via Cell's own caching; clustering.py
groups instances by physical cluster):

  - LeafModule: one decompiled Verilog module per distinct leaf CELL TYPE
    (e.g. one "sky130_fd_sc_hd__and3_2" module, however many instances of
    it exist) -- this is where "what does this cell compute" gets solved,
    once per type, and is the only place gate/flip-flop LOGIC lives.
  - ClusterModule: one Verilog module per clustering.Cluster, whose body
    is *purely structural* -- wires plus instantiations of LeafModules,
    built directly from cluster_io_nets()/Instance.global_pins (already
    correct, geometrically-resolved connectivity; no new net analysis
    here). No cluster-level behavioral logic is ever synthesized -- a
    16-cell shift register cluster is not characterized as one black box;
    it's 8 dfrtp_2 instances + 8 mux2_1 instances + wires, each instance's
    own behavior coming from its LeafModule. This is what keeps sequential
    decompilation tractable: only ONE flip-flop's pin behavior needs
    characterizing, not a whole register file's.
  - ChipModule: the top module, instantiating one ClusterModule per
    cluster (cluster_chip() covers every chip.Instance, so nothing else
    needs instantiating at this level) and wiring them together by the
    same chip-wide net identity chip.py already resolved.

Net naming is purely cosmetic here, same as chip.py's own _net_name(): a
net crossing a cluster boundary gets a human-readable Verilog identifier
from a TRUE top-level port label if it has one (clustering.py's own
_all_top_level_port_nets() -- PDK-legitimate/public-interface data, not
this design's internal secret, see HANDOFF.md), else a sanitized version
of its synthetic chip-wide name. Purely-internal, never-boundary-crossing
nets are named per-cluster-module, since Verilog modules are separately
scoped and don't need to agree with each other.

Combinational leaf cells: z3 truth table + Quine-McCluskey
------------------------------------------------------------

Cell.z3_circuit/z3_inputs/z3_outputs already gives a validated,
uniquely-determined boolean function per output pin (Cell.simulate_z3()'s
own can-be-high/can-be-low check already proves this for whichever input
combo it's given). Standard cells have few inputs (this project's own
MAX_COMB_INPUTS mirrors test_suite.py's MAX_INPUTS=10 guard), so the full
truth table is just brute-force enumeration via simulate_z3() -- if ANY
combination raises ValueError ("isn't uniquely determined"), that's
exactly test_suite.py's own documented signal that this cell is
sequential (has internal state), not a bug in the enumeration.

The resulting truth table gets minimized into a sum-of-products boolean
expression via a classic Quine-McCluskey pass (_minimize_sop) -- no
don't-cares needed, since every combinational standard cell's truth table
is fully specified for every input combination. This produces one
"assign OUT = (A & ~B) | C;" line per output, not just a correct-but-ugly
case/lookup-table -- closer to what a human would actually write for that
gate.

Sequential leaf cells: known-cell table (tier 1) + characterization (tier 2)
------------------------------------------------------------------------------

Per HANDOFF.md's own rule, a standard-cell library's OWN pin semantics
(e.g. sky130_fd_sc_hd__dfrtp_2 always has D/Q/CLK/RESET_B meaning exactly
what a datasheet says) are legitimate PDK/vendor data, not this specific
taped-out design's secret -- fine to use, unlike a per-instance net label.
KNOWN_SEQUENTIAL_CELLS is that table: a hand-authored SequentialPinMap per
known cell name, matching test_suite.py's own SEQUENTIAL_TESTS entries
(currently just dfrtp_2 -- active-low async RESET_B, positive-edge D
capture, the one sequential cell this project has validated so far).

For any sequential cell NOT in that table (an unknown library, or
whatever puzzle.gds turns out to use), _characterize_sequential_tier2()
does real reverse engineering, in two stages that deliberately use
different tools for exactly the reasons clustering.py's own module
docstring already worked out:

  1. Reset/set pin(s): a pure switch-level z3 model over-determines a
     stateful cell's output for MOST input patterns (that's what "has
     internal state" means), but an asynchronous reset/set pin, by
     construction, connects the output node directly to a rail through a
     transistor path that doesn't depend on clock phase -- so pinning
     JUST that one pin (leaving every other input, including the clock,
     completely free) still forces every output to one value. This is the
     exact test clustering.py's _cluster_is_sequential() already uses
     successfully (see its own docstring: pinning RESET_B=0 alone
     deterministically forces Q even with everything else free) --
     reused here per-pin instead of per-whole-pattern, to find WHICH pin
     and WHICH polarity does the forcing, not just "some pin does".
     Structurally can only ever find ASYNCHRONOUS overrides -- a purely
     synchronous reset only takes effect at a clock edge, so leaving the
     clock free would keep the output ambiguous, not force it. That's a
     real, known gap (documented on SequentialOverride), not an oversight.
  2. Clock + data pins: switch-level z3 has no notion of history/edges
     (see clustering.py's own extensive account of why a bounded-model-
     -checking attempt at this failed on the real, transistor-shared
     dfrtp_2 layout) -- so this uses actual PySpice transient simulation
     (Cell.simulate_transient(), the same tool cell.py's own sequential
     test already trusts) with a clocked stimulus sequence shaped to
     distinguish "edge-triggered capture" from "just changed with the
     other pin": capture D=0 on an edge, change D without an edge (Q
     must hold), transition the OTHER edge direction (Q must still hold),
     then capture D=1 on a real edge. Every (candidate-clock-pin,
     candidate-data-pin, edge-polarity) hypothesis is tried in turn; the
     first whose resulting Q trace matches (or exactly inverts, for a
     QN-style output) the expected shape is accepted.

FORCE_TIER2 in __main__ (--force-tier2) exists so Tier 2 can be run and
checked against Tier 1's known-correct answer on the ONE cell this
project has ground truth for (dfrtp_2, via warmup/04_final.gds) --
_validate_tier2_against_known() diffs the two structurally and prints a
PASS/mismatch report. That's the validation loop before trusting Tier 2
as the ONLY path on a design (like puzzle.gds) with no known-cell table
entry to fall back on at all.

Cluster-type deduplication (networkx graph isomorphism)
----------------------------------------------------------

clustering.cluster_chip() groups instances by physical proximity, not by
"is this the same circuit as some other cluster" -- so two structurally
identical clusters (this project's own two 8-bit shift registers) come
out as two entirely separate Cluster objects with no relationship to each
other recorded anywhere. Emitting one ClusterModule per Cluster (the
first version of this file) is correct but throws that reuse away --
adder_demo.v ends up with two near-duplicate module bodies instead of one
shared module instantiated twice, unlike 00_source.v's own single
shift_register module.

_dedupe_cluster_types() recovers that reuse the same way LeafModule reuse
already works one level down (cache by identity, here "identity" just
means something richer than a name string): build one graph per cluster
-- one node per instance (colored by its LeafModule name, so a dfrtp_2
can only ever match another dfrtp_2), one node per boundary/internal net
(colored by port direction, so an input can't match an output), edges
labeled by pin name -- and treat two clusters as the same TYPE if their
graphs are isomorphic. networkx.weisfeiler_lehman_graph_hash() is used
only as a cheap same-bucket prefilter (a WL hash collision does not by
itself prove isomorphism, especially for regular/symmetric netlists like
a shift register's own repeating bit-slice structure); the actual
decision always comes from networkx's exact GraphMatcher.is_isomorphic()
with node_match/edge_match enforcing the colorings above. This makes no
assumption about what the circuit computes -- purely a graph-structure
fact -- so it's exactly as legitimate to run on puzzle.gds as on the
warmup design.

Once two clusters are confirmed isomorphic, GraphMatcher's own node
mapping gives, for free, a correspondence between the CANONICAL cluster's
boundary nets and the other cluster's own boundary nets in the same
structural position -- that's what lets every instance of a shared type
be connected correctly at the top level using only the one canonical
module body.

Bus/vector port grouping (net-endpoint analysis, no external library)
--------------------------------------------------------------------------

A second, independent AST pass: group several of a cluster's own scalar
boundary ports into one Verilog vector port when they share the exact
same *other* cluster on their far end (e.g. a shift-register cluster's 8
separate Q outputs, which -- after dedup -- would otherwise all be
declared as 8 separate scalar ports even though every one of them feeds
the SAME adder cluster). The grouping key is purely net-identity-based
(chip.py's own union-find, via which cluster's cluster_io_nets() lists a
given net as an input/output), never a per-design label -- so this is
just as legitimate on puzzle.gds as it is here.

Deliberately scoped to point-to-point cluster-to-cluster nets only (a net
whose consumers span more than one cluster, or whose driver is a true
top-level primary input, is left as an ordinary scalar port) --
grouping a genuinely broadcast control signal (clk, reset) together with
unrelated data bits purely because they happen to both trace back to "the
chip's own top-level boundary" would be a real correctness-of-meaning
risk (see clustering.label_free_nets()'s own transitive-fanout dance for
exactly this problem one level up); restricting to "exactly one other
CLUSTER on the far end" sidesteps it entirely without needing that
heavier machinery. Like label_free_nets(), the resulting bit ORDER is a
stable, deterministic sort -- not a reconstruction of true bit
significance (MSB/LSB), the same documented limitation as elsewhere in
this project.

The grouping decision itself is computed once, from the canonical
cluster of a type only, then applied structurally (by canonical port
POSITION) to every other instance of that type -- always electrically
correct even if a given member's own wiring wouldn't have independently
produced the identical grouping, since concatenating a member's own
per-bit nets in the same position order connects every bit to exactly
the right net either way; only the cosmetic "is this really one bus"
framing is inherited from the canonical instance, not re-verified per
member.
"""

from collections import Counter, defaultdict
from dataclasses import dataclass, field
from itertools import permutations, product

import re

import networkx as nx
import z3

from clustering import _all_top_level_port_nets, cluster_chip, cluster_io_nets

MAX_COMB_INPUTS = 10  # mirrors test_suite.py's own MAX_INPUTS guard


class DecompileError(Exception):
    """Raised when a leaf cell couldn't be turned into a LeafModule --
    caught by build_ast(), which emits an 'unsupported' stub instead of
    failing the whole chip."""


class SequentialCharacterizationError(DecompileError):
    """Tier 2 characterization (_characterize_sequential_tier2) couldn't
    find a (clock, data, edge-polarity) hypothesis whose simulated Q
    trace matched an edge-triggered flip-flop's expected shape -- e.g.
    the cell isn't a plain D-flip-flop (a latch, a cell with more than
    one non-override input pin, scan-enable, ...), a documented, known
    gap rather than a silent wrong answer."""


# ---------------------------------------------------------------------------
# Verilog identifier / net naming
# ---------------------------------------------------------------------------


def _sanitize_ident(s):
    s = re.sub(r"[^0-9A-Za-z_$]", "_", str(s))
    if not s or s[0].isdigit():
        s = "_" + s
    return s


def _dedupe(base, used):
    ident, n = base, 0
    while ident in used:
        n += 1
        ident = f"{base}_{n}"
    return ident


class _ScopedNamer:
    """A per-module-scope net->identifier resolver, seeded with the
    chip-wide boundary net names so a cluster module's own port names
    always agree with the top module's wire names for the same physical
    net, while purely-internal nets get fresh, locally-deduped names
    (Verilog modules are separately scoped, so internal nets in different
    clusters don't need to agree with each other)."""

    def __init__(self, base_map, used):
        self._map = dict(base_map)
        self._used = set(used)

    def __call__(self, net):
        if net in self._map:
            return self._map[net]
        ident = _dedupe(_sanitize_ident(net), self._used)
        self._used.add(ident)
        self._map[net] = ident
        return ident


class _NetNamer:
    """Consistent, collision-free Verilog identifiers for every net that
    crosses a cluster module's own boundary (see _ScopedNamer). Prefers a
    real top-level port label (clustering._all_top_level_port_nets --
    the chip's own public interface, not a per-design secret) when the
    net has one, purely for readability; the net's own IDENTITY driving
    every wiring decision is still chip.py's geometric net resolution,
    never a label."""

    def __init__(self, chip, boundary_nets):
        port_label_by_net = {net: name for name, net in _all_top_level_port_nets(chip).items()}
        self._map = {}
        used = set()
        for net in sorted(boundary_nets):
            base = _sanitize_ident(port_label_by_net.get(net, net))
            ident = _dedupe(base, used)
            used.add(ident)
            self._map[net] = ident
        self._used = used

    def scope(self):
        return _ScopedNamer(self._map, self._used)


# ---------------------------------------------------------------------------
# Combinational leaf cells: z3 truth table -> minimized SOP
# ---------------------------------------------------------------------------


def _cell_truth_table(cell):
    """Full input->output truth table for `cell` via Cell.simulate_z3(),
    or None if any input combination leaves an output undetermined --
    exactly test_suite.py's own documented signal that this cell has
    internal state (see its module docstring) rather than a bug here."""
    n = len(cell.input_labels)
    table = {}
    for combo in product((0, 1), repeat=n):
        try:
            table[combo] = tuple(cell.simulate_z3(combo))
        except ValueError:
            return None
    return table


def _combo_to_int(combo):
    n = len(combo)
    return sum(v << (n - 1 - i) for i, v in enumerate(combo))


def _minimize_sop(minterms, n):
    """Prime-implicant cover for `minterms` (ints in [0, 2**n), no
    don't-cares) via classic Quine-McCluskey, reduced with essential-
    prime-implicant extraction plus greedy set cover for anything left
    uncovered (not provably minimum, but always a valid, much-reduced
    cover -- good enough for readable Verilog, not a formal minimizer).

    Returns a list of terms, each a length-n string over '0'/'1'/'-'.
    """
    groups = defaultdict(dict)  # ones-count -> {pattern: set(minterms)}
    for m in minterms:
        pat = format(m, f"0{n}b")
        groups[pat.count("1")][pat] = {m}

    primes = {}
    while groups:
        next_groups = defaultdict(dict)
        used = set()
        counts = sorted(groups)
        for c in counts:
            nxt = c + 1
            if nxt not in groups:
                continue
            for pat_a, cov_a in groups[c].items():
                for pat_b, cov_b in groups[nxt].items():
                    diff = [i for i in range(n) if pat_a[i] != pat_b[i]]
                    if len(diff) == 1 and pat_a[diff[0]] != "-" and pat_b[diff[0]] != "-":
                        i = diff[0]
                        merged = pat_a[:i] + "-" + pat_a[i + 1:]
                        next_groups[merged.count("1")][merged] = cov_a | cov_b
                        used.add((c, pat_a))
                        used.add((nxt, pat_b))
        for c, pats in groups.items():
            for pat, cov in pats.items():
                if (c, pat) not in used:
                    primes[pat] = cov
        groups = next_groups

    remaining = set(minterms)
    chosen = []
    prime_items = list(primes.items())

    changed = True
    while remaining and changed:
        changed = False
        coverage = defaultdict(list)
        for pat, cov in prime_items:
            for m in cov & remaining:
                coverage[m].append(pat)
        for m, pats in coverage.items():
            if len(pats) == 1 and pats[0] not in chosen:
                chosen.append(pats[0])
                remaining -= primes[pats[0]]
                changed = True
                break

    while remaining:
        candidates = [(pat, primes[pat] & remaining) for pat, _ in prime_items if pat not in chosen]
        candidates = [(pat, cov) for pat, cov in candidates if cov]
        if not candidates:
            break
        best_pat, best_cov = max(candidates, key=lambda pc: len(pc[1]))
        chosen.append(best_pat)
        remaining -= best_cov

    return chosen


def _term_to_verilog(pattern, input_labels):
    lits = [name if ch == "1" else f"~{name}" for ch, name in zip(pattern, input_labels) if ch != "-"]
    return " & ".join(lits) if lits else "1'b1"


def _sop_expr(minterms, n, input_labels):
    if not minterms:
        return "1'b0"
    if len(minterms) == 2 ** n:
        return "1'b1"
    parts = [_term_to_verilog(t, input_labels) for t in _minimize_sop(minterms, n)]
    if len(parts) == 1:
        return parts[0]
    return " | ".join(f"({p})" if " & " in p else p for p in parts)


# ---------------------------------------------------------------------------
# Sequential leaf cells: tier 1 (known cell) + tier 2 (characterized)
# ---------------------------------------------------------------------------


@dataclass
class SequentialOverride:
    """One pin whose own value alone forces this cell's output(s) to a
    fixed value regardless of clock/data -- an async reset/set. `forced`
    maps output pin name -> the value it's forced to."""

    pin: str
    active_value: int
    forced: dict
    is_async: bool = True


@dataclass
class SequentialPinMap:
    data: str
    clk: str
    clk_edge: str  # "pos" or "neg"
    outputs: dict  # output pin name -> "normal" | "inverted" (relative to data)
    overrides: list = field(default_factory=list)
    source: str = "tier2"


KNOWN_SEQUENTIAL_CELLS = {
    # sky130_fd_sc_hd__dfrtp_2: positive-edge D flip-flop, asynchronous
    # active-low RESET_B -- matches test_suite.py's own SEQUENTIAL_TESTS
    # entry exactly (same cell, same documented behavior).
    "sky130_fd_sc_hd__dfrtp_2": SequentialPinMap(
        data="D", clk="CLK", clk_edge="pos", outputs={"Q": "normal"},
        overrides=[SequentialOverride(pin="RESET_B", active_value=0, forced={"Q": 0}, is_async=True)],
        source="tier1(known-cell)",
    ),
}


def _find_overrides_z3(cell):
    """Every (pin, value) whose pinning alone (everything else free)
    forces every one of `cell`'s outputs to a single value -- see the
    module docstring for why this can only ever find ASYNCHRONOUS
    overrides, and why that's a structural property, not a shortcoming
    of this particular search."""
    overrides = []
    for pin in cell.input_labels:
        pin_var = cell.z3_inputs[pin]
        for v in (0, 1):
            solver = z3.Solver()
            solver.add(cell.z3_circuit)
            solver.add(pin_var == bool(v))
            forced = {}
            ok = True
            for out in cell.output_labels:
                out_var = cell.z3_outputs[out]
                solver.push()
                solver.add(out_var == True)
                can_high = solver.check() == z3.sat
                solver.pop()
                solver.push()
                solver.add(out_var == False)
                can_low = solver.check() == z3.sat
                solver.pop()
                if can_high and not can_low:
                    forced[out] = 1
                elif can_low and not can_high:
                    forced[out] = 0
                else:
                    ok = False
                    break
            if ok and forced:
                overrides.append(SequentialOverride(pin=pin, active_value=v, forced=forced, is_async=True))
    return overrides


def _try_clk_data_hypothesis(cell, clk_pin, data_pin, edge, inactive):
    """Run one (clock pin, data pin, edge polarity) hypothesis through a
    clocked stimulus and check the resulting Q trace against the shape a
    genuine edge-triggered D-flip-flop must produce: capture on an edge,
    hold across a data change with no edge, hold across the OTHER edge
    direction, capture again on the next real edge. Returns
    {output_label: "normal"|"inverted"} if every output matches (directly
    or exactly inverted, for a QN-style output), else None."""
    idle, active = (0, 1) if edge == "pos" else (1, 0)
    step = 20e-9
    edge_time = 1e-9

    base = dict(inactive)
    for p in cell.input_labels:
        base.setdefault(p, 0)
    base[clk_pin] = idle
    base[data_pin] = 0

    events = [(0.0, dict(base))]

    def push(t, **changes):
        vals = dict(events[-1][1])
        vals.update(changes)
        events.append((t, vals))

    push(1 * step, **{clk_pin: active})   # edge0: capture D=0
    push(2 * step, **{data_pin: 1})       # D changes, no edge -- must hold
    push(3 * step, **{clk_pin: idle})     # opposite-direction transition -- must hold
    push(4 * step, **{clk_pin: active})   # edge1: capture D=1
    push(5 * step, **{data_pin: 0})       # D changes, no edge -- must hold
    push(6 * step, **{clk_pin: idle})     # opposite-direction transition -- must hold
    push(7 * step, **{clk_pin: active})   # edge2: capture D=0

    probe_times = [(i + 0.5) * step for i in range(1, 8)]
    expected = [0, 0, 0, 1, 1, 1, 0]

    try:
        traces = cell.simulate_transient(events, probe_times, edge_time=edge_time)
    except Exception:
        return None

    outputs = {}
    for oi, out_name in enumerate(cell.output_labels):
        actual = [row[oi] for row in traces]
        if actual == expected:
            outputs[out_name] = "normal"
        elif actual == [1 - v for v in expected]:
            outputs[out_name] = "inverted"
        else:
            return None
    return outputs


def _characterize_sequential_tier2(cell, overrides):
    claimed = {o.pin for o in overrides}
    remaining = [p for p in cell.input_labels if p not in claimed]
    if len(remaining) != 2:
        raise SequentialCharacterizationError(
            f"{cell.cell_name}: expected exactly 2 non-override input pin(s) "
            f"(clock + data) after {[o.pin for o in overrides]}, found {remaining}"
        )

    inactive = {o.pin: 1 - o.active_value for o in overrides}
    for clk_pin, data_pin in permutations(remaining, 2):
        for edge in ("pos", "neg"):
            outputs = _try_clk_data_hypothesis(cell, clk_pin, data_pin, edge, inactive)
            if outputs is not None:
                return SequentialPinMap(
                    data=data_pin, clk=clk_pin, clk_edge=edge, outputs=outputs,
                    overrides=overrides, source="tier2(characterized)",
                )

    raise SequentialCharacterizationError(
        f"{cell.cell_name}: no (clock, data, edge) hypothesis over {remaining} "
        f"produced an edge-triggered D-flip-flop trace"
    )


def _validate_tier2_against_known(cell_name, tier2, known):
    def override_key(o):
        return (o.pin, o.active_value, tuple(sorted(o.forced.items())), o.is_async)

    mismatches = []
    if tier2.clk != known.clk:
        mismatches.append(f"clk: tier2={tier2.clk!r} known={known.clk!r}")
    if tier2.clk_edge != known.clk_edge:
        mismatches.append(f"clk_edge: tier2={tier2.clk_edge!r} known={known.clk_edge!r}")
    if tier2.data != known.data:
        mismatches.append(f"data: tier2={tier2.data!r} known={known.data!r}")
    if tier2.outputs != known.outputs:
        mismatches.append(f"outputs: tier2={tier2.outputs!r} known={known.outputs!r}")
    t2_ov = sorted(override_key(o) for o in tier2.overrides)
    k_ov = sorted(override_key(o) for o in known.overrides)
    if t2_ov != k_ov:
        mismatches.append(f"overrides: tier2={t2_ov!r} known={k_ov!r}")

    if mismatches:
        print(f"TIER2 VALIDATION MISMATCH for {cell_name}:")
        for m in mismatches:
            print(f"  {m}")
    else:
        print(f"TIER2 VALIDATION OK for {cell_name}: matches known-cell table exactly.")


# ---------------------------------------------------------------------------
# Leaf module decompilation
# ---------------------------------------------------------------------------


@dataclass
class LeafModule:
    name: str
    cell_name: str
    input_labels: list
    output_labels: list
    kind: str  # "combinational" | "sequential" | "unsupported"
    comb_exprs: dict = None
    seq_pinmap: object = None
    tier: str = ""


def decompile_leaf(cell, force_tier2=False):
    name = _sanitize_ident(cell.cell_name)
    if len(cell.input_labels) > MAX_COMB_INPUTS:
        raise DecompileError(f"{len(cell.input_labels)} input(s) exceeds MAX_COMB_INPUTS={MAX_COMB_INPUTS}")

    table = _cell_truth_table(cell)
    if table is not None:
        n = len(cell.input_labels)
        exprs = {}
        for oi, out in enumerate(cell.output_labels):
            minterms = [_combo_to_int(combo) for combo, outs in table.items() if outs[oi]]
            exprs[out] = _sop_expr(minterms, n, cell.input_labels)
        return LeafModule(
            name=name, cell_name=cell.cell_name,
            input_labels=cell.input_labels, output_labels=cell.output_labels,
            kind="combinational", comb_exprs=exprs, tier="combinational(z3-truth-table)",
        )

    tier1 = None if force_tier2 else KNOWN_SEQUENTIAL_CELLS.get(cell.cell_name)
    if tier1 is not None:
        pinmap, tier = tier1, "tier1(known-cell)"
    else:
        overrides = _find_overrides_z3(cell)
        pinmap = _characterize_sequential_tier2(cell, overrides)
        tier = "tier2(characterized)"

    if force_tier2 and cell.cell_name in KNOWN_SEQUENTIAL_CELLS:
        _validate_tier2_against_known(cell.cell_name, pinmap, KNOWN_SEQUENTIAL_CELLS[cell.cell_name])

    return LeafModule(
        name=name, cell_name=cell.cell_name,
        input_labels=cell.input_labels, output_labels=cell.output_labels,
        kind="sequential", seq_pinmap=pinmap, tier=tier,
    )


def _stub_leaf(cell, reason):
    return LeafModule(
        name=_sanitize_ident(cell.cell_name), cell_name=cell.cell_name,
        input_labels=cell.input_labels, output_labels=cell.output_labels,
        kind="unsupported", tier=f"unsupported: {reason}",
    )


# ---------------------------------------------------------------------------
# Transparent-buffer collapsing
# ---------------------------------------------------------------------------


def _is_transparent_buffer(leaf):
    """True if `leaf`'s ENTIRE recovered function is `assign X = A;` --
    a single input, a single output, and the output's own minimized SOP
    expression is literally the input pin's name (no inversion, no other
    dependency). This is a structural fact about the boolean function
    decompile_leaf() already derived via z3, not a guess based on the
    cell's name -- any cell type meeting it is electrically a pure
    repeater no matter what it's called or why it's in the design (a
    clock-tree buffer, a routing/drive-strength repeater, ...)."""
    return (
        leaf.kind == "combinational"
        and len(leaf.input_labels) == 1
        and len(leaf.output_labels) == 1
        and leaf.comb_exprs[leaf.output_labels[0]] == leaf.input_labels[0]
    )


def _collapse_transparent_buffers(chip, clusters, leaf_modules):
    """Splice out every instance of a transparent-buffer leaf type,
    aliasing its output net to its input net (transitively, for a
    chained buffer tree) and dropping the instance -- the same "buffer
    tree collapsing" real EDA flows use before comparing a routed,
    post-clock-tree-synthesis netlist back against pre-synthesis RTL
    (see the conversation this was added for: 04_final.gds's own routed
    clkbuf_16 instances have no counterpart at all in 00_source.v,
    because CTS inserts them during physical implementation, well after
    the RTL stage -- purely a fan-out/skew-management detail, logically
    invisible).

    Mutates chip.instances' own global_pins dicts (net aliasing) and
    each Cluster's own instances list (buffer removal) in place --
    deliberate, not an oversight: build_ast() only ever receives a
    Chip/Cluster list built fresh for one decompiler.py run (see
    __main__), never shared or cached state anything else depends on
    staying unmodified.
    """
    alias = {}
    for inst in chip.instances:
        leaf = leaf_modules.get(inst.cell.cell_name)
        if leaf is None or not _is_transparent_buffer(leaf):
            continue
        in_net = inst.global_pins.get(inst.cell.input_labels[0])
        out_net = inst.global_pins.get(inst.cell.output_labels[0])
        if in_net is not None and out_net is not None:
            alias[out_net] = in_net

    def resolve(net):
        seen = set()
        while net in alias and net not in seen:
            seen.add(net)
            net = alias[net]
        return net

    for inst in chip.instances:
        for pin, net in list(inst.global_pins.items()):
            inst.global_pins[pin] = resolve(net)

    def is_buffer_instance(inst):
        leaf = leaf_modules.get(inst.cell.cell_name)
        return leaf is not None and _is_transparent_buffer(leaf)

    n_collapsed = sum(1 for inst in chip.instances if is_buffer_instance(inst))
    chip.instances[:] = [inst for inst in chip.instances if not is_buffer_instance(inst)]
    for c in clusters:
        c.instances[:] = [inst for inst in c.instances if not is_buffer_instance(inst)]

    if n_collapsed:
        print(f"collapsed {n_collapsed} transparent-buffer instance(s)")


# ---------------------------------------------------------------------------
# AST assembly
# ---------------------------------------------------------------------------


@dataclass
class InstanceCall:
    module_name: str
    inst_name: str
    # local port name -> connected net identifier: a plain str for a
    # scalar port, or a list of net identifiers (LSB-first) for a bus
    # port -- see _emit_conn().
    port_map: dict


@dataclass
class PortDecl:
    name: str
    width: int = 1  # >1 renders as a Verilog vector port [width-1:0]


@dataclass
class ClusterModule:
    name: str
    input_ports: list  # list of PortDecl
    output_ports: list  # list of PortDecl
    internal_wires: list  # list of PortDecl (all width 1 -- no cluster-internal bus grouping)
    instances: list


@dataclass
class ChipModule:
    name: str
    input_ports: list  # list of PortDecl (all width 1 -- see module docstring)
    output_ports: list  # list of PortDecl
    wires: list  # list of PortDecl -- see _group_top_wires()
    instances: list


# ---------------------------------------------------------------------------
# Cluster-type deduplication (networkx graph isomorphism)
# ---------------------------------------------------------------------------


@dataclass
class _RawCluster:
    """One cluster's own scalar-net-level structure, before type
    deduplication or bus grouping -- exactly what the first version of
    this file emitted directly as one ClusterModule per Cluster."""

    cluster_id: int
    input_ports: list  # net identifiers, fixed order
    output_ports: list
    internal_wires: list
    instances: list  # InstanceCall, module_name = LeafModule.name


def _cluster_graph(rc):
    """networkx.Graph for one _RawCluster: one node per instance (colored
    by its leaf module name) and one node per net (colored by boundary
    direction), edges labeled by pin name -- see the module docstring for
    why this coloring is what makes an isomorphism check meaningful."""
    g = nx.Graph()
    for net in rc.input_ports:
        g.add_node(("net", net), kind="port_in")
    for net in rc.output_ports:
        g.add_node(("net", net), kind="port_out")
    for net in rc.internal_wires:
        g.add_node(("net", net), kind="net")
    for i, inst in enumerate(rc.instances):
        inode = ("inst", i)
        g.add_node(inode, kind=f"inst:{inst.module_name}")
        for pin, net in inst.port_map.items():
            g.add_edge(inode, ("net", net), pin=pin)
    return g


def _node_match(a, b):
    return a["kind"] == b["kind"]


def _edge_match(a, b):
    return a["pin"] == b["pin"]


def _dedupe_cluster_types(raw_clusters):
    """Group _RawClusters into shared types by graph isomorphism.

    Returns a list of dicts, each {"canonical": _RawCluster,
    "members": [(_RawCluster, {node: node}), ...]} -- `members` includes
    the canonical cluster itself (with an identity mapping) plus every
    other cluster confirmed isomorphic to it, each mapped to the
    CANONICAL cluster's own net/instance nodes (see the module docstring
    for how that mapping is used to connect each instance correctly).
    """
    buckets = defaultdict(list)
    for rc in raw_clusters:
        counts = tuple(sorted(Counter(inst.module_name for inst in rc.instances).items()))
        buckets[(counts, len(rc.input_ports), len(rc.output_ports))].append(rc)

    types = []
    for group in buckets.values():
        for rc in group:
            g = _cluster_graph(rc)
            wl = nx.weisfeiler_lehman_graph_hash(g, edge_attr="pin", node_attr="kind", iterations=3)

            found = None
            for t in types:
                if t["wl"] != wl:
                    continue
                gm = nx.algorithms.isomorphism.GraphMatcher(t["graph"], g, node_match=_node_match, edge_match=_edge_match)
                if gm.is_isomorphic():
                    found = (t, gm.mapping)
                    break

            if found is None:
                types.append({"wl": wl, "graph": g, "canonical": rc, "members": [(rc, {n: n for n in g.nodes})]})
            else:
                t, mapping = found
                t["members"].append((rc, mapping))
    return types


# ---------------------------------------------------------------------------
# Bus/vector port grouping
# ---------------------------------------------------------------------------


def _net_endpoints(raw_clusters):
    """driver_of: net -> the one cluster_id that drives it (if any);
    consumers_of: net -> set of cluster_id that consume it. Built from
    _RawCluster.input_ports/output_ports specifically (not directly from
    cluster_io_nets()) so the keys are the same already-renamed Verilog
    net identifiers _group_ports_into_buses() looks them up by -- mixing
    raw chip-wide net names with renamed ones here would silently miss
    almost every lookup."""
    driver_of = {}
    consumers_of = defaultdict(set)
    for rc in raw_clusters:
        for n in rc.output_ports:
            driver_of[n] = rc.cluster_id
        for n in rc.input_ports:
            consumers_of[n].add(rc.cluster_id)
    return driver_of, consumers_of


def _group_ports_into_buses(canonical_rc, driver_of, consumers_of, preferred_order=None):
    """Partition a canonical cluster's own input_ports/output_ports (flat
    net-identifier lists) into PortDecl groups: several ports collapse
    into one vector PortDecl when they all connect to the exact same
    single OTHER cluster (see module docstring for why "exactly one other
    cluster" is the deliberately conservative grouping condition).

    Args:
        preferred_order: optional {net: bit_index}, already-fixed bit
            positions for nets whose bus bit order was forced by some
            OTHER (constrained, i.e. multi-instance) cluster type on the
            far end of this exact connection -- see build_ast's
            constrained-types-first / free-types-second ordering and the
            module docstring's "propagation" section. When every net in
            a candidate bus group has an entry here, that order is used
            instead of the default alphabetical-by-net-name sort, so a
            free (single-instance) type's own bus port agrees bit-for-
            bit with whichever type actually has no freedom to choose
            otherwise -- turning what would otherwise be a bit-select
            concatenation at every connection back into a plain,
            unscrambled name-to-name one.

    Returns (input_decls, output_decls, positions) where positions maps
    each PortDecl.name -> the list of indices into the ORIGINAL flat
    input_ports/output_ports list it covers, in bus bit order (index 0 =
    bit 0) -- used by _instantiate_ports() to pull the matching nets out
    of any other member's own (differently-ordered-by-net-identity, but
    positionally-corresponding) flat list.
    """
    preferred_order = preferred_order or {}

    def group(nets, key_fn):
        groups = defaultdict(list)
        order = []
        for i, n in enumerate(nets):
            key = key_fn(n)
            if key not in groups:
                order.append(key)
            groups[key].append(i)
        decls, positions = [], {}
        for key in order:
            idxs = groups[key]
            if len(idxs) == 1:
                name = nets[idxs[0]]
                decls.append(PortDecl(name=name, width=1))
            else:
                if all(nets[i] in preferred_order for i in idxs):
                    idxs = sorted(idxs, key=lambda i: preferred_order[nets[i]])
                else:
                    idxs = sorted(idxs, key=lambda i: nets[i])
                name = f"{nets[idxs[0]]}_bus"
                decls.append(PortDecl(name=name, width=len(idxs)))
            positions[decls[-1].name] = idxs
        return decls, positions

    # a net with no single driver cluster (a true chip primary input) must
    # never merge with another such net just because both key to "None" --
    # key by net identity itself (a group of exactly 1) in that case.
    in_decls, in_pos = group(
        canonical_rc.input_ports,
        lambda n: driver_of[n] if n in driver_of else ("_unique", n),
    )
    out_decls, out_pos = group(
        canonical_rc.output_ports,
        lambda n: next(iter(consumers_of[n])) if len(consumers_of.get(n, ())) == 1 else ("_unique", n),
    )
    return in_decls, out_decls, {**in_pos, **out_pos}


def _bit_select_aliases(decls, positions, flat_ports):
    """net identifier -> Verilog bit-select expression (e.g. "A0_bus[3]")
    for every net a bus PortDecl absorbed. Necessary, not cosmetic: the
    canonical cluster's own internal gate instances still reference these
    original per-bit net names directly in their own port_map (bus
    grouping only ever touches the cluster's own EXTERNAL port
    declarations -- see _group_ports_into_buses), and once such a net is
    no longer its own port or wire, that raw name is an undeclared
    identifier inside the module. A scalar (width-1) PortDecl needs no
    aliasing: _group_ports_into_buses already names it after the net
    itself, so the port declaration IS already usable under that name."""
    aliases = {}
    for decl in decls:
        if decl.width == 1:
            continue
        for bit, i in enumerate(positions[decl.name]):
            aliases[flat_ports[i]] = f"{decl.name}[{bit}]"
    return aliases


def _instantiate_ports(decls, positions, flat_ports):
    """Build one InstanceCall.port_map fragment {port_decl_name: net or
    [nets]} for a specific member's own flat net list, using bus/scalar
    groupings computed from the canonical member (see
    _group_ports_into_buses) but applied by POSITION to `flat_ports` --
    always electrically correct for this member even though the grouping
    decision itself came from a different (canonical) instance."""
    port_map = {}
    for decl in decls:
        idxs = positions[decl.name]
        nets = [flat_ports[i] for i in idxs]
        port_map[decl.name] = nets[0] if decl.width == 1 else nets
    return port_map


def _group_top_wires(top):
    """Collapse top-level scalar wire declarations into one Verilog
    vector wire wherever a group of them only ever appears together as
    one bus connection -- the top-level counterpart to
    _group_ports_into_buses(), reusing the exact same net groupings
    (already computed, one per multi-net list-valued InstanceCall.port_map
    entry -- see _instantiate_ports) instead of recomputing anything.

    Groups by the SET of nets, deliberately NOT by the exact ordered
    tuple: two connections to the very same physical bus are NOT
    guaranteed to list their nets in the same order, because each
    cluster TYPE's own bus bit order is fixed independently by ITS OWN
    canonical member (see _group_ports_into_buses) -- there's no reason
    two unrelated types' matching ports happen to agree bit-for-bit just
    because they carry the same physical nets. Grouping by exact tuple
    equality would silently treat two differently-ordered connections to
    the SAME bus as two unrelated ones -- two disconnected wires instead
    of one -- a real bug an earlier version of this function had.

    The resulting vector wire's own bit order is taken from whichever
    connection reaches this bus FIRST (not independently re-derived, e.g.
    by sorting net names) -- so when build_ast's own bit-order
    propagation already made every connection to this bus agree with
    each other bit-for-bit, the wire agrees with them too, and every
    connection collapses to a bare name. Only a connection whose own
    order genuinely couldn't be reconciled (see build_ast's own
    docstring on the one case propagation can't resolve: two constrained
    types wired directly to each other) falls back to a concatenation of
    bit-selects into the shared wire -- still 100% correct, just not
    simplified to a bare name.
    """
    groups = {}     # frozenset(nets) -> PortDecl
    bit_index = {}  # frozenset(nets) -> {net: canonical bit index}
    for inst in top.instances:
        for net in inst.port_map.values():
            if not isinstance(net, list):
                continue
            key = frozenset(net)
            if key not in groups:
                # the wire's own bit order is whichever order this FIRST
                # connection to it already needs -- not independently
                # re-sorted alphabetically -- so that when build_ast's
                # bit-order propagation already made every connection to
                # this bus agree with each other, the wire agrees with
                # them too, instead of introducing a fresh mismatch of
                # its own right at the last step.
                groups[key] = PortDecl(name=f"{sorted(key)[0]}_bus", width=len(net))
                bit_index[key] = {n: i for i, n in enumerate(net)}

    grouped_nets = {n for key in groups for n in key}
    scalar_wires = [PortDecl(name=w, width=1) for w in top.wires if w not in grouped_nets]
    top.wires = scalar_wires + list(groups.values())

    for inst in top.instances:
        for pin, net in list(inst.port_map.items()):
            if not isinstance(net, list):
                continue
            key = frozenset(net)
            decl, idx = groups[key], bit_index[key]
            if [idx[n] for n in net] == list(range(len(net))):
                inst.port_map[pin] = decl.name
            else:
                inst.port_map[pin] = [f"{decl.name}[{idx[n]}]" for n in net]


# ---------------------------------------------------------------------------
# AST assembly
# ---------------------------------------------------------------------------


def build_ast(chip, clusters, force_tier2=False):
    leaf_modules = {}

    def get_leaf(cell):
        if cell.cell_name not in leaf_modules:
            try:
                leaf_modules[cell.cell_name] = decompile_leaf(cell, force_tier2=force_tier2)
            except DecompileError as e:
                print(f"warning: could not decompile {cell.cell_name}: {e}")
                leaf_modules[cell.cell_name] = _stub_leaf(cell, str(e))
        return leaf_modules[cell.cell_name]

    # every distinct leaf type has to be decompiled BEFORE buffer
    # collapsing, since collapsing needs to know which types are pure
    # single-input identity buffers (_is_transparent_buffer).
    for inst in chip.instances:
        get_leaf(inst.cell)
    _collapse_transparent_buffers(chip, clusters, leaf_modules)

    # cluster_io_nets() has to be recomputed AFTER collapsing -- it reads
    # chip.instances/Cluster.instances directly, both just mutated above.
    cluster_io = {c.id: cluster_io_nets(chip, c) for c in clusters}
    boundary_nets = set()
    for ins, outs in cluster_io.values():
        boundary_nets.update(ins)
        boundary_nets.update(outs)

    namer = _NetNamer(chip, boundary_nets)

    raw_clusters = []
    for c in clusters:
        ins, outs = cluster_io[c.id]
        local = namer.scope()
        boundary = set(ins) | set(outs)
        instances = []
        internal = set()
        for i, inst in enumerate(c.instances):
            leaf = get_leaf(inst.cell)
            port_map = {}
            for pin in inst.cell.input_labels + inst.cell.output_labels:
                net = inst.global_pins.get(pin)
                if net is None:
                    continue
                port_map[pin] = local(net)
                if net not in boundary:
                    internal.add(net)
            instances.append(InstanceCall(leaf.name, f"{leaf.name}_i{i}", port_map))
        if not instances:
            # every instance in this cluster was a collapsed buffer (e.g.
            # a lone leftover clock-buffer "cluster" from
            # _merge_small_clusters) -- nothing left to emit or instantiate.
            continue
        raw_clusters.append(_RawCluster(
            cluster_id=c.id,
            input_ports=[local(n) for n in ins],
            output_ports=[local(n) for n in outs],
            internal_wires=sorted({local(n) for n in internal}),
            instances=instances,
        ))

    driver_of, consumers_of = _net_endpoints(raw_clusters)

    types = _dedupe_cluster_types(raw_clusters)
    cluster_modules = []
    member_port_maps = {}  # cluster_id -> (type_name, {port_decl_name: net or [nets]})

    # bit-order PROPAGATION: a type with more than one member (a real
    # dedup, see _dedupe_cluster_types) has NO freedom in its own bus bit
    # order -- it's forced once, by its canonical member's own net names,
    # and every other member has to live with whatever that forces onto
    # its own nets (see the module docstring's isomorphism section). A
    # type with exactly one member has no such constraint, so instead of
    # independently inventing its own alphabetical order (which is what
    # caused the scrambled bit-select concatenations a design with a
    # shared type could produce), it can just ADOPT whatever order a
    # constrained neighbor already forced onto the same physical nets.
    # Processing constrained types first, and recording every net's
    # realized bit position as we go, is what makes that adoption
    # possible -- free types processed afterward simply look it up.
    preferred_order = {}  # net -> bit index already forced by some other type
    ordered_types = sorted(types, key=lambda t: len(t["members"]) == 1)  # constrained (>1 member) first

    for type_index, t in enumerate(ordered_types):
        canonical_rc = t["canonical"]
        type_name = f"cluster_type_{type_index}"
        in_decls, out_decls, positions = _group_ports_into_buses(
            canonical_rc, driver_of, consumers_of, preferred_order=preferred_order,
        )

        aliases = _bit_select_aliases(in_decls, positions, canonical_rc.input_ports)
        aliases.update(_bit_select_aliases(out_decls, positions, canonical_rc.output_ports))
        body_instances = [
            InstanceCall(inst.module_name, inst.inst_name,
                          {pin: aliases.get(net, net) for pin, net in inst.port_map.items()})
            for inst in canonical_rc.instances
        ]

        cluster_modules.append(ClusterModule(
            name=type_name,
            input_ports=in_decls, output_ports=out_decls,
            internal_wires=[PortDecl(name=w, width=1) for w in canonical_rc.internal_wires],
            instances=body_instances,
        ))

        for rc, mapping in t["members"]:
            member_in = [mapping[("net", n)][1] for n in canonical_rc.input_ports]
            member_out = [mapping[("net", n)][1] for n in canonical_rc.output_ports]
            port_map = _instantiate_ports(in_decls, positions, member_in)
            port_map.update(_instantiate_ports(out_decls, positions, member_out))
            member_port_maps[rc.cluster_id] = (type_name, port_map)

            # publish this member's own realized bit order so a free type
            # touching these same nets on their other end can adopt it
            # too (setdefault: never override a bit position a
            # constrained type already forced).
            for decl in in_decls + out_decls:
                if decl.width > 1:
                    for bit, net in enumerate(port_map[decl.name]):
                        preferred_order.setdefault(net, bit)

    top_namer = namer.scope()
    driven, consumed = set(), set()
    for ins, outs in cluster_io.values():
        driven.update(outs)
        consumed.update(ins)

    top_inputs, top_outputs, top_wires = [], [], []
    for net in sorted(driven | consumed):
        ident = top_namer(net)
        if net in driven and net in consumed:
            top_wires.append(ident)
        elif net in driven:
            top_outputs.append(PortDecl(name=ident))
        else:
            top_inputs.append(PortDecl(name=ident))

    top_instances = []
    for c in clusters:
        if c.id not in member_port_maps:
            continue  # emptied out entirely by buffer collapsing
        type_name, port_map = member_port_maps[c.id]
        top_instances.append(InstanceCall(type_name, f"{type_name}_i{c.id}", port_map))

    top = ChipModule(
        name=_sanitize_ident(chip.top_cell.name),
        input_ports=top_inputs, output_ports=top_outputs, wires=top_wires,
        instances=top_instances,
    )
    _group_top_wires(top)
    return leaf_modules, cluster_modules, top


# ---------------------------------------------------------------------------
# Verilog emission
# ---------------------------------------------------------------------------


def _emit_leaf_module(leaf):
    out_kw = "output reg" if leaf.kind == "sequential" else "output"
    ports = [f"    input {p}," for p in leaf.input_labels]
    ports += [f"    {out_kw} {p}," for p in leaf.output_labels]
    if ports:
        ports[-1] = ports[-1].rstrip(",")

    lines = [f"module {leaf.name} (", *ports, ");"]
    lines.append(f"    // {leaf.tier}")

    if leaf.kind == "combinational":
        for out, expr in leaf.comb_exprs.items():
            lines.append(f"    assign {out} = {expr};")
    elif leaf.kind == "sequential":
        pm = leaf.seq_pinmap
        edge_kw = "posedge" if pm.clk_edge == "pos" else "negedge"
        sens = [f"{edge_kw} {pm.clk}"]
        async_overrides = [o for o in pm.overrides if o.is_async]
        for o in async_overrides:
            sens.append(f"{'posedge' if o.active_value == 1 else 'negedge'} {o.pin}")

        lines.append(f"    always @({' or '.join(sens)}) begin")
        for i, o in enumerate(async_overrides):
            cond = o.pin if o.active_value == 1 else f"!{o.pin}"
            lines.append(f"        {'if' if i == 0 else 'else if'} ({cond}) begin")
            for out_name in leaf.output_labels:
                if out_name in o.forced:
                    lines.append(f"            {out_name} <= 1'b{o.forced[out_name]};")
            lines.append("        end")
        lines.append(f"        {'else ' if async_overrides else ''}begin")
        for out_name in leaf.output_labels:
            rhs = pm.data if pm.outputs.get(out_name, "normal") == "normal" else f"~{pm.data}"
            lines.append(f"            {out_name} <= {rhs};")
        lines.append("        end")
        lines.append("    end")
    else:
        for out in leaf.output_labels:
            lines.append(f"    assign {out} = 1'bx;")

    lines.append("endmodule")
    return "\n".join(lines)


def _port_decl_str(direction, decl):
    width = "" if decl.width == 1 else f"[{decl.width - 1}:0] "
    return f"    {direction} {width}{decl.name},"


def _conn_str(net):
    """A scalar net identifier connects directly; a bus (list of net
    identifiers, index 0 = bit 0) connects via Verilog concatenation,
    MSB-first as {...} syntax requires -- only still needed where the far
    end isn't a matching vector wire (see _group_top_wires(), which
    replaces this with a direct name reference wherever it applies)."""
    if isinstance(net, str):
        return net
    return "{" + ", ".join(reversed(net)) + "}"


def _wire_decl_str(decl):
    width = "" if decl.width == 1 else f"[{decl.width - 1}:0] "
    return f"    wire {width}{decl.name};"


def _emit_structural_module(name, input_ports, output_ports, wires, instances):
    ports = [_port_decl_str("input", p) for p in input_ports] + [_port_decl_str("output", p) for p in output_ports]
    if ports:
        ports[-1] = ports[-1].rstrip(",")
    lines = [f"module {name} (", *ports, ");"]
    for w in wires:
        lines.append(_wire_decl_str(w))
    for inst in instances:
        conns = ", ".join(f".{pin}({_conn_str(net)})" for pin, net in inst.port_map.items())
        lines.append(f"    {inst.module_name} {inst.inst_name} ({conns});")
    lines.append("endmodule")
    return "\n".join(lines)


def render_verilog(leaf_modules, cluster_modules, top):
    # a leaf type with zero surviving instantiations (e.g. a transparent
    # buffer collapsed out of every cluster body -- see
    # _collapse_transparent_buffers) has nothing left to instantiate it,
    # so its own module definition would just be dead code in the file.
    used = {inst.module_name for cm in cluster_modules for inst in cm.instances}

    parts = [f"// Auto-generated by decompiler.py from {top.name} -- do not hand-edit."]
    for name in sorted(leaf_modules):
        if leaf_modules[name].name in used:
            parts.append(_emit_leaf_module(leaf_modules[name]))
    for cm in cluster_modules:
        parts.append(_emit_structural_module(cm.name, cm.input_ports, cm.output_ports, cm.internal_wires, cm.instances))
    parts.append(_emit_structural_module(top.name, top.input_ports, top.output_ports, top.wires, top.instances))
    return "\n\n".join(parts)


if __name__ == "__main__":
    import sys

    from chip import Chip

    # python decompiler.py [gds_file] [top_cell_name] [epsilon_scale] [--force-tier2]
    args = sys.argv[1:]
    force_tier2 = "--force-tier2" in args
    args = [a for a in args if a != "--force-tier2"]

    gds_file = args[0] if len(args) > 0 else "./warmup/04_final.gds"
    top_cell_name = args[1] if len(args) > 1 else "adder_demo"
    epsilon_scale = float(args[2]) if len(args) > 2 else 1.0

    chip = Chip(gds_file, top_cell_name)
    clusters = cluster_chip(chip, epsilon_scale=epsilon_scale)
    print(f"{top_cell_name}: {len(chip.instances)} instance(s), {len(clusters)} cluster(s)")
    if force_tier2:
        print("(--force-tier2: sequential cells will be characterized structurally, "
              "known-cell table used only to validate the result)")

    leaf_modules, cluster_modules, top = build_ast(chip, clusters, force_tier2=force_tier2)

    print(f"\n{len(leaf_modules)} leaf module(s):")
    for name in sorted(leaf_modules):
        leaf = leaf_modules[name]
        print(f"  {name}: {leaf.kind} ({leaf.tier})")

    print(f"\n{len(cluster_modules)} cluster module type(s) recovered from {len(clusters)} cluster(s):")
    for cm in cluster_modules:
        n_chip_insts = sum(1 for i in top.instances if i.module_name == cm.name)
        ports = ", ".join(f"{p.name}[{p.width}]" if p.width > 1 else p.name for p in cm.input_ports + cm.output_ports)
        print(f"  {cm.name}: {len(cm.instances)} gate(s), {n_chip_insts} chip instantiation(s), ports: {ports}")

    out_path = f"{top_cell_name}.v"
    verilog = render_verilog(leaf_modules, cluster_modules, top)
    with open(out_path, "w") as f:
        f.write(verilog + "\n")
    print(f"\nwrote {out_path}")
