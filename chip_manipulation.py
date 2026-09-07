"""
Chip manipulation: build a decompiled AST from a chip.Chip + clustering.py's
recovered clusters, and apply a series of AST -> AST simplification passes
to it. This is the "what does the design compute" half of the last pipeline
stage (geometry -> transistors -> cells -> chip netlist -> clusters -> RTL);
decompiler.py is the other half -- it takes the AST this module builds and
turns it into actual Verilog text (decompile()/render_verilog()), and knows
nothing about how the AST itself got built or simplified.

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
    is normally *purely structural* -- wires plus instantiations of
    LeafModules, built directly from cluster_io_nets()/Instance.global_pins
    (already correct, geometrically-resolved connectivity; no new net
    analysis here). A 16-cell shift register cluster is not characterized
    as one black box; it's 8 dfrtp_2 instances + 8 mux2_1 instances +
    wires, each instance's own behavior coming from its LeafModule. This
    is what keeps sequential decompilation tractable: only ONE flip-flop's
    pin behavior needs characterizing, not a whole register file's.

    The one exception: every net that a cluster's own combinational gates
    compute -- whether that's the WHOLE cluster (a purely combinational
    one, comparator496-style) or just the glue feeding a register's D
    pin inside an otherwise-sequential cluster (a shift register's own
    per-bit mux) -- gets replaced by an `assign` instead of a sky130 gate
    instantiation, one of two ways: a whole boundary-level expression
    where that's cheap enough to derive exactly (z3 truth-table
    enumeration, "Cluster-level combinational flattening" below), or
    failing that, one small per-instance expression at a time (plain
    textual substitution, no derivation at all, "Per-instance
    combinational inlining" below) -- see those two sections for why
    BOTH exist rather than just pushing the first one harder. Such a
    ClusterModule has `comb_exprs` set; `instances` then holds only
    whichever SEQUENTIAL instances survive, plus any "unsupported" leaf
    (decompile_leaf() genuinely couldn't determine its function) --
    empty whenever the cluster has neither.
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
gate. The same _minimize_sop/_sop_expr machinery is reused one level up,
at cluster granularity, by the combinational-flattening pass below.

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

FORCE_TIER2 in decompiler.py's __main__ (--force-tier2) exists so Tier 2
can be run and checked against Tier 1's known-correct answer on the ONE
cell this project has ground truth for (dfrtp_2, via warmup/04_final.gds)
-- _validate_tier2_against_known() diffs the two structurally and prints a
PASS/mismatch report. That's the validation loop before trusting Tier 2
as the ONLY path on a design (like puzzle.gds) with no known-cell table
entry to fall back on at all.

Passes applied by build_ast(), in order
----------------------------------------

1. Transparent-buffer collapsing (_collapse_transparent_buffers)
2. Cluster-type deduplication (_dedupe_cluster_types, networkx graph
   isomorphism)
3. Per-type bus/vector port grouping (_group_ports_into_buses)
4. Per-type combinational flattening (_flatten_cluster_combinational, z3)
5. Per-instance combinational inlining (_inline_combinational_instances,
   no z3 -- the fallback for whatever Pass 4 couldn't flatten)
6. Top-level bus/vector wire grouping (_group_top_wires)

### 1. Transparent-buffer collapsing

Splices out every instance of a pure single-input/single-output identity
leaf type (a clock-tree buffer, a routing/drive-strength repeater --
detected structurally from its own already-decompiled truth table, never
by name), aliasing its output net to its input net and dropping the
instance -- the same "buffer tree collapsing" real EDA flows use before
comparing a routed, post-clock-tree-synthesis netlist back against
pre-synthesis RTL.

### 2. Cluster-type deduplication (networkx graph isomorphism)

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

### 3. Bus/vector port grouping (net-endpoint analysis, no external library)

Groups several of a cluster TYPE's own scalar boundary ports into one
Verilog vector port when they share the exact same *other* cluster on
their far end (e.g. a shift-register cluster's 8 separate Q outputs,
which -- after dedup -- would otherwise all be declared as 8 separate
scalar ports even though every one of them feeds the SAME adder cluster).
The grouping key is purely net-identity-based (chip.py's own union-find,
via which cluster's cluster_io_nets() lists a given net as an input/
output), never a per-design label -- so this is just as legitimate on
puzzle.gds as it is here.

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

### 4. Cluster-level combinational flattening (z3)

This is the pass that stops a cluster's own recovered combinational logic
from being rendered as sky130 gate instances wired together, per the
project goal of not leaving ALL combinational logic built directly on top
of standard-cell building blocks -- including the combinational glue that
sits INSIDE an otherwise-sequential cluster (a shift register's own
per-bit mux2_1 feeding each dfrtp_2's D pin is exactly as much "logic
built on sky130 building blocks" as a standalone comparator cluster is;
there's no structural reason to only simplify the former).

_flatten_cluster_combinational() first partitions a cluster TYPE's own
canonical instances (paired positionally with the underlying
clustering.Cluster's own chip.Instance list, the same correspondence
build_ast() constructs them with) into two groups, using nothing more
than each instance's own already-decompiled LeafModule.kind:

  - KEPT: every instance whose leaf is "sequential" -- always stays
    instantiated exactly as before, unchanged. Flip-flops/latches are
    never flattened; switch-level z3 has no notion of history (see
    clustering.py's own extensive account of why that fails), so their
    own behavior only ever lives in a LeafModule's `always @(posedge...)`
    block, never in a flattened `assign`.
  - REMOVABLE: everything else -- ordinary combinational leaves, AND
    anything decompile_leaf() couldn't classify ("unsupported", e.g. a
    single gate that individually exceeded MAX_COMB_INPUTS). This is
    safe because the z3 model used here is built directly from
    transistor.py-level switch physics (build_cluster_z3_circuit), never
    from decompile_leaf()'s own per-cell truth table -- a removable
    instance's own individual decompile failure is irrelevant to whether
    the cluster-level flatten succeeds, and if an "unsupported" leaf
    actually turns out to hide genuine internal state (a decompile_leaf()
    bug, not the expected MAX_COMB_INPUTS case), the same can-be-high-
    AND-low ambiguity check Cell.simulate_z3()/_cluster_is_sequential()
    already rely on catches it below -- defense in depth, never a silent
    wrong answer.

Two kinds of net then become flattening TARGETS: this cluster's own
external OUTPUT nets, and every KEPT instance's own input pin nets (D,
CLK, RESET_B, ... -- treated uniformly; a flattened clock-gating
expression driving a KEPT flip-flop's own CLK port is exactly how real
synthesized RTL already represents clock gating) -- but only the ones
actually DRIVEN, within this cluster, by a REMOVABLE instance. A net
that's already wired directly from a free input or a KEPT instance's own
Q pin, with no intervening logic, needs no flattening at all and is left
as a plain wire connection, exactly as before.

For each target, _relevant_free_nets() does a backward reachability walk
(_cluster_instance_dependency: every removable instance's own output net
depends on ALL of its own input pin nets -- a coarse, safe over-
approximation of true boolean dependency, cheap and never wrong) to find
the MINIMAL set of free variables that target could possibly depend on --
this cluster's own external inputs, or a KEPT instance's own output pin,
wherever reachable through the removable sub-network. This is scoped PER
TARGET, not once for the whole cluster: a shift register's D-pin logic
that only actually depends on the previous stage's Q and a couple of
control signals doesn't pay for every other bit in a wide cluster, and a
whole cluster's own worth of gates still fits under MAX_COMB_INPUTS even
when the cluster's TOTAL external-input-plus-register-state count would
not.

Whether a target's own relevant-variable count fits MAX_COMB_INPUTS (the
same cap decompile_leaf() already uses one level down, same reason: this
is truth-table enumeration, 2**n z3 solves) is checked PER TARGET, not
once for the whole cluster type -- this is the actual reason a wide
combinational block (e.g. an 8-bit ripple-carry adder) still gets MOST of
its own gates flattened even though it, as a whole, would never fit: each
LOW-order sum bit's own carry chain is short (a handful of relevant
variables), while only the HIGH-order bits' full carry-chain fan-in
actually needs the whole input width. An earlier version of this pass
checked eligibility once for the entire cluster type and bailed
completely the moment ANY single target was too wide -- throwing away
every one of the small, easy wins along with the one genuinely oversized
one. Splitting targets into ELIGIBLE (fits the cap) and INELIGIBLE (too
wide, or a dead end) up front fixes that.

An INELIGIBLE target's own value still has to be computed by real gates,
which means every instance ANYWHERE in its own backward fan-in
(_backward_driven_nets -- the same walk _relevant_free_nets() does, but
collecting every DRIVEN net passed through instead of stopping at the
free-variable leaves) has to stay instantiated, in full, no matter how
small it individually looks. That in turn can force a demotion: an
otherwise-ELIGIBLE target whose own direct-driving instance turns out to
be on the SAME fan-in (because it also happens to feed the ineligible
target, directly or several hops upstream) can't be flattened either --
its driving instance has to stay anyway, and adding a redundant `assign`
for a net a real gate is already driving would just be a second,
conflicting driver for the same wire. Only after this demotion pass does
the actual flatten target list get fixed.

For exactly that surviving list, the pass builds ONE combined
switch-level z3 model of EVERY removable instance -- not just the ones
actually being removed (clustering.build_cluster_z3_circuit, via the tiny
_InstanceSubset stand-in it only ever reads `.instances` off of); a
target being flattened can have a dependency chain that runs straight
through an instance staying instantiated for some OTHER (ineligible)
target's sake, and that instance's own transistors are still needed to
correctly derive the flattened target's value, even though it separately
also survives in the rendered structural body -- and, for each target
independently, enumerates its own relevant free variables and
reads off its value the same can-be-high/can-be-low way Cell.simulate_z3()
does for one leaf cell. If ANY combination shows both, that instance
wasn't actually purely combinational after all -- an "unsupported" leaf
hiding real internal state, see above -- and THIS ONE target is dropped
(its own driving instance stays instantiated instead), never the whole
cluster type; every other target already computed, or still to be
computed, is unaffected, since the ambiguity is a property of that one
target's own dependency chain, not the cluster as a whole. Each
successful truth table gets minimized with the identical Quine-McCluskey
pass leaf cells use (_minimize_sop/_sop_expr).

The net effect for a purely combinational cluster (no KEPT instances, and
every target small enough) is the same full-cluster flattening as before
-- every external output is a target, nothing is left instantiated. For a
MIXED cluster, the flip-flops/latches always stay instantiated, and
whichever OTHER instances turned out to be both eligible and not needed
by anything ineligible disappear, replaced by their own `assign`s -- a
wide adder cluster ends up with its low-order sum bits flattened and its
high-order carry chain (plus whatever low-order gates that chain also
happens to reuse) left fully structural, in the SAME rendered module. If
NOTHING in a cluster is removable at all (fully sequential, e.g. a lone
register with no surrounding logic), or nothing ends up both eligible and
safe, there's nothing to flatten and the pass is a no-op for that type.

Runs from the canonical member only (once per TYPE, like bus grouping),
using the aliased local net names bus grouping already computed so a
flattened expression -- and a KEPT instance's own remaining port
connections -- correctly reads e.g. "A0_bus[3]" instead of a raw internal
net name that no longer has its own wire declaration once it's been
absorbed into a bus port. This pass itself doesn't work out which
internal wire declarations are still needed afterward -- build_ast()
does that once, uniformly, AFTER Pass 5 (below) has also had a chance to
remove more instances, since Pass 5's own removals can orphan even more
internal nets than this pass alone would ever see.

### 5. Per-instance combinational inlining (no z3)

Pass 4 is the good outcome: a whole boundary target collapses into ONE
clean expression, and every net it used to route through -- gates AND
their own intermediate wires alike -- disappears entirely. But it's
fundamentally bounded (MAX_COMB_INPUTS, brute-force truth-table
enumeration), and for something genuinely wide -- an 8-bit ripple-carry
adder's own high-order carry chain, say -- that's not just a
missed-optimization gap, it's the RIGHT thing to bail on: forcing a
carry chain into one flat two-level (sum-of-products) expression doesn't
just risk being slow to compute, the RESULT is worse than what it
replaced -- a flat SOP for that kind of recursive, every-bit-depends-on-
every-earlier-bit function needs exponentially many terms, while the
original multi-level gate chain (or an equivalent short chain of small
equations) stays linear. So Pass 4 is deliberately never pushed harder
to "try to flatten more" -- the fix isn't a bigger cap, it's a
DIFFERENT, cheaper technique for whatever Pass 4 correctly declined.

_inline_combinational_instances() is that technique, and it's
deliberately about as simple as this file gets: decompile_leaf() already
computed one minimized expression per LEAF CELL TYPE (e.g. one 2-3-
variable expression for "what does a mux2_1 compute", shared by every
mux2_1 instance in the design) -- this pass just takes every SURVIVING
combinational instance (whatever Pass 4 didn't already remove) and
restates that same expression in terms of THIS instance's own actually-
connected nets (_substitute_expr: one simultaneous word-boundary regex
substitution, pin name -> connected net, per instance), turning
`sky130_fd_sc_hd__xor2_2 xor2_2_i7 (.A(p1), .B(c0), .X(sum1));` into
`assign sum1 = (p1 & ~c0) | (~p1 & c0);` directly. No z3, no truth-table
enumeration, no size cap -- a leaf cell's own expression already exists,
this is a straight textual rewrite, so it applies uniformly regardless
of how wide the surrounding cluster is.

The result keeps every intermediate wire name Pass 4 would have erased
(a ripple-carry chain's own per-bit carry/sum signals stay named and
visible, one small equation each) instead of forcing everything into a
single expression -- which is exactly the "close to what a human would
write" shape for this kind of function: a short chain of small,
individually-readable equations, not one giant boolean blob and not an
anonymous wall of vendor-cell instantiations either. This is what gets a
wide cluster like an 8-bit adder to have ZERO remaining sky130 gate
instantiations in the rendered Verilog -- some targets collapse into one
clean expression (Pass 4), everything else becomes its own small
`assign` (this pass) -- while a KEPT sequential instance (a flip-flop/
latch) or an "unsupported" leaf (decompile_leaf() genuinely doesn't know
its function) is always left exactly as it was, since there's no
expression to inline for either.

Never fails and never bails -- unlike every other pass here, there's no
eligibility check, no cap, and no "returns None" case, because it isn't
deriving anything new: it's restating an expression that already exists.

### 6. Top-level bus/vector wire grouping

The top-module counterpart to step 3, reusing the exact same net
groupings (one per multi-net list-valued InstanceCall.port_map entry
computed above) instead of recomputing anything -- see _group_top_wires()
for the full reasoning, including the real bug (a chip-level port's own
net silently getting folded into a synthesized bus name) found and fixed
while building it.
"""

from collections import Counter, defaultdict
from dataclasses import dataclass, field
from itertools import permutations, product

import re

import networkx as nx
import z3

from clustering import (
    _all_top_level_port_nets,
    build_cluster_z3_circuit,
    cluster_io_nets,
)

MAX_COMB_INPUTS = 10  # mirrors test_suite.py's own MAX_INPUTS guard

# A target within MAX_COMB_INPUTS free variables can still minimize to a
# large number of OR-ed terms if the function just doesn't factor nicely
# (a handful of mid-complexity adder bits observed at exactly 6-7 free
# variables came out 30-80 terms long) -- clearly LESS readable than
# chip_manipulation._inline_combinational_instances()'s own per-gate
# equation chain would have been for the same logic, even though it's a
# single expression. _flatten_cluster_combinational() only accepts a
# Pass 4 result up to this many top-level terms; anything larger is left
# for Pass 5 instead. See "Cluster-level combinational flattening" in
# the module docstring.
MAX_SOP_TERMS = 6


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
# Pass 1: transparent-buffer collapsing
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
    Chip/Cluster list built fresh for one decompile() run (see
    decompiler.py's own __main__), never shared or cached state anything
    else depends on staying unmodified.
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
# AST node types
# ---------------------------------------------------------------------------


@dataclass
class InstanceCall:
    module_name: str
    inst_name: str
    # local port name -> connected net identifier: a plain str for a
    # scalar port, or a list of net identifiers (LSB-first) for a bus
    # port -- see decompiler._emit_conn().
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
    # {port_or_bus_name: verilog_expr_string}, set only when pass 4
    # (combinational flattening, below) successfully replaced this
    # module's ENTIRE structural body -- `instances` is empty whenever
    # this is set. None for an ordinary structural cluster module.
    comb_exprs: dict = None


@dataclass
class ChipModule:
    name: str
    input_ports: list  # list of PortDecl (all width 1 -- see module docstring)
    output_ports: list  # list of PortDecl
    wires: list  # list of PortDecl -- see _group_top_wires()
    instances: list


# ---------------------------------------------------------------------------
# Pass 2: cluster-type deduplication (networkx graph isomorphism)
# ---------------------------------------------------------------------------


@dataclass
class _RawCluster:
    """One cluster's own scalar-net-level structure, before type
    deduplication, bus grouping, or combinational flattening -- exactly
    what the first version of this file emitted directly as one
    ClusterModule per Cluster."""

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

    # Every multi-member type here is one fewer module DEFINITION than
    # there are physical clusters (build_ast emits one ClusterModule per
    # type, then one instantiation per member -- see its own docstring) --
    # printed here, not just left implicit, because that's exactly the
    # gap between "cluster_type_N" module count in the rendered Verilog
    # and the physical Cluster count plot_clusters()/plot_clusters_
    # optimized() shows: several clusters sharing one shape don't each
    # get their own module.
    for t in types:
        if len(t["members"]) > 1:
            ids = sorted(rc.cluster_id for rc, _ in t["members"])
            print(f"deduplicated {len(ids)} clusters into one shared type: cluster {ids}")

    return types


# ---------------------------------------------------------------------------
# Pass 3: bus/vector port grouping
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


def _group_ports_into_buses(canonical_rc, driver_of, consumers_of, members, preferred_order=None):
    """Partition a canonical cluster's own input_ports/output_ports (flat
    net-identifier lists) into PortDecl groups: several ports collapse
    into one vector PortDecl when they all connect to the exact same
    single OTHER cluster (see module docstring for why "exactly one other
    cluster" is the deliberately conservative grouping condition).

    Args:
        members: every (_RawCluster, mapping) pair for this cluster
            TYPE (see _dedupe_cluster_types), canonical included -- the
            grouping decision below is made from the canonical member's
            own net identities, but MUST hold for every OTHER member's
            own corresponding nets too before it's trusted (see the
            "uniform across members" note below): a two-instance type is
            otherwise free to have its bus/scalar decision, made once
            from whichever member happened to be picked canonical,
            silently disagree with a DIFFERENT member's own actual
            wiring.
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

    def _their_net(canonical_net, rc, mapping):
        return mapping[("net", canonical_net)][1]

    def _uniform_single_consumer(canonical_net):
        """True iff EVERY member's own net at this same structural
        position -- not just the canonical member's -- has exactly one
        consumer CLUSTER. Isomorphic (same internal structure) cluster
        instances can still have completely different EXTERNAL fan-out
        at the very same port position: confirmed on puzzle.gds, a
        two-instance cluster type where the canonical member's own net
        fanned out to three different downstream cluster types (so the
        type's own port correctly stayed scalar) while the OTHER
        member's own corresponding net was a clean single-consumer
        connection -- the single, canonical-only decision got applied to
        BOTH instances regardless, so the second instance's own real
        single-bus driver never got consulted at all, leaving that
        driver's own output permanently unconnected to anything. Without
        this check, a multi-instance type's own grouping decision only
        ever reflects one arbitrarily-chosen member."""
        for rc, mapping in members:
            if len(consumers_of.get(_their_net(canonical_net, rc, mapping), ())) != 1:
                return False
        return True

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
    #
    # Symmetric with the output side's own "exactly one consumer" gate
    # below -- and for the same reason: a net's DRIVER only bus-groups
    # its own output ports when every net in the group has exactly one
    # consumer CLUSTER. If a net actually fans out to more than one
    # consumer cluster (this bus broadcast to two different downstream
    # types, say), the driver correctly leaves it scalar -- but every
    # INDIVIDUAL consumer, looked at on its own, still only ever sees
    # "one driver cluster" and would happily group it into a bus anyway,
    # with no way to know a SIBLING consumer exists. That produced a
    # real, confirmed bug on puzzle.gds: the driver's own scalar ports
    # and the (wrongly) bus-grouped consumer port ended up as two
    # DIFFERENT, disconnected top-level wires after Pass 6 synthesized a
    # new name for the consumer's own bus -- the consumer silently read
    # a permanently-undriven net instead of the real signal (confirmed:
    # a whole message-printing cluster fed from a "byte select" bus that
    # read as constant 0 for exactly this reason). Requiring the SAME
    # "exactly one consumer, globally" condition here as the output side
    # already uses means a net only ever bus-groups on the input side
    # when the driver's own decision is GUARANTEED to agree (grouped the
    # same way, or scalar on both sides) -- never a mismatch. And that
    # condition is checked via _uniform_single_consumer -- across every
    # member of THIS type, not just the canonical one -- for the same
    # cross-member reason documented on that helper above.
    in_decls, in_pos = group(
        canonical_rc.input_ports,
        lambda n: driver_of[n] if n in driver_of and _uniform_single_consumer(n) else ("_unique", n),
    )
    out_decls, out_pos = group(
        canonical_rc.output_ports,
        lambda n: next(iter(consumers_of[n])) if _uniform_single_consumer(n) else ("_unique", n),
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


# ---------------------------------------------------------------------------
# Pass 4: cluster-level combinational flattening (z3)
# ---------------------------------------------------------------------------


class _InstanceSubset:
    """Minimal stand-in for a clustering.Cluster -- just enough shape (an
    `.instances` attribute) for build_cluster_z3_circuit(), which only
    ever reads that one attribute off whatever it's given. Used to build
    a z3 model over a SUBSET of one cluster's own instances (its
    non-sequential, "removable" ones -- see _flatten_cluster_
    combinational) without needing a real Cluster for it."""

    __slots__ = ("instances",)

    def __init__(self, instances):
        self.instances = instances


def _cluster_instance_dependency(instances):
    """For every `chip.Instance` in `instances` (assumed REMOVABLE -- see
    _flatten_cluster_combinational), map each of its own output-pin nets
    to the list of its own input-pin nets it structurally depends on -- a
    coarse, safe over-approximation of true boolean dependency (an output
    is treated as depending on EVERY one of its own input pins, not just
    the ones its own minimized expression actually uses), good enough to
    SCOPE which free variables a downstream target net could possibly
    depend on without needing to inspect any leaf cell's own logic."""
    depends_on = {}
    for inst in instances:
        in_nets = [inst.global_pins.get(p) for p in inst.cell.input_labels]
        in_nets = [n for n in in_nets if n is not None]
        for pin in inst.cell.output_labels:
            out_net = inst.global_pins.get(pin)
            if out_net is not None:
                depends_on[out_net] = in_nets
    return depends_on


def _cluster_output_drivers(instances):
    """net -> the one `chip.Instance` in `instances` whose own output pin
    drives it -- the removal candidate for that net, if it ever gets
    flattened (see _flatten_cluster_combinational)."""
    driver = {}
    for inst in instances:
        for pin in inst.cell.output_labels:
            net = inst.global_pins.get(pin)
            if net is not None:
                driver[net] = inst
    return driver


def _relevant_free_nets(target, depends_on, free_nets):
    """Backward reachability walk from `target` through `depends_on` (see
    _cluster_instance_dependency), collecting every FREE net (a member of
    `free_nets` -- this cluster's own external inputs, or a KEPT
    sequential instance's own output pin) reachable from it -- the
    minimal set of variables `target`'s own flattened expression can
    possibly depend on.

    A free net terminates the walk there (its own value is exogenous to
    the removable sub-network being flattened, never itself decomposed
    further even if -- in some cell library this project hasn't seen yet
    -- it happened to coincide with a `depends_on` key too). A net that's
    neither free nor has its own `depends_on` entry is a dead end this
    function can't account for (shouldn't happen for a net genuinely
    internal to the cluster, since cluster_io_nets() already guarantees
    every such net has SOME driver inside the cluster) -- returns None
    rather than silently producing an incomplete variable set.
    """
    seen, stack, relevant = set(), [target], set()
    while stack:
        net = stack.pop()
        if net in seen:
            continue
        seen.add(net)
        if net in free_nets:
            relevant.add(net)
        elif net in depends_on:
            stack.extend(depends_on[net])
        else:
            return None
    return relevant


def _backward_driven_nets(target, depends_on):
    """Every net -- including `target` itself -- reachable backward from
    `target` through `depends_on` that's actually driven by a removable
    instance (i.e. every net with its own `depends_on` entry visited
    along the way, EXCLUDING the free-net leaves _relevant_free_nets()
    would stop at). This is "every net `target`'s own computation needs a
    REAL WIRE from" -- used to find every instance an INELIGIBLE target
    still needs kept, transitively, even when some of those same
    instances also happen to feed a DIFFERENT, otherwise-flattenable
    target (see _flatten_cluster_combinational's own "stays" reasoning)."""
    seen, stack, driven = set(), [target], set()
    while stack:
        net = stack.pop()
        if net in seen:
            continue
        seen.add(net)
        if net in depends_on:
            driven.add(net)
            stack.extend(depends_on[net])
    return driven


def _flatten_cluster_combinational(chip, cluster, canonical_rc, leaf_modules, aliases):
    """Try to replace as much of a cluster's own recovered gate network as
    possible with minimized z3-derived boolean `assign`s, WITHOUT
    requiring the whole cluster -- or even every individual target within
    it -- to be small enough to flatten in one go. See the module
    docstring's "Cluster-level combinational flattening" section for the
    full reasoning; this is the part that decides, PER TARGET NET, whether
    IT specifically can be flattened, rather than an all-or-nothing swap
    for the whole cluster TYPE. A wide ripple-carry adder is the
    motivating case: its low-order sum bits each depend on only a couple
    of input bits and flatten happily, even though its own high-order sum
    bit's full carry-chain fan-in is far too wide to brute-force -- an
    all-or-nothing rule would have thrown away every one of the small,
    easy wins just because ONE target in the same cluster was too big.

    Args:
        chip: the chip.Chip `cluster` came from.
        cluster: the clustering.Cluster to flatten (`canonical_rc`'s own
            physical cluster).
        canonical_rc: that cluster's own _RawCluster.
        leaf_modules: {cell_name: LeafModule} for every leaf type in the
            design -- used only to read each instance's own `.kind`
            (sequential instances are always KEPT, never flattened away).
        aliases: this type's own bit-select alias map (from
            _bit_select_aliases, already computed by the caller for bus
            grouping) -- reused here so a flattened expression, or a
            surviving instance's own remaining connections, correctly
            reads e.g. "A0_bus[3]" instead of a raw net name a bus port
            may have absorbed.

    Returns (comb_exprs, kept_instances):
      - comb_exprs: {target_alias: verilog_expr_string} for every net
        that successfully flattened -- a cluster external output, or a
        surviving (sequential, or too-big-to-flatten) instance's own
        input pin net -- keyed the same way the caller's own `aliases`
        already keys instance connections.
      - kept_instances: every instance that's NOT being replaced by an
        `assign` -- every sequential instance, unconditionally, plus
        whichever combinational instances still have at least one output
        net that stayed unflattened (see below). Unchanged InstanceCalls,
        still needing the caller's own `aliases` substitution applied
        when building their final port_map, same as every other instance.
        (The caller runs a SECOND, much simpler pass -- see
        _inline_combinational_instances below -- over exactly these
        survivors before deciding what still needs a `wire` declaration
        at all, so this function doesn't need to work that out itself.)

    Or (None, None) if there's nothing to flatten at all: every instance
    is sequential (nothing removable), every target is already wired
    directly with no intervening logic, or NOTHING ended up both eligible
    (small enough free-variable set) and safe (see below).
    """
    kept_instances, kept_chip_instances, removable_chip_instances = [], [], []
    for local_inst, chip_inst in zip(canonical_rc.instances, cluster.instances):
        leaf = leaf_modules[chip_inst.cell.cell_name]
        if leaf.kind == "sequential":
            kept_instances.append(local_inst)
            kept_chip_instances.append(chip_inst)
        else:
            removable_chip_instances.append(chip_inst)

    if not removable_chip_instances:
        return None, None

    depends_on = _cluster_instance_dependency(removable_chip_instances)
    driver_of = _cluster_output_drivers(removable_chip_instances)

    # every net _relevant_free_nets() can return is, by construction,
    # touched by SOME instance's own pin inside this cluster (an external
    # input must be read by one, per cluster_io_nets()'s own definition;
    # a KEPT instance's own output net is touched by that instance's own
    # pin) -- so this map, built the same way canonical_rc's own local
    # names were built in the first place, covers every free net a
    # flattened expression could ever need to name.
    chip_to_local = {}
    local_by_chip_inst = {}
    for local_inst, chip_inst in zip(canonical_rc.instances, cluster.instances):
        local_by_chip_inst[id(chip_inst)] = local_inst
        for pin, net in chip_inst.global_pins.items():
            if pin in local_inst.port_map:
                chip_to_local[net] = local_inst.port_map[pin]

    def local_alias(chip_net):
        local_net = chip_to_local.get(chip_net, chip_net)
        return aliases.get(local_net, local_net)

    chip_ins, chip_outs = cluster_io_nets(chip, cluster)
    free_nets = set(chip_ins)
    for chip_inst in kept_chip_instances:
        for pin in chip_inst.cell.output_labels:
            net = chip_inst.global_pins.get(pin)
            if net is not None:
                free_nets.add(net)

    targets = []  # chip-wide net names
    for net in chip_outs:
        if net in depends_on:
            targets.append(net)
    for chip_inst in kept_chip_instances:
        for pin in chip_inst.cell.input_labels:
            net = chip_inst.global_pins.get(pin)
            if net is not None and net in depends_on:
                targets.append(net)

    if not targets:
        return None, None  # every kept-instance input & external output is already wired directly

    # split into ELIGIBLE (small enough free-variable set to brute-force)
    # and INELIGIBLE (too wide, or a dead end -- see _relevant_free_nets)
    # -- purely a per-target size check, independent of every other
    # target, unlike the old all-or-nothing version.
    relevant_by_target, eligible, ineligible = {}, [], []
    for net in targets:
        relevant = _relevant_free_nets(net, depends_on, free_nets)
        if relevant is not None and len(relevant) <= MAX_COMB_INPUTS:
            relevant_by_target[net] = sorted(relevant)
            eligible.append(net)
        else:
            ineligible.append(net)

    def instances_needed_for(nets):
        """Every removable instance some net in `nets` needs a REAL WIRE
        from, transitively (its full backward-driven closure through
        depends_on, not just the free-variable leaves _relevant_free_
        nets() stops at) -- an instance can't be removed while ANY net in
        its own upstream-of-something-still-real chain is one of these."""
        needed_nets = set()
        for net in nets:
            needed_nets |= _backward_driven_nets(net, depends_on)
        return {driver_of[net] for net in needed_nets}

    # An INELIGIBLE target's own value still needs real gates, so every
    # instance transitively feeding it has to stay instantiated no matter
    # how small some OTHER target sharing part of that same fan-in
    # happens to be -- otherwise flattening the small one would delete
    # the very instance the big one still needs a physical connection to.
    # An eligible target whose own driver got caught up in this is
    # demoted right back to "leave alone" BEFORE any z3 work is spent on
    # it: its own instance has to stay anyway, so flattening it too would
    # just add a second, conflicting driver for the same net.
    stays_instances = instances_needed_for(ineligible)
    flatten_targets = [net for net in eligible if driver_of[net] not in stays_instances]
    if not flatten_targets:
        return None, None

    # the z3 model needs EVERY removable instance's own transistors, not
    # just the ones we're actually about to remove: a flattened target's
    # own dependency chain can legitimately pass straight through an
    # instance that ends up staying instantiated (shared upstream logic
    # an ineligible target also needs) -- that instance still has to be
    # in the model to correctly derive the flattened target's own value,
    # even though it separately also survives in `kept_instances` below.
    circuit, net_vars = build_cluster_z3_circuit(_InstanceSubset(removable_chip_instances))
    solver = z3.Solver()
    solver.add(circuit)

    comb_exprs = {}
    flattened_nets = set()
    for net in flatten_targets:
        input_nets = relevant_by_target[net]
        input_labels = [local_alias(n) for n in input_nets]
        n = len(input_nets)
        target_var = net_vars[net]

        minterms = []
        ambiguous = False
        for combo in product((0, 1), repeat=n):
            solver.push()
            for in_net, v in zip(input_nets, combo):
                solver.add(net_vars[in_net] == bool(v))
            solver.push()
            solver.add(target_var == True)
            can_high = solver.check() == z3.sat
            solver.pop()
            solver.push()
            solver.add(target_var == False)
            can_low = solver.check() == z3.sat
            solver.pop()
            solver.pop()
            if can_high == can_low:  # both (ambiguous) or neither (unsat) -- not eligible after all
                ambiguous = True
                break
            if can_high:
                minterms.append(_combo_to_int(combo))

        if ambiguous:
            # this target's own removable sub-network turned out not to
            # be purely combinational after all (an "unsupported" leaf
            # hiding real internal state, see the module docstring) --
            # defense in depth: leave THIS target's own driving instance
            # in place rather than trust a wrong answer. Every other
            # already/still-to-be-computed target is unaffected.
            continue

        # fits MAX_COMB_INPUTS doesn't mean the minimized result is
        # actually READABLE -- a mid-complexity function of even 6-7
        # variables can still need dozens of OR-ed terms if it just
        # doesn't factor nicely (see MAX_SOP_TERMS's own comment). A
        # result that big is worse for "close to what a human would
        # write" than Pass 5's own per-gate equation chain would be for
        # the SAME logic, so it's rejected here exactly like an ambiguous
        # target -- left alone, not flattened, no instance removed.
        if minterms and len(minterms) < 2 ** n and len(_minimize_sop(minterms, n)) > MAX_SOP_TERMS:
            continue

        comb_exprs[local_alias(net)] = _sop_expr(minterms, n, input_labels)
        flattened_nets.add(net)

    if not comb_exprs:
        return None, None

    # Dead-code sweep: an instance whose own output was never itself a
    # TARGET (a purely internal net, e.g. one ripple-carry stage's own
    # intermediate signal) doesn't need flattening of its own -- its
    # value already gets bypassed symbolically by whichever downstream
    # target's expression absorbed it -- but it only stays REMOVABLE as
    # long as nothing still-unflattened needs a real wire from it either.
    # Recomputed from every target that DIDN'T end up with a comb_expr
    # (ineligible, demoted, OR ambiguous -- all three collapse into the
    # same "still needs real gates" case here), not just `ineligible`:
    # this is what correctly drops D#1/C#2-style intermediate gates once
    # their sole consumer (the target that used to read them) itself got
    # flattened away, even though those intermediate nets were never
    # targets in their own right and never got their own `assign`.
    unflattened_targets = [net for net in targets if net not in flattened_nets]
    still_needed = instances_needed_for(unflattened_targets)

    for chip_inst in removable_chip_instances:
        if chip_inst in still_needed:
            kept_instances.append(local_by_chip_inst[id(chip_inst)])

    return comb_exprs, kept_instances


# ---------------------------------------------------------------------------
# Pass 5: per-instance combinational inlining (no z3 -- see docstring)
# ---------------------------------------------------------------------------


def _substitute_expr(expr, subs):
    """Textually substitute every whole-word occurrence of a key in
    `subs` with its own value, in ONE simultaneous regex pass -- never
    sequential substitution, which could re-substitute a net name that
    happens to also match a DIFFERENT pin's own name. Word-boundary
    matching is safe here because every key is a leaf cell's own PIN
    name: always a plain identifier, never a substring that could span
    into an adjacent token."""
    if not subs:
        return expr
    pattern = re.compile(r"\b(" + "|".join(re.escape(p) for p in subs) + r")\b")
    return pattern.sub(lambda m: subs[m.group(1)], expr)


def _inline_combinational_instances(body_instances, leaf_by_name):
    """Replace every COMBINATIONAL instance in `body_instances` with its
    own already-known boolean expression, inlined directly as an
    `assign` -- see the module docstring's "Per-instance combinational
    inlining" section. A pure 1:1 rewrite, no z3, no enumeration, no size
    cap of any kind: decompile_leaf() already minimized this cell TYPE's
    own expression once (reused across every instance of it), so this
    just restates that same expression in terms of THIS instance's own
    actually-connected nets (`body_instances` is already alias-
    substituted by the caller, same as every other instance connection,
    so no extra aliasing is needed here).

    This is what removes the sky130 gates Pass 4 (_flatten_cluster_
    combinational) leaves behind because their own boundary target was
    too wide to brute-force -- without trying to force them into one
    giant expression, which is exactly the wrong representation for
    something like a ripple-carry chain (see the module docstring). It
    keeps the SAME wire-by-wire structure a human would naturally write
    (one small equation per signal), just with every `sky130_fd_sc_hd__*`
    instantiation gone.

    A KEPT sequential instance, or an "unsupported" leaf (decompile_leaf()
    couldn't determine its function at all -- see LeafModule.kind), is
    left exactly as it was: there's no expression to inline for either.

    Returns (remaining_instances, inline_exprs): `remaining_instances` is
    `body_instances` minus every inlined one; `inline_exprs` is
    {net: expr_string} for everything that got inlined.
    """
    remaining, inline_exprs = [], {}
    for inst in body_instances:
        leaf = leaf_by_name[inst.module_name]
        if leaf.kind != "combinational":
            remaining.append(inst)
            continue
        subs = {pin: net for pin, net in inst.port_map.items() if pin in leaf.input_labels}
        for out_pin, expr in leaf.comb_exprs.items():
            out_net = inst.port_map.get(out_pin)
            if out_net is not None:
                inline_exprs[out_net] = _substitute_expr(expr, subs)
    return remaining, inline_exprs


# ---------------------------------------------------------------------------
# Pass 6: top-level bus/vector wire grouping
# ---------------------------------------------------------------------------


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

    EXCEPT for a group containing a real chip-level port (one of
    top.input_ports/output_ports): that net's own identifier is the
    module's public interface, already fixed by build_ast's earlier
    `net in chip.primary_outputs`/`primary_inputs` classification -- it
    is not this function's to rename. Folding it into a synthesized
    "<some_other_net>_bus" name here would silently orphan the port
    declaration: the PortDecl would still say e.g. "output success", but
    every actual connection to that net would now read the new bus name
    instead, leaving "success" declared and never referenced anywhere in
    the module body. Skipping the collapse for these groups leaves that
    net as an explicit concatenation member instead (still 100% correct
    Verilog, decompiler._conn_str() already renders a list as "{...}"),
    which keeps the port's own name the thing every connection actually
    uses.

    Three more real bugs, all found on puzzle.gds -- large enough to hit
    cases adder_demo never did -- and fixed here:

    1. The synthesized bus name itself (`f"{sorted(key)[0]}_bus"`) is
       only unique GIVEN a fixed group of nets -- nothing stopped two
       DIFFERENT groups (different physical buses, different width, e.g.
       one cluster type's own 4-bit input and a totally different type's
       own 5-bit input that happens to also start with the same net)
       from sharing an alphabetically-first member and colliding on the
       identical wire name, silently redeclaring it with a different
       width the second time (confirmed: two `wire ... A_40_bus;`
       declarations, 4-bit and 5-bit, on the real design). Now deduped
       via the same _dedupe() every other net-naming path here already
       uses.
    2. A net can be a plain SCALAR port on its own DRIVER instance's
       side while ALSO being grouped into a bus on some (different)
       CONSUMER instance's side -- e.g. a cluster type whose own
       grouping decision (_group_ports_into_buses, made once from the
       canonical member and now checked for consistency across every
       member of that SAME type -- see its own docstring) still can't
       know anything about a COMPLETELY DIFFERENT type on the other end
       of the connection: the driver's own single net can have exactly
       one consumer and safely bus-group on ITS OWN side, while that one
       consumer's own port stays scalar anyway because a totally
       unrelated SIBLING instance of the consumer's type has some other,
       unrelated fan-out at that same structural port position (confirmed
       on puzzle.gds: a cluster type with two instances, where the
       instance picked canonical had a 3-way fan-out forcing that type's
       port scalar, while the OTHER instance's own actual driver had a
       clean single-consumer bus ready to feed it -- that driver bus-
       grouped anyway, since nothing here told it its one consumer would
       end up scalar). The old fix for this (keep the scalar wire
       declared so the driver's own connection doesn't reference an
       undeclared identifier) only prevented a SYNTAX error -- it left
       the driver's own scalar wire and the consumer's newly-synthesized
       bus wire as two DIFFERENT, disconnected nets, which is exactly as
       wrong, just no longer a compile error. The actual fix: a net that
       is EVER referenced as a bare scalar anywhere in the design is
       simply never eligible to anchor a synthesized bus wire at all --
       every OTHER (list-valued) connection to that same net group falls
       back to an explicit concatenation of the scalar wires that
       already exist (`{n1, n2, ...}`, still 100% correct Verilog, see
       the "EXCEPT for a real chip-level port" case above for the exact
       same fallback already used there), rather than inventing a
       parallel, physically-unconnected identity for nets that already
       have one.
    """
    protected = {p.name for p in top.input_ports} | {p.name for p in top.output_ports}

    # a net referenced as a bare SCALAR anywhere (any instance, any pin,
    # regardless of direction) can never safely anchor a synthesized bus
    # wire for some OTHER connection to the same nets -- see bug 2 above.
    # Computed FIRST and excluded from group formation entirely, not
    # patched up after the fact, so a conflicting group never gets a
    # PortDecl (and hence a wire declaration) in the first place; the
    # rewriting loop below then leaves every connection to it as a plain
    # list, which decompiler._conn_str() renders as a concatenation of
    # the already-existing scalar wires -- always electrically correct,
    # never a second, disconnected identity for the same physical nets.
    scalar_refs = {net for inst in top.instances for net in inst.port_map.values() if isinstance(net, str)}

    groups = {}     # frozenset(nets) -> PortDecl
    bit_index = {}  # frozenset(nets) -> {net: canonical bit index}
    used_names = set(top.wires) | protected
    for inst in top.instances:
        for net in inst.port_map.values():
            if not isinstance(net, list):
                continue
            if any(n in protected or n in scalar_refs for n in net):
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
                name = _dedupe(f"{sorted(key)[0]}_bus", used_names)
                used_names.add(name)
                groups[key] = PortDecl(name=name, width=len(net))
                bit_index[key] = {n: i for i, n in enumerate(net)}

    grouped_nets = {n for key in groups for n in key}
    scalar_wires = [PortDecl(name=w, width=1) for w in top.wires if w not in grouped_nets]
    top.wires = scalar_wires + list(groups.values())

    for inst in top.instances:
        for pin, net in list(inst.port_map.items()):
            if not isinstance(net, list):
                continue
            key = frozenset(net)
            if key not in groups:
                continue  # a protected or scalar-conflicting group -- left as an explicit concatenation, see above
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

    cluster_by_id = {c.id: c for c in clusters}

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
            # Printed, not just silently dropped: this is the OTHER way
            # (besides _dedupe_cluster_types's own reporting) the
            # rendered Verilog's cluster-module count can come out lower
            # than plot_clusters()/plot_clusters_optimized()'s physical
            # cluster count -- an entire cluster vanishing, not several
            # sharing one module definition.
            print(f"cluster {c.id} emptied out entirely by buffer collapsing -- no module or instantiation emitted for it")
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
    leaf_by_name = {leaf.name: leaf for leaf in leaf_modules.values()}
    n_flattened_types = n_flattened_clusters = n_flattened_gates = 0
    n_inlined_types = n_inlined_gates = 0

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
            canonical_rc, driver_of, consumers_of, t["members"], preferred_order=preferred_order,
        )

        canonical_cluster = cluster_by_id[canonical_rc.cluster_id]

        aliases = _bit_select_aliases(in_decls, positions, canonical_rc.input_ports)
        aliases.update(_bit_select_aliases(out_decls, positions, canonical_rc.output_ports))

        # Pass 4: try to flatten as much of this type's structural
        # body as possible into minimized combinational assigns -- a
        # KEPT sequential instance (if any) stays instantiated
        # regardless; see the module docstring for the full per-
        # instance/per-target design. Run from the canonical member
        # only, like bus grouping; the resulting ClusterModule is
        # shared by every isomorphic member.
        comb_exprs, kept_instances = _flatten_cluster_combinational(
            chip, canonical_cluster, canonical_rc, leaf_modules, aliases,
        )

        if comb_exprs is None:
            comb_exprs, kept_instances = {}, canonical_rc.instances
        else:
            n_flattened_types += 1
            n_flattened_clusters += len(t["members"])
            n_flattened_gates += len(canonical_rc.instances) - len(kept_instances)

        body_instances = [
            InstanceCall(inst.module_name, inst.inst_name,
                          {pin: aliases.get(net, net) for pin, net in inst.port_map.items()})
            for inst in kept_instances
        ]

        # Pass 5: inline every SURVIVING combinational instance -- Pass
        # 4's own boundary-level flatten already removed as much as it
        # safely/cheaply could; this is the fallback that removes every
        # remaining sky130 gate regardless of width, at the cost of
        # keeping intermediate wire names (a carry chain, say) instead
        # of collapsing them away -- see the module docstring. Never
        # fails, so there's no branch to fall back from here.
        body_instances, inline_exprs = _inline_combinational_instances(body_instances, leaf_by_name)
        if inline_exprs:
            n_inlined_types += 1
            n_inlined_gates += len(inline_exprs)
        comb_exprs.update(inline_exprs)

        used_nets = {net for inst in body_instances for net in inst.port_map.values()} | set(comb_exprs)
        internal_wires = [PortDecl(name=w, width=1) for w in canonical_rc.internal_wires if w in used_nets]

        cluster_modules.append(ClusterModule(
            name=type_name,
            input_ports=in_decls, output_ports=out_decls,
            internal_wires=internal_wires,
            instances=body_instances,
            comb_exprs=comb_exprs or None,
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

    if n_flattened_types:
        print(f"flattened {n_flattened_types} cluster type(s) ({n_flattened_clusters} physical cluster(s), "
              f"{n_flattened_gates} combinational gate instance(s) eliminated from the canonical member "
              f"alone) into minimized combinational assign(s) via z3")
    if n_inlined_types:
        print(f"inlined {n_inlined_gates} more combinational gate instance(s), across {n_inlined_types} "
              f"cluster type(s), as their own per-instance assign(s) (Pass 5 -- see chip_manipulation.py)")

    top_namer = namer.scope()
    driven, consumed = set(), set()
    for ins, outs in cluster_io.values():
        driven.update(outs)
        consumed.update(ins)

    # Which category a net falls into is decided by chip.primary_inputs/
    # primary_outputs -- chip.py's own geometric (routing-crosses-the-
    # chip-boundary) test, NOT by re-deriving "driven but never consumed
    # = output" here. That re-derivation used to be exactly what this
    # loop did, and it's wrong for the same two reasons chip.py's own
    # version was: a real output can ALSO be read internally (e.g. a
    # status flag also fed back into control logic -- it'd land in
    # `driven and consumed` above and get demoted to an internal wire,
    # silently losing its own top-level port), and a net that's merely
    # driven-with-no-consumer isn't necessarily a real port at all (a
    # spare/unused cell's dangling output looks identical by this
    # topology-only test). `driven`/`consumed` are still needed here,
    # though -- they're what tells us EVERY net that crosses a cluster-
    # module boundary and therefore needs SOME top-level representation
    # (port or wire); chip.primary_inputs/outputs then decides which of
    # those get a real port instead of just an internal wire.
    top_inputs, top_outputs, top_wires = [], [], []
    for net in sorted(driven | consumed):
        ident = top_namer(net)
        if net in chip.primary_outputs:
            top_outputs.append(PortDecl(name=ident))
        elif net in chip.primary_inputs:
            top_inputs.append(PortDecl(name=ident))
        else:
            top_wires.append(ident)

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
