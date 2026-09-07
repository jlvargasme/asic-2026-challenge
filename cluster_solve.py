"""
cluster_solve.py: solve ONE cluster's own combinational logic for the
input values that produce a target output -- IGNORING every flip-flop
inside it entirely (no clock, no reset, no D/Q relationship modeled at
all; a flip-flop's own Q becomes just another free variable, exactly
like a genuine primary input). Built on netlist.py's yosys-derived JSON
netlist (see its own module docstring for why yosys instead of a
hand-written Verilog parser).

This is the replacement this project settled on after the earlier
automated, multi-cycle, multi-cluster z3 backward-search chain
(decompiler.backtrack_inputs/backtrack_sequence,
clustering._solve_sequential_cluster/find_cluster_sequence_for_output --
all removed) turned out not to work reliably enough on puzzle.gds to
trust: that machinery tried to ALSO verify a flip-flop actually captures
the solved value across a real clock edge (a z3-then-PySpice hybrid),
and to find a genuine multi-cycle loading SEQUENCE when one edge from
reset wasn't enough -- both real, hard problems this project already
spent significant effort on (see HANDOFF.md's own account: confirmed
z3-vs-PySpice mismatches on an 8-flip-flop cluster with no settle-time
explanation found).

The approach here is deliberately much weaker, in exchange for actually
being tractable: instead of solving "what sequence of inputs, over
however many clock cycles, makes this register capture this value",
this asks a strictly easier question one cluster at a time -- "IF this
flip-flop's own Q could be set to whatever value is needed (in one step,
completely ignoring how many cycles or what the actual prior state would
need to be), what does that require of the stage BEFORE it" -- and lets
you (or a driver script, see backtrace_solve() below) chain that
question backward, cluster by cluster, using backtrace.py's own fan-in
graph to know which cluster to visit next. This is intentionally NOT a
claim that the resulting assignment is reachable from reset in any
particular number of cycles -- it tells you what's LOGICALLY consistent
one hop at a time, nothing about the sequential TIMING to get there.
That's a real, deliberate trade-off, not an oversight.

z3 BitVec, not per-bit Bool: every net (a whole port, a whole cell pin,
a whole bus) is modeled as ONE z3 BitVec -- width N, bit i = that net's
i-th bit -- built out of 1-bit BitVec "atoms" (ClusterModel.var(), one
per distinct yosys bit ID) via Concat, rather than N separate boolean
variables reasoned about one at a time. Two real, non-cosmetic reasons:

  - Every gate/query target here is naturally a WORD ("what 9-bit value
    does A_2_bus need"), not an isolated bit, and BitVec equality
    (`word == BitVecVal(n, width)`) says that directly, in ONE
    constraint, instead of N separate per-bit constraints -- the code
    matches the question being asked.
  - It removes a real bug class the previous per-bit-Bool version of
    this file had to work around by hand: solving a wide bus bit-by-bit
    across separate solver calls lets z3 pick arbitrary, mutually
    INCONSISTENT values for that bus's other (that call's own
    unconstrained) bits each time, since a fresh solver.check() has no
    memory of a previous call's choices. The old code patched over this
    by manually batching bits into one solve_for_bits() call per port
    before recursing (see git history). With BitVec, a whole port IS
    one variable, solved in one shot -- there's no bit-by-bit path left
    to accidentally take, so the inconsistency simply can't occur; nothing
    to remember to batch.

Bit 0 of a net's own BitVec is defined as whichever bit module.bit_name()
labels "[0]" (or the bit itself, for a scalar net) -- i.e. list position
0 in module.port_bits()/a cell's own connection list. Per
chip_manipulation.py's own documented limitation (still true here, same
underlying nets), this is a stable, deterministic convention, NOT a
reconstruction of true numeric significance (MSB/LSB) -- fine, since
nothing here claims otherwise; only internal consistency matters.
"""

import z3

from netlist import load_netlist, modules


def _bit_val(bit):
    """A yosys connection bit is either an int (a real net bit ID) or
    one of the strings "0"/"1"/"x"/"z" (a constant tie) -- resolve a
    constant to a concrete 0/1, or None for a real net bit / "x"/"z"
    (don't-care, left unconstrained)."""
    if bit == "0":
        return False
    if bit == "1":
        return True
    return None


def _split_bus_index(name):
    """"A_10_bus[3]" -> ("A_10_bus", 3); "B" -> ("B", None)."""
    if name.endswith("]") and "[" in name:
        base, idx = name.rsplit("[", 1)
        return base, int(idx[:-1])
    return name, None


class ClusterModel:
    """A z3 BitVec model of one module's own PURELY COMBINATIONAL cells --
    every '$'-prefixed yosys primitive cell (see YOSYS_CELL_OPS)
    contributes ONE word-level constraint tying its own output net to a
    BitVec function of its own input net(s); any OTHER cell -- a
    submodule instantiation, which in this project's own decompiled
    output is always one of the KEPT sequential leaf types left
    instantiated (e.g. sky130_fd_sc_hd__dfrtp_2) -- is skipped entirely,
    its own output bit(s) left as ordinary FREE z3 variables with no
    constraint at all. That skip is the literal meaning of "ignoring
    flip-flops" here: no clock, no reset, no D/Q relationship, just
    "this bit's value isn't determined by this module's own
    combinational logic."

    Attributes:
        module: the netlist.Module this was built from.
        solver: a z3.Solver already populated with every combinational
            cell's own constraint (nothing pushed/popped yet).
        skipped: {inst_name: cell_type} for every non-'$' cell that was
            left unmodeled (informational -- lets a caller confirm which
            flip-flops actually got ignored, rather than silently
            trusting it).
    """

    def __init__(self, module):
        self.module = module
        self.solver = z3.Solver()
        self._vars = {}
        self._driven_bits = set()  # bit IDs fully determined by a modeled cell -- not free
        self.skipped = {}
        for inst_name, cell in module.cells.items():
            ctype = cell["type"]
            if not ctype.startswith("$"):
                self.skipped[inst_name] = ctype
                continue
            handler = YOSYS_CELL_OPS.get(ctype)
            if handler is None:
                raise ValueError(
                    f"{module.name}.{inst_name}: unsupported yosys cell type {ctype!r} -- "
                    f"add a handler to cluster_solve.YOSYS_CELL_OPS rather than silently ignoring it"
                )
            handler(self, cell["connections"])
            directions = cell.get("port_directions", {})
            for port, bits in cell["connections"].items():
                if directions.get(port) == "output":
                    self._driven_bits.update(b for b in bits if isinstance(b, int))

    def var(self, bit):
        """The 1-bit z3 BitVec for one bit ID -- a fresh free variable
        the first time it's seen, a concrete z3.BitVecVal for a yosys
        constant tie ("0"/"1"), or a genuinely free (unconstrained)
        fresh variable for "x"/"z" (don't-care)."""
        if isinstance(bit, str):
            v = _bit_val(bit)
            if v is not None:
                return z3.BitVecVal(1 if v else 0, 1)
            bit = f"const_{bit}_{len(self._vars)}"  # "x"/"z" -- unique per occurrence
        if bit not in self._vars:
            self._vars[bit] = z3.BitVec(f"b{bit}", 1)
        return self._vars[bit]

    def _word(self, bits):
        """The N-bit z3 BitVec for a yosys bit-ID list -- bits[0] is bit
        0 (see the module docstring's bit-order convention)."""
        if len(bits) == 1:
            return self.var(bits[0])
        return z3.Concat(*[self.var(b) for b in reversed(bits)])

    def solve_for_bits(self, bits, value, necessary_only=True):
        """Find ONE satisfying assignment making the net formed by
        `bits` (bits[0] = bit 0) equal the non-negative integer `value`
        -- a single BitVec equality, so a wide target is always solved
        as ONE atomic word, never bit-by-bit (see the module docstring
        for why that distinction is load-bearing here, not cosmetic).
        The general primitive both solve_for_output() and
        solve_for_pin() are built on; use this directly for a target
        that isn't cleanly "one whole port" or "one whole cell pin"
        (e.g. a specific slice of a wide internal bus).

        necessary_only=False skips the necessity check below entirely
        and returns z3's own raw witness instead -- ONE valid input
        combination that reaches the target, with no claim that any
        particular part of it was actually required. Useful when you
        want a concrete example to look at (e.g. "some pair of operands
        that sums to 457") rather than the (very often much smaller, or
        even empty) set of things truly forced -- e.g. adder_demo's own
        S=1 has 15 valid (A_10_bus, A0_4_bus) pairs, so NEITHER operand
        is individually necessary at all; necessary_only=True correctly
        reports that as "0 required", which is honest but not always
        what you're after.

        Returns {name: int} for every GENUINELY free variable this
        model's own combinational logic leaves undetermined -- this
        cluster's own input port bits (regrouped into one integer per
        bus via module.bit_name()), plus any skipped flip-flop/
        submodule's own output pin bits (still ordinary free variables
        here, per the module docstring's "ignoring flip-flops" design).
        Every bit that IS the output of one of this model's own modeled
        cells is left out entirely -- its value is fully implied by the
        free variables above, not a separate decision, so reporting it
        would just be noise, not information.

        Returns None if UNSAT (no assignment of this cluster's own free
        variables satisfies the target at all, given its OWN
        combinational logic alone -- e.g. a target that can only be
        reached with a specific PRIOR flip-flop state this single-step
        model can't express, see the module docstring's own account of
        what "ignoring flip-flops" deliberately gives up).

        A reported free variable is always NECESSARY for the target, not
        just "one witness z3 happened to pick" -- checked directly (does
        the target still hold if this one is flipped, everything else
        still free?), not assumed. This matters a lot for a target that's
        ALREADY a free variable itself (e.g. a port that's directly a
        flip-flop's own Q -- see solve_for_output()'s own note on that
        trivial case): every OTHER free variable is then a genuine
        don't-care for this target, and z3's model_completion still has
        to pick SOME concrete value for each of them to hand back a
        model at all -- reporting those arbitrary picks as if the target
        required them was a real, confirmed bug in backtrace_solve()'s
        own use of this method: it recursed into (and permanently
        pinned) a wholly unnecessary value, manufacturing a downstream
        conflict with a completely unrelated, genuinely-necessary
        requirement on the SAME shared instance reached via a different
        path -- see backtrace_solve's own docstring for the exact case
        this was found on.
        """
        width = len(bits)
        word = self._word(bits)
        self.solver.push()
        self.solver.add(word == z3.BitVecVal(value, width))
        if self.solver.check() != z3.sat:
            self.solver.pop()
            return None
        m = self.solver.model()
        per_bit = {}
        for bit_id, v in self._vars.items():
            if isinstance(bit_id, str) or bit_id in self._driven_bits:
                continue  # an "x"/"z" placeholder, or a fully-derived internal net -- not free
            per_bit[bit_id] = m.eval(v, model_completion=True).as_long()

        assignment = {}
        grouped = {}
        for bit_id, bit_val in per_bit.items():
            name = self.module.bit_name(bit_id)
            base, idx = _split_bus_index(name)
            if idx is None:
                assignment[name] = bit_val
            else:
                grouped.setdefault(base, {})[idx] = bit_val
        for base, idxs in grouped.items():
            n = 0
            for i, bit_val in idxs.items():
                n |= bit_val << i
            assignment[base] = n

        if not necessary_only:
            self.solver.pop()
            return assignment

        # necessity check -- still under the target constraint (pushed
        # above, not yet popped): for each reported (name, val), is
        # val the ONLY value consistent with the target, or does the
        # target still hold with this net at some OTHER value too? Only
        # the former is a real requirement worth reporting/recursing
        # into; the latter is exactly the "arbitrary witness" gap
        # described above.
        necessary = {}
        for name, val in assignment.items():
            net_bits = self.module.netnames[name]["bits"]
            net_width = len(net_bits)
            self.solver.push()
            self.solver.add(self._word(net_bits) != z3.BitVecVal(val, net_width))
            still_sat = self.solver.check() == z3.sat
            self.solver.pop()
            if not still_sat:
                necessary[name] = val
        self.solver.pop()
        return necessary

    def solve_for_output(self, port_name, value, necessary_only=True):
        """solve_for_bits(), targeting the whole `port_name` net as one
        integer (bit 0 of `value` = port_bits(port_name)[0]).

        Note this is TRIVIAL (imposes no real constraint at all) for a
        port that's directly a kept flip-flop's own Q pin, since Q is
        itself one of this model's own free variables -- see
        solve_for_pin() instead for the actually useful question in that
        case: what does a flip-flop's own D pin (an INTERNAL net, not a
        port) need to be. See solve_for_bits() for necessary_only."""
        return self.solve_for_bits(self.module.port_bits(port_name), value, necessary_only)

    def solve_for_pin(self, inst_name, pin_name, value, necessary_only=True):
        """solve_for_bits(), targeting one CELL INSTANCE's own pin --
        typically a kept flip-flop's own D input, the "logic before the
        flip-flop" target this module's own docstring describes: what
        does the surrounding combinational logic need to produce so that
        THIS flip-flop would capture `value` on its next real clock edge
        (a claim this single-step model can't itself verify -- see the
        module docstring). See solve_for_bits() for necessary_only."""
        return self.solve_for_bits(self.module.cells[inst_name]["connections"][pin_name], value, necessary_only)

    def pin(self, assignment):
        """Permanently constrain every (name, value) pair in
        `assignment` -- the SAME {name: int} shape solve_for_bits() and
        friends return -- on this model's OWN solver, with no push/pop:
        stays in effect for every solve on this SAME ClusterModel from
        here on.

        Needed when the SAME physical instance is reached more than once
        -- e.g. by backtrace_solve(), when two different downstream
        consumers both trace back to one shared cluster instance. Left
        unpinned, each visit's own solve_for_bits() call is independent
        (push/pop only ever holds the TARGET constraint, never the free
        variables a previous call happened to read off the model), so a
        later visit is free to pick a DIFFERENT, possibly conflicting
        value for this SAME instance's other free variables -- a real,
        confirmed bug: two visits to the same flip-flop-bearing cluster
        picked "enable=1" and "enable=0" independently, and the caller's
        own flattened {label: value} report silently kept whichever
        visit happened to run last, hiding the disagreement entirely.
        Pinning the first visit's own result before recursing further
        means every later solve on this model is forced to stay
        consistent with it -- if that's genuinely impossible, the later
        solve_for_bits() call now correctly returns None (UNSAT) instead
        of silently returning an incompatible witness.

        Every name here is guaranteed to be a module.netnames key: that
        is exactly where solve_for_bits() sourced it from in the first
        place (via module.bit_name(), itself always derived from
        module.netnames -- see netlist.Module.__init__)."""
        for name, value in assignment.items():
            bits = self.module.netnames[name]["bits"]
            self.solver.add(self._word(bits) == z3.BitVecVal(value, len(bits)))

    def pin_bits(self, bits, value):
        """Permanently constrain the net formed by `bits` (bits[0] =
        bit 0) to equal `value`, no push/pop -- the raw-bit-id
        counterpart of pin(), for when the caller already has a bit-ID
        list rather than a display name (e.g. backtrace_solve()'s own
        retry path, replaying an earlier target's own raw local_bits
        onto a freshly rebuilt model -- see its own docstring)."""
        self.solver.add(self._word(bits) == z3.BitVecVal(value, len(bits)))


# ---------------------------------------------------------------------------
# yosys internal cell type -> z3 BitVec constraint builder
# ---------------------------------------------------------------------------
#
# Only $not/$and/$or are actually exercised by puzzle.v today (confirmed
# via `yosys stat` -- every cluster's own combinational logic is plain
# AND/OR/NOT sum-of-products, see chip_manipulation.py's own SOP
# minimizer); the rest are included for the same reason bus grouping
# generally is -- so this keeps working if a future run's own recognized
# logic looks different, rather than silently mis-modeling it. An
# UNSUPPORTED cell type raises (see ClusterModel.__init__) rather than
# being skipped like a flip-flop would be -- skipping is only ever
# correct for a genuine submodule/sequential instance, never for a
# primitive gate this file just doesn't know yet.


def _word_unary(fn):
    def handler(model, conns):
        model.solver.add(model._word(conns["Y"]) == fn(model._word(conns["A"])))
    return handler


def _word_binary(fn):
    def handler(model, conns):
        model.solver.add(model._word(conns["Y"]) == fn(model._word(conns["A"]), model._word(conns["B"])))
    return handler


def _op_mux(model, conns):
    # Y = S ? B : A -- S is a single select bit shared across every bit
    # of A/B/Y (yosys's own $mux is always a plain 2-to-1 here, this
    # project's own decompiler only ever emits a scalar `cond ? a : b`).
    s = model.var(conns["S"][0])
    model.solver.add(
        model._word(conns["Y"]) == z3.If(s == z3.BitVecVal(1, 1), model._word(conns["B"]), model._word(conns["A"]))
    )


def _bit_of(cond):
    return z3.If(cond, z3.BitVecVal(1, 1), z3.BitVecVal(0, 1))


def _nonzero(word):
    return word != z3.BitVecVal(0, word.size())


def _op_logic_not(model, conns):
    model.solver.add(model.var(conns["Y"][0]) == _bit_of(z3.Not(_nonzero(model._word(conns["A"])))))


def _op_logic_and(model, conns):
    a, b = model._word(conns["A"]), model._word(conns["B"])
    model.solver.add(model.var(conns["Y"][0]) == _bit_of(z3.And(_nonzero(a), _nonzero(b))))


def _op_logic_or(model, conns):
    a, b = model._word(conns["A"]), model._word(conns["B"])
    model.solver.add(model.var(conns["Y"][0]) == _bit_of(z3.Or(_nonzero(a), _nonzero(b))))


def _op_reduce_or(model, conns):
    model.solver.add(model.var(conns["Y"][0]) == _bit_of(_nonzero(model._word(conns["A"]))))


def _op_reduce_and(model, conns):
    a = model._word(conns["A"])
    ones = z3.BitVecVal((1 << a.size()) - 1, a.size())
    model.solver.add(model.var(conns["Y"][0]) == _bit_of(a == ones))


YOSYS_CELL_OPS = {
    "$not": _word_unary(lambda a: ~a),
    "$and": _word_binary(lambda a, b: a & b),
    "$or": _word_binary(lambda a, b: a | b),
    "$xor": _word_binary(lambda a, b: a ^ b),
    "$xnor": _word_binary(lambda a, b: ~(a ^ b)),
    "$logic_not": _op_logic_not,
    "$logic_and": _op_logic_and,
    "$logic_or": _op_logic_or,
    "$reduce_and": _op_reduce_and,
    "$reduce_or": _op_reduce_or,
    "$reduce_bool": _op_reduce_or,
    "$mux": _op_mux,
}


def load_cluster(verilog_path, top_module, module_name):
    """Load `module_name` (e.g. one specific "cluster_type_N") out of
    `verilog_path`'s own yosys netlist and build its ClusterModel."""
    data = load_netlist(verilog_path, top_module)
    mod = modules(data)[module_name]
    return ClusterModel(mod)


# ---------------------------------------------------------------------------
# Iterative cross-cluster driver
# ---------------------------------------------------------------------------


def _top_bit_drivers(top):
    """net bit ID -> the TOP-level cell instance name that drives it
    (from every instance's own OUTPUT port connections) -- the same
    "which cluster produces this signal" question backtrace.py's own
    instance_fanin() answers, reused here to decide which cluster to
    solve next."""
    driver = {}
    for inst_name, cell in top.cells.items():
        directions = cell.get("port_directions", {})
        for port, bits in cell["connections"].items():
            if directions.get(port) != "output":
                continue
            for b in bits:
                driver[b] = inst_name
    return driver


def _cell_output_port_bit(cell, bit):
    """(port_name, index_within_port) for whichever OUTPUT port of this
    TOP-level cell instance connects to `bit` (a bit ID in the TOP
    module's OWN numbering), or None if `bit` isn't one of this cell's
    own outputs.

    Needed because yosys's write_json numbers bits INDEPENDENTLY per
    module -- a cell instance's own "connections" record (in the TOP
    module's own JSON) uses the TOP module's numbering for every pin
    regardless of direction, but that SAME net has a completely
    different, unrelated bit ID inside the child module's own JSON
    record (its own numbering, starting fresh). Port name + bit index
    is the only thing that actually stays stable across that boundary,
    so any time a TOP-level bit needs to become an argument to a CHILD
    ClusterModel (built from the child's own Module), it has to be
    translated through (port, index) rather than reused directly."""
    directions = cell.get("port_directions", {})
    for port, bits in cell["connections"].items():
        if directions.get(port) != "output":
            continue
        if bit in bits:
            return port, bits.index(bit)
    return None


def backtrace_solve(verilog_path, top_module, target_port, target_value, target_bit_index=None, max_depth=12):
    """Iteratively solve, cluster by cluster, for a full recipe of
    top-level primary-input assumptions (plus any KEPT flip-flop's own
    state this recipe has to assume, since flip-flops are ignored -- see
    the module docstring) that would make `target_port` (its whole net,
    or just `target_port[target_bit_index]` if given) equal
    `target_value` at the TOP module's own boundary.

    At each step: find which TOP-level cell instance drives the current
    target's own net, build ITS OWN ClusterModel (ignoring its own
    internal flip-flops, same as everywhere else in this file), and
    solve for the required value on the specific pin/bus that was
    targeted -- as ONE BitVec word, never bit-by-bit (see the module
    docstring). Every resulting free variable that turns out to be one
    of THIS cluster's own INPUT ports gets resolved, via the TOP
    module's own connections for this instance, to whichever bit(s)
    drive it next -- recursing into THAT cluster in turn, again as one
    whole-port word. Anything else (a genuine top-level primary input,
    or a free variable that ISN'T simply one of this cluster's own input
    ports -- e.g. a DIFFERENT flip-flop's own Q living inside this SAME
    cluster, not separately targetable this way) becomes a final,
    reported assumption instead: for that last case, use
    ClusterModel.solve_for_pin() directly on this SAME cluster (already
    loaded once per visited instance -- see the printed report for which
    cluster/instance to target) to push one hop further "before the
    flip-flop", the same manual step this module's own docstring
    describes -- this driver only automates the cross-cluster hops, not
    that one, since going past a flip-flop needs a human decision about
    WHICH of its own pins is actually the next meaningful question.

    A single physical instance can be reached more than once -- once
    from each downstream consumer whose own fan-in passes through it --
    and every visit after the first is solved under every EARLIER
    visit's own result already permanently pinned on that instance's
    own model (ClusterModel.pin(), called after every successful solve
    here); see its own docstring for the real, confirmed bug this fixes:
    without it, two independent visits to the same instance could each
    find a locally-valid but MUTUALLY INCONSISTENT witness (e.g. one
    visit solving "enable=1", a later one independently solving
    "enable=0" for that SAME physical enable pin), and the flattened
    assumptions dict below would silently keep whichever visit happened
    to run last, hiding the disagreement entirely.

    A later visit that conflicts with pinned state gets ONE retry before
    being reported as a genuine conflict: a fresh ClusterModel for the
    same instance, with every EARLIER target for it (tracked in
    `inst_targets`, in the order first established) replayed as hard
    constraints via pin_bits() -- NOT the full incidental-turned-
    necessary set pin() would add, just the raw targets themselves, so
    z3 has room to pick different, still-valid internal choices for
    everything else. This is what lets a later visit succeed where the
    original, now-stale model couldn't: solve_for_bits()'s own necessity
    filtering (see its docstring) means an earlier visit's incidental
    free-variable choices were never baked in as hard facts in the first
    place, so a joint re-solve is usually strictly easier to satisfy
    than the original conflict suggested. If the retry is STILL UNSAT,
    the conflict is real and structural (the two requirements are
    mutually exclusive given this instance's own combinational logic,
    full stop) and gets reported as such rather than silently producing
    a wrong, contradictory recipe.

    Prints each hop as it's solved. Returns {description: int} -- the
    flattened set of primary-input/unresolved-free-variable assumptions
    this recipe rests on (a primary input wider than 1 bit is reported
    one bit at a time, as "port[i]", so every entry stays a plain 0/1).
    """
    data = load_netlist(verilog_path, top_module)
    mods = modules(data)
    top = mods[top_module]
    driver_of = _top_bit_drivers(top)
    bit_to_port = {}
    for name, info in top.ports.items():
        for i, b in enumerate(info["bits"]):
            bit_to_port[b] = (name, i)

    assumptions = {}
    visited = set()
    cluster_models = {}  # inst_name -> ClusterModel, built once, reused across hops (rebuilt on a successful retry)
    inst_targets = {}  # inst_name -> [(local_bits, value), ...], every real target solved for so far, in order

    def visit(bits, value, depth, path):
        # every bit here is assumed to share ONE driving instance --
        # true by construction for every group this function is ever
        # called with (see below: groups are built from one whole PORT
        # of one specific consumer instance, and a Verilog port
        # connection always wires to exactly one net group / one driver).
        key = (tuple(bits), value)
        if key in visited:
            return
        visited.add(key)
        indent = "  " * depth
        if depth > max_depth:
            print(f"{indent}max_depth reached -- stopping, treating {path} as an assumption")
            assumptions[path] = value
            return

        inst = driver_of.get(bits[0])
        if inst is None:
            parts = []
            for i, b in enumerate(bits):
                bit_val = (value >> i) & 1
                port_bit = bit_to_port.get(b)
                label = f"{port_bit[0]}[{port_bit[1]}]" if port_bit else f"<bit {b}>"
                assumptions[label] = bit_val
                parts.append(f"{label}={bit_val}")
            print(f"{indent}primary input: {path}={value}  ({', '.join(parts)})")
            return

        cell = top.cells[inst]
        cluster_type = cell["type"]
        if inst not in cluster_models:
            cluster_models[inst] = ClusterModel(mods[cluster_type])
        model = cluster_models[inst]

        # `bits` are TOP-module-scoped bit IDs -- translate each into
        # this cluster's OWN bit numbering via (port, index), which is
        # the only thing stable across the module boundary (see
        # _cell_output_port_bit's own docstring for why raw bit IDs
        # can't be reused directly here).
        local_bits = []
        for b in bits:
            found = _cell_output_port_bit(cell, b)
            if found is None:
                raise RuntimeError(f"{inst}: bit {b} isn't one of its own output pins")
            port, idx = found
            local_bits.append(model.module.port_bits(port)[idx])

        result = model.solve_for_bits(local_bits, value)
        retried = False
        if result is None and inst in inst_targets:
            # Conflicts with something an EARLIER visit to this SAME
            # instance already pinned -- retry once, on a freshly
            # rebuilt model, before giving up: replay every earlier
            # target for this instance (in the order they were first
            # established -- see the module docstring's "remembers
            # fixed values from previous visits" note) as hard
            # constraints via pin_bits(), THEN solve for the current
            # target on top of them via the normal necessity-checked
            # solve_for_bits(). This can succeed where the original
            # model couldn't: solve_for_bits() only ever PERMANENTLY
            # pins a target's own NECESSARY free variables (see its own
            # docstring), so an earlier visit's incidental choices
            # never got baked in as hard facts in the first place --
            # replaying just the raw targets gives z3 full freedom to
            # find a genuinely joint witness, rather than being stuck
            # with whichever arbitrary values the ORIGINAL, now-stale
            # model happened to settle on.
            retry_model = ClusterModel(mods[cluster_type])
            for prior_bits, prior_value in inst_targets[inst]:
                retry_model.pin_bits(prior_bits, prior_value)
            retry_result = retry_model.solve_for_bits(local_bits, value)
            if retry_result is not None:
                model = retry_model
                cluster_models[inst] = retry_model
                result = retry_result
                retried = True

        if result is None:
            conflict = " -- CONFLICTS with an earlier visit to this same instance" if inst in inst_targets else ""
            print(f"{indent}{inst} ({cluster_type}): UNSAT for {path}={value}{conflict}"
                  + (" (retried with a fresh witness, still unsatisfiable)" if inst in inst_targets else ""))
            assumptions[f"{inst}: UNSAT ({path})"] = value
            return

        # Pin this solve's own result PERMANENTLY on `model` before doing
        # anything else with it -- see ClusterModel.pin()'s own docstring
        # for why: `inst` can be reached again later, from a DIFFERENT
        # downstream consumer, and without this, that later visit's own
        # solve_for_bits() call has no memory of what THIS visit already
        # required, and is free to pick a different, silently conflicting
        # value for this SAME physical instance's other free variables.
        model.pin(result)
        inst_targets.setdefault(inst, []).append((local_bits, value))

        inputs_str = ", ".join(f"{k}={v}" for k, v in sorted(result.items())) or "(no free variables)"
        if retried:
            print(f"{indent}{inst} ({cluster_type}): {path}={value} was UNSAT under the earlier witness -- "
                  f"retried with a fresh one, now consistent")
        print(f"{indent}{inst} ({cluster_type}): {path}={value}  <-  {inputs_str}")

        input_port_names = set(model.module.input_ports())
        for name, v in result.items():
            if name in input_port_names:
                next_bits = cell["connections"][name]
                visit(next_bits, v, depth + 1, f"{inst}.{name}")
            else:
                # not simply one of this cluster's own input ports -- a
                # purely internal free variable (e.g. a DIFFERENT
                # flip-flop's own Q living inside this SAME cluster) this
                # driver doesn't auto-resolve further -- see the
                # docstring above for the manual next step.
                akey = f"{inst}.{name}"
                if akey not in assumptions:
                    print(f"{indent}  (unresolved, not an input port of {cluster_type} -- "
                          f"use ClusterModel.solve_for_pin on {inst!r} to go further)")
                assumptions[akey] = v

    if target_bit_index is not None:
        target_bits = [top.port_bits(target_port)[target_bit_index]]
        label = f"{target_port}[{target_bit_index}]"
    else:
        target_bits = top.port_bits(target_port)
        label = target_port
    print(f"backtrace_solve: {top_module}.{label} = {target_value}")
    visit(target_bits, target_value, 0, label)
    return assumptions


if __name__ == "__main__":
    import sys

    args = sys.argv[1:]

    if len(args) >= 4 and args[2] == "--chip":
        # python cluster_solve.py <verilog_file> <top_module> --chip <port>[=value] [max_depth]
        verilog_file, top_module_name = args[0], args[1]
        spec = args[3]
        if "=" not in spec:
            print("usage: python cluster_solve.py <verilog_file> <top_module> --chip <port>=<value> [max_depth]")
            raise SystemExit(1)
        port, value_str = spec.split("=", 1)
        value = int(value_str, 0)
        max_depth = int(args[4]) if len(args) > 4 else 12
        assumptions = backtrace_solve(verilog_file, top_module_name, port, value, max_depth=max_depth)
        print(f"\n{len(assumptions)} assumption(s):")
        for k, v in sorted(assumptions.items()):
            print(f"  {k} = {v}")
        raise SystemExit(0)

    any_witness = "--any" in args
    args = [a for a in args if a != "--any"]

    if len(args) < 4:
        print("usage:")
        print("  python cluster_solve.py <verilog_file> <top_module> <cluster_module> "
              "<output_port>[=value] [<output_port>[=value] ...] [--any]")
        print("  python cluster_solve.py <verilog_file> <top_module> --chip <output_port>=<value> [max_depth]")
        print("  e.g.: python cluster_solve.py puzzle.v puzzle cluster_type_10 success=1")
        print("  e.g.: python cluster_solve.py puzzle.v puzzle --chip success=1")
        print("  --any: report ANY one valid witness instead of only what's strictly necessary "
              "(see ClusterModel.solve_for_bits' own necessary_only note)")
        raise SystemExit(1)

    verilog_file, top_module_name, cluster_module_name = args[0:3]
    model = load_cluster(verilog_file, top_module_name, cluster_module_name)
    if model.skipped:
        print(f"ignoring {len(model.skipped)} flip-flop/submodule instance(s): {model.skipped}")

    for spec in args[3:]:
        if "=" not in spec:
            print(f"skipping {spec!r} -- expected PORT=VALUE (VALUE an int, 0x../0b.. ok)")
            continue
        port, value_str = spec.split("=", 1)
        value = int(value_str, 0)
        result = model.solve_for_output(port, value, necessary_only=not any_witness)
        print(f"\n{cluster_module_name}.{port} = {value}:")
        if result is None:
            print("  UNSAT -- no assignment of this cluster's own free variables achieves this")
            continue
        for name, val in sorted(result.items()):
            print(f"  {name} = {val}")
