"""cluster0.py: concrete cycle-by-cycle simulation of cluster_type_0.

    module cluster_type_0 (
        input A_40, input A_41, input A_42, input A_43, input A_8, input I,
        input clk, input rst_n,
        output [10:0] A_48_bus
    );
        ... 22 dfrtp_2 registers, ~40 combinational intermediates ...
    endmodule

A_40/A_41/A_42/A_43 are cluster_type_9's own current register outputs
(its "low digit", see cluster9.py) and A_48_bus is exactly what feeds
cluster_type_3's A_48_bus input (the AND-of-all-11-bits gate for
A_23_bus[1] -- see cluster3.py). So this module's real job: watch
cluster9's counter go by (while A_8=I=1) and decide, bit by bit, whether
A_48_bus should read all-1s.

22 registers (2**22 states) is too large to hand-transcribe safely (the
cluster2.py problem, just with real sequential state added on top), so
this reuses cluster2.py's approach: parse the ACTUAL `assign` text and
dfrtp wiring straight out of the module below and evaluate it, rather
than retyping ~60 boolean expressions into Python by hand.

Every A_48_bus[i] assign (see OUTPUT_ASSIGNS below) turns out to depend
ONLY on this module's own registers -- none of them read A_40..A_43/A_8/I
directly -- so a48_bus() can be computed from `state` alone, no other
inputs needed.
"""

import re

SOURCE = """
assign A_48_bus[0] = A1_24 & ~A_N_4;
assign A_48_bus[1] = A1_27 & ~A_N_1;
assign A_48_bus[2] = A_10 & ~A_12;
assign A_48_bus[3] = ~A_47 & A_49;
assign A_48_bus[4] = A_37 & ~A_52;
assign A_48_bus[5] = A1_31 & ~A_N_7;
assign A_48_bus[6] = A1_25 & ~A_N_3;
assign A_48_bus[7] = A1_26 & ~A_N_5;
assign A_48_bus[8] = A1_28 & ~A_N_6;
assign A_48_bus[9] = A1_23 & ~A_N_2;
assign A_48_bus[10] = A_44 & ~A_53;
assign D_12 = (~A_40 & ~A_41 & A_42 & ~A_43 & A_52 & A_8 & I) | A_37;
assign D_15 = (~A_40 & ~A_41 & ~A_42 & A_43 & A_53 & A_8 & I) | A_44;
assign D_22 = (A_40 & A_41 & ~A_42 & ~A_43 & A_8 & A_N_4 & I) | A1_24;
assign D_21 = (~A_40 & A_41 & ~A_42 & ~A_43 & A_47 & A_8 & I) | A_49;
assign D_23 = (~A_40 & A_41 & ~A_42 & A_43 & A_8 & A_N_5 & I) | A1_26;
assign D_24 = (A_40 & ~A_41 & A_42 & A_43 & A_8 & A_N_3 & I) | A1_25;
assign D_30 = (A_40 & ~A_41 & A_42 & ~A_43 & A_8 & A_N_1 & I) | A1_27;
assign D_14 = (~A_40 & ~A_41 & A_42 & A_43 & A_8 & A_N_2 & I) | A1_23;
assign D_27 = (A_40 & ~A_41 & ~A_42 & A_43 & A_8 & A_N_6 & I) | A1_28;
assign D_1 = (A_12 & A_40 & ~A_41 & ~A_42 & ~A_43 & A_8 & I) | A_10;
assign D_34 = (~A_40 & ~A_41 & ~A_42 & ~A_43 & A_8 & A_N_7 & I) | A1_31;
assign B1_2 = B_3 | A_11 | B_2 | A_10;
assign D_2 = (A2_1 & B1_2) | (A_12 & B1_2);
assign B_19 = ~I | ~A_8;
assign B1_16 = (I & A_8 & A3_11) | A_N_1;
assign B1_17 = (I & A_8 & A3_12) | A_N_2;
assign D_13 = (A2_20 & B1_17) | (A1_23 & B1_17);
assign B1_18 = B_19 | A_39 | B_20 | A_37;
assign A3_12 = ~A_40 & ~A_41 & A_42 & A_43;
assign A3_13 = ~A_41 & A_42 & A_40 & A_43;
assign B1_19 = B_22 | A_45 | B_21 | A_44;
assign B_22 = ~I | ~A_8;
assign B1_20 = (I & A_8 & A3_14) | A_N_4;
assign B1_21 = (I & A_8 & A3_15) | A_N_5;
assign A3_15 = ~A_40 & ~A_42 & A_41 & A_43;
assign B_23 = ~I | ~A_8;
assign B1_22 = B_23 | A_50 | B_24 | A_49;
assign A2_21 = ~I | ~A_8 | ~A_N_5 | ~A3_15;
assign A2_22 = ~I | ~A_8 | ~A_N_4 | ~A3_14;
assign D_17 = (A2_22 & B1_20) | (A1_24 & B1_20);
assign A3_14 = ~A_43 & ~A_42 & A_41 & A_40;
assign A2_23 = ~A_50 & ~B_23;
assign A2_24 = ~I | ~A_8 | ~A_N_3 | ~A3_13;
assign D_19 = (A2_23 & B1_22) | (A_47 & B1_22);
assign B_24 = ~A_47;
assign A_50 = ~A_41 | A_42 | A_40 | A_43;
assign D_18 = (A2_21 & B1_21) | (A1_26 & B1_21);
assign D_16 = (A2_24 & B1_23) | (A1_25 & B1_23);
assign B1_23 = (I & A_8 & A3_13) | A_N_3;
assign A2_20 = ~I | ~A_8 | ~A_N_2 | ~A3_12;
assign A_45 = ~A_43 | A_42 | A_41 | A_40;
assign A2_25 = ~I | ~A_8 | ~A_N_1 | ~A3_11;
assign D_20 = (A2_27 & B1_24) | (A1_28 & B1_24);
assign D_26 = (A2_26 & B1_18) | (A_52 & B1_18);
assign A3_16 = ~A_41 & ~A_42 & A_40 & A_43;
assign D_11 = (A2_25 & B1_16) | (A1_27 & B1_16);
assign D_29 = (A2_28 & B1_19) | (A_53 & B1_19);
assign A2_26 = ~A_39 & ~B_19;
assign A_39 = ~A_42 | A_41 | A_40 | A_43;
assign A3_11 = ~A_43 & ~A_41 & A_42 & A_40;
assign B_20 = ~A_52;
assign B_21 = ~A_53;
assign B_2 = ~A_12;
assign A2_27 = ~I | ~A_8 | ~A_N_6 | ~A3_16;
assign B1_24 = (I & A_8 & A3_16) | A_N_6;
assign A2_28 = ~A_45 & ~B_22;
assign A2_1 = ~A_11 & ~B_3;
assign B1_25 = (I & A_8 & A3_17) | A_N_7;
assign D_31 = (A2_31 & B1_25) | (A1_31 & B1_25);
assign A3_17 = ~A_43 & ~A_40 & ~A_41 & ~A_42;
assign A2_31 = ~I | ~A_8 | ~A_N_7 | ~A3_17;
assign B_3 = ~I | ~A_8;
assign A_11 = ~A_40 | A_42 | A_41 | A_43;
"""

# (D-signal, Q-signal) for each dfrtp_2, straight from the instance list
REG_WIRING = [
    ("D_11", "A_N_1"), ("D_12", "A_37"), ("D_13", "A_N_2"), ("D_15", "A_44"),
    ("D_2", "A_12"), ("D_16", "A_N_3"), ("D_17", "A_N_4"), ("D_18", "A_N_5"),
    ("D_19", "A_47"), ("D_20", "A_N_6"), ("D_22", "A1_24"), ("D_21", "A_49"),
    ("D_23", "A1_26"), ("D_24", "A1_25"), ("D_30", "A1_27"), ("D_26", "A_52"),
    ("D_14", "A1_23"), ("D_27", "A1_28"), ("D_29", "A_53"), ("D_1", "A_10"),
    ("D_31", "A_N_7"), ("D_34", "A1_31"),
]
REG_BITS = [q for _, q in REG_WIRING]  # bit i = REG_BITS[i]'s own Q, in dfrtp instance order

_ASSIGN_RE = re.compile(r"assign\s+([\w\[\]]+)\s*=\s*(.+?);")
_TOKEN_RE = re.compile(r"[A-Za-z_][A-Za-z0-9_]*(?:\[\d+\])?")
ASSIGNS = {lhs: rhs for lhs, rhs in _ASSIGN_RE.findall(SOURCE)}


class Bit(int):
    """0/1 int with LOGICAL (not bitwise) inversion, so `~` matches
    Verilog's meaning on a single-bit signal."""
    def __invert__(self):
        return Bit(1 - int(self))

    def __and__(self, other):
        return Bit(int(self) & int(other))

    def __or__(self, other):
        return Bit(int(self) | int(other))


def _sanitize(tok):
    return tok.replace("[", "_").replace("]", "")


def evaluate(inputs):
    """inputs: {name: 0/1} for every primary input AND every register's
    current Q this module's combinational logic reads (A_40, A_41,
    A_42, A_43, A_8, I, plus every REG_BITS entry). Returns {name: 0/1}
    for every `assign` target, resolved lazily/recursively and memoized
    -- same approach as cluster2.py's evaluate()."""
    memo = {}

    def get(name):
        if name in memo:
            return memo[name]
        if name in inputs:
            v = Bit(inputs[name])
        elif name in ASSIGNS:
            v = eval_rhs(ASSIGNS[name])
        else:
            raise KeyError(f"unknown signal {name!r}")
        memo[name] = v
        return v

    def eval_rhs(expr):
        ns = {_sanitize(tok): get(tok) for tok in set(_TOKEN_RE.findall(expr))}
        py_expr = _TOKEN_RE.sub(lambda m: _sanitize(m.group(0)), expr)
        return Bit(eval(py_expr, {"__builtins__": {}}, ns))

    for name in ASSIGNS:
        get(name)
    return {k: int(v) for k, v in memo.items()}


def next_state(state, A_40, A_41, A_42, A_43, A_8, I):
    """state: bitvector value of REG_BITS (bit i = REG_BITS[i]'s current
    Q -- this module's own registers, read here purely as D-rule
    inputs, exactly like every other cluster*.py file). Returns
    next_state (same packing)."""
    inputs = {"A_40": A_40, "A_41": A_41, "A_42": A_42, "A_43": A_43, "A_8": A_8, "I": I}
    for i, name in enumerate(REG_BITS):
        inputs[name] = (state >> i) & 1
    result = evaluate(inputs)
    next_bits = [result[d] for d, q in REG_WIRING]
    return sum(b << i for i, b in enumerate(next_bits))


def reset_state():
    return 0


def a48_bus(state):
    """A_48_bus as an 11-bit int (bit i = A_48_bus[i]) -- computed
    purely from `state`. evaluate() always resolves every `assign` in
    SOURCE (including plenty of D_xx targets that genuinely DO need
    A_40..A_43/A_8/I), so those are still supplied here -- but as
    placeholder 0s, which is fine: each A_48_bus[i]'s own assign reads
    only A1_xx/A_N_x names (see SOURCE), themselves already fully
    determined by `state` alone, so whatever the D_xx side-computations
    come out to under the placeholders is simply never read back out."""
    inputs = {name: (state >> i) & 1 for i, name in enumerate(REG_BITS)}
    inputs.update({"A_40": 0, "A_41": 0, "A_42": 0, "A_43": 0, "A_8": 0, "I": 0})
    result = evaluate(inputs)
    return sum(result[f"A_48_bus[{i}]"] << i for i in range(11))


def run_with_cluster9(cycles=44, I_value=1):
    """Co-simulates this module alongside a REAL cluster9.py counter:
    A_40/A_41/A_42/A_43 come from cluster9.next_state() each cycle (held
    enabled, A_8=1), I held at `I_value` throughout. Returns a list of
    (cycle, A_40..A_43 raw nibble, cluster9 count_value, A_48_bus) rows,
    and prints whenever A_48_bus reaches all-1s (0b111_1111_1111 = 2047
    -- the value cluster3.py's AND-of-all-11-bits gate needs)."""
    import cluster9

    state0 = reset_state()
    lo_state = cluster9.reset_state()
    rows = []
    for cycle in range(cycles):
        A_40 = (lo_state >> 0) & 1
        A_41 = (lo_state >> 1) & 1
        A_42 = (lo_state >> 2) & 1
        A_43 = (lo_state >> 3) & 1
        bus = a48_bus(state0)
        rows.append((cycle, lo_state, cluster9.count_value(lo_state), bus))
        state0 = next_state(state0, A_40, A_41, A_42, A_43, 1, I_value)
        lo_state, _ = cluster9.next_state(lo_state, 1)
    return rows


def run_i3_via_cluster2(cycles=30, A_8=1):
    """Co-simulates the OTHER cluster_type_0 instance in puzzle.v --
    cluster_type_0_i3, per the top module's own wiring:

        cluster_type_2 cluster_type_2_i1 (.A_40(A_40), .A_41(A_41),
            .A_42(A_42), .A_43(A_43), .A_94_bus(A_94_bus),
            .A_60_bus({B_31, A_N_10, A_61, A_60}));
        cluster_type_0 cluster_type_0_i3 (.A_40(A_60), .A_41(A_N_10),
            .A_42(B_31), .A_43(A_61), .A_8(A_8), .I(I), .clk(clk),
            .rst_n(rst_n), .A_48_bus(A_100_bus));

    i.e. i3's own A_40..A_43 inputs are NOT cluster9's counter directly
    (that's cluster_type_0_i2 -- see run_with_cluster9()) -- they're
    cluster_type_2's own A_60_bus OUTPUT, remapped bit-for-bit:
        i3.A_40 = A_60_bus[0]   i3.A_41 = A_60_bus[2]
        i3.A_42 = A_60_bus[3]   i3.A_43 = A_60_bus[1]
    and A_60_bus is itself cluster2.compute() of cluster9's AND
    cluster8's CURRENT states each cycle (both counters share A_8; only
    cluster8 also needs A_75 -- see cluster9.py/cluster8.py).

    Since `I` genuinely branches i3's own next state (unlike A_8, which
    only gates it), this tracks the full REACHABLE SET of i3 register
    states each cycle -- deduplicated, same approach as
    cluster5.py/cluster7.py's own trellis functions -- rather than
    picking one fixed I value, so "simulate I=0 and I=1" means both
    branches are kept alive every cycle, not just one path.

    Returns a list of (cycle, A_40..A_43 fed to i3, reachable i3 states,
    whether ANY of them has A_100_bus all-ones) per cycle."""
    import cluster2
    import cluster8
    import cluster9

    lo_state = cluster9.reset_state()
    hi_state = cluster8.reset_state()
    i3_states = {reset_state()}
    rows = []
    for cycle in range(cycles):
        A_40 = (lo_state >> 0) & 1
        A_41 = (lo_state >> 1) & 1
        A_42 = (lo_state >> 2) & 1
        A_43 = (lo_state >> 3) & 1
        lo_next, A_75 = cluster9.next_state(lo_state, A_8)  # A_75 read off the CURRENT state, same cycle

        a60_bus = cluster2.compute(A_40, A_41, A_42, A_43, hi_state)
        A_60 = (a60_bus >> 0) & 1
        A_61 = (a60_bus >> 1) & 1
        A_N_10 = (a60_bus >> 2) & 1
        B_31 = (a60_bus >> 3) & 1
        i3_A_40, i3_A_41, i3_A_42, i3_A_43 = A_60, A_N_10, B_31, A_61

        any_all_ones = any(a48_bus(s) == 0b111_1111_1111 for s in i3_states)
        rows.append((cycle, (i3_A_40, i3_A_41, i3_A_42, i3_A_43), set(i3_states), any_all_ones))

        next_i3_states = set()
        for s in i3_states:
            for I in (0, 1):
                ns = next_state(s, i3_A_40, i3_A_41, i3_A_42, i3_A_43, A_8, I)
                next_i3_states.add(ns)
        i3_states = next_i3_states

        hi_state, _ = cluster8.next_state(hi_state, A_75, A_8)
        lo_state = lo_next
    return rows


# (gate, match, trigger_nibble, A_48_bus bit) -- one entry per one of the
# 11 pattern-matched register pairs found by inspecting SOURCE (every
# A_48_bus[i] output traces to exactly one gate/match pair, and every
# gate/match D-rule only ever references that SAME pair, the current
# nibble, and I/A_8 -- see verify_pair_independence()). This is what
# makes the 22-register joint state fake complexity: it factors into 11
# independent 2-bit subsystems, each of which only changes on the rare
# cycles its own trigger_nibble is actually presented.
PAIRS = [
    ("A_N_1", "A1_27", 5, 1), ("A_N_2", "A1_23", 12, 9), ("A_N_3", "A1_25", 13, 6),
    ("A_N_4", "A1_24", 3, 0), ("A_N_5", "A1_26", 10, 7), ("A_N_6", "A1_28", 9, 8),
    ("A_N_7", "A1_31", 0, 5), ("A_52", "A_37", 4, 4), ("A_53", "A_44", 8, 10),
    ("A_47", "A_49", 2, 3), ("A_12", "A_10", 1, 2),
]


def verify_pair_independence(samples=50):
    """Randomized check (not a proof, but a strong empirical one) that
    each PAIRS entry's own next-state depends ONLY on its own 2 bits,
    the fed nibble, and I/A_8 -- built by embedding the pair's 2 bits
    into two otherwise-RANDOM 22-bit backgrounds and confirming
    next_state() extracts the SAME pair result from both -- and that it
    genuinely holds whenever the nibble isn't its own trigger pattern.
    Returns (independence_violations, hold_violations); both 0 is what
    run_i3_decomposed() relies on."""
    import random

    idx = {name: i for i, name in enumerate(REG_BITS)}
    rng = random.Random(0)
    independence_violations = 0
    hold_violations = 0
    for gate, match, pattern, _ in PAIRS:
        gi, mi = idx[gate], idx[match]
        for _ in range(samples):
            gv, mv = rng.randint(0, 1), rng.randint(0, 1)
            bg_a, bg_b = rng.getrandbits(22), rng.getrandbits(22)

            def build(bg):
                return (bg & ~(1 << gi) & ~(1 << mi)) | (gv << gi) | (mv << mi)

            for A_8 in (0, 1):
                for I in (0, 1):
                    for nib in (pattern, (pattern + 1) % 16):
                        a40, a41, a42, a43 = (nib >> 0) & 1, (nib >> 1) & 1, (nib >> 2) & 1, (nib >> 3) & 1
                        ns_a = next_state(build(bg_a), a40, a41, a42, a43, A_8, I)
                        ns_b = next_state(build(bg_b), a40, a41, a42, a43, A_8, I)
                        pair_a = ((ns_a >> gi) & 1, (ns_a >> mi) & 1)
                        pair_b = ((ns_b >> gi) & 1, (ns_b >> mi) & 1)
                        if pair_a != pair_b:
                            independence_violations += 1
                        if nib != pattern and pair_a != (gv, mv):
                            hold_violations += 1
    return independence_violations, hold_violations


def _pair_next(gate_idx, match_idx, gate_val, match_val, nibble, A_8, I):
    """next_state() called on a throwaway 22-bit state with ONLY this
    pair's 2 bits set (everything else 0) -- safe per
    verify_pair_independence(), and reuses the SAME verified D-rule
    evaluator rather than re-deriving each pair's boolean formula by
    hand (the two families of pairs -- A_N_x/A1_xx vs A_52/A_47/A_12 --
    use genuinely different algebraic shapes, so hand-unifying them
    would be a real place to introduce a bug)."""
    a40, a41, a42, a43 = (nibble >> 0) & 1, (nibble >> 1) & 1, (nibble >> 2) & 1, (nibble >> 3) & 1
    dummy = (gate_val << gate_idx) | (match_val << match_idx)
    ns = next_state(dummy, a40, a41, a42, a43, A_8, I)
    return (ns >> gate_idx) & 1, (ns >> match_idx) & 1


def run_i3_decomposed(cycles, A_8=1):
    """Same simulation as run_i3_via_cluster2(), but tracking 11
    independent small reachable-state sets (one per PAIRS entry, each
    holding at most 4 (gate,match) tuples) instead of one exploding
    22-bit joint set -- practical for arbitrarily long horizons, unlike
    run_i3_via_cluster2()'s ~49s-for-30-cycles blowup.

    Returns a list of (cycle, nibble, all_11_pairs_can_win,
    per_pair_reachable_set_sizes)."""
    import cluster2
    import cluster8
    import cluster9

    idx = {name: i for i, name in enumerate(REG_BITS)}
    pairs = [(idx[g], idx[m], pat, bit) for g, m, pat, bit in PAIRS]
    pair_states = [{(0, 0)} for _ in pairs]  # (gate,match) reachable tuples

    lo_state = cluster9.reset_state()
    hi_state = cluster8.reset_state()
    rows = []
    for cycle in range(cycles):
        A_40 = (lo_state >> 0) & 1
        A_41 = (lo_state >> 1) & 1
        A_42 = (lo_state >> 2) & 1
        A_43 = (lo_state >> 3) & 1
        lo_next, A_75 = cluster9.next_state(lo_state, A_8)

        a60_bus = cluster2.compute(A_40, A_41, A_42, A_43, hi_state)
        A_60 = (a60_bus >> 0) & 1
        A_61 = (a60_bus >> 1) & 1
        A_N_10 = (a60_bus >> 2) & 1
        B_31 = (a60_bus >> 3) & 1
        nibble = A_60 | (A_N_10 << 1) | (B_31 << 2) | (A_61 << 3)

        winners = []
        for k, (gi, mi, pattern, bit) in enumerate(pairs):
            if nibble == pattern:
                nxt = set()
                for gv, mv in pair_states[k]:
                    for I in (0, 1):
                        nxt.add(_pair_next(gi, mi, gv, mv, nibble, A_8, I))
                pair_states[k] = nxt
            winners.append((0, 1) in pair_states[k])  # gate=0, match=1 -> A_48_bus[bit]=1

        rows.append((cycle, nibble, all(winners), [len(s) for s in pair_states]))

        hi_state, _ = cluster8.next_state(hi_state, A_75, A_8)
        lo_state = lo_next
    return rows


def run_i3_I_always_1(cycles, A_8=1):
    """Since I=0 NEVER changes any pair's state (verified: 0 violations
    across all 11 pairs x all 16 nibbles), fixing I=1 for every cycle
    makes this fully deterministic -- one exact (gate,match) value per
    pair, no reachable SETS needed at all. Each pair just advances its
    own 0->1->2->3 sequence (value = gate + 2*match) by exactly 1 every
    time its own trigger nibble is presented, absorbing at 3 -- so its
    fate is purely "how many times has my nibble shown up so far",
    nothing else.

    Returns a list of (cycle, nibble, [(gate,match) per pair], all_win)."""
    import cluster2
    import cluster8
    import cluster9

    idx = {name: i for i, name in enumerate(REG_BITS)}
    pairs = [(idx[g], idx[m], pat, bit) for g, m, pat, bit in PAIRS]
    pair_vals = [(0, 0)] * len(pairs)

    lo_state = cluster9.reset_state()
    hi_state = cluster8.reset_state()
    rows = []
    for cycle in range(cycles):
        A_40 = (lo_state >> 0) & 1
        A_41 = (lo_state >> 1) & 1
        A_42 = (lo_state >> 2) & 1
        A_43 = (lo_state >> 3) & 1
        lo_next, A_75 = cluster9.next_state(lo_state, A_8)

        a60_bus = cluster2.compute(A_40, A_41, A_42, A_43, hi_state)
        A_60 = (a60_bus >> 0) & 1
        A_61 = (a60_bus >> 1) & 1
        A_N_10 = (a60_bus >> 2) & 1
        B_31 = (a60_bus >> 3) & 1
        nibble = A_60 | (A_N_10 << 1) | (B_31 << 2) | (A_61 << 3)

        for k, (gi, mi, pattern, bit) in enumerate(pairs):
            if nibble == pattern:
                gv, mv = pair_vals[k]
                pair_vals[k] = _pair_next(gi, mi, gv, mv, nibble, A_8, 1)

        all_win = all(v == (0, 1) for v in pair_vals)
        rows.append((cycle, nibble, list(pair_vals), all_win))

        hi_state, _ = cluster8.next_state(hi_state, A_75, A_8)
        lo_state = lo_next
    return rows


if __name__ == "__main__":
    print(f"parsed {len(ASSIGNS)} assign statements, {len(REG_WIRING)} registers")

    rows = run_with_cluster9(cycles=44, I_value=1)
    print(f"{'cycle':>5} {'cluster9 count':>14} | A_48_bus")
    for cycle, lo_state, count, bus in rows:
        marker = "  <-- ALL 11 BITS SET" if bus == 0b111_1111_1111 else ""
        print(f"{cycle:>5} {count:>14} | {bus:011b}{marker}")

    print()
    print("=== cluster_type_0_i3 (fed by cluster2's A_60_bus output, I branching both ways) ===")
    i3_rows = run_i3_via_cluster2(cycles=30)
    print(f"{'cycle':>5} {'i3 A_40..43':>12} {'#reachable states':>18}  any-all-ones?")
    for cycle, (a40, a41, a42, a43), states, any_all_ones in i3_rows:
        print(f"{cycle:>5}   {a40}{a41}{a42}{a43}        {len(states):>18}  {any_all_ones}")
