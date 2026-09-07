"""cluster2.py: simulate cluster_type_2 -- purely combinational (no
dfrtp instances, no clk/rst_n at all), but with ~150 intermediate wires
chaining through each other. Hand-transcribing that many `assign`
statements into Python (the approach cluster3/5/8/9/10/11/12 all use)
would be extremely error-prone at this size, so this file instead
evaluates the ACTUAL Verilog `assign` text directly: a tiny expression
evaluator (only `~`, `&`, `|`, `(`, `)`, identifiers, and bus indices
`name[N]` ever appear in this project's z3-flattened output -- no `^`,
confirmed by inspection) that recursively resolves each signal's own
`assign` right-hand side, memoized, straight off the module text below
-- so there's no possibility of a transcription typo silently producing
a wrong function.

    module cluster_type_2 (
        input A_40, input A_41, input A_42, input A_43,
        input [3:0] A_94_bus,
        output [3:0] A_60_bus
    );
        ... ~150 `assign` statements ...
    endmodule

Inputs: A_40/A_41/A_42/A_43 are cluster_type_9's own current register
outputs (its "low digit"), A_94_bus is cluster_type_8's own current
register outputs (its "high digit") -- see cluster9.py/cluster8.py. This
module is purely combinational, so there's no state to step through
cycle-by-cycle here: just one function of those 8 input bits, evaluated
for every one of the 2**8=256 possible combinations to characterize what
A_60_bus actually computes.
"""

import re

SOURCE = """
assign B_56 = ~A_102 & ~A_103;
assign B1_49 = ~A_104 & ~A2_46;
assign A1_47 = ~A_105 & ~A_106;
assign A_108 = (~A_107 & ~B_57) | (A_107 & B_57);
assign A_110 = (~A_109 & A2_47) | (A_109 & ~A2_47);
assign B_58 = (A0_14 & A_108 & C1_8) | (A1_48 & A_108 & C1_8);
assign A1_49 = (A2_48 & A_108 & C1_9) | (A_110 & A_108 & C1_9) | (A1_47 & A_108 & C1_9);
assign B_59 = (~A_42 & A_94_bus[3]) | (~A2_N & A_94_bus[3]) | (~A_42 & B1_50) | (~A2_N & B1_50);
assign A2_49 = (~A_94_bus[0] & A_94_bus[3]) | (A_94_bus[0] & ~A_94_bus[3]);
assign A_109 = (A_111 & A2_50) | B1_51;
assign A2_51 = ~A_94_bus[2] | ~A2_49;
assign B2_22 = (A_94_bus[1] & A_94_bus[3] & A_94_bus[2]) | A2_52;
assign A2_N = (~A_94_bus[1] & ~B1_52) | (~A2_49 & ~B1_52);
assign A_112 = ~A_94_bus[0] | ~A_94_bus[3];
assign A_113 = ~A_94_bus[1] & ~A2_51;
assign B1_53 = (A0_14 & ~A2_53) | (A_106 & A2_53);
assign B_60 = ~A_114;
assign A2_54 = (~A_106 & ~A1_50) | (~B1_54 & ~A_115);
assign A_106 = (~A_111 & ~B_61) | (A_111 & B_61);
assign B1_N = (A_106 & A2_55) | A_110 | B1_55;
assign A2_59 = (~A_102 & ~A2_58) | ~A_110;
assign A_60_bus[2] = (A2_60 & B1_57 & B1_58) | (A1_54 & B1_57 & B1_58);
assign B1_60 = (B1_59 & B2_23) | (A_110 & A2_54) | A_117;
assign B_61 = (~A_118 & ~B_59) | (A_118 & B_59);
assign A_117 = (~A_119 & ~B_63) | (A_119 & B_63);
assign A_115 = (~A_107 & B_57) | (A_107 & ~B_57);
assign A1_54 = (~A_119 & B_63) | (A_119 & ~B_63);
assign C1_10 = C_11 | B_58 | A_117;
assign A_60_bus[3] = (A2_61 & B1_60 & B1_58) | (A1_54 & B1_60 & B1_58);
assign A2_47 = ~A_122 & ~B_65;
assign A2_50 = ~A_118 | ~B_59;
assign A_107 = (A_109 & A2_47) | A_122;
assign B1_64 = (A_110 & A2_64) | A_108 | B1_53;
assign A2_65 = (~A_94_bus[1] & ~A2_51) | (~B1_65 & ~B2_22);
assign B_63 = (A_123 & A2_52) | B1_66;
assign B_65 = A_124 & A2_66 & B1_67;
assign B1_67 = C_12 | A2_52 | A_113;
assign B1_58 = ~A_125 & ~B_66;
assign B1_68 = A_108 & B_67 & C_13;
assign A_119 = (~A_107 & ~A_N_15) | (~A2_65 & ~A_N_15);
assign B1_51 = ~A_118 & ~B_59;
assign B_57 = ~A_N_15 & A2_65;
assign C_12 = A_112 & A2_51 & B_68;
assign A_125 = ~A_119 & ~B_63;
assign A2_52 = ~A_112 & ~B_68;
assign A_123 = ~A_94_bus[1] | ~A_94_bus[3];
assign B1_66 = (~A_94_bus[2] & ~A2_52) | (~A_123 & ~A2_52);
assign B1_65 = (~A_94_bus[1] & ~A_94_bus[3]) | (~A_94_bus[2] & ~A_94_bus[3]);
assign A_122 = (~A_124 & ~B1_67) | (~A2_66 & ~B1_67);
assign A_60_bus[1] = (A2_67 & B1_69 & B1_58) | (A1_54 & B1_69 & B1_58);
assign A_118 = (~A_41 & ~B_69) | (A_41 & B_69);
assign B_69 = (~A_123 & ~B_70) | (A_123 & B_70);
assign B_68 = (~A_94_bus[1] & ~A_94_bus[2]) | (A_94_bus[1] & A_94_bus[2]);
assign B_70 = (~A_94_bus[2] & A2_49) | (A_94_bus[2] & ~A2_49);
assign A_124 = ~B_70 | A_123;
assign A2_66 = ~A_41 | ~B_69;
assign A2_61 = (~A_115 & ~A2_68) | ~B1_70;
assign A0_15 = B_71 | A_126;
assign C_11 = (~A1_48 & ~A_108) | (~A2_69 & ~A_108);
assign A2_67 = (A_115 & A2_70) | B1_68;
assign A2_71 = A_106 & B_60 & A3_32;
assign C_13 = (~A0_14 & ~A2_71) | ~A_110;
assign A2_60 = (A0_16 & ~A_115) | (A1_56 & A_115);
assign A2_73 = (A_115 & A2_72 & A3_33) | A1_54;
assign A2_74 = (A0_15 & ~A_110) | (A1_57 & A_110);
assign A_60_bus[0] = (A2_73 & B1_58 & C1_10) | (A1_49 & B1_58 & C1_10);
assign B1_69 = (~A_115 & ~A2_74) | ~B1_64 | ~A1_54;
assign A0_16 = (A3_34 & B1_71) | (A3_34 & B1_49) | (A2_48 & B1_71) | (A2_48 & B1_49) | (A_110 & B1_71) | (A_110 & B1_49);
assign B2_23 = (~A_115 & ~A_110) | (~A2_75 & ~A_110);
assign B1_59 = B_56 | A_115;
assign B1_57 = (A_115 & A2_76) | A_117 | B1_72;
assign A2_68 = (A3_35 & B2_24) | (A3_35 & B1_71) | (A2_48 & B2_24) | (A2_48 & B1_71) | (A_110 & B2_24) | (A_110 & B1_71);
assign A2_70 = (A_110 & A2_77) | ~B1_N;
assign A_139 = (~A_138 & A_N_20) | (A_138 & ~A_N_20);
assign B1_80 = (A2_53 & A_140) | (A2_53 & A3_40) | (A_102 & A_140) | (A_102 & A3_40);
assign C_14 = (A_139 & A_106 & A_105) | (A_141 & A_106 & A_105);
assign A_N_15 = ~A_94_bus[0] & ~A_94_bus[1] & A_94_bus[3] & A_94_bus[2];
assign A_142 = (~A_138 & ~A_N_20) | (A_138 & A_N_20);
assign A_102 = (~A_111 & B_61) | (A_111 & ~B_61);
assign A2_69 = C_14 | A0_14 | A_110;
assign A1_57 = (A_143 & A_106) | (A1_50 & A0_14);
assign A3_34 = ~A_106 & ~A0_17;
assign C1_9 = ~A_110 | ~B_78;
assign A0_14 = ~A_106 & ~A_104;
assign A_126 = ~A_102 & ~A2_64;
assign B1_54 = A_143 | A_102 | A_141;
assign A1_63 = A1_62 | B_79 | A_141;
assign A2_58 = ~A_143;
assign A2_46 = ~A_106 | ~A1_50;
assign A0_18 = (A2_46 & A2_85) | (A_141 & A2_85);
assign A3_40 = A2_64 | A_106;
assign A2_72 = (A_142 & A2_85) | ~B1_N_1;
assign A1_64 = (A_102 & A_103 & A3_32) | A2_86;
assign B_80 = (~A_141 & ~A_102 & ~A_139) | ~A_105;
assign A2_76 = (A_110 & A2_46 & A3_40) | (B1_80 & B1_N_1);
assign A2_53 = ~A2_64 | A2_86;
assign B_71 = (A_114 & A2_64) | (A_102 & A2_64);
assign B1_71 = (~A_106 & ~A1_62 & ~A2_58) | ~A_110;
assign B_78 = (A0_17 & ~A_102) | (A1_63 & A_102);
assign A2_75 = (A2_55 & ~A2_77) | (A_106 & ~A2_77);
assign A3_33 = ~A_110 | ~B_80;
assign A3_35 = (~A_106 & ~A0_17) | (~A2_86 & ~A0_17);
assign B1_81 = (A2_87 & A3_40) | (A_102 & A3_40);
assign A2_85 = B_79 | A_106;
assign B1_55 = (A_143 & A_102) | (A_141 & A_102);
assign C1_8 = A_126 | B_56 | A_110;
assign A_143 = B_79 | A_139;
assign A2_55 = ~A_138 | ~A_114;
assign A_144 = ~A_43 | ~A2_88;
assign B1_70 = B1_55 | A1_63 | A_108 | A_110;
assign B_66 = (A_94_bus[1] & A_94_bus[3] & A_94_bus[2]) | (A_94_bus[0] & A_94_bus[3] & A_94_bus[2]);
assign A3_41 = ~A_N_20 & B_81;
assign A_104 = ~A_142 & ~B_79;
assign B_81 = ~A_145 | ~B_82;
assign B_67 = (A_138 & A_110 & A3_41) | A2_71 | A0_14;
assign B1_N_1 = (~A_102 & ~A_110) | (~A_104 & ~A_110);
assign A2_87 = A_103 & A3_32;
assign A3_42 = (~A_42 & A2_N) | (A_42 & ~A2_N);
assign A2_88 = (~A_94_bus[0] & A_94_bus[1]) | (A_94_bus[0] & ~A_94_bus[1]);
assign A_111 = (A_43 & A2_88 & A3_42) | (A_138 & A_N_20);
assign A_145 = (~A_43 & ~A2_88) | (A_43 & A2_88);
assign A1_56 = (A0_18 & ~A_110) | (A1_64 & A_110);
assign B1_72 = (A2_59 & A_110 & A_108) | (A2_59 & B1_81 & A_108) | (A1_62 & A_110 & A_108) | (A1_62 & B1_81 & A_108);
assign A_138 = (~A_144 & ~A3_42) | (A_144 & A3_42);
assign A_146 = A_40 & A_94_bus[0];
assign A2_48 = A_106 & A_142 & A2_86;
assign A_114 = ~A_147 & ~B_81;
assign A_105 = ~A_138 | ~A_141;
assign A_141 = ~A_145 & ~A_148;
assign A1_50 = ~A_142 | ~B_79;
assign A_148 = A_147 | A_146;
assign B_82 = ~A_40 | ~A_94_bus[0];
assign A0_17 = B_79 | A_142 | A_141;
assign A2_64 = A3_41 | A_139;
assign A1_48 = ~A_110 | ~A2_46;
assign A2_86 = A_148 & A3_41;
assign B1_50 = ~A_94_bus[0] | ~A_94_bus[1];
assign A_147 = ~A_40 & ~A_94_bus[0];
assign B1_52 = ~A_94_bus[1] & ~A_94_bus[3];
assign A_103 = ~A_138 | ~A3_41;
assign A2_77 = ~A_105 & ~A_102;
assign A_140 = ~A_146 & ~A_147;
assign B2_24 = ~A_102 & ~A_104;
assign A3_32 = ~A_148 | ~A_142;
assign A1_62 = ~A_142 & ~A_114;
assign B_79 = ~A_140 & ~A3_41;
assign A_N_20 = ~A_145 & ~B_82;
"""

_ASSIGN_RE = re.compile(r"assign\s+([\w\[\]]+)\s*=\s*(.+?);")
_TOKEN_RE = re.compile(r"[A-Za-z_][A-Za-z0-9_]*(?:\[\d+\])?")

ASSIGNS = {lhs: rhs for lhs, rhs in _ASSIGN_RE.findall(SOURCE)}
INPUT_NAMES = ["A_40", "A_41", "A_42", "A_43",
               "A_94_bus[0]", "A_94_bus[1]", "A_94_bus[2]", "A_94_bus[3]"]
OUTPUT_NAMES = ["A_60_bus[0]", "A_60_bus[1]", "A_60_bus[2]", "A_60_bus[3]"]


class Bit(int):
    """0/1 int with LOGICAL (not bitwise) inversion, so `~` matches
    Verilog's meaning on a single-bit signal instead of Python's
    bitwise-complement-on-a-multi-bit-int behavior."""
    def __invert__(self):
        return Bit(1 - int(self))

    def __and__(self, other):
        return Bit(int(self) & int(other))

    def __or__(self, other):
        return Bit(int(self) | int(other))


def _sanitize(tok):
    return tok.replace("[", "_").replace("]", "")


def evaluate(inputs):
    """inputs: {name: 0/1} for every entry in INPUT_NAMES. Returns
    {name: 0/1} for every `assign` target in the module (all ~150 of
    them, including the 4 A_60_bus[i] outputs) -- computed by lazily,
    recursively resolving each signal's own ASSIGNS entry, memoized, so
    each signal is only ever evaluated once regardless of fan-out."""
    memo = {}

    def get(name):
        if name in memo:
            return memo[name]
        if name in inputs:
            v = Bit(inputs[name])
        elif name in ASSIGNS:
            # no cycle guard needed: this project's own z3-flattened
            # output is always acyclic combinational logic, so a genuine
            # cycle here would just hit Python's own RecursionError
            # rather than silently miscomputing anything.
            v = eval_rhs(ASSIGNS[name])
        else:
            raise KeyError(f"unknown signal {name!r} -- not a declared input and no assign for it")
        memo[name] = v
        return v

    def eval_rhs(expr):
        ns = {_sanitize(tok): get(tok) for tok in set(_TOKEN_RE.findall(expr))}
        py_expr = _TOKEN_RE.sub(lambda m: _sanitize(m.group(0)), expr)
        return Bit(eval(py_expr, {"__builtins__": {}}, ns))

    for name in ASSIGNS:
        get(name)
    return {k: int(v) for k, v in memo.items()}


def compute(A_40, A_41, A_42, A_43, A_94_bus):
    """A_94_bus: 4-bit int (bit i = A_94_bus[i]). Returns A_60_bus as a
    4-bit int (bit i = A_60_bus[i])."""
    inputs = {
        "A_40": A_40, "A_41": A_41, "A_42": A_42, "A_43": A_43,
        "A_94_bus[0]": (A_94_bus >> 0) & 1, "A_94_bus[1]": (A_94_bus >> 1) & 1,
        "A_94_bus[2]": (A_94_bus >> 2) & 1, "A_94_bus[3]": (A_94_bus >> 3) & 1,
    }
    result = evaluate(inputs)
    return sum(result[f"A_60_bus[{i}]"] << i for i in range(4))


def full_truth_table():
    """All 2**8 = 256 (A_40 A_41 A_42 A_43, A_94_bus) -> A_60_bus rows."""
    rows = []
    for lo in range(16):
        A_40 = (lo >> 0) & 1
        A_41 = (lo >> 1) & 1
        A_42 = (lo >> 2) & 1
        A_43 = (lo >> 3) & 1
        for A_94_bus in range(16):
            out = compute(A_40, A_41, A_42, A_43, A_94_bus)
            rows.append((lo, A_94_bus, out))
    return rows


def combined_count(A_40, A_41, A_42, A_43, A_94_bus):
    """Packs BOTH inputs into one 8-bit value using the true counting
    bit order established for each (cluster9.py/cluster8.py's own
    count_value(), NOT raw port-declaration order): low nibble = the
    cluster9 count (bit0=A_40, bit1=A_43, bit2=A_42, bit3=A_41), high
    nibble = the cluster8 count (bit0=bus[0], bit1=bus[1], bit2=bus[3],
    bit3=bus[2]) -- i.e. bit order
    {A_94_bus[2], A_94_bus[3], A_94_bus[1], A_94_bus[0], A_41, A_42, A_43, A_40}
    from MSB to LSB. Only values 0..120 (lo in 0..10, hi in 0..10) are
    ones the real counters can ever actually reach."""
    import cluster8
    import cluster9

    lo_state = A_40 | (A_41 << 1) | (A_42 << 2) | (A_43 << 3)
    lo = cluster9.count_value(lo_state)
    hi = cluster8.count_value(A_94_bus)
    return (hi << 4) | lo


def search_output_bit_orders(reachable_only=True):
    """A_60_bus[0:3]'s DECLARED order is just as likely to be a
    decompiler-assigned identifier order with no intrinsic meaning as
    A_40..A_43/A_94_bus turned out to be (see cluster9.py/cluster8.py).
    Rather than assume the declared order is correct, this brute-forces
    all 4! = 24 ways to relabel the 4 output bits and checks each one
    against a handful of simple candidate functions of (lo_count,
    hi_count) -- the true-count values already established for the two
    inputs.

    Returns a list of (permutation, hypothesis_name) for every
    (relabeling, hypothesis) pair that matches EVERY reachable row
    exactly -- empty if nothing simple fits under any relabeling."""
    from itertools import permutations

    reordered = full_truth_table_reordered(reachable_only=reachable_only)

    hypotheses = {
        "out == lo_count": lambda lo, hi: lo,
        "out == hi_count": lambda lo, hi: hi,
        "out == lo_count XOR hi_count": lambda lo, hi: lo ^ hi,
        "out == lo_count AND hi_count": lambda lo, hi: lo & hi,
        "out == lo_count OR hi_count": lambda lo, hi: lo | hi,
        "out == (lo_count + hi_count) mod 16": lambda lo, hi: (lo + hi) & 0xF,
        "out == (hi_count - lo_count) mod 16": lambda lo, hi: (hi - lo) & 0xF,
        "out == (lo_count - hi_count) mod 16": lambda lo, hi: (lo - hi) & 0xF,
        "out == 1 if lo_count==10 else 0": lambda lo, hi: 1 if lo == 10 else 0,
        "out == 1 if hi_count==10 else 0": lambda lo, hi: 1 if hi == 10 else 0,
        "out == 1 if lo_count==10 and hi_count==10 else 0": lambda lo, hi: 1 if (lo == 10 and hi == 10) else 0,
        "out == min(lo_count, hi_count) mod 16": lambda lo, hi: min(lo, hi),
        "out == max(lo_count, hi_count) mod 16": lambda lo, hi: max(lo, hi),
    }

    matches = []
    for perm in permutations(range(4)):
        # perm[i] = which ORIGINAL output bit becomes new bit i
        def relabel(out, perm=perm):
            return sum(((out >> perm[i]) & 1) << i for i in range(4))

        for name, fn in hypotheses.items():
            if all(relabel(out) == fn(lo, hi) for combined, lo, hi, out in reordered):
                matches.append((perm, name))
    return matches


def full_truth_table_reordered(reachable_only=True):
    """Same 256 (or, with reachable_only, the 121 reachable) input
    combinations as full_truth_table(), but indexed/sorted by
    combined_count() instead of raw declaration bits -- lets the table
    be read in actual counting order (0, 1, 2, ... 120) instead of the
    scrambled order the raw bit patterns fall in.

    Returns a list of (combined, lo_count, hi_count, A_60_bus) rows,
    sorted by `combined`."""
    rows = []
    for lo_raw in range(16):
        A_40 = (lo_raw >> 0) & 1
        A_41 = (lo_raw >> 1) & 1
        A_42 = (lo_raw >> 2) & 1
        A_43 = (lo_raw >> 3) & 1
        for A_94_bus in range(16):
            combined = combined_count(A_40, A_41, A_42, A_43, A_94_bus)
            lo_count = combined & 0xF
            hi_count = combined >> 4
            if reachable_only and (lo_count > 10 or hi_count > 10):
                continue
            out = compute(A_40, A_41, A_42, A_43, A_94_bus)
            rows.append((combined, lo_count, hi_count, out))
    rows.sort(key=lambda r: r[0])
    return rows


if __name__ == "__main__":
    print(f"parsed {len(ASSIGNS)} assign statements")
    missing = [n for n in ["A_60_bus[0]", "A_60_bus[1]", "A_60_bus[2]", "A_60_bus[3]"] if n not in ASSIGNS]
    print(f"missing expected outputs: {missing}")

    rows = full_truth_table()
    print(f"evaluated {len(rows)} input combinations")

    print()
    print("checking simple hypotheses against every row:")
    hyp_eq = all(out == a94 for lo, a94, out in rows)
    hyp_sum = all(out == (lo + a94) & 0xF for lo, a94, out in rows)
    hyp_diff = all(out == (a94 - lo) & 0xF for lo, a94, out in rows)
    hyp_xor = all(out == (lo ^ a94) for lo, a94, out in rows)
    hyp_and = all(out == (lo & a94) for lo, a94, out in rows)
    hyp_or = all(out == (lo | a94) for lo, a94, out in rows)
    hyp_lo = all(out == lo for lo, a94, out in rows)
    print(f"  A_60_bus == A_94_bus:                          {hyp_eq}")
    print(f"  A_60_bus == (lo-nibble + A_94_bus) mod 16:      {hyp_sum}")
    print(f"  A_60_bus == (A_94_bus - lo-nibble) mod 16:      {hyp_diff}")
    print(f"  A_60_bus == lo-nibble XOR A_94_bus:             {hyp_xor}")
    print(f"  A_60_bus == lo-nibble AND A_94_bus:              {hyp_and}")
    print(f"  A_60_bus == lo-nibble OR A_94_bus:               {hyp_or}")
    print(f"  A_60_bus == lo-nibble (A_94_bus ignored):        {hyp_lo}")

    print()
    print(f"no simple hypothesis matched -- full {len(rows)}-row truth table:")
    header = f"{'A_40 A_41 A_42 A_43':>19} | {'A_94_bus[3:0]':>13} | A_60_bus[3:0]"
    print(header)
    print("-" * len(header))
    for lo, a94, out in rows:
        lo_bits = " ".join(str((lo >> i) & 1) for i in range(4))
        a94_bits = f"{a94:04b}"
        out_bits = f"{out:04b}"
        print(f"{lo_bits:>19} | {a94_bits:>13} | {out_bits}")

    print()
    print("re-ordered by the TRUE counting bit order "
          "{A_94_bus[2],A_94_bus[3],A_94_bus[1],A_94_bus[0],A_41,A_42,A_43,A_40} "
          "(reachable combos only: lo_count,hi_count in 0..10):")
    reordered = full_truth_table_reordered(reachable_only=True)
    print(f"{len(reordered)} reachable rows")
    print()
    header2 = f"{'combined':>8} {'hi_count':>8} {'lo_count':>8} | A_60_bus"
    print(header2)
    print("-" * len(header2))
    for combined, lo_count, hi_count, out in reordered:
        print(f"{combined:>8} {hi_count:>8} {lo_count:>8} | {out:04b}")

    print()
    print("checking hypotheses against the REACHABLE, re-ordered rows:")
    hyp_eq2 = all(out == combined for combined, lo_count, hi_count, out in reordered)
    hyp_lo2 = all(out == lo_count for combined, lo_count, hi_count, out in reordered)
    hyp_hi2 = all(out == hi_count for combined, lo_count, hi_count, out in reordered)
    hyp_term2 = all(out == (1 if lo_count == 10 else 0) for combined, lo_count, hi_count, out in reordered)
    print(f"  A_60_bus == combined 8-bit count:               {hyp_eq2}")
    print(f"  A_60_bus == lo_count (0-10):                     {hyp_lo2}")
    print(f"  A_60_bus == hi_count (0-10):                     {hyp_hi2}")
    print(f"  A_60_bus == (lo_count==10):                      {hyp_term2}")

    print()
    print("brute-forcing all 4! = 24 output-bit relabelings against the same hypothesis set:")
    matches = search_output_bit_orders(reachable_only=True)
    if not matches:
        print("  no (relabeling, hypothesis) pair matched every reachable row")
    else:
        for perm, name in matches:
            print(f"  MATCH: relabel new_bit[i] = old_bit[{perm}][i]  ->  {name}")
