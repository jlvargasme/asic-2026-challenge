"""cluster1.py: simulate cluster_type_1, per its own comment "the output
generator, O[0-7] is ASCII text". Reuses cluster0.py/cluster2.py's
generic assign-text-evaluator approach given the size (~90 assigns).

    module cluster_type_1 (
        input [2:0] A2_7_bus, input [1:0] A_20_bus, input [13:0] A_3_bus,
        input A_32, input A_8, input [12:0] A1_12_bus, input A1_16,
        input A1_22, input B2_16, input B2_2, input I, input clk, input rst_n,
        output A_2, output A_5, output A_6, output A_7,
        output O_0_, output O_1_, ..., output O_7_
    );
        ... ~90 assigns, 12 registers ...
    endmodule

Top-level wiring (puzzle_wip.v): A2_7_bus = cluster_type_10's own
{success, A_17, A2_7} (bit2=success/Q7, bit1=A_17/Q5, bit0=A2_7/Q6).
A_3_bus comes from cluster_type_4, A1_12_bus from cluster_type_6 -- BOTH
fed by THIS module's own A_2/A_5/A_6/A_7 outputs (a real feedback loop:
cluster1 drives A_2/A_5/A_6/A_7, cluster4/cluster6 decode them into
A_3_bus/A1_12_bus, which feed back into cluster1's own next-state
logic). A_32/A1_16/A1_22/B2_16/B2_2 are undriven top-level nets (same
class as A1_55/B1_42) -- confirmed 0 for the entire puzzle.vcd trace,
never toggling (grep count of any '1' transition on their vcd codes: 0).

Every O_x assign has an `A2_7_bus[1] & ...` factor (A2_7_bus[1] is Q5,
cluster_type_10's "armed" bit) -- so ALL 8 outputs are hard-zeroed until
Q5=1, which per cluster10.py's own D_5=A_27|Q5 rule happens exactly one
cycle after A_27 first sets (cycle 121 in solve_success.py's own
finding) -- i.e. O_0_.._O_7_ only ever read as potentially non-zero
starting cycle 122 onward.

Register types (verified from the actual dfrtp_2/dfstp_2/dfxtp_2
instances -- NOT all the same reset behavior):
  dfxtp_2 (A_6,A_5,A_2,A_7): NO reset line at all. Their own D-rules
    (D_8,D_3,D_9,D_4) EVERY have an `A2_7_bus[1] & ...` factor, so while
    Q5=0 (the entire window before cycle 122) their next value is
    forced to 0 regardless of the previous value -- so whatever
    "undefined" real-silicon initial value they'd have doesn't matter;
    by cycle 1 they're already pinned to 0, and stay there through
    cycle 121. reset_state() below just initializes them to 0.
  dfrtp_2 (A_26,A_25,A_64,A_65): resets to 0 (RESET_B active low).
  dfstp_2 (A_9,A_14,A_68,A_69): resets to 1 (SET_B active low) -- a
    real, non-cosmetic difference from dfrtp; get this backwards and
    every downstream signal referencing these 4 registers is wrong from
    cycle 0.
"""

import re

SOURCE = """
assign D_8 = (A2_7_bus[1] & A_2 & A_5 & ~A_6) | (A2_7_bus[1] & ~A_2 & A_6) | (A2_7_bus[1] & ~A_5 & A_6) | (A2_7_bus[1] & A_2 & A_5 & A_7);
assign D_3 = (A2_7_bus[1] & A_2 & A_6 & A_7) | (A2_7_bus[1] & ~A_5);
assign D_9 = (A2_7_bus[1] & ~A_2 & A_5) | (A2_7_bus[1] & A_2 & ~A_5) | (A2_7_bus[1] & A_5 & A_6 & A_7);
assign D_4 = (A2_7_bus[1] & A_2 & A_5 & A_6) | (A2_7_bus[1] & A_7);
assign A3_1 = B_1 | A_9;
assign A1_1 = (~A_14 & B_4) | (A_14 & ~B_4);
assign B1_3 = (~A1_2 & ~A_15) | (~A2_2 & ~A_15);
assign A2_3 = ~A_9 | ~B_1;
assign B_5 = (A_16 & A_6) | (A_5 & A2_4);
assign A_18 = A2_7_bus[1] & A1_3;
assign A2_5 = ~A_19;
assign A_21 = (~A1_4 & ~A_20_bus[1] & ~A_20_bus[0]) | (~A2_7_bus[0] & ~A_20_bus[1] & ~A_20_bus[0]);
assign B_6 = (A3_2 & B1_5) | (A2_9 & B1_5) | (A1_6 & B1_5);
assign O_0_ = A2_7_bus[1] & B_7 & A1_3;
assign O_2_ = A2_7_bus[1] & B_8 & A1_3;
assign A2_11 = (B1_6 & A1_12_bus[6]) | (A1_12_bus[5] & A2_10);
assign O_7_ = A2_7_bus[1] & B_9 & A1_3;
assign O_5_ = A2_7_bus[1] & B_10 & A1_3;
assign B_7 = (A3_3 & B1_7) | (A2_11 & B1_7) | (A1_6 & B1_7);
assign B_10 = (A3_4 & B1_8) | (A2_12 & B1_8) | (A1_6 & B1_8);
assign A3_6 = (B1_9 & B2_2) | (A1_8 & A_24);
assign A3_4 = (B1_9 & A_3_bus[11]) | (A1_1 & A_24);
assign B_11 = (~A_25 & A_26) | (A_25 & ~A_26);
assign A3_7 = (B1_9 & A_3_bus[12]) | (A1_9 & A_24);
assign A3_2 = (B1_9 & A_3_bus[13]) | (A1_10 & A_24);
assign B_8 = (A3_7 & B1_10) | (A2_14 & B1_10) | (A1_6 & B1_10);
assign O_4_ = A2_7_bus[1] & B_12 & A1_3;
assign O_3_ = A2_7_bus[1] & B_6 & A1_3;
assign A1_3 = ~A_7 | ~A_6 | ~A_2 | ~A_5;
assign A1_4 = ~A2_7_bus[2];
assign B_13 = (A2_7_bus[0] & ~A_20_bus[1]) | (A2_7_bus[2] & ~A_20_bus[1]);
assign A_29 = ~A_20_bus[0] & ~B_13;
assign A_19 = ~A2_7_bus[0] | A_20_bus[1] | A2_7_bus[2] | A_20_bus[0];
assign O_6_ = A2_7_bus[1] & B_15 & A1_3;
assign B_16 = (A3_8 & B1_12) | (A2_15 & B1_12) | (A1_6 & B1_12);
assign A2_14 = (B1_6 & A1_12_bus[9]) | (A1_12_bus[0] & A2_10);
assign B1_13 = B_17 | A_24 | A_3_bus[0];
assign A2_15 = (B1_6 & A1_12_bus[10]) | (A1_12_bus[1] & A2_10);
assign A2_12 = (B1_6 & A1_12_bus[11]) | (A1_12_bus[2] & A2_10);
assign A2_9 = (B1_6 & A1_12_bus[12]) | (A1_12_bus[3] & A2_10);
assign A2_16 = (B1_6 & A1_12_bus[7]) | (A1_16 & A2_10);
assign A1_6 = ~A_24 & ~B_17;
assign A3_8 = (B1_9 & A_3_bus[7]) | (A1_17 & A_24);
assign A_15 = ~A_18 | A_8;
assign A_24 = ~A_29 & ~A2_5 & A_21;
assign B1_9 = ~A_21 & ~A_29 & A2_5;
assign A3_3 = (B1_9 & A_3_bus[8]) | (A1_18 & A_24);
assign B1_7 = B_17 | A_24 | A_3_bus[1];
assign A2_10 = ~A_21 & ~A2_5 & A_29;
assign B1_6 = ~A_21 & ~A_29 & ~A2_5;
assign B_12 = (A3_9 & B1_13) | (A2_16 & B1_13) | (A1_6 & B1_13);
assign B_17 = (~A_29 & ~A_21) | (~A2_5 & ~A_21);
assign B1_5 = B_17 | A_24 | A_3_bus[6];
assign A2_17 = A_8 | A_18;
assign A3_10 = (B1_9 & A_3_bus[9]) | (A1_19 & A_24);
assign A3_9 = (B1_9 & A_3_bus[10]) | (A1_20 & A_24);
assign A2_18 = (B1_6 & A1_12_bus[8]) | (A1_12_bus[4] & A2_10);
assign O_1_ = A2_7_bus[1] & B_16 & A1_3;
assign B1_8 = B_17 | A_24 | A_3_bus[2];
assign B1_14 = B_17 | A_24 | A_32;
assign B1_15 = B_17 | A_24 | A_3_bus[3];
assign B_15 = (A3_10 & B1_15) | (A2_18 & B1_15) | (A1_6 & B1_15);
assign B_9 = (A3_6 & B1_14) | (A2_19 & B1_14) | (A1_6 & B1_14);
assign A2_19 = (B1_6 & B2_16) | (A1_22 & A2_10);
assign C1 = ~A_15 & ~B_18;
assign B1_10 = B_17 | A_24 | A_3_bus[4];
assign B1_12 = B_17 | A_24 | A_3_bus[5];
assign A2_33 = (~A_7 & ~B_32) | (A_7 & B_32);
assign A_67 = (~A_66 & ~B_35) | (A_66 & B_35);
assign B_32 = A_5 & A_2 & A_6;
assign A1_10 = (~A_25 & B_36) | (A_25 & ~B_36);
assign B1_31 = ~A_8 & ~B_37 & A_67;
assign B1_32 = (~I & A_8) | (~A_67 & A_8);
assign B1_33 = (A_5 & A_6) | A3_21 | A2_38;
assign B_18 = (~A_64 & ~B_38) | (A_64 & B_38);
assign D_46 = (A_69 & B1_34) | (A_8 & A_64) | C1_1;
assign A1_35 = ~A_N_11 & B_39;
assign B_38 = (~A_25 & A_69) | (A_25 & ~A_69);
assign B1_35 = (~A_5 & ~A_70) | (~A2_38 & ~A_70) | (~A_5 & ~A3_22) | (~A2_38 & ~A3_22);
assign D_42 = (B1_34 & A_65) | (A_8 & A_25) | C1_2;
assign B2_18 = (B_39 & B_37) | (A_8 & A_9) | B1_34;
assign D_40 = (A2_17 & B2_18) | (A2_17 & B1_31) | (A_25 & B2_18) | (A_25 & B1_31);
assign C1_2 = (A2_2 & B1_3) | (A1_2 & B1_3);
assign B_37 = (~A_68 & ~B_11) | (A_68 & B_11);
assign A_N_11 = (~A_68 & B_1) | (A_68 & ~B_1);
assign B_40 = (~A_26 & A_70) | (A_26 & ~A_70);
assign B_1 = (~A_64 & ~A_66) | (A_64 & A_66);
assign A1_9 = (~A_9 & ~B_41) | (A_9 & B_41);
assign B_41 = (~A1_36 & ~A2_39) | ~B1_33;
assign C1_3 = (~I & ~A_67) | ~B1_32;
assign A2_2 = (~A_68 & A_65) | (A_68 & ~A_65);
assign C1_1 = ~A_67 & ~A_15;
assign A_71 = ~A_6 | ~A3_21;
assign A2_40 = ~A_8 & A_67 & A_N_11;
assign B2_19 = (~A_64 & ~A_72) | (A_64 & A_72);
assign B_42 = (A_6 & A2_38 & A3_22) | B1_35;
assign A3_23 = (A_8 & A_26) | B1_34;
assign A_72 = (~A_9 & ~A_14) | (A_9 & A_14);
assign A2_41 = (~A_72 & ~B_11) | (A_72 & B_11);
assign D_43 = (A3_23 & A_9) | (A3_23 & A2_17) | (A2_40 & A_9) | (A2_40 & A2_17) | (A1_35 & A_9) | (A1_35 & A2_17);
assign D_39 = (A_26 & B1_34) | (A_8 & A_68) | C1;
assign A1_17 = (~A_73 & ~B_40) | (A_73 & B_40);
assign B2_20 = ~A_2 | A_6 | A_5;
assign A_66 = (~A_65 & ~A_25) | (A_65 & A_25);
assign B_35 = (~A_14 & ~A_69) | (A_14 & A_69);
assign A1_2 = (~A_9 & A_26) | (A_9 & ~A_26);
assign A1_19 = (~A_64 & ~B_5) | (A_64 & B_5);
assign A1_20 = (~A_65 & B_43) | (A_65 & ~B_43);
assign A_73 = A2_4 | A_N_12 | A_74;
assign B1_36 = (A_14 & B1_34) | (A_8 & A_65);
assign A_16 = ~A_5 | A_2;
assign B_4 = ~A_16 | ~B_44;
assign B1_34 = ~A_18 & ~A_8;
assign A3_21 = ~A_5 & A_2;
assign B1_37 = (B1_34 & A_64) | (A_8 & A_14);
assign D_41 = (A1_37 & A2_3 & A3_1) | B1_37;
assign D_44 = (A1_37 & A2_41) | B1_36;
assign A1_37 = ~A_8 & A_18;
assign A_N_12 = ~A_5 & ~A_2;
assign A2_4 = ~A_6 & ~A_7;
assign A2_42 = ~A_2 | ~A_7;
assign D_45 = (A2_17 & B2_19 & C1_3) | (A2_17 & A_15 & C1_3) | (A_68 & B2_19 & C1_3) | (A_68 & A_15 & C1_3);
assign B_45 = (~A_71 & ~A_74) | (~A2_42 & ~A_74);
assign B_44 = (A2_33 & B2_20) | (A_74 & A_70);
assign A2_38 = ~A_7;
assign A_74 = ~A_5 & ~A2_38;
assign A3_22 = ~A_5 | ~A_2;
assign B_43 = (A3_21 & A2_42) | (A_74 & A2_42) | (A_6 & A2_42);
assign A1_36 = ~A_71;
assign A1_18 = (~A_68 & ~B_45) | (A_68 & B_45);
assign B_36 = (A3_24 & A2_39) | (A3_24 & B_46) | (A2_33 & A2_39) | (A2_33 & B_46) | (A1_36 & A2_39) | (A1_36 & B_46);
assign A1_8 = (~A_69 & ~B_42) | (A_69 & B_42);
assign A3_24 = ~A_6 & ~B_46;
assign A_70 = A_6 | A_2;
assign A2_39 = ~A_70 | ~A2_33;
assign B_46 = ~A_N_12 & A3_22;
assign B_39 = ~A_8 & ~A_67;
"""

# (D-signal, Q-signal, reset_value) -- reset_value differs by cell type,
# NOT uniformly 0 (see module docstring: dfstp_2 resets to 1)
REG_WIRING = [
    ("D_8", "A_6", 0), ("D_3", "A_5", 0), ("D_9", "A_2", 0), ("D_4", "A_7", 0),      # dfxtp_2, no real reset (see docstring)
    ("D_39", "A_26", 0), ("D_40", "A_25", 0), ("D_41", "A_64", 0), ("D_42", "A_65", 0),  # dfrtp_2 -> 0
    ("D_43", "A_9", 1), ("D_44", "A_14", 1), ("D_45", "A_68", 1), ("D_46", "A_69", 1),   # dfstp_2 -> 1
]
REG_BITS = [q for _, q, _ in REG_WIRING]

_ASSIGN_RE = re.compile(r"assign\s+([\w\[\]]+)\s*=\s*(.+?);")
_TOKEN_RE = re.compile(r"[A-Za-z_][A-Za-z0-9_]*(?:\[\d+\])?")
ASSIGNS = {lhs: rhs for lhs, rhs in _ASSIGN_RE.findall(SOURCE)}
OUTPUT_NAMES = [f"O_{i}_" for i in range(8)]


class Bit(int):
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
    current Q (A2_7_bus[i], A_20_bus[i], A_3_bus[i], A1_12_bus[i], A_32,
    A_8, A1_16, A1_22, B2_16, B2_2, I, plus every REG_BITS entry).
    Returns {name: 0/1} for every `assign` target."""
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


def next_state(state, A2_7_bus, A_20_bus, A_3_bus, A1_12_bus, A_8, I):
    """state: bitvector value of REG_BITS. A2_7_bus: 3-bit int
    (bit0=A2_7_bus[0]/Q6, bit1=A2_7_bus[1]/Q5, bit2=A2_7_bus[2]/success).
    A_20_bus: 2-bit int. A_3_bus: 14-bit int. A1_12_bus: 13-bit int.
    A_32/A1_16/A1_22/B2_16/B2_2 are hardwired to 0 (undriven, confirmed
    constant for the whole puzzle.vcd trace). Returns (next_state,
    A_2A_5A_6A_7 as a 4-bit int [bit0=A_2,bit1=A_5,bit2=A_6,bit3=A_7],
    O_bus as an 8-bit int [bit i = O_i_])."""
    inputs = {
        "A2_7_bus[0]": (A2_7_bus >> 0) & 1, "A2_7_bus[1]": (A2_7_bus >> 1) & 1,
        "A2_7_bus[2]": (A2_7_bus >> 2) & 1,
        "A_20_bus[0]": (A_20_bus >> 0) & 1, "A_20_bus[1]": (A_20_bus >> 1) & 1,
        "A_32": 0, "A_8": A_8, "A1_16": 0, "A1_22": 0, "B2_16": 0, "B2_2": 0, "I": I,
    }
    for i in range(14):
        inputs[f"A_3_bus[{i}]"] = (A_3_bus >> i) & 1
    for i in range(13):
        inputs[f"A1_12_bus[{i}]"] = (A1_12_bus >> i) & 1
    for i, name in enumerate(REG_BITS):
        inputs[name] = (state >> i) & 1

    result = evaluate(inputs)
    next_bits = [result[d] for d, q, _ in REG_WIRING]
    next_state_val = sum(b << i for i, b in enumerate(next_bits))

    A_2, A_5, A_6, A_7 = result["A_2"], result["A_5"], result["A_6"], result["A_7"]
    a2a5a6a7 = A_2 | (A_5 << 1) | (A_6 << 2) | (A_7 << 3)
    o_bus = sum(result[f"O_{i}_"] << i for i in range(8))
    return next_state_val, a2a5a6a7, o_bus


def reset_state():
    return sum(rv << i for i, (_, _, rv) in enumerate(REG_WIRING))


def read_a2_a5_a6_a7(state):
    idx = {name: i for i, name in enumerate(REG_BITS)}
    A_6 = (state >> idx["A_6"]) & 1
    A_5 = (state >> idx["A_5"]) & 1
    A_2 = (state >> idx["A_2"]) & 1
    A_7 = (state >> idx["A_7"]) & 1
    return A_2 | (A_5 << 1) | (A_6 << 2) | (A_7 << 3)
