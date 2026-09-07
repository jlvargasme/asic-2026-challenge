"""cluster8.py: concrete cycle-by-cycle simulation of cluster_type_8, the
high-digit mod-11 counter chained off cluster_type_9 (see cluster9.py):

    module cluster_type_8 (
        input A_75, input A_8, input clk, input rst_n,
        output [3:0] A_94_bus, output A2_43
    );
        assign A2_43 = ~A_94_bus[0] & A_94_bus[1] & A_94_bus[2] & ~A_94_bus[3];
        assign D_77 = (A_75 & A_8 & A_94_bus[0] & A_94_bus[1] & ~A_94_bus[2] & A_94_bus[3])
                    | (~A_94_bus[0] & A_94_bus[2] & A_94_bus[3]) | (A_94_bus[0] & A_94_bus[2] & ~A_94_bus[3])
                    | (~A_75 & A_94_bus[2]) | (~A_8 & A_94_bus[2]) | (~A_94_bus[1] & A_94_bus[2]);
        assign D_78 = (A_75 & A_8 & A_94_bus[0] & A_94_bus[1] & ~A_94_bus[3]) | (~A_75 & A_94_bus[3])
                    | (~A_8 & A_94_bus[3]) | (~A_94_bus[0] & A_94_bus[3]) | (~A_94_bus[1] & A_94_bus[3]);
        assign D_79 = (A_75 & A_8 & ~A_94_bus[0] & ~A_94_bus[1]) | (A_75 & A_8 & ~A_94_bus[0] & ~A_94_bus[2])
                    | (A_75 & A_8 & ~A_94_bus[0] & A_94_bus[3]) | (~A_75 & A_94_bus[0]) | (~A_8 & A_94_bus[0]);
        assign D_80 = (A_75 & A_8 & A_94_bus[0] & ~A_94_bus[1]) | (~A_94_bus[0] & A_94_bus[1] & ~A_94_bus[2])
                    | (~A_94_bus[0] & A_94_bus[1] & A_94_bus[3]) | (~A_75 & A_94_bus[1]) | (~A_8 & A_94_bus[1]);
        ... dfrtp_2 x4: D_77->A_94_bus[2], D_78->A_94_bus[3], D_79->A_94_bus[0], D_80->A_94_bus[1]
    endmodule

Same convention as cluster9.py: A_94_bus[0..3] (this register's OWN
current Q outputs) plus A_75/A_8 are read purely as INPUTS to the D-rule
equations above -- exactly as puzzle.v's own `assign` statements do.
D_77..D_80 are what those equations compute; the actual next register
state is just those same D values relabeled by which flip-flop each one
feeds (a wiring fact, not a boolean computation).

A_94_bus's bus index order is declared but NOT the counting bit order
(see the module's own comment) -- the register is modeled here as a
bitvector in plain declaration order (bit0=A_94_bus[0], bit1=A_94_bus[1],
bit2=A_94_bus[2], bit3=A_94_bus[3]), and count_value() separately
re-reads that same bitvector using the LSB->MSB order that was found (by
exhaustive search) to make this a clean binary counter: bit0=bus[0],
bit1=bus[1], bit2=bus[3], bit3=bus[2].
"""

REG_BITS = ["A_94_bus[0]", "A_94_bus[1]", "A_94_bus[2]", "A_94_bus[3]"]  # declaration order


def next_state(state, A_75, A_8):
    """state: bitvector value of REG_BITS (bit0=bus[0], bit1=bus[1],
    bit2=bus[2], bit3=bus[3]) -- this module's own current Q outputs,
    read here purely as D-rule INPUTS. Returns (next_state, A2_43)."""
    b0 = (state >> 0) & 1  # A_94_bus[0]
    b1 = (state >> 1) & 1  # A_94_bus[1]
    b2 = (state >> 2) & 1  # A_94_bus[2]
    b3 = (state >> 3) & 1  # A_94_bus[3]

    A2_43 = (~b0 & b1 & b2 & ~b3) & 1
    D_77 = ((A_75 & A_8 & b0 & b1 & ~b2 & b3) | (~b0 & b2 & b3) | (b0 & b2 & ~b3)
            | (~A_75 & b2) | (~A_8 & b2) | (~b1 & b2)) & 1
    D_78 = ((A_75 & A_8 & b0 & b1 & ~b3) | (~A_75 & b3) | (~A_8 & b3) | (~b0 & b3) | (~b1 & b3)) & 1
    D_79 = ((A_75 & A_8 & ~b0 & ~b1) | (A_75 & A_8 & ~b0 & ~b2) | (A_75 & A_8 & ~b0 & b3)
            | (~A_75 & b0) | (~A_8 & b0)) & 1
    D_80 = ((A_75 & A_8 & b0 & ~b1) | (~b0 & b1 & ~b2) | (~b0 & b1 & b3)
            | (~A_75 & b1) | (~A_8 & b1)) & 1

    # wiring: D_77->A_94_bus[2], D_78->A_94_bus[3], D_79->A_94_bus[0], D_80->A_94_bus[1]
    next_b0, next_b1, next_b2, next_b3 = D_79, D_80, D_77, D_78
    next_state_val = next_b0 | (next_b1 << 1) | (next_b2 << 2) | (next_b3 << 3)
    return next_state_val, A2_43


def reset_state():
    return 0  # rst_n=0 -> every A_94_bus bit = 0


def count_value(state):
    """Re-reads `state` (packed in REG_BITS/declaration order) using the
    LSB->MSB order that makes this module a clean binary counter:
    bit0=bus[0], bit1=bus[1], bit2=bus[3], bit3=bus[2] -- note bus[2]
    and bus[3] are swapped relative to declaration order."""
    b0 = (state >> 0) & 1
    b1 = (state >> 1) & 1
    b2 = (state >> 2) & 1
    b3 = (state >> 3) & 1
    return b0 | (b1 << 1) | (b3 << 2) | (b2 << 3)


def verify():
    """Exhaustively checks, over all 16 states x all 4 (A_75,A_8)
    combinations (64 cases), that count_value() behaves as a mod-11
    up-counter enabled by A_75 & A_8: holds when that AND is 0, +1 (mod
    16) when it's 1, except self-wraps 10->0 instead of reaching 11
    (A2_43 decodes count==10). Returns a list of violations -- empty if
    the claim held for every case."""
    violations = []
    for state in range(16):
        v = count_value(state)
        for A_75 in (0, 1):
            for A_8 in (0, 1):
                en = A_75 & A_8
                nstate, A2_43 = next_state(state, A_75, A_8)
                nv = count_value(nstate)
                expected = v if not en else (0 if A2_43 else (v + 1) % 16)
                if nv != expected:
                    violations.append((state, A_75, A_8, v, nv, expected))
                if A2_43 != (1 if v == 10 else 0):
                    violations.append(("A2_43 mismatch", state, v, A2_43))
    return violations


def print_transition_table():
    """Raw ground-truth table, straight out of next_state()'s own D-rule
    equations -- bits shown in plain REG_BITS/declaration order, NOT the
    count_value() reinterpretation.

    Column groups, left to right:
      - INPUTS to the D-rules: this register's own current Q outputs
        (A_94_bus[0..3]) plus the true primary inputs A_75, A_8.
      - A2_43: this module's other combinational output.
      - D-RULE OUTPUTS: D_77/D_78/D_79/D_80, exactly as puzzle.v's own
        `assign` statements compute them -- what gets latched into Q on
        the next clock edge, not the register state itself yet.
      - next register state: the same D values relabeled by which
        flip-flop each one loads (D_77->bus[2], D_78->bus[3],
        D_79->bus[0], D_80->bus[1]).

    One row per (current register state, A_75, A_8): 16 x 2 x 2 = 64
    rows total."""
    header = ("bus0 bus1 bus2 bus3 A_75 A_8 (inputs) | A2_43 | D_77 D_78 D_79 D_80 (D-rule outputs) | "
              "bus0 bus1 bus2 bus3 (next Q)")
    print(header)
    print("-" * len(header))
    for state in range(16):
        bits = [(state >> i) & 1 for i in range(4)]
        for A_75 in (0, 1):
            for A_8 in (0, 1):
                b0, b1, b2, b3 = bits
                A2_43 = (~b0 & b1 & b2 & ~b3) & 1
                D_77 = ((A_75 & A_8 & b0 & b1 & ~b2 & b3) | (~b0 & b2 & b3) | (b0 & b2 & ~b3)
                        | (~A_75 & b2) | (~A_8 & b2) | (~b1 & b2)) & 1
                D_78 = ((A_75 & A_8 & b0 & b1 & ~b3) | (~A_75 & b3) | (~A_8 & b3) | (~b0 & b3) | (~b1 & b3)) & 1
                D_79 = ((A_75 & A_8 & ~b0 & ~b1) | (A_75 & A_8 & ~b0 & ~b2) | (A_75 & A_8 & ~b0 & b3)
                        | (~A_75 & b0) | (~A_8 & b0)) & 1
                D_80 = ((A_75 & A_8 & b0 & ~b1) | (~b0 & b1 & ~b2) | (~b0 & b1 & b3)
                        | (~A_75 & b1) | (~A_8 & b1)) & 1
                next_b0, next_b1, next_b2, next_b3 = D_79, D_80, D_77, D_78
                print(f" {b0}    {b1}    {b2}    {b3}    {A_75}    {A_8}      |   {A2_43}   |"
                      f"  {D_77}    {D_78}    {D_79}    {D_80}                      | "
                      f" {next_b0}    {next_b1}    {next_b2}    {next_b3}")


if __name__ == "__main__":
    violations = verify()
    print(f"violations found: {len(violations)}")
    for v in violations[:20]:
        print(f"  {v}")

    print()
    print_transition_table()

    print()
    print("free-running count sequence from reset, A_75=A_8=1 held:")
    state = reset_state()
    for _ in range(13):
        v = count_value(state)
        state, A2_43 = next_state(state, 1, 1)
        print(f"  count={v:>2}  A2_43={A2_43}")
