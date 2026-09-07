"""cluster9.py: concrete cycle-by-cycle simulation of cluster_type_9, the
low-digit mod-11 counter from puzzle.v:

    module cluster_type_9 (
        input A_8, input clk, input rst_n,
        output A_40, output A_41, output A_42, output A_43, output A_75
    );
        assign A_75 = ~A_40 & A_41 & ~A_42 & A_43;
        assign D_48 = (A_40 & ~A_42 & A_43 & A_8) | (~A_40 & A_42) | (A_42 & ~A_43) | (A_42 & ~A_8);
        assign D_49 = (A_40 & ~A_41 & A_42 & A_43 & A_8) | (~A_40 & A_41 & A_42) | (A_40 & A_41 & ~A_42) | (A_41 & ~A_43) | (A_41 & ~A_8);
        assign D_50 = (~A_40 & ~A_41 & A_8) | (~A_40 & ~A_43 & A_8) | (~A_40 & A_42 & A_8) | (A_40 & ~A_8);
        assign D_51 = (~A_40 & ~A_41 & A_43) | (~A_40 & A_42 & A_43) | (A_40 & ~A_43 & A_8) | (A_43 & ~A_8);
        ... dfrtp_2 x4: D_48->A_42, D_49->A_41, D_50->A_40, D_51->A_43
    endmodule

A_40/A_41/A_42/A_43 are decompiler-assigned identifiers with NO intrinsic
bit order (see puzzle.v's own comment on this module, and
chip_manipulation.py's _sanitize_ident/_dedupe naming) -- so the register
here is modeled as a bitvector in plain PORT DECLARATION order
(bit0=A_40, bit1=A_41, bit2=A_42, bit3=A_43), and a SEPARATE function,
count_value(), re-reads that same bitvector using the LSB->MSB order
that was found (by exhaustive search, see puzzle_wip.v's own comment) to
make this a clean binary counter: bit0=A_40, bit1=A_43, bit2=A_42,
bit3=A_41.
"""

REG_BITS = ["A_40", "A_41", "A_42", "A_43"]  # port-declaration order, NOT counting order


def next_state(state, A_8):
    """state: bitvector value of REG_BITS (bit0=A_40, bit1=A_41,
    bit2=A_42, bit3=A_43). Returns (next_state, A_75)."""
    A_40 = (state >> 0) & 1
    A_41 = (state >> 1) & 1
    A_42 = (state >> 2) & 1
    A_43 = (state >> 3) & 1

    A_75 = (~A_40 & A_41 & ~A_42 & A_43) & 1
    D_48 = ((A_40 & ~A_42 & A_43 & A_8) | (~A_40 & A_42) | (A_42 & ~A_43) | (A_42 & ~A_8)) & 1
    D_49 = ((A_40 & ~A_41 & A_42 & A_43 & A_8) | (~A_40 & A_41 & A_42) | (A_40 & A_41 & ~A_42)
            | (A_41 & ~A_43) | (A_41 & ~A_8)) & 1
    D_50 = ((~A_40 & ~A_41 & A_8) | (~A_40 & ~A_43 & A_8) | (~A_40 & A_42 & A_8) | (A_40 & ~A_8)) & 1
    D_51 = ((~A_40 & ~A_41 & A_43) | (~A_40 & A_42 & A_43) | (A_40 & ~A_43 & A_8) | (A_43 & ~A_8)) & 1

    next_A_40, next_A_41, next_A_42, next_A_43 = D_50, D_49, D_48, D_51
    next_state_val = next_A_40 | (next_A_41 << 1) | (next_A_42 << 2) | (next_A_43 << 3)
    return next_state_val, A_75


def reset_state():
    return 0  # rst_n=0 -> A_40=A_41=A_42=A_43=0


def count_value(state):
    """Re-reads `state` (packed in REG_BITS/port order) using the
    LSB->MSB order that makes this module a clean binary counter:
    bit0=A_40, bit1=A_43, bit2=A_42, bit3=A_41 -- NOT the same order
    next_state() packs the bitvector in."""
    A_40 = (state >> 0) & 1
    A_41 = (state >> 1) & 1
    A_42 = (state >> 2) & 1
    A_43 = (state >> 3) & 1
    return A_40 | (A_43 << 1) | (A_42 << 2) | (A_41 << 3)


def verify():
    """Exhaustively checks, over all 16 states x both values of A_8 (32
    cases), that count_value() really behaves as a mod-11 up-counter:
    holds when A_8=0, +1 (mod 16) when A_8=1, except self-wraps 10->0
    instead of reaching 11 (A_75 decodes count==10). Returns a list of
    violations -- empty if the claim held for every case."""
    violations = []
    for state in range(16):
        v = count_value(state)
        for A_8 in (0, 1):
            nstate, A_75 = next_state(state, A_8)
            nv = count_value(nstate)
            expected = v if A_8 == 0 else (0 if A_75 else (v + 1) % 16)
            if nv != expected:
                violations.append((state, A_8, v, nv, expected))
            if A_75 != (1 if v == 10 else 0):
                violations.append(("A_75 mismatch", state, v, A_75))
    return violations


def print_transition_table():
    """Raw ground-truth table, straight out of next_state()'s own D-rule
    equations -- bits shown in plain REG_BITS/port-declaration order, NOT
    the count_value() reinterpretation, so nothing here assumes or
    depends on the module comment's claimed bit order.

    Column groups, left to right:
      - INPUTS to the D-rules: the register's own current Q outputs
        (A_40 A_41 A_42 A_43 -- read combinationally here, same as any
        other input) plus the true primary input A_8.
      - D-RULE OUTPUTS: D_48/D_49/D_50/D_51, exactly as puzzle.v's own
        `assign` statements compute them from the inputs above -- these
        are what actually get latched into Q on the next clock edge, NOT
        the register state itself yet.
      - next register state: D_48/D_49/D_50/D_51 relabeled by which
        flip-flop each one loads (D_48->A_42, D_49->A_41, D_50->A_40,
        D_51->A_43 -- see the dfrtp_2 instances in puzzle.v), i.e. what
        Q actually becomes after the clock edge.

    One row per (current register state, A_8) pair: 16 states x 2 = 32
    rows total."""
    header = ("A_40 A_41 A_42 A_43 A_8 (inputs) | A_75 | D_48 D_49 D_50 D_51 (D-rule outputs) | "
              "A_40 A_41 A_42 A_43 (next Q)")
    print(header)
    print("-" * len(header))
    for state in range(16):
        bits = [(state >> i) & 1 for i in range(4)]
        for A_8 in (0, 1):
            A_40, A_41, A_42, A_43 = bits
            A_75 = (~A_40 & A_41 & ~A_42 & A_43) & 1
            D_48 = ((A_40 & ~A_42 & A_43 & A_8) | (~A_40 & A_42) | (A_42 & ~A_43) | (A_42 & ~A_8)) & 1
            D_49 = ((A_40 & ~A_41 & A_42 & A_43 & A_8) | (~A_40 & A_41 & A_42) | (A_40 & A_41 & ~A_42)
                    | (A_41 & ~A_43) | (A_41 & ~A_8)) & 1
            D_50 = ((~A_40 & ~A_41 & A_8) | (~A_40 & ~A_43 & A_8) | (~A_40 & A_42 & A_8) | (A_40 & ~A_8)) & 1
            D_51 = ((~A_40 & ~A_41 & A_43) | (~A_40 & A_42 & A_43) | (A_40 & ~A_43 & A_8) | (A_43 & ~A_8)) & 1
            next_A_40, next_A_41, next_A_42, next_A_43 = D_50, D_49, D_48, D_51
            print(f" {A_40}    {A_41}    {A_42}    {A_43}    {A_8}       |  {A_75}   |"
                  f"  {D_48}    {D_49}    {D_50}    {D_51}                    | "
                  f" {next_A_40}    {next_A_41}    {next_A_42}    {next_A_43}")


if __name__ == "__main__":
    violations = verify()
    print(f"violations found: {len(violations)}")
    for v in violations[:20]:
        print(f"  {v}")

    print()
    print_transition_table()

    print()
    print("free-running count sequence from reset, A_8=1 held:")
    state = reset_state()
    for _ in range(13):
        v = count_value(state)
        state, A_75 = next_state(state, 1)
        print(f"  count={v:>2}  A_75={A_75}")
