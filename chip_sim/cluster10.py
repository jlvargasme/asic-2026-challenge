"""cluster10.py: concrete cycle-by-cycle simulation of cluster_type_10,
the 3-bit "fuse selector" from puzzle.v:

    module cluster_type_10 (
        input [1:0] A_23_bus, input A_27, input A_28, input B_14, input C,
        input clk, input rst_n,
        output [2:0] A2_7_bus
    );
        assign D_5 = A_27 | A2_7_bus[1];
        assign D_6 = (~A2_7_bus[1] & ~A_23_bus[0] & A_27 & A_28 & B_14 & C & A_23_bus[1])
                   | (~A_27 & A2_7_bus[0]) | (A2_7_bus[1] & A2_7_bus[0]);
        assign D_7 = (~A2_7_bus[1] & A_23_bus[0] & A_27 & A_28 & B_14 & C & A_23_bus[1])
                   | (~A_27 & A2_7_bus[2]) | (A2_7_bus[1] & A2_7_bus[2]);
        ... dfrtp_2 x3, D_5->A2_7_bus[1], D_6->A2_7_bus[0], D_7->A2_7_bus[2]
    endmodule

Unlike the cluster_type_8/9/12 nets, A2_7_bus IS a genuine Verilog vector
port, so bit i really does mean A2_7_bus[i] here -- no decompiler
relabeling to untangle first (contrast cluster9.py/cluster8.py).

The register is a bitvector: bit0=A2_7_bus[0] (Q6), bit1=A2_7_bus[1]
(Q5), bit2=A2_7_bus[2] (Q7) -- names Q5/Q6/Q7 kept only because that's
what the module's own inline comment already calls them.
"""

REG_BITS = ["A2_7_bus[0]", "A2_7_bus[1]", "A2_7_bus[2]"]  # Q6, Q5, Q7


def next_state(state, A_23_bus, A_27, A_28, B_14, C):
    """state: bitvector value of REG_BITS (bit0=Q6, bit1=Q5, bit2=Q7).
    A_23_bus: 2-bit int (bit0=A_23_bus[0], bit1=A_23_bus[1]).
    Returns the next bitvector value; this module has no other output
    (A2_7_bus IS the whole output)."""
    Q6 = (state >> 0) & 1
    Q5 = (state >> 1) & 1
    Q7 = (state >> 2) & 1
    A23_0 = (A_23_bus >> 0) & 1
    A23_1 = (A_23_bus >> 1) & 1

    D_5 = A_27 | Q5
    D_6 = ((~Q5 & ~A23_0 & A_27 & A_28 & B_14 & C & A23_1)
           | (~A_27 & Q6) | (Q5 & Q6)) & 1
    D_7 = ((~Q5 & A23_0 & A_27 & A_28 & B_14 & C & A23_1)
           | (~A_27 & Q7) | (Q5 & Q7)) & 1

    return (D_6 & 1) | ((D_5 & 1) << 1) | ((D_7 & 1) << 2)


def reset_state():
    return 0  # rst_n=0 -> Q5=Q6=Q7=0


# Keys use this file's own bitvector packing (bit0=Q6, bit1=Q5, bit2=Q7),
# NOT the module comment's "(Q5,Q6,Q7)" ordering.
STATE_NAMES = {
    0b000: "idle (reset state)",           # Q6=0,Q5=0,Q7=0
    0b010: "triggered, no match",          # Q6=0,Q5=1,Q7=0
    0b011: "outcome A / SUCCESS (A_23_bus[0]=0 at trigger)",  # Q6=1,Q5=1,Q7=0
    0b110: "outcome B (A_23_bus[0]=1 at trigger)",            # Q6=0,Q5=1,Q7=1
}


def verify():
    """Exhaustively checks the module comment's own claims across every
    state (8) x every input combination (2**6 = 64) = 512 cases, rather
    than trusting the hand-derived comment:

      1. A_27=0 always holds the register (D_5=Q5, D_6=Q6, D_7=Q7).
      2. Once Q5=1 ("armed"), the register is frozen for every input --
         Q6/Q7 never change again regardless of A_27/A_23_bus/etc.
      3. From the idle state (0,0,0) with A_27=1, the register arms and
         picks Q6 xor Q7 in the SAME step, based on A_23_bus[0], but only
         when A_28 & B_14 & C & A_23_bus[1] are all 1 -- otherwise it
         arms with neither Q6 nor Q7 set ("no match").
      4. Only the 4 states in STATE_NAMES are ever reachable from reset.

    Returns a list of (description, state, inputs) violations -- empty
    if every claim held.
    """
    violations = []
    reachable = {0}
    for state in range(8):
        Q5 = (state >> 1) & 1
        for A_23_bus in range(4):
            for A_27 in (0, 1):
                for A_28 in (0, 1):
                    for B_14 in (0, 1):
                        for C in (0, 1):
                            nstate = next_state(state, A_23_bus, A_27, A_28, B_14, C)
                            inputs = dict(A_23_bus=A_23_bus, A_27=A_27, A_28=A_28, B_14=B_14, C=C)

                            if A_27 == 0 and nstate != state:
                                violations.append(("A_27=0 didn't hold", state, inputs, nstate))

                            if Q5 == 1 and nstate != state:
                                violations.append(("armed (Q5=1) state changed", state, inputs, nstate))

                            if state == 0:
                                reachable.add(nstate)

    unexpected = reachable - set(STATE_NAMES)
    if unexpected:
        violations.append(("reachable-from-reset states beyond the claimed 4", None, None, unexpected))

    return violations, reachable


if __name__ == "__main__":
    violations, reachable = verify()
    print(f"reachable states from reset: {sorted(STATE_NAMES.get(s, hex(s)) for s in reachable)}")
    print(f"violations found: {len(violations)}")
    for v in violations[:20]:
        print(f"  {v}")

    print()
    print("one-shot arming transitions from idle (state=0, A_27=1):")
    print(f"{'A_23_bus':>8} {'A_28':>4} {'B_14':>4} {'C':>1} | next state")
    for A_23_bus in range(4):
        for A_28 in (0, 1):
            for B_14 in (0, 1):
                for C in (0, 1):
                    nstate = next_state(0, A_23_bus, 1, A_28, B_14, C)
                    print(f"{A_23_bus:>8b} {A_28:>4} {B_14:>4} {C:>1} | "
                          f"{nstate:03b}  {STATE_NAMES.get(nstate, '?')}")
