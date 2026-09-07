"""cluster3.py: concrete cycle-by-cycle simulation of cluster_type_3 --
NOT another counter like cluster9/cluster8/cluster5: reading the 13
D-rule equations shows this is a 12-bit SERIAL SHIFT REGISTER (shifts
`I` in one bit at a time while A_8=1, holds while A_8=0) plus one extra
sticky "match found" latch, A_80, built exactly like cluster_type_12's
own set-only latch (an OR of trigger terms, self-held via a trailing
`| A_80` term).

    module cluster_type_3 (
        input [10:0] A_48_bus, input [1:0] A1_38_bus,
        input A_8, input B1_42, input I, input clk, input rst_n,
        output [1:0] A_23_bus
    );
        assign A_23_bus[0] = ~A_80;
        assign D_52 = (~A_8 & A0)   | (A_8 & A0_7);    // -> A0
        assign D_53 = (~A_8 & A0_1) | (A_8 & I);        // -> A0_1  (I shifts in HERE)
        assign D_54 = (~A_8 & A0_3) | (A_8 & A0_6);     // -> A0_3
        assign D_55 = (~A_8 & A0_4) | (A_8 & A0_5);     // -> A0_4
        assign D_61 = (~A_8 & A0_2) | (A_8 & A0_9);     // -> A0_2
        assign D_59 = (A_8 & A0_9 & B1_42 & I) | (A_8 & A0_8 & A1_38_bus[0] & I)
                    | (A1_38_bus[1] & A_8 & A0_2 & I) | (A1_38_bus[1] & A_8 & A0_1 & I)
                    | A_80;                              // -> A_80 (sticky)
        assign D_62 = (~A_8 & A0_9) | (A_8 & A0_8);     // -> A0_9
        assign D_58 = (~A_8 & A0_7) | (A_8 & A0_3);     // -> A0_7
        assign D_63 = (~A_8 & A0_11)| (A_8 & A0_10);    // -> A0_11
        assign D_64 = (~A_8 & A0_8) | (A_8 & A0_11);    // -> A0_8
        assign D_57 = (~A_8 & A0_6) | (A_8 & A0_4);     // -> A0_6
        assign D_56 = (~A_8 & A0_5) | (A_8 & A0_1);     // -> A0_5
        assign D_60 = (~A_8 & A0_10)| (A_8 & A0);       // -> A0_10
        // A_23_bus[1] is a SEPARATE, purely combinational 11-input AND
        // over A_48_bus (same shape as cluster_type_11) -- unrelated to
        // the shift register/A_80 entirely.
        assign B_49 = A_48_bus[2] & A_48_bus[5] & A_48_bus[8] & A_48_bus[10];
        assign A_79 = A_48_bus[1] & A_48_bus[4] & A_48_bus[6] & A_48_bus[9];
        assign C_6  = A_48_bus[0] & A_48_bus[3] & A_48_bus[7];
        assign A_23_bus[1] = A_79 & B_49 & C_6;
    endmodule

When A_8=0 every D_xx reduces to "hold this same register" (each term's
first half is `~A_8 & <same register>`, and D_59's non-A_80 terms all
carry an `&A_8` factor) -- verified exhaustively below, not assumed.

When A_8=1, the OTHER 12 registers (everything except A_80) form ONE
shift chain (found by reading each D_xx's `A_8&...` half as "next(Q) =
this other register"):

    I -> A0_1 -> A0_5 -> A0_4 -> A0_6 -> A0_3 -> A0_7 -> A0
      -> A0_10 -> A0_11 -> A0_8 -> A0_9 -> A0_2  (shifted out / discarded)

A_80 is a 13th, separate sticky latch: it becomes (and stays) 1 the
first time ANY of D_59's trigger terms holds -- each one requires the
FRESH bit (I=1) plus one specific PAST bit still sitting in the shift
chain (A0_9, A0_8, A0_2, or A0_1) plus a specific setting of the "mode
select" inputs A1_38_bus[1:0]/B1_42 (which never appear anywhere else
-- they're plain primary inputs to this module, not shift-chain taps).

B1_42 is undriven on the real chip, though: puzzle.v declares it a
top-level wire with no assign and no instance output anywhere feeding
it (dangling, same class of net as A1_55 -- see diag2.txt/diag3.txt's
zero-drivers-found + geometrically-isolated-net checks for 'B1#42'),
and puzzle.vcd's own trace holds it at a constant 0 for the entire
simulated run (single '0/&' value change, in the initial $dumpvars
block, never toggling after). So B1_42=0 always in practice, which
permanently kills the A0_9-tap OR-term in D_59 (`A_8 & A0_9 & B1_42 &
I` is 0 regardless of A0_9/I) -- simulated below as a fixed constant,
not a free input.
"""

# instance order from puzzle.v's own dfrtp_2 list: i0,i2,i3,i9,i17,i18,i19,i21,i22,i23,i25,i27,i32
REG_BITS = ["A0", "A0_1", "A0_3", "A0_4", "A0_2", "A_80",
            "A0_9", "A0_7", "A0_11", "A0_8", "A0_6", "A0_5", "A0_10"]

# the 12-stage shift chain (excludes A_80), newest-bit-first: I shifts
# into SHIFT_CHAIN[0]'s register, and each register's next value is
# whatever the PREVIOUS entry currently holds.
SHIFT_CHAIN = ["A0_1", "A0_5", "A0_4", "A0_6", "A0_3", "A0_7", "A0",
               "A0_10", "A0_11", "A0_8", "A0_9", "A0_2"]


def next_state(state, A_8, I, A1_38_bus):
    """state: bitvector value of REG_BITS. A1_38_bus: 2-bit int
    (bit0=A1_38_bus[0], bit1=A1_38_bus[1]). Returns (next_state,
    A_23_bus0) -- A_23_bus[1] doesn't depend on this module's own state
    at all, see and11_A_48_bus() below.

    B1_42 is not a parameter here: it's undriven on the real chip and
    puzzle.vcd holds it at a constant 0 for the whole simulated run
    (see this module's own docstring), so it's simulated as the fixed
    constant 0 rather than a free input -- which kills D_59's A0_9-tap
    term outright (see below)."""
    idx = {name: i for i, name in enumerate(REG_BITS)}
    bit = lambda name: (state >> idx[name]) & 1

    A0, A0_1, A0_3, A0_4, A0_2, A_80 = (bit(n) for n in
        ("A0", "A0_1", "A0_3", "A0_4", "A0_2", "A_80"))
    A0_9, A0_7, A0_11, A0_8, A0_6, A0_5, A0_10 = (bit(n) for n in
        ("A0_9", "A0_7", "A0_11", "A0_8", "A0_6", "A0_5", "A0_10"))
    A1_38_0 = (A1_38_bus >> 0) & 1
    A1_38_1 = (A1_38_bus >> 1) & 1

    A_23_bus0 = (~A_80) & 1

    D_52 = ((~A_8 & A0) | (A_8 & A0_7)) & 1
    D_53 = ((~A_8 & A0_1) | (A_8 & I)) & 1
    D_54 = ((~A_8 & A0_3) | (A_8 & A0_6)) & 1
    D_55 = ((~A_8 & A0_4) | (A_8 & A0_5)) & 1
    D_61 = ((~A_8 & A0_2) | (A_8 & A0_9)) & 1
    # B1_42=0 always -> the `A_8 & A0_9 & B1_42 & I` term is permanently 0
    D_59 = ((A_8 & A0_8 & A1_38_0 & I)
            | (A1_38_1 & A_8 & A0_2 & I) | (A1_38_1 & A_8 & A0_1 & I) | A_80) & 1
    D_62 = ((~A_8 & A0_9) | (A_8 & A0_8)) & 1
    D_58 = ((~A_8 & A0_7) | (A_8 & A0_3)) & 1
    D_63 = ((~A_8 & A0_11) | (A_8 & A0_10)) & 1
    D_64 = ((~A_8 & A0_8) | (A_8 & A0_11)) & 1
    D_57 = ((~A_8 & A0_6) | (A_8 & A0_4)) & 1
    D_56 = ((~A_8 & A0_5) | (A_8 & A0_1)) & 1
    D_60 = ((~A_8 & A0_10) | (A_8 & A0)) & 1

    # wiring: D_52->A0, D_53->A0_1, D_54->A0_3, D_55->A0_4, D_61->A0_2,
    #         D_59->A_80, D_62->A0_9, D_58->A0_7, D_63->A0_11,
    #         D_64->A0_8, D_57->A0_6, D_56->A0_5, D_60->A0_10
    next_vals = {"A0": D_52, "A0_1": D_53, "A0_3": D_54, "A0_4": D_55, "A0_2": D_61,
                 "A_80": D_59, "A0_9": D_62, "A0_7": D_58, "A0_11": D_63,
                 "A0_8": D_64, "A0_6": D_57, "A0_5": D_56, "A0_10": D_60}
    next_state_val = sum(next_vals[name] << i for i, name in enumerate(REG_BITS))
    return next_state_val, A_23_bus0


def reset_state():
    return 0


def and11_A_48_bus(bus):
    """A_23_bus[1] -- a plain 11-input AND over A_48_bus, unrelated to
    this module's own register state (same shape as cluster_type_11)."""
    return 1 if bus == (1 << 11) - 1 else 0


def shift_in(state, bits, A_8=1, A1_38_bus=0):
    """Feeds `bits` (a list of 0/1, oldest first) into the shift chain
    one per clock (A_8 held at 1 throughout unless overridden), A1_38_bus
    held constant. Returns the final state."""
    for b in bits:
        state, _ = next_state(state, A_8, b, A1_38_bus)
    return state


def read_chain(state):
    """The 12-bit shift-chain contents, in SHIFT_CHAIN order (index 0 =
    most recently shifted in)."""
    idx = {name: i for i, name in enumerate(REG_BITS)}
    return [(state >> idx[name]) & 1 for name in SHIFT_CHAIN]


def verify_hold():
    """Exhaustively checks that A_8=0 holds every one of the 8192
    register states, for every combination of the other 2 inputs (8
    combos, B1_42 fixed at 0) -- 8192*8 cases. Returns violation
    count."""
    violations = 0
    for state in range(1 << 13):
        for I in (0, 1):
            for A1_38_bus in range(4):
                ns, _ = next_state(state, 0, I, A1_38_bus)
                if ns != state:
                    violations += 1
    return violations


def verify_shift():
    """Confirms the claimed 12-stage shift chain: feeds a distinct known
    12-bit pattern in (with A_80's trigger inputs held at 0 so it can't
    interfere) and checks read_chain() comes back exactly reversed
    (index 0 = most recently shifted bit = last bit fed in)."""
    pattern = [1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1]
    state = shift_in(reset_state(), pattern, A_8=1, A1_38_bus=0)
    got = read_chain(state)
    expected = list(reversed(pattern))
    return got == expected, got, expected


if __name__ == "__main__":
    print(f"hold violations for A_8=0: {verify_hold()}")

    ok, got, expected = verify_shift()
    print(f"shift-chain check: {'OK' if ok else 'MISMATCH'}  got={got}  expected={expected}")

    print()
    print("D_59 (A_80's set-trigger) OR-terms, human readable:")
    print("  A_8 & A0_9 & B1_42     & I   (tap=A0_9,  needs B1_42=1 -- DEAD, B1_42 is always 0)")
    print("  A_8 & A0_8 & A1_38[0]  & I   (tap=A0_8,  needs A1_38_bus[0]=1)")
    print("  A_8 & A0_2 & A1_38[1]  & I   (tap=A0_2,  needs A1_38_bus[1]=1)")
    print("  A_8 & A0_1 & A1_38[1]  & I   (tap=A0_1,  needs A1_38_bus[1]=1)")
    print("  | A_80  (self-hold once set)")

    print()
    print("earliest A_80 latch under each fixed A1_38_bus mode (B1_42=0), "
          "scanning all 2**12 possible 12-bit I sequences up to length 12:")
    for A1_38_bus in range(4):
        best = None
        for length in range(1, 13):
            for n in range(1 << length):
                bits = [(n >> i) & 1 for i in reversed(range(length))]
                state = shift_in(reset_state(), bits, A_8=1, A1_38_bus=A1_38_bus)
                _, A_23_bus0 = next_state(state, 0, 0, 0)
                A_80 = 1 - A_23_bus0
                if A_80:
                    best = (length, bits)
                    break
            if best:
                break
        print(f"  A1_38_bus={A1_38_bus:02b}: "
              f"{'no trigger found in <=12 bits' if best is None else f'shortest I sequence {best[1]} (len {best[0]})'}")
