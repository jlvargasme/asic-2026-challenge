"""cluster12.py: concrete cycle-by-cycle simulation of cluster_type_12,
the "sticky done" latch from puzzle.v. Current puzzle_wip.v text (the
flattened, authoritative form -- see its own regenerated `assign`
lines, not the earlier gate-level a31o_2/and2b_2 snippet this file was
first transcribed from):

    module cluster_type_12 (
        input A_75, input A2_43, input clk, input enable, input rst_n,
        output A_27, output A_8
    );
        wire D_47;
        assign A_8 = ~A_27 & enable;
        assign D_47 = (A_75 & A2_43 & enable) | A_27;
        sky130_fd_sc_hd__dfrtp_2 ... (.CLK(clk), .D(D_47), .RESET_B(rst_n), .Q(A_27));
    endmodule

D_47's own AND term reads `enable` directly here, not `A_8` -- an
earlier version of this file used `A_8` (matching the a31o_2 cell's A3
pin in the original gate-level snippet, which IS wired to A_8 there,
not enable) instead. Both forms are exhaustively verified equivalent
(0 mismatches across all 16 (A_27,A_75,A2_43,enable) combinations): the
only case A_8 != enable is A_27=1 (A_8=~1&enable=0 there), and that's
exactly the case where the `| A_27` term already forces D_47=1
regardless of the AND term's value -- so the substitution never
actually changes the result. Written to match `enable` here purely for
fidelity to the current source text, not because the old form was
behaviorally wrong.

Standard-cell function for the flip-flop itself (not re-derived -- a
fixed, documented gate function):
    dfrtp_2: Q <- D on the rising clk edge; RESET_B=0 asynchronously
             forces Q=0

The register is modeled as a bitvector (an int, bit i = REG_BITS[i]'s
own value) rather than a single named variable, so the same shape
generalizes directly to a cluster with more than one flip-flop -- here
REG_BITS has exactly one entry (A_27), so the "bitvector" is width 1,
but next_state()/enumerate_matrix() don't assume that.
"""

REG_BITS = ["A_27"]  # this cluster's own flip-flop(s), bit0 first


def next_state(state, A_75, A2_43, enable):
    """state: bitvector value of REG_BITS (bit0 = REG_BITS[0] = A_27).
    Returns (next_state, A_8) -- A_8 is the module's other, purely
    combinational output (not part of the register itself)."""
    A_27 = state & 1  # REG_BITS has only 1 entry, so this IS bit 0 already
    A_8 = (~A_27 & enable) & 1
    D_47 = ((A_75 & A2_43 & enable) | A_27) & 1
    return D_47, A_8


def reset_state():
    """rst_n = 0 asynchronously forces every flip-flop's Q to 0."""
    return 0


def enumerate_matrix():
    """Every register state x every input combination -- 2**len(REG_BITS)
    states x 2**3 input combos (A_75, A2_43, enable) = 16 rows for this
    module, small enough to brute-force exhaustively rather than reason
    through the boolean expressions by hand.

    Yields (state, A_75, A2_43, enable, A_8, next_state) tuples."""
    n_states = 2 ** len(REG_BITS)
    for state in range(n_states):
        for A_75 in (0, 1):
            for A2_43 in (0, 1):
                for enable in (0, 1):
                    nstate, A_8 = next_state(state, A_75, A2_43, enable)
                    yield state, A_75, A2_43, enable, A_8, nstate


def print_matrix():
    print(f"{'A_27':>4} {'A_75':>4} {'A2_43':>5} {'en':>2} | {'A_8':>3} | next_A_27")
    for state, A_75, A2_43, enable, A_8, nstate in enumerate_matrix():
        print(f"{state:>4} {A_75:>4} {A2_43:>5} {enable:>2} | {A_8:>3} | {nstate}")


if __name__ == "__main__":
    print_matrix()
