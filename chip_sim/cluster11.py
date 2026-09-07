"""cluster11.py: simulate cluster_type_11 -- purely combinational, no
flip-flops, so there's no state/clock to step through (unlike
cluster9.py/cluster8.py/cluster12.py): just one function of the 11
primary inputs, evaluated for every possible input combination to
verify it really is the plain 11-input AND the module's own comment
claims.

    module cluster_type_11 (
        input [10:0] A_59_bus,
        output C
    );
        // module and11 -- AND of all inputs in A_59_bus
        sky130_fd_sc_hd__and3_2 ... (.A(A_99), .B(B_53), .C(C_8), .X(C));
        sky130_fd_sc_hd__and4_2 ... (.A(bus[1]), .B(bus[4]), .C(bus[6]), .D(bus[9]), .X(A_99));
        sky130_fd_sc_hd__and3_2 ... (.A(bus[0]), .B(bus[3]), .C(bus[7]), .X(C_8));
        sky130_fd_sc_hd__and4_2 ... (.A(bus[2]), .B(bus[5]), .C(bus[8]), .D(bus[10]), .X(B_53));
    endmodule

Standard-cell functions (fixed, documented sky130_fd_sc_hd behavior):
    and3_2: X = A & B & C
    and4_2: X = A & B & C & D
"""

WIDTH = 11


def compute_C(bus):
    """bus: bitvector value of A_59_bus (bit0 = A_59_bus[0]). Returns C
    exactly as the gate-level netlist computes it -- three separate
    AND-trees (4+3+4 inputs, grouped for fanout/timing, not correctness)
    combined by a final 3-input AND, NOT a direct 11-input reduction."""
    bit = lambda i: (bus >> i) & 1
    A_99 = bit(1) & bit(4) & bit(6) & bit(9)     # and4_2
    C_8 = bit(0) & bit(3) & bit(7)               # and3_2
    B_53 = bit(2) & bit(5) & bit(8) & bit(10)    # and4_2
    C = A_99 & B_53 & C_8                        # and3_2
    return C


def plain_and11(bus):
    """The claimed behavior (module comment: "AND of all inputs") --
    used only to cross-check compute_C(), not as the real model."""
    return 1 if bus == (1 << WIDTH) - 1 else 0


def verify():
    """Exhaustive check over all 2**11 = 2048 input combinations --
    small enough to brute-force rather than trust the comment or
    hand-simplify the gate tree."""
    mismatches = []
    for bus in range(1 << WIDTH):
        c_gate = compute_C(bus)
        c_plain = plain_and11(bus)
        if c_gate != c_plain:
            mismatches.append((bus, c_gate, c_plain))
    return mismatches


if __name__ == "__main__":
    mismatches = verify()
    print(f"checked {1 << WIDTH} input combinations")
    print(f"mismatches vs. plain 11-input AND: {len(mismatches)}")
    for bus, c_gate, c_plain in mismatches[:10]:
        print(f"  A_59_bus={bus:011b}  gate-tree C={c_gate}  plain-AND={c_plain}")

    print()
    print("spot checks:")
    for bus in (0, (1 << WIDTH) - 1, (1 << WIDTH) - 2, 0b10101010101):
        print(f"  A_59_bus={bus:011b} -> C={compute_C(bus)}")
