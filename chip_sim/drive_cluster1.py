"""drive_cluster1.py: run the FULL chip (every module solve_success.py's
own witness I-sequence touches, plus cluster_type_1/4/6's feedback loop)
and read off O_0_.._O_7_ as they unfold, to see what ASCII text
cluster_type_1 ("the output generator") actually produces once
`success` goes high.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cluster0
import cluster1
import cluster2
import cluster3
import cluster4
import cluster5
import cluster6
import cluster7
import cluster8
import cluster9
import cluster12
import solve_success


def run(I_seq, extra_cycles=200, enable=1):
    """Returns (o_bytes, success_cycle): o_bytes is a list of (cycle,
    O_byte) for every simulated cycle; success_cycle is the first cycle
    the chip's own `success` output (cluster_type_10's Q7,
    A2_7_bus[2]) reads 1, or None if it never does within this run."""
    T = len(I_seq)
    total_cycles = T + extra_cycles
    I_full = I_seq + [0] * extra_cycles  # A_8=0 after T makes I irrelevant everywhere (verified)

    lo = cluster9.reset_state()
    hi = cluster8.reset_state()
    c12 = cluster12.reset_state()
    s0_i2 = cluster0.reset_state()
    s0_i3 = cluster0.reset_state()
    s3 = cluster3.reset_state()
    s7 = cluster7.reset_state()
    s10 = 0  # cluster10's own 3-bit state, bit0=Q6(A2_7_bus[0]), bit1=Q5(A2_7_bus[1]), bit2=Q7/success
    s5 = cluster5.reset_state()
    s1 = cluster1.reset_state()
    a2a5a6a7 = 0  # cluster1's own A_2/A_5/A_6/A_7 output, fed into cluster4/cluster6

    o_bytes = []
    success_cycle = None
    for t in range(total_cycles):
        I_t = I_full[t]
        A_40 = (lo >> 0) & 1
        A_41 = (lo >> 1) & 1
        A_42 = (lo >> 2) & 1
        A_43 = (lo >> 3) & 1
        A_8 = (~c12 & enable) & 1

        lo_next, A_75 = cluster9.next_state(lo, A_8)
        hi_next, A2_43 = cluster8.next_state(hi, A_75, A_8)
        c12_next, _ = cluster12.next_state(c12, A_75, A2_43, enable)

        a60 = cluster2.compute(A_40, A_41, A_42, A_43, hi)
        A_60 = (a60 >> 0) & 1
        A_61 = (a60 >> 1) & 1
        A_N_10 = (a60 >> 2) & 1
        B_31 = (a60 >> 3) & 1

        a1_38 = cluster7.a1_38_bus(A_40 | (A_41 << 1) | (A_42 << 2) | (A_43 << 3))

        s0_i2_next = cluster0.next_state(s0_i2, A_40, A_41, A_42, A_43, A_8, I_t)
        s0_i3_next = cluster0.next_state(s0_i3, A_60, A_N_10, B_31, A_61, A_8, I_t)
        s3_next, _ = cluster3.next_state(s3, A_8, I_t, a1_38)
        s7_next, _ = cluster7.next_state(s7, A_75, A_8, I_t)
        s5_next, A_20_bus0, B_14, A_20_bus1 = cluster5.next_state(s5, A_8, I_t)

        A_48_bus = cluster0.a48_bus(s0_i2)
        A_100_bus = cluster0.a48_bus(s0_i3)
        C = 1 if A_100_bus == 0b111_1111_1111 else 0
        A_23_bus1 = 1 if A_48_bus == 0b111_1111_1111 else 0
        _, A_23_bus0 = cluster3.next_state(s3, 0, 0, 0)
        _, A_28 = cluster7.next_state(s7, 0, 0, 0)
        A_27 = c12
        A_20_bus = A_20_bus0 | (A_20_bus1 << 1)

        A2_7_bus = s10 & 0b111  # bit0=Q6, bit1=Q5, bit2=Q7/success (CURRENT, pre-edge)

        A_3_bus = cluster4.cluster4(
            (a2a5a6a7 >> 0) & 1, (a2a5a6a7 >> 1) & 1, (a2a5a6a7 >> 2) & 1, (a2a5a6a7 >> 3) & 1
        )
        A_3_bus_int = sum((b & 1) << i for i, b in enumerate(A_3_bus))
        A1_12_bus_int = cluster6.compute(
            (a2a5a6a7 >> 0) & 1, (a2a5a6a7 >> 1) & 1, (a2a5a6a7 >> 2) & 1, (a2a5a6a7 >> 3) & 1
        )

        s1_next, a2a5a6a7_next, o_bus = cluster1.next_state(
            s1, A2_7_bus, A_20_bus, A_3_bus_int, A1_12_bus_int, A_8, I_t
        )
        o_bytes.append((t, o_bus))

        # cluster10's OWN transition: D_5=A_27|Q5, D_6/D_7 need the
        # SAME 5 conditions -- step it explicitly here (small, 3 bits)
        Q5 = (s10 >> 1) & 1
        Q6 = (s10 >> 0) & 1
        Q7 = (s10 >> 2) & 1
        D_5 = A_27 | Q5
        D_6 = ((~Q5 & ~A_23_bus0 & A_27 & A_28 & B_14 & C & A_23_bus1) | (~A_27 & Q6) | (Q5 & Q6)) & 1
        D_7 = ((~Q5 & A_23_bus0 & A_27 & A_28 & B_14 & C & A_23_bus1) | (~A_27 & Q7) | (Q5 & Q7)) & 1
        s10_next = D_6 | (D_5 << 1) | (D_7 << 2)

        lo, hi, c12 = lo_next, hi_next, c12_next
        s0_i2, s0_i3, s3, s7, s5, s1 = s0_i2_next, s0_i3_next, s3_next, s7_next, s5_next, s1_next
        s10 = s10_next
        a2a5a6a7 = a2a5a6a7_next

        if success_cycle is None and ((s10 >> 2) & 1):  # Q7/success, just latched by this cycle's edge
            success_cycle = t + 1

    return o_bytes, success_cycle


def simulate(I_input, extra_cycles=30, enable=1):
    """The project's own answer to "give me an input string and simulate
    the output": drives `I_input` into the full chip (every module
    solve_success.py/drive_cluster1.py touch, wired exactly per
    puzzle_wip.v's own top module), one bit per clock, `enable` held
    high throughout (A_8 falls out of the chip's own dynamics -- see
    cluster12.py), and reports whether/when `success` went high plus
    whatever text cluster_type_1 printed.

    I_input: a string of '0'/'1' characters (e.g. the witness sequences
    printed by solve_success.py/solve_success_ascii.py), or any
    iterable of 0/1 ints -- both accepted so you can pass a hand-typed
    string or a list straight from another function's return value.

    Returns {"success": bool, "success_cycle": int | None,
    "message": str, "o_bytes": [(cycle, byte), ...]} -- `message` is
    the printable run of bytes from the first nonzero O byte through
    the next 0x00 terminator (non-printable bytes shown as '.'), empty
    if O never leaves 0 within this run. `extra_cycles` controls how
    far past the end of I_input the simulation keeps running (needed
    because cluster_type_1 only starts producing output the cycle after
    the chip arms, which can be after every bit of I_input has already
    been consumed -- see cluster1.py's own docstring)."""
    if isinstance(I_input, str):
        I_seq = [int(c) for c in I_input.strip()]
    else:
        I_seq = [int(b) for b in I_input]

    o_bytes, success_cycle = run(I_seq, extra_cycles=extra_cycles, enable=enable)

    message_bytes = []
    started = False
    for _, o in o_bytes:
        if o != 0:
            started = True
        if started:
            if o == 0:
                break
            message_bytes.append(o)
    message = "".join(chr(b) if 32 <= b < 127 else "." for b in message_bytes)

    return {
        "success": success_cycle is not None,
        "success_cycle": success_cycle,
        "message": message,
        "o_bytes": o_bytes,
    }


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1:
        # python drive_cluster1.py <I-bit-string> [extra_cycles]
        I_input = sys.argv[1]
        extra = int(sys.argv[2]) if len(sys.argv) > 2 else 30
        result = simulate(I_input, extra_cycles=extra)
        print(f"success: {result['success']}"
              + (f"  (cycle {result['success_cycle']})" if result["success"] else ""))
        print(f"message: {result['message']!r}")
    else:
        import z3

        T = solve_success.find_T()
        z3_result, I_seq = solve_success.build_and_solve(T)
        assert z3_result == z3.sat
        o_bytes, success_cycle = run(I_seq, extra_cycles=200)

        print(f"success_cycle: {success_cycle}")
        print(f"{'cycle':>5}  O7 O6 O5 O4 O3 O2 O1 O0  byte  chr")
        prev = None
        for t, o_bus in o_bytes:
            if o_bus != prev:
                ascii_char = chr(o_bus) if 32 <= o_bus < 127 else "."
                bits = " ".join(str((o_bus >> i) & 1) for i in reversed(range(8)))
                print(f"{t:>5}  {bits}  {o_bus:3}   {ascii_char!r}")
                prev = o_bus
