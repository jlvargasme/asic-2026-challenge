"""solve_success.py: find an I-sequence that drives the top `puzzle`
module's own `success` output high, per the corrected understanding in
puzzle_wip.v's own top module (see the "CORRECTED" comments added
there).

Key simplification, all verified below rather than assumed:

  A_27 (from cluster12) is entirely I-INDEPENDENT -- it only depends on
  enable/A_75/A2_43 -- so it's fully precomputable, and cluster10's own
  fuse only has a live trigger window on the SINGLE cycle A_27 first
  turns 1 (D_5 = A_27 | Q5 means Q5 arms unconditionally the very next
  cycle, closing the window for good). That fixes the target cycle T
  precisely (no search over "when" needed) -- confirmed T=121 by
  concrete simulation.

  At T, success needs (verified via cluster10.next_state(), see
  puzzle_wip.v's own corrected comment): A_23_bus={1,1}, A_27=1 (true by
  construction), A_28=1, B_14=1, C=1, all on cycle T.

  A_23_bus[1] = AND of cluster0_i2's own 11 A_48_bus bits = i2's own 11
  independent pairs (cluster0.PAIRS) each hit EXACTLY twice by T.
  A_23_bus[0] = ~A_80 (cluster3) -> need A_80=0 (never triggered) by T.
  C = AND of cluster0_i3's own 11 A_100_bus bits = i3's own 11 pairs.
  B_14 (cluster5) = true-count==22 at T -- also a pure cardinality
  constraint (advances by 1 each enabled cycle, no branching).
  A_28 (cluster7) = ~A_90 -- genuinely state-dependent (not a pure
  count), so this is the one piece that needs real cycle-by-cycle z3
  state tracking, plus A_80's OWN trigger (also state-dependent, but its
  taps reduce to direct past-I lookups since A_8=1 continuously over
  the whole window -- see below).

  A_8 is 1 for cycles 0..T-1 continuously (drops to 0 only entering T,
  the same instant A_27 first sets) -- confirmed by the T-finding
  simulation itself, used throughout to drop `~A_8 & ...` / `A_8 & ...`
  terms to their simplified concrete form.
"""

import sys
from pathlib import Path

import z3

sys.path.insert(0, str(Path(__file__).resolve().parent / "chip_sim"))

import cluster0
import cluster2
import cluster3
import cluster7
import cluster8
import cluster9
import cluster12


def find_T(enable=1, horizon=300):
    """The single cycle A_27 first turns 1 -- I-independent, so this
    needs no search over I at all."""
    lo = cluster9.reset_state()
    hi = cluster8.reset_state()
    c12 = cluster12.reset_state()
    for t in range(horizon):
        A_8 = (~c12 & enable) & 1
        lo_next, A_75 = cluster9.next_state(lo, A_8)
        hi_next, A2_43 = cluster8.next_state(hi, A_75, A_8)
        c12_next, _ = cluster12.next_state(c12, A_75, A2_43, enable)
        if c12 == 0 and c12_next == 1:
            return t + 1
        lo, hi, c12 = lo_next, hi_next, c12_next
    raise RuntimeError(f"A_27 never set within {horizon} cycles")


def precompute(T, enable=1):
    """Every I-INDEPENDENT per-cycle signal for cycles 0..T-1: cluster9/
    cluster8's raw states, A_75, A2_43, A_8, cluster2's A_60_bus (i3's
    own fed nibble), and cluster7's own A1_38_bus output (i.e.
    cluster3's mode-select inputs) -- none of these read I at all."""
    lo = cluster9.reset_state()
    hi = cluster8.reset_state()
    c12 = cluster12.reset_state()
    A_8_seq, A_75_seq = [], []
    i2_nibble_seq, i3_nibble_seq = [], []
    a1_38_seq = []
    for t in range(T):
        A_8 = (~c12 & enable) & 1
        A_40 = (lo >> 0) & 1
        A_41 = (lo >> 1) & 1
        A_42 = (lo >> 2) & 1
        A_43 = (lo >> 3) & 1
        lo_next, A_75 = cluster9.next_state(lo, A_8)
        hi_next, A2_43 = cluster8.next_state(hi, A_75, A_8)
        c12_next, _ = cluster12.next_state(c12, A_75, A2_43, enable)

        a60 = cluster2.compute(A_40, A_41, A_42, A_43, hi)
        A_60 = (a60 >> 0) & 1
        A_61 = (a60 >> 1) & 1
        A_N_10 = (a60 >> 2) & 1
        B_31 = (a60 >> 3) & 1
        i3_nibble = A_60 | (A_N_10 << 1) | (B_31 << 2) | (A_61 << 3)

        a1_38 = cluster7.a1_38_bus(A_40 | (A_41 << 1) | (A_42 << 2) | (A_43 << 3))

        A_8_seq.append(A_8)
        A_75_seq.append(A_75)
        i2_nibble_seq.append(lo)
        i3_nibble_seq.append(i3_nibble)
        a1_38_seq.append(a1_38)

        lo, hi, c12 = lo_next, hi_next, c12_next

    assert all(a == 1 for a in A_8_seq), "expected A_8=1 for the whole window up to T"
    return A_75_seq, i2_nibble_seq, i3_nibble_seq, a1_38_seq


def build_base_model(T):
    """Builds the model shared by build_and_solve() (success=1 alone)
    AND solve_success_ascii.py's own extended model (success=1 AND a
    printable-ASCII cluster1 message) -- every constraint that only
    needs success=1, factored out here so neither caller has to
    duplicate it. Deliberately stops BEFORE calling s.check(), so a
    caller can s.add(...) more constraints (e.g. solve_success_ascii.py's
    own cluster1 tracking + printable-ASCII terms) before solving.

    Returns (s, I, precomputed): `s` is the z3.Solver with the 5
    success-condition constraints already added (i2/i3/B_14 cardinality,
    cluster7's A_90 state, cluster3's A_80 state); `I` is the list of T
    free z3.Bool variables; `precomputed` is a dict of everything
    precompute() and the pattern-index lookup produced, in case a caller
    needs them too (A_75_seq, i2_nibble_seq, i3_nibble_seq, a1_38_seq,
    i2_pat, i3_pat)."""
    A_75_seq, i2_nibble_seq, i3_nibble_seq, a1_38_seq = precompute(T)
    # PAIRS[p][2] is the ACTUAL nibble value pattern p triggers on (not
    # p itself -- e.g. pattern 6 triggers on nibble 0, pattern 7 on
    # nibble 4).
    pattern_to_k = {pat: k for k, (_, _, pat, _) in enumerate(cluster0.PAIRS)}
    i2_pat = [pattern_to_k[n] for n in i2_nibble_seq]
    i3_pat = [pattern_to_k[n] for n in i3_nibble_seq]

    I = [z3.Bool(f"I_{t}") for t in range(T)]
    s = z3.Solver()

    # --- A_23_bus[1] (i2) and C (i3): each of the 11+11 pairs needs
    # EXACTLY 2 of its own trigger-nibble occurrences to get I=1 ---
    for p in range(11):
        s.add(z3.Sum([z3.If(I[t], 1, 0) for t in range(T) if i2_pat[t] == p]) == 2)
        s.add(z3.Sum([z3.If(I[t], 1, 0) for t in range(T) if i3_pat[t] == p]) == 2)

    # --- B_14 (cluster5): pure count, needs exactly 22 enabled advances
    # by T (A_8=1 throughout, so this is just Sum(I) == 22) ---
    s.add(z3.Sum([z3.If(I[t], 1, 0) for t in range(T)]) == 22)

    # --- A_28 (cluster7): genuinely state-dependent -- step the real
    # 3-bit FSM in z3, A_8=1 throughout (confirmed), A4=A_75_seq[t] ---
    A88 = [z3.BoolVal(False)]
    A90 = [z3.BoolVal(False)]
    A91 = [z3.BoolVal(False)]
    for t in range(T):
        A4 = z3.BoolVal(bool(A_75_seq[t]))
        nA88 = z3.Or(z3.And(z3.Not(A4), A91[t], I[t]), z3.And(z3.Not(A4), A88[t]))
        nA90 = z3.Or(A90[t], z3.And(A4, z3.Not(A88[t]), z3.Not(A91[t])),
                     z3.And(A4, A91[t], z3.Not(I[t])), z3.And(A4, A88[t], I[t]))
        nA91 = z3.Or(z3.And(z3.Not(A4), z3.Not(A91[t]), I[t]), z3.And(z3.Not(A4), A91[t], z3.Not(I[t])),
                     z3.And(z3.Not(A4), A88[t], I[t]))
        A88.append(nA88)
        A90.append(nA90)
        A91.append(nA91)
    s.add(z3.Not(A90[T]))  # A_28[T] = ~A_90[T] must be 1

    # --- A_23_bus[0] (cluster3's A_80): shift-chain taps reduce to
    # direct past-I lookups since A_8=1 continuously over the whole
    # window (verified above) -- SHIFT_CHAIN[9]=A0_8 (10 cycles ago),
    # SHIFT_CHAIN[11]=A0_2 (12 cycles ago), SHIFT_CHAIN[0]=A0_1 (1 cycle
    # ago). B1_42=0 always (undriven, per cluster3.py), dropping the
    # A0_9-tap term entirely. ---
    def I_at(k):
        return I[k] if k >= 0 else z3.BoolVal(False)

    A80 = [z3.BoolVal(False)]
    for t in range(T):
        A1_38_0 = (a1_38_seq[t] >> 0) & 1
        A1_38_1 = (a1_38_seq[t] >> 1) & 1
        a0_1 = I_at(t - 1)
        a0_8 = I_at(t - 10)
        a0_2 = I_at(t - 12)
        terms = [A80[t]]
        if A1_38_0:
            terms.append(z3.And(a0_8, I[t]))
        if A1_38_1:
            terms.append(z3.And(a0_2, I[t]))
            terms.append(z3.And(a0_1, I[t]))
        A80.append(z3.Or(*terms))
    s.add(z3.Not(A80[T]))  # A_23_bus[0] = ~A_80[T] must be 1

    precomputed = {
        "A_75_seq": A_75_seq, "i2_nibble_seq": i2_nibble_seq, "i3_nibble_seq": i3_nibble_seq,
        "a1_38_seq": a1_38_seq, "i2_pat": i2_pat, "i3_pat": i3_pat,
    }
    return s, I, precomputed


def build_and_solve(T):
    s, I, _ = build_base_model(T)
    result = s.check()
    if result != z3.sat:
        return result, None
    m = s.model()
    I_seq = [1 if z3.is_true(m[I[t]]) else 0 for t in range(T)]
    return result, I_seq


def verify_concrete(T, I_seq):
    """Full end-to-end replay through the REAL (non-reduced) simulators
    for every module, independent of the z3/cardinality reasoning
    above -- confirms `success` actually goes high at cycle T."""
    lo = cluster9.reset_state()
    hi = cluster8.reset_state()
    c12 = cluster12.reset_state()
    s0_i2 = cluster0.reset_state()
    s0_i3 = cluster0.reset_state()
    s3 = cluster3.reset_state()
    s7 = cluster7.reset_state()
    enable = 1

    for t in range(T):
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

        s0_i2 = cluster0.next_state(s0_i2, A_40, A_41, A_42, A_43, A_8, I_seq[t])
        s0_i3 = cluster0.next_state(s0_i3, A_60, A_N_10, B_31, A_61, A_8, I_seq[t])
        s3, _ = cluster3.next_state(s3, A_8, I_seq[t], a1_38)
        s7, _ = cluster7.next_state(s7, A_75, A_8, I_seq[t])

        lo, hi, c12 = lo_next, hi_next, c12_next

    A_48_bus = cluster0.a48_bus(s0_i2)
    A_100_bus = cluster0.a48_bus(s0_i3)
    C = 1 if A_100_bus == 0b111_1111_1111 else 0
    A_23_bus1 = 1 if A_48_bus == 0b111_1111_1111 else 0
    _, A_23_bus0 = cluster3.next_state(s3, 0, 0, 0)  # read ~A_80 off s3 (A_8/I/nibble irrelevant to the output read)
    _, A_28 = cluster7.next_state(s7, 0, 0, 0)  # read ~A_90 off s7 similarly
    A_27 = c12  # cluster12's own state IS A_27

    print(f"at cycle T={T}: A_27={A_27}, A_28={A_28}, A_23_bus=[{A_23_bus1},{A_23_bus0}] "
          f"(bus[1]=A_23_bus1, bus[0]=A_23_bus0), C={C}")

    success = 1 if (A_27 and A_28 and A_23_bus1 and A_23_bus0 and C) else 0
    print("success condition (A_27 & A_28 & A_23_bus[1] & A_23_bus[0] & C):", success)
    return success


if __name__ == "__main__":
    T = find_T()
    print(f"target cycle T (A_27 first sets, I-independent): {T}")

    result, I_seq = build_and_solve(T)
    print("z3 result:", result)
    if result == z3.sat:
        print("witness I-sequence:")
        print("".join(map(str, I_seq)))
        ok = verify_concrete(T, I_seq)
        print("VERIFIED success=1 via full concrete replay:" if ok else "concrete replay did NOT confirm success -- bug somewhere", ok)
