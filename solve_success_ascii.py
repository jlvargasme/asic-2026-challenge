"""solve_success_ascii.py: extends solve_success.py's model (reusing
solve_success.build_base_model() directly, not a copy of it) with
cluster_type_1's own registers, symbolically tracked through z3, plus a
printable-ASCII constraint on the resulting O bytes -- because
solve_success.py's `success=1` alone turned out to be necessary but NOT
sufficient for a legible message (confirmed empirically: 8 different
valid witnesses for success=1 all produced garbage-looking-but-
structurally-consistent output, while example_inputs.vcd's own
"TRY AGAIN" run -- which never sets success at all -- reproduced
perfectly through the same Python model, ruling out a decompiler/AST
bug). The gap is that cluster1's own seed registers (A_25/A_26/A_9/
A_14/A_68/A_69) depend on the EXACT I bit pattern during cycles 0..T-1
(via C1_3 = XNOR(I, A_67), which feeds A_68's own D-rule), not just the
cardinality totals solve_success.py's model enforces -- so this file
adds those registers as real z3 state and constrains the eventual O
bytes to be printable ASCII (or the 0x00 terminator), searching for an
I-sequence that satisfies BOTH conditions at once instead of finding an
arbitrary member of the (much larger) success-only solution set.

Key simplifications verified/used (all cheaper than tracking cluster1's
FULL 12-bit state honestly for cycles 0..T-1):
  - A_2/A_5/A_6/A_7 (cluster1's own dfxtp outputs) are PROVABLY 0 for
    the entire 0..T-1 window: every one of their own D-rules
    (D_8/D_3/D_9/D_4) has an `A2_7_bus[1] & ...` factor, and
    A2_7_bus[1] (Q5) is 0 until cycle T+1 (see solve_success.py's own
    "single trigger cycle" finding) -- so cluster4/cluster6's own
    outputs (A_3_bus/A1_12_bus) are CONSTANT = their value at
    A_2=A_5=A_6=A_7=0 for the whole window, not needing to be tracked
    at all until cycle T+1.
  - A2_7_bus is CONSTANT 000 for the whole 0..T window (Q5=Q6=Q7=0
    throughout, per the same trigger-cycle finding).
  - A_20_bus[1] is CONSTANT 0 (cluster5's true count never reaches 121
    within any horizon this file uses). A_20_bus[0] is 1 exactly until
    the FIRST I=1 cycle, then permanently 0 -- tracked as a single
    monotonic z3 Bool ("any_I_so_far"), not the full cluster5 state.
  - At T+1 onward (once success=1 is asserted as a hard constraint),
    A2_7_bus, A_20_bus, A_8, and I are all CONCRETE constants (success
    implies Q5=1,Q6=0,Q7=1; A_8=0 since A_27=1 forces it; cluster5's
    counter freezes once A_8=0; I is irrelevant once A_8=0, verified in
    cluster1.py's own docstring) -- so only cluster1's 12 registers
    (now including the no-longer-pinned A_2/A_5/A_6/A_7) and the live
    cluster4/cluster6 feedback need real z3 tracking from here on.
"""

import sys
from pathlib import Path

import z3

sys.path.insert(0, str(Path(__file__).resolve().parent / "chip_sim"))

import cluster1
import cluster4
import cluster6
import solve_success


def evaluate_z3(inputs):
    """z3 counterpart of cluster1.evaluate(): `inputs` maps name ->
    z3.BoolRef (or z3.BoolVal for concrete constants). Reuses
    cluster1.ASSIGNS/cluster1._TOKEN_RE/cluster1._sanitize directly (the
    SAME parsed source, not a re-parsed copy) so there's exactly one
    place the module's own boolean formulas live -- z3.BoolRef already
    overloads `&`/`|`/`~` as And/Or/Not, so the SAME expression strings
    cluster1.evaluate() uses work here unchanged."""
    memo = {}

    def get(name):
        if name in memo:
            return memo[name]
        if name in inputs:
            v = inputs[name]
        elif name in cluster1.ASSIGNS:
            v = eval_rhs(cluster1.ASSIGNS[name])
        else:
            raise KeyError(f"unknown signal {name!r}")
        memo[name] = v
        return v

    def eval_rhs(expr):
        ns = {cluster1._sanitize(tok): get(tok) for tok in set(cluster1._TOKEN_RE.findall(expr))}
        py_expr = cluster1._TOKEN_RE.sub(lambda m: cluster1._sanitize(m.group(0)), expr)
        return eval(py_expr, {"__builtins__": {}}, ns)

    for name in cluster1.ASSIGNS:
        get(name)
    return memo


def z3_cluster4(A_2, A_5, A_6, A_7):
    """z3 transcription of cluster4.cluster4() (that file uses Python
    `and`/`or`/`not` keywords, which don't overload for z3.BoolRef --
    so this is a fresh translation to `&`/`|`/`~`, not a reuse; kept
    small and mechanical to minimize risk)."""
    nA2, nA5, nA6, nA7 = z3.Not(A_2), z3.Not(A_5), z3.Not(A_6), z3.Not(A_7)
    a = [None] * 14
    a[0] = z3.Or(z3.And(nA2, nA6, nA7), z3.And(nA5, nA6, nA7))
    a[1] = z3.Or(z3.And(A_2, nA5, nA7), z3.And(A_6, nA7))
    a[2] = z3.And(A_2, A_5, nA6, nA7)
    a[3] = z3.Or(z3.And(nA2, nA5, nA6), z3.And(nA2, nA7), z3.And(nA5, nA7), z3.And(A_6, nA7))
    a[4] = z3.Or(z3.And(nA2, A_5, A_6, nA7), z3.And(nA2, nA5, nA6))
    a[5] = z3.Or(z3.And(nA2, nA5, nA6, A_7), z3.And(nA2, A_5, nA7))
    a[6] = z3.Or(z3.And(nA2, nA5, nA6, A_7), z3.And(A_2, nA5, nA6, nA7), z3.And(A_2, A_5, A_6, nA7))
    a[7] = z3.Or(z3.And(A_2, nA6, nA7), z3.And(nA2, A_6, nA7), z3.And(A_5, nA6, A_7), z3.And(nA2, A_5, nA6))
    a[8] = z3.Or(z3.And(nA2, A_5, nA7), z3.And(A_2, nA5, nA6), z3.And(A_5, nA6, A_7))
    a[9] = z3.Or(z3.And(nA2, nA5), z3.And(nA2, nA7), z3.And(nA5, nA7), z3.And(nA6, A_7))
    a[10] = z3.Or(z3.And(A_2, nA5, A_6, nA7), z3.And(nA2, nA6, nA7), z3.And(nA5, nA6, A_7))
    a[11] = z3.And(A_2, A_5, nA7)
    a[12] = z3.Or(z3.And(nA2, nA6), z3.And(nA5, nA6), z3.And(nA2, nA7), z3.And(nA5, nA7))
    a[13] = z3.Or(z3.And(A_2, nA5, nA6, nA7), z3.And(nA2, A_5, nA6, A_7), z3.And(nA2, nA5, A_6),
                  z3.And(nA2, A_6, nA7))
    return a


def z3_cluster6(A_2, A_5, A_6, A_7):
    """z3 transcription of cluster6.compute() (same reason as
    z3_cluster4: source uses plain Python operators, needs `&`/`|`/`~`
    for z3.BoolRef)."""
    nA2, nA5, nA6, nA7 = z3.Not(A_2), z3.Not(A_5), z3.Not(A_6), z3.Not(A_7)
    b = [None] * 13
    b[0] = z3.Or(z3.And(A_2, nA5, nA7), z3.And(A_2, A_6, nA7))
    b[1] = z3.Or(z3.And(A_2, A_6, nA7), z3.And(nA5, nA7))
    b[2] = z3.And(A_2, A_5, nA6, nA7)
    b[3] = z3.Or(z3.And(nA2, A_5, nA6, nA7), z3.And(A_2, nA5, A_6, nA7))
    b[4] = z3.Or(z3.And(nA2, nA7), z3.And(nA5, nA7), z3.And(A_6, nA7))
    b[5] = z3.Or(z3.And(A_2, nA5, nA6, nA7), z3.And(nA2, A_5, nA7), z3.And(A_5, A_6, nA7))
    b[6] = z3.Or(z3.And(nA2, nA5, nA6), z3.And(nA2, nA6, nA7), z3.And(A_2, A_6, nA7), z3.And(nA2, nA5, nA7))
    b[7] = z3.Or(z3.And(nA2, nA5, nA6, A_7), z3.And(nA5, A_6, nA7), z3.And(A_2, nA6, nA7))
    b[8] = z3.Or(z3.And(nA2, nA5, nA6), z3.And(nA5, nA7), z3.And(nA6, nA7), z3.And(A_2, nA7))
    b[9] = z3.Or(z3.And(nA2, nA6, nA7), z3.And(A_5, nA6, nA7))
    b[10] = z3.And(A_2, A_6, nA7)
    b[11] = z3.And(nA2, A_5, A_6, nA7)
    b[12] = z3.Or(z3.And(nA2, nA5, nA6, A_7), z3.And(nA2, nA5, A_6, nA7), z3.And(nA2, A_5, nA6, nA7),
                  z3.And(A_2, A_5, A_6, nA7))
    return b


def build_and_solve_full(T, extra_cycles=20):
    """Full model: solve_success.build_base_model()'s own 5
    success-condition constraints (reused directly, not duplicated),
    PLUS cluster1's 8 non-dfxtp registers tracked through 0..T-1, PLUS
    the continuation past T with cluster1's full 12-bit state (A_2/A_5/
    A_6/A_7 now live) and cluster4/cluster6's feedback, constraining the
    resulting O bytes to be printable ASCII or the 0x00 terminator."""
    s, I, _ = solve_success.build_base_model(T)

    # --- cluster1's 8 non-dfxtp registers, tracked symbolically through
    # cycles 0..T-1. A_2/A_5/A_6/A_7 are concrete False the whole time
    # (proven, see module docstring); A_3_bus/A1_12_bus are therefore
    # CONSTANT at their A_2=A_5=A_6=A_7=0 value. ---
    A_3_bus_const = cluster4.cluster4(0, 0, 0, 0)
    A1_12_bus_const = cluster6.compute(0, 0, 0, 0)

    c1_regs = ["A_26", "A_25", "A_64", "A_65", "A_9", "A_14", "A_68", "A_69"]
    c1_reset = {"A_26": False, "A_25": False, "A_64": False, "A_65": False,
                "A_9": True, "A_14": True, "A_68": True, "A_69": True}
    c1_state = {name: z3.BoolVal(c1_reset[name]) for name in c1_regs}

    # everything below is the SAME on every cycle 0..T-1 (see module
    # docstring: A2_7_bus is fixed at 000, A_2/A_5/A_6/A_7 provably 0,
    # A_3_bus/A1_12_bus are constant since cluster4/cluster6 only ever
    # see those fixed A_2/A_5/A_6/A_7) -- built ONCE instead of on every
    # one of the T iterations. Only "A_20_bus[0]" (tracks any_I_so_far)
    # and "I"/c1_regs (this cycle's own free var / previous cycle's
    # state) actually change per cycle, so those are set fresh on top of
    # a shallow copy of this dict inside the loop, not rebuilt from
    # scratch.
    base_inputs_const = {
        "A2_7_bus[0]": z3.BoolVal(False), "A2_7_bus[1]": z3.BoolVal(False), "A2_7_bus[2]": z3.BoolVal(False),
        "A_20_bus[1]": z3.BoolVal(False),
        "A_32": z3.BoolVal(False), "A_8": z3.BoolVal(True),
        "A1_16": z3.BoolVal(False), "A1_22": z3.BoolVal(False), "B2_16": z3.BoolVal(False), "B2_2": z3.BoolVal(False),
        "A_2": z3.BoolVal(False), "A_5": z3.BoolVal(False), "A_6": z3.BoolVal(False), "A_7": z3.BoolVal(False),
    }
    for i in range(14):
        base_inputs_const[f"A_3_bus[{i}]"] = z3.BoolVal(bool((A_3_bus_const[i]) & 1))
    for i in range(13):
        base_inputs_const[f"A1_12_bus[{i}]"] = z3.BoolVal(bool((A1_12_bus_const >> i) & 1))

    any_I_so_far = z3.BoolVal(False)
    for t in range(T):
        base_inputs = dict(base_inputs_const)
        base_inputs["A_20_bus[0]"] = z3.Not(any_I_so_far)
        base_inputs["I"] = I[t]
        for name in c1_regs:
            base_inputs[name] = c1_state[name]

        result = evaluate_z3(base_inputs)
        c1_state = {
            "A_26": result["D_39"], "A_25": result["D_40"], "A_64": result["D_41"], "A_65": result["D_42"],
            "A_9": result["D_43"], "A_14": result["D_44"], "A_68": result["D_45"], "A_69": result["D_46"],
        }
        any_I_so_far = z3.Or(any_I_so_far, I[t])

    # --- continuation past T: success=1 hard constraint makes A2_7_bus/
    # A_20_bus/A_8/I all concrete constants from here on (see module
    # docstring); cluster1's full 12-bit state (A_2/A_5/A_6/A_7 now
    # live) plus cluster4/cluster6's feedback are tracked for real.
    full_state = dict(c1_state)
    full_state["A_2"] = z3.BoolVal(False)
    full_state["A_5"] = z3.BoolVal(False)
    full_state["A_6"] = z3.BoolVal(False)
    full_state["A_7"] = z3.BoolVal(False)

    # constant from T+1 onward too (success=1 fixes A2_7_bus/A_20_bus/
    # A_8/I -- see module docstring); A_3_bus/A1_12_bus/c1_regs/A_2..A_7
    # genuinely evolve each cycle here (the real cluster1<->cluster4/6
    # feedback loop), so those stay inside the loop.
    base_inputs_const2 = {
        "A2_7_bus[0]": z3.BoolVal(False), "A2_7_bus[1]": z3.BoolVal(True), "A2_7_bus[2]": z3.BoolVal(True),
        "A_20_bus[0]": z3.BoolVal(False), "A_20_bus[1]": z3.BoolVal(False),
        "A_32": z3.BoolVal(False), "A_8": z3.BoolVal(False),
        "A1_16": z3.BoolVal(False), "A1_22": z3.BoolVal(False), "B2_16": z3.BoolVal(False), "B2_2": z3.BoolVal(False),
        "I": z3.BoolVal(False),
    }

    printable_terms = []
    for t in range(extra_cycles):
        a3 = z3_cluster4(full_state["A_2"], full_state["A_5"], full_state["A_6"], full_state["A_7"])
        a1_12 = z3_cluster6(full_state["A_2"], full_state["A_5"], full_state["A_6"], full_state["A_7"])
        base_inputs = dict(base_inputs_const2)
        for i in range(14):
            base_inputs[f"A_3_bus[{i}]"] = a3[i]
        for i in range(13):
            base_inputs[f"A1_12_bus[{i}]"] = a1_12[i]
        for name in c1_regs:
            base_inputs[name] = full_state[name]
        base_inputs["A_2"] = full_state["A_2"]
        base_inputs["A_5"] = full_state["A_5"]
        base_inputs["A_6"] = full_state["A_6"]
        base_inputs["A_7"] = full_state["A_7"]

        result = evaluate_z3(base_inputs)
        o_bits = [result[f"O_{i}_"] for i in range(8)]
        o_word = z3.Concat(*[z3.If(b, z3.BitVecVal(1, 1), z3.BitVecVal(0, 1)) for b in reversed(o_bits)])
        printable_terms.append(z3.Or(o_word == 0, z3.And(z3.UGE(o_word, 32), z3.ULE(o_word, 126))))

        full_state = {
            "A_26": result["D_39"], "A_25": result["D_40"], "A_64": result["D_41"], "A_65": result["D_42"],
            "A_9": result["D_43"], "A_14": result["D_44"], "A_68": result["D_45"], "A_69": result["D_46"],
            "A_2": result["D_9"], "A_5": result["D_3"], "A_6": result["D_8"], "A_7": result["D_4"],
        }

    for term in printable_terms:
        s.add(term)

    check_result = s.check()
    if check_result != z3.sat:
        return check_result, None
    m = s.model()
    I_seq = [1 if z3.is_true(m[I[t]]) else 0 for t in range(T)]
    return check_result, I_seq


if __name__ == "__main__":
    T = solve_success.find_T()
    print(f"T = {T}")
    result, I_seq = build_and_solve_full(T, extra_cycles=20)
    print("z3 result:", result)
    if result == z3.sat:
        print("witness:")
        print("".join(map(str, I_seq)))

        import drive_cluster1

        o_bytes, success_cycle = drive_cluster1.run(I_seq, extra_cycles=30)
        print(f"success_cycle: {success_cycle}")
        prev = None
        for t, o in o_bytes:
            if o != prev:
                print(f"cycle {t:3d}  O=0x{o:02x} ({o:3d})  {chr(o) if 32 <= o < 127 else '.'!r}")
                prev = o
