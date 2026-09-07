"""Enumerate multiple solutions to the "success=1 AND readable ASCII"
constraint set, blocking each found witness before re-solving, to see
whether the (* TWO STARS *) message is the unique readable solution or
just the first one Z3 happened to find."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import z3
import solve_success
import solve_success_ascii
import drive_cluster1

T = solve_success.find_T()
print(f"T = {T}")

s, I, _ = solve_success.build_base_model(T)

# rebuild the ASCII-extension constraints the same way build_and_solve_full
# does, but keep a handle on `s`/`I` here so we can block and re-check.
import cluster1, cluster4, cluster6

def evaluate_z3(inputs):
    return solve_success_ascii.evaluate_z3(inputs)

extra_cycles = 20
A_3_bus_const = cluster4.cluster4(0, 0, 0, 0)
A1_12_bus_const = cluster6.compute(0, 0, 0, 0)

c1_regs = ["A_26", "A_25", "A_64", "A_65", "A_9", "A_14", "A_68", "A_69"]
c1_reset = {"A_26": False, "A_25": False, "A_64": False, "A_65": False,
            "A_9": True, "A_14": True, "A_68": True, "A_69": True}
c1_state = {name: z3.BoolVal(c1_reset[name]) for name in c1_regs}

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

full_state = dict(c1_state)
full_state["A_2"] = z3.BoolVal(False)
full_state["A_5"] = z3.BoolVal(False)
full_state["A_6"] = z3.BoolVal(False)
full_state["A_7"] = z3.BoolVal(False)

base_inputs_const2 = {
    "A2_7_bus[0]": z3.BoolVal(False), "A2_7_bus[1]": z3.BoolVal(True), "A2_7_bus[2]": z3.BoolVal(True),
    "A_20_bus[0]": z3.BoolVal(False), "A_20_bus[1]": z3.BoolVal(False),
    "A_32": z3.BoolVal(False), "A_8": z3.BoolVal(False),
    "A1_16": z3.BoolVal(False), "A1_22": z3.BoolVal(False), "B2_16": z3.BoolVal(False), "B2_2": z3.BoolVal(False),
    "I": z3.BoolVal(False),
}

printable_terms = []
for t in range(extra_cycles):
    a3 = solve_success_ascii.z3_cluster4(full_state["A_2"], full_state["A_5"], full_state["A_6"], full_state["A_7"])
    a1_12 = solve_success_ascii.z3_cluster6(full_state["A_2"], full_state["A_5"], full_state["A_6"], full_state["A_7"])
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

print("model built, enumerating solutions...\n")

N = 40
found = []
for k in range(N):
    check = s.check()
    if check != z3.sat:
        print(f"[{k}] {check} -- no more solutions")
        break
    m = s.model()
    I_seq = [1 if z3.is_true(m[I[t]]) else 0 for t in range(T)]
    o_bytes, success_cycle = drive_cluster1.run(I_seq, extra_cycles=30)
    # decode the message: bytes from success_cycle onward (the first output
    # byte -- e.g. the opening '(' -- lands ON success_cycle itself, not
    # after it) until a run of 0x00.
    msg_bytes = []
    started = False
    for t, o in o_bytes:
        if success_cycle is not None and t >= success_cycle:
            if o == 0:
                if started:
                    break
                else:
                    continue
            started = True
            msg_bytes.append(o)
    msg = "".join(chr(b) if 32 <= b < 127 else f"\\x{b:02x}" for b in msg_bytes)
    grid_rows = [I_seq[r*11:(r+1)*11] for r in range(11)]
    counts_ok = all(sum(row) == 2 for row in grid_rows) and \
                all(sum(I_seq[c::11]) == 2 for c in range(11))
    I_str = "".join(map(str, I_seq))
    print(f"[{k}] success_cycle={success_cycle} message={msg!r} 2-per-row/col={counts_ok}")
    print(f"     I = {I_str}")
    found.append((I_seq, msg))
    # block this exact I assignment
    s.add(z3.Or([I[t] != z3.BoolVal(bool(I_seq[t])) for t in range(T)]))

print("\ndone. distinct messages found:", sorted(set(m for _, m in found)))

import itertools
seqs = [f[0] for f in found]
varying = set()
for a, b in itertools.combinations(seqs, 2):
    for i in range(len(a)):
        if a[i] != b[i]:
            varying.add(i)
print("positions that ever differ across all solutions:", sorted(varying))
print("count:", len(varying))
for i, seq in enumerate(seqs):
    rows = [seq[r*11:(r+1)*11] for r in range(11)]
    print(i, [" ".join(map(str, row)) for row in rows])
