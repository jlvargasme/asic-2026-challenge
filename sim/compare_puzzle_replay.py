"""
compare_puzzle_replay.py: compare puzzle.v's own simulated O[7:0]/success
trace (sim/puzzle_replay.vcd, from sim/tb_puzzle_replay.cpp replaying a
reference VCD's own input stimulus -- see sim/vcd_stimulus.py and
sim/run_puzzle_replay.sh) against that SAME reference VCD's own recorded
O[7:0]/success trace (e.g. example_inputs.vcd). This is the actual
correctness check: does the decompiled RTL reproduce the example's known
OUTPUT, not just replay its known input identically (sim/compare_vcd.py
already confirms the input side matches).

A separate script from compare_vcd.py because puzzle.v's own top-level O
port isn't grouped into one Verilog vector -- chip_manipulation.py's own
top-level bus-grouping pass deliberately excludes true top-level ports
(see its own docstring) -- so it's rendered, and therefore traced, as 8
separate scalar ports (O_0_ .. O_7_, bit i = weight 2^i, matching every
other bus convention in this project) that have to be reassembled into
one 8-bit value before comparing against the reference's own single
"O [7:0]" bus.

Samples both traces at every clk POSEDGE in the reference (holding each
signal's last value forward, standard event-driven-simulation semantics)
rather than diffing raw transition lists, since O_0_.._O_7_ don't
necessarily change in lockstep with the reference's own single O bus
transitions.

Usage: python compare_puzzle_replay.py <reference.vcd> <our.vcd>
"""

import re
import sys


def _parse_changes(vcd_path, signal_names):
    """{signal_name: [(time, value), ...]} -- transitions only, decoded
    (int for 0/1 or a binary vector, the raw string for x/z)."""
    with open(vcd_path) as f:
        text = f.read()
    ids = {}
    for name in signal_names:
        m = re.search(
            r"\$var\s+\w+\s+\d+\s+(\S+)\s+" + re.escape(name) + r"(?:\s*\[[^\]]*\])?\s+\$end", text
        )
        if not m:
            raise ValueError(f"{name!r} not found in {vcd_path}")
        ids[m.group(1)] = name

    changes = {name: [] for name in signal_names}
    t = 0
    for line in text.splitlines():
        if line.startswith("#"):
            t = int(line[1:])
            continue
        if line[:1] in ("b", "B"):
            rest = line[1:]
            sp = rest.rfind(" ")
            if sp == -1:
                continue
            name = ids.get(rest[sp + 1:])
            if name:
                changes[name].append((t, _decode(rest[:sp])))
        elif len(line) >= 2 and line[0] in "01xz":
            name = ids.get(line[1:])
            if name:
                changes[name].append((t, _decode(line[0])))
    return changes


def _decode(value):
    if value and all(c in "01" for c in value):
        return int(value, 2)
    return value


def _sample(changes, at_times):
    """{time: value}, holding the most recent change at-or-before each
    requested time forward -- standard "current value of a signal at
    time T" event-driven-simulation semantics."""
    changes = sorted(changes)
    result = {}
    cur, ci = "x", 0
    for t in at_times:
        while ci < len(changes) and changes[ci][0] <= t:
            cur = changes[ci][1]
            ci += 1
        result[t] = cur
    return result


def main():
    ref_vcd, our_vcd = sys.argv[1], sys.argv[2]
    ref_changes = _parse_changes(ref_vcd, ["clk", "O", "success"])
    our_bits = [f"O_{i}_" for i in range(8)]
    our_changes = _parse_changes(our_vcd, ["clk", "success"] + our_bits)

    clk_edges = [t for t, v in sorted(ref_changes["clk"]) if v == 1]

    ref_o = _sample(ref_changes["O"], clk_edges)
    ref_success = _sample(ref_changes["success"], clk_edges)
    our_bit_samples = {b: _sample(our_changes[b], clk_edges) for b in our_bits}
    our_success = _sample(our_changes["success"], clk_edges)

    mismatches = []
    for t in clk_edges:
        our_o = 0
        for i, b in enumerate(our_bits):
            v = our_bit_samples[b][t]
            if v not in (0, 1):
                our_o = "x"
                break
            our_o |= v << i
        expected = (ref_o[t], ref_success[t])
        got = (our_o, our_success[t])
        if expected != got:
            mismatches.append((t, expected, got))

    print(f"compared {len(clk_edges)} clk posedge(s) between {ref_vcd} and {our_vcd}")
    if not mismatches:
        print(f"ALL MATCH -- {our_vcd} reproduces {ref_vcd}'s own O/success trace exactly")
        return 0
    print(f"{len(mismatches)} mismatch(es):")
    for t, expected, got in mismatches[:20]:
        print(f"  t={t}: expected O={expected[0]} success={expected[1]}  got O={got[0]} success={got[1]}")
    if len(mismatches) > 20:
        print(f"  ... and {len(mismatches) - 20} more")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
