"""
vcd_stimulus.py: extract just the INPUT-signal (clk/rst_n/enable/I) value
changes from a reference VCD (e.g. example_inputs.vcd) into a small,
easy-to-replay text format -- one #<time> block per timestamp, each
followed by its own "<signal>=<value>" change lines, in the same time
order as the source VCD -- so sim/tb_puzzle_replay.cpp can drive puzzle.v
with EXACTLY the same stimulus, at the exact same simulated timestamps,
that produced the reference O/success trace. The reference VCD's own
O/success columns are deliberately NOT extracted here: they're the
EXPECTED side of the comparison, read directly out of the original file
by compare_vcd.py instead of being duplicated into this stimulus.

Usage: python vcd_stimulus.py <reference.vcd> <input_signal ...> [-o out.txt]
Example: python vcd_stimulus.py example_inputs.vcd clk rst_n enable I
"""

import re
import sys


def parse_vcd_ids(text, signal_names):
    """{signal_name: vcd_id_string} for each requested signal, from the
    file's own $var declarations (tolerates a trailing "[N:0]" bit-range
    suffix on the declaration, e.g. "O [7:0]")."""
    ids = {}
    for name in signal_names:
        m = re.search(r"\$var\s+\w+\s+\d+\s+(\S+)\s+" + re.escape(name) + r"(?:\s*\[[^\]]*\])?\s+\$end", text)
        if not m:
            raise ValueError(f"signal {name!r} not found in this VCD")
        ids[name] = m.group(1)
    return ids


def extract_stimulus(vcd_path, signal_names):
    """[(time, [(signal, value), ...]), ...] -- one entry per timestamp
    that changes at least one of `signal_names`, in file order. Only
    handles scalar (1-bit) signals, the only kind this project's own
    top-level control/data-bit ports (clk/rst_n/enable/I) ever are."""
    with open(vcd_path) as f:
        text = f.read()
    id_to_name = {v: k for k, v in parse_vcd_ids(text, signal_names).items()}

    events = []
    t = 0
    current = None
    for line in text.splitlines():
        if line.startswith("#"):
            t = int(line[1:])
            current = None
            continue
        if len(line) >= 2 and line[0] in "01xz":
            sig_id = line[1:]
            name = id_to_name.get(sig_id)
            if name is None:
                continue
            if current is None or current[0] != t:
                current = (t, [])
                events.append(current)
            current[1].append((name, line[0]))
    return events


def write_stimulus_file(events, out_path):
    with open(out_path, "w") as f:
        for t, changes in events:
            f.write(f"#{t}\n")
            for name, val in changes:
                f.write(f"{name}={val}\n")
    print(f"wrote {out_path} ({len(events)} timestamp(s))")


if __name__ == "__main__":
    args = sys.argv[1:]
    out_path = None
    if "-o" in args:
        i = args.index("-o")
        out_path = args[i + 1]
        args = args[:i] + args[i + 2:]

    if len(args) < 2:
        print("usage: python vcd_stimulus.py <reference.vcd> <input_signal ...> [-o out.txt]")
        print("  e.g.: python vcd_stimulus.py example_inputs.vcd clk rst_n enable I")
        raise SystemExit(1)

    vcd_path, signal_names = args[0], args[1:]
    out_path = out_path or (vcd_path.rsplit(".", 1)[0] + "_stimulus.txt")
    events = extract_stimulus(vcd_path, signal_names)
    write_stimulus_file(events, out_path)
