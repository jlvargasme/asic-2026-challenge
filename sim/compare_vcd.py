"""Compare the top-level pin value-change traces of two VCD files -- used
both to verify the ground-truth warmup/00_source.v and the
reverse-engineered adder_demo.v produce IDENTICAL waveforms for the same
stimulus (not just matching answers at the specific points
sim/tb_adder_demo.cpp explicitly asserts), and to check puzzle.v's own
O/success trace against a known-good reference VCD (e.g.
example_inputs.vcd) replayed via sim/tb_puzzle_replay.cpp -- see
sim/run_puzzle_replay.sh. Usage: python compare_vcd.py <vcd_a> <vcd_b> [signal ...]

Handles both scalar (1-bit, "0<id>"/"1<id>", no space) and vector
(N-bit, "b<binary> <id>", space-separated) VCD value-change formats, and
a $var declaration with a trailing bit-range ("O [7:0]") -- needed for
puzzle.v's own O[7:0] bus, not just adder_demo's scalar ports.
"""

import re
import sys


def extract_signal_trace(vcd_path, signal_name):
    with open(vcd_path) as f:
        text = f.read()
    m = re.search(
        r"\$var\s+\w+\s+\d+\s+(\S+)\s+" + re.escape(signal_name) + r"(?:\s*\[[^\]]*\])?\s+\$end", text
    )
    if not m:
        raise ValueError(f"{signal_name!r} not found in {vcd_path}")
    sig_id = m.group(1)
    trace = []
    t = 0
    for line in text.splitlines():
        if line.startswith("#"):
            t = int(line[1:])
            continue
        if line[:1] in ("b", "B"):
            rest = line[1:]
            sp = rest.rfind(" ")
            if sp != -1 and rest[sp + 1:] == sig_id:
                trace.append((t, _normalize(rest[:sp])))
        elif len(line) >= 2 and line[0] in "01xz" and line[1:] == sig_id:
            trace.append((t, _normalize(line[0])))
    return trace


def _normalize(value):
    """"00000101" -> 5 (so a zero-padded vector value from one VCD
    writer compares equal to an unpadded "101" from another); "x"/"z"
    (or any non-binary token) passes through unchanged."""
    if value and all(c in "01" for c in value):
        return int(value, 2)
    return value


def main():
    vcd_a, vcd_b = sys.argv[1], sys.argv[2]
    signals = sys.argv[3:] or ["clk", "A", "B", "en", "rst_n", "S"]
    all_ok = True
    for sig in signals:
        a = extract_signal_trace(vcd_a, sig)
        b = extract_signal_trace(vcd_b, sig)
        ok = a == b
        all_ok &= ok
        print(f"{sig}: {'IDENTICAL' if ok else 'DIFFERENT'} ({len(a)} vs {len(b)} transition(s))")
        if not ok:
            print(f"  {vcd_a}: {a}")
            print(f"  {vcd_b}: {b}")
    print("ALL SIGNALS IDENTICAL" if all_ok else "MISMATCH FOUND")
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
