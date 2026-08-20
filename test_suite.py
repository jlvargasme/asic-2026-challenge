"""
Cross-validate cell.py's two independent simulation models -- PySpice
(Cell.simulate, real device physics via ngspice) and z3 (Cell.simulate_z3,
ideal switch equalities) -- against each other, across every input
combination, for every logic cell defined in a GDS file.

Both models are built from the exact same extracted transistor list
(Cell.transistors), but evaluate it in completely different ways. If they
ever disagree for a combinational cell, that means the shared extraction
(or one of the two models) has a bug -- this project has already found
several real bugs exactly this way (net_trace.py's missing poly/li1/met1
self-connectivity, gds_utils.py's pinched-diffusion-region splitting),
just by hand instead of automatically. This is that check, generalized
to every logic cell in the file instead of whichever one happened to be
under investigation.

"Logic cell" here means the same geometric test chip.py's own
_is_logic_cell() uses: does the cell have at least one pin that traces to
a real transistor gate or diffusion terminal, as opposed to only power
rails (VIA_*/decap/welltap cells all fail this and are skipped) -- not a
name-based guess.

Sequential cells (anything with genuine internal state, e.g. dfrtp_2) are
EXPECTED to raise partway through this check rather than pass cleanly --
simulate_z3()'s ideal-switch model has no notion of "previous state", so
for whichever input combination puts such a cell into "hold", its own
constraints correctly leave the output either ambiguous (both 0 and 1
satisfiable) or unsatisfiable (no consistent value at all). Those cells
are reported as ERROR, separately from genuine MISMATCHes, rather than
silently skipped or treated the same as a real extraction bug.

That "ERROR" result is a legitimate signal ("this cell has state"), but
it isn't a real check of a sequential cell's behavior -- there's nothing
to cross-validate simulate_z3() against once a cell's output depends on
history, not just its current inputs. SEQUENTIAL_TESTS below covers that
gap: each entry is a stimulus sequence (real SPICE transient simulation,
via Cell.simulate_transient(), with actual clock edges over time) plus
the expected output at specific probe times, checked against what the
named cell type is actually *supposed* to do -- reset behavior, edge-
triggered capture, and hold. This is the one place in this project that's
deliberately keyed by cell name rather than derived from geometry: an
edge-triggered flip-flop's specific timing contract (active-low vs
active-high reset, positive- vs negative-edge, synchronous vs
asynchronous) isn't something geometry alone determines -- it has to be
checked against a spec for that specific cell, the same way any datasheet
based hardware test would be.
"""

import sys
from itertools import product

import gdstk

from cell import Cell

MAX_INPUTS = 10  # 2**10 = 1024 combinations; guards against a pathological cell

_NS = 1e-9

# sky130_fd_sc_hd__dfrtp_2: D flip-flop, active-low asynchronous RESET_B,
# positive-edge-triggered. Each event gives CLK/D/RESET_B's values from
# that time on; each expectation is (probe_time, {output: expected}, why).
#
# An initial pass at 2ns spacing / 0.1ns edges produced a spurious
# capture on a FALLING edge that should have been ignored -- first
# suspected to be a real master/slave-latch race from too fast a clock
# edge (slowing the edge to 5ns did make it disappear). That theory
# turned out to be wrong: sweeping edge_time from 0.1ns to 8ns with
# simulate_transient()'s ORIGINAL step_time default (edge_time/10) failed
# at every single edge speed, including ones far too slow for a real
# race condition -- the actual cause was step_time itself being too
# coarse a reporting grid for ngspice's transient analysis, occasionally
# landing a probe on a point interpolated across a transition rather
# than a genuinely settled sample (see simulate_transient()'s own
# step_time docs, now edge_time/100 by default). With that fixed, this
# exact stimulus is correct at every edge speed tested, including the
# original fast 0.1ns edge -- so the spacing/edge here is simply a
# readable one, not a workaround for anything.
_DFRTP_2_EDGE = 1 * _NS
_DFRTP_2_EVENTS = [
    (0 * _NS, {"CLK": 0, "D": 0, "RESET_B": 0}),
    (20 * _NS, {"CLK": 0, "D": 1, "RESET_B": 1}),
    (40 * _NS, {"CLK": 1, "D": 1, "RESET_B": 1}),
    (60 * _NS, {"CLK": 1, "D": 0, "RESET_B": 1}),
    (80 * _NS, {"CLK": 0, "D": 0, "RESET_B": 1}),
    (100 * _NS, {"CLK": 1, "D": 0, "RESET_B": 1}),
    (120 * _NS, {"CLK": 1, "D": 1, "RESET_B": 1}),
    (140 * _NS, {"CLK": 0, "D": 1, "RESET_B": 1}),
    (160 * _NS, {"CLK": 1, "D": 1, "RESET_B": 1}),
    (180 * _NS, {"CLK": 1, "D": 1, "RESET_B": 0}),
]
_DFRTP_2_EXPECTATIONS = [
    (10 * _NS, {"Q": 0}, "asynchronous reset forces Q=0"),
    (30 * _NS, {"Q": 0}, "reset released, D=1 but no clock edge yet -- Q still holds 0"),
    (50 * _NS, {"Q": 1}, "rising edge captures D=1"),
    (70 * _NS, {"Q": 1}, "D changed to 0 without a clock edge -- Q holds at 1"),
    (90 * _NS, {"Q": 1}, "falling edge -- positive-edge-triggered FF ignores it, Q holds at 1"),
    (110 * _NS, {"Q": 0}, "rising edge captures D=0"),
    (130 * _NS, {"Q": 0}, "D changed to 1 without a clock edge -- Q holds at 0"),
    (150 * _NS, {"Q": 0}, "falling edge -- ignored, Q holds at 0"),
    (170 * _NS, {"Q": 1}, "rising edge captures D=1"),
    (190 * _NS, {"Q": 0}, "asynchronous reset overrides Q=1 immediately, without needing a clock edge"),
]

SEQUENTIAL_TESTS = {
    "sky130_fd_sc_hd__dfrtp_2": (_DFRTP_2_EVENTS, _DFRTP_2_EXPECTATIONS, _DFRTP_2_EDGE),
}


def find_logic_cells(gds_file):
    """Every distinct LEAF cell definition in `gds_file` that Cell can
    build and that has at least one functional (non-power) pin.

    "Leaf" is decided structurally, not by name: a cell with any
    references of its own (e.g. this project's example "adder_demo",
    which places 1099 standard-cell instances) is a routed top-level
    design, not a single cell -- Cell/count_transistors() would recurse
    through its entire placed hierarchy (depth=None) and try to treat the
    whole chip as "one cell's transistors", which is both wrong and, for
    a real design, effectively never finishes. Only cells with zero
    references of their own are candidates here.
    """
    library = gdstk.read_gds(gds_file)
    leaf_names = sorted({c.name for c in library.cells if not c.references})

    logic_cells = []
    for name in leaf_names:
        try:
            cell = Cell(gds_file, name)
        except Exception as e:
            print(f"skip {name}: couldn't build a Cell ({e!r})")
            continue
        if cell.input_labels or cell.output_labels:
            logic_cells.append(cell)
    return logic_cells


def cross_validate(cell):
    """Compare cell.simulate() against cell.simulate_z3() for every input
    combination.

    Returns (matches, mismatches, errors):
        matches: list of input combos where both models agreed.
        mismatches: list of (combo, spice_result, z3_result) where both
            computed successfully but disagreed -- a genuine bug signal
            for a combinational cell.
        errors: list of (combo, which_model, exception) where either
            model raised -- expected for a sequential cell, otherwise
            also worth investigating.
    """
    matches, mismatches, errors = [], [], []
    for combo in product((0, 1), repeat=len(cell.input_labels)):
        try:
            spice_result = cell.simulate(combo)
        except Exception as e:
            errors.append((combo, "simulate", e))
            continue
        try:
            z3_result = cell.simulate_z3(combo)
        except Exception as e:
            errors.append((combo, "simulate_z3", e))
            continue

        if spice_result == z3_result:
            matches.append(combo)
        else:
            mismatches.append((combo, spice_result, z3_result))
    return matches, mismatches, errors


def run_sequential_test(cell, events, expectations, edge_time=1e-10):
    """Run a real transient simulation through `events` and check `cell`'s
    output(s) at each expectation's probe time -- the sequential-cell
    equivalent of a truth table check, except against the specific
    cell's intended state-transition behavior rather than a second
    independent model (there's no stateless model to cross-validate
    against once a cell's output depends on history, not just its
    current inputs -- see cross_validate()'s own ERROR case).

    Returns (passed, failures) -- failures is a list of
    (probe_time, description, expected, actual), each a dict of
    {output_label: value}.
    """
    probe_times = [t for t, _, _ in expectations]
    results = cell.simulate_transient(events, probe_times, edge_time=edge_time)

    failures = []
    for (t, expected, description), actual_values in zip(expectations, results):
        actual = dict(zip(cell.output_labels, actual_values))
        expected_full = {name: expected.get(name, actual[name]) for name in cell.output_labels}
        if actual != expected_full:
            failures.append((t, description, expected_full, actual))
    return not failures, failures


def run(gds_file):
    logic_cells = find_logic_cells(gds_file)
    print(f"\n{len(logic_cells)} logic cell(s) found in {gds_file}\n")

    passed, mismatched, errored, skipped = [], [], [], []
    sequential_passed, sequential_failed = [], []
    for cell in logic_cells:
        n = len(cell.input_labels)
        if n > MAX_INPUTS:
            print(f"{cell.cell_name}: SKIPPED ({n} inputs, {2 ** n} combinations -- too many to brute-force)")
            skipped.append(cell.cell_name)
            continue

        matches, mismatches, errors = cross_validate(cell)
        total = len(matches) + len(mismatches) + len(errors)

        if errors:
            status = "ERROR"
            errored.append(cell.cell_name)
        elif mismatches:
            status = "MISMATCH"
            mismatched.append(cell.cell_name)
        else:
            status = "PASS"
            passed.append(cell.cell_name)

        print(f"{cell.cell_name} ({', '.join(cell.input_labels)} -> {', '.join(cell.output_labels)}): "
              f"{status}  ({len(matches)}/{total} match, {len(mismatches)} mismatch, {len(errors)} error)")
        for combo, spice_result, z3_result in mismatches:
            given = dict(zip(cell.input_labels, combo))
            print(f"    MISMATCH inputs={given}  spice={spice_result}  z3={z3_result}")
        for combo, which, e in errors:
            given = dict(zip(cell.input_labels, combo))
            print(f"    ERROR inputs={given}  in {which}(): {e}")

        if cell.cell_name in SEQUENTIAL_TESTS:
            events, expectations, edge_time = SEQUENTIAL_TESTS[cell.cell_name]
            ok, failures = run_sequential_test(cell, events, expectations, edge_time=edge_time)
            if ok:
                sequential_passed.append(cell.cell_name)
                print(f"    sequential test: PASS  ({len(expectations)}/{len(expectations)} probe(s) matched)")
            else:
                sequential_failed.append(cell.cell_name)
                print(f"    sequential test: FAIL  ({len(expectations) - len(failures)}/{len(expectations)} probe(s) matched)")
                for t, description, expected, actual in failures:
                    print(f"      t={t * 1e9:.0f}ns  {description}")
                    print(f"        expected={expected}  actual={actual}")

    print()
    print(f"summary: {len(passed)} passed, {len(mismatched)} mismatched, "
          f"{len(errored)} errored (likely sequential), {len(skipped)} skipped")
    if mismatched:
        print(f"  mismatched: {', '.join(mismatched)}")
    if errored:
        print(f"  errored (check whether these are genuinely sequential): {', '.join(errored)}")
    if sequential_passed or sequential_failed:
        print(f"sequential tests: {len(sequential_passed)} passed, {len(sequential_failed)} failed")
        if sequential_failed:
            print(f"  failed: {', '.join(sequential_failed)}")

    # a combinational MISMATCH or a failed sequential test are real
    # failures; a combinational ERROR alone is not (expected for any
    # sequential cell that has no matching entry in SEQUENTIAL_TESTS yet).
    return not mismatched and not sequential_failed


if __name__ == "__main__":
    gds_file = sys.argv[1] if len(sys.argv) > 1 else "./warmup/04_final.gds"
    ok = run(gds_file)
    sys.exit(0 if ok else 1)
