"""
main.py: run this project's whole pipeline end to end on one GDS --
cluster recovery + diagram, Verilog decompilation, and a joint solve for
chip primary-input (and/or register-state) assignments that drive EVERY
one of the chip's own combinational output nets to 1 at once.

The joint solve is built from find_chip_inputs_for_output()'s dict target
support (see clustering.py), with the target dict built automatically
from however many chip_output_nets build_chip_z3_circuit() actually
finds, rather than assuming there's exactly one (this project's own
adder_demo has just one -- "S" -- but puzzle.gds isn't guaranteed to).

Usage: python main.py [gds_file] [top_cell_name] [epsilon_scale] [--force-tier2]
"""

import re
import sys

from chip import Chip
from clustering import (
    build_chip_z3_circuit,
    cluster_chip,
    find_chip_inputs_for_output,
    label_free_nets,
    plot_clusters,
)
from decompiler import decompile

from plot_utilities import plot_clusters_optimized


_BIT_NAME_RE = re.compile(r"^([A-Za-z_]+)#(\d+)$")


def _decode_bit_groups(solution, target_sum=496):
    """A solution's display names sometimes come out as bit-indexed buses
    (e.g. 'A#0'..'A#7', 'B#0'..'B#7' for adder_demo's own A/B register
    inputs) rather than single named nets. Group those by prefix, rebuild
    each as a full binary number (bit i = that bit's value, #0 = LSB),
    and -- since adder_demo's own comparator496 is checking exactly
    sum(buses) == 496 -- report whether every 2-bus solution's buses sum
    to target_sum. Returns a list of (prefix, value, binary_string) plus,
    when exactly two groups are found, the (a+b, a+b == target_sum) pair;
    returns ([], None) if the solution has no bit-indexed names at all."""
    groups = {}
    for name, bit in solution.items():
        m = _BIT_NAME_RE.match(name)
        if not m:
            continue
        prefix, idx = m.group(1), int(m.group(2))
        groups.setdefault(prefix, {})[idx] = bit

    decoded = []
    for prefix, bits in sorted(groups.items()):
        width = max(bits) + 1
        value = sum(bits.get(i, 0) << i for i in range(width))
        binary = "".join(str(bits.get(i, 0)) for i in reversed(range(width)))
        decoded.append((prefix, value, binary))

    sum_check = None
    if len(decoded) == 2:
        total = decoded[0][1] + decoded[1][1]
        sum_check = (total, total == target_sum)
    return decoded, sum_check


def main():
    args = sys.argv[1:]
    force_tier2 = "--force-tier2" in args
    args = [a for a in args if a != "--force-tier2"]

    gds_file = args[0] if len(args) > 0 else "./warmup/04_final.gds"
    top_cell_name = args[1] if len(args) > 1 else "adder_demo"
    epsilon_scale = float(args[2]) if len(args) > 2 else 1.0

    chip = Chip(gds_file, top_cell_name)
    clusters = cluster_chip(chip, epsilon_scale=epsilon_scale)
    print(f"{top_cell_name}: {len(chip.instances)} instance(s), {len(clusters)} cluster(s)")

    # plot_clusters(chip, clusters, out_path=f"{top_cell_name}_clusters.png")  # prints its own "wrote ..." line

    # decompile_leaf()/_collapse_transparent_buffers() mutate chip.instances
    # and each Cluster's own instances list in place (see
    # chip_manipulation._collapse_transparent_buffers' own docstring) --
    # run this AFTER plot_clusters() and the z3 solve below, which both
    # want every instance (including the buffers this collapses) still
    # present.
    leaf_modules, cluster_modules, top, verilog = decompile(
        chip, clusters, out_path=f"{top_cell_name}.v", force_tier2=force_tier2,
    )
    print(f"({len(leaf_modules)} leaf module(s), {len(cluster_modules)} cluster module type(s))")

    # plot_clusters_optimized(chip, clusters, out_path=f"{top_cell_name}_clusters_optimized.png")

    _, _, _, chip_output_nets = build_chip_z3_circuit(chip, clusters)
    if not chip_output_nets:
        print("\nno net in this chip's combinational logic reads as a chip-level output -- nothing to solve for")
        return

    target = {net: 1 for net in chip_output_nets}
    print(f"\nsolving for all {len(target)} output net(s) == 1: {sorted(target)}")

    free_nets, solutions = find_chip_inputs_for_output(chip, clusters, target)
    labels = label_free_nets(chip, clusters, free_nets)
    display_names = [labels[net] for net in free_nets]

    print(f"\n{len(free_nets)} free net(s) (chip primary inputs + sequential-cluster/register "
          f"state -- see build_chip_z3_circuit's docstring): {display_names}")
    print(f"{len(solutions)} assignment(s) drive every output net to 1:")
    for values in solutions[:20]:
        solution = dict(zip(display_names, values))
        print(f"  {solution}")
        decoded, sum_check = _decode_bit_groups(solution)
        if decoded:
            parts = [f"{prefix}={binary} ({value})" for prefix, value, binary in decoded]
            print(f"    -> {', '.join(parts)}", end="")
            if sum_check is not None:
                total, ok = sum_check
                print(f"  sum={total}  {'== 496 OK' if ok else '!= 496 MISMATCH'}")
            else:
                print()
    if len(solutions) > 20:
        print(f"  ... and {len(solutions) - 20} more")


if __name__ == "__main__":
    main()
