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

import sys

from chip import Chip
from clustering import (
    build_chip_z3_circuit,
    cluster_chip,
    find_chip_inputs_for_output,
    label_free_nets,
    plot_clusters,
)
from decompiler import build_ast, render_verilog


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

    plot_clusters(chip, clusters, out_path=f"{top_cell_name}_clusters.png")  # prints its own "wrote ..." line

    # decompile_leaf()/_collapse_transparent_buffers() mutate chip.instances
    # and each Cluster's own instances list in place (see
    # decompiler._collapse_transparent_buffers' own docstring) -- run this
    # AFTER plot_clusters() and the z3 solve below, which both want every
    # instance (including the buffers this collapses) still present.
    leaf_modules, cluster_modules, top = build_ast(chip, clusters, force_tier2=force_tier2)
    verilog_path = f"{top_cell_name}.v"
    with open(verilog_path, "w") as f:
        f.write(render_verilog(leaf_modules, cluster_modules, top) + "\n")
    print(f"wrote {verilog_path} ({len(leaf_modules)} leaf module(s), "
          f"{len(cluster_modules)} cluster module type(s))")

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
        print(f"  {dict(zip(display_names, values))}")
    if len(solutions) > 20:
        print(f"  ... and {len(solutions) - 20} more")


if __name__ == "__main__":
    main()
