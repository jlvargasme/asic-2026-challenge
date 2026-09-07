"""
backtrace.py: visualize everything that structurally influences one
output of a decompiler.py-rendered Verilog file -- e.g. puzzle.v's own
`success` -- by tracing backward through instance connectivity, one
cluster_type_N instance at a time, down to the design's own true
primary inputs.

This is a REPLACEMENT for the automated z3 backward-search chain this
project tried earlier (decompiler.backtrack_inputs/backtrack_sequence,
clustering._solve_sequential_cluster/find_cluster_sequence_for_output,
clustering.find_chip_inputs_for_register_output -- all removed) after
it turned out not to work reliably enough on puzzle.gds to trust (see
HANDOFF.md's own documented limitations on the sequential/multi-cycle
side of that machinery). Rather than trying to automatically SOLVE for
the input VALUES that produce a target (a genuinely hard problem this
project already spent real effort on), this tool answers a much simpler
and more reliable question first: structurally, WHICH instances and
inputs even matter at all -- purely a graph-reachability fact, no
solving, no z3, nothing that can "not work." See cluster_solve.py for
the next step once you know which clusters are actually involved: a
manual, per-cluster combinational solve (still ignoring flip-flop
timing) driven by this same fan-in graph.

Built on netlist.py's yosys-derived JSON netlist (see its own module
docstring for why yosys instead of a hand-written Verilog parser), kept
at CLUSTER granularity (yosys's own `hierarchy`+`proc` passes, no
`flatten`) -- exactly the granularity chip_manipulation.py's own passes
already recovered, and small enough to actually read as a diagram (tens
of nodes, not thousands of raw gates).

Usage: python backtrace.py <verilog_file> <top_module> <output_port> [out.png]
Example: python backtrace.py puzzle.v puzzle success
"""

import networkx as nx
import matplotlib.pyplot as plt

from netlist import load_netlist, modules

PRIMARY_INPUT = "PRIMARY_INPUT"  # pseudo-instance-name prefix for boundary nodes


def _bit_drivers(module):
    """bit_id -> the cell instance name that DRIVES it, from every
    cell's own OUTPUT port connections. A bit with no entry here is
    either a genuine top-level INPUT port bit of `module`, or a
    constant (yosys ties an unconnected/const bit to a string "0"/"1"/
    "x"/"z" instead of an integer bit ID -- those never match any driver
    or port-bit lookup below and are silently treated as a dead end,
    which is the correct behavior for them: nothing upstream to trace)."""
    driver = {}
    for inst_name, cell in module.cells.items():
        directions = cell.get("port_directions", {})
        for port, bits in cell["connections"].items():
            if directions.get(port) != "output":
                continue
            for b in bits:
                driver[b] = inst_name
    return driver


def instance_fanin(module, target_port, target_bit_index=None):
    """Every cell instance (and primary-input port bit) in `module`
    reachable backward from `target_port`'s own bit(s) -- all of them,
    or just `target_bit_index` if given -- by following each bit to
    whichever instance drives it, then that instance's OWN input bits,
    recursively. Purely structural: no cell semantics are consulted at
    all, just connectivity.

    BFS is measured in INSTANCE hops, not bit hops: the target itself is
    depth 0, whatever instance directly drives target_port's own bit(s)
    is depth 1, that instance's own upstream instances are depth 2, etc.
    -- so an instance discovered via more than one path always gets the
    SHORTEST hop count to the target, and every edge in the returned
    graph connects nodes exactly one hop apart (needed for the diagram's
    own layered layout to place nodes -- and their edges -- consistently;
    an earlier version conflated "bit" and "instance" depth and ended up
    drawing some instances' own direct dependencies as if they were the
    TARGET's own, one layer too shallow).

    Returns (instances, primary_inputs, edges, depth):
      - instances: {inst_name} reached.
      - primary_inputs: {(port_name, bit_index)} -- top-level INPUT
        ports of `module` reached at the boundary.
      - edges: {(consumer, driver)} meaning consumer's own input reads
        driver's own output -- both consumer and driver are either an
        instance name, ("TARGET", target_port), or (PRIMARY_INPUT,
        port_name, bit_index).
      - depth: {node: int}, instance-hop distance from the target.
    """
    driver_of = _bit_drivers(module)
    bit_to_port = {}
    for name, info in module.ports.items():
        for i, b in enumerate(info["bits"]):
            bit_to_port[b] = (name, i)

    target_bits = module.port_bits(target_port)
    if target_bit_index is not None:
        target_bits = [target_bits[target_bit_index]]

    target_node = ("TARGET", target_port)
    instances, primary_inputs, edges = set(), set(), set()
    depth = {target_node: 0}
    expanded = set()
    frontier_bits = {target_node: target_bits}
    frontier = [target_node]
    while frontier:
        next_frontier = []
        for node in frontier:
            if node in expanded:
                continue
            expanded.add(node)
            d = depth[node]
            for bit in frontier_bits.get(node, ()):
                inst = driver_of.get(bit)
                if inst is None:
                    port_bit = bit_to_port.get(bit)
                    if port_bit is not None and module.ports[port_bit[0]]["direction"] == "input":
                        pnode = (PRIMARY_INPUT, *port_bit)
                        primary_inputs.add(port_bit)
                        depth[pnode] = min(depth.get(pnode, d + 1), d + 1)
                        edges.add((node, pnode))
                    continue
                instances.add(inst)
                edges.add((node, inst))
                nd = d + 1
                if inst not in depth or nd < depth[inst]:
                    depth[inst] = nd
                if inst not in expanded:
                    cell = module.cells[inst]
                    directions = cell.get("port_directions", {})
                    frontier_bits[inst] = [
                        b for port, bits in cell["connections"].items()
                        if directions.get(port) == "input" for b in bits
                    ]
                    next_frontier.append(inst)
        frontier = next_frontier
    return instances, primary_inputs, edges, depth


def draw_fanin(module, target_port, instances, primary_inputs, edges, depth,
               out_path="fanin.png", title=None):
    """Render the fan-in graph as a layered diagram -- the target on the
    left, its own primary inputs on the right, every instance in
    between positioned by its own BFS depth -- and save it as a PNG,
    matching this project's existing plot_clusters()/plot_utilities.py
    convention of matplotlib PNGs rather than an interactive viewer."""
    target_node = ("TARGET", target_port)
    g = nx.DiGraph()
    g.add_node(target_node, kind="target", layer=0)
    for inst in instances:
        g.add_node(inst, kind="instance", layer=depth.get(inst, 1))
    for port_bit in primary_inputs:
        node = (PRIMARY_INPUT, *port_bit)
        g.add_node(node, kind="input", layer=depth.get(node, 1))
    for consumer, driver in edges:
        g.add_edge(consumer, driver)

    max_layer = max(depth.values()) if depth else 0
    pos = nx.multipartite_layout(g, subset_key="layer", align="vertical")
    # multipartite_layout puts layer 0 on the left by default with
    # increasing x per layer already -- flip so the TARGET (layer 0)
    # reads on the right and primary inputs (deepest layer) on the left,
    # matching the natural left-to-right "inputs flow to output" reading
    # order.
    pos = {n: (-x, y) for n, (x, y) in pos.items()}

    fig_width = max(10, 2.2 * (max_layer + 1))
    fig_height = max(6, 0.45 * len(g.nodes))
    fig, ax = plt.subplots(figsize=(fig_width, fig_height))

    color_by_kind = {"target": "#d62728", "instance": "#1f77b4", "input": "#2ca02c"}
    node_colors = [color_by_kind[g.nodes[n]["kind"]] for n in g.nodes]

    def label(n):
        if n[0] == "TARGET":
            return f"{n[1]}\n(target)"
        if n[0] == PRIMARY_INPUT:
            _, port, bit = n
            return f"{port}[{bit}]" if len(module.port_bits(port)) > 1 else port
        cell_type = module.cells[n]["type"]
        return f"{n}\n({cell_type})"

    nx.draw_networkx_edges(g, pos, ax=ax, edge_color="#999999", arrows=True,
                            arrowsize=12, connectionstyle="arc3,rad=0.05")
    nx.draw_networkx_nodes(g, pos, ax=ax, node_color=node_colors, node_size=1400, alpha=0.9)
    nx.draw_networkx_labels(g, pos, labels={n: label(n) for n in g.nodes}, ax=ax, font_size=7)

    legend_handles = [
        plt.Line2D([0], [0], marker="o", color="w", label=lbl, markerfacecolor=c, markersize=10)
        for lbl, c in [("target", color_by_kind["target"]),
                       ("cluster instance", color_by_kind["instance"]),
                       ("primary input", color_by_kind["input"])]
    ]
    ax.legend(handles=legend_handles, loc="upper left", bbox_to_anchor=(1.02, 1.0))
    ax.set_title(title or f"{module.name}.{target_port}: fan-in cone "
                          f"({len(instances)} instance(s), {len(primary_inputs)} primary input(s))")
    ax.axis("off")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_path}")
    return out_path


def backtrace(verilog_path, top_module, target_port, out_path=None, target_bit_index=None):
    """End-to-end: load the netlist, compute target_port's own fan-in
    cone within `top_module`, draw it, and return
    (instances, primary_inputs, edges) for further use (e.g. as the
    starting point for cluster_solve.py's own iterative combinational
    solve)."""
    data = load_netlist(verilog_path, top_module)
    mods = modules(data)
    top = mods[top_module]
    instances, primary_inputs, edges, depth = instance_fanin(top, target_port, target_bit_index)

    print(f"{top_module}.{target_port}: {len(instances)} instance(s) in the fan-in cone, "
          f"{len(primary_inputs)} primary input(s) reached")
    by_type = {}
    for inst in instances:
        t = top.cells[inst]["type"]
        by_type.setdefault(t, []).append(inst)
    for t, insts in sorted(by_type.items()):
        print(f"  {t}: {len(insts)} instance(s) -- {sorted(insts)}")
    print(f"  primary inputs: {sorted(f'{p}[{i}]' for p, i in primary_inputs)}")

    out_path = out_path or f"{top_module}_{target_port}_fanin.png"
    draw_fanin(top, target_port, instances, primary_inputs, edges, depth, out_path)
    return instances, primary_inputs, edges


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 4:
        print("usage: python backtrace.py <verilog_file> <top_module> <output_port> [out.png]")
        raise SystemExit(1)
    verilog_file, top_module_name, target = sys.argv[1:4]
    out = sys.argv[4] if len(sys.argv) > 4 else None
    backtrace(verilog_file, top_module_name, target, out)
