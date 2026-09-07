"""
Decompiler: turn a chip.Chip + clustering.py's recovered clusters into a
best-effort <chip_name>.v Verilog file. The last stage of this project's
bottom-up pipeline (geometry -> transistors -> cells -> chip netlist ->
clusters -> RTL text).

This module only does two things:

  - decompile(): the end-to-end orchestrator. Builds the AST (via
    chip_manipulation.build_ast() -- leaf decompilation plus every AST
    simplification pass: buffer collapsing, cluster-type dedup, bus
    grouping, z3 combinational flattening) and renders it to Verilog text,
    optionally writing it to a file.
  - render_verilog() and its own helpers: pure AST -> Verilog-text
    emission, no analysis of any kind. Everything that decides WHAT the
    AST looks like (which passes to run, in what order, what a leaf
    cell's own truth table minimizes to, when a cluster's own combinational
    logic gets flattened into z3-minimized assigns instead of sky130 gate
    instances) lives in chip_manipulation.py instead -- see its own module
    docstring for the full pipeline this module's decompile() just calls
    through to.
"""

from chip_manipulation import build_ast
from clustering import cluster_chip


# ---------------------------------------------------------------------------
# Verilog emission
# ---------------------------------------------------------------------------


def _emit_leaf_module(leaf):
    out_kw = "output reg" if leaf.kind == "sequential" else "output"
    ports = [f"    input {p}," for p in leaf.input_labels]
    ports += [f"    {out_kw} {p}," for p in leaf.output_labels]
    if ports:
        ports[-1] = ports[-1].rstrip(",")

    lines = [f"module {leaf.name} (", *ports, ");"]
    lines.append(f"    // {leaf.tier}")

    if leaf.kind == "combinational":
        for out, expr in leaf.comb_exprs.items():
            lines.append(f"    assign {out} = {expr};")
    elif leaf.kind == "sequential":
        pm = leaf.seq_pinmap
        edge_kw = "posedge" if pm.clk_edge == "pos" else "negedge"
        sens = [f"{edge_kw} {pm.clk}"]
        async_overrides = [o for o in pm.overrides if o.is_async]
        for o in async_overrides:
            sens.append(f"{'posedge' if o.active_value == 1 else 'negedge'} {o.pin}")

        lines.append(f"    always @({' or '.join(sens)}) begin")
        for i, o in enumerate(async_overrides):
            cond = o.pin if o.active_value == 1 else f"!{o.pin}"
            lines.append(f"        {'if' if i == 0 else 'else if'} ({cond}) begin")
            for out_name in leaf.output_labels:
                if out_name in o.forced:
                    lines.append(f"            {out_name} <= 1'b{o.forced[out_name]};")
            lines.append("        end")
        lines.append(f"        {'else ' if async_overrides else ''}begin")
        for out_name in leaf.output_labels:
            rhs = pm.data if pm.outputs.get(out_name, "normal") == "normal" else f"~{pm.data}"
            lines.append(f"            {out_name} <= {rhs};")
        lines.append("        end")
        lines.append("    end")
    else:
        for out in leaf.output_labels:
            lines.append(f"    assign {out} = 1'bx;")

    lines.append("endmodule")
    return "\n".join(lines)


def _port_decl_str(direction, decl):
    width = "" if decl.width == 1 else f"[{decl.width - 1}:0] "
    return f"    {direction} {width}{decl.name},"


def _conn_str(net):
    """A scalar net identifier connects directly; a bus (list of net
    identifiers, index 0 = bit 0) connects via Verilog concatenation,
    MSB-first as {...} syntax requires -- only still needed where the far
    end isn't a matching vector wire (see chip_manipulation._group_top_
    wires(), which replaces this with a direct name reference wherever it
    applies)."""
    if isinstance(net, str):
        return net
    return "{" + ", ".join(reversed(net)) + "}"


def _wire_decl_str(decl):
    width = "" if decl.width == 1 else f"[{decl.width - 1}:0] "
    return f"    wire {width}{decl.name};"


def _emit_structural_module(name, input_ports, output_ports, wires, instances, comb_exprs=None):
    ports = [_port_decl_str("input", p) for p in input_ports] + [_port_decl_str("output", p) for p in output_ports]
    if ports:
        ports[-1] = ports[-1].rstrip(",")
    lines = [f"module {name} (", *ports, ");"]
    for w in wires:
        lines.append(_wire_decl_str(w))
    if comb_exprs:
        # every net a REMOVABLE (non-sequential) instance used to compute,
        # within this cluster, got replaced by minimized boolean logic --
        # see chip_manipulation.py's "Cluster-level combinational
        # flattening" pass. `instances` below may still be non-empty: a
        # mixed cluster's own KEPT sequential instances (flip-flops,
        # latches) are never flattened and are emitted right after this.
        lines.append("    // combinational logic flattened via z3 (see chip_manipulation.py)")
        for out, expr in comb_exprs.items():
            lines.append(f"    assign {out} = {expr};")
    for inst in instances:
        conns = ", ".join(f".{pin}({_conn_str(net)})" for pin, net in inst.port_map.items())
        lines.append(f"    {inst.module_name} {inst.inst_name} ({conns});")
    lines.append("endmodule")
    return "\n".join(lines)


def render_verilog(leaf_modules, cluster_modules, top):
    # a leaf type with zero surviving instantiations (e.g. a transparent
    # buffer collapsed out of every cluster body, or every cluster it
    # appeared in got flattened away by the z3 combinational pass -- see
    # chip_manipulation.py) has nothing left to instantiate it, so its own
    # module definition would just be dead code in the file.
    used = {inst.module_name for cm in cluster_modules for inst in cm.instances}

    dropped = sorted(name for name in leaf_modules if leaf_modules[name].name not in used)
    if dropped:
        print(f"omitted {len(dropped)} leaf module(s) with zero surviving instantiations: {dropped}")

    parts = [f"// Auto-generated by decompiler.py from {top.name} -- do not hand-edit."]
    for name in sorted(leaf_modules):
        if leaf_modules[name].name in used:
            parts.append(_emit_leaf_module(leaf_modules[name]))
    for cm in cluster_modules:
        parts.append(_emit_structural_module(
            cm.name, cm.input_ports, cm.output_ports, cm.internal_wires, cm.instances, cm.comb_exprs,
        ))
    parts.append(_emit_structural_module(top.name, top.input_ports, top.output_ports, top.wires, top.instances))
    return "\n\n".join(parts)


# ---------------------------------------------------------------------------
# Orchestrator
# ---------------------------------------------------------------------------


def decompile(chip, clusters, out_path=None, force_tier2=False):
    """Full decompile-to-Verilog pipeline: chip_manipulation.build_ast()
    (leaf decompilation, then every AST simplification pass -- buffer
    collapsing, cluster-type dedup, bus grouping, z3 combinational
    flattening) followed by render_verilog(). Writes `out_path` if given
    (defaults to "<top_cell_name>.v") and always returns the AST pieces
    alongside the rendered text.

    Args:
        chip: the chip.Chip to decompile.
        clusters: list of Cluster, e.g. from cluster_chip(chip).
        out_path: where to write the Verilog file; None writes nothing
            (caller can still use the returned `verilog` text directly).
        force_tier2: passed through to build_ast() -- see
            chip_manipulation.decompile_leaf()'s own docstring.

    Returns:
        (leaf_modules, cluster_modules, top, verilog) -- the same AST
        pieces build_ast() returns, plus the rendered Verilog text.
    """
    leaf_modules, cluster_modules, top = build_ast(chip, clusters, force_tier2=force_tier2)
    verilog = render_verilog(leaf_modules, cluster_modules, top)

    if out_path is not None:
        with open(out_path, "w") as f:
            f.write(verilog + "\n")
        print(f"wrote {out_path}")

    return leaf_modules, cluster_modules, top, verilog


if __name__ == "__main__":
    import sys

    from chip import Chip

    # python decompiler.py [gds_file] [top_cell_name] [epsilon_scale] [--force-tier2]
    args = sys.argv[1:]
    force_tier2 = "--force-tier2" in args
    args = [a for a in args if a != "--force-tier2"]

    gds_file = args[0] if len(args) > 0 else "./warmup/04_final.gds"
    top_cell_name = args[1] if len(args) > 1 else "adder_demo"
    epsilon_scale = float(args[2]) if len(args) > 2 else 1.0

    chip = Chip(gds_file, top_cell_name)
    clusters = cluster_chip(chip, epsilon_scale=epsilon_scale)
    print(f"{top_cell_name}: {len(chip.instances)} instance(s), {len(clusters)} cluster(s)")
    if force_tier2:
        print("(--force-tier2: sequential cells will be characterized structurally, "
              "known-cell table used only to validate the result)")

    leaf_modules, cluster_modules, top, verilog = decompile(
        chip, clusters, out_path=f"{top_cell_name}.v", force_tier2=force_tier2,
    )

    print(f"\n{len(leaf_modules)} leaf module(s):")
    for name in sorted(leaf_modules):
        leaf = leaf_modules[name]
        print(f"  {name}: {leaf.kind} ({leaf.tier})")

    print(f"\n{len(cluster_modules)} cluster module type(s) recovered from {len(clusters)} cluster(s):")
    for cm in cluster_modules:
        n_chip_insts = sum(1 for i in top.instances if i.module_name == cm.name)
        ports = ", ".join(f"{p.name}[{p.width}]" if p.width > 1 else p.name for p in cm.input_ports + cm.output_ports)
        if cm.comb_exprs is not None:
            body = f"{len(cm.comb_exprs)} z3-flattened assign(s)"
            if cm.instances:
                body += f" + {len(cm.instances)} surviving gate/register instance(s)"
        else:
            body = f"{len(cm.instances)} gate(s)"
        print(f"  {cm.name}: {body}, {n_chip_insts} chip instantiation(s), ports: {ports}")
