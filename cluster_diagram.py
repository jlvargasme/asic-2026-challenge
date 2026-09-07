"""
cluster_diagram.py: for each recovered cluster, plot every flip-flop it
contains -- its own Q output, its D input already resolved to the
minimized boolean expression chip_manipulation.py's own combinational
flattening/inlining passes computed for it, and its CLK plus any
async-override pin(s) (RESET_B/SET_B) -- plus any purely combinational
output port not already covered by a flip-flop's own Q, resolved the
same way. Every pin is labeled by its own ROLE (D/CLK/RESET_B/Q, ...),
not just a raw net name.

Built directly from chip_manipulation.py's own AST objects
(LeafModule/ClusterModule/InstanceCall, from build_ast()) -- not by
re-parsing rendered Verilog text:

  - InstanceCall.port_map gives each flip-flop instance's own actual
    net connections.
  - LeafModule.seq_pinmap (a SequentialPinMap) gives that flip-flop
    TYPE's own recovered pin ROLES (which local pin name is the data
    input, the clock, an async override, and its polarity) -- so this
    reads pin ROLES from the AST's own recovered semantics, never a
    hardcoded/guessed pin-name convention like "assume the data pin is
    always literally called 'D'".
  - ClusterModule.comb_exprs gives the already-minimized boolean
    expression for any net Pass 4/5 flattened -- exactly the "resolved
    to its simplified form" this module's own docstring describes; a
    net NOT in comb_exprs (a direct wire straight from a port or
    another flip-flop's own Q, no gates in between) is left as its own
    plain net name, which is itself already the simplest possible form.

One PNG per cluster (see plot_all_clusters()), not one combined figure,
so each stays legible even for a cluster with two dozen flip-flops.

Usage (see the module's own __main__ for a runnable example against
adder_demo, the one target this can be validated against directly --
see the module's own top-level docstring on chip.Chip/build_ast for why
this needs a live decompile pipeline run, not just a rendered .v file,
and is therefore only ever run directly on a GDS the caller has
themselves loaded, never puzzle.gds by this project's own convention):

    from chip import Chip
    from clustering import cluster_chip
    from chip_manipulation import build_ast
    from cluster_diagram import plot_all_clusters

    chip = Chip(gds_file, top_cell_name)
    clusters = cluster_chip(chip)
    leaf_modules, cluster_modules, top = build_ast(chip, clusters)
    plot_all_clusters(cluster_modules, leaf_modules, out_dir=".")
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches


def _pin_roles(leaf, inst):
    """{role: net} for one sequential-leaf InstanceCall -- role is
    "D"/"CLK"/"RESET_B"/"SET_B" (from leaf.seq_pinmap, the AST's own
    recovered pin semantics) plus one entry per output pin name (e.g.
    "Q"), each mapped to the ACTUAL net this specific instance connects
    there (InstanceCall.port_map), not just the local pin name."""
    pm = leaf.seq_pinmap
    roles = {"D": inst.port_map.get(pm.data), "CLK": inst.port_map.get(pm.clk)}
    for o in pm.overrides:
        label = "RESET_B" if o.active_value == 0 else "SET_B"
        roles[label] = inst.port_map.get(o.pin)
    for out_pin in pm.outputs:
        roles[out_pin] = inst.port_map.get(out_pin)
    return roles


def _resolve(net, comb_exprs):
    """The minimized boolean expression comb_exprs has for `net`, or
    `net` itself unresolved (already the simplest possible form) if
    Pass 4/5 never needed to flatten it -- e.g. a flip-flop's own D
    pin wired straight from another flip-flop's own Q, no gates
    in between."""
    if net is None:
        return "?"
    if isinstance(net, list):
        return "{" + ", ".join(_resolve(n, comb_exprs) for n in reversed(net)) + "}"
    return comb_exprs.get(net, net)


def collect_cluster_cards(cluster_module, leaf_modules):
    """(flip_flop_cards, comb_output_cards) for one ClusterModule:

    flip_flop_cards: [(inst_name, leaf_name, {role: resolved_expr})]
      -- one per KEPT sequential instance, role in D/CLK/RESET_B/
      SET_B/<output pin name(s)>, D and every output pin already run
      through _resolve() (D's own resolution is the actually
      interesting one: "what does this flip-flop capture next,
      simplified"); CLK/RESET_B/SET_B are left as plain net names
      (control signals, not worth expanding into their own boolean
      expression here).
    comb_output_cards: [(port_name, resolved_expr)] -- this cluster's
      own output ports whose net isn't already a flip-flop's own
      output pin above (a purely combinational output with no
      register of its own).
    """
    comb_exprs = cluster_module.comb_exprs or {}
    flip_flop_cards = []
    covered_nets = set()
    for inst in cluster_module.instances:
        leaf = leaf_modules.get(inst.module_name)
        if leaf is None or leaf.kind != "sequential" or leaf.seq_pinmap is None:
            continue
        roles = _pin_roles(leaf, inst)
        resolved = dict(roles)
        resolved["D"] = _resolve(roles.get("D"), comb_exprs)
        for out_pin in leaf.seq_pinmap.outputs:
            net = roles.get(out_pin)
            if net is not None:
                covered_nets.add(net)
        flip_flop_cards.append((inst.inst_name, inst.module_name, resolved))

    # a covered net's own BASE name too (strip a "[i]" bit-select
    # suffix), so a multi-bit output port whose every individual bit is
    # already some flip-flop's own Q (e.g. "A_10_bus" once "A_10_bus[0]"
    # .. "A_10_bus[7]" are each covered) is correctly recognized as
    # already-covered as a WHOLE, instead of showing up as a trivial,
    # uninformative "A_10_bus = A_10_bus" self-reference.
    covered_bases = {n.rsplit("[", 1)[0] for n in covered_nets if n.endswith("]") and "[" in n}

    # comb_exprs is keyed per BIT ("A_2_bus[0]", ...), never by a whole
    # multi-bit port name at once -- a width>1 PortDecl has to be
    # resolved one bit at a time (each bit gets its own line) rather
    # than looked up as a single (always-missing) key.
    comb_output_cards = []
    for decl in cluster_module.output_ports:
        if decl.name in covered_nets or decl.name in covered_bases:
            continue
        if decl.width == 1:
            comb_output_cards.append((decl.name, _resolve(decl.name, comb_exprs)))
            continue
        for i in range(decl.width):
            bit_name = f"{decl.name}[{i}]"
            if bit_name in covered_nets:
                continue
            comb_output_cards.append((bit_name, _resolve(bit_name, comb_exprs)))

    return flip_flop_cards, comb_output_cards


_ROLE_ORDER = ["Q", "QN", "D", "CLK", "RESET_B", "SET_B"]


def _role_order_key(role):
    return (_ROLE_ORDER.index(role) if role in _ROLE_ORDER else len(_ROLE_ORDER), role)


def plot_cluster(cluster_module, leaf_modules, out_path=None, title=None):
    """Render one ClusterModule's own flip-flop cards (and any leftover
    purely-combinational output cards) as a single PNG -- one rectangle
    per flip-flop, its own role: expression lines inside, stacked
    top to bottom; combinational outputs listed in a smaller block
    below. See collect_cluster_cards() for what actually gets drawn.
    """
    flip_flop_cards, comb_output_cards = collect_cluster_cards(cluster_module, leaf_modules)

    def wrap(text, width=64):
        out, line = [], ""
        for tok in text.replace("&", " & ").replace("|", " | ").split(" "):
            if len(line) + len(tok) + 1 > width:
                out.append(line)
                line = tok
            else:
                line = f"{line} {tok}".strip()
        if line:
            out.append(line)
        return out

    card_lines = []
    for inst_name, leaf_name, resolved in flip_flop_cards:
        lines = [f"{inst_name}  ({leaf_name})"]
        for role in sorted(resolved, key=_role_order_key):
            expr = resolved[role]
            wrapped = wrap(f"{role} = {expr}")
            lines.extend(wrapped)
        card_lines.append(lines)

    if not flip_flop_cards and not comb_output_cards:
        card_lines.append(["(no flip-flops, no combinational output ports -- nothing to draw)"])

    fig_height = max(2.0, sum(0.24 * len(lines) + 0.35 for lines in card_lines) + 0.24 * len(comb_output_cards) + 1.0)
    fig_width = 9.0
    fig, ax = plt.subplots(figsize=(fig_width, fig_height))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, fig_height)
    ax.axis("off")

    y = fig_height - 0.3
    for lines in card_lines:
        card_h = 0.24 * len(lines) + 0.16
        rect = patches.FancyBboxPatch(
            (0.03, y - card_h), 0.94, card_h,
            boxstyle="round,pad=0.02", linewidth=1.2,
            edgecolor="#1f77b4", facecolor="#eaf2fb",
        )
        ax.add_patch(rect)
        ty = y - 0.08
        for i, line in enumerate(lines):
            weight = "bold" if i == 0 else "normal"
            fam = "sans-serif" if i == 0 else "monospace"
            ax.text(0.06, ty, line, fontsize=8.5, family=fam, weight=weight, va="top")
            ty -= 0.22 if i == 0 else 0.20
        y -= card_h + 0.12

    if comb_output_cards:
        y -= 0.15
        ax.text(0.03, y, "combinational output(s) (no flip-flop of their own):",
                 fontsize=8.5, style="italic", va="top")
        y -= 0.26
        for name, expr in comb_output_cards:
            for line in wrap(f"{name} = {expr}"):
                ax.text(0.06, y, line, fontsize=8.5, family="monospace", va="top")
                y -= 0.20

    ax.set_title(title or f"{cluster_module.name}: {len(flip_flop_cards)} flip-flop(s), "
                          f"{len(comb_output_cards)} combinational output(s)", fontsize=10)
    fig.tight_layout()
    out_path = out_path or f"{cluster_module.name}_diagram.png"
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_path}")
    return out_path


def plot_all_clusters(cluster_modules, leaf_modules, out_dir="."):
    """plot_cluster() for every ClusterModule, one PNG each -- printed
    (and returned) as they're written, not batched into one combined
    figure."""
    paths = []
    for cm in cluster_modules:
        out_path = f"{out_dir}/{cm.name}_diagram.png"
        paths.append(plot_cluster(cm, leaf_modules, out_path))
    return paths


if __name__ == "__main__":
    import sys

    from chip import Chip
    from clustering import cluster_chip
    from chip_manipulation import build_ast

    gds_file = sys.argv[1] if len(sys.argv) > 1 else "./warmup/04_final.gds"
    top_cell_name = sys.argv[2] if len(sys.argv) > 2 else "adder_demo"

    chip = Chip(gds_file, top_cell_name)
    clusters = cluster_chip(chip)
    leaf_modules, cluster_modules, top = build_ast(chip, clusters)
    plot_all_clusters(cluster_modules, leaf_modules)
