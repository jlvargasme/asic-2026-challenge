"""
Draw a transistor-level electrical schematic (PNG) from the same
gate/kind/source/drain data tracer.py's "transistors renamed" step
prints -- one row of PMOS symbols under a VDD rail, one row of NMOS
symbols above a VSS rail, the usual way a static CMOS gate is drawn.

Two different connection styles are used, deliberately:

  - Any terminal literally named one of `vdd_names`/`vss_names` gets a
    real wire drawn straight to the rail -- that's cheap to route
    (there's only one rail per supply) and it's what makes the picture
    immediately readable as "this leg pulls up to VDD" / "this leg pulls
    down to VSS".
  - Every other terminal (every gate, and any source/drain tied to an
    internal net like "DN_0" or a pin like "in_A") is drawn as a short
    stub with its net name written at the end, instead of routing a wire
    across the page to every other terminal that shares the same name.
    This is the standard schematic-capture shorthand of a net label /
    off-page connector: two stubs with the same text are understood to
    be the same electrical node without an actual wire connecting them.
    A full auto-router for arbitrary net topology is a much bigger (and
    much more fragile) problem than this tool needs to solve -- the goal
    here is "can I see at a glance what each transistor's terminals are
    wired to", not a fabrication-ready schematic.
"""

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle

RAIL_LINEWIDTH = 1.8
LEAD_LINEWIDTH = 1.2
PLATE_LINEWIDTH = 1.6


def _field(t, *names):
    """Read the first present attribute/key out of `names` from `t`,
    whether `t` is a dict (e.g. {"kind": ..., "gate": ...}) or an object
    with attributes (e.g. build_transistor.py's Transistor dataclass,
    where the fields are named gate_label/source_label/drain_label)."""
    if isinstance(t, dict):
        for name in names:
            if name in t:
                return t[name]
        return None
    for name in names:
        if hasattr(t, name):
            return getattr(t, name)
    return None


def _transistor_fields(t):
    return (
        _field(t, "kind"),
        _field(t, "gate", "gate_label"),
        _field(t, "source", "source_label"),
        _field(t, "drain", "drain_label"),
    )


def _draw_mosfet(ax, cx, cy, kind, gate_label, top_label, bot_label,
                  height=0.8, width=0.5, gate_len=0.55, gate_gap=0.12,
                  bubble_r=0.05, fontsize=8):
    """Draw one simplified MOSFET symbol centered at (cx, cy): a vertical
    channel line (split by a small gap -- the two "plates" -- to mark
    where the gate couples in), a gate lead coming in from the left with
    a bubble on it for PMOS (active-low gate, the usual convention) and
    no bubble for NMOS, and text labels for whichever of top_label/
    bot_label/gate_label are given (pass None to omit a label -- used
    when that terminal is about to get a real rail wire drawn instead).

    Returns (top_lead_y, bottom_lead_y) so the caller can extend a rail
    wire from the exact point the symbol's lead ends.
    """
    half_h, half_w, half_gap = height / 2, width / 2, gate_gap / 2
    top_lead_y, bot_lead_y = cy + half_h, cy - half_h

    ax.add_line(Line2D([cx, cx], [top_lead_y, cy + half_gap], color="black", linewidth=LEAD_LINEWIDTH))
    ax.add_line(Line2D([cx, cx], [cy - half_gap, bot_lead_y], color="black", linewidth=LEAD_LINEWIDTH))
    ax.add_line(Line2D([cx - half_w, cx + half_w], [cy + half_gap, cy + half_gap], color="black", linewidth=PLATE_LINEWIDTH))
    ax.add_line(Line2D([cx - half_w, cx + half_w], [cy - half_gap, cy - half_gap], color="black", linewidth=PLATE_LINEWIDTH))

    gate_x0 = cx - half_w - gate_len
    if kind == "PMOS":
        bubble_x = cx - half_w - bubble_r
        ax.add_line(Line2D([gate_x0, bubble_x - bubble_r], [cy, cy], color="black", linewidth=LEAD_LINEWIDTH))
        ax.add_patch(Circle((bubble_x, cy), bubble_r, fill=False, linewidth=LEAD_LINEWIDTH))
    else:
        ax.add_line(Line2D([gate_x0, cx - half_w], [cy, cy], color="black", linewidth=LEAD_LINEWIDTH))

    if top_label is not None:
        ax.text(cx, top_lead_y + 0.08, top_label, ha="center", va="bottom", fontsize=fontsize)
    if bot_label is not None:
        ax.text(cx, bot_lead_y - 0.08, bot_label, ha="center", va="top", fontsize=fontsize)
    ax.text(gate_x0 - 0.05, cy, gate_label or "?", ha="right", va="center", fontsize=fontsize)

    return top_lead_y, bot_lead_y


def _draw_row(ax, records, kind, cy, rail_y, rail_names, n_cols, dx, x0):
    """Draw one row (all PMOS, or all NMOS) of symbols, centered within
    n_cols so a shorter row (e.g. 3 PMOS vs. 5 NMOS) lines up visually."""
    offset = (n_cols - len(records)) * dx / 2
    for i, r in enumerate(records):
        cx = x0 + offset + i * dx

        # orient the symbol so the rail-connected terminal (if any) is
        # drawn on the side nearer that rail -- purely cosmetic.
        drain_is_rail = r["drain"] in rail_names
        source_is_rail = r["source"] in rail_names
        near_rail_first = (kind == "PMOS")  # PMOS row sits above the VDD rail -> rail terminal on top
        if drain_is_rail or source_is_rail:
            rail_net = r["drain"] if drain_is_rail else r["source"]
            other_net = r["source"] if drain_is_rail else r["drain"]
            top_net, bot_net = (rail_net, other_net) if near_rail_first else (other_net, rail_net)
        else:
            top_net, bot_net = r["source"], r["drain"]

        top_is_rail = top_net in rail_names
        bot_is_rail = bot_net in rail_names
        top_y, bot_y = _draw_mosfet(
            ax, cx, cy, kind, r["gate"],
            None if top_is_rail else top_net,
            None if bot_is_rail else bot_net,
        )
        if top_is_rail:
            ax.add_line(Line2D([cx, cx], [top_y, rail_y], color="black", linewidth=1.0))
        if bot_is_rail:
            ax.add_line(Line2D([cx, cx], [bot_y, rail_y], color="black", linewidth=1.0))


def draw_schematic(transistors, out_path="schematic.png", title=None,
                    vdd_names=("VDD",), vss_names=("VSS",)):
    """Render a transistor-level schematic and save it as a PNG.

    Args:
        transistors: list of Transistor objects (build_transistor.py) or
            plain dicts with keys "kind"/"gate"/"source"/"drain" -- the
            same shape tracer.py's "transistors renamed" step prints,
            where source/drain/gate are the *display* labels (e.g.
            "in_A", "out_X", "VDD", or a raw internal label like "DN_0"
            when a net couldn't be resolved to a named pin).
        out_path: PNG file to write.
        title: optional title text drawn above the diagram.
        vdd_names, vss_names: which terminal values count as "this is the
            supply rail, draw a real wire" rather than a net-label stub.
    """
    records = []
    for t in transistors:
        kind, gate, source, drain = _transistor_fields(t)
        records.append({"kind": kind, "gate": gate, "source": source, "drain": drain})

    pmos = [r for r in records if r["kind"] == "PMOS"]
    nmos = [r for r in records if r["kind"] == "NMOS"]
    other = [r for r in records if r["kind"] not in ("PMOS", "NMOS")]

    n_cols = max(len(pmos), len(nmos), 1)
    dx = 1.6
    x0 = 1.0
    width = x0 + n_cols * dx + 1.0

    y_vdd, y_pmos = 3.8, 2.7
    y_nmos, y_vss = 1.0, -0.1
    y_other = y_vss - 1.6 if other else None

    fig_h = 5.4 + (1.8 if other else 0)
    fig, ax = plt.subplots(figsize=(max(width, 4.0), fig_h))

    rail_x1 = x0 + n_cols * dx
    ax.add_line(Line2D([0.2, rail_x1], [y_vdd, y_vdd], color="black", linewidth=RAIL_LINEWIDTH))
    ax.text(0.1, y_vdd, "VDD", ha="right", va="center", fontsize=10, fontweight="bold")
    ax.add_line(Line2D([0.2, rail_x1], [y_vss, y_vss], color="black", linewidth=RAIL_LINEWIDTH))
    ax.text(0.1, y_vss, "VSS", ha="right", va="center", fontsize=10, fontweight="bold")

    _draw_row(ax, pmos, "PMOS", y_pmos, y_vdd, vdd_names, n_cols, dx, x0)
    _draw_row(ax, nmos, "NMOS", y_nmos, y_vss, vss_names, n_cols, dx, x0)

    if other:
        offset = (n_cols - len(other)) * dx / 2
        for i, r in enumerate(other):
            cx = x0 + offset + i * dx
            _draw_mosfet(ax, cx, y_other, "NMOS", r["gate"], r["source"], r["drain"], fontsize=8)
            ax.text(cx, y_other - 0.55, "kind unresolved", ha="center", va="top", fontsize=6.5, color="crimson")

    ax.set_xlim(-0.3, rail_x1 + 0.4)
    y_bottom = (y_other - 1.0) if other else (y_vss - 0.6)
    ax.set_ylim(y_bottom, y_vdd + 0.6)
    ax.set_aspect("equal")
    ax.axis("off")
    if title:
        ax.set_title(title, fontsize=11)

    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)
    print(f"wrote {out_path}")
    return out_path


if __name__ == "__main__":
    # standalone smoke test using the and3_2 "transistors renamed" table
    # from tracer.py's own output, hardcoded here so this file can be
    # sanity-checked without re-running the whole extraction pipeline.
    demo = [
        {"gate": "GP_0", "kind": "PMOS", "source": "VDD", "drain": "out_X"},
        {"gate": "GP_1", "kind": "PMOS", "source": "out_X", "drain": "VDD"},
        {"gate": "in_A", "kind": "PMOS", "source": "DN_0", "drain": "VDD"},
        {"gate": "in_B", "kind": "PMOS", "source": "VDD", "drain": "DN_0"},
        {"gate": "in_C", "kind": "PMOS", "source": "DN_0", "drain": "VDD"},
        {"gate": "GN_5", "kind": "NMOS", "source": "VSS", "drain": "out_X"},
        {"gate": "GN_6", "kind": "NMOS", "source": "out_X", "drain": "VSS"},
        {"gate": "in_C", "kind": "NMOS", "source": "DN_2", "drain": "VSS"},
        {"gate": "in_B", "kind": "NMOS", "source": "DN_1", "drain": "DN_2"},
        {"gate": "in_A", "kind": "NMOS", "source": "DP_0", "drain": "DN_1"},
    ]
    draw_schematic(demo, out_path="and3_2_schematic.png", title="sky130_fd_sc_hd__and3_2")