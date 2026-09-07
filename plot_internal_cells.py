"""plot_internal_cells.py: plot the placement (x,y) coordinates of every
INTERNAL_3 and INTERNAL_7 cell instance across the whole puzzle.gds
layout -- WHERE these cells sit on the die, not their own internal
layout (that's what INTERNAL_3.png/INTERNAL_7.png already show).

Reuses scratch.py's own (now-fixed) recursive placement-finder rather
than importing it, since scratch.py is gitignored/personal and not
meant to be a stable import target.
"""

import gdstk
import matplotlib.pyplot as plt
import numpy as np


def get_global_positions(cell, target_cell_name, current_origin=(0.0, 0.0), current_rotation=0.0):
    """Recursively finds every global (x,y) placement of `target_cell_name`
    within `cell`'s own reference hierarchy, handling nested translation,
    rotation, and array references (gdstk 1.0.1 exposes array info via
    ref.repetition, not the old ref.columns/ref.rows/ref.spacing --
    apply_repetition() handles all 5 repetition kinds uniformly)."""
    positions = []
    for ref in cell.references:
        angle = current_rotation
        cos_a, sin_a = np.cos(angle), np.sin(angle)

        local_origins = [ref.origin]
        if ref.repetition is not None:
            local_origins += [r.origin for r in ref.apply_repetition()]

        if ref.cell.name == target_cell_name:
            for local_x, local_y in local_origins:
                global_x = current_origin[0] + (local_x * cos_a - local_y * sin_a)
                global_y = current_origin[1] + (local_x * sin_a + local_y * cos_a)
                positions.append((global_x, global_y))
        else:
            next_rotation = current_rotation + (ref.rotation or 0.0)
            for local_x, local_y in local_origins:
                next_origin = (
                    current_origin[0] + (local_x * cos_a - local_y * sin_a),
                    current_origin[1] + (local_x * sin_a + local_y * cos_a),
                )
                positions.extend(get_global_positions(ref.cell, target_cell_name, next_origin, next_rotation))
    return positions


# INTERNAL_7 is exactly 3x INTERNAL_3's own width (4.14 vs 1.38 um,
# both bounding-box widths read straight off the GDS) -- the classic
# Morse dash:dot ratio. Measuring EDGE-TO-EDGE gaps (not center-to-
# center, which conflates mark width with spacing) in units of the dot
# width clusters into exactly the 3 standard telegraph timing values:
# 1 unit = same letter, 3 = new letter, 7 = new word.
WIDTH = {"INTERNAL_3": 1.38, "INTERNAL_7": 4.14}
SYMBOL = {"INTERNAL_3": ".", "INTERNAL_7": "-"}
UNIT = 1.38

MORSE = {
    '.-': 'A', '-...': 'B', '-.-.': 'C', '-..': 'D', '.': 'E', '..-.': 'F', '--.': 'G', '....': 'H',
    '..': 'I', '.---': 'J', '-.-': 'K', '.-..': 'L', '--': 'M', '-.': 'N', '---': 'O', '.--.': 'P',
    '--.-': 'Q', '.-.': 'R', '...': 'S', '-': 'T', '..-': 'U', '...-': 'V', '.--': 'W', '-..-': 'X',
    '-.--': 'Y', '--..': 'Z', '-----': '0', '.----': '1', '..---': '2', '...--': '3', '....-': '4',
    '.....': '5', '-....': '6', '--...': '7', '---..': '8', '----.': '9',
}


def get_marks(gds_path="puzzle.gds"):
    """Every INTERNAL_3/INTERNAL_7 instance, sorted left to right, as
    (edge_start, edge_end, cell_name, symbol) tuples -- edge_start is
    the reference's own origin (the cell's local (0,0) corner, since
    these are plain unrotated/unreflected/unmagnified placements), and
    edge_end = edge_start + WIDTH[cell_name]."""
    library = gdstk.read_gds(gds_path)
    top_cell = library.top_level()[0]

    points = []
    for name in ("INTERNAL_3", "INTERNAL_7"):
        for x, y in get_global_positions(top_cell, name):
            points.append((x, name))
    points.sort()

    marks = []
    for x, name in points:
        marks.append((x, x + WIDTH[name], name, SYMBOL[name]))
    return marks


def decode_morse(marks, unit=UNIT):
    """Groups `marks` (from get_marks()) into letters/words by their
    OWN edge-to-edge gaps (see module docstring for the 1/3/7-unit
    convention) and decodes each letter via the standard Morse table.

    Returns (decoded_text, letters, gap_units) where `letters` is a
    list of (symbol_string, decoded_char, is_word_break_before) and
    `gap_units` is the raw list of gap sizes (in units) between
    consecutive marks, for anything that wants to double check the
    grouping itself rather than trust it blindly."""
    gap_units = [round((marks[i + 1][0] - marks[i][1]) / unit) for i in range(len(marks) - 1)]

    letters = [[marks[0][3]]]
    word_break_before = [False]
    for i, g in enumerate(gap_units):
        if g <= 1:
            letters[-1].append(marks[i + 1][3])
        else:
            letters.append([marks[i + 1][3]])
            word_break_before.append(g >= 7)

    decoded_letters = []
    for symbols, is_word_break in zip(letters, word_break_before):
        code = "".join(symbols)
        char = MORSE.get(code, f"[{code}]")
        decoded_letters.append((code, char, is_word_break))

    text = "".join((" " if wb else "") + ch for code, ch, wb in decoded_letters)
    return text.strip(), decoded_letters, gap_units


def plot_morse_signal(gds_path="puzzle.gds", out_path="internal_cells_morse.png"):
    """Plots the INTERNAL_3/INTERNAL_7 row as an actual telegraph-style
    pulse trace (on = mark present, off = gap) instead of a scatter of
    placement points, with the decoded letters labeled above each
    group and word boundaries marked -- the same physical row plotted
    in plot(), just read as a signal instead of a set of coordinates."""
    marks = get_marks(gds_path)
    text, letters, gap_units = decode_morse(marks)
    print(f"decoded: {text!r}")

    fig, ax = plt.subplots(figsize=(16, 3))

    # same round-dot look as plot()'s own scatter markers -- INTERNAL_3
    # is just that dot; INTERNAL_7 is a "lengthy dot" (a thick line with
    # round caps spanning its own edge_start..edge_end, so it reads as
    # an elongated dot/capsule rather than a flat rectangular bar)
    y_mid = 0.5
    dot_xs = [(s + e) / 2 for (s, e, name, sym) in marks if sym == "."]
    ax.scatter(dot_xs, [y_mid] * len(dot_xs), s=90, color="tab:blue", zorder=3, label="INTERNAL_3 (dot)")
    dash_label = "INTERNAL_7 (dash)"
    for s, e, name, sym in marks:
        if sym == "-":
            ax.plot([s, e], [y_mid, y_mid], color="tab:orange", linewidth=6,
                     solid_capstyle="round", zorder=3, label=dash_label)
            dash_label = None  # only label the first one, avoid a duplicate legend entry

    # label each decoded letter centered over its own group of marks,
    # and mark word breaks with a vertical dashed line
    idx = 0
    x_min = marks[0][0]
    for (code, char, is_word_break), n_symbols in zip(letters, (len(c) for c, ch, wb in letters)):
        group_marks = marks[idx: idx + n_symbols]
        idx += n_symbols
        left = group_marks[0][0]
        right = group_marks[-1][1]
        mid = (left + right) / 2
        if is_word_break:
            ax.axvline(left - (right - left) * 0.15, color="gray", linestyle="--", linewidth=0.8)
        ax.text(mid, 1.35, char, ha="center", va="bottom", fontsize=11, fontweight="bold")
        ax.text(mid, -0.55, code, ha="center", va="top", fontsize=7, color="gray", family="monospace")

    ax.set_ylim(-0.9, 1.9)
    ax.set_xlim(x_min - 2, marks[-1][1] + 2)
    ax.set_yticks([])
    ax.set_xlabel("x (µm)")
    ax.set_title(f"INTERNAL_3/INTERNAL_7 read as a Morse signal: {text!r}")
    ax.legend(loc="upper left", bbox_to_anchor=(1.0, 1), borderaxespad=0)
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    print(f"wrote {out_path}")
    return text


def plot(gds_path="puzzle.gds", out_path="internal_cells_placement.png"):
    library = gdstk.read_gds(gds_path)
    top_cell = library.top_level()[0]

    targets = {"INTERNAL_3": "tab:blue", "INTERNAL_7": "tab:red"}
    fig, ax = plt.subplots(figsize=(10, 8))

    for name, color in targets.items():
        positions = get_global_positions(top_cell, name)
        print(f"{name}: {len(positions)} instances")
        if not positions:
            continue
        xs, ys = zip(*positions)
        ax.scatter(xs, ys, s=14, color=color, label=f"{name} ({len(positions)})", alpha=0.85)

    ax.set_xlabel("x (µm)")
    ax.set_ylabel("y (µm)")
    ax.set_title(f"INTERNAL_3 / INTERNAL_7 placement in {gds_path}")
    ax.set_aspect("equal")
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1), borderaxespad=0)
    ax.grid(True, linewidth=0.3, alpha=0.5)
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    plot()
    plot_morse_signal()
