#!/usr/bin/env python3
"""
gds_graph.py — read a GDSII file and print its cell-reference hierarchy as a graph.

A GDS file stores geometry organized into a hierarchy of *cells*. A cell can
contain references (instances) of other cells. This script treats each cell as a
node and each reference as a directed edge (parent -> child), which is the
"connection graph" that actually lives inside a GDS file.

Usage:
    python3 gds_graph.py layout.gds
    python3 gds_graph.py layout.gds --dot hierarchy.dot   # also export Graphviz
"""

import argparse
import sys
from collections import defaultdict

import gdstk


def build_edges(library):
    """Return (edges, counts): edges is a set of (parent, child) name pairs,
    counts maps (parent, child) -> number of instances of child in parent."""
    edges = set()
    counts = defaultdict(int)
    for cell in library.cells:
        for ref in cell.references:
            child = ref.cell.name if ref.cell is not None else "<missing>"
            # A single Reference can be an array (rows x columns) of instances.
            # repetition.size is the total number of instances (1 if no array).
            n = ref.repetition.size if ref.repetition is not None else 1
            edges.add((cell.name, child))
            counts[(cell.name, child)] += n
    return edges, counts


def top_cells(library):
    """Cells that are never referenced by another cell — the roots of the tree."""
    referenced = {
        ref.cell.name
        for cell in library.cells
        for ref in cell.references
        if ref.cell is not None
    }
    return [c.name for c in library.cells if c.name not in referenced]


def print_tree(name, children_of, counts, prefix="", seen=None, root=True):
    """Print an indented tree starting from `name`. Guards against cycles."""
    seen = seen or set()
    if root:
        print(name)
    kids = sorted(children_of.get(name, []))
    for i, child in enumerate(kids):
        last = i == len(kids) - 1
        branch = "└── " if last else "├── "
        n = counts.get((name, child), 1)
        label = f"{child}" + (f"  (x{n})" if n > 1 else "")
        if child in seen:
            print(prefix + branch + label + "  [cycle]")
            continue
        print(prefix + branch + label)
        print_tree(
            child, children_of, counts,
            prefix + ("    " if last else "│   "),
            seen | {name}, root=False,
        )


def main():
    ap = argparse.ArgumentParser(description="Print a GDS cell hierarchy as a graph.")
    ap.add_argument("gdsfile", help="path to the .gds file")
    ap.add_argument("--dot", metavar="FILE", help="also write a Graphviz .dot file")
    args = ap.parse_args()

    try:
        library = gdstk.read_gds(args.gdsfile)
    except Exception as e:
        sys.exit(f"Could not read '{args.gdsfile}': {e}")

    edges, counts = build_edges(library)

    children_of = defaultdict(set)
    for parent, child in edges:
        children_of[parent].add(child)

    roots = top_cells(library)

    print(f"File: {args.gdsfile}")
    print(f"Cells: {len(library.cells)}   References (edges): {len(edges)}")
    print(f"Top cell(s): {', '.join(sorted(roots)) or '(none)'}\n")

    print("Edge list (parent -> child):")
    for parent, child in sorted(edges):
        n = counts[(parent, child)]
        print(f"  {parent} -> {child}" + (f"  (x{n})" if n > 1 else ""))

    print("\nHierarchy tree:")
    for root in sorted(roots) or sorted(c.name for c in library.cells):
        print_tree(root, children_of, counts)
        print()

    if args.dot:
        with open(args.dot, "w") as f:
            f.write("digraph gds {\n  rankdir=LR;\n  node [shape=box];\n")
            for parent, child in sorted(edges):
                n = counts[(parent, child)]
                lbl = f' [label="x{n}"]' if n > 1 else ""
                f.write(f'  "{parent}" -> "{child}"{lbl};\n')
            f.write("}\n")
        print(f"Wrote Graphviz file: {args.dot}  (render: dot -Tpng {args.dot} -o out.png)")


if __name__ == "__main__":
    main()