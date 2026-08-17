"""
End-to-end driver: count transistors, label their source/drain diffusion
regions, build Transistor objects, and trace net equivalence (VDD/VSS/
shared diffusion) for a real GDS cell.

Usage:
    python3 driver.py [gds_file] [cell_name]

If cell_name is omitted, the GDS's single top-level cell is used; if there
is more than one top-level cell, the script lists them and asks you to
pick one explicitly.
"""

import sys

import gdstk

from build_transistor import build_transistors
from count_transistors import count_transistors
from trace_net import NetTracer

DIFF_LAYER = (65, 20)
NWELL_LAYER = (64, 20)
LICON_LAYER = (66, 44)
LI1_LAYER = (67, 20)
MCON_LAYER = (67, 44)
MET1_LAYER = (68, 20)


def label_diffusion_regions(cell, gates, diff_layer=DIFF_LAYER, nwell_layer=NWELL_LAYER, precision=1e-3):
    """Find every source/drain diffusion region (diff minus the transistor
    gates/channels) and label it DN_i or DP_i by nwell containment."""
    diff_all = cell.get_polygons(depth=None, layer=diff_layer[0], datatype=diff_layer[1])
    nwell = cell.get_polygons(depth=None, layer=nwell_layer[0], datatype=nwell_layer[1])

    regions = gdstk.boolean(diff_all, gates, "not", precision=precision)

    def is_pmos(region):
        overlap = gdstk.boolean(region, nwell, "and", precision=precision)
        overlap_area = sum(p.area() for p in overlap)
        return overlap_area > 0.5 * region.area()

    # deterministic left-to-right, bottom-to-top order so indices/labels
    # are stable across runs
    regions.sort(key=lambda r: tuple(r.points.mean(axis=0)))

    labeled = []
    n_idx = p_idx = 0
    for region in regions:
        if is_pmos(region):
            labeled.append((region, f"DP_{p_idx}"))
            p_idx += 1
        else:
            labeled.append((region, f"DN_{n_idx}"))
            n_idx += 1
    return labeled


def load_cell(gds_file, cell_name=None):
    library = gdstk.read_gds(gds_file)

    if cell_name is not None:
        cell = next((c for c in library.cells if c.name == cell_name), None)
        if cell is None:
            available = ", ".join(sorted(c.name for c in library.cells))
            raise SystemExit(f"cell {cell_name!r} not found in {gds_file!r}. available: {available}")
        return cell

    top = library.top_level()
    if len(top) != 1:
        names = ", ".join(sorted(c.name for c in top))
        raise SystemExit(
            f"{len(top)} top-level cells found ({names}); pass the one you want as a second argument"
        )
    return top[0]


def main():
    gds_file = sys.argv[1] if len(sys.argv) > 1 else "./warmup/04_final.gds"
    cell_name = sys.argv[2] if len(sys.argv) > 2 else "sky130_fd_sc_hd__and3_2"

    cell = load_cell(gds_file, cell_name)
    print(f"=== {gds_file} :: {cell.name} ===\n")

    # 1. count + locate transistor gates
    result = count_transistors(gds_file, cell.name)
    gates = result["gates"]
    print(f"transistors found: {result['total']}  (nmos={result['nmos']}, pmos={result['pmos']})\n")

    # 2. label every source/drain diffusion region
    labeled_regions = label_diffusion_regions(cell, gates)
    print(f"labeled diffusion regions: {len(labeled_regions)}")
    for _, label in labeled_regions:
        print(f"  {label}")
    print()

    # 3. build Transistor objects (type + gate/source/drain labels)
    transistors = build_transistors(gates, labeled_regions)
    print("transistors:")
    for t in transistors:
        print(f"  {t.gate_label:8s} kind={t.kind or '?':5s} source={t.source_label} drain={t.drain_label}")
    print()

    # 4. trace net equivalence (VDD/VSS/shared diffusion) for every labeled region
    licon = cell.get_polygons(depth=None, layer=LICON_LAYER[0], datatype=LICON_LAYER[1])
    li1 = cell.get_polygons(depth=None, layer=LI1_LAYER[0], datatype=LI1_LAYER[1])
    mcon = cell.get_polygons(depth=None, layer=MCON_LAYER[0], datatype=MCON_LAYER[1])
    met1 = cell.get_polygons(depth=None, layer=MET1_LAYER[0], datatype=MET1_LAYER[1])
    nwell = cell.get_polygons(depth=None, layer=NWELL_LAYER[0], datatype=NWELL_LAYER[1])

    labels = cell.get_labels(depth=None)
    tracer = NetTracer(labeled_regions, licon, li1, mcon, met1, nwell, labels=labels)
    print("net traces:")
    for _, label in labeled_regions:
        print(f"  trace({label}) -> {tracer.trace(label)}")


if __name__ == "__main__":
    main()