"""
End-to-end driver: count transistors, label their source/drain diffusion
regions, build Transistor objects, trace net equivalence (VDD/VSS/shared
diffusion), and draw a transistor-level schematic for a real GDS cell.

Usage:
    python3 pipeline.py [gds_file] [cell_name]

If cell_name is omitted, the GDS's single top-level cell is used; if there
is more than one top-level cell, the script lists them and asks you to
pick one explicitly.
"""

import sys

from gds_utils import SKY130, label_diffusion_regions, load_cell
from net_trace import NetTracer
from pin import LeafCellAnalyzer
from plot_utilities import draw_schematic
from transistor import build_transistors, count_transistors


def _pin_display(gds_file, cell_name, label):
    """"in_A" / "out_X" if `label`'s net is one of the cell's own named
    pins (per LeafCellAnalyzer.classify_pin_direction), else `label` itself
    unchanged -- most gate/source/drain labels are internal nodes with no
    external pin of their own, and should keep showing as "GP_3"/"DN_2"."""
    analyzer = LeafCellAnalyzer.for_cell(gds_file, cell_name)
    pin_name, direction = analyzer.pin_name_for_net(label)
    if pin_name is None:
        return label
    prefix = {"input": "in", "output": "out"}.get(direction, direction)
    return f"{prefix}_{pin_name}"


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
        print(f"  gate={t.gate_label:8s} kind={t.kind or '?':5s} source={t.source_label} drain={t.drain_label}")
    print()

    # 4. trace net equivalence (VDD/VSS/shared diffusion) for every labeled region
    poly = cell.get_polygons(depth=None, layer=SKY130.poly[0], datatype=SKY130.poly[1])
    licon = cell.get_polygons(depth=None, layer=SKY130.licon[0], datatype=SKY130.licon[1])
    li1 = cell.get_polygons(depth=None, layer=SKY130.li1[0], datatype=SKY130.li1[1])
    mcon = cell.get_polygons(depth=None, layer=SKY130.mcon[0], datatype=SKY130.mcon[1])
    met1 = cell.get_polygons(depth=None, layer=SKY130.met1[0], datatype=SKY130.met1[1])
    nwell = cell.get_polygons(depth=None, layer=SKY130.nwell[0], datatype=SKY130.nwell[1])

    labels = cell.get_labels(depth=None)
    tracer = NetTracer(labeled_regions, transistors, poly, licon, li1, mcon, met1, nwell, labels=labels)
    print("net traces:")
    for _, label in labeled_regions:
        print(f"  trace({label}) -> {tracer.trace(label)}")

    # 5. Update transistor with cell_global labels -- and, where a net
    # turns out to be one of the cell's own named pins, show "in_A" /
    # "out_X" instead of the raw gate/diffusion label (see pin.py).
    print("transistors renamed:")
    renamed = []
    for t in transistors:
        gate = _pin_display(gds_file, cell.name, t.gate_label)
        source = _pin_display(gds_file, cell.name, tracer.find_root(t.source_label))
        drain = _pin_display(gds_file, cell.name, tracer.find_root(t.drain_label))
        print(f"  gate={gate:8s} kind={t.kind or '?':5s} source={source:8s} drain={drain:8s}")
        renamed.append({"gate": gate, "kind": t.kind, "source": source, "drain": drain})
    print()

    # 6. draw a transistor-level schematic PNG from that same renamed
    # data (see plot_utilities.py) -- one row of PMOS under VDD, one row of
    # NMOS above VSS, the usual way a static CMOS gate is drawn.
    schematic_path = f"{cell.name}_schematic.png"
    draw_schematic(renamed, out_path=schematic_path, title=f"{cell.name} ({gds_file})")


if __name__ == "__main__":
    main()
