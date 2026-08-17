"""
Count transistors in a gdstk cell by geometric layer overlap.

GDSII has no native "transistor" object -- a MOSFET is inferred from where
the gate (poly) layer crosses the active/diffusion layer. This follows the
SkyWater sky130 GDS layer map:

    nwell : 64/20
    diff  : 65/20   (the *same* layer's 65/44 datatype is "tap" -- a
                     well/substrate strap, not a transistor channel, and
                     is deliberately excluded)
    poly  : 66/20

Each disjoint polygon in (poly AND diff) is counted as one transistor gate.
For a folded multi-finger device this means each finger is counted
separately (N fingers -> N transistors), which matches how most
extraction/LVS tools report individual channel regions.
"""

import gdstk

def count_transistors(
    gds_path,
    cell_name,
    poly_layer=(66, 20),
    diff_layer=(65, 20),
    nwell_layer=(64, 20),
    precision=1e-3,
    min_area=0.0,
):
    """Count transistors in `cell_name` inside `gds_path`.

    Args:
        gds_path: path to the .gds/.gds.gz/.oas file.
        cell_name: name of the cell to inspect. The cell's full reference
            hierarchy (all sub-cells it instantiates, at any depth) is
            included -- you do not need to flatten the GDS first.
        poly_layer, diff_layer, nwell_layer: (layer, datatype) tuples.
            Defaults are the sky130 GDS layer numbers; override these if
            your PDK differs.
        precision: precision (in the library's user units) passed to
            gdstk.boolean for the poly/diff intersection.
        min_area: discard intersection regions smaller than this area
            (same units as the GDS, usually um^2) -- useful for dropping
            slivers produced by rounding at abutting-shape edges.

    Returns:
        dict with keys:
            "total": total transistor (gate/finger) count
            "nmos":  gates whose area is NOT substantially inside an nwell
            "pmos":  gates whose area IS substantially inside an nwell
            "gates": list of gdstk.Polygon for the raw poly/diff overlaps
                     (useful for debugging / visualizing what was counted)
    """
    library = gdstk.read_gds(gds_path)
    cell = next((c for c in library.cells if c.name == cell_name), None)
    if cell is None:
        available = ", ".join(sorted(c.name for c in library.cells))
        raise ValueError(
            f"Cell {cell_name!r} not found in {gds_path!r}. "
            f"Available cells: {available}"
        )

    # depth=None recurses through every reference in the cell's hierarchy
    # and applies each reference's transform, so this also catches
    # transistors defined inside instantiated sub-cells.
    poly = cell.get_polygons(depth=None, layer=poly_layer[0], datatype=poly_layer[1])
    diff = cell.get_polygons(depth=None, layer=diff_layer[0], datatype=diff_layer[1])
    nwell = cell.get_polygons(depth=None, layer=nwell_layer[0], datatype=nwell_layer[1])

    gates = gdstk.boolean(poly, diff, "and", precision=precision)
    if min_area > 0:
        gates = [g for g in gates if g.area() >= min_area]

    nmos = pmos = 0
    for gate in gates:
        overlap = gdstk.boolean(gate, nwell, "and", precision=precision)
        overlap_area = sum(p.area() for p in overlap)
        if overlap_area > 0.5 * gate.area():
            pmos += 1
        else:
            nmos += 1

    return {
        "total": len(gates),
        "nmos": nmos,
        "pmos": pmos,
        "gates": gates,
    }

# python ./count_transistors.py ./warmup/04_final.gds sky130_fd_sc_hd__and3_2
if __name__ == "__main__":
    import sys

    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} <gds_path> <cell_name>")
        sys.exit(1)

    # result = count_transistors(sys.argv[1], sys.argv[2])
    result = count_transistors("./warmup/04_final.gds", "sky130_fd_sc_hd__and3_2")
    print(f"total transistors: {result['total']}")
    print(f"  nmos: {result['nmos']}")
    print(f"  pmos: {result['pmos']}")