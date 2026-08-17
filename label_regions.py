import gdstk
from plot_layout import plot_polygon, plot_labels

def label_regions(
    gds_path,
    cell_name,
    poly_layer=(66, 20),
    diff_layer=(65, 20),
    nwell_layer=(64, 20),
    precision=1e-3
):
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

    unlabeled_regions = gdstk.boolean(diff, poly, "not", precision=precision)
    unlabeled_regions.sort(key=lambda r: tuple(r.points.mean(axis=0)))
    labeled_regions = []
    # plot_polygon(unlabeled_regions, filename="unlabeled_regions.png")

    nmos_index = pmos_index = 0
    for region in unlabeled_regions:
        overlap = gdstk.boolean(region, nwell, "and", precision=precision)
        overlap_area = sum(p.area() for p in overlap)
        label = ""
        if overlap_area == region.area():
            label = "DP_" + str(pmos_index)
            pmos_index += 1
        else:
            label = "DN_" + str(nmos_index)
            nmos_index += 1

        labeled_regions.append((region, label))

    # plot_labels(labeled_regions, filename="labeled_regions.png")
    return labeled_regions


# python ./count_transistors.py ./warmup/04_final.gds sky130_fd_sc_hd__and3_2
if __name__ == "__main__":
    import sys

    # if len(sys.argv) != 3:
    #     print(f"usage: {sys.argv[0]} <gds_path> <cell_name>")
    #     sys.exit(1)

    # result = count_transistors(sys.argv[1], sys.argv[2])
    # result = count_transistors("./warmup/04_final.gds", "sky130_fd_sc_hd__and3_2")
    # print(f"total transistors: {result['total']}")
    # print(f"  nmos: {result['nmos']}")
    # print(f"  pmos: {result['pmos']}")

    labeled_regions = label_regions("./warmup/04_final.gds", "sky130_fd_sc_hd__and3_2")
