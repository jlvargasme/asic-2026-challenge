"""
Small helpers shared between tracer.py (single-cell driver) and
pin_direction.py (per-instance pin direction classification).

These used to live inside tracer.py, with pin_direction.py importing them
from there -- but tracer.py now also imports FROM pin_direction.py (to
show "in_A"/"out_X" instead of raw gate/diffusion labels), which made
that a circular import. Splitting the shared bits out here breaks the
cycle: both tracer.py and pin_direction.py depend on cell_utils.py, and
neither depends on the other's module-level code anymore.
"""

import gdstk

POLY_LAYER = (66, 20)
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