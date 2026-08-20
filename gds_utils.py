"""
Small helpers shared across the pipeline: sky130 layer definitions, GDS cell
loading, and source/drain diffusion region labeling.

These used to live inside tracer.py, with pin_direction.py importing them
from there -- but tracer.py now also imports FROM pin_direction.py (to
show "in_A"/"out_X" instead of raw gate/diffusion labels), which made
that a circular import. Splitting the shared bits out here breaks the
cycle: both pipeline.py and pin_direction.py depend on gds_utils.py, and
neither depends on the other's module-level code anymore.
"""

from dataclasses import dataclass

import gdstk

from geometry import _split_pinched


@dataclass(frozen=True)
class Layers:
    poly: tuple = (66, 20)
    diff: tuple = (65, 20)
    nwell: tuple = (64, 20)
    licon: tuple = (66, 44)
    li1: tuple = (67, 20)
    mcon: tuple = (67, 44)
    met1: tuple = (68, 20)


SKY130 = Layers()


def label_diffusion_regions(cell, gates, layers=SKY130, precision=1e-3):
    """Find every source/drain diffusion region (diff minus the transistor
    gates/channels) and label it DN_i or DP_i by nwell containment.

    A single boolean subtraction can come back with what are really
    several disjoint diffusion islands reported as one polygon, joined
    only by a zero-width "pinch" (e.g. two nearby transistors' gate cuts
    into one diffusion strip) -- see geometry._split_pinched for why, and
    transistor.build_transistors()'s module docstring for the downstream
    symptom (a gate ends up with only one matched region instead of two,
    leaving its drain unset). Splitting those apart here, before they
    ever get a single shared label, is what fixes it at the source.
    """
    diff_all = cell.get_polygons(depth=None, layer=layers.diff[0], datatype=layers.diff[1])
    nwell = cell.get_polygons(depth=None, layer=layers.nwell[0], datatype=layers.nwell[1])

    regions = gdstk.boolean(diff_all, gates, "not", precision=precision)
    regions = _split_pinched(regions)

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


def load_cell(gds_file, cell_name=None, cell_index=0):
    """Load one gdstk.Cell definition from `gds_file`.

    Args:
        gds_file: path to the .gds/.gds.gz/.oas file.
        cell_name: name of the cell to load. If omitted, the library's
            single top-level cell is used instead (see below).
        cell_index: which cell to return when more than one cell in the
            library is named `cell_name` -- a GDS library can (unusually,
            but legally as far as gdstk is concerned) define more than one
            Cell under the same name, e.g. after merging libraries from
            different sources. This is unrelated to how many times a cell
            is *instantiated* (Reference'd) elsewhere -- library.cells
            holds one entry per cell *definition*, not per placement; see
            pin.py/plot_utilities.py's own `instance_index` for picking a
            specific placed instance instead.
    """
    library = gdstk.read_gds(gds_file)

    if cell_name is not None:
        matches = [c for c in library.cells if c.name == cell_name]
        if not matches:
            available = ", ".join(sorted(c.name for c in library.cells))
            raise SystemExit(f"cell {cell_name!r} not found in {gds_file!r}. available: {available}")
        if cell_index >= len(matches):
            raise SystemExit(
                f"only {len(matches)} cell(s) named {cell_name!r} in {gds_file!r} "
                f"(asked for index {cell_index})"
            )
        return matches[cell_index]

    top = library.top_level()
    if len(top) != 1:
        names = ", ".join(sorted(c.name for c in top))
        raise SystemExit(
            f"{len(top)} top-level cells found ({names}); pass the one you want as a second argument"
        )
    return top[0]
