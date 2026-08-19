import gdstk

# Load the GDS file
lib = gdstk.read_gds("./warmup/04_final.gds")

layers = set()

# Iterate through every cell and element in the library
for cell in lib.cells:
  for p in cell.polygons:
    layers.add(p.layer)
  for path in cell.paths:
    # Paths can store layers per element segment
    for l in path.layers:
      layers.add(l)
  for label in cell.labels:
    layers.add(label.layer)

print(f"Total number of layers: {len(layers)}")
print(f"Layer numbers present: {sorted(list(layers))}")

# sky130_fd_sc_hd_and3_2 (x1)
# sky130_fd_sc_hd_and4bb_2 (x2)