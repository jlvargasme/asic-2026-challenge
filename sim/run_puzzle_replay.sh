#!/usr/bin/env bash
# Verilate + build + run sim/tb_puzzle_replay.cpp against puzzle.v,
# replaying a reference VCD's own input stimulus (see
# sim/vcd_stimulus.py) and dumping our own O/success trace for
# sim/compare_vcd.py to diff against that same reference.
#
# Run from the repo root, inside an environment with verilator/make/g++
# on PATH (this project runs it via WSL -- see sim/vcd_stimulus.py's own
# usage and the repo's yosys/verilator local-install convention).
set -euo pipefail
cd "$(dirname "$0")/.."
repo_root="$(pwd)"

stimulus="${1:-sim/example_inputs_stimulus.txt}"
out_vcd="${2:-sim/puzzle_replay.vcd}"

outdir="sim/obj_dir_puzzle_replay"
exe="sim_puzzle_replay"

rm -rf "${outdir}"
verilator --cc --exe --trace --build \
    --top-module puzzle \
    -Mdir "${outdir}" \
    -o "${exe}" \
    "${repo_root}/puzzle.v" \
    "${repo_root}/sim/tb_puzzle_replay.cpp"

"./${outdir}/${exe}" "${stimulus}" "${out_vcd}"
