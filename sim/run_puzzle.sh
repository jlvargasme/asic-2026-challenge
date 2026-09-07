#!/usr/bin/env bash
# Verilate + build + run sim/tb_puzzle.cpp against the decompiled
# puzzle.v -- no known-correct reference exists for this design (unlike
# adder_demo/warmup), so this is a compile+elaborate sanity check plus a
# generic drive-and-observe run, not a pass/fail comparison. See
# sim/tb_puzzle.cpp's own docstring for exactly what it drives/reads.
#
# Run from the repo root, inside an environment with verilator/make/g++
# on PATH (this project runs it via WSL -- see HANDOFF.md).
set -euo pipefail
cd "$(dirname "$0")/.."
repo_root="$(pwd)"

outdir="sim/obj_dir_puzzle"
exe="sim_puzzle"
vcd="puzzle.vcd"

rm -rf "${outdir}"
verilator --cc --exe --trace --build \
    --top-module puzzle \
    -Mdir "${outdir}" \
    -o "${exe}" \
    "${repo_root}/puzzle.v" \
    "${repo_root}/sim/tb_puzzle.cpp"

"./${outdir}/${exe}" "${vcd}"
echo "wrote ${vcd}"
