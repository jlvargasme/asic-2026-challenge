#!/usr/bin/env bash
# Verilate + build + run sim/tb_adder_demo.cpp against BOTH the
# known-correct warmup/00_source.v and the reverse-engineered
# adder_demo.v (same top-level port list -- A, B, clk, en, rst_n, S --
# so the identical testbench drives either one), producing one VCD per
# variant for waveform inspection.
#
# Run from the repo root, inside an environment with verilator/make/g++
# on PATH (this project runs it via WSL -- see HANDOFF.md).
set -euo pipefail
cd "$(dirname "$0")/.."
repo_root="$(pwd)"

build_and_run () {
    local label="$1"
    local verilog_file="$2"
    local outdir="sim/obj_dir_${label}"
    local exe="sim_adder_demo_${label}"
    local vcd="adder_demo_${label}.vcd"
    echo "=== ${label}: ${verilog_file} ==="
    rm -rf "${outdir}"
    # absolute paths for both sources: verilator's generated Makefile
    # invokes `make -C ${outdir}`, so a path given relative to the repo
    # root (like the ones a caller would naturally write) no longer
    # resolves once make has already changed into that directory.
    verilator --cc --exe --trace --build \
        --top-module adder_demo \
        -Mdir "${outdir}" \
        -o "${exe}" \
        "${repo_root}/${verilog_file}" \
        "${repo_root}/sim/tb_adder_demo.cpp"
    "./${outdir}/${exe}" "${vcd}"
    echo "wrote ${vcd}"
    echo
}

build_and_run source warmup/00_source.v
build_and_run decompiled adder_demo.v
