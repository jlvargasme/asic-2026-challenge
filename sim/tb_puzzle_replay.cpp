// Verilator replay testbench for puzzle.v -- drives the chip's own TRUE
// top-level input ports (clk, rst_n, enable, I) with EXACTLY the value
// changes recorded in a reference VCD (see sim/vcd_stimulus.py, which
// turns e.g. example_inputs.vcd into the simple "#<time>\n<sig>=<val>"
// stimulus file this reads), at the SAME simulated timestamps the
// reference file used -- so the resulting O/success trace can be
// diffed directly, timestamp for timestamp, against that same
// reference VCD's own O/success columns via sim/compare_vcd.py. This is
// the correctness check this project didn't have before: an actual
// known-good example trace to validate the decompiled RTL against,
// rather than just a compile+elaborate sanity run (see tb_puzzle.cpp).
//
// Usage: sim_puzzle_replay <stimulus.txt> <out.vcd>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#include "Vpuzzle.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

static vluint64_t main_time = 0;
double sc_time_stamp() { return main_time; }

static void apply(Vpuzzle *dut, const std::string &name, int val) {
    if (name == "clk") dut->clk = val;
    else if (name == "rst_n") dut->rst_n = val;
    else if (name == "enable") dut->enable = val;
    else if (name == "I") dut->I = val;
    else {
        fprintf(stderr, "tb_puzzle_replay: unknown stimulus signal %s\n", name.c_str());
        exit(1);
    }
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s <stimulus.txt> <out.vcd>\n", argv[0]);
        return 1;
    }
    std::ifstream in(argv[1]);
    if (!in) {
        fprintf(stderr, "tb_puzzle_replay: couldn't open %s\n", argv[1]);
        return 1;
    }

    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);
    Vpuzzle *dut = new Vpuzzle;
    VerilatedVcdC *tfp = new VerilatedVcdC;
    dut->trace(tfp, 99);
    tfp->open(argv[2]);

    std::string line;
    bool have_time = false;
    long n_events = 0;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        if (line[0] == '#') {
            if (have_time) {
                dut->eval();
                tfp->dump(main_time);
            }
            main_time = std::strtoull(line.c_str() + 1, nullptr, 10);
            have_time = true;
            continue;
        }
        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        apply(dut, line.substr(0, eq), std::stoi(line.substr(eq + 1)));
        n_events++;
    }
    if (have_time) {
        dut->eval();
        tfp->dump(main_time);
    }

    tfp->close();
    delete tfp;
    delete dut;
    fprintf(stderr, "tb_puzzle_replay: replayed %ld change(s) up to t=%llu, wrote %s\n",
            n_events, (unsigned long long)main_time, argv[2]);
    return 0;
}
