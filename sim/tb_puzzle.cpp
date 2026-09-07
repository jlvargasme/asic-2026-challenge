// Verilator testbench for puzzle.v -- a generic drive-and-observe
// harness (no known-correct reference exists for this design, unlike
// adder_demo/warmup): async-style reset, then N clock edges with a
// chosen `enable`/`I` sequence, printing O[7:0]/success after every
// cycle and dumping a VCD. This is a compile+elaborate sanity check on
// the decompiled RTL and a way to eyeball behavior -- NOT a claim about
// what input sequence actually makes `success` go high (that's the
// puzzle itself; see clustering.py's backward-search tools for that).
//
// Per HANDOFF.md's own rule, this file only ever drives/reads the
// chip's own TRUE top-level ports (I, clk, enable, rst_n, O[7:0],
// success) -- genuinely public interface, not internal structure.

#include <cstdint>
#include <cstdio>
#include <vector>

#include "Vpuzzle.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

static vluint64_t main_time = 0;
double sc_time_stamp() { return main_time; }

static Vpuzzle *dut;
static VerilatedVcdC *tfp;

static uint8_t read_o() {
    return (dut->O_7_ << 7) | (dut->O_6_ << 6) | (dut->O_5_ << 5) | (dut->O_4_ << 4) |
           (dut->O_3_ << 3) | (dut->O_2_ << 2) | (dut->O_1_ << 1) | dut->O_0_;
}

static void tick() {
    dut->clk = 0;
    dut->eval();
    tfp->dump(main_time);
    main_time += 5;
    dut->clk = 1;
    dut->eval();
    tfp->dump(main_time);
    main_time += 5;
}

static void reset() {
    dut->rst_n = 0;
    dut->enable = 0;
    dut->I = 0;
    for (int i = 0; i < 4; i++) tick();
    dut->rst_n = 1;
    tick();
    printf("  after reset: O=0x%02x success=%d\n", read_o(), dut->success);
}

// Drives `bits` (MSB-first) on I with enable=1, one bit per clock edge,
// printing O/success after every single cycle so the whole trajectory
// is visible, not just the final state.
static void run_sequence(const char *label, const std::vector<int> &bits) {
    printf("--- %s ---\n", label);
    reset();
    dut->enable = 1;
    for (size_t i = 0; i < bits.size(); i++) {
        dut->I = bits[i];
        tick();
        printf("  cycle %2zu  I=%d  ->  O=0x%02x  success=%d\n", i, bits[i], read_o(), dut->success);
    }
    dut->enable = 0;
    dut->I = 0;
    tick();
    printf("  settle    ->  O=0x%02x  success=%d\n", read_o(), dut->success);
    printf("\n");
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);

    dut = new Vpuzzle;
    tfp = new VerilatedVcdC;
    dut->trace(tfp, 99);
    tfp->open(argc > 1 ? argv[1] : "wave.vcd");

    run_sequence("all zeros (8 cycles)", {0, 0, 0, 0, 0, 0, 0, 0});
    run_sequence("all ones (8 cycles)", {1, 1, 1, 1, 1, 1, 1, 1});
    run_sequence("alternating 10101010 (8 cycles)", {1, 0, 1, 0, 1, 0, 1, 0});
    run_sequence("longer mixed pattern (16 cycles)",
                 {1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0});

    tfp->close();
    delete tfp;
    delete dut;
    return 0;
}
