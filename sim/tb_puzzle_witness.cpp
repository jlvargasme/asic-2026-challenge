// Drives the EXACT witness I-sequence from solve_success.py against the
// real decompiled puzzle.v via Verilator, printing O[7:0]/success every
// cycle -- an independent cross-check on drive_cluster1.py's own
// (hand-transcribed cluster1.py) prediction, per the user's own request
// to verify the Python model against a real RTL simulator rather than
// trusting the transcription alone.

#include <cstdint>
#include <cstdio>
#include <vector>

#include "Vpuzzle.h"
#include "verilated.h"
#include "verilated_vcd_c.h"
#include "witness_bits.h"

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

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);

    dut = new Vpuzzle;
    tfp = new VerilatedVcdC;
    dut->trace(tfp, 99);
    tfp->open(argc > 1 ? argv[1] : "witness.vcd");

    dut->rst_n = 0;
    dut->enable = 0;
    dut->I = 0;
    for (int i = 0; i < 4; i++) tick();
    dut->rst_n = 1;
    dut->enable = 1;  // enable held at 1 from here on, no drop -- matches solve_success.py's model

    uint8_t prev_o = 0xFF;
    for (int t = 0; t < WITNESS_LEN; t++) {
        dut->I = WITNESS_BITS[t];
        tick();
        uint8_t o = read_o();
        if (o != prev_o || dut->success) {
            printf("cycle %3d  I=%d  O=0x%02x  success=%d\n", t, WITNESS_BITS[t], o, dut->success);
            prev_o = o;
        }
    }
    dut->I = 0;
    for (int t = WITNESS_LEN; t < WITNESS_LEN + 300; t++) {
        tick();
        uint8_t o = read_o();
        if (o != prev_o) {
            printf("cycle %3d  I=0  O=0x%02x  success=%d\n", t, o, dut->success);
            prev_o = o;
        }
    }

    tfp->close();
    delete tfp;
    delete dut;
    return 0;
}
