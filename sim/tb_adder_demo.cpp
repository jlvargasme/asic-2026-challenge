// Verilator testbench for adder_demo -- drives the top-level pins exactly
// the way warmup/00_source.v's own protocol expects (see its
// shift_register module): async active-low reset, then 8 clock edges
// with en=1 shifting A/B in MSB-first (serial_in becomes the new LSB
// each edge, so the first bit fed ends up at bit 7 after 8 shifts).
//
// Shared between the known-correct source and the reverse-engineered
// decompiled Verilog -- both expose the identical top-level port list
// (A, B, clk, en, rst_n, S), so the same stimulus and the same VCD
// reader can drive/check either one; see sim/run_adder_demo.sh.

#include <cstdint>
#include <cstdio>

#include "Vadder_demo.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

static vluint64_t main_time = 0;
double sc_time_stamp() { return main_time; }

static Vadder_demo *dut;
static VerilatedVcdC *tfp;

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
    dut->en = 0;
    dut->A = 0;
    dut->B = 0;
    tick();
    tick();
    dut->rst_n = 1;
    tick();
}

static void load(uint8_t a_val, uint8_t b_val) {
    dut->en = 1;
    for (int bit = 7; bit >= 0; bit--) {
        dut->A = (a_val >> bit) & 1;
        dut->B = (b_val >> bit) & 1;
        tick();
    }
    dut->en = 0;
    dut->A = 0;
    dut->B = 0;
    tick();  // one settle cycle for the comparator's own combinational chain
}

static bool run_case(uint8_t a_val, uint8_t b_val) {
    reset();
    load(a_val, b_val);
    dut->eval();
    int actual = dut->S;
    int expected = (a_val + b_val == 496) ? 1 : 0;
    bool ok = actual == expected;
    printf("A=%3d B=%3d sum=%3d  expected S=%d  actual S=%d  -> %s\n",
           a_val, b_val, a_val + b_val, expected, actual, ok ? "PASS" : "FAIL");
    return ok;
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);

    dut = new Vadder_demo;
    tfp = new VerilatedVcdC;
    dut->trace(tfp, 99);
    tfp->open(argc > 1 ? argv[1] : "wave.vcd");

    bool ok = true;
    ok &= run_case(248, 248);  // 496 -- exact match, S must go high
    ok &= run_case(100, 100);  // 200 -- no match
    ok &= run_case(255, 241);  // 496 -- a different bit pattern summing to the same target
    ok &= run_case(0, 0);      // 0 -- no match
    ok &= run_case(255, 255);  // 510 -- no match, exercises the carry-out bit

    tfp->close();
    delete tfp;
    delete dut;

    printf(ok ? "ALL CASES PASSED\n" : "SOME CASES FAILED\n");
    return ok ? 0 : 1;
}
