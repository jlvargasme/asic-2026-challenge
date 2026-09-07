"""cluster5.py: concrete cycle-by-cycle simulation of cluster_type_10,
an 8-flip-flop state machine with three decoded outputs -- each of
A_20_bus[0], A_20_bus[1], B_14 fires only when ALL 8 registers match one
exact bit pattern (an 8-input AND of literals, same shape as
cluster_type_10's own 3-bit decoder, just wider):

    module cluster_type_5 (
        input A_8, input I, input clk, input rst_n,
        output [1:0] A_20_bus, output B_14
    );
        assign A_20_bus[0] = ~A_81 & ~A_82 & ~A_83 & ~A_85 & ~A_86 & ~A_92 & ~A1_39 & ~A_N_14;
        assign B_14        = ~A_81 & ~A_82 & ~A_83 & ~A_85 &  A_86 &  A_92 & ~A1_39 &  A_N_14;
        assign A_20_bus[1] =  A_81 & ~A_82 &  A_83 &  A_85 &  A_86 & ~A_92 &  A1_39 & ~A_N_14;
        ... (D_73/D_65/D_74/D_66/D_75/D_69/D_76/D_67 -> the 8 registers, see next_state())
    endmodule

As with cluster9/8/10, the register outputs (A1_39, A_N_14, A_92, A_81,
A_82, A_85, A_86, A_83) plus the primary inputs (A_8, I) are read purely
as D-rule INPUTS here; next_state() computes the D_xx equations exactly
as puzzle.v's own `assign` statements do, then relabels them by which
flip-flop each one feeds (a wiring fact) to get the next register state.

B_14 is exactly cluster_type_10's own B_14 input -- one of the 4
conditions (A_28 & B_14 & C & A_23_bus[1]) that has to hold simultaneously
for that module's success/outcome transition. Finding when this module's
own B_14 can be made 1 is what actually matters downstream.

256 possible states (8 bits) x 4 input combinations is small enough to
brute-force exhaustively (reachability via BFS) rather than trying to
read the 8 D-rule equations by hand.
"""

# instance order from puzzle.v's own dfrtp_2 list: i17,i19,i21,i23,i24,i29,i30,i31
REG_BITS = ["A1_39", "A_N_14", "A_92", "A_81", "A_82", "A_85", "A_86", "A_83"]


def next_state(state, A_8, I):
    """state: bitvector value of REG_BITS (bit0=A1_39 .. bit7=A_83) --
    this module's own current Q outputs, read here purely as D-rule
    inputs. Returns (next_state, A_20_bus0, B_14, A_20_bus1)."""
    A1_39 = (state >> 0) & 1
    A_N_14 = (state >> 1) & 1
    A_92 = (state >> 2) & 1
    A_81 = (state >> 3) & 1
    A_82 = (state >> 4) & 1
    A_85 = (state >> 5) & 1
    A_86 = (state >> 6) & 1
    A_83 = (state >> 7) & 1

    # combinational intermediates, in dependency order
    A_84 = (A_81 & I & A_8) & 1
    A3_26 = (A_92 & A_N_14 & A1_39) & 1
    A1_40 = (A_86 & A_85) & 1
    A2_45 = ((~A_84) | (~A3_26)) & 1
    A1_41 = ((~A_86) | (~A_85)) & 1
    B_52 = ((A1_40 & A_84 & A3_26) | A_83) & 1
    B1_44 = ((A_86 & A_84 & A3_26) | A_85) & 1
    A_N_13 = (A_83 & A1_40 & A_84 & A3_26) & 1

    D_67 = ((~A_N_13) & B_52) & 1
    D_69 = ((A2_45 & B1_44) | (A1_41 & B1_44)) & 1
    D_75 = ((~A_82 & A_N_13) | (A_82 & ~A_N_13)) & 1
    D_76 = ((~A_86 & ~A2_45) | (A_86 & A2_45)) & 1

    D_73 = ((A_8 & A_81 & A_92 & ~A1_39 & A_N_14 & I) | (~A_8 & A1_39) | (~A_81 & A1_39)
            | (~A_92 & A1_39) | (A1_39 & ~A_N_14) | (A1_39 & ~I)) & 1
    D_65 = ((A_8 & A_81 & A_92 & ~A_N_14 & I) | (~A_8 & A_N_14) | (~A_81 & A_N_14)
            | (~A_92 & A_N_14) | (A_N_14 & ~I)) & 1
    D_74 = ((A_8 & A_81 & ~A_92 & I) | (~A_8 & A_92) | (~A_81 & A_92) | (A_92 & ~I)) & 1
    D_66 = ((A_8 & ~A_81 & I) | (~A_8 & A_81) | (A_81 & ~I)) & 1

    # outputs, computed from the CURRENT (pre-edge) register values
    A_20_bus0 = (~A_81 & ~A_82 & ~A_83 & ~A_85 & ~A_86 & ~A_92 & ~A1_39 & ~A_N_14) & 1
    B_14 = (~A_81 & ~A_82 & ~A_83 & ~A_85 & A_86 & A_92 & ~A1_39 & A_N_14) & 1
    A_20_bus1 = (A_81 & ~A_82 & A_83 & A_85 & A_86 & ~A_92 & A1_39 & ~A_N_14) & 1

    # wiring: D_73->A1_39, D_65->A_N_14, D_74->A_92, D_66->A_81,
    #         D_75->A_82, D_69->A_85, D_76->A_86, D_67->A_83
    next_bits = [D_73, D_65, D_74, D_66, D_75, D_69, D_76, D_67]
    next_state_val = sum(b << i for i, b in enumerate(next_bits))
    return next_state_val, A_20_bus0, B_14, A_20_bus1


def reset_state():
    return 0  # rst_n=0 -> every register = 0 (this is ALSO the A_20_bus[0] decode pattern)


def bfs_reachable():
    """BFS over all 256 states x 4 (A_8,I) input combinations from
    reset_state(). Returns (reachable: set[int], came_from: {state:
    (prev_state, A_8, I)}) -- came_from lets shortest_path() reconstruct
    the exact input sequence that reaches any given state."""
    from collections import deque

    start = reset_state()
    reachable = {start}
    came_from = {start: None}
    queue = deque([start])
    while queue:
        s = queue.popleft()
        for A_8 in (0, 1):
            for I in (0, 1):
                ns, _, _, _ = next_state(s, A_8, I)
                if ns not in reachable:
                    reachable.add(ns)
                    came_from[ns] = (s, A_8, I)
                    queue.append(ns)
    return reachable, came_from


def shortest_path(target_state, came_from):
    """(A_8, I) sequence from reset_state() to `target_state`, using
    bfs_reachable()'s own came_from map -- None if unreachable."""
    if target_state not in came_from:
        return None
    path = []
    s = target_state
    while came_from[s] is not None:
        prev, A_8, I = came_from[s]
        path.append((A_8, I))
        s = prev
    return list(reversed(path))


def hold_check():
    """Exhaustively checks that all 3 input combinations OTHER than
    (A_8,I)=(1,1) hold every one of the 256 states -- i.e. this is only
    ever a free-running counter under A_8=I=1, exactly like
    cluster9/cluster8's own A_8/A_75&A_8 enables. Returns the count of
    violations (0 = confirmed)."""
    violations = 0
    for state in range(256):
        for A_8, I in ((0, 0), (0, 1), (1, 0)):
            ns, *_ = next_state(state, A_8, I)
            if ns != state:
                violations += 1
    return violations


def trace_enabled_cycle():
    """Walks the free-running (A_8,I)=(1,1) sequence from reset until it
    returns to 0 -- confirms whether this is a single Hamiltonian cycle
    through all 256 states (a full-period 8-bit counter, same shape as
    cluster9/cluster8's mod-11 counters but mod-256) rather than settling
    into a smaller sub-cycle or a fixed point. Returns the list of states
    visited, in order (not including the final return to 0)."""
    seen = []
    s = reset_state()
    while True:
        seen.append(s)
        ns, *_ = next_state(s, 1, 1)
        if ns == reset_state():
            break
        s = ns
    return seen


def recover_bit_order():
    """Infers the LSB->MSB counting order of REG_BITS purely from the
    enabled-cycle trace, with no assumption about which raw register is
    "supposed" to be which bit -- the same question cluster9.py/
    cluster8.py answered by hand-inspecting a printed table, automated
    here instead: over one full 256-step period of a genuine binary
    counter, bit k (0=LSB) toggles exactly 256/2**k times (256, 128, 64,
    32, 16, 8, 4, 2 for k=0..7), and those 8 counts are all DISTINCT --
    so sorting REG_BITS by how many times each one actually flips across
    trace_enabled_cycle() recovers the true bit order directly.

    Returns (order, toggle_counts): `order` is REG_BITS reordered
    LSB->MSB; `toggle_counts` is {name: count}, included so the
    recovery can be checked against the expected 256/128/.../2 pattern
    rather than trusted blindly."""
    cycle = trace_enabled_cycle()
    n = len(cycle)
    toggle_counts = {name: 0 for name in REG_BITS}
    for i in range(n):
        s, ns = cycle[i], cycle[(i + 1) % n]
        for bit, name in enumerate(REG_BITS):
            if (s >> bit) & 1 != (ns >> bit) & 1:
                toggle_counts[name] += 1
    order = sorted(REG_BITS, key=lambda name: -toggle_counts[name])
    return order, toggle_counts


def count_value(state, order):
    """Re-reads `state` (packed in REG_BITS/declaration order) using an
    arbitrary LSB->MSB `order` -- a permutation of REG_BITS, e.g. the one
    recover_bit_order() returns."""
    idx = {name: i for i, name in enumerate(REG_BITS)}
    return sum(((state >> idx[name]) & 1) << i for i, name in enumerate(order))


def print_recoverable_trace(limit=32):
    """Prints the enabled-cycle trace with the raw declaration-order
    bits for the first `limit` steps (of the full 256-step cycle), then
    each register's toggle count computed over the WHOLE cycle -- enough
    to recover the true bit order either by eye (the column that changes
    on literally every row is bit0/LSB; half as often is bit1; ...) or
    via recover_bit_order()'s own exact count."""
    cycle = trace_enabled_cycle()
    header = "step | " + " ".join(f"{n:>7}" for n in REG_BITS)
    print(header)
    print("-" * len(header))
    for step, state in enumerate(cycle[:limit]):
        bits = [(state >> i) & 1 for i in range(len(REG_BITS))]
        row = " ".join(f"{b:>7}" for b in bits)
        print(f"{step:>4} | {row}")
    if limit < len(cycle):
        print(f" ... ({len(cycle) - limit} more steps, full cycle used for toggle counts below)")

    order, toggle_counts = recover_bit_order()
    print()
    print("toggle counts over the full 256-step cycle (expected: 256,128,64,32,16,8,4,2, "
          "all distinct -> order is fully determined):")
    for name in REG_BITS:
        print(f"  {name:>7}: {toggle_counts[name]:>3} toggles")
    print()
    print(f"recovered LSB->MSB order: {order}")

    values = [count_value(s, order) for s in cycle]
    print(f"count_value() under recovered order reproduces 0..255 in sequence: "
          f"{values == list(range(256))}")


if __name__ == "__main__":
    print(f"hold violations for (A_8,I) != (1,1): {hold_check()}")

    cycle = trace_enabled_cycle()
    print(f"(A_8,I)=(1,1)-held cycle length: {len(cycle)} (full 8-bit period = 256)")
    print(f"unique states visited: {len(set(cycle))}")

    for step, state in enumerate(cycle):
        _, A_20_bus0, B_14, A_20_bus1 = next_state(state, 0, 0)
        # outputs depend only on the CURRENT state, so (A_8,I)=(0,0)
        # here is just a throwaway call to read them off
        if A_20_bus0 or B_14 or A_20_bus1:
            flags = ", ".join(
                name for name, v in
                (("A_20_bus[0]", A_20_bus0), ("B_14", B_14), ("A_20_bus[1]", A_20_bus1)) if v
            )
            print(f"  step {step:>3}: state={state:08b} -> {flags}")

    print()
    print_recoverable_trace(limit=16)
