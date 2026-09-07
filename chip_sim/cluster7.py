"""cluster7.py: concrete cycle-by-cycle simulation of cluster_type_7, and
(mirroring cluster3.py's shift-chain trigger search) a search for `I`
sequences that satisfy or defeat A_28.

    module cluster_type_7 (
        input [4:0] A_40_bus, input A_8, input I, input clk, input rst_n,
        output A_28, output [1:0] A1_38_bus
    );
        assign A_28 = ~A_90;
        assign A1_38_bus[1] = A_40_bus[3] | A_40_bus[2] | A_40_bus[1] | A_40_bus[0];
        assign A1_38_bus[0] = ~A_40_bus[1] | ~A_40_bus[3] | A_40_bus[2] | A_40_bus[0];
        assign D_70 = (~A_40_bus[4] & A_8 & A_91 & I) | (~A_40_bus[4] & A_88) | (~A_8 & A_88);
        assign D_71 = A_90 | (A_40_bus[4] & A_8 & ~A_88 & ~A_91) | (A_40_bus[4] & A_8 & A_91 & ~I)
                    | (A_40_bus[4] & A_8 & A_88 & I);
        assign D_72 = (~A_40_bus[4] & A_8 & ~A_91 & I) | (~A_40_bus[4] & A_91 & ~I) | (~A_8 & A_91)
                    | (~A_40_bus[4] & A_8 & A_88 & I);
        ... dfrtp_2 x3: D_70->A_88, D_71->A_90, D_72->A_91
    endmodule

A_90 is a STICKY latch, same shape as cluster_type_3's A_80 and
cluster_type_12's A_27: D_71's leading `A_90 |` term self-holds it once
set, and rst_n is the only way to clear it. A_28 = ~A_90, so A_28 starts
at 1 (A_90 resets to 0) and can only ever go 1 -> 0, never back --
"search for I that satisfies A_28=1" is really "search for I that AVOIDS
ever tripping A_90", the mirror image of cluster3.py's own "search for
I that TRIGGERS the sticky latch."

Only A_40_bus[4] feeds the D-rules (A_40_bus[0:3] only feed the
unrelated A1_38_bus combinational outputs -- see and11_A_48_bus() style
note in cluster3.py/cluster2.py for the same kind of "not everything in
this port is state-relevant" split), so the register is modeled here
against a single scalar `A4` rather than the whole 5-bit bus.
"""

REG_BITS = ["A_88", "A_90", "A_91"]  # instance order i1, i3, i4


def next_state(state, A4, A_8, I):
    """state: bitvector value of REG_BITS (bit0=A_88, bit1=A_90,
    bit2=A_91) -- this module's own current Q outputs, read here purely
    as D-rule inputs. Returns (next_state, A_28)."""
    A_88 = (state >> 0) & 1
    A_90 = (state >> 1) & 1
    A_91 = (state >> 2) & 1

    A_28 = (~A_90) & 1
    D_70 = ((~A4 & A_8 & A_91 & I) | (~A4 & A_88) | (~A_8 & A_88)) & 1
    D_71 = (A_90 | (A4 & A_8 & ~A_88 & ~A_91) | (A4 & A_8 & A_91 & ~I) | (A4 & A_8 & A_88 & I)) & 1
    D_72 = ((~A4 & A_8 & ~A_91 & I) | (~A4 & A_91 & ~I) | (~A_8 & A_91) | (~A4 & A_8 & A_88 & I)) & 1

    next_state_val = D_70 | (D_71 << 1) | (D_72 << 2)
    return next_state_val, A_28


def reset_state():
    return 0  # rst_n=0 -> A_88=A_90=A_91=0


def a1_38_bus(A_40_bus):
    """A1_38_bus -- purely combinational, doesn't touch this module's
    own state at all. A_40_bus: 4-bit int covering bits [3:0] only
    (bit4 is handled separately by next_state()'s own `A4` argument)."""
    b0 = (A_40_bus >> 0) & 1
    b1 = (A_40_bus >> 1) & 1
    b2 = (A_40_bus >> 2) & 1
    b3 = (A_40_bus >> 3) & 1
    bit1 = (b3 | b2 | b1 | b0) & 1
    bit0 = ((~b1) | (~b3) | b2 | b0) & 1
    return bit0 | (bit1 << 1)


def verify_A90_gate():
    """Exhaustively checks that A_90 only ever CAN change while A4=1 AND
    A_8=1 simultaneously -- every other (A4,A_8) combination holds A_90
    at its current value, for every state and every I. Returns violation
    count (0 = confirmed)."""
    violations = 0
    for state in range(8):
        A_90 = (state >> 1) & 1
        for A4 in (0, 1):
            for A_8 in (0, 1):
                if A4 == 1 and A_8 == 1:
                    continue
                for I in (0, 1):
                    nstate, _ = next_state(state, A4, A_8, I)
                    next_A_90 = (nstate >> 1) & 1
                    if next_A_90 != A_90:
                        violations += 1
    return violations


def find_trigger_sequence(max_len=6):
    """Mirrors cluster3.py's own shift-chain trigger search, but for
    A_90: BFS over sequences of (A4, A_8, I) from reset, shortest first,
    looking for the first step where A_90 becomes 1 (A_28 drops to 0).
    Returns (length, sequence) or None if no trigger exists within
    max_len -- small state space (8 states x 8 input combos), so this is
    a real exhaustive answer up to that bound, not a heuristic."""
    from collections import deque

    start = reset_state()
    queue = deque([(start, [])])
    seen = {start}
    while queue:
        state, path = queue.popleft()
        if len(path) >= max_len:
            continue
        for A4 in (0, 1):
            for A_8 in (0, 1):
                for I in (0, 1):
                    nstate, A_28 = next_state(state, A4, A_8, I)
                    if A_28 == 0:
                        return len(path) + 1, path + [(A4, A_8, I)]
                    if nstate not in seen:
                        seen.add(nstate)
                        queue.append((nstate, path + [(A4, A_8, I)]))
    return None


def find_safe_sequence(length):
    """The search the user actually asked for: a sequence of `length`
    (A4, A_8, I) triples, starting from reset, that keeps A_28=1 the
    WHOLE way through (i.e. never trips A_90) -- exhaustive DFS over the
    (tiny) 8-state graph, restricted to only the transitions that don't
    set A_90. Returns one such sequence (there may be many; this returns
    the first found in (A4,A_8,I) enumeration order), or None if
    impossible for that length."""
    def dfs(state, remaining):
        if remaining == 0:
            return []
        for A4 in (0, 1):
            for A_8 in (0, 1):
                for I in (0, 1):
                    nstate, A_28 = next_state(state, A4, A_8, I)
                    if A_28 == 1:
                        rest = dfs(nstate, remaining - 1)
                        if rest is not None:
                            return [(A4, A_8, I)] + rest
        return None

    return dfs(reset_state(), length)


def longest_safe_run(cap=64):
    """How long A_28=1 can be sustained AT ALL, over every possible
    (A4,A_8,I) choice at every step -- i.e. is there a genuinely
    INFINITE safe trajectory (a cycle in the "A_28 stays 1" subgraph
    reachable from reset), or does every path eventually get forced into
    tripping A_90? Returns ('infinite', cycle_len) if a safe cycle is
    reachable within `cap` steps, else (length, None) for the longest
    finite safe run found."""
    # states reachable from reset while staying safe, plus which SAFE
    # transitions are available from each -- a plain reachability
    # closure, small enough (<=8 states) to do exactly.
    from collections import deque

    start = reset_state()
    safe_reachable = {start}
    edges = {}  # state -> list of (A4,A_8,I,nstate) safe transitions
    queue = deque([start])
    while queue:
        s = queue.popleft()
        edges[s] = []
        for A4 in (0, 1):
            for A_8 in (0, 1):
                for I in (0, 1):
                    ns, A_28 = next_state(s, A4, A_8, I)
                    if A_28 == 1:
                        edges[s].append((A4, A_8, I, ns))
                        if ns not in safe_reachable:
                            safe_reachable.add(ns)
                            queue.append(ns)

    # any state with a safe transition back to an ALREADY-visited state
    # on its own path means an infinite safe loop exists -- check via a
    # simple DFS cycle detection over the safe-edges graph.
    WHITE, GRAY, BLACK = 0, 1, 2
    color = {s: WHITE for s in safe_reachable}

    def dfs_cycle(s, depth):
        if depth > cap:
            return None
        color[s] = GRAY
        for A4, A_8, I, ns in edges[s]:
            if color[ns] == GRAY:
                return depth + 1  # found a back-edge -- cycle of this length
            if color[ns] == WHITE:
                found = dfs_cycle(ns, depth + 1)
                if found is not None:
                    return found
        color[s] = BLACK
        return None

    cyc = dfs_cycle(start, 0)
    if cyc is not None:
        return "infinite", cyc

    # no cycle: longest safe run is just the longest path in a DAG
    memo = {}

    def longest_from(s):
        if s in memo:
            return memo[s]
        best = 0
        for A4, A_8, I, ns in edges[s]:
            best = max(best, 1 + longest_from(ns))
        memo[s] = best
        return best

    return longest_from(start), None


def A4_from_A_75(t):
    """cluster9's own A_75 fires exactly when its count==10, which
    happens every 11th cycle starting at t=10 (see cluster9.py's
    print_transition_table()/free-running trace) -- A4 here stands in
    for A_40_bus[4] being wired to that same signal."""
    return 1 if (t % 11 == 10) else 0


def print_cycle_timeline(num_cycles, A_8=1):
    """Prints, cycle by cycle, every DISTINCT register state reachable
    from reset by SOME sequence of I choices, and where each one goes
    for both I=0 and I=1 -- the deduplicated level-by-level trellis
    (see the state-trellis diagram from earlier), not the raw
    exponential tree of I-paths (2**num_cycles of those, almost all
    landing on repeats -- see A4_from_A_75()'s own cycle-10 collapse for
    why deduping is the right unit here, not path count)."""
    states = {reset_state()}
    for t in range(num_cycles):
        A4 = A4_from_A_75(t)
        print(f"\n===== cycle {t} (A4={A4}) =====\n")
        print(f"{'state':<8}{'I':^6}{'next state':<12}A_28")
        next_states = set()
        for s in sorted(states):
            for I in (0, 1):
                ns, A_28 = next_state(s, A4, A_8, I)
                print(f"{s:03b}     -{I}->     {ns:03b}      {A_28}")
                next_states.add(ns)
        states = next_states


if __name__ == "__main__":
    print(f"A_90-gate violations (should only change under A4=1 & A_8=1): {verify_A90_gate()}")

    trig = find_trigger_sequence()
    print(f"shortest (A4,A_8,I) sequence that trips A_28=0: {trig}")

    result = longest_safe_run()
    if result[0] == "infinite":
        print(f"an INFINITE safe trajectory exists (found a {result[1]}-step cycle) -- "
              f"A_28=1 can be sustained forever with the right (A4,A_8,I) choices")
    else:
        print(f"no infinite safe trajectory exists -- longest possible run with A_28=1 "
              f"held throughout is {result[0]} steps")

    print()
    for length in range(0, 6):
        seq = find_safe_sequence(length)
        print(f"  length {length}: {'OK -- ' + str(seq) if seq is not None else 'IMPOSSIBLE'}")

    print_cycle_timeline(13)
