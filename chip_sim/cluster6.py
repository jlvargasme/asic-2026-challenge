"""cluster6.py: cluster_type_6 -- purely combinational decoder, same
A_2/A_5/A_6/A_7 inputs as cluster_type_4, feeding A1_12_bus[12:0] into
cluster_type_1 (see puzzle_wip.v's own "some sort of decoder" comment
and truth table, transcribed directly from the `assign` statements
below rather than the table, then cross-checked against it)."""


def compute(A_2, A_5, A_6, A_7):
    b = [0] * 13
    b[0] = (A_2 & ~A_5 & ~A_7) | (A_2 & A_6 & ~A_7)
    b[1] = (A_2 & A_6 & ~A_7) | (~A_5 & ~A_7)
    b[2] = A_2 & A_5 & ~A_6 & ~A_7
    b[3] = (~A_2 & A_5 & ~A_6 & ~A_7) | (A_2 & ~A_5 & A_6 & ~A_7)
    b[4] = (~A_2 & ~A_7) | (~A_5 & ~A_7) | (A_6 & ~A_7)
    b[5] = (A_2 & ~A_5 & ~A_6 & ~A_7) | (~A_2 & A_5 & ~A_7) | (A_5 & A_6 & ~A_7)
    b[6] = (~A_2 & ~A_5 & ~A_6) | (~A_2 & ~A_6 & ~A_7) | (A_2 & A_6 & ~A_7) | (~A_2 & ~A_5 & ~A_7)
    b[7] = (~A_2 & ~A_5 & ~A_6 & A_7) | (~A_5 & A_6 & ~A_7) | (A_2 & ~A_6 & ~A_7)
    b[8] = (~A_2 & ~A_5 & ~A_6) | (~A_5 & ~A_7) | (~A_6 & ~A_7) | (A_2 & ~A_7)
    b[9] = (~A_2 & ~A_6 & ~A_7) | (A_5 & ~A_6 & ~A_7)
    b[10] = A_2 & A_6 & ~A_7
    b[11] = ~A_2 & A_5 & A_6 & ~A_7
    b[12] = (~A_2 & ~A_5 & ~A_6 & A_7) | (~A_2 & ~A_5 & A_6 & ~A_7) | (~A_2 & A_5 & ~A_6 & ~A_7) | (A_2 & A_5 & A_6 & ~A_7)
    return sum((v & 1) << i for i, v in enumerate(b))


# module's own dumped truth table (A2 A5 A6 A7 -> A1_12_bus, MSB..LSB
# text order) -- used as an independent cross-check on `compute()`.
_TABLE_TEXT = """
0 0 0 0  0001101010010
0 0 0 1  1000111000000
0 0 1 0  1000111010010
0 0 1 1  0000000000000
0 1 0 0  1001101111000
0 1 0 1  0000000000000
0 1 1 0  0100000110000
0 1 1 1  0000000000000
1 0 0 0  0000110110011
1 0 0 1  0000000000000
1 0 1 0  0010111011011
1 0 1 1  0000000000000
1 1 0 0  0001110000100
1 1 0 1  0000000000000
1 1 1 0  1010101110011
1 1 1 1  0000000000000
"""


def _self_check():
    mismatches = 0
    for line in _TABLE_TEXT.strip().splitlines():
        parts = line.split()
        A_2, A_5, A_6, A_7 = (int(x) for x in parts[:4])
        got = compute(A_2, A_5, A_6, A_7)
        got_str = f"{got:013b}"
        if got_str != parts[4]:
            mismatches += 1
            print(f"MISMATCH A2={A_2} A5={A_5} A6={A_6} A7={A_7}: "
                  f"compute={got_str} table={parts[4]}")
    return mismatches


if __name__ == "__main__":
    print("mismatches against the module's own dumped truth table:", _self_check())
