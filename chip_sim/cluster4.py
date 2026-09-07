def cluster4(A2, A5, A6, A7):
    # A1_55 assumed 0 -- only A3[7] and A3[10] referenced it in the
    # original puzzle.v cluster_type_4; both simplified below with that
    # substitution applied (verified against the original formula with
    # A1_55=0 for all 16 (A2,A5,A6,A7) combinations: 0 mismatches).
    A3 = [0] * 14

    A3[0] = (
        (not A2 and not A6 and not A7)
        or (not A5 and not A6 and not A7)
    )

    A3[1] = (
        (A2 and not A5 and not A7)
        or (A6 and not A7)
    )

    A3[2] = (
        A2 and A5 and not A6 and not A7
    )

    A3[3] = (
        (not A2 and not A5 and not A6)
        or (not A2 and not A7)
        or (not A5 and not A7)
        or (A6 and not A7)
    )

    A3[4] = (
        (not A2 and A5 and A6 and not A7)
        or (not A2 and not A5 and not A6)
    )

    A3[5] = (
        (not A2 and not A5 and not A6 and A7)
        or (not A2 and A5 and not A7)
    )

    A3[6] = (
        (not A2 and not A5 and not A6 and A7)
        or (A2 and not A5 and not A6 and not A7)
        or (A2 and A5 and A6 and not A7)
    )

    A3[7] = (
        (A2 and not A6 and not A7)
        or (not A2 and A6 and not A7)
        or (A5 and not A6 and A7)
        or (not A2 and A5 and not A6)
    )

    A3[8] = (
        (not A2 and A5 and not A7)
        or (A2 and not A5 and not A6)
        or (A5 and not A6 and A7)
    )

    A3[9] = (
        (not A2 and not A5)
        or (not A2 and not A7)
        or (not A5 and not A7)
        or (not A6 and A7)
    )

    A3[10] = (
        (A2 and not A5 and A6 and not A7)
        or (not A2 and not A6 and not A7)
        or (not A5 and not A6 and A7)
    )

    A3[11] = (
        A2 and A5 and not A7
    )

    A3[12] = (
        (not A2 and not A6)
        or (not A5 and not A6)
        or (not A2 and not A7)
        or (not A5 and not A7)
    )

    A3[13] = (
        (A2 and not A5 and not A6 and not A7)
        or (not A2 and A5 and not A6 and A7)
        or (not A2 and not A5 and A6)
        or (not A2 and A6 and not A7)
    )

    return [int(x) for x in A3]


def driver():
    print("A2 A5 A6 A7 | A3_bus")
    print("-" * 30)

    for A2 in [0, 1]:
        for A5 in [0, 1]:
            for A6 in [0, 1]:
                for A7 in [0, 1]:
                    output = cluster4(A2, A5, A6, A7)

                    # Verilog bus is [13:0], so print MSB -> LSB
                    output_str = "".join(
                        map(str, reversed(output))
                    )

                    print(
                        f"{A2}  {A5}  {A6}  {A7}    "
                        f"| {output_str}"
                    )


if __name__ == "__main__":
    driver()
