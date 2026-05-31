"""Headless test — verify copper pipeline via terminal output."""

import sys
import time

import ironarm_sim

HEADER = f"{'t':>6s} {'j0':>8s} {'j1':>8s} {'j2':>8s} {'j3':>8s} {'wx':>8s} {'wy':>8s} {'wz':>8s}"
SEP = "-" * len(HEADER)


def main() -> int:
    print("Starting copper runtime (headless, polling)...")
    print(HEADER)
    print(SEP)

    ironarm_sim.start_copper()

    start = time.time()
    count = 0
    last_print = start

    deadline = time.time() + 6.0
    while time.time() < deadline:
        state = ironarm_sim.poll_state()
        if state is not None:
            count += 1
            now = time.time()
            if now - last_print >= 0.2:
                last_print = now
                elapsed = now - start
                j0, j1, j2, j3, wx, wy, wz = state
                print(
                    f"{elapsed:5.1f}s {j0:+7.3f} {j1:+7.3f} {j2:+7.3f} {j3:+7.3f} {wx:+7.3f} {wy:+7.3f} {wz:+7.3f}"
                )
        else:
            time.sleep(0.001)  # don't busy-wait

    print(SEP)
    print(f"Final: {count} states in ~6s")
    if count == 0:
        print("FAIL: No data in ring buffer")
        return 1
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
