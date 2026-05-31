"""Headless test — verify copper pipeline via terminal output.

Runs the copper DAG for ~5 seconds and logs every callback invocation.
No GUI / OpenGL required.
"""

import sys
import time

import ironarm_sim

HEADER = f"{'t':>6s} {'j0':>8s} {'j1':>8s} {'j2':>8s} {'j3':>8s} {'wx':>8s} {'wy':>8s} {'wz':>8s}"
SEP = "-" * len(HEADER)

call_count = 0
last_log_time = time.time()
start_time: float | None = None


def log_callback(
    j0: float,
    j1: float,
    j2: float,
    j3: float,
    wx: float,
    wy: float,
    wz: float,
) -> None:
    global call_count, last_log_time, start_time
    if start_time is None:
        start_time = time.time()
    call_count += 1
    now = time.time()
    if now - last_log_time >= 0.2:
        last_log_time = now
        elapsed = now - start_time
        print(
            f"{elapsed:5.1f}s {j0:+7.3f} {j1:+7.3f} {j2:+7.3f} {j3:+7.3f} {wx:+7.3f} {wy:+7.3f} {wz:+7.3f}"
        )


def main() -> int:
    # Register callback BEFORE starting copper
    ironarm_sim.register_callback(log_callback)
    print("Callback registered.")
    print("Starting copper runtime...")
    ironarm_sim.start_copper()

    # Wait a moment and check if copper is actually running
    time.sleep(0.5)
    print(f"After 0.5s: {call_count} callbacks received")

    time.sleep(5.5)
    print(SEP)
    print(f"Final: {call_count} callbacks in ~6s")

    if call_count == 0:
        print("\nFAIL: No callbacks — copper DAG is not producing output")
        return 1

    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
