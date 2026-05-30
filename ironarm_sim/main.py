"""IronArm simulation using Rust-powered IK."""

import time

from ironarm_sim import compute_angles


def main():
    t = 0.0
    dt = 0.016

    for _ in range(5):
        angles = compute_angles(1.0, 2.0, 0.5, t)
        if angles:
            print(f"t={t:.2f}s  j0={angles[0]:.3f}  j1={angles[1]:.3f}")
        else:
            print(f"t={t:.2f}s  unreachable")
        t += dt
        time.sleep(0.016)


if __name__ == "__main__":
    main()
