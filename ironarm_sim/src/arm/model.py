"""MuJoCo arm model path and IK helpers."""

import math
import os

# ---------------------------------------------------------------------------
# MJCF model path
# ---------------------------------------------------------------------------

MODEL_PATH = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "ironarm_model", "ironarm.xml"
)


# ---------------------------------------------------------------------------
# Trajectory (used for target marker — IK is done in Rust)
# ---------------------------------------------------------------------------


def trajectory(t: float, base_z: float) -> tuple[float, float, float]:
    """Circular trajectory in MuJoCo Z-up coordinates.

    Uses workspace-reachable parameters:
    (r - l0)² + (z - base_z)² = l1² where l0=1, l1=2.
    For z = base_z + 0.5: r ≈ 2.936.
    """
    r = 2.936
    z = base_z + 0.5
    phase = t * 2.0 * math.pi / 20.0
    return (r * math.cos(phase), r * math.sin(phase), z)
