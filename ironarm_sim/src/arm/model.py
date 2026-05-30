"""MuJoCo arm model path and IK helpers."""

import math
import os

# ---------------------------------------------------------------------------
# MJCF model path
# ---------------------------------------------------------------------------

_MODEL_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "models")
MODEL_PATH = os.path.join(_MODEL_DIR, "ironarm.xml")


# ---------------------------------------------------------------------------
# Trajectory (used for target marker — IK is done in Rust)
# ---------------------------------------------------------------------------

def trajectory(t: float, base_z: float) -> tuple[float, float, float]:

    """Circular trajectory in MuJoCo Z-up coordinates."""
    phase = t * 2.0 * math.pi / 5.0
    return (1.2 * math.cos(phase), 1.2 * math.sin(phase), base_z + 0.5)
