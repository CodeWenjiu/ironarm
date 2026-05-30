"""MuJoCo arm model and simulation state."""

import math
import os

# ---------------------------------------------------------------------------
# MJCF model path
# ---------------------------------------------------------------------------

_MODEL_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "models")
MODEL_PATH = os.path.join(_MODEL_DIR, "ironarm.xml")


# ---------------------------------------------------------------------------
# IK helpers (Python version of ironarm_core logic)
# ---------------------------------------------------------------------------


def compute_angles(
    t: float, l0: float, l1: float, base_y: float
) -> tuple[float, float, float, float, float] | None:
    """Return (j0, j1, tx, ty, tz) for the circular trajectory at time *t*."""
    phase = t * 2.0 * math.pi / 5.0
    tx, ty, tz = 1.2 * math.cos(phase), base_y + 0.5, 1.2 * math.sin(phase)

    dx, dz = tx, tz
    r = math.hypot(dx, dz)
    h = ty - base_y
    d_sq = r * r + h * h
    d = math.sqrt(d_sq)

    if d > l0 + l1 or d < abs(l0 - l1):
        return None

    cos_elbow = (d_sq - l0 * l0 - l1 * l1) / (2.0 * l0 * l1)
    elbow_angle = math.acos(max(-1.0, min(1.0, cos_elbow)))
    target_elevation = math.atan2(h, r)
    link1_offset = math.atan2(
        l1 * math.sin(elbow_angle), l0 + l1 * math.cos(elbow_angle)
    )
    j0 = math.atan2(tz, tx)
    j1 = target_elevation - link1_offset

    return (j0, j1, tx, ty, tz)
