---
name: ironarm-debug
description: Debug IronArm pipeline issues — arm not moving, joints stuck, segments invisible, IK producing wrong angles, copper DAG not producing callbacks. Use when the user reports arm behavior problems.
---

# IronArm Debug Checklist

When the arm doesn't move, looks wrong, or copper produces no output, run through
this checklist IN ORDER.  Every fix is verified with a headless test BEFORE
touching the GUI.

---

## 0. Always test headless first

Use `ironarm_std/examples/check.rs` (pure Rust) or
`ironarm_sim/src/headless_test.py` (Python) before opening the GUI.
Never debug through the GUI directly.

```bash
# Rust headless test
cargo run --example check -p ironarm_std

# Python headless test
cd ironarm_sim && uv run --no-sync python -m src.headless_test
```

If the headless test produces zero callbacks, **the DAG is not executing**.
Go to Section 1.

---

## 1. Copper DAG produces zero callbacks

### 1.1 CuConsoleMon blocks in headless mode

The `CuConsoleMon` TUI monitor requires a real terminal.  In headless mode
(CI, ssh, background), it blocks the entire copper run loop.

**Fix**: Remove the `monitor` line from `copperconfig.ron` for headless testing.

### 1.2 StateSink adaptive skip with NAN initial value

`StateSink` skips callbacks when joint angles haven't changed.  If initialized
with `f32::NAN`, the comparison `(NAN - a).abs() < 1e-6` is always `false`,
so the FIRST callback is skipped — and the pipeline appears dead.

**Fix**: Initialize `last: [f32::INFINITY; N]` instead of `[f32::NAN; N]`.

### 1.3 Config parse failure (silent panic in background thread)

`#[copper_runtime(config = "...")]` embeds the config at compile time.
If the config has a type mismatch or bad format, the copper builder panics.
In the sim, copper runs in a background thread; the panic is silent.

**Fix**: Use `std::panic::catch_unwind` around `run_tui()` and expose the
error message to Python via `get_last_error()`.

---

## 2. Arm doesn't move (joints stuck at 0)

### 2.1 Trajectory outside workspace

If ALL joint angles are 0, the waypoint is outside the arm's reachable
workspace.  The IK solver returns `None` and defaults to `[0; N]`.

**Check**: Compute `d = sqrt(r² + (z - shoulder_z)²)`.  Must satisfy:
`|l1 - l2_eff| ≤ d ≤ l1 + l2_eff`.

**Fix**: Adjust trajectory radius/height in `copperconfig.ron`.

### 2.2 Wrong IK sign convention

The MuJoCo arm uses Z-up coordinates.  Positive j1/j2 rotates the arm DOWN
(around Y axis, right-hand rule sends +X toward -Z).  Negative j1/j2 lifts
the arm UP.

**Verify with MuJoCo FK**:
```python
import mujoco
m = mujoco.MjModel.from_xml_path(MODEL_PATH)
d = mujoco.MjData(m)
d.joint('j1').qpos[0] = -0.5  # should lift arm up
mujoco.mj_forward(m, d)
print(d.xpos[m.body('elbow').id])  # z should increase
```

---

## 3. Visible joints but some appear frozen

### 3.1 Circle at constant height/radius only exercises j0

A horizontal circle trajectory (constant z, constant r) produces waypoints
that differ only in (x, y).  The IK solution for j1 and j2 depends only
on r and z, so they stay constant while j0 sweeps.

**Fix**: Use a trajectory that varies in z or r.  Linear trajectory between
two points at different heights is a quick test.

---

## 4. Arm segments not visible

### 4.1 Elbow underground (wrong IK elbow configuration)

The 2-link positional IK has TWO solutions: elbow-up and elbow-down.
The default `j2 = alpha - pi` picks elbow-down, which can put the elbow
below ground.

```rust
// Elbow-down (may go underground):
let j2 = alpha - PI;

// Elbow-up (stays above shoulder-target line):
let j2 = PI - alpha;
```

**Check**: Look at `el.z` in the diagnostic output.  If near or below 0,
the elbow configuration is wrong.

**Also check**: Expand joint range limits in the model XML if the new
configuration exceeds the old limits.

### 4.2 End-effector offset too short

If the wrist-to-EE distance is < 10cm, the segment is visually indistinct.

**Fix**: Make the wrist extension at least 20-25cm with a distinct color:
```xml
<geom type="capsule" size="0.025" fromto="0 0 0 0.25 0 0" rgba="1.0 0.6 0.2 1"/>
```

### 4.3 Camera pointing at wrong location

The default camera `lookat` may not frame the arm well.  Adjust to center
on the expected elbow/EE position:
```python
self._cam.lookat[:] = (1.0, 0.0, 0.8)  # higher, further right
```

### 4.4 Forearm geom too thin

Capsule `size` sets the radius.  0.03 is 3cm — barely visible at scale.
Use 0.04–0.05 for visibility.

---

## 5. Model validation checklist

Before trusting the IK solver, validate with MuJoCo FK:

1. Load the model: `m = MjModel.from_xml_path(MODEL_PATH)`
2. Set known joint angles: `d.joint('j0').qpos[0] = val`
3. Compute FK: `mj_forward(m, d)`
4. Check body positions: `d.xpos[m.body('elbow').id]`
5. Trace the kinematic chain: shoulder → upper arm → elbow → forearm → wrist → ee

The arm should extend +X when all joints are 0, bend up with negative j1/j2,
and bend down with positive j1/j2.

---

## 6. Coordinate system conventions (MuJoCo Z-up)

| Action | Joint | Axis | Sign |
|--------|-------|------|------|
| Arm lifts up | j1, j2 | Y | **negative** |
| Arm lowers | j1, j2 | Y | **positive** |
| Base rotates CCW (top view) | j0 | Z | positive |
| Wrist rolls | j3 | X | depends on orientation |

Right-hand rule around Y: thumb +Y, fingers curl from +X → -Z.
So positive rotation sends the arm tip DOWN.
