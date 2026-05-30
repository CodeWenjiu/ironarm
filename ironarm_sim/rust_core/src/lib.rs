//! PyO3 bindings: expose ironarm_core algorithms to Python.

use pyo3::prelude::*;
use std::f32::consts::PI;

/// End effector target.
struct EETarget {
    x: f32,
    y: f32,
    z: f32,
}

/// Arm geometry.
struct Geo {
    l0: f32,
    l1: f32,
    base_y: f32,
}

/// Solve 2-joint inverse kinematics.
fn solve_ik(target: &EETarget, geo: &Geo) -> Option<(f32, f32)> {
    let dx = target.x;
    let dy = target.y - geo.base_y;
    let dz = target.z;

    let j0 = dz.atan2(dx);
    let r = (dx * dx + dz * dz).sqrt();
    let h = dy;
    let l0 = geo.l0;
    let l1 = geo.l1;
    let d_sq = r * r + h * h;
    let d = d_sq.sqrt();

    if d > l0 + l1 || d < (l0 - l1).abs() {
        return None;
    }

    let cos_elbow = (d_sq - l0 * l0 - l1 * l1) / (2.0 * l0 * l1);
    let elbow_angle = cos_elbow.clamp(-1.0, 1.0).acos();
    let target_elevation = h.atan2(r);
    let link1_offset = (l1 * elbow_angle.sin()).atan2(l0 + l1 * elbow_angle.cos());
    let j1 = target_elevation - link1_offset;

    Some((j0, j1))
}

/// Circle trajectory.
fn tick(t: f32, geo: &Geo) -> (f32, f32, f32) {
    let phase = t * 2.0 * PI / 5.0;
    let x = 1.2 * phase.cos();
    let z = 1.2 * phase.sin();
    let y = geo.base_y + 0.5;
    (x, y, z)
}

/// Python: compute joint angles for given geometry and time.
#[pyo3::pyfunction]
fn compute_angles(l0: f32, l1: f32, base_y: f32, t: f32) -> Option<(f32, f32)> {
    let geo = Geo { l0, l1, base_y };
    let (x, y, z) = tick(t, &geo);
    let target = EETarget { x, y, z };
    solve_ik(&target, &geo)
}

#[pyo3::pymodule]
fn ironarm_sim(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(pyo3::wrap_pyfunction!(compute_angles, m)?)?;
    Ok(())
}
