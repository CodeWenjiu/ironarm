//! PyO3 bindings: expose ironarm_core algorithms to Python.

use pyo3::prelude::*;

/// Python: compute joint angles for a circular trajectory at time *t*.
#[pyo3::pyfunction]
fn compute_angles(l0: f32, l1: f32, base_z: f32, t: f32) -> Option<(f32, f32)> {
    ironarm_core::trajectory::compute_circle_angles(t, l0, l1, base_z)
}

#[pyo3::pymodule]
fn ironarm_sim(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(pyo3::wrap_pyfunction!(compute_angles, m)?)?;
    Ok(())
}
