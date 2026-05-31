//! PyO3 bindings: expose ironarm_core algorithms and copper runtime to Python.

use pyo3::prelude::*;
use std::sync::Once;
use std::thread::JoinHandle;

static COPPER_STARTED: Once = Once::new();
static mut COPPER_HANDLE: Option<JoinHandle<()>> = None;

fn ensure_copper() {
    COPPER_STARTED.call_once(|| {
        let logger_path = std::path::Path::new("target/ironarm_tui_log.copper");
        let handle = std::thread::spawn(move || {
            ironarm_std::run_tui(logger_path, Some(1024 * 1024 * 10));
        });
        unsafe {
            COPPER_HANDLE = Some(handle);
        }
    });
}

/// Python: compute joint angles for the circular trajectory.
#[pyo3::pyfunction]
fn compute_angles(l0: f32, l1: f32, base_z: f32, t: f32) -> Option<(f32, f32)> {
    ironarm_core::trajectory::compute_circle_angles(t, l0, l1, base_z)
}

/// Python: start the copper runtime in a background thread.
#[pyo3::pyfunction]
fn start_copper() {
    ensure_copper();
}

/// Python: register a callback that copper will call with (j0, j1, wx, wy, wz) each cycle.
#[pyo3::pyfunction]
fn register_callback(cb: PyObject) {
    ironarm_std::tasks::state_sink::set_callback(Box::new(
        move |j0: f32, j1: f32, wx: f32, wy: f32, wz: f32| {
            Python::with_gil(|py| {
                let _ = cb.call1(py, (j0, j1, wx, wy, wz));
            });
        },
    ));
}

#[pyo3::pymodule]
fn ironarm_sim(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(pyo3::wrap_pyfunction!(compute_angles, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(start_copper, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(register_callback, m)?)?;
    Ok(())
}
