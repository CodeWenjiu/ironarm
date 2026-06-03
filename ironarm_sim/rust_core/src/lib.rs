//! PyO3 bindings: expose ironarm_core algorithms and copper runtime to Python.

use pyo3::prelude::*;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Mutex, Once};
use std::thread::JoinHandle;
use std::time::Duration;

static COPPER_STARTED: Once = Once::new();
static COPPER_HANDLE: Mutex<Option<JoinHandle<()>>> = Mutex::new(None);
static COPPER_RUNNING: AtomicBool = AtomicBool::new(false);

fn ensure_copper() {
    COPPER_STARTED.call_once(|| {
        let logger_path = std::path::Path::new("target/ironarm_tui_log.copper");
        COPPER_RUNNING.store(true, Ordering::SeqCst);
        let handle = std::thread::spawn(move || {
            ironarm_std::run_tui(logger_path, Some(1024 * 1024 * 10));
            COPPER_RUNNING.store(false, Ordering::SeqCst);
        });
        *COPPER_HANDLE.lock().unwrap() = Some(handle);
    });
}

#[pyo3::pyfunction]
fn start_copper() {
    ensure_copper();
}

#[pyo3::pyfunction]
fn is_copper_alive() -> bool {
    COPPER_RUNNING.load(Ordering::SeqCst)
}

#[pyo3::pyfunction]
fn join_copper(timeout_secs: f64) -> bool {
    let mut guard = COPPER_HANDLE.lock().unwrap();
    if let Some(handle) = guard.take() {
        if timeout_secs <= 0.0 {
            let _ = handle.join();
            return true;
        }
        let deadline = std::time::Instant::now() + Duration::from_secs_f64(timeout_secs);
        while std::time::Instant::now() < deadline {
            if handle.is_finished() {
                let _ = handle.join();
                return true;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
        *guard = Some(handle);
        return false;
    }
    true
}

#[pyo3::pyfunction]
fn poll_state() -> Option<(f32, f32, f32, f32, f32, f32, f32, f32, f32)> {
    ironarm_std::ringbuf::read().map(|s| (s.j0, s.j1, s.j2, s.j3, s.j4, s.j5, s.wx, s.wy, s.wz))
}

#[pyo3::pymodule]
fn ironarm_sim(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(pyo3::wrap_pyfunction!(start_copper, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(is_copper_alive, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(join_copper, m)?)?;
    m.add_function(pyo3::wrap_pyfunction!(poll_state, m)?)?;
    Ok(())
}
