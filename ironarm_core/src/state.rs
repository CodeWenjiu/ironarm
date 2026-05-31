//! Shared state between copper tasks and external consumers.
//! Uses a callback — copper pushes data out.  `no_std` compatible.

use alloc::boxed::Box;
use core::sync::atomic::{AtomicBool, Ordering};

type AngleCallback = Box<dyn Fn(f32, f32) + Send + Sync>;

static CALLBACK_SET: AtomicBool = AtomicBool::new(false);
static mut CALLBACK: Option<AngleCallback> = None;

/// Register a callback invoked each cycle with new joint angles.
/// Must be called before the copper runtime starts.
pub fn set_callback(cb: AngleCallback) {
    unsafe {
        CALLBACK = Some(cb);
    }
    CALLBACK_SET.store(true, Ordering::Release);
}

/// Called by the copper DAG sink when new angles are available.
pub fn notify_joint_angles(j0: f32, j1: f32) {
    if CALLBACK_SET.load(Ordering::Acquire) {
        unsafe {
            if let Some(ref cb) = CALLBACK {
                cb(j0, j1);
            }
        }
    }
}
