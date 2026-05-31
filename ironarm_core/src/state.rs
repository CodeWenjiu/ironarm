//! Shared state between copper tasks and external consumers.
//! Uses a callback — copper pushes data out.  `no_std` compatible.

use alloc::boxed::Box;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

// -- joint angles --
static J0: AtomicU32 = AtomicU32::new(0);
static J1: AtomicU32 = AtomicU32::new(0);

// -- Cartesian waypoint (from MotionPlanner) --
static WX: AtomicU32 = AtomicU32::new(0);
static WY: AtomicU32 = AtomicU32::new(0);
static WZ: AtomicU32 = AtomicU32::new(0);

type StateCallback = Box<dyn Fn(f32, f32, f32, f32, f32) + Send + Sync>;
static CALLBACK_SET: AtomicBool = AtomicBool::new(false);
static mut CALLBACK: Option<StateCallback> = None;

pub fn set_callback(cb: StateCallback) {
    unsafe {
        CALLBACK = Some(cb);
    }
    CALLBACK_SET.store(true, Ordering::Release);
}

pub fn set_waypoint(x: f32, y: f32, z: f32) {
    WX.store(x.to_bits(), Ordering::Relaxed);
    WY.store(y.to_bits(), Ordering::Relaxed);
    WZ.store(z.to_bits(), Ordering::Relaxed);
}

pub fn notify_joint_angles(j0: f32, j1: f32) {
    J0.store(j0.to_bits(), Ordering::Relaxed);
    J1.store(j1.to_bits(), Ordering::Relaxed);

    if CALLBACK_SET.load(Ordering::Acquire) {
        let wx = f32::from_bits(WX.load(Ordering::Relaxed));
        let wy = f32::from_bits(WY.load(Ordering::Relaxed));
        let wz = f32::from_bits(WZ.load(Ordering::Relaxed));
        let j0 = f32::from_bits(J0.load(Ordering::Relaxed));
        let j1 = f32::from_bits(J1.load(Ordering::Relaxed));
        unsafe {
            if let Some(ref cb) = CALLBACK {
                cb(j0, j1, wx, wy, wz);
            }
        }
    }
}
