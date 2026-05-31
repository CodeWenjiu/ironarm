use cu29::prelude::*;
use ironarm_core::messages::{CartesianWaypoint, JointState};
use std::sync::{Mutex, OnceLock};

// ---------------------------------------------------------------------------
// Safe callback registry (std, no unsafe)
// ---------------------------------------------------------------------------

type ArmCallback = Box<dyn Fn(f32, f32, f32, f32, f32) + Send + Sync>;
static CALLBACK: OnceLock<Mutex<Option<ArmCallback>>> = OnceLock::new();

/// Register the external callback (called from PyO3).
pub fn set_callback(cb: ArmCallback) {
    CALLBACK
        .get_or_init(|| Mutex::new(None))
        .lock()
        .unwrap()
        .replace(cb);
}

/// Push data to the registered callback (no-op if none registered).
fn notify(j0: f32, j1: f32, wx: f32, wy: f32, wz: f32) {
    if let Some(mutex) = CALLBACK.get() {
        if let Some(ref cb) = *mutex.lock().unwrap() {
            cb(j0, j1, wx, wy, wz);
        }
    }
}

// ---------------------------------------------------------------------------
// StateSink — DAG endpoint, collects messages and forwards to callback
// ---------------------------------------------------------------------------

/// DAG sink — receives waypoint + joint states via copper messages and
/// forwards them to the external callback.
///
/// Adaptive: only invokes the callback when joint angles have actually
/// changed (within 1e-6 rad tolerance).  This avoids flooding the
/// callback with duplicate data when the DAG runs unbounded.
#[derive(Reflect)]
pub struct StateSink {
    last_j0: f32,
    last_j1: f32,
    /// Latest Cartesian waypoint (set by MotionPlanner, carried along).
    waypoint: CartesianWaypoint,
}

impl Freezable for StateSink {}

/// Input: (CartesianWaypoint from MotionPlanner, JointState from joint_0, JointState from joint_1).
impl CuSinkTask for StateSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, CartesianWaypoint, JointState, JointState);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            last_j0: f32::NAN,
            last_j1: f32::NAN,
            waypoint: CartesianWaypoint {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let (wp, j0, j1) = *input;

        // Update waypoint when a new one arrives.
        if let Some(w) = wp.payload() {
            self.waypoint = w.clone();
        }

        let a0 = j0.payload().map(|s| s.current_angle).unwrap_or(0.0);
        let a1 = j1.payload().map(|s| s.current_angle).unwrap_or(0.0);

        // Adaptive: skip callback if angles haven't changed meaningfully.
        if (self.last_j0 - a0).abs() < 1e-6 && (self.last_j1 - a1).abs() < 1e-6 {
            return Ok(());
        }
        self.last_j0 = a0;
        self.last_j1 = a1;

        notify(a0, a1, self.waypoint.x, self.waypoint.y, self.waypoint.z);
        Ok(())
    }
}
