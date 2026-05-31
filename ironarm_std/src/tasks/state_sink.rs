use cu29::prelude::*;
use ironarm_core::messages::{CartesianWaypoint, JointState};
use std::sync::{Mutex, OnceLock};

// ---------------------------------------------------------------------------
// Safe callback registry (std, no unsafe)
// ---------------------------------------------------------------------------

type ArmCallback = Box<dyn Fn(f32, f32, f32, f32, f32, f32, f32) + Send + Sync>;
static CALLBACK: OnceLock<Mutex<Option<ArmCallback>>> = OnceLock::new();

pub fn set_callback(cb: ArmCallback) {
    CALLBACK
        .get_or_init(|| Mutex::new(None))
        .lock()
        .unwrap()
        .replace(cb);
}

fn notify(j0: f32, j1: f32, j2: f32, j3: f32, wx: f32, wy: f32, wz: f32) {
    if let Some(mutex) = CALLBACK.get() {
        if let Some(ref cb) = *mutex.lock().unwrap() {
            cb(j0, j1, j2, j3, wx, wy, wz);
        }
    }
}

// ---------------------------------------------------------------------------
// StateSink — DAG endpoint for 4-DOF arm
// ---------------------------------------------------------------------------

#[derive(Reflect)]
pub struct StateSink {
    last: [f32; 4],
    waypoint: CartesianWaypoint,
}

impl Freezable for StateSink {}

impl CuSinkTask for StateSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(
        'm,
        CartesianWaypoint,
        JointState,
        JointState,
        JointState,
        JointState
    );

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            last: [f32::INFINITY; 4],
            waypoint: CartesianWaypoint {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let (wp, j0, j1, j2, j3) = *input;

        if let Some(w) = wp.payload() {
            self.waypoint = w.clone();
        }

        let angles = [
            j0.payload().map(|s| s.current_angle).unwrap_or(0.0),
            j1.payload().map(|s| s.current_angle).unwrap_or(0.0),
            j2.payload().map(|s| s.current_angle).unwrap_or(0.0),
            j3.payload().map(|s| s.current_angle).unwrap_or(0.0),
        ];

        // Adaptive skip
        let changed = self
            .last
            .iter()
            .zip(&angles)
            .any(|(l, a)| (l - a).abs() >= 1e-6);
        if !changed {
            return Ok(());
        }
        self.last = angles;

        notify(
            angles[0],
            angles[1],
            angles[2],
            angles[3],
            self.waypoint.x,
            self.waypoint.y,
            self.waypoint.z,
        );
        Ok(())
    }
}
