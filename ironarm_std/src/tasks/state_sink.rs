use crate::ringbuf::{self, ArmState};
use cu29::prelude::*;
use ironarm_core::messages::{CartesianWaypoint, JointState};

/// DAG sink — collects waypoint + 4 joint states and pushes to lock-free
/// ring buffer.  Python side polls the buffer via QTimer (no GIL, no mutex).
#[derive(Reflect)]
pub struct StateSink {
    last: [f32; 6],
    waypoint: CartesianWaypoint,
}

impl Freezable for StateSink {}

impl CuSinkTask for StateSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(
        'm, CartesianWaypoint, JointState, JointState, JointState, JointState, JointState, JointState
    );

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            last: [f32::INFINITY; 6],
            waypoint: CartesianWaypoint::default(),
        })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let (wp, j0, j1, j2, j3, j4, j5) = *input;

        if let Some(w) = wp.payload() {
            self.waypoint = w.clone();
        }

        let angles = [
            j0.payload().map(|s| s.current_angle).unwrap_or(0.0),
            j1.payload().map(|s| s.current_angle).unwrap_or(0.0),
            j2.payload().map(|s| s.current_angle).unwrap_or(0.0),
            j3.payload().map(|s| s.current_angle).unwrap_or(0.0),
            j4.payload().map(|s| s.current_angle).unwrap_or(0.0),
            j5.payload().map(|s| s.current_angle).unwrap_or(0.0),
        ];

        let changed = self
            .last
            .iter()
            .zip(&angles)
            .any(|(l, a)| (l - a).abs() >= 1e-6);
        if !changed {
            return Ok(());
        }
        self.last = angles;

        ringbuf::write(ArmState {
            j0: angles[0],
            j1: angles[1],
            j2: angles[2],
            j3: angles[3],
            j4: angles[4],
            j5: angles[5],
            wx: self.waypoint.x,
            wy: self.waypoint.y,
            wz: self.waypoint.z,
        });

        Ok(())
    }
}
