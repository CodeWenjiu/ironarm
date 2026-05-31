use crate::messages::JointState;
use cu29::prelude::*;

/// DAG sink — forwards joint states to the registered callback.
///
/// Adaptive: only invokes the callback when joint angles have actually
/// changed (within 1e-6 rad tolerance).  This avoids flooding the Python
/// callback with duplicate data when the DAG runs unbounded.
#[derive(Reflect)]
pub struct StateSink {
    last_j0: f32,
    last_j1: f32,
}

impl Freezable for StateSink {}

impl CuSinkTask for StateSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, JointState, JointState);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            last_j0: f32::NAN,
            last_j1: f32::NAN,
        })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let (j0, j1) = *input;
        let a0 = j0.payload().map(|s| s.current_angle).unwrap_or(0.0);
        let a1 = j1.payload().map(|s| s.current_angle).unwrap_or(0.0);

        // Adaptive: skip callback if angles haven't changed meaningfully.
        if (self.last_j0 - a0).abs() < 1e-6 && (self.last_j1 - a1).abs() < 1e-6 {
            return Ok(());
        }
        self.last_j0 = a0;
        self.last_j1 = a1;

        crate::state::notify_joint_angles(a0, a1);
        Ok(())
    }
}
