use crate::messages::{JointState};
use cu29::prelude::*;

/// DAG sink — forwards joint states to the registered callback.
#[derive(Reflect)]
pub struct StateSink;

impl Freezable for StateSink {}

impl CuSinkTask for StateSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, JointState, JointState);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let (j0, j1) = *input;
        let a0 = j0.payload().map(|s| s.current_angle).unwrap_or(0.0);
        let a1 = j1.payload().map(|s| s.current_angle).unwrap_or(0.0);
        crate::state::notify_joint_angles(a0, a1);
        Ok(())
    }
}
