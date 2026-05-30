use cu29::prelude::*;
use crate::messages::{JointCommand, JointState};

/// Joint driver — receives angle commands, produces joint state.
/// No status output here; angle info is shown upstream at the interpolator.
#[derive(Reflect)]
pub struct JointDriver {
    pub joint_index: u64,
}

impl Freezable for JointDriver {}

impl CuTask for JointDriver {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(JointCommand);
    type Output<'m> = output_msg!(JointState);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let cfg = config.ok_or_else(|| CuError::from("JointDriver requires config"))?;
        let joint_index = cfg.get::<u64>("joint_index").ok().flatten().unwrap_or(0);
        Ok(Self { joint_index })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let cmd = input
            .payload()
            .ok_or_else(|| CuError::from("JointDriver: no JointCommand payload"))?;

        output.set_payload(JointState {
            current_angle: cmd.target_angle,
            current_velocity: cmd.target_velocity,
        });

        Ok(())
    }
}
