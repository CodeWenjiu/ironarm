use crate::math::interpolate;
use crate::messages::{JointCommand, JointWaypoint};
use cu29::prelude::*;

/// Receives joint-angle waypoints, outputs smooth per-joint commands.
#[derive(Reflect)]
pub struct JointInterpolator {
    joint_index: usize,
    current_angle: f32,
    target_angle: f32,
    smoothing: f32,
}

impl Freezable for JointInterpolator {}

impl CuTask for JointInterpolator {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(JointWaypoint);
    type Output<'m> = output_msg!(JointCommand);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let cfg = config.ok_or_else(|| CuError::from("JointInterpolator requires config"))?;
        let joint_index = cfg.get::<u64>("joint_index").ok().flatten().unwrap_or(0) as usize;
        let smoothing = cfg.get::<f32>("smoothing").ok().flatten().unwrap_or(0.3);
        Ok(Self {
            joint_index,
            current_angle: 0.0,
            target_angle: 0.0,
            smoothing,
        })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        if let Some(wp) = input.payload() {
            if let Some(&angle) = wp.angles.first() {
                self.target_angle = angle;
            }
        }

        self.current_angle = interpolate(self.current_angle, self.target_angle, self.smoothing);
        output.set_payload(JointCommand {
            target_angle: self.current_angle,
            target_velocity: 0.0,
            stiffness: 1.0,
        });

        output.metadata.set_status(format!(
            "j{}: {:.3} rad",
            self.joint_index, self.current_angle
        ));

        Ok(())
    }
}
