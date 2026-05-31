use alloc::format;
use alloc::vec;
use crate::ik::{EETarget, solve_ik};
use crate::messages::{CartesianWaypoint, JointWaypoint};
use crate::motion::ArmGeometry;
use cu29::prelude::*;

/// Receives Cartesian waypoints, outputs joint-angle waypoints.
/// Status shows the angle for the configured joint.
#[derive(Reflect)]
pub struct IKSolver {
    joint_index: usize,
    geo: ArmGeometry,
}

impl Freezable for IKSolver {}

impl CuTask for IKSolver {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CartesianWaypoint);
    type Output<'m> = output_msg!(JointWaypoint);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let cfg = config.unwrap_or_else(|| panic!("IKSolver requires config"));
        let joint_index = cfg.get::<u64>("joint_index").ok().flatten().unwrap_or(0) as usize;
        let l0 = cfg.get::<f64>("l0").ok().flatten().unwrap_or(1.0) as f32;
        let l1 = cfg.get::<f64>("l1").ok().flatten().unwrap_or(2.0) as f32;
        let base_z = cfg.get::<f64>("base_z").ok().flatten().unwrap_or(0.15) as f32;
        Ok(Self {
            joint_index,
            geo: ArmGeometry { l0, l1, base_z },
        })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let wp = input
            .payload()
            .ok_or_else(|| CuError::from("IKSolver: no waypoint"))?;
        let target = EETarget {
            x: wp.x,
            y: wp.y,
            z: wp.z,
        };
        let angle = match solve_ik(&target, &self.geo) {
            Some((j0, j1)) => {
                if self.joint_index == 0 {
                    j0
                } else {
                    j1
                }
            }
            None => 0.0,
        };
        output.set_payload(JointWaypoint {
            angles: vec![angle],
        });

        output
            .metadata
            .set_status(format!("IK j{}: {:.3} rad", self.joint_index, angle));

        Ok(())
    }
}
