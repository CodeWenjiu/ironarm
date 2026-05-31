use crate::ik::{EETarget, solve_ik};
use crate::messages::{CartesianWaypoint, JointWaypoint};
use crate::motion::ArmGeometry4Dof;
use alloc::format;
use alloc::vec;
use cu29::prelude::*;

/// Receives Cartesian waypoints, outputs joint-angle waypoints for 4-DOF arm.
///
/// Adaptive: skips IK recomputation when waypoint hasn't changed.
#[derive(Reflect)]
pub struct IKSolver {
    joint_index: usize,
    geo: ArmGeometry4Dof,
    last_input: CartesianWaypoint,
    last_output: JointWaypoint,
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
        let l1 = cfg.get::<f64>("l1").ok().flatten().unwrap_or(0.8) as f32;
        let l2 = cfg.get::<f64>("l2").ok().flatten().unwrap_or(0.7) as f32;
        let l2_eff = cfg.get::<f64>("l2_eff").ok().flatten().unwrap_or(0.85) as f32;
        let shoulder_z = cfg.get::<f64>("shoulder_z").ok().flatten().unwrap_or(0.18) as f32;
        Ok(Self {
            joint_index,
            geo: ArmGeometry4Dof {
                l1,
                l2,
                l2_eff,
                shoulder_z,
            },
            last_input: CartesianWaypoint {
                x: f32::NAN,
                y: f32::NAN,
                z: f32::NAN,
            },
            last_output: JointWaypoint::default(),
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

        if *wp == self.last_input {
            output.set_payload(self.last_output.clone());
            return Ok(());
        }
        self.last_input = wp.clone();

        let target = EETarget {
            x: wp.x,
            y: wp.y,
            z: wp.z,
        };
        let angles = match solve_ik(&target, &self.geo) {
            Some((j0, j1, j2, j3)) => [j0, j1, j2, j3],
            None => [0.0; 4],
        };
        let raw = angles.get(self.joint_index).copied().unwrap_or(0.0);

        // Phase unwrap: ensure shortest angular path from previous output
        let prev = self.last_output.angles.first().copied().unwrap_or(raw);
        let mut angle = raw;
        while angle - prev > core::f32::consts::PI {
            angle -= 2.0 * core::f32::consts::PI;
        }
        while angle - prev < -core::f32::consts::PI {
            angle += 2.0 * core::f32::consts::PI;
        }

        self.last_output = JointWaypoint {
            angles: vec![angle],
        };
        output.set_payload(self.last_output.clone());

        output
            .metadata
            .set_status(format!("IK j{}: {:.3} rad", self.joint_index, angle));
        Ok(())
    }
}
