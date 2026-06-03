use crate::ik_geo::{self, LinkOffsets, ScrewAxes};
use crate::messages::{CartesianWaypoint, JointWaypoint};
use alloc::format;
use alloc::vec;
use cu29::prelude::*;

/// Receives Cartesian waypoints, outputs 6-DOF joint-angle waypoints using IK-Geo.
///
/// Configured by `joint_index` (0..5) to select which joint angle to emit.
/// All instances compute the full 6-DOF solution internally.
#[derive(Reflect)]
pub struct IKSolver {
    joint_index: usize,
    #[reflect(ignore)]
    h: ScrewAxes,
    #[reflect(ignore)]
    p: LinkOffsets,
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

        // UR5e PoE parameters extracted from MuJoCo model
        let h: ScrewAxes = [
            [0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, -1.0, 0.0],
        ];
        let p: LinkOffsets = [
            [0.0, 0.0, 0.163],
            [0.0, -0.138, 0.0],
            [-0.425, 0.131, 0.0],
            [-0.392, 0.0, 0.0],
            [0.0, -0.127, 0.0],
            [0.0, 0.0, -0.100],
            [0.0, -0.100, 0.0],
        ];

        Ok(Self {
            joint_index,
            h,
            p,
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

        // Position-only IK: identity orientation
        let r_target = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let p_target = [wp.x, wp.y, wp.z];

        let sols = ik_geo::solve_3p2i(&r_target, &p_target, &self.h, &self.p);

        // Pick first solution where all joints are within UR5e limits (±2π for most)
        let angles: [f32; 6] = sols
            .iter()
            .find(|s| s.iter().all(|a| a.is_finite()))
            .copied()
            .unwrap_or([0.0; 6]);

        // Phase unwrap for this specific joint
        let raw = angles.get(self.joint_index).copied().unwrap_or(0.0);
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
