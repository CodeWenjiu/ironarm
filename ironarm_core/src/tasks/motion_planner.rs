use crate::messages::CartesianWaypoint;
use crate::motion::{ArmGeometry, circle_waypoint};
use cu29::prelude::*;

/// Generates Cartesian waypoints on a circular trajectory.
/// Status shows the current waypoint coordinates.
#[derive(Reflect)]
pub struct MotionPlanner {
    t: f32,
    geo: ArmGeometry,
    period: f32,
    tick: u64,
}

impl Freezable for MotionPlanner {}

impl CuSrcTask for MotionPlanner {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(CartesianWaypoint);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let cfg = config.unwrap_or_else(|| panic!("MotionPlanner requires config"));
        let l0 = cfg.get::<f64>("l0").ok().flatten().unwrap_or(1.0) as f32;
        let l1 = cfg.get::<f64>("l1").ok().flatten().unwrap_or(2.0) as f32;
        let base_z = cfg.get::<f64>("base_z").ok().flatten().unwrap_or(0.15) as f32;
        let period = cfg.get::<f64>("period").ok().flatten().unwrap_or(20.0) as f32;
        Ok(Self {
            t: 0.0,
            geo: ArmGeometry { l0, l1, base_z },
            period,
            tick: 0,
        })
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        self.tick += 1;
        self.t = self.tick as f32 * 0.02;
        let wp = circle_waypoint(self.t, &self.geo, self.period);
        output.set_payload(wp.clone());

        output
            .metadata
            .set_status(format!("WP: ({:.2}, {:.2}, {:.2})", wp.x, wp.y, wp.z));

        Ok(())
    }
}
