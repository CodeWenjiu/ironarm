use cu29::prelude::*;
use ironarm_core::messages::CartesianWaypoint;
use ironarm_core::motion::{ArmGeometry, circle_waypoint};
use std::time::Instant;

/// Generates Cartesian waypoints on a circular trajectory using real time.
/// Timing lives in std — core remains pure math.
#[derive(Reflect)]
pub struct MotionPlanner {
    start: Instant,
    geo: ArmGeometry,
    period: f32,
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
            start: Instant::now(),
            geo: ArmGeometry { l0, l1, base_z },
            period,
        })
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let t = self.start.elapsed().as_secs_f32();
        let wp = circle_waypoint(t, &self.geo, self.period);
        output.set_payload(wp.clone());

        output.metadata.set_status(format!(
            "WP: ({:.2}, {:.2}, {:.2}) t={t:.1}s",
            wp.x, wp.y, wp.z
        ));

        Ok(())
    }
}
