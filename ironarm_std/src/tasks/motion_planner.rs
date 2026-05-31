use crate::trajectory::{self, Trajectory};
use cu29::prelude::*;
use ironarm_core::messages::CartesianWaypoint;
use std::time::Instant;

/// Generates Cartesian waypoints using a configurable trajectory.
///
/// Config keys:
/// - `"type"`: `"circle"`, `"tilted_circle"`, or `"linear"`
/// - Circle: `"radius"`, `"height"`, `"period"`, `"shoulder_z"`
/// - TiltedCircle: `"cx"`,`"cy"`,`"cz"`,`"nx"`,`"ny"`,`"nz"`,`"r"`,`"period"`
/// - Linear: `"start_x"`..`"end_z"`, `"duration"`
/// - `"wp_rate_hz"`: waypoint rate (default 10)
#[derive(Reflect)]
pub struct MotionPlanner {
    start: Instant,
    #[reflect(ignore)]
    traj: Trajectory,
    wp_interval: f32,
    last_wp_time: f32,
    last_wp: CartesianWaypoint,
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
        let traj_type = cfg
            .get::<String>("type")
            .ok()
            .flatten()
            .unwrap_or_else(|| "circle".into());
        let shoulder_z = cfg.get::<f64>("shoulder_z").ok().flatten().unwrap_or(0.18) as f32;

        let traj = match traj_type.as_str() {
            "tilted_circle" => {
                let cx = cfg.get::<f64>("cx").ok().flatten().unwrap_or(0.0) as f32;
                let cy = cfg.get::<f64>("cy").ok().flatten().unwrap_or(0.0) as f32;
                let cz = cfg.get::<f64>("cz").ok().flatten().unwrap_or(0.0) as f32;
                let nx = cfg.get::<f64>("nx").ok().flatten().unwrap_or(0.0) as f32;
                let ny = cfg.get::<f64>("ny").ok().flatten().unwrap_or(0.0) as f32;
                let nz = cfg.get::<f64>("nz").ok().flatten().unwrap_or(1.0) as f32;
                let r = cfg.get::<f64>("r").ok().flatten().unwrap_or(0.5) as f32;
                let period = cfg.get::<f64>("period").ok().flatten().unwrap_or(5.0) as f32;
                trajectory::tilted_circle(cx, cy, cz, nx, ny, nz, r, period)
            }
            "linear" => {
                let sx = cfg.get::<f64>("start_x").ok().flatten().unwrap_or(0.0) as f32;
                let sy = cfg.get::<f64>("start_y").ok().flatten().unwrap_or(0.0) as f32;
                let sz = cfg.get::<f64>("start_z").ok().flatten().unwrap_or(0.0) as f32;
                let ex = cfg.get::<f64>("end_x").ok().flatten().unwrap_or(0.0) as f32;
                let ey = cfg.get::<f64>("end_y").ok().flatten().unwrap_or(0.0) as f32;
                let ez = cfg.get::<f64>("end_z").ok().flatten().unwrap_or(0.0) as f32;
                let dur = cfg.get::<f64>("duration").ok().flatten().unwrap_or(5.0) as f32;
                trajectory::linear(
                    CartesianWaypoint {
                        x: sx,
                        y: sy,
                        z: sz,
                    },
                    CartesianWaypoint {
                        x: ex,
                        y: ey,
                        z: ez,
                    },
                    dur,
                )
            }
            _ => {
                let radius = cfg.get::<f64>("radius").ok().flatten().unwrap_or(1.0) as f32;
                let height = cfg.get::<f64>("height").ok().flatten().unwrap_or(0.5) as f32;
                let period = cfg.get::<f64>("period").ok().flatten().unwrap_or(5.0) as f32;
                trajectory::circle(0.0, 0.0, radius, shoulder_z + height, period)
            }
        };

        let wp_rate_hz = cfg.get::<f64>("wp_rate_hz").ok().flatten().unwrap_or(10.0) as f32;
        let wp_interval = if wp_rate_hz > 0.0 {
            1.0 / wp_rate_hz
        } else {
            0.0
        };
        let last_wp = traj.sample(0.0);

        Ok(Self {
            start: Instant::now(),
            traj,
            wp_interval,
            last_wp_time: -wp_interval,
            last_wp,
        })
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let t = self.start.elapsed().as_secs_f32();

        if t - self.last_wp_time >= self.wp_interval {
            self.last_wp = self.traj.sample(t);
            self.last_wp_time = t;
        }

        let wp = self.last_wp.clone();
        output.set_payload(wp.clone());

        output.metadata.set_status(format!(
            "WP@{}Hz: ({:.2},{:.2},{:.2}) t={t:.1}s",
            (1.0 / self.wp_interval) as u32,
            wp.x,
            wp.y,
            wp.z
        ));

        Ok(())
    }
}
