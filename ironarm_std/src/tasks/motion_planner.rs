use cu29::prelude::*;
use ironarm_core::messages::CartesianWaypoint;
use ironarm_core::motion::ArmGeometry;
use ironarm_core::trajectory;
use std::time::Instant;

/// Generates Cartesian waypoints using a configurable trajectory.
///
/// Config keys:
/// - `"type"`: `"circle"` (default) or `"linear"`
/// - Circle: `"radius"`, `"height"`, `"period"` (float, seconds)
/// - Linear: `"start_x"`..`"start_z"`, `"end_x"`..`"end_z"`, `"duration"` (float)
/// - `"wp_rate_hz"`: waypoint output rate in Hz (default 10).  The DAG may
///   tick faster; this task holds the last waypoint between updates so
///   downstream interpolators can run at full rate.
#[derive(Reflect)]
pub struct MotionPlanner {
    start: Instant,
    geo: ArmGeometry,

    // -- trajectory config ------------------------------------------------
    traj_circle: bool,
    // circle
    radius: f32,
    height: f32,
    period: f32,
    // linear
    sx: f32,
    sy: f32,
    sz: f32,
    ex: f32,
    ey: f32,
    ez: f32,
    dur: f32,

    // -- waypoint rate control --------------------------------------------
    /// Minimum interval between new waypoints (seconds).
    wp_interval: f32,
    /// Time of last waypoint emission.
    last_wp_time: f32,
    /// Cached last waypoint — emitted on every tick, only recomputed
    /// when the interval has elapsed.
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
        let l0 = cfg.get::<f64>("l0").ok().flatten().unwrap_or(1.0) as f32;
        let l1 = cfg.get::<f64>("l1").ok().flatten().unwrap_or(2.0) as f32;
        let base_z = cfg.get::<f64>("base_z").ok().flatten().unwrap_or(0.15) as f32;
        let geo = ArmGeometry { l0, l1, base_z };

        let traj_type = cfg
            .get::<String>("type")
            .ok()
            .flatten()
            .unwrap_or_else(|| "circle".into());
        let is_circle = traj_type != "linear";

        // circle defaults
        let radius = cfg.get::<f64>("radius").ok().flatten().unwrap_or(1.2) as f32;
        let height = cfg.get::<f64>("height").ok().flatten().unwrap_or(0.5) as f32;
        let period = cfg.get::<f64>("period").ok().flatten().unwrap_or(20.0) as f32;
        // linear defaults
        let sx = cfg.get::<f64>("start_x").ok().flatten().unwrap_or(0.0) as f32;
        let sy = cfg.get::<f64>("start_y").ok().flatten().unwrap_or(0.0) as f32;
        let sz = cfg.get::<f64>("start_z").ok().flatten().unwrap_or(0.0) as f32;
        let ex = cfg.get::<f64>("end_x").ok().flatten().unwrap_or(0.0) as f32;
        let ey = cfg.get::<f64>("end_y").ok().flatten().unwrap_or(0.0) as f32;
        let ez = cfg.get::<f64>("end_z").ok().flatten().unwrap_or(0.0) as f32;
        let dur = cfg.get::<f64>("duration").ok().flatten().unwrap_or(5.0) as f32;

        let wp_rate_hz = cfg.get::<f64>("wp_rate_hz").ok().flatten().unwrap_or(10.0) as f32;
        let wp_interval = if wp_rate_hz > 0.0 {
            1.0 / wp_rate_hz
        } else {
            0.0 // 0 = emit every tick (unbounded)
        };

        // Compute the initial waypoint so we have something to emit on the
        // very first tick (before elapsed time has progressed).
        let traj = build_traj(
            is_circle,
            radius,
            geo.base_z + height,
            period,
            sx,
            sy,
            sz,
            ex,
            ey,
            ez,
            dur,
        );
        let last_wp = traj.sample(0.0);

        Ok(Self {
            start: Instant::now(),
            geo,
            traj_circle: is_circle,
            radius,
            height,
            period,
            sx,
            sy,
            sz,
            ex,
            ey,
            ez,
            dur,
            wp_interval,
            last_wp_time: -wp_interval, // force first tick to emit
            last_wp,
        })
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let t = self.start.elapsed().as_secs_f32();

        // Only sample a new waypoint when the interval has elapsed.
        if t - self.last_wp_time >= self.wp_interval {
            let traj = build_traj(
                self.traj_circle,
                self.radius,
                self.geo.base_z + self.height,
                self.period,
                self.sx,
                self.sy,
                self.sz,
                self.ex,
                self.ey,
                self.ez,
                self.dur,
            );
            self.last_wp = traj.sample(t);
            self.last_wp_time = t;
        }

        let wp = self.last_wp.clone();
        output.set_payload(wp.clone());
        ironarm_core::state::set_waypoint(wp.x, wp.y, wp.z);

        output.metadata.set_status(format!(
            "WP@{}Hz: ({:.2}, {:.2}, {:.2}) t={t:.1}s",
            (1.0 / self.wp_interval) as u32,
            wp.x,
            wp.y,
            wp.z
        ));

        Ok(())
    }
}

// ---------------------------------------------------------------------------
// helper — avoid repeating trajectory construction
// ---------------------------------------------------------------------------

fn build_traj(
    is_circle: bool,
    radius: f32,
    z: f32,
    period: f32,
    sx: f32,
    sy: f32,
    sz: f32,
    ex: f32,
    ey: f32,
    ez: f32,
    dur: f32,
) -> ironarm_core::trajectory::Trajectory {
    if is_circle {
        trajectory::circle(0.0, 0.0, radius, z, period)
    } else {
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
}
