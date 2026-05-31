use cu29::prelude::*;
use ironarm_core::messages::{JointCommand, JointWaypoint};

/// Receives joint-angle waypoints, outputs time-based smooth joint commands.
///
/// Uses linear interpolation over a configurable real-time duration.
/// Regardless of how slowly waypoints arrive (e.g. 10 Hz), the joint
/// always takes `transition_ms` to move between targets, producing a
/// genuinely smooth trajectory at the DAG's native tick rate.
///
/// Lives in `ironarm_std` because time-aware interpolation requires
/// `ctx.now()` — core remains `no_std`, time-free.
///
/// Config keys:
/// - `"joint_index"` (u64): which joint this instance drives
/// - `"transition_ms"` (f64): move duration in milliseconds (default 100)
#[derive(Reflect)]
pub struct JointInterpolator {
    joint_index: usize,

    /// Current interpolated angle (output every tick).
    current_angle: f32,
    /// Desired final angle (set by latest waypoint).
    target_angle: f32,

    /// Angle at the start of the current transition.
    start_angle: f32,
    /// Clock time when the current transition started.
    #[reflect(ignore)]
    transition_start: CuTime,
    /// Duration of one full transition (nanoseconds).
    #[reflect(ignore)]
    transition_dur: CuDuration,
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
        let transition_ms = cfg
            .get::<f64>("transition_ms")
            .ok()
            .flatten()
            .unwrap_or(100.0);
        Ok(Self {
            joint_index,
            current_angle: 0.0,
            target_angle: 0.0,
            start_angle: 0.0,
            transition_start: CuTime(0),
            transition_dur: CuDuration::from_millis(transition_ms as u64),
        })
    }

    fn process(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        // Detect a new target angle → start a new timed transition.
        if let Some(wp) = input.payload() {
            if let Some(&new_target) = wp.angles.first() {
                let changed = (self.target_angle - new_target).abs() > f32::EPSILON;
                if changed {
                    self.start_angle = self.current_angle;
                    self.target_angle = new_target;
                    self.transition_start = ctx.now();
                }
            }
        }

        // Time-based linear interpolation.
        let elapsed = ctx.now() - self.transition_start;
        let progress = if self.transition_dur.0 > 0 {
            let p = elapsed.0 as f64 / self.transition_dur.0 as f64;
            (p as f32).clamp(0.0, 1.0)
        } else {
            1.0
        };

        self.current_angle = self.start_angle + progress * (self.target_angle - self.start_angle);

        output.set_payload(JointCommand {
            target_angle: self.current_angle,
            target_velocity: 0.0,
            stiffness: 1.0,
        });

        Ok(())
    }
}
