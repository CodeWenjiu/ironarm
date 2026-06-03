//! 关节插值器——接收完整 IK 结果，对单个关节做时间平滑插值。
//!
//! 使用线性插值，在可配置的 `transition_ms` 时长内平滑过渡。
//! 时间源通过 `ironarm_core::clock::set_clock()` 在启动时注入。
//!
//! 配置键：
//! - `"joint_index"` (u64): 本实例驱动哪个关节
//! - `"transition_ms"` (f64): 过渡时长，毫秒（默认 100）

use crate::clock;
use crate::messages::{JointCommand, JointWaypoint};
use alloc::format;
use cu29::prelude::*;

#[derive(Reflect)]
pub struct JointInterpolator {
    joint_index: usize,

    /// 当前插值角度（每 tick 输出）。
    current_angle: f32,
    /// 目标角度（最新 IK 结果中对应索引的值）。
    target_angle: f32,

    /// 当前过渡的起始角度。
    start_angle: f32,
    /// 当前过渡开始的时刻（秒）。
    transition_start: f32,
    /// 一次完整过渡的时长（秒）。
    transition_dur: f32,
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
        let cfg = config.ok_or_else(|| CuError::from("JointInterpolator 需要 config"))?;
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
            transition_start: 0.0,
            transition_dur: transition_ms as f32 / 1000.0,
        })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        if let Some(wp) = input.payload() {
            let new_target = wp.angles[self.joint_index];
            let changed = (self.target_angle - new_target).abs() > f32::EPSILON;
            if changed {
                self.start_angle = self.current_angle;
                self.target_angle = new_target;
                self.transition_start = clock::now_secs();
            }
        }

        let elapsed = clock::now_secs() - self.transition_start;
        let progress = if self.transition_dur > 0.0 {
            (elapsed / self.transition_dur).clamp(0.0, 1.0)
        } else {
            1.0
        };

        self.current_angle = self.start_angle + progress * (self.target_angle - self.start_angle);

        output.set_payload(JointCommand {
            target_angle: self.current_angle,
            target_velocity: 0.0,
            stiffness: 1.0,
        });

        output.metadata.set_status(format!(
            "j{}: {:.3} rad ({:.0}%)",
            self.joint_index,
            self.current_angle,
            progress * 100.0
        ));

        Ok(())
    }
}
