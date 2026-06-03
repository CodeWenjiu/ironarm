use crate::messages::{JointCommand, JointState};
use cu29::prelude::*;

/// 关节驱动器——接收角度指令，输出关节状态。
///
/// 当前为纯软件仿真模式，直接将指令作为当前状态透传。
/// 后续接真实硬件时在此处对接电机驱动协议。
#[derive(Reflect)]
pub struct JointDriver {
    pub joint_index: u64,
}

impl Freezable for JointDriver {}

impl CuTask for JointDriver {
    type Resources<'r> = ();
    /// 输入：插值器发来的关节指令（目标角度 + 速度）。
    type Input<'m> = input_msg!(JointCommand);
    /// 输出：当前关节状态（角度 + 速度）。
    type Output<'m> = output_msg!(JointState);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let cfg = config.ok_or_else(|| CuError::from("JointDriver 需要 config"))?;
        let joint_index = cfg.get::<u64>("joint_index").ok().flatten().unwrap_or(0);
        Ok(Self { joint_index })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let cmd = input
            .payload()
            .ok_or_else(|| CuError::from("JointDriver: 无 JointCommand"))?;

        // 仿真模式下直接透传；实机模式下此处发送电机指令
        output.set_payload(JointState {
            current_angle: cmd.target_angle,
            current_velocity: cmd.target_velocity,
        });

        Ok(())
    }
}
