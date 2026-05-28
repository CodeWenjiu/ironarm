mod joint_driver;

pub use joint_driver::JointDriver;

pub mod monitor {
    pub type AppMonitor = cu_consolemon::CuConsoleMon;
}

use cu29::prelude::*;
use ironarm_core::messages::{JointCommand, JointState};

/// 临时：产生测试 JointCommand。作为 DAG 的 source（无上游连接），实现 CuSrcTask。
#[derive(Reflect)]
pub struct CmdSource {
    tick: u64,
}

impl Freezable for CmdSource {}

impl CuSrcTask for CmdSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(JointCommand);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { tick: 0 })
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        self.tick += 1;
        let angle = if (self.tick / 50) % 2 == 0 { 0.5 } else { -0.5 };
        output.set_payload(JointCommand {
            target_angle: angle,
            target_velocity: 0.0,
            stiffness: 1.0,
        });
        output.metadata.set_status(format!("cmd={angle:.3} rad"));
        debug!("CmdSource: tick={}, angle={}", self.tick, angle);
        Ok(())
    }
}

/// 临时：消费 JointState。作为 DAG 的 sink（无下游连接），实现 CuSinkTask。
#[derive(Reflect)]
pub struct StateSink;

impl Freezable for StateSink {}

impl CuSinkTask for StateSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, JointState, JointState);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let (j0, j1) = *input;
        let a0 = j0.payload().map(|s| s.current_angle);
        let a1 = j1.payload().map(|s| s.current_angle);
        debug!("StateSink[0]: angle={:.3} rad", a0.unwrap_or(0.0));
        debug!("StateSink[1]: angle={:.3} rad", a1.unwrap_or(0.0));
        Ok(())
    }
}
