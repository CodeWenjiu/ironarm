mod joint_driver;

pub use joint_driver::JointDriver;

pub mod monitor {
    pub type AppMonitor = cu_consolemon::CuConsoleMon;
}

use cu29::prelude::*;
use ironarm_core::messages::{JointCommand, JointState};

/// 关节 0 指令源。sim 模式下由 callback 覆写。
#[derive(Reflect)]
pub struct Src0 {
    tick: u64,
}

impl Freezable for Src0 {}

impl CuSrcTask for Src0 {
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
        output.set_payload(JointCommand::default());
        Ok(())
    }
}

/// 关节 1 指令源。sim 模式下由 callback 覆写。
#[derive(Reflect)]
pub struct Src1 {
    tick: u64,
}

impl Freezable for Src1 {}

impl CuSrcTask for Src1 {
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
        output.set_payload(JointCommand::default());
        Ok(())
    }
}

/// 消费 JointState。作为 DAG 的 sink。
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
