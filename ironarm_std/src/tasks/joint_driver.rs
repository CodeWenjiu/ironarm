use cu29::prelude::*;
use ironarm_core::messages::{JointCommand, JointState};

/// 通用关节驱动 task。
/// 每个关节实例通过 `copperconfig.ron` 中的 `config` 字典指定 `joint_index`、
/// 角度限位等参数。多个关节共享同一份代码，靠配置区分。
#[derive(Reflect)]
pub struct JointDriver {
    pub joint_index: u64,
    current_angle: f32,
    current_velocity: f32,
}

impl Freezable for JointDriver {}

impl CuTask for JointDriver {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(JointCommand);
    type Output<'m> = output_msg!(JointState);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let cfg = config.ok_or_else(|| CuError::from("JointDriver requires config"))?;
        let joint_index = cfg.get::<u64>("joint_index").ok().flatten().unwrap_or(0);
        Ok(Self {
            joint_index,
            current_angle: 0.0,
            current_velocity: 0.0,
        })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let cmd = input
            .payload()
            .ok_or_else(|| CuError::from("JointDriver: no JointCommand payload"))?;

        self.current_angle = cmd.target_angle;
        self.current_velocity = cmd.target_velocity;

        output.set_payload(JointState {
            current_angle: self.current_angle,
            current_velocity: self.current_velocity,
        });

        output
            .metadata
            .set_status(format!("angle={:.3} rad", self.current_angle));

        debug!(
            "Joint #{index}: angle={angle:.3} rad, velocity={vel:.3} rad/s",
            index = self.joint_index,
            angle = self.current_angle,
            vel = self.current_velocity,
        );

        Ok(())
    }
}
