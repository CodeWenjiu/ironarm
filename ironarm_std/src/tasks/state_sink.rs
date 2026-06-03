use crate::ringbuf::{self, ArmState};
use cu29::prelude::*;
use ironarm_core::messages::{JointState, JointWaypoint};

/// DAG 汇集节点 — 从 ik 拿目标位置，从 6 个驱动器拿关节状态，
/// 统一写入锁无关环形缓冲供 Python 侧读取。
#[derive(Reflect)]
pub struct StateSink {
    last: [f32; 6],
}

impl Freezable for StateSink {}

impl CuSinkTask for StateSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(
        'm, JointWaypoint, JointState, JointState, JointState, JointState, JointState, JointState
    );

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            last: [f32::INFINITY; 6],
        })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let (ik, j0, j1, j2, j3, j4, j5) = *input;

        let angles = [
            j0.payload().map(|s| s.current_angle).unwrap_or(0.0),
            j1.payload().map(|s| s.current_angle).unwrap_or(0.0),
            j2.payload().map(|s| s.current_angle).unwrap_or(0.0),
            j3.payload().map(|s| s.current_angle).unwrap_or(0.0),
            j4.payload().map(|s| s.current_angle).unwrap_or(0.0),
            j5.payload().map(|s| s.current_angle).unwrap_or(0.0),
        ];

        let changed = self
            .last
            .iter()
            .zip(&angles)
            .any(|(l, a)| (l - a).abs() >= 1e-6);
        if !changed {
            return Ok(());
        }
        self.last = angles;

        let wp = ik.payload();
        ringbuf::write(ArmState {
            j0: angles[0],
            j1: angles[1],
            j2: angles[2],
            j3: angles[3],
            j4: angles[4],
            j5: angles[5],
            wx: wp.map(|w| w.target.x).unwrap_or(0.0),
            wy: wp.map(|w| w.target.y).unwrap_or(0.0),
            wz: wp.map(|w| w.target.z).unwrap_or(0.0),
        });

        Ok(())
    }
}
