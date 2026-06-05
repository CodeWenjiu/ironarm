use core::f32::consts::PI;
use cu29::prelude::*;
use glam::{Mat3, Vec3};

use crate::collision::CollisionConfig;
use crate::ik_geo::{self, LinkOffsets, ScrewAxes};
use crate::messages::{CartesianWaypoint, JointWaypoint};

// ---------------------------------------------------------------------------
// IK 缓存：避免重复计算 + 相位解绕
// ---------------------------------------------------------------------------

impl Default for IkCache {
    fn default() -> Self {
        Self::new()
    }
}

struct IkCache {
    last_input: CartesianWaypoint,
    last_output: JointWaypoint,
}

impl IkCache {
    fn new() -> Self {
        Self {
            last_input: CartesianWaypoint {
                x: f32::NAN,
                y: f32::NAN,
                z: f32::NAN,
                rx: 0.0,
                ry: 0.0,
                rz: 0.0,
            },
            last_output: JointWaypoint::default(),
        }
    }

    /// 若输入路径点未变化，直接返回缓存的结果。
    /// 否则记录新输入，返回 None 要求调用方重新计算。
    fn get_or_none(&mut self, wp: &CartesianWaypoint) -> Option<&JointWaypoint> {
        if *wp == self.last_input {
            return Some(&self.last_output);
        }
        self.last_input = *wp;
        None
    }

    /// 存储新的 IK 结果，同时对全部关节做相位解绕。
    fn update(&mut self, wp: &CartesianWaypoint, raw: &mut [f32; ironarm_model::N_JOINTS]) {
        for i in 0..ironarm_model::N_JOINTS {
            let prev = self.last_output.angles[i];
            while raw[i] - prev > PI {
                raw[i] -= 2.0 * PI;
            }
            while raw[i] - prev < -PI {
                raw[i] += 2.0 * PI;
            }
        }
        self.last_output = JointWaypoint {
            target: *wp,
            angles: *raw,
        };
    }
}

/// 从 XYZ 欧拉角构建旋转矩阵（Rz*Ry*Rx）。
fn euler_xyz(rx: f32, ry: f32, rz: f32) -> Mat3 {
    Mat3::from_rotation_z(rz) * Mat3::from_rotation_y(ry) * Mat3::from_rotation_x(rx)
}

// ---------------------------------------------------------------------------
// IKSolver — Copper 任务
// ---------------------------------------------------------------------------

/// 接收笛卡尔路径点，输出全部关节的目标角度。
///
/// 只计算一次逆运动学，通过 Copper fan-out 将同一个结果广播给
/// 各 JointInterpolator 实例，各自按 joint_index 取自己的关节角。
///
/// 关节数量由 ironarm_model 在编译期从 XML 自动确定。
#[cfg_attr(feature = "std", derive(cu29_traits::Reflect))]
pub struct IKSolver {
    #[cfg_attr(feature = "std", reflect(ignore))]
    h: ScrewAxes,
    #[cfg_attr(feature = "std", reflect(ignore))]
    p: LinkOffsets,
    #[cfg_attr(feature = "std", reflect(ignore))]
    collision: CollisionConfig,
    #[cfg_attr(feature = "std", reflect(ignore))]
    cache: IkCache,
}

impl Freezable for IKSolver {}

impl CuTask for IKSolver {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CartesianWaypoint);
    type Output<'m> = output_msg!(JointWaypoint);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            h: ironarm_model::SCREW_AXES,
            p: ironarm_model::LINK_OFFSETS,
            collision: CollisionConfig::from_config(config),
            cache: IkCache::new(),
        })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let wp = input
            .payload()
            .ok_or_else(|| CuError::from("IKSolver: 无路径点"))?;

        // 命中缓存则直接返回
        if let Some(cached) = self.cache.get_or_none(wp) {
            output.set_payload(*cached);
            return Ok(());
        }

        let r_target = euler_xyz(wp.rx, wp.ry, wp.rz);
        let p_target = Vec3::new(wp.x, wp.y, wp.z);
        let sols = ik_geo::solve_3p2i(&r_target, &p_target, &self.h, &self.p);

        // 碰撞过滤：选无碰撞且离障碍物最远的解
        let mut raw = self.collision.pick(&self.h, &self.p, &sols);

        self.cache.update(wp, &mut raw);
        output.set_payload(self.cache.last_output.clone());
        Ok(())
    }
}
