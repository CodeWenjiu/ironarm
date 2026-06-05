//! 数学工具——glam 未涵盖的小函数。

use core::f32::consts::PI;

/// 将角度归一化到 [-π, π] 范围。
pub fn wrap_to_pi(theta: f32) -> f32 {
    (theta + PI).rem_euclid(2.0 * PI) - PI
}
