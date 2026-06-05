//! 正运动学（指数积公式）。

use super::types::{LinkOffsets, ScrewAxes};
use glam::{Mat3, Vec3};

/// 由关节角计算工具法兰位姿。
///
/// 返回值：(旋转矩阵, 平移向量)。
pub fn fk(h: &ScrewAxes, p: &LinkOffsets, q: &[f32; 6]) -> (Mat3, Vec3) {
    let mut r = Mat3::IDENTITY;
    let mut pos = p[0];
    for i in 0..6 {
        let ri = Mat3::from_axis_angle(h[i], q[i]);
        r = r * ri;
        pos += r * p[i + 1];
    }
    (r, pos)
}
