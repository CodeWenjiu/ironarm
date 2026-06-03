//! 正运动学（指数积公式）。

use super::types::{LinkOffsets, ScrewAxes};
use crate::ik_geo::mat::{mat_mul, mat_mul_vec, rot};
use crate::ik_geo::vec::add;

/// 由关节角计算工具法兰位姿。
///
/// 返回值：(旋转矩阵列主序 3×3, 平移向量)。
///
/// 算法：
/// ```text
///   R = I
///   pos = p[0]
///   for i in 0..6:
///       R = R * Rot(h[i], q[i])     // 关节旋转
///       pos = pos + R * p[i+1]      // 旋转后的连杆偏移
/// ```
pub fn fk(h: &ScrewAxes, p: &LinkOffsets, q: &[f32; 6]) -> ([f32; 9], [f32; 3]) {
    let mut r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    let mut pos = p[0];
    for i in 0..6 {
        let ri = rot(&h[i], q[i]);
        r = mat_mul(&r, &ri);
        pos = add(&pos, &mat_mul_vec(&r, &p[i + 1]));
    }
    (r, pos)
}
