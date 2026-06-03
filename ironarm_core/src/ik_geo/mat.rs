//! 3×3 矩阵运算与 Rodrigues 旋转公式。

use core::f32::consts::PI;

/// Rodrigues 旋转公式：绕单位轴 h 旋转 θ 角的旋转矩阵。
/// 返回列主序 3×3 矩阵。
pub fn rot(h: &[f32; 3], theta: f32) -> [f32; 9] {
    let c = f32::cos(theta);
    let s = f32::sin(theta);
    let v = 1.0 - c;
    let (x, y, z) = (h[0], h[1], h[2]);
    [
        c + x * x * v,
        y * x * v + z * s,
        z * x * v - y * s,
        x * y * v - z * s,
        c + y * y * v,
        z * y * v + x * s,
        x * z * v + y * s,
        y * z * v - x * s,
        c + z * z * v,
    ]
}

/// 3×3 矩阵 × 向量。
pub fn mat_mul_vec(r: &[f32; 9], v: &[f32; 3]) -> [f32; 3] {
    [
        r[0] * v[0] + r[3] * v[1] + r[6] * v[2],
        r[1] * v[0] + r[4] * v[1] + r[7] * v[2],
        r[2] * v[0] + r[5] * v[1] + r[8] * v[2],
    ]
}

/// 矩阵转置。
pub fn mat_transpose(r: &[f32; 9]) -> [f32; 9] {
    [r[0], r[3], r[6], r[1], r[4], r[7], r[2], r[5], r[8]]
}

/// 3×3 矩阵乘法：a * b。
pub fn mat_mul(a: &[f32; 9], b: &[f32; 9]) -> [f32; 9] {
    [
        a[0] * b[0] + a[3] * b[1] + a[6] * b[2],
        a[1] * b[0] + a[4] * b[1] + a[7] * b[2],
        a[2] * b[0] + a[5] * b[1] + a[8] * b[2],
        a[0] * b[3] + a[3] * b[4] + a[6] * b[5],
        a[1] * b[3] + a[4] * b[4] + a[7] * b[5],
        a[2] * b[3] + a[5] * b[4] + a[8] * b[5],
        a[0] * b[6] + a[3] * b[7] + a[6] * b[8],
        a[1] * b[6] + a[4] * b[7] + a[7] * b[8],
        a[2] * b[6] + a[5] * b[7] + a[8] * b[8],
    ]
}

/// 将角度归一化到 [-π, π] 范围。
pub fn wrap_to_pi(theta: f32) -> f32 {
    let tau = 2.0 * PI;
    (theta + PI).rem_euclid(tau) - PI
}
