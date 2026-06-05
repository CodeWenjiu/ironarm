//! 子问题 1/3/4 — IK-Geo 几何构造块。

use crate::ik_geo::math::wrap_to_pi;
use glam::Vec3;

/// 子问题 1：求 θ 使 Rot(h, θ) * p = q。
/// 返回最多 2 个解。
pub fn sub1(p: &Vec3, q: &Vec3, h: &Vec3) -> [f32; 2] {
    let hp = h.dot(*p);
    let hq = h.dot(*q);
    let p_perp = *p - *h * hp;
    let q_perp = *q - *h * hq;
    let n = p_perp.length();
    if n < 1e-10 {
        return [0.0, f32::NAN];
    }
    let cos_theta = p_perp.dot(q_perp) / (n * n);
    let cos_theta = cos_theta.clamp(-1.0, 1.0);
    let theta = f32::acos(cos_theta);
    let cross_pq = p_perp.cross(q_perp);
    let sign = if h.dot(cross_pq) > 0.0 { 1.0 } else { -1.0 };
    [sign * theta, f32::NAN]
}

/// 子问题 3：求 θ 使 || Rot(h, θ) * p - q || = d。
/// 返回最多 2 个解（对应肘部向上/向下）。
pub fn sub3(p: &Vec3, q: &Vec3, h: &Vec3, d: f32) -> [f32; 2] {
    let hp = h.dot(*p);
    let hq = h.dot(*q);
    let p_perp = *p - *h * hp;
    let q_perp = *q - *h * hq;
    let np2 = p_perp.length_squared();
    let nq2 = q_perp.length_squared();
    if np2 < 1e-10 || nq2 < 1e-10 {
        return [0.0, f32::NAN];
    }
    let rhs = ((hp - hq) * (hp - hq) + np2 + nq2 - d * d) / (2.0 * f32::sqrt(np2 * nq2));
    let rhs = rhs.clamp(-1.0, 1.0);
    let phi = f32::acos(rhs);
    let theta0 = f32::atan2(h.dot(p_perp.cross(q_perp)), p_perp.dot(q_perp));
    [wrap_to_pi(theta0 + phi), wrap_to_pi(theta0 - phi)]
}

/// 子问题 4：求 θ 使 h1ᵀ * Rot(k, θ) * h2 = d。
/// 返回最多 2 个解。
pub fn sub4(h1: &Vec3, h2: &Vec3, k: &Vec3, d: f32) -> [f32; 2] {
    let k_cross_h2 = k.cross(*h2);
    let a = h1.dot(k_cross_h2);
    let b = h1.dot(*h2 - *k * k.dot(*h2));
    let c = d - k.dot(*h1) * k.dot(*h2);
    let mag = f32::sqrt(a * a + b * b);
    if mag < 1e-10 {
        return [0.0, f32::NAN];
    }
    let phi = f32::atan2(a, b);
    let cos_arg = (c / mag).clamp(-1.0, 1.0);
    let alpha = f32::acos(cos_arg);
    [wrap_to_pi(phi + alpha), wrap_to_pi(phi - alpha)]
}
