//! 子问题 1/3/4 — IK-Geo 几何构造块。

use crate::ik_geo::mat::wrap_to_pi;
use crate::ik_geo::vec::{cross, dot, norm, norm2, scale, sub};

/// 子问题 1：求 θ 使 Rot(h, θ) * p = q。
/// 返回最多 2 个解。
pub fn sub1(p: &[f32; 3], q: &[f32; 3], h: &[f32; 3]) -> [f32; 2] {
    let hp = dot(h, p);
    let hq = dot(h, q);
    let p_perp = sub(p, &scale(h, hp));
    let q_perp = sub(q, &scale(h, hq));
    let n = norm(&p_perp);
    if n < 1e-10 {
        return [0.0, f32::NAN];
    }
    let cos_theta = dot(&p_perp, &q_perp) / (n * n);
    let cos_theta = cos_theta.clamp(-1.0, 1.0);
    let theta = f32::acos(cos_theta);
    let cross_pq = cross(&p_perp, &q_perp);
    let sign = if dot(h, &cross_pq) > 0.0 { 1.0 } else { -1.0 };
    [sign * theta, f32::NAN]
}

/// 子问题 3：求 θ 使 || Rot(h, θ) * p - q || = d。
/// 返回最多 2 个解（对应肘部向上/向下）。
pub fn sub3(p: &[f32; 3], q: &[f32; 3], h: &[f32; 3], d: f32) -> [f32; 2] {
    let hp = dot(h, p);
    let hq = dot(h, q);
    let p_perp = sub(p, &scale(h, hp));
    let q_perp = sub(q, &scale(h, hq));
    let np2 = norm2(&p_perp);
    let nq2 = norm2(&q_perp);
    if np2 < 1e-10 || nq2 < 1e-10 {
        return [0.0, f32::NAN];
    }
    let rhs = ((hp - hq) * (hp - hq) + np2 + nq2 - d * d) / (2.0 * f32::sqrt(np2 * nq2));
    let rhs = rhs.clamp(-1.0, 1.0);
    let phi = f32::acos(rhs);
    let theta0 = f32::atan2(dot(h, &cross(&p_perp, &q_perp)), dot(&p_perp, &q_perp));
    [wrap_to_pi(theta0 + phi), wrap_to_pi(theta0 - phi)]
}

/// 子问题 4：求 θ 使 h1ᵀ * Rot(k, θ) * h2 = d。
/// 返回最多 2 个解。
pub fn sub4(h1: &[f32; 3], h2: &[f32; 3], k: &[f32; 3], d: f32) -> [f32; 2] {
    let k_cross_h2 = cross(k, h2);
    let a = dot(h1, &k_cross_h2);
    let b = dot(h1, &sub(h2, &scale(k, dot(k, h2))));
    let c = d - dot(k, h1) * dot(k, h2);
    let mag = f32::sqrt(a * a + b * b);
    if mag < 1e-10 {
        return [0.0, f32::NAN];
    }
    let phi = f32::atan2(a, b);
    let cos_arg = (c / mag).clamp(-1.0, 1.0);
    let alpha = f32::acos(cos_arg);
    [wrap_to_pi(phi + alpha), wrap_to_pi(phi - alpha)]
}
