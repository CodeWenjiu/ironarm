//! Analytical inverse kinematics for Pieper-type 6-DOF robots.
//!
//! Implements the "three parallel axes + two intersecting axes" solver
//! from the IK-Geo algorithm (https://arxiv.org/abs/2211.05737).
//!
//! This covers robots like UR5, UR10, and similar industrial manipulators
//! where joints 2,3,4 are parallel and joints 5,6 intersect.
//!
//! All math is hand-rolled — no external LA crate needed.
//! Compatible with `#![no_std]`.

use core::f32::consts::PI;

// ---------------------------------------------------------------------------
// Kinematics: Product-of-Exponentials representation
// ---------------------------------------------------------------------------

/// Joint screw axes (unit vectors in base frame at zero configuration).
pub type ScrewAxes = [[f32; 3]; 6];

/// Link offsets: p[i] = vector from joint i to joint i+1 (or to tool for p[6]).
/// p[0] = base to joint 1, p[1] = joint 1 to 2, ..., p[6] = joint 6 to tool.
pub type LinkOffsets = [[f32; 3]; 7];

// ---------------------------------------------------------------------------
// 3-D vector math
// ---------------------------------------------------------------------------

fn cross(a: &[f32; 3], b: &[f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot(a: &[f32; 3], b: &[f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm2(v: &[f32; 3]) -> f32 {
    dot(v, v)
}

fn norm(v: &[f32; 3]) -> f32 {
    f32::sqrt(norm2(v))
}

fn sub(a: &[f32; 3], b: &[f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn add(a: &[f32; 3], b: &[f32; 3]) -> [f32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale(v: &[f32; 3], s: f32) -> [f32; 3] {
    [v[0] * s, v[1] * s, v[2] * s]
}

/// Rodrigues' rotation formula: rotation matrix for axis h (unit) by angle θ.
/// Returns column-major 3×3 matrix.
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

fn mat_mul_vec(r: &[f32; 9], v: &[f32; 3]) -> [f32; 3] {
    [
        r[0] * v[0] + r[3] * v[1] + r[6] * v[2],
        r[1] * v[0] + r[4] * v[1] + r[7] * v[2],
        r[2] * v[0] + r[5] * v[1] + r[8] * v[2],
    ]
}

fn mat_transpose(r: &[f32; 9]) -> [f32; 9] {
    [r[0], r[3], r[6], r[1], r[4], r[7], r[2], r[5], r[8]]
}

fn mat_mul(a: &[f32; 9], b: &[f32; 9]) -> [f32; 9] {
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

fn wrap_to_pi(theta: f32) -> f32 {
    let tau = 2.0 * PI;
    (theta + PI).rem_euclid(tau) - PI
}

// ---------------------------------------------------------------------------
// Forward kinematics (Product of Exponentials)
// ---------------------------------------------------------------------------

/// Compute flange pose from joint angles.
///
/// Returns (rotation matrix column-major 3×3, translation vector).
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

// ---------------------------------------------------------------------------
// Subproblems (geometric building blocks)
// ---------------------------------------------------------------------------

/// Subproblem 1: find θ such that Rot(h, θ) * p = q.
/// Returns up to 2 solutions.
fn subproblem1(p: &[f32; 3], q: &[f32; 3], h: &[f32; 3]) -> [f32; 2] {
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
    // Determine sign
    let cross_pq = cross(&p_perp, &q_perp);
    let sign = if dot(h, &cross_pq) > 0.0 { 1.0 } else { -1.0 };
    [sign * theta, f32::NAN]
}

/// Subproblem 3: find θ such that || Rot(h, θ) * p - q || = d.
/// Returns up to 2 solutions.
fn subproblem3(p: &[f32; 3], q: &[f32; 3], h: &[f32; 3], d: f32) -> [f32; 2] {
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

/// Subproblem 4: find θ such that h1^T * Rot(k, θ) * h2 = d.
/// Returns up to 2 solutions.
fn subproblem4(h1: &[f32; 3], h2: &[f32; 3], k: &[f32; 3], d: f32) -> [f32; 2] {
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

// ---------------------------------------------------------------------------
// Analytical IK: three parallel axes + two intersecting
// ---------------------------------------------------------------------------

/// Solve IK for a robot with joints 2,3,4 parallel and joints 5,6 intersecting.
///
/// `r_target`: desired tool rotation (column-major 3×3), or identity for position-only.
/// `p_target`: desired tool position.
/// `h`: joint screw axes (unit vectors in base frame at q=0).
/// `p`: link offsets (p[0]=base→joint1, ..., p[6]=joint6→tool).
///
/// Returns up to 8 solutions.  Invalid entries contain `f32::NAN`.
pub fn solve_3p2i(
    r_target: &[f32; 9],
    p_target: &[f32; 3],
    h: &ScrewAxes,
    p: &LinkOffsets,
) -> [[f32; 6]; 8] {
    let mut sols = [[f32::NAN; 6]; 8];
    let mut idx = 0;

    // Sum of link offsets from joint 1 to joint 5
    let sum_p_2_5 = add(&add(&add(&p[1], &p[2]), &p[3]), &p[4]);
    let d1 = dot(&h[1], &sum_p_2_5);

    // p_16 = p_target - p[0] - R_target * p[6]
    let r_p6 = mat_mul_vec(r_target, &p[6]);
    let p_16 = sub(&sub(p_target, &p[0]), &r_p6);

    // Step 1: q1 (joint 1)
    let q1_sols = subproblem4(&h[1], &p_16, &scale(&h[0], -1.0), d1);

    for &q1 in &q1_sols {
        if !q1.is_finite() {
            continue;
        }

        let r_01 = rot(&h[0], q1);
        let r_01_t = mat_transpose(&r_01);

        // Step 2: q5 (joint 5)
        let r01t_r06 = mat_mul(&r_01_t, r_target);
        let d5 = dot(&h[1], &mat_mul_vec(&r01t_r06, &h[5]));
        let q5_sols = subproblem4(&h[1], &h[5], &h[4], d5);

        for &q5 in &q5_sols {
            if !q5.is_finite() {
                continue;
            }

            // Step 3: theta14 = q1 + q2 + q3 + q4 (combined rotation)
            let r_45 = rot(&h[4], q5);
            let r45_h5 = mat_mul_vec(&r_45, &h[5]);
            let r01t_r06_h5 = mat_mul_vec(&r01t_r06, &h[5]);
            let t14_sols = subproblem1(&r45_h5, &r01t_r06_h5, &h[1]);
            let theta14 = t14_sols[0]; // take first solution
            if !theta14.is_finite() {
                continue;
            }

            let r_14 = rot(&h[1], theta14);

            // Step 4: q3 (joint 3, elbow — law of cosines)
            let p_45_total = add(&p[4], &p[5]); // p_45 + p_5t
            let r14_p45 = mat_mul_vec(&r_14, &p_45_total);
            let d_inner = sub(&sub(&mat_mul_vec(&r_01_t, &p_16), &p[1]), &r14_p45);
            let d = norm(&d_inner);

            let neg_p34 = scale(&p[3], -1.0);
            let q3_sols = subproblem3(&neg_p34, &p[2], &h[1], d);

            for &q3 in &q3_sols {
                if !q3.is_finite() {
                    continue;
                }

                // Step 5: q2
                let r_h_q3_p34 = mat_mul_vec(&rot(&h[1], q3), &p[3]);
                let p23_plus = add(&p[2], &r_h_q3_p34);
                let q2_sols = subproblem1(&p23_plus, &d_inner, &h[1]);
                let q2 = q2_sols[0];
                if !q2.is_finite() {
                    continue;
                }

                // Step 6: q4
                let q4 = wrap_to_pi(theta14 - q2 - q3);

                // Step 7: q6
                let r_45_t = mat_transpose(&r_45);
                let r_14_t = mat_transpose(&r_14);
                let r45t_r14t_r01t_r06_h4 =
                    mat_mul_vec(&mat_mul(&mat_mul(&r_45_t, &r_14_t), &r01t_r06), &h[4]);
                let q6_sols = subproblem1(&h[4], &r45t_r14t_r01t_r06_h4, &h[5]);
                let q6 = q6_sols[0];
                if !q6.is_finite() {
                    continue;
                }

                if idx < 8 {
                    sols[idx] = [q1, q2, q3, q4, q5, q6];
                    idx += 1;
                }
            }
        }
    }

    sols
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// MuJoCo-extracted PoE parameters for UR5e (world frame at q=0).
    fn ur5_kinematics() -> (ScrewAxes, LinkOffsets) {
        let h: ScrewAxes = [
            [0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, -1.0, 0.0],
        ];
        let p: LinkOffsets = [
            [0.0, 0.0, 0.163],
            [0.0, -0.138, 0.0],
            [-0.425, 0.131, 0.0],
            [-0.392, 0.0, 0.0],
            [0.0, -0.127, 0.0],
            [0.0, 0.0, -0.100],
            [0.0, -0.100, 0.0],
        ];
        (h, p)
    }

    #[test]
    fn test_fk_q0() {
        let (h, p) = ur5_kinematics();
        let q = [0.0f32; 6];
        let (_r, pos) = fk(&h, &p, &q);
        assert!((pos[0] + 0.817).abs() < 0.01, "pos[0]={}", pos[0]);
        assert!((pos[2] - 0.063).abs() < 0.01, "pos[2]={}", pos[2]);
    }


    #[test]
    fn test_fk_ik_roundtrip() {
        let (h, p) = ur5_kinematics();
        let test_qs = [
            [0.0f32, 0.0, 0.0, 0.0, 0.0, 0.0],
        ];
        for q_in in &test_qs {
            let (r, pos) = fk(&h, &p, q_in);
            let sols = solve_3p2i(&r, &pos, &h, &p);
            let found = sols.iter().any(|s| {
                s.iter().all(|a| a.is_finite()) && {
                    let (_r_back, p_back) = fk(&h, &p, s);
                    norm(&sub(&p_back, &pos)) < 0.05
                }
            });
            assert!(found, "FK→IK roundtrip failed for q={q_in:?}");
        }
    }
}
