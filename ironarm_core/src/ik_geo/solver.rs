//! solve_3p2i — 解析逆运动学：三平行轴 + 两相交轴。
//!
//! 求解步骤：
//! 1. q1   — 子问题 4 解 shoulder_pan
//! 2. q5   — 子问题 4 解 wrist_2
//! 3. θ₁₄  — 子问题 1 解关节 2+3+4 合成旋转量
//! 4. q3   — 子问题 3（余弦定理）解 elbow
//! 5. q2   — 子问题 1 解 shoulder_lift
//! 6. q4   — 代数：q4 = θ₁₄ - q2 - q3
//! 7. q6   — 子问题 1 解 wrist_3

use crate::ik_geo::mat::{mat_mul, mat_mul_vec, mat_transpose, rot, wrap_to_pi};
use crate::ik_geo::subprobs::{sub1, sub3, sub4};
use crate::ik_geo::types::{LinkOffsets, ScrewAxes};
use crate::ik_geo::vec::{add, dot, norm, scale, sub};

/// 对关节 2,3,4 平行、关节 5,6 相交的机械臂求解逆运动学。
///
/// `r_target`: 目标旋转（列主序 3×3），仅位置 IK 时传单位阵。
/// `p_target`: 目标位置。
///
/// 返回最多 8 组解，无效项为 `f32::NAN`。
pub fn solve_3p2i(
    r_target: &[f32; 9],
    p_target: &[f32; 3],
    h: &ScrewAxes,
    p: &LinkOffsets,
) -> [[f32; 6]; 8] {
    let mut sols = [[f32::NAN; 6]; 8];
    let mut idx = 0;

    let sum_p_2_5 = add(&add(&add(&p[1], &p[2]), &p[3]), &p[4]);
    let d1 = dot(&h[1], &sum_p_2_5);

    let r_p6 = mat_mul_vec(r_target, &p[6]);
    let p_16 = sub(&sub(p_target, &p[0]), &r_p6);

    // ---------- 第 1 步：q1 ----------
    let q1_sols = sub4(&h[1], &p_16, &scale(&h[0], -1.0), d1);

    for &q1 in &q1_sols {
        if !q1.is_finite() {
            continue;
        }
        let r_01 = rot(&h[0], q1);
        let r_01_t = mat_transpose(&r_01);

        // ---------- 第 2 步：q5 ----------
        let r01t_r06 = mat_mul(&r_01_t, r_target);
        let d5 = dot(&h[1], &mat_mul_vec(&r01t_r06, &h[5]));
        let q5_sols = sub4(&h[1], &h[5], &h[4], d5);

        for &q5 in &q5_sols {
            if !q5.is_finite() {
                continue;
            }

            // ---------- 第 3 步：θ₁₄ ----------
            let r_45 = rot(&h[4], q5);
            let r45_h5 = mat_mul_vec(&r_45, &h[5]);
            let r01t_r06_h5 = mat_mul_vec(&r01t_r06, &h[5]);
            let t14_sols = sub1(&r45_h5, &r01t_r06_h5, &h[1]);
            let theta14 = t14_sols[0];
            if !theta14.is_finite() {
                continue;
            }
            let r_14 = rot(&h[1], theta14);

            // ---------- 第 4 步：q3 ----------
            let p_45_total = add(&p[4], &p[5]);
            let r14_p45 = mat_mul_vec(&r_14, &p_45_total);
            let d_inner = sub(&sub(&mat_mul_vec(&r_01_t, &p_16), &p[1]), &r14_p45);
            let d = norm(&d_inner);

            let neg_p34 = scale(&p[3], -1.0);
            let q3_sols = sub3(&neg_p34, &p[2], &h[1], d);

            for &q3 in &q3_sols {
                if !q3.is_finite() {
                    continue;
                }

                // ---------- 第 5 步：q2 ----------
                let r_h_q3_p34 = mat_mul_vec(&rot(&h[1], q3), &p[3]);
                let p23_plus = add(&p[2], &r_h_q3_p34);
                let q2_sols = sub1(&p23_plus, &d_inner, &h[1]);
                let q2 = q2_sols[0];
                if !q2.is_finite() {
                    continue;
                }

                // ---------- 第 6 步：q4 ----------
                let q4 = wrap_to_pi(theta14 - q2 - q3);

                // ---------- 第 7 步：q6 ----------
                let r_45_t = mat_transpose(&r_45);
                let r_14_t = mat_transpose(&r_14);
                let r45t_r14t_r01t_r06_h4 =
                    mat_mul_vec(&mat_mul(&mat_mul(&r_45_t, &r_14_t), &r01t_r06), &h[4]);
                let q6_sols = sub1(&h[4], &r45t_r14t_r01t_r06_h4, &h[5]);
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
// 单元测试
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ik_geo::fk::fk;
    use crate::ik_geo::vec::norm;
    use crate::ik_geo::vec::sub as vsub;

    fn ur5_kinematics() -> (ScrewAxes, LinkOffsets) {
        (ironarm_model::SCREW_AXES, ironarm_model::LINK_OFFSETS)
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
        let test_qs = [[0.0f32, 0.0, 0.0, 0.0, 0.0, 0.0]];
        for q_in in &test_qs {
            let (r, pos) = fk(&h, &p, q_in);
            let sols = solve_3p2i(&r, &pos, &h, &p);
            let found = sols.iter().any(|s| {
                s.iter().all(|a| a.is_finite()) && {
                    let (_r_back, p_back) = fk(&h, &p, s);
                    norm(&vsub(&p_back, &pos)) < 0.05
                }
            });
            assert!(found, "FK→IK 往返验证失败：q={q_in:?}");
        }
    }
}
