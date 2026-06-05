//! solve_3p2i — 解析逆运动学：三平行轴 + 两相交轴。

use crate::ik_geo::math::wrap_to_pi;
use crate::ik_geo::subprobs::{sub1, sub3, sub4};
use crate::ik_geo::types::{LinkOffsets, ScrewAxes};
use glam::{Mat3, Vec3};

/// 对关节 2,3,4 平行、关节 5,6 相交的机械臂求解逆运动学。
///
/// `r_target`: 目标旋转矩阵。
/// `p_target`: 目标位置。
///
/// 返回最多 8 组解，无效项为 `f32::NAN`。
pub fn solve_3p2i(
    r_target: &Mat3,
    p_target: &Vec3,
    h: &ScrewAxes,
    p: &LinkOffsets,
) -> [[f32; 6]; 8] {
    let mut sols = [[f32::NAN; 6]; 8];
    let mut idx = 0;

    let sum_p_2_5 = p[1] + p[2] + p[3] + p[4];
    let d1 = h[1].dot(sum_p_2_5);

    let r_p6 = *r_target * p[6];
    let p_16 = *p_target - p[0] - r_p6;

    // ---------- 第 1 步：q1 ----------
    let q1_sols = sub4(&h[1], &p_16, &(h[0] * -1.0), d1);

    for &q1 in &q1_sols {
        if !q1.is_finite() {
            continue;
        }
        let r_01 = Mat3::from_axis_angle(h[0], q1);
        let r_01_t = r_01.transpose();

        // ---------- 第 2 步：q5 ----------
        let r01t_r06 = r_01_t * *r_target;
        let d5 = h[1].dot(r01t_r06 * h[5]);
        let q5_sols = sub4(&h[1], &h[5], &h[4], d5);

        for &q5 in &q5_sols {
            if !q5.is_finite() {
                continue;
            }

            // ---------- 第 3 步：θ₁₄ ----------
            let r_45 = Mat3::from_axis_angle(h[4], q5);
            let r45_h5 = r_45 * h[5];
            let r01t_r06_h5 = r01t_r06 * h[5];
            let t14_sols = sub1(&r45_h5, &r01t_r06_h5, &h[1]);
            let theta14 = t14_sols[0];
            if !theta14.is_finite() {
                continue;
            }
            let r_14 = Mat3::from_axis_angle(h[1], theta14);

            // ---------- 第 4 步：q3 ----------
            let p_45_total = p[4] + p[5];
            let r14_p45 = r_14 * p_45_total;
            let d_inner = (r_01_t * p_16) - p[1] - r14_p45;
            let d = d_inner.length();

            let neg_p34 = p[3] * -1.0;
            let q3_sols = sub3(&neg_p34, &p[2], &h[1], d);

            for &q3 in &q3_sols {
                if !q3.is_finite() {
                    continue;
                }

                // ---------- 第 5 步：q2 ----------
                let r_h_q3_p34 = Mat3::from_axis_angle(h[1], q3) * p[3];
                let p23_plus = p[2] + r_h_q3_p34;
                let q2_sols = sub1(&p23_plus, &d_inner, &h[1]);
                let q2 = q2_sols[0];
                if !q2.is_finite() {
                    continue;
                }

                // ---------- 第 6 步：q4 ----------
                let q4 = wrap_to_pi(theta14 - q2 - q3);

                // ---------- 第 7 步：q6 ----------
                let r_45_t = r_45.transpose();
                let r_14_t = r_14.transpose();
                let r45t_r14t_r01t_r06_h4 = (r_45_t * r_14_t * r01t_r06) * h[4];
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

    fn ur5_kinematics() -> (ScrewAxes, LinkOffsets) {
        (ironarm_model::SCREW_AXES, ironarm_model::LINK_OFFSETS)
    }

    #[test]
    fn test_fk_q0() {
        let (h, p) = ur5_kinematics();
        let q = [0.0f32; 6];
        let (_r, pos) = fk(&h, &p, &q);
        assert!((pos.x + 0.817).abs() < 0.01, "pos.x={}", pos.x);
        assert!((pos.z - 0.063).abs() < 0.01, "pos.z={}", pos.z);
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
                    (p_back - pos).length() < 0.05
                }
            });
            assert!(found, "FK→IK 往返验证失败：q={q_in:?}");
        }
    }
}
