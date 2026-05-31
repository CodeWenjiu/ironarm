//! Inverse kinematics for a 4-DOF articulated arm (MuJoCo Z-up coordinates).
//!
//! Joint layout:
//!   j0 — waist yaw around Z
//!   j1 — shoulder pitch around Y
//!   j2 — elbow pitch around Y
//!   j3 — wrist pitch around Y
//!
//! All three pitch joints (j1, j2, j3) form a 3-link planar arm in the
//! vertical plane.  The solver:
//!   1. Places the wrist with j1/j2 using standard 2-link IK
//!   2. Uses j3 to reach from wrist to the target (final L3 link)

use crate::motion::ArmGeometry4Dof;

pub struct EETarget {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

pub fn solve_ik(target: &EETarget, geo: &ArmGeometry4Dof) -> Option<(f32, f32, f32, f32)> {
    let r = f32::hypot(target.x, target.y);
    let h = target.z - geo.shoulder_z;
    let d = f32::hypot(r, h);

    let l1 = geo.l1;
    let l2 = geo.l2;
    let l3 = geo.l2_eff - geo.l2;

    let max2 = l1 + l2;
    let min2 = (l1 - l2).abs();

    if d > l1 + l2 + l3 + 1e-5 || d < (l1 - l2 - l3).abs() - 1e-5 {
        return None;
    }

    // Place wrist along shoulder→target line, L3 back from target
    let dw = if d > l3 {
        (d - l3).clamp(min2, max2)
    } else {
        min2
    };
    let wr = if d > 1e-6 { r * dw / d } else { 0.0 };
    let wh = if d > 1e-6 { h * dw / d } else { 0.0 };

    // 2-link IK for wrist position
    let dw_sq = wr * wr + wh * wh;
    let cos_a = (l1 * l1 + l2 * l2 - dw_sq) / (2.0 * l1 * l2);
    let cos_a = cos_a.clamp(-1.0, 1.0);
    let alpha = cos_a.acos();
    let j2 = core::f32::consts::PI - alpha;

    let a = l1 + l2 * j2.cos();
    let b = l2 * j2.sin();
    let j1 = (-wh).atan2(wr) - b.atan2(a);

    // j3: wrist→target angle relative to forearm
    let to_target = (-(h - wh)).atan2(r - wr);
    let j3 = to_target - (j1 + j2);

    let j0 = target.y.atan2(target.x);

    Some((j0, j1, j2, j3))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: f32, b: f32) -> bool {
        (a - b).abs() < 0.02
    }

    fn geo() -> ArmGeometry4Dof {
        ArmGeometry4Dof {
            l1: 0.8,
            l2: 0.7,
            l2_eff: 0.85,
            shoulder_z: 0.18,
        }
    }

    #[test]
    fn test_all_zero() {
        let t = EETarget {
            x: 1.65,
            y: 0.0,
            z: 0.18,
        };
        let (j0, j1, j2, j3) = solve_ik(&t, &geo()).unwrap();
        assert!(near(j0, 0.0));
        assert!(near(j1, 0.0));
        assert!(near(j2, 0.0));
        assert!(near(j3, 0.0));
    }

    #[test]
    fn test_j3_bends() {
        let t = EETarget {
            x: 1.0,
            y: 0.0,
            z: 0.68,
        };
        let (_, _, j2, j3) = solve_ik(&t, &geo()).unwrap();
        assert!(j2.abs() > 0.1);
        assert!(j3.abs() > 0.01);
    }
}
