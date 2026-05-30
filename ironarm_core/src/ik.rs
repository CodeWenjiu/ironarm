//! Inverse kinematics for a 2-joint arm (MuJoCo Z-up coordinates).

use crate::motion::ArmGeometry;

/// End-effector target in Cartesian space (Z-up).
pub struct EETarget {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Solve 2-joint IK.  Returns (j0, j1) — base rotation and elevation.
pub fn solve_ik(target: &EETarget, geo: &ArmGeometry) -> Option<(f32, f32)> {
    let r = f32::hypot(target.x, target.y); // horizontal distance in XY
    let h = target.z - geo.base_z; // vertical offset

    let d_sq = r * r + h * h;
    let d = d_sq.sqrt();
    let l0 = geo.l0;
    let l1 = geo.l1;

    if d > l0 + l1 || d < (l0 - l1).abs() {
        return None;
    }

    let cos_elbow = (d_sq - l0 * l0 - l1 * l1) / (2.0 * l0 * l1);
    let elbow_angle = cos_elbow.clamp(-1.0, 1.0).acos();
    let target_elevation = h.atan2(r);
    let link1_offset = (l1 * elbow_angle.sin()).atan2(l0 + l1 * elbow_angle.cos());

    let j0 = target.y.atan2(target.x);
    let j1 = target_elevation - link1_offset;

    Some((j0, j1))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ik_reachable() {
        let geo = ArmGeometry {
            l0: 1.0,
            l1: 2.0,
            base_z: 0.15,
        };
        let t = EETarget {
            x: 1.5,
            y: 0.0,
            z: 1.0,
        };
        let (j0, _) = solve_ik(&t, &geo).expect("reachable");
        assert!(j0.abs() < 0.1);
    }

    #[test]
    fn test_ik_unreachable() {
        let geo = ArmGeometry {
            l0: 0.5,
            l1: 0.5,
            base_z: 0.15,
        };
        let t = EETarget {
            x: 10.0,
            y: 0.0,
            z: 10.0,
        };
        assert!(solve_ik(&t, &geo).is_none());
    }
}
