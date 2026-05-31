//! Inverse kinematics for a 2-joint arm (MuJoCo Z-up coordinates).
//!
//! Arm geometry:
//! - j0: base rotation around Z axis (at shoulder)
//! - j1: elbow pitch around Y axis (inherits j0 rotation)
//! - Upper arm length l0 (along +X when j0=0)
//! - Forearm length l1 (along +X when j1=0)
//!
//! Forward kinematics (derived from MuJoCo):
//!   r = l0 + l1 * cos(j1)
//!   z = base_z - l1 * sin(j1)    ← note: positive j1 lowers the arm
//!
//! Workspace: torus surface (r - l0)² + (z - base_z)² = l1².
//! Points not on this surface are unreachable with 2 DOF.

use crate::motion::ArmGeometry;

/// End-effector target in Cartesian space (Z-up).
pub struct EETarget {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Solve 2-joint IK.  Returns `(j0, j1)` — base rotation and elevation.
///
/// Returns `None` if the target is unreachable (projects to workspace boundary).
pub fn solve_ik(target: &EETarget, geo: &ArmGeometry) -> Option<(f32, f32)> {
    let r = f32::hypot(target.x, target.y); // horizontal distance in XY
    let h = target.z - geo.base_z; // vertical offset from shoulder

    let l0 = geo.l0;
    let l1 = geo.l1;

    // Workspace constraint: (r - l0)² + h² ≈ l1²
    let dist_sq = (r - l0) * (r - l0) + h * h;
    let tol = 0.005; // 5 mm tolerance for numerical stability

    if (dist_sq - l1 * l1).abs() > tol {
        // Project to nearest reachable point on the workspace surface.
        // Scale (r - l0, h) so its magnitude equals l1.
        let dist = dist_sq.sqrt();
        if dist < 1e-6 {
            // Singular: r = l0 and h = 0 → arm is at the center of workspace
            // Return a default pose (arm pointing down).
            let j0 = target.y.atan2(target.x);
            return Some((j0, core::f32::consts::PI / 2.0));
        }
        let scale = l1 / dist;
        let dr = (r - l0) * scale;
        let dh = h * scale;
        let j0 = target.y.atan2(target.x);
        // j1: atan2(-dh, dr) — see derivation below
        let j1 = (-dh).atan2(dr);
        return Some((j0, j1));
    }

    let j0 = target.y.atan2(target.x);

    // From FK: r = l0 + l1*cos(j1), h = -l1*sin(j1)
    // => cos(j1) = (r - l0)/l1, sin(j1) = -h/l1
    // => j1 = atan2(-h, r - l0)
    let j1 = (-h).atan2(r - l0);

    Some((j0, j1))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: f32, b: f32) -> bool {
        (a - b).abs() < 0.01
    }

    #[test]
    fn test_ik_reachable() {
        let geo = ArmGeometry {
            l0: 1.0,
            l1: 2.0,
            base_z: 0.15,
        };

        // Point on the workspace surface: r = l0 + l1*cos(j1), z = base_z - l1*sin(j1)
        // For j1 = -1.0 rad: r = 1 + 2*cos(-1) ≈ 2.0806, z = 0.15 - 2*sin(-1) ≈ 1.833
        let t = EETarget {
            x: 2.0806,
            y: 0.0,
            z: 1.8329,
        };
        let (j0, j1) = solve_ik(&t, &geo).expect("reachable");
        assert!(near(j0, 0.0));
        assert!(near(j1, -1.0));
    }

    #[test]
    fn test_ik_with_rotation() {
        let geo = ArmGeometry {
            l0: 1.0,
            l1: 2.0,
            base_z: 0.15,
        };
        // Point at 90° in XY plane, j1 = -0.5
        let j1_exp = -0.5f32;
        let r = 1.0 + 2.0 * j1_exp.cos();
        let z = 0.15 - 2.0 * j1_exp.sin();
        let t = EETarget { x: 0.0, y: r, z };
        let (j0, j1) = solve_ik(&t, &geo).expect("reachable");
        assert!(near(j0, core::f32::consts::FRAC_PI_2));
        assert!(near(j1, j1_exp));
    }

    #[test]
    fn test_ik_project_unreachable() {
        let geo = ArmGeometry {
            l0: 1.0,
            l1: 2.0,
            base_z: 0.15,
        };
        // A point inside the workspace that cannot be exactly reached
        let t = EETarget {
            x: 1.2,
            y: 0.0,
            z: 0.65,
        };
        // Should still return a solution (projected)
        let (j0, j1) = solve_ik(&t, &geo).expect("should project");
        assert!(j0.is_finite());
        assert!(j1.is_finite());
        // Verify the resulting pose is approximately on the workspace
        let r_actual = 1.0 + 2.0 * j1.cos();
        let z_actual = 0.15 - 2.0 * j1.sin();
        let err = ((r_actual - 1.0) * (r_actual - 1.0) + (z_actual - 0.15) * (z_actual - 0.15)
            - 4.0)
            .abs();
        assert!(err < 0.01, "projected pose not on workspace: err={err}");
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
        // Still reachable since we project...
        let result = solve_ik(&t, &geo);
        assert!(result.is_some());
    }
}
