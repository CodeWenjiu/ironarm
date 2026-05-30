//! Convenience combinators — used by PyO3 bindings.

use crate::ik::{EETarget, solve_ik};
use crate::motion::{ArmGeometry, circle_waypoint};

/// Compute joint angles for the circular trajectory at time *t*.
pub fn compute_circle_angles(t: f32, l0: f32, l1: f32, base_z: f32) -> Option<(f32, f32)> {
    let geo = ArmGeometry { l0, l1, base_z };
    let wp = circle_waypoint(t, &geo, 20.0);
    let target = EETarget {
        x: wp.x,
        y: wp.y,
        z: wp.z,
    };
    solve_ik(&target, &geo)
}
