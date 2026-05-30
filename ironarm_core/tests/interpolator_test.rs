/// Integration test: JointInterpolator logic (trajectory → IK → interpolation)
/// without the copper runtime — directly tests the functions that process() calls.
use ironarm_core::ik::{EETarget, solve_ik};
use ironarm_core::math::interpolate;
use ironarm_core::motion::{ArmGeometry, circle_waypoint};

#[test]
fn test_both_joints_produce_nonzero_angles() {
    let geo = ArmGeometry {
        l0: 1.0,
        l1: 2.0,
        base_z: 0.15,
    };

    // Simulate 100 ticks (2 seconds at 50 Hz).
    let mut j0_current = 0.0f32;
    let mut j1_current = 0.0f32;
    let smoothing = 0.3;

    for tick in 1..=100 {
        let t = tick as f32 * 0.02;
        let wp = circle_waypoint(t, &geo, 20.0);
        let target = EETarget {
            x: wp.x,
            y: wp.y,
            z: wp.z,
        };
        let (j0_target, j1_target) = solve_ik(&target, &geo).expect("target must be reachable");

        j0_current = interpolate(j0_current, j0_target, smoothing);
        j1_current = interpolate(j1_current, j1_target, smoothing);
    }

    println!("After 100 ticks: j0={j0_current:.4}, j1={j1_current:.4}");

    assert!(
        j0_current.abs() > 0.01,
        "j0 should have moved away from 0, got {j0_current}"
    );
    assert!(
        j1_current.abs() > 0.01,
        "j1 should have moved away from 0, got {j1_current}"
    );
}

#[test]
fn test_ik_at_various_points() {
    let geo = ArmGeometry {
        l0: 1.0,
        l1: 2.0,
        base_z: 0.15,
    };

    // Check several points on the trajectory.
    for tick in [0, 25, 50, 75] {
        let t = tick as f32 * 0.02;
        let wp = circle_waypoint(t, &geo, 20.0);
        let target = EETarget {
            x: wp.x,
            y: wp.y,
            z: wp.z,
        };
        let (j0, j1) = solve_ik(&target, &geo).expect("reachable");
        println!(
            "t={t:.2}: wp=({:.2},{:.2},{:.2}) j0={j0:.4} j1={j1:.4}",
            wp.x, wp.y, wp.z
        );
        assert!(j0.is_finite(), "j0 not finite");
        assert!(j1.is_finite(), "j1 not finite");
    }
}
