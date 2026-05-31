/// Integration test: trajectory → IK for 4-DOF arm.
use ironarm_core::ik::{EETarget, solve_ik};
use ironarm_core::motion::ArmGeometry4Dof;
use ironarm_core::trajectory;

#[test]
fn test_ik_circle_reachable() {
    let geo = ArmGeometry4Dof {
        l1: 1.2,
        l2: 1.0,
        l2_eff: 1.06,
        shoulder_z: 0.18,
    };

    let traj = trajectory::circle(0.0, 0.0, 1.5, geo.shoulder_z + 0.5, 20.0);

    for tick in [0, 25, 50, 75] {
        let t = tick as f32 * 0.02;
        let wp = traj.sample(t);
        let target = EETarget {
            x: wp.x,
            y: wp.y,
            z: wp.z,
        };
        let (j0, j1, j2, j3) = solve_ik(&target, &geo).expect("reachable");
        println!(
            "t={t:.2}: wp=({:.2},{:.2},{:.2}) j=({j0:.3},{j1:.3},{j2:.3},{j3:.3})",
            wp.x, wp.y, wp.z
        );
        assert!(j0.is_finite());
        assert!(j1.is_finite());
        assert!(j2.is_finite());
        assert!(j3.is_finite());
    }
}
