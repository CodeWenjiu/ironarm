//! Trajectory-planning layer — outputs Cartesian waypoints.

use crate::messages::CartesianWaypoint;

/// Geometry parameters shared with the IK module.
#[derive(Debug, Clone, cu29_traits::Reflect)]
pub struct ArmGeometry {
    pub l0: f32,
    pub l1: f32,
    pub base_z: f32,
}

/// Generate the next waypoint on a circular trajectory.
///
/// *period* is the time in seconds for one full circle (default 20 s).
pub fn circle_waypoint(t: f32, geo: &ArmGeometry, period: f32) -> CartesianWaypoint {
    use core::f32::consts::PI;
    let phase = t * 2.0 * PI / period;
    CartesianWaypoint {
        x: 1.2 * phase.cos(),
        y: 1.2 * phase.sin(),
        z: geo.base_z + 0.5,
    }
}
