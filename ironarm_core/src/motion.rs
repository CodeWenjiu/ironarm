//! Trajectory-planning layer — outputs Cartesian waypoints.
//!
//! `ArmGeometry` is shared with the IK module.  For trajectory types,
//! see `crate::trajectory::Trajectory`.

/// Geometry parameters shared with the IK module.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "std", derive(cu29_traits::Reflect))]
pub struct ArmGeometry {
    pub l0: f32,
    pub l1: f32,
    pub base_z: f32,
}
