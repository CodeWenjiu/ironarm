//! Geometry parameters shared with the IK module.
//!
//! For 2-DOF arm, see `ArmGeometry`.  For 4-DOF, see `ArmGeometry4Dof`.

/// Geometry for the original 2-DOF arm.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "std", derive(cu29_traits::Reflect))]
pub struct ArmGeometry {
    pub l0: f32,
    pub l1: f32,
    pub base_z: f32,
}

/// Geometry for the 4-DOF articulated arm.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "std", derive(cu29_traits::Reflect))]
pub struct ArmGeometry4Dof {
    /// Upper arm length (shoulder → elbow).
    pub l1: f32,
    /// Forearm length (elbow → wrist).
    pub l2: f32,
    /// Effective second link for ee position (L2 + ee_offset).
    pub l2_eff: f32,
    /// Shoulder height above ground (base + waist + offset).
    pub shoulder_z: f32,
}
