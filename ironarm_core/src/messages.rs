use bincode::{Decode, Encode};
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Joint-level messages
// ---------------------------------------------------------------------------

/// Task → joint driver: target pose for a single joint.
#[derive(Debug, Clone, Encode, Decode, Serialize, Deserialize)]
#[cfg_attr(feature = "std", derive(cu29_traits::Reflect))]
pub struct JointCommand {
    pub target_angle: f32,
    pub target_velocity: f32,
    pub stiffness: f32,
}

/// Joint driver → monitor: current joint state.
#[derive(Debug, Clone, Encode, Decode, Serialize, Deserialize)]
#[cfg_attr(feature = "std", derive(cu29_traits::Reflect))]
pub struct JointState {
    pub current_angle: f32,
    pub current_velocity: f32,
}

// ---------------------------------------------------------------------------
// Pipeline messages (motion → IK → interpolation → joints)
// ---------------------------------------------------------------------------

/// Motion planner → IK solver: a target in Cartesian space.
#[derive(Debug, Clone, Encode, Decode, Serialize, Deserialize)]
#[cfg_attr(feature = "std", derive(Default, cu29_traits::Reflect))]
pub struct CartesianWaypoint {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// IK solver → interpolator: raw joint angles for all joints.
#[derive(Debug, Clone, Default, Encode, Decode, Serialize, Deserialize)]
#[cfg_attr(feature = "std", derive(cu29_traits::Reflect))]
pub struct JointWaypoint {
    pub angles: alloc::vec::Vec<f32>,
}

// ---------------------------------------------------------------------------
// Defaults (custom — stiffness defaults to 1.0)
// ---------------------------------------------------------------------------

impl Default for JointCommand {
    fn default() -> Self {
        Self {
            target_angle: 0.0,
            target_velocity: 0.0,
            stiffness: 1.0,
        }
    }
}

impl Default for JointState {
    fn default() -> Self {
        Self {
            current_angle: 0.0,
            current_velocity: 0.0,
        }
    }
}
