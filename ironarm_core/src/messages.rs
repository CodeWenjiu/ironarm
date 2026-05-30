use bincode::{Decode, Encode};
use cu29_traits::Reflect;

use serde::{Deserialize, Serialize};

/// 上层发给关节驱动器的目标指令。
#[derive(Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct JointCommand {
    /// 目标角度 (rad)
    pub target_angle: f32,
    /// 目标角速度 (rad/s)，预留
    pub target_velocity: f32,
    /// 力矩柔顺度 0.0~1.0，预留
    pub stiffness: f32,
}

/// 关节驱动器反馈的当前状态。
#[derive(Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct JointState {
    /// 当前角度 (rad)
    pub current_angle: f32,
    /// 当前角速度 (rad/s)
    pub current_velocity: f32,
}

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
