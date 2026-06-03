use bincode::{Decode, Encode};
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// 关节级消息
// ---------------------------------------------------------------------------

/// 任务 → 关节驱动器：单个关节的目标位姿。
#[derive(Debug, Clone, Encode, Decode, Serialize, Deserialize)]
#[cfg_attr(feature = "std", derive(cu29_traits::Reflect))]
pub struct JointCommand {
    pub target_angle: f32,
    pub target_velocity: f32,
    pub stiffness: f32,
}

/// 关节驱动器 → 监视器：当前关节状态。
#[derive(Debug, Clone, Encode, Decode, Serialize, Deserialize)]
#[cfg_attr(feature = "std", derive(cu29_traits::Reflect))]
pub struct JointState {
    pub current_angle: f32,
    pub current_velocity: f32,
}

// ---------------------------------------------------------------------------
// 流水线消息（运动规划 → IK → 插值 → 关节）
// ---------------------------------------------------------------------------

/// 运动规划器 → IK 求解器：笛卡尔空间中的目标点。
#[derive(Debug, Clone, Copy, PartialEq, Encode, Decode, Serialize, Deserialize)]
#[cfg_attr(feature = "std", derive(Default, cu29_traits::Reflect))]
pub struct CartesianWaypoint {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// IK 求解器 → 插值器 / 状态收集器。
///
/// 包含全部关节的目标角度以及原始的目标位置，
/// 下游各取所需：Interpolator 取 angles[i]，StateSink 取 target。
#[derive(Debug, Clone, Copy, Encode, Decode, Serialize, Deserialize)]
#[cfg_attr(feature = "std", derive(cu29_traits::Reflect))]
pub struct JointWaypoint {
    pub target: CartesianWaypoint,
    pub angles: [f32; ironarm_model::N_JOINTS],
}

impl Default for JointWaypoint {
    fn default() -> Self {
        Self {
            target: CartesianWaypoint {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            angles: [0.0; ironarm_model::N_JOINTS],
        }
    }
}

// ---------------------------------------------------------------------------
// 默认实现
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
