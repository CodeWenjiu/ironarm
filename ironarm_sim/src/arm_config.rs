use serde::Deserialize;
use std::fs;

#[derive(Deserialize)]
pub struct ArmConfig {
    pub base: Segment,
    pub link0: Segment,
    pub link1: Segment,
    pub joint0: JointConfig,
    pub joint1: JointConfig,
}

#[derive(Deserialize)]
pub struct Segment {
    pub size: (f32, f32, f32),
    pub center: (f32, f32, f32),
}

#[derive(Deserialize)]
pub struct JointConfig {
    pub axis: JointAxis,
    pub anchor1: (f32, f32, f32),
    pub anchor2: (f32, f32, f32),
}

#[derive(Deserialize)]
pub enum JointAxis {
    X,
    Y,
    Z,
}

impl ArmConfig {
    pub fn load() -> Self {
        let text = fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/assets/arm_config.ron"
        ))
        .expect("Failed to read arm_config.ron");
        ron::from_str(&text).expect("Failed to parse arm_config.ron")
    }
}
