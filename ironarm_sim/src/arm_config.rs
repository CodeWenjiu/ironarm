use bevy::asset::{AssetLoader, LoadContext, io::Reader};
use bevy::prelude::*;
use serde::Deserialize;

/// RON 文件 AssetLoader，使 Bevy 能加载和热重载 arm_config.ron。
#[derive(TypePath)]
pub struct ArmConfigLoader;

impl AssetLoader for ArmConfigLoader {
    type Asset = ArmConfig;
    type Settings = ();
    type Error = Box<dyn std::error::Error + Send + Sync + 'static>;

    async fn load(
        &self,
        reader: &mut dyn Reader,
        _settings: &Self::Settings,
        _load_context: &mut LoadContext<'_>,
    ) -> Result<Self::Asset, Self::Error> {
        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes).await?;
        let cfg: ArmConfig = ron::de::from_bytes(&bytes)?;
        Ok(cfg)
    }

    fn extensions(&self) -> &[&str] {
        &["ron"]
    }
}

#[derive(Asset, Deserialize, TypePath, Clone, PartialEq)]
pub struct ArmConfig {
    pub base: Segment,
    pub link0: Segment,
    pub link1: Segment,
    pub joint0: JointConfig,
    pub joint1: JointConfig,
    /// 动态连杆的线性阻尼（0=无阻尼，越大减速越快）
    pub linear_damping: f32,
    /// 动态连杆的角阻尼
    pub angular_damping: f32,
}

#[derive(Deserialize, Clone, PartialEq)]
pub struct Segment {
    pub size: (f32, f32, f32),
    pub center: (f32, f32, f32),
}

#[derive(Deserialize, Clone, PartialEq)]
pub struct JointConfig {
    pub axis: JointAxis,
    pub anchor1: (f32, f32, f32),
    pub anchor2: (f32, f32, f32),
    /// 电机弹簧频率（Hz，越大越硬，推荐 2~10）
    pub motor_frequency: f32,
    /// 电机阻尼比（0=振荡，1=临界阻尼不超调，>1=过阻尼缓慢）
    pub motor_damping_ratio: f32,
    /// 关节角度下限（rad）
    pub angle_limit_min: f32,
    /// 关节角度上限（rad）
    pub angle_limit_max: f32,
}

#[derive(Deserialize, Clone, PartialEq)]
pub enum JointAxis {
    X,
    Y,
    Z,
}

#[derive(Resource, Clone)]
pub struct ArmConfigHandle(pub Handle<ArmConfig>);
