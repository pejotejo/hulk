use ros_z_config::ConfigMetadata;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct SimDriverConfig {
    pub timing: TimingConfig,
    pub odometry: OdometryConfig,
    pub image: ImageConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct TimingConfig {
    pub publish_hz: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct OdometryConfig {
    pub pattern: String,
    pub step_x: f32,
    pub step_y: f32,
    pub step_theta: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct ImageConfig {
    pub enabled: bool,
    pub width: u32,
    pub height: u32,
    pub pattern: String,
}
