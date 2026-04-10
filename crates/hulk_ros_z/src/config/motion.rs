use ros_z_config::ConfigMetadata;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct MotionConfig {
    pub timing: TimingConfig,
    pub output: OutputConfig,
    pub limits: LimitsConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct TimingConfig {
    pub publish_hz: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct OutputConfig {
    pub mode: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct LimitsConfig {
    pub max_forward: f32,
    pub max_lateral: f32,
    pub max_angular: f32,
}
