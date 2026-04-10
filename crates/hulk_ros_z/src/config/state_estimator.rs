use ros_z_config::ConfigMetadata;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct StateEstimatorConfig {
    pub timing: TimingConfig,
    pub inputs: InputConfig,
    pub smoothing: SmoothingConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct TimingConfig {
    pub publish_hz: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct InputConfig {
    pub odometry_max_age_ms: u64,
    pub fall_down_max_age_ms: u64,
    pub button_event_max_age_ms: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct SmoothingConfig {
    pub odometry_alpha: f32,
}
