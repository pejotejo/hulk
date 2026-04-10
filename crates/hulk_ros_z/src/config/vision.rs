use ros_z_config::ConfigMetadata;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct VisionConfig {
    pub inputs: InputConfig,
    pub status: StatusConfig,
    pub debug: DebugConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct InputConfig {
    pub image_required: bool,
    pub camera_info_required: bool,
    pub max_frame_age_ms: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct StatusConfig {
    pub publish_hz: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct DebugConfig {
    pub log_frame_rate: bool,
}
