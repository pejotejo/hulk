use std::path::PathBuf;

use ros_z_config::ConfigMetadata;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct ObjectDetectionConfig {
    pub inputs: InputConfig,
    pub status: StatusConfig,
    pub debug: DebugConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct InputConfig {
    pub enabled: bool,
    pub rgb_neural_network_path: PathBuf,
    pub nv12_neural_network_path: PathBuf,
    pub maximum_intersection_over_union: f32,
    pub confidence_threshold: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct StatusConfig {}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct DebugConfig {}
