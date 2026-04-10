use ros_z_config::ConfigMetadata;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct RobotHwConfig {
    pub mode: String,
    pub output: OutputConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct OutputConfig {
    pub enabled: bool,
}
