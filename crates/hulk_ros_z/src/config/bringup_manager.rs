use ros_z_config::ConfigMetadata;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct BringupManagerConfig {
    pub shutdown_grace_ms: u64,
}
