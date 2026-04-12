use ros_z_config::ConfigMetadata;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct BallFilterConfig {
    pub ball_radius: f32,
    // pub output: OutputConfig,
}

// #[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
// #[serde(deny_unknown_fields)]
// pub struct OutputConfig {
//     pub mode: String,
// }
