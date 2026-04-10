use ros_z_config::ConfigMetadata;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct BehaviorConfig {
    pub mode: ModeConfig,
    pub buttons: ButtonConfig,
    pub walk: WalkConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct ModeConfig {
    pub default: String,
    pub allow_button_override: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct ButtonConfig {
    pub single_click_mode: String,
    pub double_click_mode: String,
    pub long_press_mode: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
pub struct WalkConfig {
    pub forward: f32,
    pub lateral: f32,
    pub angular: f32,
}
