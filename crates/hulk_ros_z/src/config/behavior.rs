use serde::{Deserialize, Serialize};

use crate::msgs::DemoMode;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BehaviorConfig {
    pub mode: ModeConfig,
    pub buttons: ButtonConfig,
    pub walk: WalkConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ModeConfig {
    pub default: DemoMode,
    pub allow_button_override: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ButtonConfig {
    pub single_click_mode: DemoMode,
    pub double_click_mode: DemoMode,
    pub long_press_mode: DemoMode,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WalkConfig {
    pub forward: f32,
    pub lateral: f32,
    pub angular: f32,
}
