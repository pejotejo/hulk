use ros_z_config::{ConfigFieldMetadata, ConfigMetadata};
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

impl ConfigMetadata for BehaviorConfig {
    fn config_metadata() -> Vec<ConfigFieldMetadata> {
        vec![
            field(
                "mode.default",
                std::any::type_name::<DemoMode>(),
                "Default demo mode. Valid values: Idle, Stand, Walk.",
            ),
            field(
                "mode.allow_button_override",
                std::any::type_name::<bool>(),
                "Allow button events to override the current demo mode.",
            ),
            field(
                "buttons.single_click_mode",
                std::any::type_name::<DemoMode>(),
                "Mode activated on SingleClick. Valid values: Idle, Stand, Walk.",
            ),
            field(
                "buttons.double_click_mode",
                std::any::type_name::<DemoMode>(),
                "Mode activated on DoubleClick. Valid values: Idle, Stand, Walk.",
            ),
            field(
                "buttons.long_press_mode",
                std::any::type_name::<DemoMode>(),
                "Mode activated on LongPressStart. Valid values: Idle, Stand, Walk.",
            ),
            field(
                "walk.forward",
                std::any::type_name::<f32>(),
                "Forward walk command component.",
            ),
            field(
                "walk.lateral",
                std::any::type_name::<f32>(),
                "Lateral walk command component.",
            ),
            field(
                "walk.angular",
                std::any::type_name::<f32>(),
                "Angular walk command component.",
            ),
        ]
    }
}

fn field(path: &str, type_name: &str, description: &str) -> ConfigFieldMetadata {
    ConfigFieldMetadata {
        path: path.to_owned(),
        type_name: type_name.to_owned(),
        description: description.to_owned(),
        writable: true,
        min: None,
        max: None,
    }
}
