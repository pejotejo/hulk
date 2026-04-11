use ros_z_config::{ConfigFieldMetadata, ConfigMetadata};
use serde::{Deserialize, Serialize};


#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BehaviorConfig {
    pub walk: WalkConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WalkConfig {
    pub forward: f32,
    pub angular_scale: f32,
    pub angular_max: f32,
}

impl ConfigMetadata for BehaviorConfig {
    fn config_metadata() -> Vec<ConfigFieldMetadata> {
        vec![
            field(
                "walk.forward",
                std::any::type_name::<f32>(),
                "Forward walk command component.",
            ),
            field(
                "walk.angular_scale",
                std::any::type_name::<f32>(),
                "Angular walk command scale.",
            ),
            field(
                "walk.angular_max",
                std::any::type_name::<f32>(),
                "Maximum angular walk command component.",
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
