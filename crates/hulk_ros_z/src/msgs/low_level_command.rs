use ros_z::ExtendedMessageTypeInfo;
use serde::{Deserialize, Serialize};

use crate::msgs::motion_intent::DemoMode;

#[derive(Debug, Clone, Serialize, Deserialize, ExtendedMessageTypeInfo)]
#[ros_msg(type_name = "hulk_ros_z/msg/LowLevelCommand")]
pub struct LowLevelCommand {
    pub timestamp_ns: u64,
    pub mode: DemoMode,
    pub forward: f32,
    pub lateral: f32,
    pub angular: f32,
}

impl ros_z::msg::ZMessage for LowLevelCommand {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}
