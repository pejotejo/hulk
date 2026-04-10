use ros_z::MessageTypeInfo;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, MessageTypeInfo)]
#[ros_msg(type_name = "hulk_ros_z/msg/LowLevelCommand")]
pub struct LowLevelCommand {
    pub timestamp_ns: u64,
    pub mode: String,
    pub forward: f32,
    pub lateral: f32,
    pub angular: f32,
}

impl ros_z::msg::ZMessage for LowLevelCommand {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}
