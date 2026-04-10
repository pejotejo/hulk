use ros_z::MessageTypeInfo;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize, MessageTypeInfo)]
#[ros_msg(type_name = "hulk_ros_z/msg/OdometryState")]
pub struct OdometryState {
    pub timestamp_ns: u64,
    pub x: f32,
    pub y: f32,
    pub theta: f32,
}

impl Default for OdometryState {
    fn default() -> Self {
        Self {
            timestamp_ns: 0,
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        }
    }
}

impl ros_z::msg::ZMessage for OdometryState {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}
