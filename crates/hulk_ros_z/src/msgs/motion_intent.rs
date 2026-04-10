use ros_z::ExtendedMessageTypeInfo;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, ExtendedMessageTypeInfo)]
#[ros_msg(type_name = "hulk_ros_z/msg/DemoMode")]
pub enum DemoMode {
    Idle,
    Stand,
    Walk,
}

#[derive(Debug, Clone, Serialize, Deserialize, ExtendedMessageTypeInfo)]
#[ros_msg(type_name = "hulk_ros_z/msg/MotionIntent")]
pub struct MotionIntent {
    pub timestamp_ns: u64,
    pub mode: DemoMode,
    pub forward: f32,
    pub lateral: f32,
    pub angular: f32,
}

impl MotionIntent {
    pub fn idle(timestamp_ns: u64) -> Self {
        Self {
            timestamp_ns,
            mode: DemoMode::Idle,
            forward: 0.0,
            lateral: 0.0,
            angular: 0.0,
        }
    }
}

impl ros_z::msg::ZMessage for MotionIntent {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}
