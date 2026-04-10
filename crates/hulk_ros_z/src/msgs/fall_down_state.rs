use ros_z::MessageTypeInfo;
use serde::{Deserialize, Serialize};

pub const FALL_DOWN_IS_READY: &str = "IsReady";
pub const FALL_DOWN_IS_FALLING: &str = "IsFalling";
pub const FALL_DOWN_HAS_FALLEN: &str = "HasFallen";
pub const FALL_DOWN_IS_GETTING_UP: &str = "IsGettingUp";

#[derive(Debug, Clone, Serialize, Deserialize, MessageTypeInfo)]
#[ros_msg(type_name = "hulk_ros_z/msg/FallDownState")]
pub struct FallDownState {
    pub timestamp_ns: u64,
    pub fall_down_state: String,
    pub is_recovery_available: bool,
}

impl Default for FallDownState {
    fn default() -> Self {
        Self {
            timestamp_ns: 0,
            fall_down_state: FALL_DOWN_IS_READY.to_owned(),
            is_recovery_available: true,
        }
    }
}

impl ros_z::msg::ZMessage for FallDownState {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}
