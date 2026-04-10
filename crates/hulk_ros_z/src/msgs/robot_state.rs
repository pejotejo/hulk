use ros_z::MessageTypeInfo;
use serde::{Deserialize, Serialize};

use crate::msgs::{
    button_event::ButtonEvent, fall_down_state::FallDownState, odometry_state::OdometryState,
};

#[derive(Debug, Clone, Serialize, Deserialize, MessageTypeInfo)]
#[ros_msg(type_name = "hulk_ros_z/msg/RobotState")]
pub struct RobotState {
    pub timestamp_ns: u64,
    pub odometry: OdometryState,
    pub fall_down_state: FallDownState,
    pub has_button_event: bool,
    pub last_button_event: ButtonEvent,
}

impl RobotState {
    pub fn is_upright(&self) -> bool {
        self.fall_down_state.fall_down_state == crate::msgs::fall_down_state::FALL_DOWN_IS_READY
    }
}

impl ros_z::msg::ZMessage for RobotState {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}
