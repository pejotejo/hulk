use ros_z::MessageTypeInfo;
use serde::{Deserialize, Serialize};

pub const BUTTON_F1: i32 = 0;
pub const BUTTON_STAND: i32 = 1;
pub const BUTTON_WALKING: i32 = 2;

pub const BUTTON_EVENT_PRESS_DOWN: &str = "PressDown";
pub const BUTTON_EVENT_PRESS_UP: &str = "PressUp";
pub const BUTTON_EVENT_SINGLE_CLICK: &str = "SingleClick";
pub const BUTTON_EVENT_DOUBLE_CLICK: &str = "DoubleClick";
pub const BUTTON_EVENT_TRIPLE_CLICK: &str = "TripleClick";
pub const BUTTON_EVENT_LONG_PRESS_START: &str = "LongPressStart";
pub const BUTTON_EVENT_LONG_PRESS_HOLD: &str = "LongPressHold";
pub const BUTTON_EVENT_LONG_PRESS_END: &str = "LongPressEnd";

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq, MessageTypeInfo)]
#[ros_msg(type_name = "hulk_ros_z/msg/ButtonEvent")]
pub struct ButtonEvent {
    pub timestamp_ns: u64,
    pub button: i32,
    pub event_type: String,
}

impl ros_z::msg::ZMessage for ButtonEvent {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}
