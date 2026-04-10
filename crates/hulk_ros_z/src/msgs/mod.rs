use std::time::{SystemTime, UNIX_EPOCH};

use ros_z_msgs::{builtin_interfaces::Time, std_msgs::Header};

pub mod button_event;
pub mod fall_down_state;
pub mod low_level_command;
pub mod motion_intent;
pub mod odometry_state;
pub mod robot_state;
pub mod vision_status;

pub use button_event::{
    BUTTON_EVENT_DOUBLE_CLICK, BUTTON_EVENT_LONG_PRESS_END, BUTTON_EVENT_LONG_PRESS_HOLD,
    BUTTON_EVENT_LONG_PRESS_START, BUTTON_EVENT_PRESS_DOWN, BUTTON_EVENT_PRESS_UP,
    BUTTON_EVENT_SINGLE_CLICK, BUTTON_EVENT_TRIPLE_CLICK, BUTTON_F1, BUTTON_STAND, BUTTON_WALKING,
    ButtonEvent,
};
pub use fall_down_state::{
    FALL_DOWN_HAS_FALLEN, FALL_DOWN_IS_FALLING, FALL_DOWN_IS_GETTING_UP, FALL_DOWN_IS_READY,
    FallDownState,
};
pub use low_level_command::LowLevelCommand;
pub use motion_intent::{DemoMode, MotionIntent};
pub use odometry_state::OdometryState;
pub use robot_state::RobotState;
pub use vision_status::VisionStatus;

pub fn timestamp_now() -> u64 {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system time before unix epoch");
    now.as_nanos() as u64
}

pub fn ros_time_from_ns(timestamp_ns: u64) -> Time {
    Time {
        sec: (timestamp_ns / 1_000_000_000) as i32,
        nanosec: (timestamp_ns % 1_000_000_000) as u32,
    }
}

pub fn header(frame_id: &str, timestamp_ns: u64) -> Header {
    Header {
        stamp: ros_time_from_ns(timestamp_ns),
        frame_id: frame_id.to_owned(),
    }
}
