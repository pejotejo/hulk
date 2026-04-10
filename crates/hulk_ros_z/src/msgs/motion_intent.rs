use std::str::FromStr;

use ros_z::MessageTypeInfo;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DemoMode {
    Idle,
    Stand,
    Walk,
}

impl DemoMode {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Idle => "Idle",
            Self::Stand => "Stand",
            Self::Walk => "Walk",
        }
    }
}

impl FromStr for DemoMode {
    type Err = String;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        match value {
            "Idle" | "idle" => Ok(Self::Idle),
            "Stand" | "stand" => Ok(Self::Stand),
            "Walk" | "walk" => Ok(Self::Walk),
            _ => Err(format!("unsupported demo mode: {value}")),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, MessageTypeInfo)]
#[ros_msg(type_name = "hulk_ros_z/msg/MotionIntent")]
pub struct MotionIntent {
    pub timestamp_ns: u64,
    pub mode: String,
    pub forward: f32,
    pub lateral: f32,
    pub angular: f32,
}

impl MotionIntent {
    pub fn idle(timestamp_ns: u64) -> Self {
        Self {
            timestamp_ns,
            mode: DemoMode::Idle.as_str().to_owned(),
            forward: 0.0,
            lateral: 0.0,
            angular: 0.0,
        }
    }
}

impl ros_z::msg::ZMessage for MotionIntent {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}
