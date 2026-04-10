use ros_z::MessageTypeInfo;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, MessageTypeInfo)]
#[ros_msg(type_name = "hulk_ros_z/msg/VisionStatus")]
pub struct VisionStatus {
    pub frame_count: u64,
    pub last_frame_timestamp_ns: u64,
    pub last_camera_info_timestamp_ns: u64,
    pub heartbeat_timestamp_ns: u64,
}

impl ros_z::msg::ZMessage for VisionStatus {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}
