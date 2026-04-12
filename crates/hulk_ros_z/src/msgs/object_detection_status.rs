use std::time::Duration;

use ros_z::MessageTypeInfo;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, MessageTypeInfo)]
#[ros_msg(type_name = "hulk_ros_z/msg/ObjectDetectionStatus")]
pub struct ObjectDetectionStatus {
    pub last_inference_duration: Duration,
    pub last_post_processing_duration: Duration,
    pub last_non_maximum_suppression_duration: Duration,
}

impl ros_z::msg::ZMessage for ObjectDetectionStatus {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}
