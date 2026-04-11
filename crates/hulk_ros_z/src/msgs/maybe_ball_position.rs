use coordinate_systems::Ground;
use ros_z::ExtendedMessageTypeInfo;
use serde::{Deserialize, Serialize};
use types::ball_position::BallPosition;

#[derive(Debug, Clone, Serialize, Deserialize, ExtendedMessageTypeInfo)]
#[ros_msg(type_name = "hulk_ros_z/msg/MaybeBallPosition")]
pub struct MaybeBallPosition {
    pub position: Option<BallPosition<Ground>>,
}

impl ros_z::msg::ZMessage for MaybeBallPosition {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}
