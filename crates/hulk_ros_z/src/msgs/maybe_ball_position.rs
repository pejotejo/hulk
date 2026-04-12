use coordinate_systems::Ground;
use linear_algebra::{Point2, Vector2};
use ros_z::{ExtendedMessageTypeInfo, MessageTypeInfo, time::ZTime};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize, MessageTypeInfo)]
#[ros_msg(type_name = "hulk_ros_z/msg/ZBallPosition")]
pub struct ZBallPosition<Frame> {
    pub position: Point2<Frame>,
    pub velocity: Vector2<Frame>,
    pub last_seen: ZTime,
}

#[derive(Debug, Clone, Serialize, Deserialize, ExtendedMessageTypeInfo)]
#[ros_msg(type_name = "hulk_ros_z/msg/MaybeBallPosition")]
pub struct MaybeBallPosition {
    pub position: Option<ZBallPosition<Ground>>,
}

impl ros_z::msg::ZMessage for MaybeBallPosition {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}
