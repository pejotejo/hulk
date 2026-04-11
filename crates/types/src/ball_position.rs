use std::{ops::Mul, sync::Arc, time::SystemTime};

use ros_z::{
    MessageTypeInfo, WithTypeInfo,
    dynamic::{FieldSchema, FieldType, MessageSchema},
    msg::{SerdeCdrSerdes, ZMessage},
};
use serde::{Deserialize, Serialize};

use coordinate_systems::{Field, Ground};
use linear_algebra::{Isometry2, Point2, Vector2};
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};

#[derive(
    Debug, Clone, Copy, PathDeserialize, PathSerialize, PathIntrospect, Serialize, Deserialize,
)]
pub struct BallPosition<Frame> {
    pub position: Point2<Frame>,
    pub velocity: Vector2<Frame>,
    pub last_seen: SystemTime,
}

impl<Frame> MessageTypeInfo for BallPosition<Frame> {
    fn type_name() -> &'static str {
        "hulk_ros_z/msg/BallPosition"
    }

    fn type_hash() -> ros_z::TypeHash {
        ros_z::TypeHash::zero()
    }

    fn message_schema() -> Option<Arc<MessageSchema>> {
        Some(Arc::new(MessageSchema {
            type_name: "hulk_ros_z/msg/BallPosition".to_owned(),
            package: "hulks".to_owned(),
            name: "BallPosition".to_owned(),
            fields: vec![
                FieldSchema {
                    name: "position".to_owned(),
                    field_type: FieldType::Message(Arc::new(MessageSchema {
                        type_name: "position".to_owned(),
                        package: "hulks".to_owned(),
                        name: "Point2".to_owned(),
                        fields: vec![
                            FieldSchema {
                                name: "x".to_owned(),
                                field_type: FieldType::Float32,
                                default_value: None,
                            },
                            FieldSchema {
                                name: "y".to_owned(),
                                field_type: FieldType::Float32,
                                default_value: None,
                            },
                        ],
                        type_hash: None,
                    })),
                    default_value: None,
                },
                FieldSchema {
                    name: "velocity".to_owned(),
                    field_type: FieldType::Message(Arc::new(MessageSchema {
                        type_name: "velocity".to_owned(),
                        package: "hulks".to_owned(),
                        name: "Vector2".to_owned(),
                        fields: vec![
                            FieldSchema {
                                name: "x".to_owned(),
                                field_type: FieldType::Float32,
                                default_value: None,
                            },
                            FieldSchema {
                                name: "y".to_owned(),
                                field_type: FieldType::Float32,
                                default_value: None,
                            },
                        ],
                        type_hash: None,
                    })),
                    default_value: None,
                },
                FieldSchema {
                    name: "last_seen".to_owned(),
                    field_type: FieldType::Message(Arc::new(MessageSchema {
                        type_name: "SystemTime".to_owned(),
                        package: "hulks".to_owned(),
                        name: "SystemTime".to_owned(),
                        fields: vec![FieldSchema {
                            name: "TimeSpec".to_owned(),
                            field_type: FieldType::Message(Arc::new(MessageSchema {
                                type_name: "TimeSpec".to_owned(),
                                package: "hulks".to_owned(),
                                name: "TimeSpec".to_owned(),
                                fields: vec![
                                    FieldSchema {
                                        name: "seconds".to_owned(),
                                        field_type: FieldType::Int64,
                                        default_value: None,
                                    },
                                    FieldSchema {
                                        name: "nano_seconds".to_owned(),
                                        field_type: FieldType::Message(Arc::new(MessageSchema {
                                            type_name: "NanoSeconds".to_owned(),
                                            package: "hulks".to_owned(),
                                            name: "NanoSeconds".to_owned(),
                                            fields: vec![FieldSchema {
                                                name: "NanoSeconds".to_owned(),
                                                field_type: FieldType::Uint32,
                                                default_value: None,
                                            }],
                                            type_hash: None,
                                        })),
                                        default_value: None,
                                    },
                                ],
                                type_hash: None,
                            })),
                            default_value: None,
                        }],
                        type_hash: None,
                    })),
                    default_value: None,
                },
            ],
            type_hash: None,
        }))
    }
}

impl ZMessage for BallPosition<Ground> {
    type Serdes = SerdeCdrSerdes<Self>;
}

impl<Frame> WithTypeInfo for BallPosition<Frame> {}

impl<Frame> BallPosition<Frame> {
    pub fn from_network_ball(
        network_ball: hsl_network_messages::BallPosition<Frame>,
        message_time: SystemTime,
    ) -> Self {
        Self {
            position: network_ball.position,
            velocity: Vector2::zeros(),
            last_seen: message_time - network_ball.age,
        }
    }
}

impl<From, To> Mul<BallPosition<From>> for Isometry2<From, To> {
    type Output = BallPosition<To>;

    fn mul(self, rhs: BallPosition<From>) -> Self::Output {
        BallPosition {
            position: self * rhs.position,
            velocity: self * rhs.velocity,
            last_seen: rhs.last_seen,
        }
    }
}

#[derive(
    Clone,
    Copy,
    Debug,
    Default,
    Deserialize,
    PathDeserialize,
    PathIntrospect,
    PathSerialize,
    Serialize,
)]
pub struct SimulatorBallState {
    pub position: Point2<Field>,
    pub velocity: Vector2<Field>,
}

#[derive(
    Clone, Copy, Debug, Deserialize, Serialize, PathSerialize, PathDeserialize, PathIntrospect,
)]
pub struct HypotheticalBallPosition<Frame> {
    pub position: Point2<Frame>,
    pub validity: f32,
}
