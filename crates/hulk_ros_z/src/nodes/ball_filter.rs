use std::sync::Arc;

use color_eyre::Result;
use linear_algebra::vector;
use projection::{Projection, camera_matrix::CameraMatrix};
use ros_z::{Builder, MessageTypeInfo, TypeHash, context::ZContext};
use ros_z_config::prelude::*;
use serde::{Deserialize, Serialize};
use types::object_detection::{Detection, NaoLabelPartyObjectDetectionLabel};

use crate::{
    IntoEyreResultExt,
    config::BallFilterConfig,
    msgs::{MaybeBallPosition, ZBallPosition},
};
use coordinate_systems::Ground;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraMatrixOption {
    camera_matrix: Option<CameraMatrix>,
}
impl MessageTypeInfo for CameraMatrixOption {
    fn type_name() -> &'static str {
        "ros_z_config::msg::dds_::NodeConfigEvent_"
    }

    fn type_hash() -> TypeHash {
        TypeHash::zero()
    }
}
impl ros_z::msg::ZMessage for CameraMatrixOption {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}
impl CameraMatrixOption {
    pub fn idle() -> Self {
        Self {
            camera_matrix: None,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectionVec {
    detected_objects: Vec<Detection<NaoLabelPartyObjectDetectionLabel>>,
}
impl MessageTypeInfo for DetectionVec {
    fn type_name() -> &'static str {
        "ros_z_config::msg::dds_::NodeConfigEvent_"
    }

    fn type_hash() -> TypeHash {
        TypeHash::zero()
    }
}
impl ros_z::msg::ZMessage for DetectionVec {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}
impl DetectionVec {
    pub fn idle() -> Self {
        Self {
            detected_objects: Vec::new(),
        }
    }
}

pub async fn run(ctx: Arc<ZContext>) -> Result<()> {
    let node = ctx
        .create_node("ball_filter")
        .with_type_description_service()
        .with_extended_type_description_service()
        .build()
        .into_eyre()?;
    let config = node
        .bind_config_with_metadata_as::<BallFilterConfig>("ball_filter")
        .into_eyre()?;

    let camera_matrix_sub = node
        .create_sub::<CameraMatrixOption>("camera_matrix")
        .build()
        .into_eyre()?;
    let detected_objects_sub = node
        .create_sub::<DetectionVec>("detected_objects")
        .build()
        .into_eyre()?;
    let ball_position_pub = node
        .create_pub::<MaybeBallPosition>("ball_filter/ball_position")
        .build()
        .into_eyre()?;

    let mut latest_camera_matrix = CameraMatrixOption::idle();

    loop {
        let cfg = config.snapshot().typed().clone();

        tokio::select! {
            msg = camera_matrix_sub.async_recv() => {
                latest_camera_matrix = msg.into_eyre()?;
            }
            msg = detected_objects_sub.async_recv() => {
                let latest_detected_objects = msg.into_eyre()?;
                let ball_positions: Vec<ZBallPosition<Ground>> = latest_detected_objects.detected_objects
                    .into_iter()
                    .filter_map(|detection| {
                        if detection.label != NaoLabelPartyObjectDetectionLabel::Ball {
                            return None;
                        }
                        let area = detection.bounding_box.area;
                        let position = latest_camera_matrix.camera_matrix.as_ref()?
                            .pixel_to_ground_with_z(area.center(), cfg.ball_radius)
                            .ok()?;
                        Some(ZBallPosition{
                            position: position,
                            velocity: vector![0.0, 0.0],
                            last_seen: node.clock().now(),
                        })
                    }).collect();
                    ball_position_pub.async_publish(&MaybeBallPosition { position: ball_positions.first().copied()}).await.into_eyre()?;
            }
        }
    }
}
