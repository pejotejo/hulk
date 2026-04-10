use std::{sync::Arc, time::Duration};

use color_eyre::Result;
use ros_z::Builder;
use ros_z_config::prelude::*;
use ros_z_msgs::sensor_msgs::{CameraInfo, Image};
use tracing::{info, warn};

use crate::{
    config::VisionConfig,
    into_eyre,
    msgs::{VisionStatus, timestamp_now},
    stack::{NodeTaskHandle, StackContext},
    topics,
};

pub fn spawn(stack: Arc<StackContext>) -> Result<NodeTaskHandle> {
    let node = into_eyre(
        stack
            .ros_z
            .create_node("vision")
            .with_type_description_service()
            .with_extended_type_description_service()
            .build(),
    )?;
    let config = into_eyre(node.bind_config_with_metadata_as::<VisionConfig>("vision"))?;

    let image_sub = into_eyre(node.create_sub::<Image>(topics::SENSORS_IMAGE).build())?;
    let camera_info_sub = into_eyre(
        node.create_sub::<CameraInfo>(topics::SENSORS_CAMERA_INFO)
            .build(),
    )?;
    let status_pub = into_eyre(
        node.create_pub::<VisionStatus>(topics::VISION_STATUS)
            .build(),
    )?;

    Ok(tokio::spawn(async move {
        let _node = node;
        let mut frame_count = 0u64;
        let mut last_frame_timestamp_ns = 0u64;
        let mut last_camera_info_timestamp_ns = 0u64;

        loop {
            let cfg = config.snapshot().typed().clone();
            let publish_hz = cfg.status.publish_hz.max(0.5);

            tokio::select! {
                _ = stack.shutdown.cancelled() => break,
                msg = image_sub.async_recv() => {
                    let image = into_eyre(msg)?;
                    frame_count += 1;
                    last_frame_timestamp_ns = timestamp_from_stamp(&image.header.stamp);
                    if cfg.debug.log_frame_rate && frame_count % 30 == 0 {
                        info!(frame_count, "vision received image frames");
                    }
                }
                msg = camera_info_sub.async_recv() => {
                    let camera_info = into_eyre(msg)?;
                    last_camera_info_timestamp_ns = timestamp_from_stamp(&camera_info.header.stamp);
                }
                _ = tokio::time::sleep(Duration::from_secs_f64(1.0 / publish_hz)) => {
                    let now = timestamp_now();
                    if cfg.inputs.image_required && is_stale(last_frame_timestamp_ns, now, cfg.inputs.max_frame_age_ms) {
                        warn!("vision has not received a fresh image frame");
                    }
                    if cfg.inputs.camera_info_required && is_stale(last_camera_info_timestamp_ns, now, cfg.inputs.max_frame_age_ms) {
                        warn!("vision has not received fresh camera info");
                    }

                    into_eyre(status_pub.async_publish(&VisionStatus {
                        frame_count,
                        last_frame_timestamp_ns,
                        last_camera_info_timestamp_ns,
                        heartbeat_timestamp_ns: now,
                    }).await)?;
                }
            }
        }

        Ok(())
    }))
}

fn is_stale(value: u64, now: u64, max_age_ms: u64) -> bool {
    value == 0 || now.saturating_sub(value) > max_age_ms * 1_000_000
}

fn timestamp_from_stamp(stamp: &ros_z_msgs::builtin_interfaces::Time) -> u64 {
    (stamp.sec as u64) * 1_000_000_000 + u64::from(stamp.nanosec)
}
