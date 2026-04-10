use std::{sync::Arc, time::Duration};

use color_eyre::Result;
use ros_z::{Builder, context::ZContext};
use ros_z_config::prelude::*;
use ros_z_msgs::sensor_msgs::{CameraInfo, Image};
use tracing::{info, warn};

use crate::{
    IntoEyreResultExt,
    config::VisionConfig,
    msgs::{VisionStatus, timestamp_now},
    topics,
};

pub async fn run(ctx: Arc<ZContext>) -> Result<()> {
    let node = ctx
        .create_node("vision")
        .with_type_description_service()
        .with_extended_type_description_service()
        .build()
        .into_eyre()?;
    let config = node
        .bind_config_with_metadata_as::<VisionConfig>("vision")
        .into_eyre()?;
    config
        .add_validation_hook(validate_vision_config)
        .into_eyre()?;

    let image_sub = node
        .create_sub::<Image>(topics::SENSORS_IMAGE)
        .build()
        .into_eyre()?;
    let camera_info_sub = node
        .create_sub::<CameraInfo>(topics::SENSORS_CAMERA_INFO)
        .build()
        .into_eyre()?;
    let status_pub = node
        .create_pub::<VisionStatus>(topics::VISION_STATUS)
        .build()
        .into_eyre()?;

    let mut frame_count = 0u64;
    let mut last_frame_timestamp_ns = 0u64;
    let mut last_camera_info_timestamp_ns = 0u64;
    let mut timer = node.clock().timer(Duration::from_secs_f64(1.0));

    loop {
        let cfg = config.snapshot().typed().clone();
        let publish_hz = cfg.status.publish_hz.max(0.5);
        timer.set_period(Duration::from_secs_f64(1.0 / publish_hz));

        tokio::select! {
            msg = image_sub.async_recv() => {
                let image = msg.into_eyre()?;
                frame_count += 1;
                last_frame_timestamp_ns = timestamp_from_stamp(&image.header.stamp);
                if cfg.debug.log_frame_rate && frame_count % 30 == 0 {
                    info!(frame_count, "vision received image frames");
                }
            }
            msg = camera_info_sub.async_recv() => {
                let camera_info = msg.into_eyre()?;
                last_camera_info_timestamp_ns = timestamp_from_stamp(&camera_info.header.stamp);
            }
            _ = timer.tick() => {
                let now = timestamp_now();
                if cfg.inputs.image_required && is_stale(last_frame_timestamp_ns, now, cfg.inputs.max_frame_age_ms) {
                    warn!("vision has not received a fresh image frame");
                }
                if cfg.inputs.camera_info_required && is_stale(last_camera_info_timestamp_ns, now, cfg.inputs.max_frame_age_ms) {
                    warn!("vision has not received fresh camera info");
                }

                status_pub.async_publish(&VisionStatus {
                    frame_count,
                    last_frame_timestamp_ns,
                    last_camera_info_timestamp_ns,
                    heartbeat_timestamp_ns: now,
                }).await.into_eyre()?;
            }
        }
    }

    #[allow(unreachable_code)]
    Ok(())
}

fn is_stale(value: u64, now: u64, max_age_ms: u64) -> bool {
    value == 0 || now.saturating_sub(value) > max_age_ms * 1_000_000
}

fn timestamp_from_stamp(stamp: &ros_z_msgs::builtin_interfaces::Time) -> u64 {
    (stamp.sec as u64) * 1_000_000_000 + u64::from(stamp.nanosec)
}

fn validate_vision_config(cfg: &VisionConfig) -> std::result::Result<(), String> {
    if !cfg.status.publish_hz.is_finite() || cfg.status.publish_hz <= 0.0 {
        return Err("vision.status.publish_hz must be > 0".to_owned());
    }
    if cfg.inputs.max_frame_age_ms == 0 {
        return Err("vision.inputs.max_frame_age_ms must be > 0".to_owned());
    }
    Ok(())
}
