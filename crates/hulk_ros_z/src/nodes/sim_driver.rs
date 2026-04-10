use std::{sync::Arc, time::Duration};

use color_eyre::Result;
use ros_z::{Builder, ZBuf};
use ros_z_config::prelude::*;
use ros_z_msgs::sensor_msgs::{CameraInfo, Image};
use tracing::warn;

use crate::{
    config::SimDriverConfig,
    into_eyre,
    msgs::{
        BUTTON_EVENT_SINGLE_CLICK, BUTTON_F1, ButtonEvent, FALL_DOWN_IS_READY, FallDownState,
        OdometryState, header, timestamp_now,
    },
    stack::{NodeTaskHandle, StackContext},
    topics,
};

pub fn spawn(stack: Arc<StackContext>) -> Result<NodeTaskHandle> {
    let node = into_eyre(
        stack
            .ros_z
            .create_node("sim_driver")
            .with_type_description_service()
            .with_extended_type_description_service()
            .build(),
    )?;
    let config = into_eyre(node.bind_config_with_metadata_as::<SimDriverConfig>("sim_driver"))?;

    let odom_pub = into_eyre(
        node.create_pub::<OdometryState>(topics::SENSORS_ODOMETRY)
            .build(),
    )?;
    let fall_pub = into_eyre(
        node.create_pub::<FallDownState>(topics::SENSORS_FALL_DOWN_STATE)
            .build(),
    )?;
    let button_pub = into_eyre(
        node.create_pub::<ButtonEvent>(topics::SENSORS_BUTTON_EVENT)
            .build(),
    )?;
    let image_pub = into_eyre(node.create_pub::<Image>(topics::SENSORS_IMAGE).build())?;
    let camera_info_pub = into_eyre(
        node.create_pub::<CameraInfo>(topics::SENSORS_CAMERA_INFO)
            .build(),
    )?;

    Ok(tokio::spawn(async move {
        let _node = node;
        let mut tick: u64 = 0;
        let mut x = 0.0f32;
        let mut y = 0.0f32;
        let mut theta = 0.0f32;

        loop {
            let cfg = config.snapshot().typed().clone();
            let publish_hz = cfg.timing.publish_hz.max(1.0);

            tokio::select! {
                _ = stack.shutdown.cancelled() => break,
                _ = tokio::time::sleep(Duration::from_secs_f64(1.0 / publish_hz)) => {
                    tick += 1;
                    let timestamp_ns = timestamp_now();

                    match cfg.odometry.pattern.as_str() {
                        "stationary" => {}
                        "straight" => {
                            x += cfg.odometry.step_x;
                            y += cfg.odometry.step_y;
                            theta += cfg.odometry.step_theta;
                        }
                        "circle" => {
                            theta += cfg.odometry.step_theta;
                            x += cfg.odometry.step_x * theta.cos();
                            y += cfg.odometry.step_x * theta.sin();
                        }
                        other => {
                            warn!(pattern = other, "unsupported odometry pattern, falling back to stationary");
                        }
                    }

                    into_eyre(odom_pub.async_publish(&OdometryState {
                        timestamp_ns,
                        x,
                        y,
                        theta,
                    }).await)?;

                    into_eyre(fall_pub.async_publish(&FallDownState {
                        timestamp_ns,
                        fall_down_state: FALL_DOWN_IS_READY.to_owned(),
                        is_recovery_available: true,
                    }).await)?;

                    if tick % 150 == 0 {
                        into_eyre(button_pub.async_publish(&ButtonEvent {
                            timestamp_ns,
                            button: BUTTON_F1,
                            event_type: BUTTON_EVENT_SINGLE_CLICK.to_owned(),
                        }).await)?;
                    }

                    if cfg.image.enabled {
                        let image = make_image(&cfg, timestamp_ns);
                        into_eyre(image_pub.async_publish(&image).await)?;

                        let camera_info = make_camera_info(&cfg, timestamp_ns);
                        into_eyre(camera_info_pub.async_publish(&camera_info).await)?;
                    }
                }
            }
        }

        Ok(())
    }))
}

fn make_image(config: &SimDriverConfig, timestamp_ns: u64) -> Image {
    let width = config.image.width;
    let height = config.image.height;
    let mut data = vec![0u8; (width * height * 3) as usize];

    for y in 0..height {
        for x in 0..width {
            let idx = ((y * width + x) * 3) as usize;
            let is_white = match config.image.pattern.as_str() {
                "checkerboard" => ((x / 32) + (y / 32)) % 2 == 0,
                _ => ((x / 32) + (y / 32)) % 2 == 0,
            };
            let value = if is_white { 255 } else { 32 };
            data[idx] = value;
            data[idx + 1] = value;
            data[idx + 2] = value;
        }
    }

    Image {
        header: header("camera", timestamp_ns),
        height,
        width,
        encoding: "rgb8".to_owned(),
        is_bigendian: 0,
        step: width * 3,
        data: ZBuf::from(data),
    }
}

fn make_camera_info(config: &SimDriverConfig, timestamp_ns: u64) -> CameraInfo {
    CameraInfo {
        header: header("camera", timestamp_ns),
        width: config.image.width,
        height: config.image.height,
        distortion_model: "plumb_bob".to_owned(),
        ..CameraInfo::default()
    }
}
