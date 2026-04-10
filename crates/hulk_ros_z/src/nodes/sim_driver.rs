use std::{sync::Arc, time::Duration};

use color_eyre::Result;
use ros_z::{Builder, ZBuf, context::ZContext};
use ros_z_config::prelude::*;
use ros_z_msgs::sensor_msgs::{CameraInfo, Image};
use tracing::warn;

use crate::{
    IntoEyreResultExt,
    config::SimDriverConfig,
    msgs::{
        BUTTON_EVENT_SINGLE_CLICK, BUTTON_F1, ButtonEvent, FALL_DOWN_IS_READY, FallDownState,
        OdometryState, header, timestamp_now,
    },
    topics,
};

pub async fn run(ctx: Arc<ZContext>) -> Result<()> {
    let node = ctx
        .create_node("sim_driver")
        .with_type_description_service()
        .with_extended_type_description_service()
        .build()
        .into_eyre()?;
    let config = node
        .bind_config_with_metadata_as::<SimDriverConfig>("sim_driver")
        .into_eyre()?;
    config
        .add_validation_hook(Arc::new(validate_sim_driver_config))
        .into_eyre()?;

    let odom_pub = node
        .create_pub::<OdometryState>(topics::SENSORS_ODOMETRY)
        .build()
        .into_eyre()?;
    let fall_pub = node
        .create_pub::<FallDownState>(topics::SENSORS_FALL_DOWN_STATE)
        .build()
        .into_eyre()?;
    let button_pub = node
        .create_pub::<ButtonEvent>(topics::SENSORS_BUTTON_EVENT)
        .build()
        .into_eyre()?;
    let image_pub = node
        .create_pub::<Image>(topics::SENSORS_IMAGE)
        .build()
        .into_eyre()?;
    let camera_info_pub = node
        .create_pub::<CameraInfo>(topics::SENSORS_CAMERA_INFO)
        .build()
        .into_eyre()?;

    let mut tick: u64 = 0;
    let mut x = 0.0f32;
    let mut y = 0.0f32;
    let mut theta = 0.0f32;
    let mut timer = node.clock().timer(Duration::from_secs_f64(1.0));

    loop {
        let cfg = config.snapshot().typed().clone();
        let publish_hz = cfg.timing.publish_hz.max(1.0);

        timer.set_period(Duration::from_secs_f64(1.0 / publish_hz));
        timer.tick().await;
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
                warn!(
                    pattern = other,
                    "unsupported odometry pattern, falling back to stationary"
                );
            }
        }

        odom_pub
            .async_publish(&OdometryState {
                timestamp_ns,
                x,
                y,
                theta,
            })
            .await
            .into_eyre()?;

        fall_pub
            .async_publish(&FallDownState {
                timestamp_ns,
                fall_down_state: FALL_DOWN_IS_READY.to_owned(),
                is_recovery_available: true,
            })
            .await
            .into_eyre()?;

        if tick % 150 == 0 {
            button_pub
                .async_publish(&ButtonEvent {
                    timestamp_ns,
                    button: BUTTON_F1,
                    event_type: BUTTON_EVENT_SINGLE_CLICK.to_owned(),
                })
                .await
                .into_eyre()?;
        }

        if cfg.image.enabled {
            let image = make_image(&cfg, timestamp_ns);
            image_pub.async_publish(&image).await.into_eyre()?;

            let camera_info = make_camera_info(&cfg, timestamp_ns);
            camera_info_pub
                .async_publish(&camera_info)
                .await
                .into_eyre()?;
        }
    }

    #[allow(unreachable_code)]
    Ok(())
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

fn validate_sim_driver_config(cfg: &SimDriverConfig) -> std::result::Result<(), String> {
    if !cfg.timing.publish_hz.is_finite() || cfg.timing.publish_hz <= 0.0 {
        return Err("sim_driver.timing.publish_hz must be > 0".to_owned());
    }
    match cfg.odometry.pattern.as_str() {
        "stationary" | "straight" | "circle" => {}
        _ => {
            return Err(
                "sim_driver.odometry.pattern must be one of: stationary, straight, circle"
                    .to_owned(),
            );
        }
    }
    if cfg.image.width == 0 {
        return Err("sim_driver.image.width must be > 0".to_owned());
    }
    if cfg.image.height == 0 {
        return Err("sim_driver.image.height must be > 0".to_owned());
    }
    Ok(())
}
