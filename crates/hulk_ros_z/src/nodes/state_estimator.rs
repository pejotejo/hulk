use std::{sync::Arc, time::Duration};

use color_eyre::Result;
use ros_z::{Builder, context::ZContext};
use ros_z_config::prelude::*;

use crate::{
    IntoEyreResultExt,
    config::StateEstimatorConfig,
    msgs::{ButtonEvent, FallDownState, OdometryState, RobotState, timestamp_now},
    topics,
};

pub async fn run(ctx: Arc<ZContext>) -> Result<()> {
    let node = ctx
        .create_node("state_estimator")
        .with_type_description_service()
        .with_extended_type_description_service()
        .build()
        .into_eyre()?;
    let config = node
        .bind_config_with_metadata_as::<StateEstimatorConfig>("state_estimator")
        .into_eyre()?;

    let odom_sub = node
        .create_sub::<OdometryState>(topics::SENSORS_ODOMETRY)
        .build()
        .into_eyre()?;
    let fall_sub = node
        .create_sub::<FallDownState>(topics::SENSORS_FALL_DOWN_STATE)
        .build()
        .into_eyre()?;
    let button_sub = node
        .create_sub::<ButtonEvent>(topics::SENSORS_BUTTON_EVENT)
        .build()
        .into_eyre()?;
    let robot_state_pub = node
        .create_pub::<RobotState>(topics::STATE_ROBOT_STATE)
        .build()
        .into_eyre()?;

    let mut latest_odometry = OdometryState::default();
    let mut latest_fall_down = FallDownState::default();
    let mut last_button_event: Option<ButtonEvent> = None;

    loop {
        let cfg = config.snapshot().typed().clone();
        let publish_hz = cfg.timing.publish_hz.max(1.0);

        tokio::select! {
            msg = odom_sub.async_recv() => {
                let msg = msg.into_eyre()?;
                let alpha = cfg.smoothing.odometry_alpha.clamp(0.0, 1.0);
                latest_odometry.x = latest_odometry.x * (1.0 - alpha) + msg.x * alpha;
                latest_odometry.y = latest_odometry.y * (1.0 - alpha) + msg.y * alpha;
                latest_odometry.theta = latest_odometry.theta * (1.0 - alpha) + msg.theta * alpha;
                latest_odometry.timestamp_ns = msg.timestamp_ns;
            }
            msg = fall_sub.async_recv() => {
                latest_fall_down = msg.into_eyre()?;
            }
            msg = button_sub.async_recv() => {
                last_button_event = Some(msg.into_eyre()?);
            }
            _ = tokio::time::sleep(Duration::from_secs_f64(1.0 / publish_hz)) => {
                let now = timestamp_now();
                let fresh_button_event = last_button_event.clone().filter(|event| {
                    now.saturating_sub(event.timestamp_ns) <= cfg.inputs.button_event_max_age_ms * 1_000_000
                });

                robot_state_pub.async_publish(&RobotState {
                    timestamp_ns: now,
                    odometry: latest_odometry,
                    fall_down_state: latest_fall_down.clone(),
                    has_button_event: fresh_button_event.is_some(),
                    last_button_event: fresh_button_event.unwrap_or(ButtonEvent {
                        timestamp_ns: 0,
                        button: 0,
                        event_type: String::new(),
                    }),
                }).await.into_eyre()?;
            }
        }
    }

    #[allow(unreachable_code)]
    Ok(())
}
