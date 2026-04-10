use std::{sync::Arc, time::Duration};

use color_eyre::Result;
use ros_z::Builder;
use ros_z_config::prelude::*;
use tracing::info;

use crate::{
    IntoEyreResultExt,
    config::MotionConfig,
    msgs::{LowLevelCommand, MotionIntent, timestamp_now},
    topics,
};

pub async fn run(ctx: Arc<ros_z::context::ZContext>) -> Result<()> {
    let node = ctx
        .create_node("motion")
        .with_type_description_service()
        .with_extended_type_description_service()
        .build()
        .into_eyre()?;
    let config = node
        .bind_config_with_metadata_as::<MotionConfig>("motion")
        .into_eyre()?;

    let intent_sub = node
        .create_sub::<MotionIntent>(topics::BEHAVIOR_MOTION_INTENT)
        .build()
        .into_eyre()?;
    let command_pub = node
        .create_pub::<LowLevelCommand>(topics::CONTROL_LOW_LEVEL_COMMAND)
        .build()
        .into_eyre()?;

    let mut latest_intent = MotionIntent::idle(timestamp_now());

    loop {
        let cfg = config.snapshot().typed().clone();
        let publish_hz = cfg.timing.publish_hz.max(1.0);

        tokio::select! {
            msg = intent_sub.async_recv() => {
                latest_intent = msg.into_eyre()?;
            }
            _ = tokio::time::sleep(Duration::from_secs_f64(1.0 / publish_hz)) => {
                let command = LowLevelCommand {
                    timestamp_ns: timestamp_now(),
                    mode: latest_intent.mode.clone(),
                    forward: latest_intent.forward.clamp(-cfg.limits.max_forward, cfg.limits.max_forward),
                    lateral: latest_intent.lateral.clamp(-cfg.limits.max_lateral, cfg.limits.max_lateral),
                    angular: latest_intent.angular.clamp(-cfg.limits.max_angular, cfg.limits.max_angular),
                };

                command_pub.async_publish(&command).await.into_eyre()?;

                if cfg.output.mode == "log_only" {
                    info!(?command, "published low-level command in log_only mode");
                }
            }
        }
    }

    #[allow(unreachable_code)]
    Ok(())
}
