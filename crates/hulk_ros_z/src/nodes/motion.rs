use std::{sync::Arc, time::Duration};

use color_eyre::Result;
use ros_z::Builder;
use ros_z_config::prelude::*;
use tracing::info;

use crate::{
    config::MotionConfig,
    into_eyre,
    msgs::{LowLevelCommand, MotionIntent, timestamp_now},
    stack::{NodeTaskHandle, StackContext},
    topics,
};

pub fn spawn(stack: Arc<StackContext>) -> Result<NodeTaskHandle> {
    let node = into_eyre(
        stack
            .ros_z
            .create_node("motion")
            .with_type_description_service()
            .with_extended_type_description_service()
            .build(),
    )?;
    let config = into_eyre(node.bind_config_with_metadata_as::<MotionConfig>("motion"))?;

    let intent_sub = into_eyre(
        node.create_sub::<MotionIntent>(topics::BEHAVIOR_MOTION_INTENT)
            .build(),
    )?;
    let command_pub = into_eyre(
        node.create_pub::<LowLevelCommand>(topics::CONTROL_LOW_LEVEL_COMMAND)
            .build(),
    )?;

    Ok(tokio::spawn(async move {
        let _node = node;
        let mut latest_intent = MotionIntent::idle(timestamp_now());

        loop {
            let cfg = config.snapshot().typed().clone();
            let publish_hz = cfg.timing.publish_hz.max(1.0);

            tokio::select! {
                _ = stack.shutdown.cancelled() => break,
                msg = intent_sub.async_recv() => {
                    latest_intent = into_eyre(msg)?;
                }
                _ = tokio::time::sleep(Duration::from_secs_f64(1.0 / publish_hz)) => {
                    let command = LowLevelCommand {
                        timestamp_ns: timestamp_now(),
                        mode: latest_intent.mode.clone(),
                        forward: latest_intent.forward.clamp(-cfg.limits.max_forward, cfg.limits.max_forward),
                        lateral: latest_intent.lateral.clamp(-cfg.limits.max_lateral, cfg.limits.max_lateral),
                        angular: latest_intent.angular.clamp(-cfg.limits.max_angular, cfg.limits.max_angular),
                    };

                    into_eyre(command_pub.async_publish(&command).await)?;

                    if cfg.output.mode == "log_only" {
                        info!(?command, "published low-level command in log_only mode");
                    }
                }
            }
        }

        Ok(())
    }))
}
