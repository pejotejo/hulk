use std::{str::FromStr, sync::Arc, time::Duration};

use color_eyre::{Result, eyre::eyre};
use ros_z::Builder;
use ros_z_config::prelude::*;

use crate::{
    config::BehaviorConfig,
    into_eyre,
    msgs::{
        BUTTON_EVENT_DOUBLE_CLICK, BUTTON_EVENT_LONG_PRESS_START, BUTTON_EVENT_SINGLE_CLICK,
        DemoMode, MotionIntent, RobotState, timestamp_now,
    },
    stack::{NodeTaskHandle, StackContext},
    topics,
};

pub fn spawn(stack: Arc<StackContext>) -> Result<NodeTaskHandle> {
    let node = into_eyre(
        stack
            .ros_z
            .create_node("behavior")
            .with_type_description_service()
            .with_extended_type_description_service()
            .build(),
    )?;
    let config = into_eyre(node.bind_config_with_metadata_as::<BehaviorConfig>("behavior"))?;

    let state_sub = into_eyre(
        node.create_sub::<RobotState>(topics::STATE_ROBOT_STATE)
            .build(),
    )?;
    let intent_pub = into_eyre(
        node.create_pub::<MotionIntent>(topics::BEHAVIOR_MOTION_INTENT)
            .build(),
    )?;

    Ok(tokio::spawn(async move {
        let _node = node;
        let mut latest_state: Option<RobotState> = None;
        let mut current_mode = DemoMode::Stand;
        let mut last_button_timestamp_ns = 0u64;

        loop {
            let cfg = config.snapshot().typed().clone();

            tokio::select! {
                _ = stack.shutdown.cancelled() => break,
                msg = state_sub.async_recv() => {
                    let state = into_eyre(msg)?;
                    if state.has_button_event {
                        let button_event = &state.last_button_event;
                        if cfg.mode.allow_button_override && button_event.timestamp_ns > last_button_timestamp_ns {
                            current_mode = match button_event.event_type.as_str() {
                                BUTTON_EVENT_SINGLE_CLICK => parse_mode(&cfg.buttons.single_click_mode)?,
                                BUTTON_EVENT_DOUBLE_CLICK => parse_mode(&cfg.buttons.double_click_mode)?,
                                BUTTON_EVENT_LONG_PRESS_START => parse_mode(&cfg.buttons.long_press_mode)?,
                                _ => current_mode,
                            };
                            last_button_timestamp_ns = button_event.timestamp_ns;
                        }
                    }
                    latest_state = Some(state);
                }
                _ = tokio::time::sleep(Duration::from_secs_f64(1.0 / 30.0)) => {
                    let default_mode = parse_mode(&cfg.mode.default)?;
                    if !cfg.mode.allow_button_override {
                        current_mode = default_mode;
                    }

                    let mode = match latest_state.as_ref() {
                        Some(state) if !state.is_upright() => DemoMode::Stand,
                        Some(_) => current_mode,
                        None => default_mode,
                    };

                    let walk = if matches!(mode, DemoMode::Walk) {
                        (cfg.walk.forward, cfg.walk.lateral, cfg.walk.angular)
                    } else {
                        (0.0, 0.0, 0.0)
                    };

                    into_eyre(intent_pub.async_publish(&MotionIntent {
                        timestamp_ns: timestamp_now(),
                        mode: mode.as_str().to_owned(),
                        forward: walk.0,
                        lateral: walk.1,
                        angular: walk.2,
                    }).await)?;
                }
            }
        }

        Ok(())
    }))
}

fn parse_mode(mode: &str) -> Result<DemoMode> {
    DemoMode::from_str(mode).map_err(|error| eyre!(error))
}
