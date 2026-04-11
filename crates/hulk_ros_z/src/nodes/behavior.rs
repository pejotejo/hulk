use std::{sync::Arc, time::Duration};

use color_eyre::Result;
use ros_z::{Builder, context::ZContext};
use ros_z_config::prelude::*;

use crate::{
    IntoEyreResultExt,
    config::BehaviorConfig,
    msgs::{
        BUTTON_EVENT_DOUBLE_CLICK, BUTTON_EVENT_LONG_PRESS_START, BUTTON_EVENT_SINGLE_CLICK,
        DemoMode, MotionIntent, RobotState, timestamp_now,
    },
};

pub async fn run(ctx: Arc<ZContext>) -> Result<()> {
    let node = ctx
        .create_node("behavior")
        .with_type_description_service()
        .with_extended_type_description_service()
        .build()
        .into_eyre()?;
    let config = node
        .bind_config_with_metadata_as::<BehaviorConfig>("behavior")
        .into_eyre()?;
    config
        .add_validation_hook(|cfg: &BehaviorConfig| {
            for (name, value, min, max) in [
                ("behavior.walk.forward", cfg.walk.forward, -1.0, 1.0),
                ("behavior.walk.lateral", cfg.walk.lateral, -1.0, 1.0),
                ("behavior.walk.angular", cfg.walk.angular, -2.0, 2.0),
            ] {
                if !value.is_finite() {
                    return Err(format!("{name} must be finite"));
                }
                if value < min || value > max {
                    return Err(format!("{name} must be between {min} and {max}"));
                }
            }
            Ok(())
        })
        .into_eyre()?;

    let state_sub = node
        .create_sub::<RobotState>("state/robot_state")
        .build()
        .into_eyre()?;
    let intent_pub = node
        .create_pub::<MotionIntent>("behavior/motion_intent")
        .build()
        .into_eyre()?;

    let mut latest_state: Option<RobotState> = None;
    let mut current_mode = DemoMode::Stand;
    let mut last_button_timestamp_ns = 0u64;
    let mut timer = node.clock().timer(Duration::from_secs_f64(1.0 / 30.0));

    loop {
        let cfg = config.snapshot().typed().clone();

        tokio::select! {
            msg = state_sub.async_recv() => {
                    let state = msg.into_eyre()?;
                    if state.has_button_event {
                        let button_event = &state.last_button_event;
                        if cfg.mode.allow_button_override && button_event.timestamp_ns > last_button_timestamp_ns {
                            current_mode = match button_event.event_type.as_str() {
                                BUTTON_EVENT_SINGLE_CLICK => cfg.buttons.single_click_mode,
                                BUTTON_EVENT_DOUBLE_CLICK => cfg.buttons.double_click_mode,
                                BUTTON_EVENT_LONG_PRESS_START => cfg.buttons.long_press_mode,
                                _ => current_mode,
                            };
                            last_button_timestamp_ns = button_event.timestamp_ns;
                        }
                    }
                    latest_state = Some(state);
                }
                _ = timer.tick() => {
                let default_mode = cfg.mode.default;
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

                intent_pub.async_publish(&MotionIntent {
                    timestamp_ns: timestamp_now(),
                    mode,
                    forward: walk.0,
                    lateral: walk.1,
                    angular: walk.2,
                }).await.into_eyre()?;
            }
        }
    }
}
