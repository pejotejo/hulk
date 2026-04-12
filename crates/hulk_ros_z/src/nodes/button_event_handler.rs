use std::{sync::Arc, time::Duration};

use booster::{ButtonEventMsg, ButtonEventType};
use color_eyre::Result;
use ros_z::{Builder, context::ZContext};
use types::buttons::{ButtonPressType, Buttons};

use crate::IntoEyreResultExt;

pub async fn run(ctx: Arc<ZContext>) -> Result<()> {
    let node = ctx
        .create_node("button_event_handler")
        .with_type_description_service()
        .with_extended_type_description_service()
        .build()
        .into_eyre()?;

    let button_event_cache = node
        .create_cache::<ButtonEventMsg>("robot_hw/button_event", 256)
        .build()
        .into_eyre()?;
    let status_pub = node
        .create_pub::<Buttons<Option<ButtonPressType>>>("button_event_handler/buttons")
        .build()
        .into_eyre()?;

    let mut timer = node.clock().timer(Duration::from_secs_f64(1.0));

    let mut last_button_event_types: Buttons<Option<ButtonEventType>> = Default::default();
    let mut last_query_time = node.clock().now();

    loop {
        timer.set_period(Duration::from_secs_f64(1.0 / 30.0));
        timer.tick().await;

        let now = node.clock().now();

        let button_events = button_event_cache.get_interval(last_query_time, now);

        let mut buttons = Buttons::default();

        for event in button_events {
            let event = event.as_ref();

            buttons[event.button] = ButtonPressType::from_button_event_types(
                &last_button_event_types[event.button],
                &event.event,
            );
            last_button_event_types[event.button] = Some(event.event);
        }

        status_pub.publish(&buttons).into_eyre()?;
        last_query_time = now;
    }
}
