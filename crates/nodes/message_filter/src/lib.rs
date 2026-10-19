use std::sync::Arc;
use std::{boxed::Box, future::Future, pin::Pin};

use color_eyre::Result;

use hsl_network_messages::PlayerNumber;
use ros_z::{prelude::*, qos::QosDurability};
use types::{messages::IncomingMessage, time_wrapper::TimeWrapper};

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx))
}

async fn run(ctx: Arc<Context>) -> Result<()> {
    let node = ctx.create_node("message_filter").build().await?;

    let player_number_sub = node
        .subscriber::<PlayerNumber>("player_number")
        .qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        })
        .build()
        .await?;
    let message_sub = node
        .subscriber::<TimeWrapper<IncomingMessage>>("inputs/message")
        .build()
        .await?;
    let filtered_message_pub = node
        .publisher::<TimeWrapper<IncomingMessage>>("filtered_message")
        .build()
        .await?;

    let mut player_number = None;

    loop {
        tokio::select! {
            received_time_wrapped_message = message_sub.recv() => {
                let Some(current_player_number) = player_number else {continue;};
                let time_wrapped_message = received_time_wrapped_message?;

                let should_filter_message_out = is_own_message(
                    &time_wrapped_message.inner,
                    current_player_number,
                );

                if !should_filter_message_out {
                    filtered_message_pub.publish(
                        &TimeWrapper {
                            time: time_wrapped_message.time,
                            inner: time_wrapped_message.inner
                        }
                    ).await?;
                }
            }
            received_player_number = player_number_sub.recv() => {
                player_number = Some(received_player_number?);
            }
        }
    }
}

fn is_own_message(message: &IncomingMessage, own_player: PlayerNumber) -> bool {
    matches!(message, IncomingMessage::Hsl(message) if message.state().player_number == own_player)
}

#[cfg(test)]
mod tests {
    use hsl_network_messages::{CoordinatedStateMessage, HulkMessage, StateMessage};

    use super::*;

    #[test]
    fn both_state_envelopes_filter_the_embedded_sender() {
        let state = StateMessage {
            player_number: PlayerNumber::Two,
            ..Default::default()
        };
        for message in [
            HulkMessage::State(state),
            HulkMessage::CoordinatedState(CoordinatedStateMessage {
                state,
                ..Default::default()
            }),
        ] {
            let message = IncomingMessage::Hsl(message);
            assert!(is_own_message(&message, PlayerNumber::Two));
            assert!(!is_own_message(&message, PlayerNumber::Three));
        }
    }
}
