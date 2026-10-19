use std::{pin::Pin, sync::Arc, time::Duration};

use color_eyre::Result;
use hsl_network_messages::{HulkMessage, PlayerNumber};
use ros_z::{prelude::*, qos::QosDurability, time::Time};
use serde::{Deserialize, Serialize};
use types::{
    ball_position::BallPosition, filtered_game_controller_state::FilteredGameControllerState,
    messages::IncomingMessage, players::Players, time_wrapper::TimeWrapper,
    world_state::PlayerState,
};

#[derive(Debug, Clone, Serialize, Deserialize, Message)]
#[serde(deny_unknown_fields)]
pub struct Parameters {
    pub maximum_age: Duration,
}

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx))
}

pub async fn run(ctx: Arc<Context>) -> Result<()> {
    let node = ctx.create_node("player_states_receiver").build().await?;

    let parameters = node.bind_parameter_as::<Parameters>("player_states_receiver")?;
    let filtered_game_controller_state_sub = node
        .subscriber::<FilteredGameControllerState>("filtered_game_controller_state")
        .build()
        .await?;
    let filtered_message_sub = node
        .subscriber::<TimeWrapper<IncomingMessage>>("filtered_message")
        .build()
        .await?;
    let player_states_pub = node
        .publisher::<Players<Option<TimeWrapper<PlayerState>>>>("player_states")
        .qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        })
        .build()
        .await?;

    let mut player_states = Players::new(None);
    // Keep receipt watermarks after expiration or penalties so a delayed packet
    // cannot restore an older pass offer after a newer cancellation.
    let mut last_received_times = Players::new(None);
    let mut last_game_controller_state = None;

    let mut expiry_tick = node.create_timer(Duration::from_millis(100));
    loop {
        tokio::select! {
            _ = expiry_tick.tick() => {
                let maximum_age = parameters.snapshot().typed().maximum_age;
                if clear_old_player_states(&mut player_states, node.clock().now(), maximum_age) {
                    player_states_pub.publish(&player_states).await?;
                }
            }
            received_game_controller_state = filtered_game_controller_state_sub.recv() => {
                last_game_controller_state = received_game_controller_state.ok();
                if let Some(game_controller_state) = &last_game_controller_state{
                    clear_penalized_players(&mut player_states, game_controller_state);
                }
                let maximum_age = parameters.snapshot().typed().maximum_age;
                clear_old_player_states(&mut player_states, node.clock().now(), maximum_age);
                player_states_pub.publish(&player_states).await?;
            }
            received_message = filtered_message_sub.recv() => {
                apply_message(
                    &mut player_states,
                    &mut last_received_times,
                    received_message?,
                    node.clock().now(),
                );
                if let Some(game_controller_state) = &last_game_controller_state{
                    clear_penalized_players(&mut player_states, game_controller_state);
                }
                let maximum_age = parameters.snapshot().typed().maximum_age;
                clear_old_player_states(&mut player_states, node.clock().now(), maximum_age);
                player_states_pub.publish(&player_states).await?;
            }
        }
    }
}

fn apply_message(
    player_states: &mut Players<Option<TimeWrapper<PlayerState>>>,
    last_received_times: &mut Players<Option<Time>>,
    message: TimeWrapper<IncomingMessage>,
    now: Time,
) {
    let TimeWrapper {
        time,
        inner: IncomingMessage::Hsl(message),
    } = message
    else {
        return;
    };

    let (state_message, pass, pass_ready, unavailable) = match message {
        HulkMessage::State(state) => (state, None, None, false),
        HulkMessage::CoordinatedState(message) => {
            (message.state, message.pass, message.pass_ready, false)
        }
        HulkMessage::UnavailableState(state) => (state, None, None, true),
    };
    if time > now
        || last_received_times[state_message.player_number].is_some_and(|previous| time <= previous)
    {
        return;
    }
    last_received_times[state_message.player_number] = Some(time);

    player_states[state_message.player_number] = Some(TimeWrapper {
        time,
        inner: PlayerState {
            pose: state_message.pose,
            ball_position: state_message
                .ball_position
                .map(|ball| BallPosition::from_network_ball(ball, time)),
            pass,
            pass_ready,
            unavailable,
        },
    });
}

fn clear_penalized_players(
    player_states: &mut Players<Option<TimeWrapper<PlayerState>>>,
    game_controller_state: &FilteredGameControllerState,
) {
    for (player_number, penalty) in game_controller_state.penalties.iter() {
        if penalty.is_some() {
            player_states[player_number] = None;
        }
    }
}

fn clear_old_player_states(
    player_states: &mut Players<Option<TimeWrapper<PlayerState>>>,
    now: Time,
    maximum_age: Duration,
) -> bool {
    let mut removed_any = false;
    for player_number in [
        PlayerNumber::One,
        PlayerNumber::Two,
        PlayerNumber::Three,
        PlayerNumber::Four,
        PlayerNumber::Five,
    ] {
        let player_state = &mut player_states[player_number];
        if player_state.as_ref().is_some_and(|player_state| {
            now < player_state.time || now.duration_since(player_state.time) >= maximum_age
        }) {
            *player_state = None;
            removed_any = true;
        }
    }
    removed_any
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    use hsl_network_messages::{
        CoordinatedStateMessage, PassIntent, PassReady, Penalty, PlayerNumber, StateMessage,
    };
    use linear_algebra::{Pose2, point};
    use ros_z::time::Time;
    use types::messages::IncomingMessage;

    #[test]
    fn state_message_updates_player_with_receive_time() {
        let mut states = Players::new(None);
        let mut last_received_times = Players::new(None);
        let time = Time::from_nanos(42);
        let pose = Pose2::default();

        apply_message(
            &mut states,
            &mut last_received_times,
            TimeWrapper {
                time,
                inner: IncomingMessage::Hsl(HulkMessage::State(StateMessage {
                    player_number: PlayerNumber::Two,
                    pose,
                    head_yaw: 0.0,
                    ball_position: None,
                })),
            },
            time,
        );

        let player = states[PlayerNumber::Two].as_ref().unwrap();
        assert_eq!(player.time, time);
        assert_eq!(player.inner.pose, pose);
        assert!(!player.inner.unavailable);
    }

    fn coordinated_message(time: i64) -> TimeWrapper<IncomingMessage> {
        TimeWrapper {
            time: Time::from_nanos(time),
            inner: IncomingMessage::Hsl(HulkMessage::CoordinatedState(CoordinatedStateMessage {
                state: StateMessage {
                    player_number: PlayerNumber::Two,
                    ..Default::default()
                },
                pass: Some(PassIntent {
                    receiver: PlayerNumber::Three,
                    target: point!(1.0, 0.5),
                }),
                pass_ready: Some(PassReady {
                    sender: PlayerNumber::Four,
                    target: point!(-1.0, -0.5),
                }),
            })),
        }
    }

    #[test]
    fn coordinated_state_preserves_receipt_time_and_pass_status() {
        let mut states = Players::new(None);
        let mut last_received_times = Players::new(None);
        let message = coordinated_message(42);
        apply_message(
            &mut states,
            &mut last_received_times,
            message,
            Time::from_nanos(50),
        );

        let state = states.two.as_ref().unwrap();
        assert_eq!(state.time, Time::from_nanos(42));
        assert!(!state.inner.unavailable);
        assert_eq!(
            state.inner.pass,
            Some(PassIntent {
                receiver: PlayerNumber::Three,
                target: point!(1.0, 0.5)
            })
        );
        assert_eq!(
            state.inner.pass_ready,
            Some(PassReady {
                sender: PlayerNumber::Four,
                target: point!(-1.0, -0.5)
            })
        );
    }

    #[test]
    fn legacy_state_clears_pass_status_and_old_packets_cannot_restore_it() {
        let mut states = Players::new(None);
        let mut last_received_times = Players::new(None);
        apply_message(
            &mut states,
            &mut last_received_times,
            coordinated_message(42),
            Time::from_nanos(50),
        );
        apply_message(
            &mut states,
            &mut last_received_times,
            TimeWrapper {
                time: Time::from_nanos(43),
                inner: IncomingMessage::Hsl(HulkMessage::State(StateMessage {
                    player_number: PlayerNumber::Two,
                    ..Default::default()
                })),
            },
            Time::from_nanos(50),
        );
        apply_message(
            &mut states,
            &mut last_received_times,
            coordinated_message(42),
            Time::from_nanos(50),
        );
        let state = states.two.as_ref().unwrap();
        assert_eq!(state.time, Time::from_nanos(43));
        assert!(state.inner.pass.is_none());
        assert!(state.inner.pass_ready.is_none());

        clear_old_player_states(
            &mut states,
            Time::from_nanos(200),
            Duration::from_nanos(100),
        );
        assert!(states.two.is_none());
        apply_message(
            &mut states,
            &mut last_received_times,
            coordinated_message(42),
            Time::from_nanos(201),
        );
        assert!(states.two.is_none());
    }

    #[test]
    fn unavailable_state_keeps_the_pose_withdraws_coordination_and_recovers_monotonically() {
        for recover_with_coordination in [false, true] {
            let mut states = Players::new(None);
            let mut last_received_times = Players::new(None);
            let now = Time::from_nanos(100);
            apply_message(
                &mut states,
                &mut last_received_times,
                coordinated_message(42),
                now,
            );
            let pose = Pose2::new(point![2.0, -1.0], 0.7);
            let unavailable = HulkMessage::UnavailableState(StateMessage {
                player_number: PlayerNumber::Two,
                pose,
                ball_position: Some(hsl_network_messages::BallPosition {
                    position: point![3.0, -0.5],
                    age: Duration::from_nanos(10),
                }),
                ..Default::default()
            });
            apply_message(
                &mut states,
                &mut last_received_times,
                TimeWrapper {
                    time: Time::from_nanos(43),
                    inner: IncomingMessage::Hsl(unavailable),
                },
                now,
            );
            let state = states.two.as_ref().unwrap();
            assert_eq!(state.time, Time::from_nanos(43));
            assert!(state.inner.unavailable);
            assert_eq!(state.inner.pose, pose);
            assert_eq!(
                state.inner.ball_position.unwrap().position,
                point![3.0, -0.5]
            );
            assert_eq!(
                state.inner.ball_position.unwrap().last_seen,
                Time::from_nanos(33)
            );
            assert!(state.inner.pass.is_none());
            assert!(state.inner.pass_ready.is_none());

            apply_message(
                &mut states,
                &mut last_received_times,
                coordinated_message(42),
                now,
            );
            assert!(states.two.as_ref().unwrap().inner.unavailable);
            assert_eq!(states.two.as_ref().unwrap().time, Time::from_nanos(43));

            let recovery = if recover_with_coordination {
                coordinated_message(44)
            } else {
                TimeWrapper {
                    time: Time::from_nanos(44),
                    inner: IncomingMessage::Hsl(HulkMessage::State(StateMessage {
                        player_number: PlayerNumber::Two,
                        ..Default::default()
                    })),
                }
            };
            apply_message(&mut states, &mut last_received_times, recovery, now);
            let state = states.two.as_ref().unwrap();
            assert_eq!(state.time, Time::from_nanos(44));
            assert!(!state.inner.unavailable);
            assert_eq!(state.inner.pass.is_some(), recover_with_coordination);

            // Both delayed and duplicate unavailability packets must not undo recovery.
            for time in [43, 44] {
                apply_message(
                    &mut states,
                    &mut last_received_times,
                    TimeWrapper {
                        time: Time::from_nanos(time),
                        inner: IncomingMessage::Hsl(unavailable),
                    },
                    now,
                );
                assert!(!states.two.as_ref().unwrap().inner.unavailable);
                assert_eq!(states.two.as_ref().unwrap().time, Time::from_nanos(44));
            }
        }
    }

    #[test]
    fn expired_unavailable_state_keeps_its_receipt_watermark_until_a_new_packet() {
        let mut states = Players::new(None);
        let mut last_received_times = Players::new(None);
        apply_message(
            &mut states,
            &mut last_received_times,
            TimeWrapper {
                time: Time::from_nanos(43),
                inner: IncomingMessage::Hsl(HulkMessage::UnavailableState(StateMessage {
                    player_number: PlayerNumber::Two,
                    ..Default::default()
                })),
            },
            Time::from_nanos(50),
        );
        assert!(clear_old_player_states(
            &mut states,
            Time::from_nanos(200),
            Duration::from_nanos(100)
        ));
        assert!(states.two.is_none());
        apply_message(
            &mut states,
            &mut last_received_times,
            coordinated_message(42),
            Time::from_nanos(200),
        );
        assert!(states.two.is_none());
        assert_eq!(last_received_times.two, Some(Time::from_nanos(43)));
        apply_message(
            &mut states,
            &mut last_received_times,
            coordinated_message(201),
            Time::from_nanos(201),
        );
        assert!(!states.two.as_ref().unwrap().inner.unavailable);
        assert_eq!(states.two.as_ref().unwrap().time, Time::from_nanos(201));
    }

    #[test]
    fn future_packets_are_rejected_without_preventing_the_next_valid_packet() {
        let mut states = Players::new(None);
        let mut last_received_times = Players::new(None);
        apply_message(
            &mut states,
            &mut last_received_times,
            coordinated_message(100),
            Time::from_nanos(50),
        );
        assert!(states.two.is_none());
        assert!(last_received_times.two.is_none());
        apply_message(
            &mut states,
            &mut last_received_times,
            coordinated_message(45),
            Time::from_nanos(50),
        );
        assert_eq!(states.two.as_ref().unwrap().time, Time::from_nanos(45));
    }

    #[test]
    fn penalized_players_are_cleared() {
        let mut states = Players::new(None);
        states[PlayerNumber::Two] = Some(TimeWrapper {
            time: Time::from_nanos(10),
            inner: PlayerState::default(),
        });
        states[PlayerNumber::Three] = Some(TimeWrapper {
            time: Time::from_nanos(20),
            inner: PlayerState::default(),
        });

        let game_controller_state = FilteredGameControllerState {
            penalties: Players {
                two: Some(Penalty::PickUp {
                    remaining: Duration::ZERO,
                }),
                ..Default::default()
            },
            ..Default::default()
        };

        clear_penalized_players(&mut states, &game_controller_state);

        assert!(states[PlayerNumber::Two].is_none());
        assert!(states[PlayerNumber::Three].is_some());
    }

    #[test]
    fn old_player_states_are_cleared() {
        let mut states = Players::new(None);
        states[PlayerNumber::Two] = Some(TimeWrapper {
            time: Time::from_nanos(1_000_000_000),
            inner: PlayerState::default(),
        });
        states[PlayerNumber::Three] = Some(TimeWrapper {
            time: Time::from_nanos(1_500_000_000),
            inner: PlayerState::default(),
        });

        let removed_any = clear_old_player_states(
            &mut states,
            Time::from_nanos(2_000_000_000),
            Duration::from_secs(1),
        );

        assert!(removed_any);
        assert!(states[PlayerNumber::Two].is_none());
        assert!(states[PlayerNumber::Three].is_some());
    }

    #[test]
    fn fresh_player_states_are_kept() {
        let mut states = Players::new(None);
        states[PlayerNumber::Two] = Some(TimeWrapper {
            time: Time::from_nanos(1_000_000_001),
            inner: PlayerState::default(),
        });

        let removed_any = clear_old_player_states(
            &mut states,
            Time::from_nanos(2_000_000_000),
            Duration::from_secs(1),
        );

        assert!(!removed_any);
        assert!(states[PlayerNumber::Two].is_some());
    }
}
