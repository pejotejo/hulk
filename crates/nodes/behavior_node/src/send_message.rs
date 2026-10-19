use std::{net::SocketAddr, time::Duration};

use hsl_network_messages::{
    CoordinatedStateMessage, GameControllerReturnMessage, GamePhase, Half, HulkMessage, PassIntent,
    StateMessage,
};
use ros_z::time::Time;
use types::{
    fall_detection::Posture, filtered_game_controller_state::FilteredGameControllerState,
    messages::OutgoingMessage, parameters::HslNetworkParameters, primary_state::PrimaryState,
    time_wrapper::TimeWrapper,
};

use crate::{
    kick::{passing::matches_announced_pass, strategy::BallAction},
    node::Blackboard,
};

impl Blackboard {
    pub fn game_controller_return_message(
        &mut self,
        game_controller_address: Option<&SocketAddr>,
    ) -> Option<OutgoingMessage> {
        let now = self.world_state.now;

        if !self.is_return_message_cooldown_elapsed(now, &self.parameters.network) {
            return None;
        }
        let address = game_controller_address?;

        let ground_to_field = self.world_state.robot.ground_to_field.unwrap_or_default();

        let ball_position = self
            .world_state
            .ball
            .map(|ball| hsl_network_messages::BallPosition {
                age: now
                    .to_wallclock()
                    .duration_since(ball.last_seen_ball)
                    .unwrap(),
                position: ball.ball_in_ground,
            });

        self.last_sent_game_controller_return_message_time = Some(now);

        Some(OutgoingMessage::GameController(
            *address,
            GameControllerReturnMessage {
                player_number: self.world_state.robot.player_number,
                fallen: self
                    .world_state
                    .fall_detection
                    .is_none_or(|fall_detection| {
                        !fall_detection.is_valid(now) || fall_detection.posture != Posture::Upright
                    }),
                pose: ground_to_field.as_pose(),
                ball: ball_position,
            },
        ))
    }

    fn is_return_message_cooldown_elapsed(
        &self,
        now: Time,
        hsl_network_parameters: &HslNetworkParameters,
    ) -> bool {
        is_cooldown_elapsed(
            now,
            self.last_sent_game_controller_return_message_time,
            hsl_network_parameters.game_controller_return_message_interval,
        )
    }

    pub fn try_sending_state_message(&mut self) -> Option<OutgoingMessage> {
        let unavailable = self.world_state.fall_detection.is_some_and(|fall| {
            fall.is_valid(self.world_state.now) && fall.posture != Posture::Upright
        });
        if unavailable {
            // Withdraw coordination locally immediately; peers learn availability
            // through the same budgeted state-message cadence as ordinary poses.
            if matches!(self.ball_action, Some(BallAction::Pass { .. })) {
                self.ball_action = None;
                self.ball_action_active = false;
                self.kick_target = None;
            }
            self.last_announced_pass = None;
            self.pass_started_at = None;
            self.ready_to_receive_pass = None;
            self.receiving_pass = None;
        }
        let pass = self.current_outgoing_pass();
        if pass.is_none() {
            // Cancel the local handshake immediately, even while waiting for the
            // next budgeted state update to tell peers that this intent ended.
            self.last_announced_pass = None;
        }
        if self.world_state.robot.primary_state != PrimaryState::Playing {
            self.last_announced_pass = None;
            return None;
        }

        let now = self.world_state.now;
        let game_controller_state = self.world_state.filtered_game_controller_state.as_ref()?;
        let send_interval =
            state_message_send_interval(game_controller_state, &self.parameters.network)?;
        if !is_cooldown_elapsed(now, self.last_sent_hsl_message_time, send_interval) {
            return None;
        }
        if let Some(ground_to_field) = self.world_state.robot.ground_to_field {
            let pose = ground_to_field.as_pose();

            let ball_position =
                self.world_state
                    .ball
                    .map(|ball| hsl_network_messages::BallPosition {
                        age: now
                            .to_wallclock()
                            .duration_since(ball.last_seen_ball)
                            .unwrap(),
                        position: ball.ball_in_field,
                    });

            let state = StateMessage {
                player_number: self.world_state.robot.player_number,
                pose,
                head_yaw: self.head_yaw,
                ball_position,
            };
            let pass_ready = self.ready_to_receive_pass.filter(|ready| {
                ready.sender != self.world_state.robot.player_number
                    && self.valid_pass_target(ready.target)
            });
            let announcement_time = pass
                .filter(|pass| matches_announced_pass(self, *pass))
                .and_then(|_| self.last_announced_pass.as_ref().map(|pass| pass.time))
                .unwrap_or(now);
            self.last_announced_pass = pass.map(|inner| TimeWrapper {
                time: announcement_time,
                inner,
            });
            let message = if unavailable {
                HulkMessage::UnavailableState(state)
            } else if pass.is_some() || pass_ready.is_some() {
                HulkMessage::CoordinatedState(CoordinatedStateMessage {
                    state,
                    pass,
                    pass_ready,
                })
            } else {
                HulkMessage::State(state)
            };

            self.last_sent_hsl_message_time = Some(now);
            Some(OutgoingMessage::Hsl(message))
        } else {
            None
        }
    }

    /// A two-way handshake must fit inside the preparation timeout at the
    /// ordinary game-controller budgeted rate; passing never buys extra packets.
    pub(crate) fn has_team_message_budget(&self) -> bool {
        let timeout = self.parameters.kicking.strategy.passing.preparation_timeout;
        !timeout.is_zero()
            && self
                .world_state
                .filtered_game_controller_state
                .as_ref()
                .and_then(|game| state_message_send_interval(game, &self.parameters.network))
                .is_some_and(|interval| {
                    interval <= timeout / 2
                        && interval <= self.parameters.kicking.strategy.passing.maximum_ready_age
                })
    }

    fn current_outgoing_pass(&self) -> Option<PassIntent> {
        if !self.ball_action_active || !self.has_team_message_budget() {
            return None;
        }
        let BallAction::Pass { receiver } = self.ball_action? else {
            return None;
        };
        if receiver == self.world_state.robot.player_number {
            return None;
        }
        let transform = self.world_state.robot.ground_to_field?;
        let target = transform * self.kick_target?;
        self.valid_pass_target(target)
            .then_some(PassIntent { receiver, target })
    }

    fn valid_pass_target(&self, target: linear_algebra::Point2<coordinate_systems::Field>) -> bool {
        target.x().is_finite()
            && target.y().is_finite()
            && target.x().abs() < self.field_dimensions.length / 2.0
            && target.y().abs() < self.field_dimensions.width / 2.0
    }
}

pub(super) fn state_message_send_interval(
    game_controller_state: &FilteredGameControllerState,
    parameters: &HslNetworkParameters,
) -> Option<Duration> {
    let second_half_duration = match (game_controller_state.game_phase, game_controller_state.half)
    {
        (GamePhase::Normal, Half::First) => parameters.half_duration,
        (GamePhase::Extratime, Half::First) => parameters.extra_half_duration,
        _ => Duration::ZERO,
    };
    let remaining_time_in_game =
        game_controller_state.remaining_time_in_half + second_half_duration;
    let available_messages = game_controller_state
        .remaining_number_of_messages
        .saturating_sub(parameters.remaining_amount_of_messages_to_stop_sending);
    let active_player_count = game_controller_state
        .penalties
        .iter()
        .filter(|(_, penalty)| penalty.is_none())
        .count() as u32;

    if available_messages == 0 || active_player_count == 0 || remaining_time_in_game.is_zero() {
        return None;
    }

    Some(remaining_time_in_game * active_player_count / u32::from(available_messages))
}

fn is_cooldown_elapsed(now: Time, last: Option<Time>, cooldown: Duration) -> bool {
    match last {
        None => true,
        Some(last_time) => now.duration_since(last_time) > cooldown,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hsl_network_messages::{PassReady, PlayerNumber};
    use linear_algebra::point;
    use types::{fall_detection::FallDetection, field_dimensions::FieldDimensions};

    fn blackboard() -> Blackboard {
        let mut b = Blackboard::default();
        b.field_dimensions = FieldDimensions::SPL_2025;
        b.world_state.robot.primary_state = PrimaryState::Playing;
        b.world_state.robot.player_number = PlayerNumber::Two;
        b.world_state.robot.ground_to_field = Some(Default::default());
        b.world_state.filtered_game_controller_state = Some(FilteredGameControllerState {
            game_phase: GamePhase::Normal,
            half: Half::Second,
            remaining_time_in_half: Duration::from_secs(100),
            remaining_number_of_messages: 1000,
            ..Default::default()
        });
        b.parameters
            .network
            .remaining_amount_of_messages_to_stop_sending = 20;
        b.parameters.kicking.strategy.passing.preparation_timeout = Duration::from_secs(6);
        b
    }

    fn announced_receiver(b: &mut Blackboard) {
        b.ball_action_active = true;
        b.ball_action = Some(BallAction::Pass {
            receiver: PlayerNumber::Three,
        });
        b.kick_target = Some(point![1.5, 0.0]);
    }

    #[test]
    fn coordination_uses_normal_cooldown_and_preserves_first_announcement() {
        let mut b = blackboard();
        announced_receiver(&mut b);
        let Some(OutgoingMessage::Hsl(HulkMessage::CoordinatedState(message))) =
            b.try_sending_state_message()
        else {
            panic!("expected pass envelope")
        };
        assert_eq!(message.pass.unwrap().receiver, PlayerNumber::Three);
        let first = b.last_announced_pass.as_ref().unwrap().time;
        b.world_state.now = b.world_state.now + Duration::from_millis(100);
        assert!(b.try_sending_state_message().is_none());
        assert_eq!(b.last_announced_pass.as_ref().unwrap().time, first);
        b.world_state.now = b.world_state.now + Duration::from_secs(1);
        b.kick_target = Some(point![1.6, 0.0]);
        assert!(b.try_sending_state_message().is_some());
        assert_eq!(b.last_announced_pass.as_ref().unwrap().time, first);
        b.world_state.now = b.world_state.now + Duration::from_secs(1);
        b.kick_target = Some(point![1.6, 1.0]);
        assert!(b.try_sending_state_message().is_some());
        assert_eq!(
            b.last_announced_pass.as_ref().unwrap().time,
            b.world_state.now
        );
        b.ball_action_active = false;
        assert!(b.try_sending_state_message().is_none());
        assert!(b.last_announced_pass.is_none());
        b.world_state.now = b.world_state.now + Duration::from_secs(1);
        assert!(matches!(
            b.try_sending_state_message(),
            Some(OutgoingMessage::Hsl(HulkMessage::State(_)))
        ));
    }

    #[test]
    fn exhausted_or_slow_budget_disables_handshake_without_changing_state_schedule() {
        let mut b = blackboard();
        assert!(b.has_team_message_budget());
        b.world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .remaining_number_of_messages = 100;
        assert!(!b.has_team_message_budget());
        // A handshake can fit the six-second timeout while the offer still
        // expires before the peer's next update (normal pose TTL is one second).
        b.world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .remaining_number_of_messages = 250;
        assert!(!b.has_team_message_budget());
        announced_receiver(&mut b);
        assert!(matches!(
            b.try_sending_state_message(),
            Some(OutgoingMessage::Hsl(HulkMessage::State(_)))
        ));
        b.world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .remaining_number_of_messages = 20;
        assert!(!b.has_team_message_budget());
        b.last_sent_hsl_message_time = None;
        assert!(b.try_sending_state_message().is_none());
    }

    #[test]
    fn receiver_ready_uses_envelope_and_invalid_targets_never_announce() {
        let mut b = blackboard();
        b.ready_to_receive_pass = Some(PassReady {
            sender: PlayerNumber::Three,
            target: point![1.5, 0.0],
        });
        let Some(OutgoingMessage::Hsl(HulkMessage::CoordinatedState(message))) =
            b.try_sending_state_message()
        else {
            panic!("expected readiness envelope")
        };
        assert!(message.pass.is_none());
        assert_eq!(message.pass_ready, b.ready_to_receive_pass);
        b.ready_to_receive_pass = None;
        b.last_sent_hsl_message_time = None;
        announced_receiver(&mut b);
        b.kick_target = Some(point![f32::NAN, 0.0]);
        assert!(matches!(
            b.try_sending_state_message(),
            Some(OutgoingMessage::Hsl(HulkMessage::State(_)))
        ));
        assert!(b.last_announced_pass.is_none());
    }

    fn fall_sample(b: &Blackboard, posture: Posture) -> FallDetection {
        FallDetection {
            time: b.world_state.now,
            valid_until: b.world_state.now + Duration::from_secs(5),
            posture,
        }
    }

    #[test]
    fn all_valid_non_upright_postures_advertise_unavailability() {
        for posture in [
            Posture::Falling,
            Posture::Fallen {
                is_ready_for_standup: false,
            },
            Posture::Fallen {
                is_ready_for_standup: true,
            },
            Posture::StandingUp,
        ] {
            let mut b = blackboard();
            b.world_state.fall_detection = Some(fall_sample(&b, posture));
            announced_receiver(&mut b);
            b.ready_to_receive_pass = Some(PassReady {
                sender: PlayerNumber::Four,
                target: point![1.0, 0.0],
            });
            let Some(OutgoingMessage::Hsl(HulkMessage::UnavailableState(state))) =
                b.try_sending_state_message()
            else {
                panic!("non-upright robot must withdraw its ball role: {posture:?}")
            };
            assert_eq!(state.player_number, PlayerNumber::Two);
            assert!(b.last_announced_pass.is_none());
            assert!(b.ready_to_receive_pass.is_none());
            assert!(b.ball_action.is_none());
            assert!(b.kick_target.is_none());
        }
    }

    #[test]
    fn unavailability_cancels_coordination_without_buying_extra_packets_and_recovers() {
        let mut b = blackboard();
        announced_receiver(&mut b);
        assert!(matches!(
            b.try_sending_state_message(),
            Some(OutgoingMessage::Hsl(HulkMessage::CoordinatedState(_)))
        ));
        let sent = b.last_sent_hsl_message_time;
        b.pass_started_at = Some(b.world_state.now);
        b.ready_to_receive_pass = Some(PassReady {
            sender: PlayerNumber::Four,
            target: point![1.0, 0.0],
        });
        b.receiving_pass = Some(crate::kick::passing::ReceivingPass {
            sender: PlayerNumber::Four,
            target: point![1.0, 0.0],
            sender_position: point![2.0, 0.0],
            launched_at: b.world_state.now,
            released_at: None,
        });
        b.world_state.now = b.world_state.now + Duration::from_millis(100);
        b.world_state.fall_detection = Some(fall_sample(&b, Posture::Falling));
        assert!(b.try_sending_state_message().is_none());
        assert_eq!(b.last_sent_hsl_message_time, sent);
        assert!(b.last_announced_pass.is_none());
        assert!(b.pass_started_at.is_none());
        assert!(b.ready_to_receive_pass.is_none());
        assert!(b.receiving_pass.is_none());
        b.world_state.now = b.world_state.now + Duration::from_secs(1);
        assert!(matches!(
            b.try_sending_state_message(),
            Some(OutgoingMessage::Hsl(HulkMessage::UnavailableState(_)))
        ));
        b.world_state.now = b.world_state.now + Duration::from_secs(1);
        b.world_state.fall_detection = Some(fall_sample(&b, Posture::Upright));
        assert!(matches!(
            b.try_sending_state_message(),
            Some(OutgoingMessage::Hsl(HulkMessage::State(_)))
        ));
    }

    #[test]
    fn unknown_invalid_or_upright_posture_does_not_claim_unavailability() {
        for sample in ["missing", "expired", "future", "upright"] {
            let mut b = blackboard();
            b.world_state.now = Time::from_nanos(10_000_000_000);
            let mut fall = fall_sample(&b, Posture::Falling);
            match sample {
                "missing" => (),
                "expired" => {
                    fall.time = b.world_state.now - Duration::from_secs(1);
                    fall.valid_until = b.world_state.now - Duration::from_millis(1);
                    b.world_state.fall_detection = Some(fall);
                }
                "future" => {
                    fall.time = b.world_state.now + Duration::from_millis(1);
                    b.world_state.fall_detection = Some(fall);
                }
                _ => b.world_state.fall_detection = Some(fall_sample(&b, Posture::Upright)),
            }
            assert!(
                matches!(
                    b.try_sending_state_message(),
                    Some(OutgoingMessage::Hsl(HulkMessage::State(_)))
                ),
                "sample {sample}"
            );
        }
    }

    #[test]
    fn unavailability_does_not_bypass_an_exhausted_message_budget() {
        let mut b = blackboard();
        b.world_state.fall_detection = Some(fall_sample(&b, Posture::Falling));
        b.world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .remaining_number_of_messages = 20;
        announced_receiver(&mut b);
        b.pass_started_at = Some(b.world_state.now);
        assert!(b.try_sending_state_message().is_none());
        assert!(b.last_sent_hsl_message_time.is_none());
        assert!(b.ball_action.is_none());
        assert!(b.pass_started_at.is_none());
    }
}
