use std::{net::SocketAddr, time::Duration};

use hsl_network_messages::{GameControllerReturnMessage, HulkMessage, PassIntent, StateMessage};
use ros_z::time::Time;
use types::{
    fall_detection::Posture, messages::OutgoingMessage, parameters::HslNetworkParameters,
    primary_state::PrimaryState, time_wrapper::TimeWrapper,
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
        if self.world_state.robot.primary_state != PrimaryState::Playing {
            return None;
        }

        let now = self.world_state.now;
        let network_parameters = &self.parameters.network;

        if !self.is_state_message_cooldown_elapsed(now, network_parameters) {
            return None;
        }
        if !self.has_team_message_budget() {
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

            let pass = match (self.ball_action, self.kick_target) {
                (Some(BallAction::Pass { receiver }), Some(target)) => Some(PassIntent {
                    receiver,
                    target: ground_to_field * target,
                }),
                _ => None,
            };
            let announcement_time = pass
                .filter(|pass| matches_announced_pass(self, *pass))
                .and_then(|_| self.last_announced_pass.as_ref().map(|pass| pass.time))
                .unwrap_or(now);
            self.last_announced_pass = pass.map(|inner| TimeWrapper {
                time: announcement_time,
                inner,
            });
            let message = HulkMessage::State(StateMessage {
                player_number: self.world_state.robot.player_number,
                pose,
                head_yaw: self.head_yaw,
                ball_position,
                pass,
            });

            self.last_sent_hsl_message_time = Some(now);
            Some(OutgoingMessage::Hsl(message))
        } else {
            None
        }
    }

    pub(crate) fn has_team_message_budget(&self) -> bool {
        self.world_state
            .filtered_game_controller_state
            .as_ref()
            .is_some_and(|state| {
                state.remaining_number_of_messages > 0
                    && state.remaining_number_of_messages
                        >= self
                            .parameters
                            .network
                            .remaining_amount_of_messages_to_stop_sending
            })
    }

    fn is_state_message_cooldown_elapsed(
        &self,
        now: Time,
        hsl_network_parameters: &HslNetworkParameters,
    ) -> bool {
        is_cooldown_elapsed(
            now,
            self.last_sent_hsl_message_time,
            hsl_network_parameters.hsl_state_message_send_interval,
        )
    }
}

fn is_cooldown_elapsed(now: Time, last: Option<Time>, cooldown: Duration) -> bool {
    match last {
        None => true,
        Some(last_time) => now.duration_since(last_time) > cooldown,
    }
}
