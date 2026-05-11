use color_eyre::{Result, eyre::Context};
use coordinate_systems::Field;
use linear_algebra::{Orientation2, Pose2, point};
use std::f32::consts::PI;
use std::{
    net::SocketAddr,
    sync::Arc,
    time::{Duration, SystemTime},
};

use booster::FallDownStateType;
use hardware::NetworkInterface;
use hsl_network_messages::{GameControllerReturnMessage, HulkMessage, StateMessage};
use types::{
    cycle_time::CycleTime,
    messages::OutgoingMessage,
    motion_command::MotionCommand,
    parameters::HslNetworkParameters,
    path::PathSegment,
    players::Players,
    world_state::{PlayerState, WorldState},
};

use crate::{behavior::node::Behavior, player_states_receiver::predict_current_pose};

impl Behavior {
    pub fn send_game_controller_return_message(
        &mut self,
        world_state: &WorldState,
        game_controller_address: Option<&SocketAddr>,
        hsl_network_parameters: &HslNetworkParameters,
        hardware: &Arc<impl NetworkInterface>,
    ) -> Result<()> {
        let now = world_state.now;

        if !self.is_return_message_cooldown_elapsed(now, hsl_network_parameters) {
            return Ok(());
        }
        let Some(address) = game_controller_address else {
            return Ok(());
        };

        let ground_to_field = world_state.robot.ground_to_field.unwrap_or_default();

        let ball_position = world_state
            .ball
            .map(|ball| hsl_network_messages::BallPosition {
                age: now.duration_since(ball.last_seen_ball).unwrap(),
                position: ball.ball_in_ground,
            });

        self.last_sent_game_controller_return_message_time = Some(now);

        hardware
            .write_to_network(OutgoingMessage::GameController(
                *address,
                GameControllerReturnMessage {
                    player_number: world_state.robot.player_number,
                    fallen: world_state
                        .fall_down_state
                        .is_some_and(|state| state.fall_down_state != FallDownStateType::IsReady),
                    pose: ground_to_field.as_pose(),
                    ball: ball_position,
                },
            ))
            .wrap_err("failed to write GameControllerReturnMessage to hardware")
    }

    fn is_return_message_cooldown_elapsed(
        &self,
        now: SystemTime,
        hsl_network_parameters: &HslNetworkParameters,
    ) -> bool {
        is_cooldown_elapsed(
            now,
            self.last_sent_game_controller_return_message_time,
            hsl_network_parameters.game_controller_return_message_interval,
        )
    }

    pub fn send_state_message(
        &mut self,
        world_state: &WorldState,
        motion_command: &MotionCommand,
        _player_states: &Players<Option<PlayerState>>,
        cycle_time: &CycleTime,
        hsl_network_parameters: &HslNetworkParameters,
        remaining_amount_of_messages: Option<&u16>,
        hardware: &Arc<impl NetworkInterface>,
    ) -> Result<()> {
        let now = world_state.now;

        if !self.is_state_message_cooldown_elapsed(now, hsl_network_parameters) {
            return Ok(());
        }
        if remaining_amount_of_messages.is_none_or(|remaining_amount_of_messages| {
            *remaining_amount_of_messages
                < hsl_network_parameters.remaining_amount_of_messages_to_stop_sending
        }) {
            return Ok(());
        }

        let ground_to_field = world_state.robot.ground_to_field.unwrap_or_default();

        let pose = ground_to_field.as_pose();
        let target_pose = get_target_pose(motion_command, world_state);

        let ball_position = world_state
            .ball
            .map(|ball| hsl_network_messages::BallPosition {
                age: now.duration_since(ball.last_seen_ball).unwrap(),
                position: ball.ball_in_field,
            });

        let message = HulkMessage::State(StateMessage {
            player_number: world_state.robot.player_number,
            pose,
            target_pose,
            ball_position,
        });

        if !is_message_different(
            &message,
            self.last_sent_hsl_message.as_ref(),
            cycle_time,
            self.last_sent_hsl_message_time,
        ) {
            return Ok(());
        }

        self.last_sent_hsl_message = Some(message);
        self.last_sent_hsl_message_time = Some(now);
        hardware
            .write_to_network(OutgoingMessage::Hsl(message))
            .wrap_err("failed to write StateMessage to hardware")
    }

    fn is_state_message_cooldown_elapsed(
        &self,
        now: SystemTime,
        hsl_network_parameters: &HslNetworkParameters,
    ) -> bool {
        is_cooldown_elapsed(
            now,
            self.last_sent_hsl_message_time,
            hsl_network_parameters.hsl_state_message_send_interval,
        )
    }
}

fn is_cooldown_elapsed(now: SystemTime, last: Option<SystemTime>, cooldown: Duration) -> bool {
    match last {
        None => true,
        Some(last_time) => now.duration_since(last_time).expect("time ran backwards") > cooldown,
    }
}

fn get_target_pose(motion_command: &MotionCommand, world_state: &WorldState) -> Pose2<Field> {
    if let Some(ground_to_field) = &world_state.robot.ground_to_field {
        match motion_command {
            MotionCommand::Prepare | MotionCommand::Stand { .. } | MotionCommand::StandUp => {
                ground_to_field.as_pose()
            }
            MotionCommand::Walk {
                path,
                target_orientation,
                ..
            } => {
                let target_position = match path.last_segment() {
                    PathSegment::LineSegment(line_segment) => line_segment.1,
                    PathSegment::Arc(arc) => arc.circle.point_at_angle(arc.end),
                };
                ground_to_field * Pose2::from_parts(target_position, *target_orientation)
            }
            MotionCommand::WalkWithVelocity {
                velocity,
                angular_velocity,
                ..
            } => {
                const TIMESCALE: f32 = 3.0; //TODO
                ground_to_field
                    * Pose2::from_parts(
                        point![velocity.x() * TIMESCALE, velocity.y() * TIMESCALE],
                        Orientation2::new(angular_velocity * TIMESCALE),
                    )
            }
            MotionCommand::VisualKick {
                ball_position,
                kick_direction,
                ..
            } => ground_to_field * Pose2::from_parts(*ball_position, *kick_direction),
        }
    } else {
        Pose2::default()
    }
}

fn is_message_different(
    message: &HulkMessage,
    last_sent_message: Option<&HulkMessage>,
    cycle_time: &CycleTime,
    last_sent_message_time: Option<SystemTime>,
) -> bool {
    const MAX_POSE_DIFFERENCE: f32 = 0.1;

    let (Some(last_sent_message), Some(last_sent_message_time)) = (last_sent_message, last_sent_message_time) else {
        return true;
    };

    let (HulkMessage::State(message), HulkMessage::State(last_message)) =
        (message, last_sent_message);

    let predicted_pose = predict_current_pose(
        last_message.pose,
        last_message.target_pose,
        last_sent_message_time,
        cycle_time,
    );

    let pose_position_difference = (message.pose.position() - predicted_pose.position()).norm();
    let pose_angle_difference = angular_difference(
        message.pose.orientation().angle(),
        predicted_pose.orientation().angle(),
    );
    let ball_position_difference = match (message.ball_position, last_message.ball_position) {
        (None, None) => 0.0,
        (Some(left), Some(right)) => (left.position - right.position).norm(),
        _ => f32::INFINITY,
    };

    pose_position_difference > MAX_POSE_DIFFERENCE
        || pose_angle_difference > MAX_POSE_DIFFERENCE
        || ball_position_difference > MAX_POSE_DIFFERENCE
}

fn angular_difference(from: f32, to: f32) -> f32 {
    ((from - to + PI).rem_euclid(2.0 * PI) - PI).abs()
}
