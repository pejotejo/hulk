use std::{f32::consts::PI, net::SocketAddr, time::SystemTime};

use booster::FallDownState;
use color_eyre::Result;
use context_attribute::context;
use coordinate_systems::{Field, Ground};
use framework::{MainOutput, PerceptionInput};
use hardware::NetworkInterface;
use hsl_network_messages::{HulkMessage, PlayerNumber};
use linear_algebra::{Isometry2, Orientation2, Pose2, Vector2};
use serde::{Deserialize, Serialize};
use types::filtered_game_controller_state::FilteredGameControllerState;
use types::{
    ball_position::BallPosition, cycle_time::CycleTime, messages::IncomingMessage,
    parameters::HslNetworkParameters, players::Players, world_state::PlayerState,
};

#[derive(Serialize, Deserialize)]
pub struct PlayerStatesReceiver {
    pub last_player_states: Players<Option<PlayerState>>,
}

#[context]
pub struct CreationContext {}

#[context]
pub struct CycleContext {
    game_controller_address: Input<Option<SocketAddr>, "game_controller_address?">,
    cycle_time: Input<CycleTime, "cycle_time">,
    ground_to_field: Input<Option<Isometry2<Ground, Field>>, "ground_to_field?">,
    ball_position: Input<Option<BallPosition<Ground>>, "ball_position?">,
    game_controller_state:
        Input<Option<FilteredGameControllerState>, "filtered_game_controller_state?">,

    fall_down_state: PerceptionInput<Option<FallDownState>, "FallDownState", "fall_down_state?">,
    network_message: PerceptionInput<Option<IncomingMessage>, "HslNetwork", "filtered_message?">,

    player_number: Parameter<PlayerNumber, "player_number">,
    hsl_network_parameters: Parameter<HslNetworkParameters, "hsl_network">,

    hardware: HardwareInterface,
}

#[context]
pub struct MainOutputs {
    pub player_states: MainOutput<Players<Option<PlayerState>>>,
}

impl PlayerStatesReceiver {
    pub fn new(_context: CreationContext) -> Result<Self> {
        Ok(Self {
            last_player_states: Players {
                one: None,
                two: None,
                three: None,
                four: None,
                five: None,
            },
        })
    }

    pub fn cycle(&mut self, context: CycleContext<impl NetworkInterface>) -> Result<MainOutputs> {
        if let Some(game_controller_state) = context.game_controller_state {
            let penaltys = game_controller_state.penalties;
            for (player_number, penalty) in penaltys.iter() {
                if penalty.is_some() {
                    self.last_player_states[player_number] = None;
                }
            }
        }

        let messages = context
            .network_message
            .persistent
            .values()
            .flat_map(|messages| messages.iter().filter_map(|message| *message))
            .filter_map(|message| match message {
                IncomingMessage::Hsl(message) => Some(*message),
                _ => None,
            });

        let mut player_states = self.last_player_states;
        for message in messages {
            match message {
                HulkMessage::State(state_message) => {
                    player_states[state_message.player_number] = Some(PlayerState {
                        last_received_pose: state_message.pose,
                        pose: state_message.pose,
                        target_pose: state_message.target_pose,
                        last_updated: *context.cycle_time,
                        ball_position: state_message.ball_position.map(|ball| BallPosition::<
                            Field,
                        > {
                            position: ball.position,
                            velocity: Vector2::zeros(),
                            last_seen: context.cycle_time.start_time - ball.age,
                        }),
                    });
                }
            }
        }

        for (_, player_state) in player_states.iter_mut() {
            if let Some(state) = player_state {
                state.pose = predict_current_pose(
                    state.last_received_pose,
                    state.target_pose,
                    state.last_updated.start_time,
                    context.cycle_time,
                );
            }
        }

        self.last_player_states = player_states;

        Ok(MainOutputs {
            player_states: player_states.into(),
        })
    }
}

pub fn predict_current_pose(
    start: Pose2<Field>,
    target: Pose2<Field>,
    last_updated: SystemTime,
    cycle_time: &CycleTime,
) -> Pose2<Field> {
    const WALK_SPEED: f32 = 0.25; //TODO

    let elapsed = cycle_time
        .start_time
        .duration_since(last_updated)
        .unwrap_or_default()
        .as_secs_f32();

    let delta = target.position() - start.position();
    let distance = delta.norm();

    if distance <= f32::EPSILON {
        return target;
    }

    let travel_time = distance / WALK_SPEED;
    let progress = elapsed / travel_time;
    if progress >= 1.0 {
        target
    } else {
        let new_position = start.position() + delta * progress;
        let new_angle = start.orientation().angle()
            + shortest_angular_difference(
                start.orientation().angle(),
                target.orientation().angle(),
            ) * progress;

        Pose2::from_parts(new_position, Orientation2::new(new_angle))
    }
}

fn shortest_angular_difference(from: f32, to: f32) -> f32 {
    (to - from + PI).rem_euclid(2.0 * PI) - PI
}
