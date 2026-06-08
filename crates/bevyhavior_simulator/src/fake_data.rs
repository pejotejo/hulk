use std::{collections::HashMap, net::SocketAddr, time::SystemTime};

use booster::FallDownState;
use color_eyre::Result;
use serde::{Deserialize, Serialize};

use context_attribute::context;
use coordinate_systems::{Field, Ground};
use framework::MainOutput;
use geometry::{circle::Circle, rectangle::Rectangle};
use hsl_network_messages::{GameState, PlayerNumber, SubState, Team};
use linear_algebra::{Isometry2, Point2, distance, point, vector};
use types::{
    ball_position::{BallPosition, HypotheticalBallPosition},
    cycle_time::CycleTime,
    field_dimensions::Side,
    filtered_game_controller_state::FilteredGameControllerState,
    filtered_game_state::FilteredGameState,
    game_controller_state::GameControllerState,
    obstacles::Obstacle,
    parameters::GameStateFilterParameters,
    players::Players,
    primary_state::PrimaryState,
    rule_obstacles::RuleObstacle,
    world_state::{BallState, RobotState, WorldState},
};

use crate::interfake::FakeDataInterface;

#[derive(Deserialize, Serialize)]
pub struct FakeData {}

#[context]
#[allow(dead_code)]
pub struct CreationContext {
    ball_filter: Parameter<types::parameters::BallFilterParameters, "ball_filter">,
    head_motion: Parameter<types::parameters::HeadMotionParameters, "head_motion">,
    rl_walking: Parameter<types::parameters::RLWalkingParameters, "rl_walking">,
}

#[context]
#[allow(dead_code)]
pub struct CycleContext {
    hardware_interface: HardwareInterface,
    center_circle_ballspace_free_obstacle_radius:
        Parameter<f32, "rule_obstacles.center_circle_ballspace_free_obstacle_radius">,
    center_circle_obstacle_radius_increase:
        Parameter<f32, "rule_obstacles.center_circle_obstacle_radius_increase">,
    field_dimensions: Parameter<types::field_dimensions::FieldDimensions, "field_dimensions">,
    free_kick_obstacle_radius: Parameter<f32, "rule_obstacles.free_kick_obstacle_radius">,
    game_state_filter: Parameter<GameStateFilterParameters, "game_state_filter">,
    penaltykick_box_extension: Parameter<f32, "rule_obstacles.penaltykick_box_extension">,
    player_number: Parameter<PlayerNumber, "player_number">,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    pub cycle_time: MainOutput<CycleTime>,
    pub fall_down_state: MainOutput<Option<FallDownState>>,
    pub filtered_game_controller_state: MainOutput<Option<FilteredGameControllerState>>,
    pub game_controller_address: MainOutput<Option<SocketAddr>>,
    pub game_controller_state: MainOutput<Option<GameControllerState>>,
    pub ground_to_field: MainOutput<Option<Isometry2<Ground, Field>>>,
    pub ball_position: MainOutput<Option<BallPosition<Ground>>>,
    pub hypothetical_ball_positions: MainOutput<Vec<HypotheticalBallPosition<Ground>>>,
    pub obstacles: MainOutput<Vec<Obstacle>>,
    pub primary_state: MainOutput<PrimaryState>,
    pub world_state: MainOutput<WorldState>,
}

impl FakeData {
    pub fn new(_context: CreationContext) -> Result<Self> {
        Ok(Self {})
    }

    pub fn cycle(&mut self, context: CycleContext<impl FakeDataInterface>) -> Result<MainOutputs> {
        let mut receiver = context
            .hardware_interface
            .get_last_database_receiver()
            .lock();
        let last_database = &receiver.borrow_and_mark_as_seen().main_outputs;
        let ball = last_database
            .ball_position
            .and_then(|ball_position| ball_state(ball_position, last_database.ground_to_field));
        let filtered_game_controller_state =
            last_database
                .game_controller_state
                .as_ref()
                .map(|game_controller_state| {
                    filtered_game_controller_state(
                        game_controller_state,
                        *context.player_number,
                        last_database.cycle_time.start_time,
                        context.game_state_filter,
                        ball,
                    )
                });
        let primary_state = last_database
            .game_controller_state
            .as_ref()
            .map(|game_controller_state| {
                primary_state(game_controller_state, *context.player_number)
            })
            .unwrap_or(last_database.primary_state);
        let robot = RobotState {
            ground_to_field: last_database.ground_to_field,
            player_number: *context.player_number,
            primary_state,
        };
        let rule_obstacles = rule_obstacles(
            filtered_game_controller_state.as_ref(),
            ball,
            context.field_dimensions,
            *context.center_circle_ballspace_free_obstacle_radius,
            *context.center_circle_obstacle_radius_increase,
            *context.free_kick_obstacle_radius,
            *context.penaltykick_box_extension,
        );
        let world_state = WorldState {
            ball,
            filtered_game_controller_state: filtered_game_controller_state.clone(),
            hypothetical_ball_positions: last_database.hypothetical_ball_positions.clone(),
            now: last_database.cycle_time.start_time,
            obstacles: last_database.obstacles.clone(),
            player_states: Players::default(),
            position_of_interest: Point2::origin(),
            robot,
            rule_ball: ball,
            rule_obstacles,
            fall_down_state: last_database.fall_down_state,
            suggested_search_position: None,
        };
        Ok(MainOutputs {
            cycle_time: last_database.cycle_time.into(),
            fall_down_state: last_database.fall_down_state.into(),
            filtered_game_controller_state: filtered_game_controller_state.into(),
            game_controller_state: last_database.game_controller_state.clone().into(),
            game_controller_address: last_database.game_controller_address.into(),
            ball_position: last_database.ball_position.into(),
            hypothetical_ball_positions: last_database.hypothetical_ball_positions.clone().into(),
            obstacles: last_database.obstacles.clone().into(),
            ground_to_field: last_database.ground_to_field.into(),
            primary_state: primary_state.into(),
            world_state: world_state.into(),
        })
    }
}

fn ball_state(
    ball_position: BallPosition<Ground>,
    ground_to_field: Option<Isometry2<Ground, Field>>,
) -> Option<BallState> {
    let ground_to_field = ground_to_field?;
    let ball_in_field = ground_to_field * ball_position.position;
    Some(BallState {
        ball_in_ground: ball_position.position,
        ball_in_field,
        ball_in_ground_velocity: ball_position.velocity,
        last_seen_ball: ball_position.last_seen,
        field_side: if ball_in_field.y() >= 0.0 {
            Side::Left
        } else {
            Side::Right
        },
    })
}

fn filtered_game_controller_state(
    game_controller_state: &GameControllerState,
    player_number: PlayerNumber,
    now: SystemTime,
    game_state_filter: &GameStateFilterParameters,
    ball: Option<BallState>,
) -> FilteredGameControllerState {
    FilteredGameControllerState {
        game_state: filtered_game_state(
            game_controller_state,
            Team::Hulks,
            now,
            game_state_filter,
            ball,
        ),
        opponent_game_state: filtered_game_state(
            game_controller_state,
            Team::Opponent,
            now,
            game_state_filter,
            ball,
        ),
        remaining_time_in_half: game_controller_state.remaining_time_in_half,
        game_phase: game_controller_state.game_phase,
        kicking_team: game_controller_state.kicking_team,
        penalties: game_controller_state.penalties.clone(),
        remaining_number_of_messages: game_controller_state
            .hulks_team
            .remaining_amount_of_messages,
        sub_state: game_controller_state.sub_state,
        global_field_side: game_controller_state.global_field_side,
        new_own_penalties_last_cycle: game_controller_state.penalties[player_number]
            .map(|penalty| HashMap::from([(player_number, penalty)]))
            .unwrap_or_default(),
        new_opponent_penalties_last_cycle: HashMap::new(),
    }
}

fn primary_state(
    game_controller_state: &GameControllerState,
    player_number: PlayerNumber,
) -> PrimaryState {
    if game_controller_state.stopped {
        return PrimaryState::Stop;
    }
    if game_controller_state.penalties[player_number].is_some() {
        return PrimaryState::Penalized;
    }
    match game_controller_state.game_state {
        GameState::Initial => PrimaryState::Initial,
        GameState::Ready => PrimaryState::Ready,
        GameState::Set => PrimaryState::Set,
        GameState::Playing => PrimaryState::Playing,
        GameState::Finished => PrimaryState::Finished,
    }
}

fn filtered_game_state(
    game_controller_state: &GameControllerState,
    team: Team,
    now: SystemTime,
    game_state_filter: &GameStateFilterParameters,
    ball: Option<BallState>,
) -> FilteredGameState {
    match game_controller_state.game_state {
        GameState::Initial => FilteredGameState::Initial,
        GameState::Ready => FilteredGameState::Ready,
        GameState::Set => FilteredGameState::Set,
        GameState::Playing => {
            let opponent_is_kicking_team = game_controller_state.kicking_team != Some(team);
            let kick_off_grace_period = game_controller_state.sub_state.is_none()
                && game_controller_state.kicking_team.is_some()
                && now
                    .duration_since(game_controller_state.last_game_state_change)
                    .is_ok_and(|age| {
                        age < game_state_filter.kick_off_grace_period
                            + game_state_filter.game_controller_controller_delay
                    });
            let ball_detected_far_from_kick_off_point = ball.is_some_and(|ball| {
                distance(Point2::origin(), ball.ball_in_field)
                    > game_state_filter.distance_to_consider_ball_moved_in_kick_off
            });
            let opponent_kick_off = opponent_is_kicking_team
                && kick_off_grace_period
                && !ball_detected_far_from_kick_off_point;
            let opponent_sub_state =
                opponent_is_kicking_team && game_controller_state.sub_state.is_some();
            FilteredGameState::Playing {
                ball_is_free: !opponent_kick_off && !opponent_sub_state,
                kick_off: kick_off_grace_period,
            }
        }
        GameState::Finished => FilteredGameState::Finished,
    }
}

fn rule_obstacles(
    filtered_game_controller_state: Option<&FilteredGameControllerState>,
    ball: Option<BallState>,
    field_dimensions: &types::field_dimensions::FieldDimensions,
    center_circle_ballspace_free_obstacle_radius: f32,
    center_circle_obstacle_radius_increase: f32,
    free_kick_obstacle_radius: f32,
    penaltykick_box_extension: f32,
) -> Vec<RuleObstacle> {
    match (filtered_game_controller_state, ball) {
        (
            Some(FilteredGameControllerState {
                game_state: FilteredGameState::Ready,
                sub_state: None,
                kicking_team: Some(Team::Hulks),
                ..
            }),
            _,
        ) => vec![RuleObstacle::Circle(Circle {
            center: point![0.0, 0.0],
            radius: center_circle_ballspace_free_obstacle_radius,
        })],
        (
            Some(FilteredGameControllerState {
                game_state: FilteredGameState::Ready,
                sub_state: None,
                kicking_team: None | Some(Team::Opponent),
                ..
            }),
            _,
        ) => vec![RuleObstacle::Circle(Circle {
            center: point![0.0, 0.0],
            radius: field_dimensions.center_circle_diameter * 0.5
                + center_circle_obstacle_radius_increase,
        })],
        (
            Some(FilteredGameControllerState {
                sub_state:
                    Some(
                        SubState::ThrowIn
                        | SubState::CornerKick
                        | SubState::GoalKick
                        | SubState::PenaltyKick
                        | SubState::DirectFreeKick
                        | SubState::IndirectFreeKick,
                    ),
                kicking_team: Some(Team::Opponent),
                game_state: FilteredGameState::Playing { .. },
                ..
            }),
            Some(ball),
        ) => vec![RuleObstacle::Circle(Circle::new(
            ball.ball_in_field,
            free_kick_obstacle_radius,
        ))],
        (
            Some(FilteredGameControllerState {
                game_state:
                    FilteredGameState::Playing {
                        ball_is_free: false,
                        kick_off: true,
                    },
                ..
            }),
            _,
        ) => vec![
            RuleObstacle::Circle(Circle::new(
                Point2::origin(),
                field_dimensions.center_circle_diameter * 0.5
                    + center_circle_obstacle_radius_increase,
            )),
            RuleObstacle::Rectangle(Rectangle {
                min: point![0.0, -field_dimensions.width / 2.0],
                max: point![field_dimensions.length / 2.0, field_dimensions.width / 2.0],
            }),
        ],
        (
            Some(FilteredGameControllerState {
                sub_state: Some(SubState::PenaltyKick),
                game_state: FilteredGameState::Playing { .. },
                kicking_team,
                ..
            }),
            _,
        ) => match kicking_team {
            Some(Team::Hulks) => vec![create_penalty_box(
                field_dimensions,
                Team::Hulks,
                penaltykick_box_extension,
            )],
            Some(Team::Opponent) => vec![create_penalty_box(
                field_dimensions,
                Team::Opponent,
                penaltykick_box_extension,
            )],
            None => vec![
                create_penalty_box(field_dimensions, Team::Hulks, penaltykick_box_extension),
                create_penalty_box(field_dimensions, Team::Opponent, penaltykick_box_extension),
            ],
        },
        _ => Vec::new(),
    }
}

fn create_penalty_box(
    field_dimensions: &types::field_dimensions::FieldDimensions,
    kicking_team: Team,
    penaltykick_box_extension: f32,
) -> RuleObstacle {
    let side_factor = match kicking_team {
        Team::Hulks => 1.0,
        Team::Opponent => -1.0,
    };
    let half_field_length = field_dimensions.length / 2.0;
    let half_penalty_area_length = field_dimensions.penalty_area_length / 2.0;
    let center_x = side_factor
        * (half_field_length - half_penalty_area_length + penaltykick_box_extension / 2.0);

    RuleObstacle::Rectangle(Rectangle::new_with_center_and_size(
        point![center_x, 0.0],
        vector![
            field_dimensions.penalty_area_length + penaltykick_box_extension,
            field_dimensions.penalty_area_width
        ],
    ))
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use hsl_network_messages::{GamePhase, Penalty, SubState, Team, TeamColor, TeamState};
    use linear_algebra::vector;
    use types::{field_dimensions::GlobalFieldSide, players::Players};

    use super::*;

    #[test]
    fn opponent_substate_marks_ball_not_free_for_hulks() {
        let game_controller_state = GameControllerState {
            game_state: GameState::Playing,
            stopped: false,
            game_phase: GamePhase::Normal,
            remaining_time_in_half: Duration::ZERO,
            kicking_team: Some(Team::Opponent),
            last_game_state_change: std::time::SystemTime::UNIX_EPOCH,
            penalties: Players::new(None::<Penalty>),
            opponent_penalties: Players::new(None::<Penalty>),
            sub_state: Some(SubState::GoalKick),
            global_field_side: GlobalFieldSide::Home,
            hulks_team: team_state(),
            opponent_team: team_state(),
        };

        let filtered = filtered_game_controller_state(
            &game_controller_state,
            PlayerNumber::One,
            SystemTime::UNIX_EPOCH,
            &game_state_filter(),
            None,
        );

        assert_eq!(
            filtered.game_state,
            FilteredGameState::Playing {
                ball_is_free: false,
                kick_off: false,
            }
        );
        assert_eq!(
            filtered.opponent_game_state,
            FilteredGameState::Playing {
                ball_is_free: true,
                kick_off: false,
            }
        );
    }

    #[test]
    fn opponent_kickoff_grace_marks_ball_not_free_for_hulks() {
        let game_controller_state = GameControllerState {
            game_state: GameState::Playing,
            stopped: false,
            game_phase: GamePhase::Normal,
            remaining_time_in_half: Duration::ZERO,
            kicking_team: Some(Team::Opponent),
            last_game_state_change: SystemTime::UNIX_EPOCH,
            penalties: Players::new(None::<Penalty>),
            opponent_penalties: Players::new(None::<Penalty>),
            sub_state: None,
            global_field_side: GlobalFieldSide::Home,
            hulks_team: team_state(),
            opponent_team: team_state(),
        };

        let filtered = filtered_game_controller_state(
            &game_controller_state,
            PlayerNumber::One,
            SystemTime::UNIX_EPOCH + Duration::from_secs(5),
            &game_state_filter(),
            None,
        );

        assert_eq!(
            filtered.game_state,
            FilteredGameState::Playing {
                ball_is_free: false,
                kick_off: true,
            }
        );
        assert_eq!(
            filtered.opponent_game_state,
            FilteredGameState::Playing {
                ball_is_free: true,
                kick_off: true,
            }
        );
    }

    #[test]
    fn opponent_kickoff_grace_marks_ball_free_after_ball_moved() {
        let game_controller_state = GameControllerState {
            game_state: GameState::Playing,
            stopped: false,
            game_phase: GamePhase::Normal,
            remaining_time_in_half: Duration::ZERO,
            kicking_team: Some(Team::Opponent),
            last_game_state_change: SystemTime::UNIX_EPOCH,
            penalties: Players::new(None::<Penalty>),
            opponent_penalties: Players::new(None::<Penalty>),
            sub_state: None,
            global_field_side: GlobalFieldSide::Home,
            hulks_team: team_state(),
            opponent_team: team_state(),
        };
        let ball = BallState {
            ball_in_ground: point![0.4, 0.0],
            ball_in_field: point![0.4, 0.0],
            ball_in_ground_velocity: vector![0.0, 0.0],
            last_seen_ball: SystemTime::UNIX_EPOCH,
            field_side: Side::Left,
        };

        let filtered = filtered_game_controller_state(
            &game_controller_state,
            PlayerNumber::One,
            SystemTime::UNIX_EPOCH + Duration::from_secs(5),
            &game_state_filter(),
            Some(ball),
        );

        assert_eq!(
            filtered.game_state,
            FilteredGameState::Playing {
                ball_is_free: true,
                kick_off: true,
            }
        );
    }

    #[test]
    fn opponent_setplay_adds_rule_obstacle_around_ball() {
        let filtered_game_controller_state = FilteredGameControllerState {
            game_state: FilteredGameState::Playing {
                ball_is_free: false,
                kick_off: false,
            },
            opponent_game_state: FilteredGameState::Playing {
                ball_is_free: true,
                kick_off: false,
            },
            kicking_team: Some(Team::Opponent),
            sub_state: Some(SubState::GoalKick),
            ..Default::default()
        };
        let ball = BallState {
            ball_in_ground: point![1.0, 0.0],
            ball_in_field: point![2.0, 0.5],
            ball_in_ground_velocity: vector![0.0, 0.0],
            last_seen_ball: std::time::SystemTime::UNIX_EPOCH,
            field_side: Side::Left,
        };

        let obstacles = rule_obstacles(
            Some(&filtered_game_controller_state),
            Some(ball),
            &types::field_dimensions::FieldDimensions::SPL_2025,
            0.2,
            0.2,
            0.75,
            0.2,
        );

        assert_eq!(obstacles.len(), 1);
        assert!(obstacles[0].contains(point![2.0, 0.5]));
        assert!(!obstacles[0].contains(point![3.0, 0.5]));
    }

    #[test]
    fn opponent_ready_adds_center_circle_rule_obstacle() {
        let filtered_game_controller_state = FilteredGameControllerState {
            game_state: FilteredGameState::Ready,
            kicking_team: Some(Team::Opponent),
            sub_state: None,
            ..Default::default()
        };

        let obstacles = rule_obstacles(
            Some(&filtered_game_controller_state),
            None,
            &types::field_dimensions::FieldDimensions::SPL_2025,
            0.2,
            0.2,
            0.75,
            0.2,
        );

        assert_eq!(obstacles.len(), 1);
        assert!(obstacles[0].contains(point![0.0, 0.0]));
        assert!(!obstacles[0].contains(point![1.0, 0.0]));
    }

    #[test]
    fn opponent_live_kickoff_adds_center_circle_and_half_field_rule_obstacles() {
        let filtered_game_controller_state = FilteredGameControllerState {
            game_state: FilteredGameState::Playing {
                ball_is_free: false,
                kick_off: true,
            },
            kicking_team: Some(Team::Opponent),
            sub_state: None,
            ..Default::default()
        };
        let field_dimensions = types::field_dimensions::FieldDimensions::SPL_2025;

        let obstacles = rule_obstacles(
            Some(&filtered_game_controller_state),
            None,
            &field_dimensions,
            0.2,
            0.2,
            0.75,
            0.2,
        );

        assert_eq!(obstacles.len(), 2);
        assert!(obstacles[0].contains(point![0.0, 0.0]));
        assert!(obstacles[1].contains(point![field_dimensions.length / 4.0, 0.0]));
        assert!(!obstacles[1].contains(point![-field_dimensions.length / 4.0, 0.0]));
    }

    #[test]
    fn own_penalty_kick_adds_penalty_box_rule_obstacle() {
        let filtered_game_controller_state = FilteredGameControllerState {
            game_state: FilteredGameState::Playing {
                ball_is_free: true,
                kick_off: false,
            },
            kicking_team: Some(Team::Hulks),
            sub_state: Some(SubState::PenaltyKick),
            ..Default::default()
        };
        let field_dimensions = types::field_dimensions::FieldDimensions::SPL_2025;

        let obstacles = rule_obstacles(
            Some(&filtered_game_controller_state),
            None,
            &field_dimensions,
            0.2,
            0.2,
            0.75,
            0.2,
        );

        assert_eq!(obstacles.len(), 1);
        assert!(obstacles[0].contains(point![field_dimensions.length / 2.0 - 0.1, 0.0]));
        assert!(!obstacles[0].contains(point![-field_dimensions.length / 2.0 + 0.1, 0.0]));
    }

    #[test]
    fn hulks_ready_adds_small_ballspace_rule_obstacle() {
        let filtered_game_controller_state = FilteredGameControllerState {
            game_state: FilteredGameState::Ready,
            kicking_team: Some(Team::Hulks),
            sub_state: None,
            ..Default::default()
        };

        let obstacles = rule_obstacles(
            Some(&filtered_game_controller_state),
            None,
            &types::field_dimensions::FieldDimensions::SPL_2025,
            0.2,
            0.2,
            0.75,
            0.2,
        );

        assert_eq!(obstacles.len(), 1);
        assert!(obstacles[0].contains(point![0.0, 0.0]));
        assert!(!obstacles[0].contains(point![0.3, 0.0]));
    }

    fn team_state() -> TeamState {
        TeamState {
            team_number: 0,
            field_player_color: TeamColor::Green,
            goal_keeper_color: TeamColor::Red,
            goal_keeper_player_number: Some(PlayerNumber::One),
            score: 0,
            penalty_shoot_index: 0,
            penalty_shoots: Vec::new(),
            remaining_amount_of_messages: 1200,
            players: Vec::new(),
        }
    }

    fn game_state_filter() -> GameStateFilterParameters {
        GameStateFilterParameters {
            game_controller_controller_delay: Duration::from_secs(3),
            kick_off_grace_period: Duration::from_secs(10),
            distance_to_consider_ball_moved_in_kick_off: 0.3,
            ..Default::default()
        }
    }
}
