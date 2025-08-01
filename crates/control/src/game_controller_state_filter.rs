use std::{
    collections::HashMap,
    time::{Duration, SystemTime},
};

use color_eyre::Result;
use context_attribute::context;
use coordinate_systems::{Field, Ground};
use framework::{AdditionalOutput, MainOutput};
use linear_algebra::{distance, Isometry2, Point2, Vector2};
use serde::{Deserialize, Serialize};
use spl_network_messages::{GamePhase, GameState, Penalty, PlayerNumber, SubState, Team};
use types::{
    ball_position::BallPosition,
    cycle_time::CycleTime,
    field_dimensions::FieldDimensions,
    filtered_game_controller_state::FilteredGameControllerState,
    filtered_game_state::FilteredGameState,
    filtered_whistle::FilteredWhistle,
    game_controller_state::GameControllerState,
    parameters::GameStateFilterParameters,
    players::Players,
    world_state::{BallState, LastBallState},
};

#[derive(Deserialize, Serialize)]
pub struct GameControllerStateFilter {
    state: State,
    opponent_state: State,
    last_game_controller_state: Option<GameControllerState>,
    whistle_in_set_ball_position: Option<Point2<Field>>,
    last_observed_ball: Option<(SystemTime, BallState)>,
    last_time_hulk_was_penalized: Option<SystemTime>,
    last_time_opponent_was_penalized: Option<SystemTime>,
}

#[context]
pub struct CreationContext {}

#[context]
pub struct CycleContext {
    ball_position: Input<Option<BallPosition<Ground>>, "ball_position?">,
    cycle_time: Input<CycleTime, "cycle_time">,
    filtered_whistle: Input<FilteredWhistle, "filtered_whistle">,
    visual_referee_proceed_to_ready: Input<bool, "visual_referee_proceed_to_ready">,
    detected_free_kick_kicking_team: Input<Option<Team>, "detected_free_kick_kicking_team?">,
    game_controller_state: RequiredInput<Option<GameControllerState>, "game_controller_state?">,
    config: Parameter<GameStateFilterParameters, "game_state_filter">,
    field_dimensions: Parameter<FieldDimensions, "field_dimensions">,
    player_number: Parameter<PlayerNumber, "player_number">,

    ground_to_field: CyclerState<Option<Isometry2<Ground, Field>>, "ground_to_field">,
    last_ball_state: CyclerState<Option<LastBallState>, "last_ball_state">,

    whistle_in_set_ball_position:
        AdditionalOutput<Option<Point2<Field>>, "whistle_in_set_ball_position">,
}

#[context]
pub struct MainOutputs {
    pub filtered_game_controller_state: MainOutput<Option<FilteredGameControllerState>>,
}

impl GameControllerStateFilter {
    pub fn new(_context: CreationContext) -> Result<Self> {
        Ok(Self {
            last_game_controller_state: None,
            state: State::Initial,
            opponent_state: State::Initial,
            whistle_in_set_ball_position: None,
            last_observed_ball: Default::default(),
            last_time_hulk_was_penalized: Default::default(),
            last_time_opponent_was_penalized: Default::default(),
        })
    }

    pub fn cycle(&mut self, mut context: CycleContext) -> Result<MainOutputs> {
        let (new_own_penalties_last_cycle, new_opponent_penalties_last_cycle) = self
            .last_game_controller_state
            .as_ref()
            .map(|last| {
                (
                    penalty_diff(last.penalties, context.game_controller_state.penalties),
                    penalty_diff(
                        last.opponent_penalties,
                        context.game_controller_state.opponent_penalties,
                    ),
                )
            })
            .unwrap_or_default();

        let did_receive_motion_in_set_penalty = new_own_penalties_last_cycle
            .iter()
            .chain(new_opponent_penalties_last_cycle.iter())
            .any(|(_, penalty)| matches!(penalty, Penalty::IllegalMotionInSet { .. }));

        let kicking_team = self.find_kicking_team(
            &context,
            &new_own_penalties_last_cycle,
            &new_opponent_penalties_last_cycle,
            context.detected_free_kick_kicking_team.copied(),
        );

        let game_states = self.filter_game_states(
            *context.ground_to_field,
            context.ball_position,
            context.field_dimensions,
            context.config,
            context.game_controller_state,
            context.filtered_whistle,
            context.cycle_time,
            *context.visual_referee_proceed_to_ready,
            *context.player_number,
            did_receive_motion_in_set_penalty,
            kicking_team,
        );

        let filtered_game_controller_state = FilteredGameControllerState {
            game_state: game_states.own,
            opponent_game_state: game_states.opponent,
            remaining_time_in_half: context.game_controller_state.remaining_time_in_half,
            game_phase: context.game_controller_state.game_phase,
            kicking_team,
            penalties: context.game_controller_state.penalties,
            remaining_number_of_messages: context
                .game_controller_state
                .hulks_team
                .remaining_amount_of_messages,
            sub_state: context.game_controller_state.sub_state,
            global_field_side: context.game_controller_state.global_field_side,
            new_own_penalties_last_cycle,
            new_opponent_penalties_last_cycle,
        };
        context
            .whistle_in_set_ball_position
            .fill_if_subscribed(|| self.whistle_in_set_ball_position);

        self.last_game_controller_state = Some(context.game_controller_state.clone());
        Ok(MainOutputs {
            filtered_game_controller_state: Some(filtered_game_controller_state).into(),
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn filter_game_states(
        &mut self,
        ground_to_field: Option<Isometry2<Ground, Field>>,
        ball_position: Option<&BallPosition<Ground>>,
        field_dimensions: &FieldDimensions,
        config: &GameStateFilterParameters,
        game_controller_state: &GameControllerState,
        filtered_whistle: &FilteredWhistle,
        cycle_time: &CycleTime,
        visual_referee_proceed_to_ready: bool,
        player_number: PlayerNumber,
        did_receive_motion_in_set_penalty: bool,
        filtered_kicking_team: Option<Team>,
    ) -> FilteredGameStates {
        let ball_detected_far_from_any_goal = ground_to_field.is_some_and(|ground_to_field| {
            ball_detected_far_from_any_goal(
                ground_to_field,
                ball_position,
                field_dimensions,
                config.whistle_acceptance_goal_distance,
            )
        });
        self.state = next_filtered_state(
            self.state,
            game_controller_state,
            filtered_whistle.is_detected,
            cycle_time.start_time,
            config,
            ball_detected_far_from_any_goal,
            visual_referee_proceed_to_ready,
            did_receive_motion_in_set_penalty,
        );
        self.opponent_state = next_filtered_state(
            self.opponent_state,
            game_controller_state,
            filtered_whistle.is_detected,
            cycle_time.start_time,
            config,
            ball_detected_far_from_any_goal,
            visual_referee_proceed_to_ready,
            did_receive_motion_in_set_penalty,
        );

        if let (State::WhistleInSet { .. }, Some(ground_to_field), None) = (
            self.state,
            ground_to_field,
            self.whistle_in_set_ball_position,
        ) {
            ball_position.map(|ball| ground_to_field * ball.position);
        };

        let motion_in_set = matches!(
            game_controller_state.penalties[player_number],
            Some(Penalty::IllegalMotionInSet { .. })
        );
        if matches!(self.state, State::Playing) || motion_in_set {
            self.whistle_in_set_ball_position = None;
        }

        let ball_detected_far_from_kick_off_point = ground_to_field
            .zip(ball_position)
            .map(|(ground_to_field, ball)| {
                let absolute_ball_position = ground_to_field * ball.position;
                let reference_ball_position = self.whistle_in_set_ball_position.unwrap_or_default();
                distance(reference_ball_position, absolute_ball_position)
                    > config.distance_to_consider_ball_moved_in_kick_off
            })
            .unwrap_or(false);

        let filtered_game_state = self.state.construct_filtered_game_state_for_team(
            game_controller_state,
            Team::Hulks,
            cycle_time.start_time,
            ball_detected_far_from_kick_off_point,
            config,
            visual_referee_proceed_to_ready,
            filtered_kicking_team,
        );

        let filtered_opponent_game_state =
            self.opponent_state.construct_filtered_game_state_for_team(
                game_controller_state,
                Team::Opponent,
                cycle_time.start_time,
                ball_detected_far_from_kick_off_point,
                config,
                visual_referee_proceed_to_ready,
                filtered_kicking_team,
            );

        FilteredGameStates {
            own: filtered_game_state,
            opponent: filtered_opponent_game_state,
        }
    }

    fn find_kicking_team(
        &mut self,
        context: &CycleContext,
        new_own_penalties_last_cycle: &HashMap<PlayerNumber, Penalty>,
        new_opponent_penalties_last_cycle: &HashMap<PlayerNumber, Penalty>,
        detected_free_kick_kicking_team: Option<Team>,
    ) -> Option<Team> {
        let game_controller_state = context.game_controller_state;

        if let Some(kicking_team) = game_controller_state.kicking_team {
            return Some(kicking_team);
        }

        if let Some(LastBallState { time, ball }) = *context.last_ball_state {
            self.last_observed_ball = Some((time, ball));
        };

        let ball_is_in_opponent_half =
            self.last_observed_ball
                .map(|(last_observed_ball_time, last_observed_ball)| {
                    let is_not_in_penalty_kick =
                        game_controller_state.sub_state != Some(SubState::PenaltyKick);

                    if is_not_in_penalty_kick
                        && context
                            .cycle_time
                            .start_time
                            .duration_since(last_observed_ball_time)
                            .expect("time ran backwards")
                            > context.config.duration_to_keep_observed_ball
                    {
                        self.last_observed_ball = None;
                    }
                    last_observed_ball.ball_in_field.x().is_sign_positive()
                });

        if !new_own_penalties_last_cycle.is_empty() {
            self.last_time_hulk_was_penalized = Some(context.cycle_time.start_time);
        }

        if self
            .last_time_hulk_was_penalized
            .is_some_and(|last_time_hulk_was_penalized| {
                context
                    .cycle_time
                    .start_time
                    .duration_since(last_time_hulk_was_penalized)
                    .expect("time ran backwards")
                    > context.config.duration_to_keep_new_penalties
            })
        {
            self.last_time_hulk_was_penalized = None;
        }

        if !new_opponent_penalties_last_cycle.is_empty() {
            self.last_time_opponent_was_penalized = Some(context.cycle_time.start_time);
        }

        if self
            .last_time_opponent_was_penalized
            .is_some_and(|last_time_opponent_was_penalized| {
                context
                    .cycle_time
                    .start_time
                    .duration_since(last_time_opponent_was_penalized)
                    .expect("time ran backwards")
                    > context.config.duration_to_keep_new_penalties
            })
        {
            self.last_time_opponent_was_penalized = None;
        }

        match game_controller_state {
            GameControllerState {
                sub_state: Some(SubState::CornerKick),
                ..
            } if ball_is_in_opponent_half? => Some(Team::Hulks),
            GameControllerState {
                sub_state: Some(SubState::CornerKick),
                ..
            } if !ball_is_in_opponent_half? => Some(Team::Opponent),
            GameControllerState {
                sub_state: Some(SubState::GoalKick),
                ..
            } if ball_is_in_opponent_half? => Some(Team::Opponent),
            GameControllerState {
                sub_state: Some(SubState::GoalKick),
                ..
            } if !ball_is_in_opponent_half? => Some(Team::Hulks),
            GameControllerState {
                sub_state: Some(SubState::PenaltyKick),
                ..
            } if ball_is_in_opponent_half? => Some(Team::Hulks),
            GameControllerState {
                sub_state: Some(SubState::PenaltyKick),
                ..
            } if !ball_is_in_opponent_half? => Some(Team::Opponent),
            GameControllerState {
                sub_state: Some(SubState::PushingFreeKick),
                ..
            } if self.last_time_hulk_was_penalized.is_some() => Some(Team::Opponent),
            GameControllerState {
                sub_state: Some(SubState::PushingFreeKick),
                ..
            } if self.last_time_opponent_was_penalized.is_some() => Some(Team::Hulks),
            GameControllerState {
                sub_state: Some(SubState::KickIn),
                ..
            } if detected_free_kick_kicking_team.is_some() => detected_free_kick_kicking_team,
            GameControllerState {
                game_state: GameState::Playing,
                sub_state: None,
                ..
            } => match (
                context.filtered_whistle.is_detected,
                ball_is_in_opponent_half?,
            ) {
                (true, false) => Some(Team::Opponent),
                (true, true) => Some(Team::Hulks),
                _ => None,
            },
            _ => None,
        }
    }
}

struct FilteredGameStates {
    own: FilteredGameState,
    opponent: FilteredGameState,
}

#[allow(clippy::too_many_arguments)]
fn next_filtered_state(
    current_state: State,
    game_controller_state: &GameControllerState,
    is_whistle_detected: bool,
    cycle_start_time: SystemTime,
    config: &GameStateFilterParameters,
    ball_detected_far_from_any_goal: bool,
    visual_referee_proceed_to_ready: bool,
    did_receive_motion_in_set_penalty: bool,
) -> State {
    match (current_state, game_controller_state.game_state) {
        (State::Finished, GameState::Initial) => State::Initial,
        (State::Finished, _) => match game_controller_state.game_phase {
            GamePhase::PenaltyShootout { .. } => State::Set,
            _ => State::Finished,
        },
        (
            State::TentativeFinished {
                time_when_finished_clicked,
            },
            GameState::Finished,
        ) if cycle_start_time
            .duration_since(time_when_finished_clicked)
            .unwrap()
            >= config.tentative_finish_duration =>
        {
            State::Finished
        }
        (
            State::TentativeFinished {
                time_when_finished_clicked,
            },
            GameState::Finished,
        ) => State::TentativeFinished {
            time_when_finished_clicked,
        },
        (State::TentativeFinished { .. }, game_state) => State::from_game_state(game_state),
        (_, GameState::Finished) => State::TentativeFinished {
            time_when_finished_clicked: cycle_start_time,
        },
        (State::Standby, GameState::Standby) => {
            if visual_referee_proceed_to_ready {
                State::Ready
            } else {
                State::Standby
            }
        }

        (State::Ready, GameState::Standby) => State::Ready,

        (State::Initial | State::Ready | State::Standby, _)
        | (
            State::Set,
            GameState::Initial | GameState::Ready | GameState::Playing | GameState::Standby,
        )
        | (
            State::WhistleInSet { .. },
            GameState::Initial | GameState::Ready | GameState::Playing | GameState::Standby,
        )
        | (
            State::Playing,
            GameState::Initial | GameState::Ready | GameState::Set | GameState::Standby,
        )
        | (
            State::WhistleInPlaying { .. },
            GameState::Initial | GameState::Ready | GameState::Set | GameState::Standby,
        ) => State::from_game_state(game_controller_state.game_state),
        (State::Set, GameState::Set) => {
            if is_whistle_detected {
                State::WhistleInSet {
                    time_when_whistle_was_detected: cycle_start_time,
                }
            } else {
                State::Set
            }
        }
        (State::WhistleInSet { .. }, GameState::Set) if did_receive_motion_in_set_penalty => {
            State::Set
        }
        (
            State::WhistleInSet {
                time_when_whistle_was_detected,
            },
            GameState::Set,
        ) => {
            if cycle_start_time
                .duration_since(time_when_whistle_was_detected)
                .unwrap()
                < config.playing_message_delay + config.game_controller_controller_delay
            {
                State::WhistleInSet {
                    time_when_whistle_was_detected,
                }
            } else {
                State::Playing
            }
        }
        (State::Playing, GameState::Playing) => {
            if is_whistle_detected && !ball_detected_far_from_any_goal {
                State::WhistleInPlaying {
                    time_when_whistle_was_detected: cycle_start_time,
                }
            } else {
                State::Playing
            }
        }
        (
            State::WhistleInPlaying {
                time_when_whistle_was_detected,
            },
            GameState::Playing,
        ) => {
            if cycle_start_time
                .duration_since(time_when_whistle_was_detected)
                .unwrap()
                < config.ready_message_delay + config.game_controller_controller_delay
            {
                State::WhistleInPlaying {
                    time_when_whistle_was_detected,
                }
            } else {
                State::Playing
            }
        }
    }
}

fn ball_detected_far_from_any_goal(
    ground_to_field: Isometry2<Ground, Field>,
    ball: Option<&BallPosition<Ground>>,
    field_dimensions: &FieldDimensions,
    whistle_acceptance_goal_distance: Vector2<Field>,
) -> bool {
    match ball {
        Some(ball) => {
            let ball_on_field = ground_to_field * ball.position;
            ball_on_field.x().abs()
                < field_dimensions.length / 2.0 - whistle_acceptance_goal_distance.x()
                || ball_on_field.y().abs()
                    > field_dimensions.goal_inner_width / 2.0 + whistle_acceptance_goal_distance.y()
        }
        None => false,
    }
}

fn is_in_grace_period(
    cycle_start_time: SystemTime,
    start_time: SystemTime,
    grace_period: Duration,
) -> bool {
    cycle_start_time
        .duration_since(start_time)
        .expect("Time ran backwards")
        < grace_period
}

#[derive(Clone, Copy, Deserialize, Serialize)]
pub enum State {
    Initial,
    Ready,
    Set,
    WhistleInSet {
        time_when_whistle_was_detected: SystemTime,
    },
    Playing,
    WhistleInPlaying {
        time_when_whistle_was_detected: SystemTime,
    },
    TentativeFinished {
        time_when_finished_clicked: SystemTime,
    },
    Finished,
    Standby,
}

impl State {
    fn from_game_state(game_state: GameState) -> Self {
        match game_state {
            GameState::Initial => State::Initial,
            GameState::Ready => State::Ready,
            GameState::Set => State::Set,
            GameState::Playing => State::Playing,
            GameState::Finished => State::Finished,
            GameState::Standby => State::Standby,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn construct_filtered_game_state_for_team(
        &self,
        game_controller_state: &GameControllerState,
        team: Team,
        cycle_start_time: SystemTime,
        ball_detected_far_from_kick_off_point: bool,
        config: &GameStateFilterParameters,
        visual_referee_proceed_to_ready: bool,
        filtered_kicking_team: Option<Team>,
    ) -> FilteredGameState {
        let is_in_sub_state = game_controller_state.sub_state.is_some();
        let opponent_is_kicking_team = filtered_kicking_team != Some(team);

        match self {
            State::Initial => FilteredGameState::Initial,
            State::Standby => {
                if visual_referee_proceed_to_ready {
                    FilteredGameState::Ready
                } else {
                    FilteredGameState::Standby
                }
            }
            State::Ready => FilteredGameState::Ready,
            State::Set => FilteredGameState::Set,
            State::WhistleInSet {
                time_when_whistle_was_detected,
            } => {
                let kick_off_grace_period = is_in_grace_period(
                    cycle_start_time,
                    *time_when_whistle_was_detected,
                    config.kick_off_grace_period + config.game_controller_controller_delay,
                );
                let opponent_kick_off = opponent_is_kicking_team
                    && kick_off_grace_period
                    && !ball_detected_far_from_kick_off_point;
                let opponent_sub_state = opponent_is_kicking_team && is_in_sub_state;

                FilteredGameState::Playing {
                    ball_is_free: !opponent_kick_off && !opponent_sub_state,
                    kick_off: !is_in_sub_state,
                }
            }
            State::Playing => FilteredGameState::Playing {
                ball_is_free: !(is_in_sub_state && opponent_is_kicking_team),
                kick_off: false,
            },
            State::WhistleInPlaying { .. } => FilteredGameState::Ready,
            State::Finished => match game_controller_state.game_phase {
                GamePhase::PenaltyShootout { .. } => FilteredGameState::Set,
                _ => FilteredGameState::Finished,
            },
            // is hack @schluis
            State::TentativeFinished { .. } => FilteredGameState::Set,
        }
    }
}

fn penalty_diff(
    last: Players<Option<Penalty>>,
    current: Players<Option<Penalty>>,
) -> HashMap<PlayerNumber, Penalty> {
    let current_penalties = current
        .iter()
        .fold(HashMap::new(), |mut map, (player, penalty)| {
            if let Some(penalty) = penalty {
                map.insert(player, *penalty);
            }
            map
        });
    last.iter()
        .fold(current_penalties, |mut map, (player, penalty)| {
            if penalty.is_some() {
                map.remove(&player);
            }
            map
        })
}
