use std::time::{Duration, SystemTime};

use bevy::prelude::*;

use hsl_network_messages::{
    GamePhase, GameState, Penalty, PlayerNumber, SubState, Team, TeamColor, TeamState,
};
use types::{
    field_dimensions::GlobalFieldSide, game_controller_state::GameControllerState, players::Players,
};

use crate::whistle::WhistleResource;

const STARTING_PENALTY_DURATION: Duration = Duration::from_secs(45);

#[derive(Resource, Default)]
struct GameControllerControllerState {
    last_state_change: Time,
}

#[derive(Clone, Copy, Message)]
pub enum GameControllerCommand {
    SetGameState(GameState),
    SetGamePhase(GamePhase),
    SetSubState(Option<SubState>, Team, Option<PlayerNumber>),
    SetKickingTeam(Team),
    Goal(Team),
    Penalize(PlayerNumber, Penalty, Team),
    Unpenalize(PlayerNumber, Team),
    BallIsFree,
}

fn game_controller_controller(
    mut commands: MessageReader<GameControllerCommand>,
    mut state: ResMut<GameControllerControllerState>,
    mut game_controller: ResMut<GameController>,
    whistle: ResMut<WhistleResource>,
    time: ResMut<Time>,
) {
    for command in commands.read() {
        match *command {
            GameControllerCommand::SetGameState(game_state) => {
                game_controller.state.game_state = game_state;
                game_controller.state.last_game_state_change =
                    SystemTime::UNIX_EPOCH + time.elapsed();
                state.last_state_change = time.as_generic();
            }
            GameControllerCommand::SetGamePhase(game_phase) => {
                game_controller.state.game_phase = game_phase;
                state.last_state_change = time.as_generic();
            }
            GameControllerCommand::SetKickingTeam(team) => {
                game_controller.state.kicking_team = Some(team);
                state.last_state_change = time.as_generic();
            }
            GameControllerCommand::Goal(team) => {
                match team {
                    Team::Hulks => {
                        game_controller.state.kicking_team = Some(Team::Opponent);
                        &mut game_controller.state.hulks_team
                    }
                    Team::Opponent => {
                        game_controller.state.kicking_team = Some(Team::Hulks);
                        &mut game_controller.state.opponent_team
                    }
                }
                .score += 1;
                game_controller.state.game_state = GameState::Ready;
                game_controller.state.last_game_state_change =
                    SystemTime::UNIX_EPOCH + time.elapsed();
                state.last_state_change = time.as_generic();
            }
            GameControllerCommand::Penalize(player_number, penalty, team) => match team {
                Team::Hulks => game_controller.state.penalties[player_number] = Some(penalty),
                Team::Opponent => {
                    game_controller.state.opponent_penalties[player_number] = Some(penalty)
                }
            },
            GameControllerCommand::Unpenalize(player_number, team) => match team {
                Team::Hulks => game_controller.state.penalties[player_number] = None,
                Team::Opponent => game_controller.state.opponent_penalties[player_number] = None,
            },
            GameControllerCommand::SetSubState(sub_state, team, penalized_player_number) => {
                game_controller.state.sub_state = sub_state;
                game_controller.state.kicking_team = sub_state.map(|_| team);
                match sub_state {
                    Some(SubState::PenaltyKick) => {
                        game_controller.state.game_state = GameState::Ready;
                        game_controller.state.last_game_state_change =
                            SystemTime::UNIX_EPOCH + time.elapsed();
                        match team {
                            Team::Hulks => {
                                game_controller.state.opponent_penalties[penalized_player_number
                                    .expect("this sub state requires a penalized player number.")] =
                                    Some(Penalty::Pushing {
                                        remaining: STARTING_PENALTY_DURATION,
                                    })
                            }
                            Team::Opponent => {
                                game_controller.state.penalties[penalized_player_number
                                    .expect("this sub state requires a penalized player number.")] =
                                    Some(Penalty::Pushing {
                                        remaining: STARTING_PENALTY_DURATION,
                                    })
                            }
                        }
                    }
                    _ => {}
                }
                state.last_state_change = time.as_generic();
            }
            GameControllerCommand::BallIsFree => {
                game_controller.state.sub_state = None;
                game_controller.state.kicking_team = None;
                state.last_state_change = time.as_generic();
            }
        }
    }

    match game_controller.state.game_state {
        GameState::Initial => {
            game_controller.state.game_state = GameState::Ready;
            game_controller.state.last_game_state_change = SystemTime::UNIX_EPOCH + time.elapsed();
            state.last_state_change = time.as_generic();
        }
        GameState::Ready => {
            if time.elapsed_secs() - state.last_state_change.elapsed_secs() > 30.0 {
                game_controller.state.game_state = GameState::Set;
                game_controller.state.last_game_state_change =
                    SystemTime::UNIX_EPOCH + time.elapsed();
                state.last_state_change = time.as_generic();
            }
        }
        GameState::Set => {
            if whistle.last_whistle.is_some_and(|last_whistle| {
                last_whistle >= state.last_state_change.elapsed() && last_whistle <= time.elapsed()
            }) {
                game_controller.state.game_state = GameState::Playing;
                game_controller.state.last_game_state_change =
                    SystemTime::UNIX_EPOCH + time.elapsed();
                state.last_state_change = time.as_generic();
            }
        }
        GameState::Playing => {}
        GameState::Finished => {}
    }

    if game_controller.state.sub_state.is_some()
        && time.elapsed_secs() - state.last_state_change.elapsed_secs() > 30.0
    {
        game_controller.state.sub_state = None;
        game_controller.state.kicking_team = None;
        state.last_state_change = time.as_generic();
    }
}

#[derive(Resource)]
pub struct GameController {
    pub state: GameControllerState,
}

impl Default for GameController {
    fn default() -> Self {
        Self {
            state: GameControllerState {
                game_state: GameState::Initial,
                stopped: false,
                game_phase: GamePhase::Normal,
                remaining_time_in_half: Duration::ZERO,
                kicking_team: Some(Team::Hulks),
                last_game_state_change: SystemTime::UNIX_EPOCH,
                penalties: Players::new(None),
                opponent_penalties: Players::new(None),
                sub_state: None,
                global_field_side: GlobalFieldSide::Home,
                hulks_team: TeamState {
                    team_number: 24,
                    field_player_color: TeamColor::Green,
                    goal_keeper_color: TeamColor::Red,
                    goal_keeper_player_number: Some(PlayerNumber::One),
                    score: 0,
                    penalty_shoot_index: 0,
                    penalty_shoots: Vec::new(),
                    remaining_amount_of_messages: 1200,
                    players: Vec::new(),
                },
                opponent_team: TeamState {
                    team_number: 1,
                    field_player_color: TeamColor::Black,
                    goal_keeper_color: TeamColor::Gray,
                    goal_keeper_player_number: Some(PlayerNumber::One),
                    score: 0,
                    penalty_shoot_index: 0,
                    penalty_shoots: Vec::new(),
                    remaining_amount_of_messages: 1200,
                    players: Vec::new(),
                },
            },
        }
    }
}

pub fn game_controller_plugin(app: &mut App) {
    app.init_resource::<GameControllerControllerState>()
        .add_message::<GameControllerCommand>()
        .add_systems(Update, game_controller_controller);
}

#[cfg(test)]
mod tests {
    use bevy::app::{App, Update};

    use super::*;

    #[test]
    fn set_state_enters_playing_after_whistle_from_previous_tick() {
        let mut time = Time::<()>::default();
        time.advance_by(Duration::from_millis(12));

        let mut app = App::new();
        app.add_message::<GameControllerCommand>()
            .insert_resource(GameControllerControllerState::default())
            .insert_resource(GameController::default())
            .insert_resource(WhistleResource {
                last_whistle: Some(Duration::ZERO),
            })
            .insert_resource(time)
            .add_systems(Update, game_controller_controller);
        app.world_mut()
            .resource_mut::<GameController>()
            .state
            .game_state = GameState::Set;

        app.update();

        assert_eq!(
            app.world().resource::<GameController>().state.game_state,
            GameState::Playing,
        );
    }

    #[test]
    fn set_game_state_updates_public_last_game_state_change_timestamp() {
        let mut time = Time::<()>::default();
        time.advance_by(Duration::from_secs(1));

        let mut app = App::new();
        app.add_message::<GameControllerCommand>()
            .insert_resource(GameControllerControllerState::default())
            .insert_resource(GameController::default())
            .insert_resource(WhistleResource::default())
            .insert_resource(time)
            .add_systems(Update, game_controller_controller);

        app.world_mut()
            .write_message(GameControllerCommand::SetGameState(GameState::Ready));

        app.update();

        assert_eq!(
            app.world()
                .resource::<GameController>()
                .state
                .last_game_state_change,
            SystemTime::UNIX_EPOCH + Duration::from_secs(1),
        );
    }

    #[test]
    fn set_sub_state_preserves_kicking_team_for_non_penalty_set_plays() {
        let mut app = App::new();
        app.add_message::<GameControllerCommand>()
            .insert_resource(GameControllerControllerState::default())
            .insert_resource(GameController::default())
            .insert_resource(WhistleResource::default())
            .insert_resource(Time::<()>::default())
            .add_systems(Update, game_controller_controller);

        app.world_mut()
            .write_message(GameControllerCommand::SetSubState(
                Some(SubState::GoalKick),
                Team::Opponent,
                None,
            ));

        app.update();

        let state = &app.world().resource::<GameController>().state;
        assert_eq!(state.sub_state, Some(SubState::GoalKick));
        assert_eq!(state.kicking_team, Some(Team::Opponent));
    }

    #[test]
    fn ball_is_free_clears_set_play_kicking_team() {
        let mut app = App::new();
        app.add_message::<GameControllerCommand>()
            .insert_resource(GameControllerControllerState::default())
            .insert_resource(GameController::default())
            .insert_resource(WhistleResource::default())
            .insert_resource(Time::<()>::default())
            .add_systems(Update, game_controller_controller);

        app.world_mut()
            .write_message(GameControllerCommand::SetSubState(
                Some(SubState::GoalKick),
                Team::Opponent,
                None,
            ));
        app.update();
        app.world_mut()
            .write_message(GameControllerCommand::BallIsFree);

        app.update();

        let state = &app.world().resource::<GameController>().state;
        assert_eq!(state.sub_state, None);
        assert_eq!(state.kicking_team, None);
    }

    #[test]
    fn set_play_timeout_clears_kicking_team() {
        let mut time = Time::<()>::default();
        time.advance_by(Duration::from_secs(31));

        let mut app = App::new();
        app.add_message::<GameControllerCommand>()
            .insert_resource(GameControllerControllerState::default())
            .insert_resource(GameController::default())
            .insert_resource(WhistleResource::default())
            .insert_resource(time)
            .add_systems(Update, game_controller_controller);
        app.world_mut()
            .resource_mut::<GameController>()
            .state
            .game_state = GameState::Playing;
        app.world_mut()
            .resource_mut::<GameController>()
            .state
            .sub_state = Some(SubState::GoalKick);
        app.world_mut()
            .resource_mut::<GameController>()
            .state
            .kicking_team = Some(Team::Opponent);

        app.update();

        let state = &app.world().resource::<GameController>().state;
        assert_eq!(state.sub_state, None);
        assert_eq!(state.kicking_team, None);
    }
}
