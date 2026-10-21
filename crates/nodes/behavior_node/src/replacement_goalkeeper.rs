use std::time::Duration;

use coordinate_systems::Field;
use hsl_network_messages::{GamePhase, PlayerNumber, SubState};
use linear_algebra::{Point2, point};
use ros_z::time::Time;
use serde::{Deserialize, Serialize};
use types::{
    behavior_tree::Status, fall_detection::Posture, filtered_game_state::FilteredGameState,
    motion_type::MotionType, players::Players, primary_state::PrimaryState,
};

use crate::{
    action,
    actions::stand,
    behavior_tree::Node,
    conditions::is_finishing_striker_kick,
    goalkeeper::{goalkeeper_shot_block_position_subtree, walk_to_goalkeeper_default_position},
    head::look_at_ball_subtree,
    kick::passing::is_pass_receiver,
    node::{Blackboard, ball_timeout},
    selection, sequence, subtree,
    switch_motion_type::switch_motion_type,
};

// Based on Sonja's replacement_keeper claim election; retain its timing and rank backoff.
const ELECTION_WINDOW: Duration = Duration::from_millis(500);
const NO_CLAIM_TIMEOUT: Duration = Duration::from_secs(2);
const BACKOFF_STEP: Duration = Duration::from_millis(200);

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, ros_z::Message)]
enum ReplacementGoalkeeperState {
    #[default]
    Inactive,
    Candidate,
    Active,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize, ros_z::Message)]
pub struct ReplacementGoalkeeperCoordinator {
    state: ReplacementGoalkeeperState,
    known_replacement_goalkeeper: Option<PlayerNumber>,
    election_started_at: Option<Time>,
    replacement_needed_since: Option<Time>,
}

struct ElectionInput {
    now: Time,
    own_number: PlayerNumber,
    needed: bool,
    available: Players<bool>,
    eligible: bool,
    preferred: Option<PlayerNumber>,
    other_claim: Option<PlayerNumber>,
}

impl ReplacementGoalkeeperCoordinator {
    pub fn keeper(&self) -> Option<PlayerNumber> {
        self.known_replacement_goalkeeper
    }

    pub fn is_active(&self) -> bool {
        matches!(self.state, ReplacementGoalkeeperState::Active)
    }

    pub fn claims_role(&self) -> bool {
        matches!(
            self.state,
            ReplacementGoalkeeperState::Candidate | ReplacementGoalkeeperState::Active
        )
    }

    pub fn is_electing(&self) -> bool {
        self.election_started_at.is_some() && self.keeper().is_none()
    }

    fn remember(&mut self, winner: PlayerNumber, own_number: PlayerNumber) {
        self.known_replacement_goalkeeper = Some(winner);
        self.state = if winner == own_number {
            ReplacementGoalkeeperState::Active
        } else {
            ReplacementGoalkeeperState::Inactive
        };
        self.election_started_at = None;
    }

    fn update(&mut self, input: ElectionInput) {
        if !input.needed
            || input
                .available
                .iter()
                .filter(|(_, available)| **available)
                .count()
                < 2
        {
            *self = Self::default();
            return;
        }
        if self
            .replacement_needed_since
            .is_some_and(|time| time > input.now)
            || self
                .election_started_at
                .is_some_and(|time| time > input.now)
        {
            *self = Self::default();
        }
        let needed_since = *self.replacement_needed_since.get_or_insert(input.now);
        if self.keeper().is_some_and(|winner| {
            !input.available[winner]
                || (winner != input.own_number && input.other_claim != Some(winner))
        }) {
            self.known_replacement_goalkeeper = None;
            self.state = ReplacementGoalkeeperState::Inactive;
            self.election_started_at = None;
        }
        if let Some(winner) = self.keeper() {
            // Resolve simultaneous claims, but never launch a new challenge to a fresh winner.
            self.remember(
                input.other_claim.map_or(winner, |other| other.min(winner)),
                input.own_number,
            );
            return;
        }
        if matches!(self.state, ReplacementGoalkeeperState::Candidate) && !input.eligible {
            self.state = ReplacementGoalkeeperState::Inactive;
        }
        let rank = input
            .available
            .iter()
            .filter(|(number, available)| **available && *number < input.own_number)
            .count() as u32;
        let fallback_due =
            input.now.duration_since(needed_since) >= NO_CLAIM_TIMEOUT + BACKOFF_STEP * rank;
        if input.eligible
            && input.other_claim.is_none()
            && (input.preferred == Some(input.own_number) || fallback_due)
        {
            self.state = ReplacementGoalkeeperState::Candidate;
        }
        let own_claim = self.claims_role().then_some(input.own_number);
        let Some(winner) = input.other_claim.into_iter().chain(own_claim).min() else {
            self.election_started_at = None;
            return;
        };
        let election_start = *self.election_started_at.get_or_insert(input.now);
        if input.now.duration_since(election_start) >= ELECTION_WINDOW {
            self.remember(winner, input.own_number);
        }
    }
}

pub fn refresh_replacement_goalkeeper_role(blackboard: &mut Blackboard) -> Status {
    let input = election_input(blackboard);
    match input {
        Some(input) => blackboard.replacement_goalkeeper_coordinator.update(input),
        None => blackboard.replacement_goalkeeper_coordinator = Default::default(),
    }
    if blackboard.replacement_goalkeeper_coordinator.is_active() {
        blackboard.ball_action = None;
        blackboard.kick_target = None;
        blackboard.last_closest_to_ball = false;
        blackboard.closest_to_ball_entered_area_since = None;
        blackboard.closest_to_ball_left_area_since = None;
    }
    Status::Success
}

fn election_input(blackboard: &mut Blackboard) -> Option<ElectionInput> {
    let protected_ball_action =
        is_finishing_striker_kick(blackboard) || is_pass_receiver(blackboard);
    let world = &blackboard.world_state;
    let game = world.filtered_game_controller_state.as_ref()?;
    let transform = world.robot.ground_to_field?;
    if world.robot.primary_state != PrimaryState::Playing
        || !matches!(game.game_phase, GamePhase::Normal | GamePhase::Extratime)
        || !matches!(
            game.game_state,
            FilteredGameState::Playing {
                ball_is_free: true,
                kick_off: false
            }
        )
        || game.sub_state == Some(SubState::PenaltyKick)
        || blackboard.remote_control_enabled
        || blackboard
            .parameters
            .control
            .injected_motion_command
            .is_some()
        || blackboard.is_injected_motion_command
        || !world
            .fall_detection
            .is_some_and(|fall| fall.is_valid(world.now) && fall.posture == Posture::Upright)
    {
        return None;
    }
    let own_number = world.robot.player_number;
    let regular_keeper = blackboard.parameters.goalkeeper.player_number;
    let timeout = blackboard
        .parameters
        .network
        .hsl_striker_message_receive_timeout;
    let mut positions = Players::new(None);
    for (number, state) in world.player_states.iter() {
        if game.penalties[number].is_some() {
            continue;
        }
        let pose = if number == own_number {
            Some(transform.as_pose())
        } else {
            state
                .as_ref()
                .filter(|state| {
                    state.time <= world.now && world.now.duration_since(state.time) < timeout
                })
                .map(|state| state.inner.pose)
        };
        positions[number] = pose
            .filter(|pose| {
                pose.position().inner.iter().all(|value| value.is_finite())
                    && pose.orientation().angle().is_finite()
            })
            .map(|pose| pose.position());
    }
    positions[own_number]?;
    let field = blackboard.field_dimensions;
    let offset = blackboard.parameters.goalkeeper.x_offset;
    if ![field.length, field.goal_inner_width, offset]
        .into_iter()
        .all(f32::is_finite)
        || field.length <= 0.0
        || field.goal_inner_width <= 0.0
        || offset < 0.0
    {
        return None;
    }
    let goal = point![-field.length / 2.0, 0.0];
    let return_radius = field.goal_inner_width / 2.0;
    let keeper_radius = return_radius
        + if blackboard
            .replacement_goalkeeper_coordinator
            .replacement_needed_since
            .is_some()
        {
            0.0
        } else {
            offset
        };
    let needed = positions[regular_keeper]
        .is_none_or(|position| position.x() < goal.x() || (position - goal).norm() > keeper_radius);
    let available = positions.map(|position| position.is_some());
    let mut fielders = available;
    fielders[regular_keeper] = false;
    let ball_carrier = blackboard
        .ball
        .as_ref()
        .filter(|ball| {
            ball.age <= world.now
                && world.now.duration_since(ball.age)
                    < ball_timeout(&blackboard.parameters, ball.source)
                && ball.position.inner.iter().all(|value| value.is_finite())
        })
        .and_then(|ball| nearest_player(&positions, ball.position, |_, _| true));
    let preferred = nearest_player(&positions, goal, |number, _| {
        number != regular_keeper && Some(number) != ball_carrier
    });
    let other_claim = world
        .player_states
        .iter()
        .filter_map(|(number, state)| {
            (number != own_number
                && fielders[number]
                && state
                    .as_ref()
                    .is_some_and(|state| state.inner.replacement_goalkeeper_claim))
            .then_some(number)
        })
        .min();
    Some(ElectionInput {
        now: world.now,
        own_number,
        needed,
        available: fielders,
        eligible: fielders[own_number]
            && Some(own_number) != ball_carrier
            && !protected_ball_action,
        preferred,
        other_claim,
    })
}

fn nearest_player(
    positions: &Players<Option<Point2<Field>>>,
    target: Point2<Field>,
    eligible: impl Fn(PlayerNumber, Point2<Field>) -> bool,
) -> Option<PlayerNumber> {
    positions
        .iter()
        .filter_map(|(number, position)| {
            let position = (*position)?;
            eligible(number, position).then_some((number, (position - target).norm_squared()))
        })
        .min_by(
            |(left_number, left_distance), (right_number, right_distance)| {
                left_distance
                    .total_cmp(right_distance)
                    .then(left_number.cmp(right_number))
            },
        )
        .map(|(number, _)| number)
}

pub fn is_active_replacement_goalkeeper(blackboard: &mut Blackboard) -> bool {
    blackboard.replacement_goalkeeper_coordinator.is_active()
}

pub fn replacement_goalkeeper_subtree() -> Node<Blackboard> {
    sequence!(
        subtree!(look_at_ball_subtree),
        selection!(
            subtree!(goalkeeper_shot_block_position_subtree),
            switch_motion_type(
                MotionType::Walk,
                action!(walk_to_goalkeeper_default_position),
                action!(stand),
            ),
            action!(stand),
        ),
    )
}

#[cfg(test)]
mod tests {
    use hsl_network_messages::Penalty;
    use linear_algebra::{Pose2, vector};
    use types::{
        fall_detection::FallDetection,
        field_dimensions::{FieldDimensions, Side},
        filtered_game_controller_state::FilteredGameControllerState,
        motion_command::BodyMotion,
        path::traits::EndPoints,
        time_wrapper::TimeWrapper,
        world_state::{BallSource, PlayerState},
    };

    use super::*;
    use crate::node::LastBall;

    fn board(number: PlayerNumber) -> Blackboard {
        let now = Time::zero() + Duration::from_secs(10);
        let mut board = Blackboard {
            field_dimensions: FieldDimensions {
                length: 14.0,
                goal_inner_width: 2.4,
                ..FieldDimensions::SPL_2025
            },
            last_motion_type: Some(MotionType::Walk),
            ..Default::default()
        };
        board.parameters.goalkeeper.player_number = PlayerNumber::One;
        board.parameters.goalkeeper.x_offset = 0.3;
        board.parameters.ball.last_ball_timeout = Duration::from_secs(1);
        board.parameters.network.hsl_striker_message_receive_timeout = Duration::from_secs(3);
        board.world_state.now = now;
        board.world_state.robot.player_number = number;
        board.world_state.robot.primary_state = PrimaryState::Playing;
        board.world_state.filtered_game_controller_state = Some(FilteredGameControllerState {
            game_state: FilteredGameState::Playing {
                ball_is_free: true,
                kick_off: false,
            },
            ..Default::default()
        });
        board.world_state.fall_detection = Some(FallDetection {
            time: now,
            valid_until: now + Duration::from_secs(30),
            posture: Posture::Upright,
        });
        for (player, position) in [
            (PlayerNumber::One, point![-4.0, 0.0]),
            (PlayerNumber::Two, point![-5.0, 2.0]),
            (PlayerNumber::Three, point![-2.0, 0.0]),
        ] {
            board.world_state.player_states[player] = Some(TimeWrapper {
                time: now,
                inner: PlayerState {
                    pose: position.into(),
                    ..Default::default()
                },
            });
        }
        board.world_state.robot.ground_to_field = Some(
            board.world_state.player_states[number]
                .as_ref()
                .unwrap()
                .inner
                .pose
                .as_transform(),
        );
        board.ball = Some(LastBall {
            position: point![-2.0, 0.0],
            velocity: vector![0.0, 0.0],
            age: now,
            field_side: Side::Left,
            source: BallSource::Own,
        });
        board
    }

    fn advance(board: &mut Blackboard, duration: Duration) {
        board.world_state.now = board.world_state.now + duration;
        board.ball.as_mut().unwrap().age = board.world_state.now;
        for number in [
            PlayerNumber::One,
            PlayerNumber::Two,
            PlayerNumber::Three,
            PlayerNumber::Four,
            PlayerNumber::Five,
        ] {
            if let Some(state) = &mut board.world_state.player_states[number] {
                state.time = board.world_state.now;
            }
        }
    }

    fn elect(board: &mut Blackboard) {
        refresh_replacement_goalkeeper_role(board);
        advance(board, ELECTION_WINDOW);
        refresh_replacement_goalkeeper_role(board);
    }

    #[test]
    fn one_defender_wins_and_conflicting_claims_converge_to_lower_number() {
        let mut defender = board(PlayerNumber::Two);
        let mut striker = board(PlayerNumber::Three);
        refresh_replacement_goalkeeper_role(&mut defender);
        refresh_replacement_goalkeeper_role(&mut striker);
        assert!(defender.replacement_goalkeeper_coordinator.claims_role());
        assert!(!striker.replacement_goalkeeper_coordinator.claims_role());
        striker
            .world_state
            .player_states
            .two
            .as_mut()
            .unwrap()
            .inner
            .replacement_goalkeeper_claim = true;
        advance(&mut defender, ELECTION_WINDOW);
        advance(&mut striker, ELECTION_WINDOW);
        refresh_replacement_goalkeeper_role(&mut defender);
        refresh_replacement_goalkeeper_role(&mut striker);
        advance(&mut striker, ELECTION_WINDOW);
        refresh_replacement_goalkeeper_role(&mut striker);
        assert!(defender.replacement_goalkeeper_coordinator.is_active());
        assert_eq!(
            striker.replacement_goalkeeper_coordinator.keeper(),
            Some(PlayerNumber::Two)
        );
        assert!(!striker.replacement_goalkeeper_coordinator.is_active());

        // A disconnected election can have two winners until the first claims arrive.
        striker
            .replacement_goalkeeper_coordinator
            .remember(PlayerNumber::Three, PlayerNumber::Three);
        defender
            .world_state
            .player_states
            .three
            .as_mut()
            .unwrap()
            .inner
            .replacement_goalkeeper_claim = true;
        refresh_replacement_goalkeeper_role(&mut defender);
        refresh_replacement_goalkeeper_role(&mut striker);
        assert!(defender.replacement_goalkeeper_coordinator.is_active());
        assert!(!striker.replacement_goalkeeper_coordinator.claims_role());
        assert_eq!(
            striker.replacement_goalkeeper_coordinator.keeper(),
            Some(PlayerNumber::Two)
        );

        defender.ball.as_mut().unwrap().position = point![-5.0, 2.0];
        defender.ball_action = Some(crate::kick::strategy::BallAction::Dribble);
        defender.kick_target = Some(point![1.0, 0.0]);
        defender.last_closest_to_ball = true;
        refresh_replacement_goalkeeper_role(&mut defender);
        assert!(defender.replacement_goalkeeper_coordinator.is_active());
        assert!(defender.ball_action.is_none());
        assert!(defender.kick_target.is_none());
        assert!(!defender.last_closest_to_ball);
    }

    #[test]
    fn fresh_existing_claim_is_stable_and_invalid_winners_are_replaced() {
        for invalidation in 0..3 {
            let mut board = board(PlayerNumber::Two);
            board
                .world_state
                .player_states
                .three
                .as_mut()
                .unwrap()
                .inner
                .replacement_goalkeeper_claim = true;
            elect(&mut board);
            assert_eq!(
                board.replacement_goalkeeper_coordinator.keeper(),
                Some(PlayerNumber::Three)
            );
            assert!(!board.replacement_goalkeeper_coordinator.claims_role());
            refresh_replacement_goalkeeper_role(&mut board);
            assert_eq!(
                board.replacement_goalkeeper_coordinator.keeper(),
                Some(PlayerNumber::Three)
            );

            board.world_state.player_states.four = Some(TimeWrapper {
                time: board.world_state.now,
                inner: PlayerState {
                    pose: point![1.0, 1.0].into(),
                    ..Default::default()
                },
            });
            match invalidation {
                0 => board.world_state.player_states.three.as_mut().unwrap().time = Time::zero(),
                1 => {
                    board
                        .world_state
                        .filtered_game_controller_state
                        .as_mut()
                        .unwrap()
                        .penalties
                        .three = Some(Penalty::PickUp {
                        remaining: Duration::from_secs(30),
                    })
                }
                _ => {
                    board
                        .world_state
                        .player_states
                        .three
                        .as_mut()
                        .unwrap()
                        .inner
                        .replacement_goalkeeper_claim = false
                }
            }
            refresh_replacement_goalkeeper_role(&mut board);
            assert!(
                board.replacement_goalkeeper_coordinator.keeper().is_none(),
                "case {invalidation}"
            );
            assert!(
                board.replacement_goalkeeper_coordinator.claims_role(),
                "case {invalidation}"
            );
            board.world_state.now = board.world_state.now + ELECTION_WINDOW;
            board.ball.as_mut().unwrap().age = board.world_state.now;
            refresh_replacement_goalkeeper_role(&mut board);
            assert!(
                board.replacement_goalkeeper_coordinator.is_active(),
                "case {invalidation}"
            );
        }
    }

    #[test]
    fn keeper_excursion_and_return_have_hysteresis_and_use_configured_number() {
        let mut board = board(PlayerNumber::Two);
        board.parameters.goalkeeper.player_number = PlayerNumber::Three;
        board.ball.as_mut().unwrap().position = point![-4.0, 0.0];
        let set_keeper = |board: &mut Blackboard, x| {
            board
                .world_state
                .player_states
                .three
                .as_mut()
                .unwrap()
                .inner
                .pose = point![x, 0.0].into();
        };
        set_keeper(&mut board, -5.6);
        refresh_replacement_goalkeeper_role(&mut board);
        assert!(!board.replacement_goalkeeper_coordinator.claims_role());
        set_keeper(&mut board, -5.4);
        elect(&mut board);
        assert!(board.replacement_goalkeeper_coordinator.is_active());
        set_keeper(&mut board, -5.6);
        refresh_replacement_goalkeeper_role(&mut board);
        assert!(board.replacement_goalkeeper_coordinator.is_active());
        set_keeper(&mut board, -5.9);
        refresh_replacement_goalkeeper_role(&mut board);
        assert!(board.replacement_goalkeeper_coordinator.keeper().is_none());
        set_keeper(&mut board, -7.1);
        elect(&mut board);
        assert!(board.replacement_goalkeeper_coordinator.is_active());
    }

    #[test]
    fn chasing_regular_keeper_counts_as_ball_carrier_and_own_pose_needs_no_echo() {
        let mut board = board(PlayerNumber::Two);
        board.ball.as_mut().unwrap().position = point![-4.0, 0.0];
        board.world_state.player_states.two = None;
        elect(&mut board);
        assert!(board.replacement_goalkeeper_coordinator.is_active());
        assert_eq!(
            board.replacement_goalkeeper_coordinator.keeper(),
            Some(PlayerNumber::Two)
        );

        board.replacement_goalkeeper_coordinator = Default::default();
        board.ball.as_mut().unwrap().position = point![-5.0, 2.0];
        refresh_replacement_goalkeeper_role(&mut board);
        assert!(!board.replacement_goalkeeper_coordinator.claims_role());
    }

    #[test]
    fn no_claim_fallback_preserves_rank_backoff_and_election_window() {
        for (number, rank) in [(PlayerNumber::Two, 0), (PlayerNumber::Three, 1)] {
            let mut coordinator = ReplacementGoalkeeperCoordinator::default();
            let input = |elapsed| ElectionInput {
                now: Time::zero() + elapsed,
                own_number: number,
                needed: true,
                available: Players {
                    two: true,
                    three: true,
                    ..Players::new(false)
                },
                eligible: true,
                preferred: None,
                other_claim: None,
            };
            let delay = NO_CLAIM_TIMEOUT + BACKOFF_STEP * rank;
            coordinator.update(input(Duration::ZERO));
            coordinator.update(input(delay - Duration::from_millis(1)));
            assert!(!coordinator.claims_role());
            coordinator.update(input(delay));
            assert!(coordinator.claims_role());
            assert!(coordinator.is_electing());
            assert!(!coordinator.is_active());
            coordinator.update(input(delay + ELECTION_WINDOW));
            assert!(coordinator.is_active());
        }
    }

    #[test]
    fn last_field_player_and_unsafe_or_setup_states_drop_local_role() {
        for case in 0..10 {
            let mut board = board(PlayerNumber::Two);
            elect(&mut board);
            assert!(board.replacement_goalkeeper_coordinator.is_active());
            match case {
                0 => board.world_state.player_states.three = None,
                1 => board.remote_control_enabled = true,
                2 => board.world_state.fall_detection.as_mut().unwrap().posture = Posture::Falling,
                3 => board.world_state.filtered_game_controller_state = None,
                4 => board.world_state.robot.ground_to_field = None,
                5 => board.world_state.robot.primary_state = PrimaryState::Penalized,
                6 => {
                    board
                        .world_state
                        .filtered_game_controller_state
                        .as_mut()
                        .unwrap()
                        .sub_state = Some(SubState::PenaltyKick)
                }
                7 => {
                    board
                        .world_state
                        .filtered_game_controller_state
                        .as_mut()
                        .unwrap()
                        .game_state = FilteredGameState::Ready
                }
                8 => {
                    board
                        .world_state
                        .filtered_game_controller_state
                        .as_mut()
                        .unwrap()
                        .game_state = FilteredGameState::Playing {
                        ball_is_free: true,
                        kick_off: true,
                    }
                }
                _ => board.parameters.control.injected_motion_command = Some(Default::default()),
            }
            refresh_replacement_goalkeeper_role(&mut board);
            assert!(
                !board.replacement_goalkeeper_coordinator.claims_role(),
                "case {case}"
            );
            assert!(
                board.replacement_goalkeeper_coordinator.keeper().is_none(),
                "case {case}"
            );
        }
    }

    #[test]
    fn cover_position_never_chases_or_kicks_a_nearby_ball() {
        let mut board = board(PlayerNumber::Two);
        let transform = Pose2::new(point![-5.0, 2.0], 0.0).as_transform();
        board.world_state.robot.ground_to_field = Some(transform);
        board.ball.as_mut().unwrap().position = point![-4.7, 2.0];
        let (status, _) = replacement_goalkeeper_subtree().tick_with_trace(&mut board);
        assert_eq!(status, Status::Success);
        let Some(BodyMotion::Walk { path, .. }) = board.body_motion else {
            panic!("cover must walk toward its goal instead of taking the nearby ball");
        };
        assert!(board.ball_action.is_none());
        let target = transform * path.end_point();
        assert!((target.x() + 6.7).abs() < 1e-5);
        assert!(target.y().abs() < board.field_dimensions.goal_inner_width / 2.0);
    }
}
