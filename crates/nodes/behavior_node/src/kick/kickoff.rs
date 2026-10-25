use std::time::Duration;

use coordinate_systems::Field;
use geometry::{Distance, line_segment::LineSegment};
use hsl_network_messages::{PlayerNumber, Team};
use linear_algebra::{Point2, Pose2};
use types::{
    behavior_tree::Status, filtered_game_state::FilteredGameState, motion_type::MotionType,
    obstacles::ObstacleKind,
};

use crate::{
    action,
    actions::stand,
    behavior_tree::Node,
    condition,
    head::look_at_ball_subtree,
    node::{Blackboard, ball_timeout},
    selection, sequence, subtree,
    switch_motion_type::switch_motion_type,
    walk::{select_kickoff_pose, walk_to_kickoff_pose},
};

use super::{execution::execute_ball_action, strategy::BallAction};

pub fn kickoff_kicker(blackboard: &Blackboard) -> Option<PlayerNumber> {
    blackboard
        .world_state
        .filtered_game_controller_state
        .as_ref()?
        .penalties
        .iter()
        .filter(|(player, penalty)| {
            *player != blackboard.parameters.goalkeeper.player_number && penalty.is_none()
        })
        .map(|(player, _)| player)
        .max()
}

pub fn kickoff_receiver(blackboard: &Blackboard) -> Option<PlayerNumber> {
    let kicker = kickoff_kicker(blackboard)?;
    // Choose the Ready receiver before checking freshness, so stale messages cannot change roles.
    blackboard
        .world_state
        .filtered_game_controller_state
        .as_ref()?
        .penalties
        .iter()
        .filter(|(player, penalty)| {
            *player != kicker
                && *player != blackboard.parameters.goalkeeper.player_number
                && penalty.is_none()
        })
        .map(|(player, _)| player)
        .max()
        .filter(|&player| player_pose(blackboard, player, Duration::from_secs(2)).is_some())
}

pub(crate) fn kickoff_receiving_pose(blackboard: &Blackboard) -> Option<Pose2<Field>> {
    if !own_kickoff(blackboard) {
        return None;
    }
    let field = &blackboard.field_dimensions;
    let clearance = blackboard.parameters.walking.path_planning.robot_radius + 0.05;
    if !clearance.is_finite()
        || clearance < 0.0
        || !field.length.is_finite()
        || field.length <= 2.0 * clearance
        || !field.width.is_finite()
        || field.width <= 2.0 * clearance
    {
        return None;
    }
    Some(select_kickoff_pose(
        blackboard,
        kickoff_receiver(blackboard)?,
        blackboard
            .world_state
            .filtered_game_controller_state
            .as_ref()?,
        blackboard.parameters.goalkeeper.player_number,
        &blackboard.parameters.kickoff,
        field,
    ))
}

fn is_own_kickoff(blackboard: &mut Blackboard) -> bool {
    let receiver = kickoff_receiver(blackboard);
    let receiving_ball = Some(blackboard.player_number) == receiver
        && blackboard.ball.as_ref().is_some_and(|ball| {
            let now = blackboard.world_state.now;
            let speed = ball.velocity.norm();
            ball.age <= now
                && now.duration_since(ball.age) < ball_timeout(&blackboard.parameters, ball.source)
                && ball.position.coords().norm() > blackboard.field_dimensions.ball_radius / 2.0
                && speed.is_finite()
                && speed
                    > blackboard
                        .parameters
                        .ball
                        .interception
                        .minimum_ball_velocity
        });
    let active = own_kickoff(blackboard)
        && !receiving_ball
        && blackboard.has_team_message_budget()
        && blackboard.player_number != blackboard.parameters.goalkeeper.player_number
        && Some(blackboard.parameters.goalkeeper.player_number) != kickoff_kicker(blackboard)
        && player_pose(
            blackboard,
            kickoff_kicker(blackboard).unwrap_or(blackboard.player_number),
            Duration::from_secs(2),
        )
        .is_some();
    if !active || receiver.is_some() {
        blackboard.kickoff_receiver_wait_started = None;
        return active;
    }
    if Some(blackboard.player_number) != kickoff_kicker(blackboard) {
        return false;
    }
    let now = blackboard.world_state.now;
    let since = *blackboard.kickoff_receiver_wait_started.get_or_insert(now);
    now.duration_since(since) < Duration::from_secs(2)
}

pub(crate) fn needs_kickoff_updates(blackboard: &Blackboard) -> bool {
    kickoff_passing_enabled(blackboard)
        && blackboard.player_number != blackboard.parameters.goalkeeper.player_number
        && blackboard
            .world_state
            .filtered_game_controller_state
            .as_ref()
            .is_some_and(|game| {
                game.kicking_team == Some(Team::Hulks)
                    && game.sub_state.is_none()
                    && (matches!(
                        game.game_state,
                        FilteredGameState::Ready | FilteredGameState::Set
                    ) || own_kickoff(blackboard))
            })
}

pub(super) fn own_kickoff(blackboard: &Blackboard) -> bool {
    kickoff_passing_enabled(blackboard)
        && blackboard
            .world_state
            .filtered_game_controller_state
            .as_ref()
            .is_some_and(|game| {
                game.kicking_team == Some(Team::Hulks)
                    && game.sub_state.is_none()
                    && matches!(game.game_state, FilteredGameState::Playing { .. })
                    && (matches!(
                        game.game_state,
                        FilteredGameState::Playing { kick_off: true, .. }
                    ) || blackboard.kickoff_started.is_some())
            })
        && blackboard
            .ball
            .as_ref()
            .is_some_and(|ball| ball.position.coords().norm() < 0.35)
}

fn receiver_pose(blackboard: &Blackboard, receiver: PlayerNumber) -> Option<Pose2<Field>> {
    if Some(receiver) != kickoff_receiver(blackboard) {
        return None;
    }
    player_pose(
        blackboard,
        receiver,
        blackboard
            .parameters
            .kicking
            .strategy
            .passing
            .maximum_receiver_age,
    )
}

fn player_pose(
    blackboard: &Blackboard,
    player: PlayerNumber,
    maximum_age: Duration,
) -> Option<Pose2<Field>> {
    let world = &blackboard.world_state;
    let game = world.filtered_game_controller_state.as_ref()?;
    if game.penalties[player].is_some() {
        return None;
    }
    if player == blackboard.player_number {
        return world
            .robot
            .ground_to_field
            .map(|transform| transform.as_pose())
            .filter(|pose| {
                pose.position().inner.iter().all(|v| v.is_finite()) && pose.angle().is_finite()
            });
    }
    let state = world.player_states[player].as_ref()?;
    (state.inner.is_finite()
        && state.time <= world.now
        && world.now.duration_since(state.time) < maximum_age)
        .then_some(state.inner.pose)
}

/// Kickoff permits a sideways/backwards pass, while retaining receiver and corridor checks.
pub(super) fn kickoff_pass_target(
    blackboard: &Blackboard,
    receiver: PlayerNumber,
) -> Option<Point2<Field>> {
    if !own_kickoff(blackboard) || !blackboard.has_team_message_budget() {
        return None;
    }
    let parameters = &blackboard.parameters.kicking.strategy;
    parameters.validate().ok()?;
    let pose = receiver_pose(blackboard, receiver)?;
    let stance = kickoff_receiving_pose(blackboard)?;
    let walking = blackboard.parameters.walking.walk_and_stand;
    let orientation_error = pose
        .orientation()
        .rotation_to(stance.orientation())
        .angle()
        .abs();
    if (pose.position() - stance.position()).norm()
        > walking
            .target_reached_thresholds
            .x
            .max(blackboard.field_dimensions.ball_radius)
        || !orientation_error.is_finite()
        || orientation_error > walking.target_reached_thresholds.y + walking.hysteresis.y
    {
        return None;
    }
    let position = pose.position();
    let ball = blackboard.ball.as_ref()?.position;
    let delta = position - ball;
    let distance = delta.norm();
    if !distance.is_finite()
        || distance <= parameters.passing.receiver_standoff
        || distance > parameters.passing.maximum_distance
    {
        return None;
    }
    let target = stance.position();
    if (target.y() > 0.0) != blackboard.parameters.kickoff.pass_to_left {
        return None;
    }
    if (target - position).norm()
        > parameters.passing.receiver_standoff + 2.0 * parameters.passing.target_half_width
        || (target - ball).norm() > parameters.passing.maximum_distance
    {
        return None;
    }
    let field = &blackboard.field_dimensions;
    if !field.length.is_finite()
        || !field.width.is_finite()
        || target.x().abs() >= field.length / 2.0
        || target.y().abs() >= field.width / 2.0
    {
        return None;
    }
    let transform = blackboard.world_state.robot.ground_to_field?;
    // Check the corridor up to the receiver's clearance, rather than treating it as a blocker.
    let direction = (target - ball).try_normalize(f32::EPSILON)?;
    let path = LineSegment(
        ball,
        target - direction * parameters.passing.receiver_standoff,
    );
    blackboard
        .world_state
        .obstacles
        .iter()
        .filter(|obstacle| obstacle.kind != ObstacleKind::Ball)
        .all(|obstacle| {
            path.distance_to(transform * obstacle.position)
                > obstacle.radius_at_foot_height + parameters.obstacle_clearance + field.ball_radius
        })
        .then_some(target)
}

fn is_striker_with_receiver(blackboard: &mut Blackboard) -> bool {
    Some(blackboard.player_number) == kickoff_kicker(blackboard)
        && blackboard.has_team_message_budget()
}

fn is_other_field_player(blackboard: &mut Blackboard) -> bool {
    let player = blackboard.player_number;
    Some(player) != kickoff_kicker(blackboard)
        && player != blackboard.parameters.goalkeeper.player_number
        && Some(player) != kickoff_receiver(blackboard)
}

fn is_kickoff_receiver(blackboard: &mut Blackboard) -> bool {
    Some(blackboard.player_number) == kickoff_receiver(blackboard)
        && blackboard.player_number != blackboard.parameters.goalkeeper.player_number
}

fn select_kickoff_pass(blackboard: &mut Blackboard) -> Status {
    blackboard.ball_action = None;
    blackboard.kick_target = None;
    let Some(receiver) = kickoff_receiver(blackboard) else {
        return Status::Failure;
    };
    if kickoff_pass_target(blackboard, receiver).is_none() {
        return Status::Failure;
    }
    blackboard.ball_action = Some(BallAction::Pass { receiver });
    Status::Success
}

pub fn kickoff_subtree() -> Node<Blackboard> {
    sequence!(
        condition!(is_own_kickoff),
        subtree!(look_at_ball_subtree),
        selection!(
            sequence!(
                condition!(is_striker_with_receiver),
                selection!(
                    sequence!(action!(select_kickoff_pass), action!(execute_ball_action)),
                    action!(stand),
                ),
            ),
            sequence!(
                selection!(
                    condition!(is_kickoff_receiver),
                    condition!(is_other_field_player)
                ),
                switch_motion_type(
                    MotionType::Walk,
                    action!(walk_to_kickoff_pose),
                    action!(stand)
                ),
            ),
        ),
    )
}

fn kickoff_passing_enabled(blackboard: &Blackboard) -> bool {
    blackboard
        .parameters
        .kicking
        .strategy
        .passing
        .maximum_distance
        > 0.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use hsl_network_messages::{HulkMessage, SubState};
    use linear_algebra::{point, vector};
    use types::{
        messages::OutgoingMessage, motion_command::BodyMotion, obstacles::Obstacle,
        primary_state::PrimaryState, time_wrapper::TimeWrapper, world_state::PlayerState,
    };

    fn ready_pose(board: &Blackboard, player: PlayerNumber) -> Pose2<Field> {
        select_kickoff_pose(
            board,
            player,
            board
                .world_state
                .filtered_game_controller_state
                .as_ref()
                .unwrap(),
            board.parameters.goalkeeper.player_number,
            &board.parameters.kickoff,
            &board.field_dimensions,
        )
    }

    fn kickoff() -> Blackboard {
        let mut board =
            crate::test_support::restart_blackboard(SubState::DirectFreeKick, Point2::origin());
        board.player_number = PlayerNumber::Five;
        board
            .world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .sub_state = None;
        board
            .world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .game_state = FilteredGameState::Playing {
            ball_is_free: false,
            kick_off: true,
        };
        let pose = ready_pose(&board, PlayerNumber::Four);
        board.world_state.player_states.four = Some(TimeWrapper {
            time: board.world_state.now,
            inner: PlayerState {
                pose,
                ..crate::test_support::player_state()
            },
        });
        board
    }

    #[test]
    fn disabled_passing_keeps_base_kickoff_behavior_and_pose_slots() {
        let mut board = kickoff();
        board.parameters.kicking.strategy.passing.maximum_distance = 0.0;
        board.parameters.control.is_simple = true;
        assert!(!own_kickoff(&board));
        assert!(!needs_kickoff_updates(&board));
        assert!(kickoff_receiving_pose(&board).is_none());
        assert_eq!(
            kickoff_subtree().tick_with_trace(&mut board).0,
            Status::Failure
        );
        crate::tree::create_tree().tick_with_trace(&mut board);
        assert!(matches!(board.body_motion, Some(BodyMotion::Kick { .. })));
        assert!(!matches!(board.ball_action, Some(BallAction::Pass { .. })));

        board
            .world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .game_state = FilteredGameState::Ready;
        assert!(!needs_kickoff_updates(&board));
        assert!(kickoff_receiving_pose(&board).is_none());
    }

    #[test]
    fn kickoff_waits_announces_immediately_prepares_150ms_and_forces_strong_when_allowed() {
        let mut board = kickoff();
        board.last_sent_hsl_message_time = Some(board.world_state.now);
        assert_eq!(kickoff_kicker(&board), Some(PlayerNumber::Five));
        assert_eq!(kickoff_receiver(&board), Some(PlayerNumber::Four));
        assert_eq!(select_kickoff_pass(&mut board), Status::Success);
        assert_eq!(execute_ball_action(&mut board), Status::Success);
        assert_eq!(board.body_motion, Some(BodyMotion::Stand));
        let Some(OutgoingMessage::Hsl(HulkMessage::State(message))) =
            board.try_sending_state_message()
        else {
            panic!("pass announcement must bypass adaptive cooldown");
        };
        assert_eq!(message.pass.unwrap().receiver, PlayerNumber::Four);
        assert!(
            (message.pass.unwrap().target - ready_pose(&board, PlayerNumber::Four).position())
                .norm()
                < 1e-5
        );
        let announced_at = board.last_announced_pass.as_ref().unwrap().time;
        for (age, ready) in [(149, false), (150, true)] {
            board.world_state.now = announced_at + Duration::from_millis(age);
            execute_ball_action(&mut board);
            assert_eq!(
                matches!(
                    board.body_motion,
                    Some(BodyMotion::Kick {
                        strong: true,
                        soft: false,
                        quick: false,
                        ..
                    })
                ),
                ready
            );
        }
        board.parameters.kicking.allow_strong_kicks = false;
        execute_ball_action(&mut board);
        assert!(matches!(
            board.body_motion,
            Some(BodyMotion::Kick { strong: false, .. })
        ));
    }

    #[test]
    fn kickoff_targets_the_receiver_ready_slot_on_either_side() {
        for left in [false, true] {
            let mut board = kickoff();
            board.parameters.kickoff.pass_to_left = left;
            board.parameters.kickoff.relative_aggressive_positions[0] =
                point![-0.1, if left { 0.7 } else { -0.7 }];
            let receiving = ready_pose(&board, PlayerNumber::Four);
            board
                .world_state
                .player_states
                .four
                .as_mut()
                .unwrap()
                .inner
                .pose = receiving;
            let position = receiving.position();
            assert_eq!(position.y() > 0.0, left);
            assert!(position.x() <= -board.parameters.walking.path_planning.robot_radius);
            assert!(position.coords().norm() > board.field_dimensions.center_circle_diameter / 2.0);
            let target = kickoff_pass_target(&board, PlayerNumber::Four).unwrap();
            assert_eq!(target, position);
            assert!(target.x() < 0.0);
            assert!(target.y().abs() < board.field_dimensions.width / 2.0);
            let transform = board.world_state.robot.ground_to_field.unwrap();
            board.world_state.obstacles =
                vec![Obstacle::robot(transform.inverse() * position, 0.2, 0.2)];
            assert_eq!(
                kickoff_pass_target(&board, PlayerNumber::Four),
                Some(position)
            );
            for state in [FilteredGameState::Ready, FilteredGameState::Set] {
                board
                    .world_state
                    .filtered_game_controller_state
                    .as_mut()
                    .unwrap()
                    .game_state = state;
                assert!(kickoff_receiving_pose(&board).is_none());
            }
            let game = board
                .world_state
                .filtered_game_controller_state
                .as_mut()
                .unwrap();
            game.game_state = FilteredGameState::Playing {
                ball_is_free: false,
                kick_off: true,
            };
            game.kicking_team = Some(Team::Opponent);
            assert!(kickoff_receiving_pose(&board).is_none());
        }
    }

    #[test]
    fn kickoff_receiver_and_supporter_stay_in_their_ready_slots() {
        for player in [PlayerNumber::Four, PlayerNumber::Three] {
            let mut board = kickoff();
            let kicker_pose = board.world_state.robot.ground_to_field.unwrap().as_pose();
            board.world_state.player_states.five = Some(TimeWrapper {
                time: board.world_state.now,
                inner: PlayerState {
                    pose: kicker_pose,
                    ..crate::test_support::player_state()
                },
            });
            let pose = ready_pose(&board, player);
            board.player_number = player;
            board.world_state.robot.ground_to_field = Some(pose.as_transform());
            assert_eq!(
                kickoff_subtree().tick_with_trace(&mut board).0,
                Status::Success
            );
            assert_eq!(board.body_motion, Some(BodyMotion::Stand));
        }
    }

    #[test]
    fn ready_walk_in_keeps_base_slots_despite_passing_and_peer_updates() {
        for team in [Team::Hulks, Team::Opponent] {
            for player in [
                PlayerNumber::One,
                PlayerNumber::Two,
                PlayerNumber::Three,
                PlayerNumber::Four,
                PlayerNumber::Five,
            ] {
                let mut reference = kickoff();
                reference.player_number = player;
                reference.world_state.robot.primary_state = PrimaryState::Ready;
                let game = reference
                    .world_state
                    .filtered_game_controller_state
                    .as_mut()
                    .unwrap();
                game.game_state = FilteredGameState::Ready;
                game.kicking_team = Some(team);
                let mut expected = reference.clone();
                expected
                    .parameters
                    .kicking
                    .strategy
                    .passing
                    .maximum_distance = 0.0;
                assert_eq!(
                    crate::tree::create_tree().tick_with_trace(&mut expected).0,
                    Status::Success
                );
                assert!(matches!(
                    expected.body_motion,
                    Some(BodyMotion::Walk { .. })
                ));
                for peer_age in [Some(Duration::ZERO), Some(Duration::from_secs(3)), None] {
                    let mut board = reference.clone();
                    if let Some(age) = peer_age {
                        board.world_state.player_states.four.as_mut().unwrap().time =
                            board.world_state.now - age;
                    } else {
                        board.world_state.player_states.four = None;
                    }
                    assert_eq!(
                        crate::tree::create_tree().tick_with_trace(&mut board).0,
                        Status::Success
                    );
                    assert_eq!(
                        board.body_motion, expected.body_motion,
                        "{team:?}, {player:?}, {peer_age:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn kickoff_rejects_missing_stale_ineligible_or_blocked_receiver_and_holds() {
        let board = kickoff();
        for case in 0..7 {
            let mut invalid = board.clone();
            match case {
                0 => invalid.world_state.player_states.four = None,
                1 => {
                    invalid
                        .world_state
                        .player_states
                        .four
                        .as_mut()
                        .unwrap()
                        .time = invalid.world_state.now - Duration::from_secs(3)
                }
                2 => {
                    invalid
                        .world_state
                        .player_states
                        .four
                        .as_mut()
                        .unwrap()
                        .time = invalid.world_state.now + Duration::from_secs(1)
                }
                3 => {
                    invalid
                        .world_state
                        .filtered_game_controller_state
                        .as_mut()
                        .unwrap()
                        .penalties
                        .four = Some(hsl_network_messages::Penalty::PickUp {
                        remaining: Duration::from_secs(1),
                    })
                }
                5 => invalid.field_dimensions.length = f32::NAN,
                6 => invalid.field_dimensions.width = 0.0,
                _ => {
                    let target = kickoff_pass_target(&invalid, PlayerNumber::Four).unwrap();
                    let transform = invalid.world_state.robot.ground_to_field.unwrap();
                    invalid.world_state.obstacles.push(Obstacle::robot(
                        transform.inverse() * (target / 2.0),
                        0.6,
                        0.6,
                    ));
                }
            }
            assert!(
                kickoff_pass_target(&invalid, PlayerNumber::Four).is_none(),
                "case {case}"
            );
            assert_eq!(
                kickoff_subtree().tick_with_trace(&mut invalid).0,
                Status::Success
            );
            assert_eq!(invalid.body_motion, Some(BodyMotion::Stand));
            assert!(invalid.ball_action.is_none());
        }
        let mut absent = board;
        absent.world_state.player_states.four = None;
        absent.world_state.player_states.three = Some(TimeWrapper {
            time: absent.world_state.now,
            inner: PlayerState {
                pose: ready_pose(&absent, PlayerNumber::Three),
                ..crate::test_support::player_state()
            },
        });
        assert!(kickoff_receiver(&absent).is_none());
        assert!(is_own_kickoff(&mut absent));
        absent.world_state.now = absent.world_state.now + Duration::from_secs(2);
        assert!(!is_own_kickoff(&mut absent));
    }

    #[test]
    fn ready_set_do_not_send_messages_and_kickoff_lifecycle_survives_controller_release() {
        let mut board = kickoff();
        for state in [FilteredGameState::Ready, FilteredGameState::Set] {
            board
                .world_state
                .filtered_game_controller_state
                .as_mut()
                .unwrap()
                .game_state = state;
            board.world_state.robot.primary_state = if state == FilteredGameState::Ready {
                PrimaryState::Ready
            } else {
                PrimaryState::Set
            };
            board.world_state.now = board.world_state.now + Duration::from_secs(2);
            board.update_kickoff();
            assert!(board.kickoff_prepared);
            assert!(board.try_sending_state_message().is_none());
        }
        board.world_state.robot.primary_state = PrimaryState::Playing;
        board
            .world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .game_state = FilteredGameState::Playing {
            ball_is_free: true,
            kick_off: false,
        };
        board.update_kickoff();
        assert!(board.kickoff_started.is_some());
        assert!(own_kickoff(&board));
        assert!(board.try_sending_state_message().is_some());
        board.ball.as_mut().unwrap().position += vector![0.6, 0.0];
        board.update_kickoff();
        assert!(board.kickoff_started.is_none());
        assert!(!own_kickoff(&board));
    }
}
