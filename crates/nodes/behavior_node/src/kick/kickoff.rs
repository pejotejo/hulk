use std::time::Duration;

use coordinate_systems::Field;
use geometry::{Distance, line_segment::LineSegment};
use hsl_network_messages::{PlayerNumber, Team};
use linear_algebra::{Point2, point};
use types::{
    behavior_tree::Status, filtered_game_state::FilteredGameState, motion_type::MotionType,
    obstacles::ObstacleKind,
};

use crate::{
    action, actions::stand, behavior_tree::Node, condition, head::look_at_ball_subtree,
    node::Blackboard, selection, sequence, subtree, switch_motion_type::switch_motion_type,
    walk::walk_to_kickoff_pose,
};

use super::{execution::execute_ball_action, strategy::BallAction};

pub fn kickoff_receiver(blackboard: &Blackboard) -> Option<PlayerNumber> {
    let candidates = if blackboard.parameters.kickoff.pass_to_left {
        [PlayerNumber::Five, PlayerNumber::Two, PlayerNumber::Four]
    } else {
        [PlayerNumber::Four, PlayerNumber::Two, PlayerNumber::Five]
    };
    candidates.into_iter().find(|&player| {
        player != blackboard.parameters.goalkeeper.player_number
            && player_position(blackboard, player, Duration::from_secs(2)).is_some()
    })
}

pub fn kickoff_receiver_position(blackboard: &Blackboard) -> Point2<Field> {
    let side = if blackboard.parameters.kickoff.pass_to_left {
        PlayerNumber::Five
    } else {
        PlayerNumber::Four
    };
    let y = blackboard.parameters.kickoff.standard_positions[side]
        .position
        .y()
        .abs();
    point![
        -0.3,
        if blackboard.parameters.kickoff.pass_to_left {
            y
        } else {
            -y
        }
    ]
}

fn is_own_kickoff(blackboard: &mut Blackboard) -> bool {
    let active = own_kickoff(blackboard)
        && blackboard.has_team_message_budget()
        && blackboard.world_state.robot.player_number
            != blackboard.parameters.goalkeeper.player_number
        && blackboard.parameters.goalkeeper.player_number != PlayerNumber::Three
        && player_position(blackboard, PlayerNumber::Three, Duration::from_secs(2)).is_some();
    if !active || kickoff_receiver(blackboard).is_some() {
        blackboard.kickoff_receiver_wait_started = None;
        return active;
    }
    if blackboard.world_state.robot.player_number != PlayerNumber::Three {
        return false;
    }
    let now = blackboard.world_state.now;
    let since = *blackboard.kickoff_receiver_wait_started.get_or_insert(now);
    now.duration_since(since) < Duration::from_secs(2)
}

pub(crate) fn needs_kickoff_updates(blackboard: &Blackboard) -> bool {
    blackboard.world_state.robot.player_number != blackboard.parameters.goalkeeper.player_number
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

fn own_kickoff(blackboard: &Blackboard) -> bool {
    blackboard
        .world_state
        .filtered_game_controller_state
        .as_ref()
        .is_some_and(|game| {
            game.kicking_team == Some(Team::Hulks)
                && game.sub_state.is_none()
                && matches!(
                    game.game_state,
                    FilteredGameState::Playing { kick_off: true, .. }
                )
        })
        && blackboard
            .ball
            .as_ref()
            .is_some_and(|ball| ball.position.coords().norm() < 0.35)
}

fn receiver_position(blackboard: &Blackboard, receiver: PlayerNumber) -> Option<Point2<Field>> {
    if Some(receiver) != kickoff_receiver(blackboard) {
        return None;
    }
    player_position(
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

fn player_position(
    blackboard: &Blackboard,
    player: PlayerNumber,
    maximum_age: Duration,
) -> Option<Point2<Field>> {
    let world = &blackboard.world_state;
    let game = world.filtered_game_controller_state.as_ref()?;
    if game.penalties[player].is_some() {
        return None;
    }
    if player == world.robot.player_number {
        return world
            .robot
            .ground_to_field
            .map(|transform| transform.as_pose().position());
    }
    let state = world.player_states[player].as_ref()?;
    (state.time <= world.now && world.now.duration_since(state.time) < maximum_age)
        .then_some(state.inner.pose.position())
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
    let position = receiver_position(blackboard, receiver)?;
    if (position - kickoff_receiver_position(blackboard)).norm()
        > parameters.passing.receiver_standoff
    {
        return None;
    }
    let ball = blackboard.ball.as_ref()?.position;
    let delta = position - ball;
    let distance = delta.norm();
    if !distance.is_finite()
        || distance <= parameters.passing.receiver_standoff
        || distance > parameters.passing.maximum_distance
    {
        return None;
    }
    let target = position - delta / distance * parameters.passing.receiver_standoff;
    let field = &blackboard.field_dimensions;
    if !field.length.is_finite()
        || !field.width.is_finite()
        || target.x().abs() >= field.length / 2.0
        || target.y().abs() >= field.width / 2.0
    {
        return None;
    }
    let transform = blackboard.world_state.robot.ground_to_field?;
    let path = LineSegment(ball, target);
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
    blackboard.world_state.robot.player_number == PlayerNumber::Three
        && blackboard.has_team_message_budget()
}

fn is_waiting_for_receiver(blackboard: &mut Blackboard) -> bool {
    kickoff_receiver(blackboard)
        .and_then(|receiver| receiver_position(blackboard, receiver))
        .is_none_or(|position| {
            (position - kickoff_receiver_position(blackboard)).norm()
                > blackboard
                    .parameters
                    .kicking
                    .strategy
                    .passing
                    .receiver_standoff
        })
}

fn is_other_field_player(blackboard: &mut Blackboard) -> bool {
    let player = blackboard.world_state.robot.player_number;
    player != PlayerNumber::Three
        && player != blackboard.parameters.goalkeeper.player_number
        && Some(player) != kickoff_receiver(blackboard)
}

fn is_kickoff_receiver(blackboard: &mut Blackboard) -> bool {
    Some(blackboard.world_state.robot.player_number) == kickoff_receiver(blackboard)
        && blackboard.world_state.robot.player_number
            != blackboard.parameters.goalkeeper.player_number
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
                    sequence!(condition!(is_waiting_for_receiver), action!(stand)),
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

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use hsl_network_messages::{HulkMessage, Penalty};
    use linear_algebra::Pose2;
    use types::{
        fall_detection::{FallDetection, Posture},
        field_dimensions::{FieldDimensions, Side},
        filtered_game_controller_state::FilteredGameControllerState,
        messages::OutgoingMessage,
        motion_command::BodyMotion,
        obstacles::Obstacle,
        path::PathSegment,
        primary_state::PrimaryState,
        time_wrapper::TimeWrapper,
        world_state::{BallSource, BallState, PlayerState},
    };

    use crate::node::LastBall;

    use super::*;

    fn blackboard() -> Blackboard {
        let mut board = Blackboard {
            field_dimensions: FieldDimensions::SPL_2025,
            ball: Some(LastBall {
                position: Point2::origin(),
                velocity: Default::default(),
                age: Default::default(),
                field_side: Side::Right,
                source: BallSource::Own,
            }),
            ..Default::default()
        };
        board.parameters.goalkeeper.player_number = PlayerNumber::One;
        board.world_state.robot.ground_to_field = Some(Default::default());
        board.world_state.fall_detection = Some(FallDetection {
            time: board.world_state.now,
            valid_until: board.world_state.now + Duration::from_secs(60),
            posture: Posture::Upright,
        });
        board.world_state.filtered_game_controller_state = Some(FilteredGameControllerState {
            game_state: FilteredGameState::Playing {
                ball_is_free: true,
                kick_off: true,
            },
            kicking_team: Some(Team::Hulks),
            remaining_number_of_messages: 100,
            remaining_time_in_half: Duration::from_secs(1200),
            ..Default::default()
        });
        board.parameters.kickoff.standard_positions.four.position = point![-2.5, -1.5];
        board.parameters.kickoff.standard_positions.five.position = point![-2.5, 1.5];
        board
            .parameters
            .walking
            .walk_and_stand
            .target_reached_thresholds = nalgebra::vector![0.05, 0.05];
        board.parameters.walking.speed.velocity_fade_distance = 3.0;
        board.parameters.kicking.strategy = types::parameters::KickStrategyParameters {
            maximum_shot_distance: 3.0,
            maximum_dribble_distance: 1.0,
            obstacle_clearance: 0.1,
            minimum_shot_angle: 0.02,
            shot_angle_hysteresis: 0.05,
            pressure_distance: 0.8,
            clear_goal_distance: 1.5,
            clear_distance_hysteresis: 0.2,
            passing: types::parameters::PassingParameters {
                maximum_distance: 3.0,
                minimum_progress: 0.5,
                receiver_standoff: 0.5,
                target_half_width: 0.2,
                selection_hysteresis: 0.25,
                maximum_receiver_age: Duration::from_millis(500),
            },
        };
        for receiver in [PlayerNumber::Four, PlayerNumber::Five] {
            let y = if receiver == PlayerNumber::Four {
                -1.5
            } else {
                1.5
            };
            board.world_state.player_states[receiver] = Some(TimeWrapper {
                time: board.world_state.now,
                inner: PlayerState {
                    pose: Pose2::new(point![-0.3, y], 0.0),
                    ..Default::default()
                },
            });
        }
        board
    }

    #[test]
    fn kickoff_waits_for_fresh_receiver_and_announces_before_adaptive_cooldown() {
        let mut board = blackboard();
        board.world_state.robot.player_number = PlayerNumber::Three;
        board.world_state.robot.primary_state = types::primary_state::PrimaryState::Playing;
        board.parameters.kicking.kick_activation_distance = 1.3;
        board
            .world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .remaining_time_in_half = Duration::from_secs(1200);
        board.last_sent_hsl_message_time = Some(board.world_state.now);
        assert_eq!(select_kickoff_pass(&mut board), Status::Success);
        assert_eq!(execute_ball_action(&mut board), Status::Success);
        assert_eq!(
            board.body_motion,
            Some(types::motion_command::BodyMotion::Stand)
        );
        board.world_state.now = board.world_state.now + Duration::from_millis(251);
        assert!(board.try_sending_state_message().is_some());
        board.world_state.now = board.world_state.now + Duration::from_millis(150);
        assert_eq!(execute_ball_action(&mut board), Status::Success);
        assert!(matches!(
            board.body_motion,
            Some(types::motion_command::BodyMotion::Kick {
                soft: false,
                strong: false,
                ..
            })
        ));

        // A missing update delays the side pass rather than starting an ordinary shot.
        board.world_state.player_states.four = None;
        board.world_state.player_states.five = None;
        let (status, _) = kickoff_subtree().tick_with_trace(&mut board);
        assert_eq!(status, Status::Success);
        assert_eq!(
            board.body_motion,
            Some(types::motion_command::BodyMotion::Stand)
        );
        assert_eq!(board.ball_action, None);
        board.world_state.now = board.world_state.now + Duration::from_secs(2);
        let (status, _) = kickoff_subtree().tick_with_trace(&mut board);
        assert_eq!(status, Status::Failure);
        board.world_state.robot.player_number = PlayerNumber::Four;
        assert!(!is_own_kickoff(&mut board));
        board
            .world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .remaining_number_of_messages = 0;
        assert!(board.try_sending_state_message().is_none());
    }

    #[test]
    fn side_parameter_mirrors_receiver_and_backward_pass() {
        let mut board = blackboard();
        assert_eq!(kickoff_receiver(&board), Some(PlayerNumber::Four));
        let right = kickoff_pass_target(&board, PlayerNumber::Four).unwrap();
        assert!(right.x() < 0.0 && right.y() < 0.0);
        board.parameters.kickoff.pass_to_left = true;
        assert_eq!(kickoff_receiver(&board), Some(PlayerNumber::Five));
        let left = kickoff_pass_target(&board, PlayerNumber::Five).unwrap();
        assert_eq!(left.x(), right.x());
        assert_eq!(left.y(), -right.y());
        assert!(kickoff_pass_target(&board, PlayerNumber::Four).is_none());
    }

    #[test]
    fn available_receiver_prepares_during_ready_and_gets_kickoff_pass() {
        let mut striker = blackboard();
        striker.world_state.robot.player_number = PlayerNumber::Three;
        striker.world_state.robot.primary_state = PrimaryState::Ready;
        striker
            .world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .game_state = FilteredGameState::Ready;
        striker.world_state.player_states.four = None;
        striker.world_state.player_states.five = None;
        striker.world_state.player_states.two = Some(TimeWrapper {
            time: striker.world_state.now,
            inner: PlayerState {
                pose: Pose2::new(point![-3.0, -0.5], 0.0),
                ..Default::default()
            },
        });
        let mut receiver = striker.clone();
        receiver.world_state.robot.player_number = PlayerNumber::Two;
        receiver.world_state.robot.ground_to_field =
            Some(Pose2::new(point![-3.0, -0.5], 0.0).as_transform());
        receiver.world_state.player_states.three = Some(TimeWrapper {
            time: receiver.world_state.now,
            inner: PlayerState::default(),
        });
        receiver.ball = None;
        assert_eq!(kickoff_receiver(&striker), Some(PlayerNumber::Two));
        assert_eq!(kickoff_receiver(&receiver), Some(PlayerNumber::Two));

        let tree = crate::tree::create_tree();
        assert_eq!(tree.tick_with_trace(&mut receiver).0, Status::Success);
        let Some(BodyMotion::Walk { ref path, .. }) = receiver.body_motion else {
            panic!("kickoff receiver should walk during Ready");
        };
        let PathSegment::LineSegment(segment) = path.last_segment() else {
            panic!("expected straight final approach");
        };
        let target = kickoff_receiver_position(&receiver);
        let actual = receiver.world_state.robot.ground_to_field.unwrap() * segment.1;
        assert!((actual - target).norm() < 0.001);

        // Ready discovers available teammates without publishing an old pass intent.
        receiver.ball_action = Some(BallAction::Pass {
            receiver: PlayerNumber::Four,
        });
        receiver.kick_target = Some(point![1.0, 1.0]);
        let Some(OutgoingMessage::Hsl(HulkMessage::State(message))) =
            receiver.try_sending_state_message()
        else {
            panic!("receiver must announce its Ready position");
        };
        assert_eq!(message.pass, None);

        receiver.world_state.now = receiver.world_state.now + Duration::from_millis(251);
        receiver.world_state.robot.primary_state = PrimaryState::Set;
        receiver
            .world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .game_state = FilteredGameState::Set;
        assert!(receiver.try_sending_state_message().is_some());

        let pose = Pose2::new(target, (-target.y()).atan2(-target.x()));
        receiver.world_state.robot.ground_to_field = Some(pose.as_transform());
        receiver.world_state.robot.primary_state = PrimaryState::Playing;
        receiver
            .world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .game_state = FilteredGameState::Playing {
            ball_is_free: true,
            kick_off: true,
        };
        receiver.ball = striker.ball.clone();
        receiver.ball_action = None;
        receiver.kick_target = None;
        assert_eq!(tree.tick_with_trace(&mut receiver).0, Status::Success);
        assert_eq!(receiver.body_motion, Some(BodyMotion::Stand));

        striker.world_state.now = receiver.world_state.now;
        striker.world_state.robot.primary_state = PrimaryState::Playing;
        striker
            .world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .game_state = FilteredGameState::Playing {
            ball_is_free: true,
            kick_off: true,
        };
        striker.world_state.ball = Some(BallState::default());
        striker.world_state.player_states.two = Some(TimeWrapper {
            time: striker.world_state.now,
            inner: PlayerState {
                pose,
                ..Default::default()
            },
        });
        striker.parameters.kicking.kick_activation_distance = 1.3;
        assert_eq!(tree.tick_with_trace(&mut striker).0, Status::Success);
        assert_eq!(
            striker.ball_action,
            Some(BallAction::Pass {
                receiver: PlayerNumber::Two
            })
        );
        assert_eq!(striker.body_motion, Some(BodyMotion::Stand));
        assert!(striker.try_sending_state_message().is_some());
        striker.world_state.now = striker.world_state.now + Duration::from_millis(150);
        assert_eq!(tree.tick_with_trace(&mut striker).0, Status::Success);
        assert!(matches!(
            striker.body_motion,
            Some(BodyMotion::Kick {
                soft: false,
                strong: false,
                ..
            })
        ));
    }

    #[test]
    fn receiver_selection_excludes_goalkeeper_penalties_and_missing_players() {
        let mut board = blackboard();
        board.world_state.robot.player_number = PlayerNumber::Three;
        board.world_state.player_states.two = board.world_state.player_states.four.clone();
        board
            .world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .penalties
            .four = Some(Penalty::default());
        assert_eq!(kickoff_receiver(&board), Some(PlayerNumber::Two));
        board.parameters.goalkeeper.player_number = PlayerNumber::Two;
        assert_eq!(kickoff_receiver(&board), Some(PlayerNumber::Five));
        board.world_state.player_states.five = None;
        assert_eq!(kickoff_receiver(&board), None);
        assert!(is_own_kickoff(&mut board));
        board.world_state.now = board.world_state.now + Duration::from_secs(2);
        assert!(!is_own_kickoff(&mut board));

        let mut board = blackboard();
        board.world_state.now = board.world_state.now + Duration::from_secs(2);
        assert_eq!(kickoff_receiver(&board), None);
        board.world_state.robot.player_number = board.parameters.goalkeeper.player_number;
        assert!(!is_own_kickoff(&mut board));
        board.world_state.robot.player_number = PlayerNumber::Three;
        board.world_state.robot.primary_state = PrimaryState::Ready;
        board
            .world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .kicking_team = Some(Team::Opponent);
        assert!(board.try_sending_state_message().is_none());
    }

    #[test]
    fn kickoff_pass_rejects_stale_receiver_blocked_lane_and_other_restarts() {
        let mut board = blackboard();
        board.world_state.now = board.world_state.now + Duration::from_millis(500);
        assert!(kickoff_pass_target(&board, PlayerNumber::Four).is_none());
        let mut board = blackboard();
        board
            .world_state
            .obstacles
            .push(Obstacle::robot(point![-0.1, -0.5], 0.2, 0.2));
        assert!(kickoff_pass_target(&board, PlayerNumber::Four).is_none());
        let mut board = blackboard();
        board
            .world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .kicking_team = Some(Team::Opponent);
        assert!(kickoff_pass_target(&board, PlayerNumber::Four).is_none());
        let mut board = blackboard();
        board.ball.as_mut().unwrap().position = point![0.4, 0.0];
        assert!(kickoff_pass_target(&board, PlayerNumber::Four).is_none());
    }
}
