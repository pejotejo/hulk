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

pub fn kickoff_receiver(blackboard: &Blackboard) -> PlayerNumber {
    if blackboard.parameters.kickoff.pass_to_left {
        PlayerNumber::Five
    } else {
        PlayerNumber::Four
    }
}

pub fn kickoff_receiver_position(blackboard: &Blackboard) -> Point2<Field> {
    let y = blackboard.parameters.kickoff.standard_positions[kickoff_receiver(blackboard)]
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
        && kickoff_receiver(blackboard) != blackboard.parameters.goalkeeper.player_number
        && blackboard
            .world_state
            .filtered_game_controller_state
            .as_ref()
            .is_some_and(|game| game.penalties[kickoff_receiver(blackboard)].is_none());
    if !active || receiver_position(blackboard, kickoff_receiver(blackboard)).is_some() {
        blackboard.kickoff_receiver_wait_started = None;
        return active;
    }
    let now = blackboard.world_state.now;
    let since = *blackboard.kickoff_receiver_wait_started.get_or_insert(now);
    now.duration_since(since) < Duration::from_secs(2)
}

pub(crate) fn needs_kickoff_updates(blackboard: &Blackboard) -> bool {
    own_kickoff(blackboard)
        && (blackboard.world_state.robot.player_number == PlayerNumber::Three
            || blackboard.world_state.robot.player_number == kickoff_receiver(blackboard))
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
    if receiver != kickoff_receiver(blackboard)
        || receiver == blackboard.parameters.goalkeeper.player_number
    {
        return None;
    }
    let world = &blackboard.world_state;
    let game = world.filtered_game_controller_state.as_ref()?;
    if game.penalties[receiver].is_some() {
        return None;
    }
    if receiver == world.robot.player_number {
        return world
            .robot
            .ground_to_field
            .map(|transform| transform.as_pose().position());
    }
    let state = world.player_states[receiver].as_ref()?;
    let parameters = &blackboard.parameters.kicking.strategy.passing;
    (state.time <= world.now
        && world.now.duration_since(state.time) < parameters.maximum_receiver_age)
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
    receiver_position(blackboard, kickoff_receiver(blackboard)).is_none_or(|position| {
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
        && player != kickoff_receiver(blackboard)
}

fn is_kickoff_receiver(blackboard: &mut Blackboard) -> bool {
    blackboard.world_state.robot.player_number == kickoff_receiver(blackboard)
        && blackboard.world_state.robot.player_number
            != blackboard.parameters.goalkeeper.player_number
}

fn select_kickoff_pass(blackboard: &mut Blackboard) -> Status {
    blackboard.ball_action = None;
    blackboard.kick_target = None;
    let receiver = kickoff_receiver(blackboard);
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

    use linear_algebra::Pose2;
    use types::{
        field_dimensions::{FieldDimensions, Side},
        filtered_game_controller_state::FilteredGameControllerState,
        obstacles::Obstacle,
        time_wrapper::TimeWrapper,
        world_state::{BallSource, PlayerState},
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
        board.world_state.robot.ground_to_field = Some(Default::default());
        board.world_state.filtered_game_controller_state = Some(FilteredGameControllerState {
            game_state: FilteredGameState::Playing {
                ball_is_free: true,
                kick_off: true,
            },
            kicking_team: Some(Team::Hulks),
            remaining_number_of_messages: 100,
            ..Default::default()
        });
        board.parameters.kickoff.standard_positions.four.position = point![-2.5, -1.5];
        board.parameters.kickoff.standard_positions.five.position = point![-2.5, 1.5];
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
                soft: true,
                strong: false,
                ..
            })
        ));

        // A missing update delays the side pass rather than starting an ordinary shot.
        board.world_state.player_states.four = None;
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
        assert!(is_own_kickoff(&mut board));
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
        assert_eq!(kickoff_receiver(&board), PlayerNumber::Four);
        let right = kickoff_pass_target(&board, PlayerNumber::Four).unwrap();
        assert!(right.x() < 0.0 && right.y() < 0.0);
        board.parameters.kickoff.pass_to_left = true;
        assert_eq!(kickoff_receiver(&board), PlayerNumber::Five);
        let left = kickoff_pass_target(&board, PlayerNumber::Five).unwrap();
        assert_eq!(left.x(), right.x());
        assert_eq!(left.y(), -right.y());
        assert!(kickoff_pass_target(&board, PlayerNumber::Four).is_none());
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
