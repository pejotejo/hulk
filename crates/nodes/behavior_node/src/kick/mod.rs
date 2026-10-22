pub mod execution;
pub mod geometry;
pub mod kickoff;
pub mod passing;
pub mod reception;
pub mod strategy;

use ::geometry::line_segment::LineSegment;
use coordinate_systems::{Field, Ground};
use hsl_network_messages::PlayerNumber;
use linear_algebra::{Isometry2, Orientation2, Point2, Rotation2, Vector2, point, vector};
use types::{
    behavior_tree::Status,
    motion_command::{BodyMotion, HeadMotion, ImageRegion, KickPower, MotionCommand},
    motion_type::MotionType,
};

use crate::{
    action,
    actions::stand,
    behavior_tree::Node,
    condition,
    node::Blackboard,
    selection, sequence, subtree,
    switch_motion_type::{is_last_motion_type, switch_motion_type},
    walk::walk_to_ball,
};

pub fn kick_subtree() -> Node<Blackboard> {
    switch_motion_type(
        MotionType::Kick,
        sequence!(
            action!(kick),
            action!(select_kick_target),
            subtree!(kick_strength_subtree),
        ),
        subtree!(kick_alternatives_subtree),
    )
}

pub fn kick_alternatives_subtree() -> Node<Blackboard> {
    selection!(
        sequence!(
            condition!(is_last_motion_type, MotionType::Walk),
            action!(walk_to_ball)
        ),
        action!(stand)
    )
}

pub fn kick(blackboard: &mut Blackboard) -> Status {
    blackboard.ball_action = None;
    blackboard.kick_target = None;
    if let Some(ground_to_field) = &blackboard.world_state.robot.ground_to_field
        && let Some((ball_in_ground, _ball_velocity)) =
            kick_ball_in_ground(blackboard, ground_to_field)
    {
        blackboard.deflecting_ball = false;
        blackboard.body_motion = Some(kick_motion(
            blackboard,
            ball_in_ground,
            ball_in_ground + vector![1.0, 0.0],
            false,
        ));
        if blackboard.last_motion_type == Some(MotionType::Kick) {
            use_last_kick_settings(blackboard);
        }
        blackboard.head_motion = Some(HeadMotion::LookAt {
            target: ball_in_ground,
            image_region_target: ImageRegion::Center,
        });

        Status::Success
    } else {
        Status::Failure
    }
}

pub fn select_kick_target(blackboard: &mut Blackboard) -> Status {
    let field = &blackboard.field_dimensions;
    blackboard.ball_action = Some(strategy::BallAction::Shoot {
        opening: LineSegment(
            point![field.length / 2.0, -field.goal_inner_width / 2.0],
            point![field.length / 2.0, field.goal_inner_width / 2.0],
        ),
    });
    let Some(target) = execution::resolve_target(blackboard) else {
        blackboard.ball_action = None;
        blackboard.kick_target = None;
        stand(blackboard);
        return Status::Failure;
    };
    apply_kick_target(blackboard, target)
}

fn kick_target_offset_angle(blackboard: &Blackboard) -> f32 {
    if blackboard.remote_control_enabled
        || blackboard.deflecting_ball
        || blackboard.player_number == blackboard.parameters.goalkeeper.player_number
        || blackboard.replacement_goalkeeper_coordinator.keeper() == Some(blackboard.player_number)
    {
        0.0
    } else {
        blackboard.parameters.kicking.kick_target_offset_angle
    }
}

pub(super) fn kick_motion(
    blackboard: &Blackboard,
    ball: Point2<Ground>,
    target: Point2<Ground>,
    strong: bool,
) -> BodyMotion {
    BodyMotion::VisualKick {
        ball_position: ball,
        kick_direction: Orientation2::from_vector(target - ball),
        // Preserve the SDK target calibration independently of the tactical corridor.
        target_position: Rotation2::new(kick_target_offset_angle(blackboard)) * target,
        robot_theta_to_field: blackboard
            .world_state
            .robot
            .ground_to_field
            .unwrap()
            .orientation(),
        kick_power: if strong {
            KickPower::Schlong
        } else {
            KickPower::Rumpelstilzchen
        },
    }
}

pub fn apply_kick_target(
    blackboard: &mut Blackboard,
    target_position_in_field: Point2<Field>,
) -> Status {
    let offset = kick_target_offset_angle(blackboard);
    if let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field
        && let Some(BodyMotion::VisualKick {
            ball_position,
            kick_direction,
            target_position: motion_target,
            ..
        }) = blackboard.body_motion.as_mut()
    {
        let target_position = ground_to_field.inverse() * target_position_in_field;
        *kick_direction = Orientation2::from_vector(target_position - *ball_position);
        *motion_target = Rotation2::new(offset) * target_position;
        blackboard.kick_target = Some(target_position);
        return Status::Success;
    }
    Status::Failure
}

pub fn kick_strength_subtree() -> Node<Blackboard> {
    // A long clearance must not inherit a preceding interception's weaker flag.
    selection!(
        sequence!(
            condition!(is_target_in_strong_kick_range),
            condition!(allow_schlong),
            action!(use_strong_kick)
        ),
        sequence!(
            condition!(is_last_motion_type, MotionType::Kick),
            action!(use_last_kick_settings)
        ),
        action!(disable_strong_kick)
    )
}

pub fn is_target_in_strong_kick_range(blackboard: &mut Blackboard) -> bool {
    if let Some(target_position) = blackboard.kick_target
        && let Some(BodyMotion::VisualKick { ball_position, .. }) = &blackboard.body_motion
    {
        (target_position - *ball_position).norm()
            >= blackboard
                .parameters
                .kicking
                .target_distance_kick_power_threshold
    } else {
        false
    }
}

pub fn allow_schlong(blackboard: &mut Blackboard) -> bool {
    blackboard.parameters.kicking.allow_schlong
}

pub fn use_last_kick_settings(blackboard: &mut Blackboard) -> Status {
    if let Some(MotionCommand::VisualKick { kick_power, .. }) = blackboard.last_motion_command
        && let Some(BodyMotion::VisualKick {
            kick_power: power, ..
        }) = blackboard.body_motion.as_mut()
    {
        *power = kick_power;
        return Status::Success;
    }
    Status::Failure
}

pub fn use_kick(blackboard: &mut Blackboard, strong: bool) -> Status {
    if let Some(BodyMotion::VisualKick { kick_power, .. }) = blackboard.body_motion.as_mut() {
        *kick_power = if strong {
            KickPower::Schlong
        } else {
            KickPower::Rumpelstilzchen
        };
        return Status::Success;
    }
    Status::Failure
}

pub fn use_strong_kick(blackboard: &mut Blackboard) -> Status {
    use_kick(blackboard, true)
}

pub fn disable_strong_kick(blackboard: &mut Blackboard) -> Status {
    use_kick(blackboard, false)
}

pub fn intercept(blackboard: &mut Blackboard) -> Status {
    let Some(transform) = blackboard.world_state.robot.ground_to_field else {
        return Status::Failure;
    };
    let Some((position, velocity)) = kick_ball_in_ground(blackboard, &transform) else {
        return Status::Failure;
    };
    if velocity.norm() < f32::EPSILON {
        return Status::Failure;
    }
    let time = -position.coords().dot(&velocity) / velocity.norm_squared();
    if time < 0.0 {
        return Status::Failure;
    }
    let contact = position + velocity * time;
    if contact.x()
        < blackboard
            .parameters
            .kicking
            .minimum_interception_forward_distance
        || contact.coords().norm()
            > blackboard
                .parameters
                .ball
                .interception
                .maximum_intercept_distance
    {
        return Status::Failure;
    }
    if !matches!(blackboard.body_motion, Some(BodyMotion::VisualKick { .. })) {
        return Status::Failure;
    }
    blackboard.kick_target = Some(position);
    blackboard.body_motion = Some(kick_motion(blackboard, contact, position, false));
    Status::Success
}

pub fn set_kick_target_beyond_ball(blackboard: &mut Blackboard) -> Status {
    let Some(transform) = blackboard.world_state.robot.ground_to_field else {
        return Status::Failure;
    };
    let Some(ball) = kick_ball_position_in_ground(blackboard, &transform) else {
        return Status::Failure;
    };
    if blackboard.last_motion_type != Some(MotionType::Kick) {
        let Some(direction) = ball.coords().try_normalize(f32::EPSILON) else {
            return Status::Failure;
        };
        blackboard.last_kick_target = Some(transform * (ball + direction * 10.0));
    }
    if let Some(target) = blackboard.last_kick_target {
        return apply_kick_target(blackboard, target);
    }
    Status::Failure
}

pub(crate) fn kick_ball_position_in_ground(
    blackboard: &Blackboard,
    ground_to_field: &Isometry2<Ground, Field>,
) -> Option<Point2<Ground>> {
    kick_ball_in_ground(blackboard, ground_to_field).map(|(position, _)| position)
}

pub(crate) fn kick_ball_in_ground(
    blackboard: &Blackboard,
    ground_to_field: &Isometry2<Ground, Field>,
) -> Option<(Point2<Ground>, Vector2<Ground>)> {
    let velocity = blackboard
        .world_state
        .ball
        .map(|ball| ball.ball_in_ground_velocity)
        .or_else(|| {
            blackboard
                .ball
                .as_ref()
                .map(|ball| ground_to_field.inverse() * ball.velocity)
        })
        .filter(|velocity| velocity.inner.iter().all(|value| value.is_finite()))
        .unwrap_or_else(Vector2::zeros);
    // Percepts provide the close position; their selector does not estimate velocity.
    blackboard
        .visual_kick_ball_position
        .as_ref()
        .filter(|ball| {
            ball.age_at(blackboard.world_state.now)
                .is_some_and(|age| age <= std::time::Duration::from_millis(100))
                && ball.position.inner.iter().all(|value| value.is_finite())
                && ball.position.coords().norm()
                    <= blackboard.parameters.kicking.distance_for_kick
                        + blackboard.parameters.kicking.distance_for_kick_hysteresis
        })
        .map(|ball| (ball.position, velocity))
        .or_else(|| {
            blackboard
                .ball
                .as_ref()
                .map(|ball| (ground_to_field.inverse() * ball.position, velocity))
        })
}

pub(super) fn pass_receiver_position(
    blackboard: &Blackboard,
    ball: Point2<Field>,
    receiver: PlayerNumber,
) -> Option<Point2<Field>> {
    let world = &blackboard.world_state;
    let parameters = &blackboard.parameters.kicking.strategy.passing;
    if receiver == blackboard.player_number
        || receiver == blackboard.parameters.goalkeeper.player_number
        || Some(receiver) == blackboard.replacement_goalkeeper_coordinator.keeper()
        || world
            .filtered_game_controller_state
            .as_ref()
            .is_some_and(|state| state.penalties[receiver].is_some())
    {
        return None;
    }
    let state = world.player_states[receiver].as_ref()?;
    if state.time > world.now
        || world.now.duration_since(state.time) >= parameters.maximum_receiver_age
    {
        return None;
    }
    let position = state.inner.pose.position();
    let field = &blackboard.field_dimensions;
    (position
        .inner
        .iter()
        .all(|coordinate| coordinate.is_finite())
        && position.x().abs() < field.length / 2.0
        && position.y().abs() < field.width / 2.0
        && (position.x() - ball.x() >= parameters.minimum_progress
            || passing::is_executing_announced_pass(blackboard, receiver)
            || world
                .filtered_game_controller_state
                .as_ref()
                .is_some_and(|game| {
                    game.kicking_team == Some(hsl_network_messages::Team::Hulks)
                        && game.sub_state.is_some_and(|state| {
                            state != hsl_network_messages::SubState::PenaltyKick
                        })
                })))
    .then_some(position)
}

#[cfg(test)]
mod tests {
    use linear_algebra::vector;
    use types::{
        field_dimensions::{FieldDimensions, Side},
        obstacles::Obstacle,
        world_state::BallSource,
    };

    use crate::node::LastBall;

    use super::*;

    #[test]
    fn fresh_close_percept_overrides_position_without_erasing_rolling_velocity() {
        use std::time::Duration;
        use types::{ball_position::BallPosition, world_state::BallState};
        let mut board = Blackboard::default();
        board.parameters.kicking.distance_for_kick = 1.3;
        board.ball = Some(LastBall {
            position: point![1.0, 0.2],
            velocity: vector![-1.5, 0.0],
            age: Default::default(),
            field_side: Side::Left,
            source: BallSource::Own,
        });
        board.world_state.ball = Some(BallState {
            ball_in_ground_velocity: vector![-1.5, 0.0],
            ..Default::default()
        });
        board.visual_kick_ball_position = Some(BallPosition {
            position: point![0.4, 0.1],
            velocity: Vector2::zeros(),
            last_seen: Default::default(),
        });
        assert_eq!(
            kick_ball_in_ground(&board, &Isometry2::identity()),
            Some((point![0.4, 0.1], vector![-1.5, 0.0]))
        );
        board.world_state.now = board.world_state.now + Duration::from_millis(101);
        assert_eq!(
            kick_ball_in_ground(&board, &Isometry2::identity())
                .unwrap()
                .0,
            point![1.0, 0.2]
        );
        board.visual_kick_ball_position.as_mut().unwrap().last_seen = board.world_state.now;
        board.visual_kick_ball_position.as_mut().unwrap().position = point![3.0, 0.0];
        assert_eq!(
            kick_ball_in_ground(&board, &Isometry2::identity())
                .unwrap()
                .0,
            point![1.0, 0.2]
        );
    }

    #[test]
    fn setplay_shots_choose_clear_goal_opening_and_stop_when_blocked() {
        let mut blackboard = Blackboard {
            field_dimensions: FieldDimensions::SPL_2025,
            ball: Some(LastBall {
                position: point![0.5, 0.0],
                velocity: Vector2::zeros(),
                age: Default::default(),
                field_side: Side::Left,
                source: BallSource::Own,
            }),
            ..Default::default()
        };
        blackboard.world_state.robot.ground_to_field = Some(Default::default());
        blackboard.parameters.kicking.strategy.maximum_shot_distance = 12.0;
        blackboard.parameters.kicking.strategy.obstacle_clearance = 0.05;
        blackboard.world_state.obstacles = vec![Obstacle::robot(point![3.0, 0.0], 0.2, 0.2)];
        assert_eq!(kick(&mut blackboard), Status::Success);
        assert_eq!(select_kick_target(&mut blackboard), Status::Success);
        assert!(blackboard.kick_target.unwrap().y().abs() > 0.1);

        blackboard.world_state.obstacles[0].radius_at_foot_height = 1.0;
        assert_eq!(kick(&mut blackboard), Status::Success);
        assert_eq!(select_kick_target(&mut blackboard), Status::Failure);
        assert_eq!(blackboard.body_motion, Some(BodyMotion::Stand));
        assert_eq!(blackboard.kick_target, None);
    }

    #[test]
    fn interception_uses_predicted_contact_and_regular_power_after_a_strong_command() {
        let mut board = Blackboard::default();
        board.world_state.robot.ground_to_field = Some(Isometry2::identity());
        board.ball = Some(LastBall {
            position: point![1.0, 1.0],
            velocity: vector![-1.0, -2.0],
            age: board.world_state.now,
            field_side: Side::Left,
            source: BallSource::Own,
        });
        board.parameters.ball.last_ball_timeout = std::time::Duration::from_secs(1);
        board
            .parameters
            .ball
            .interception
            .maximum_intercept_distance = 1.5;
        board
            .parameters
            .kicking
            .minimum_interception_forward_distance = 0.3;
        assert_eq!(kick(&mut board), Status::Success);
        use_strong_kick(&mut board);
        assert_eq!(intercept(&mut board), Status::Success);
        let Some(BodyMotion::VisualKick {
            ball_position,
            target_position,
            kick_power,
            ..
        }) = board.body_motion
        else {
            panic!("expected interception");
        };
        assert!((ball_position - point![0.4, -0.2]).norm() < 1e-6);
        assert_eq!(target_position, point![1.0, 1.0]);
        assert_eq!(kick_power, KickPower::Rumpelstilzchen);
    }

    #[test]
    fn sdk_target_keeps_calibration_and_field_orientation() {
        let mut board = Blackboard::default();
        board.player_number = PlayerNumber::Two;
        board.parameters.goalkeeper.player_number = PlayerNumber::One;
        let transform = linear_algebra::Pose2::new(point![1.0, 2.0], 0.5).as_transform();
        board.world_state.robot.ground_to_field = Some(transform);
        board.parameters.kicking.kick_target_offset_angle = 0.1;
        let ball = point![0.3, 0.1];
        let target = point![2.0, 0.5];
        let BodyMotion::VisualKick {
            target_position,
            kick_direction,
            robot_theta_to_field,
            ..
        } = kick_motion(&board, ball, target, false)
        else {
            unreachable!()
        };
        assert!((target_position - Rotation2::new(0.1) * target).norm() < 1e-6);
        assert_eq!(kick_direction, Orientation2::from_vector(target - ball));
        assert_eq!(robot_theta_to_field, transform.orientation());
        for context in 0..3 {
            board.player_number = if context == 0 {
                PlayerNumber::One
            } else {
                PlayerNumber::Two
            };
            board.remote_control_enabled = context == 1;
            board.deflecting_ball = context == 2;
            let BodyMotion::VisualKick {
                target_position, ..
            } = kick_motion(&board, ball, target, false)
            else {
                unreachable!()
            };
            assert_eq!(target_position, target);
        }
    }
}
