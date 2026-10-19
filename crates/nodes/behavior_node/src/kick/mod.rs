pub mod execution;
pub mod geometry;
pub mod kickoff;
pub mod passing;
pub mod reception;
pub mod strategy;

use ::geometry::line_segment::LineSegment;
use coordinate_systems::{Field, Ground};
use hsl_network_messages::PlayerNumber;
use linear_algebra::{Isometry2, Orientation2, Point2, Vector2, point};
use types::{
    behavior_tree::Status,
    motion_command::{BodyMotion, HeadMotion, ImageRegion, MotionCommand},
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
        && let Some(ball_in_ground) = kick_ball_position_in_ground(blackboard, ground_to_field)
    {
        blackboard.body_motion = Some(BodyMotion::Kick {
            ball_position: ball_in_ground,
            ball_velocity: blackboard
                .world_state
                .ball
                .map_or(Vector2::zeros(), |ball| ball.ball_in_ground_velocity),
            target_speed: blackboard.parameters.kicking.target_speed,
            soft: false,
            quick: blackboard.parameters.kicking.quick,
            kick_direction: Default::default(),
            strong: false,
        });
        if blackboard.last_motion_type == Some(MotionType::Kick) {
            use_last_kick_settings(blackboard);
        }
        blackboard.head_motion = Some(HeadMotion::LookAt {
            target: ball_in_ground,
            height_above_ground: blackboard.field_dimensions.ball_radius,
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

pub fn apply_kick_target(
    blackboard: &mut Blackboard,
    target_position_in_field: Point2<Field>,
) -> Status {
    if let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field
        && let Some(BodyMotion::Kick {
            ball_position,
            kick_direction,
            ..
        }) = blackboard.body_motion.as_mut()
    {
        let target_position = ground_to_field.inverse() * target_position_in_field;
        *kick_direction = Orientation2::from_vector(target_position - *ball_position);
        blackboard.kick_target = Some(target_position);
        return Status::Success;
    }

    Status::Failure
}

pub fn kick_strength_subtree() -> Node<Blackboard> {
    selection!(
        sequence!(
            condition!(is_last_motion_type, MotionType::Kick),
            action!(use_last_kick_settings)
        ),
        sequence!(
            condition!(is_target_in_strong_kick_range),
            condition!(allow_strong_kicks),
            action!(use_strong_kick)
        ),
        action!(disable_strong_kick)
    )
}

pub fn is_target_in_strong_kick_range(blackboard: &mut Blackboard) -> bool {
    if let Some(target_position) = blackboard.kick_target
        && let Some(BodyMotion::Kick { ball_position, .. }) = &blackboard.body_motion
    {
        (target_position - *ball_position).norm()
            >= blackboard
                .parameters
                .kicking
                .strong_kick_min_target_distance
    } else {
        false
    }
}

pub fn allow_strong_kicks(blackboard: &mut Blackboard) -> bool {
    blackboard.parameters.kicking.allow_strong_kicks
}

pub fn use_last_kick_settings(blackboard: &mut Blackboard) -> Status {
    if let Some(MotionCommand::Kick {
        strong: last_strong,
        quick: last_quick,
        target_speed: last_target_speed,
        ..
    }) = blackboard.last_motion_command
        && let Some(BodyMotion::Kick {
            strong: motion_strong,
            soft,
            quick,
            target_speed,
            ..
        }) = blackboard.body_motion.as_mut()
    {
        *motion_strong = last_strong;
        *soft = false;
        *quick = last_quick;
        *target_speed = last_target_speed;

        return Status::Success;
    }
    Status::Failure
}

pub fn use_kick(blackboard: &mut Blackboard, strong: bool) -> Status {
    if let Some(BodyMotion::Kick {
        strong: motion_strong,
        ..
    }) = blackboard.body_motion.as_mut()
    {
        *motion_strong = strong;

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
    if let Some(BodyMotion::Kick {
        ball_position,
        ball_velocity,
        kick_direction: motion_kick_direction,
        strong,
        soft,
        ..
    }) = blackboard.body_motion.as_mut()
    {
        if ball_velocity.norm() < f32::EPSILON {
            return Status::Failure;
        }
        let time_to_closest_approach =
            -(*ball_position).coords().dot(ball_velocity) / ball_velocity.norm_squared();
        if time_to_closest_approach < 0.0 {
            return Status::Failure;
        }

        let interception_point = *ball_position + *ball_velocity * time_to_closest_approach;
        if interception_point.x()
            < blackboard
                .parameters
                .kicking
                .minimum_interception_forward_distance
        {
            return Status::Failure;
        }

        if interception_point.coords().norm()
            > blackboard
                .parameters
                .ball
                .interception
                .maximum_intercept_distance
        {
            return Status::Failure;
        }

        let kick_direction = Orientation2::from_vector(*ball_position - interception_point);

        // The policy needs the observed position and velocity together.
        // Use the predicted interception point only to choose kick direction.
        blackboard.kick_target = Some(*ball_position);
        *motion_kick_direction = kick_direction;
        *strong = false;
        *soft = false;
        return Status::Success;
    }
    Status::Failure
}
pub fn set_kick_target_beyond_ball(blackboard: &mut Blackboard) -> Status {
    if let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field
        && let Some(ball_in_ground) = kick_ball_position_in_ground(blackboard, &ground_to_field)
        && let Some(BodyMotion::Kick {
            kick_direction: motion_kick_direction,
            ..
        }) = blackboard.body_motion.as_mut()
    {
        if blackboard.last_motion_type != Some(MotionType::Kick) {
            let Some(direction) = ball_in_ground.coords().try_normalize(f32::EPSILON) else {
                return Status::Failure;
            };
            let kick_target = ground_to_field * (ball_in_ground + direction * 10.0);
            blackboard.last_kick_target = Some(kick_target);
        }

        if let Some(target_in_field) = blackboard.last_kick_target {
            let field_to_ground = ground_to_field.inverse();
            let target_position = field_to_ground * target_in_field;
            let kick_direction = Orientation2::from_vector(target_position - ball_in_ground);

            blackboard.kick_target = Some(target_position);
            *motion_kick_direction = kick_direction;

            return Status::Success;
        }
    }
    Status::Failure
}

pub(crate) fn kick_ball_position_in_ground(
    blackboard: &Blackboard,
    ground_to_field: &Isometry2<Ground, Field>,
) -> Option<Point2<Ground>> {
    blackboard
        .visual_kick_ball_position
        .as_ref()
        .map(|ball| ball.position)
        .or_else(|| {
            blackboard
                .ball
                .as_ref()
                .map(|ball| ground_to_field.inverse() * ball.position)
        })
}

pub(super) fn pass_receiver_position(
    blackboard: &Blackboard,
    ball: Point2<Field>,
    receiver: PlayerNumber,
) -> Option<Point2<Field>> {
    let world = &blackboard.world_state;
    let parameters = &blackboard.parameters.kicking.strategy.passing;
    if receiver == world.robot.player_number
        || receiver == blackboard.parameters.goalkeeper.player_number
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
        && position.x() - ball.x() >= parameters.minimum_progress)
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
    fn interception_uses_regular_kick_after_a_strong_or_soft_command() {
        let mut blackboard = Blackboard {
            body_motion: Some(BodyMotion::Kick {
                ball_position: point![1.0, 1.0],
                ball_velocity: vector![-1.0, -2.0],
                kick_direction: Default::default(),
                target_speed: 3.4,
                strong: true,
                soft: true,
                quick: false,
            }),
            ..Default::default()
        };
        blackboard
            .parameters
            .ball
            .interception
            .maximum_intercept_distance = 1.5;
        blackboard
            .parameters
            .kicking
            .minimum_interception_forward_distance = 0.3;
        assert_eq!(intercept(&mut blackboard), Status::Success);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick {
                strong: false,
                soft: false,
                target_speed: 3.4,
                ..
            })
        ));
    }
}
