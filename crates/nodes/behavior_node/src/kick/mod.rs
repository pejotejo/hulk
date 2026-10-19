pub mod ball;
pub mod execution;
pub mod geometry;
pub mod passing;
pub mod restarts;
pub mod strategy;

use coordinate_systems::{Field, Ground};
use linear_algebra::{Isometry2, Orientation2, Point2, point};
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
    selection, sequence,
    switch_motion_type::{is_allowed_to_switch, is_last_motion_type},
    walk::walk_to_ball,
};

pub fn kick_subtree() -> Node<Blackboard> {
    sequence!(
        action!(strategy::select_ball_action),
        action!(execution::execute_ball_action),
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
        && let Some((ball_in_ground, ball_velocity)) =
            ball::kick_ball_state(blackboard, ground_to_field)
    {
        blackboard.body_motion = Some(BodyMotion::Kick {
            ball_position: ball_in_ground,
            ball_velocity,
            target_speed: blackboard.parameters.kicking.target_speed,
            soft: blackboard.parameters.kicking.soft,
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
    let goal_position: Point2<Field> = point!(blackboard.field_dimensions.length / 2.0, 0.0);

    apply_kick_target(blackboard, goal_position)
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
    // Legacy kick paths can keep emitting Kick without switching motion type.
    // Preserve tactical settings only for the actual commitment interval, and
    // refresh elapsed time even when the caller short-circuits its switch check.
    if blackboard.last_motion_type != Some(MotionType::Kick) || is_allowed_to_switch(blackboard) {
        return Status::Failure;
    }
    if let MotionCommand::Kick {
        strong: last_strong,
        soft: last_soft,
        quick: last_quick,
        target_speed: last_target_speed,
        ..
    } = blackboard.last_motion_command
        && let Some(BodyMotion::Kick {
            strong: motion_strong,
            soft,
            quick,
            target_speed,
            ..
        }) = blackboard.body_motion.as_mut()
    {
        *motion_strong = last_strong;
        *soft = last_soft;
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
        kick_direction,
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

        // Legacy keeper interceptions have no tactical target. Retain their
        // ball-relative clearance while preserving an explicitly selected target.
        if blackboard.kick_target.is_none() {
            *kick_direction = Orientation2::from_vector(*ball_position - interception_point);
            blackboard.kick_target = Some(*ball_position);
        }
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
    ball::kick_ball_state(blackboard, ground_to_field).map(|(position, _)| position)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::node::LastBall;
    use linear_algebra::vector;
    use ros_z::time::Time;
    use std::time::Duration;
    use types::{
        field_dimensions::{FieldDimensions, Side},
        world_state::BallSource,
    };

    fn legacy_kick_blackboard(elapsed: Duration) -> Blackboard {
        let mut b = Blackboard::default();
        b.field_dimensions = FieldDimensions::SPL_2025;
        b.parameters.kicking.target_speed = 3.4;
        b.parameters.kicking.soft = false;
        b.parameters.kicking.quick = false;
        b.parameters.kicking.allow_strong_kicks = true;
        b.parameters.kicking.strong_kick_min_target_distance = 2.0;
        b.parameters.control.allow_switch.kick = Duration::from_millis(500);
        b.parameters.ball.last_ball_timeout = Duration::from_millis(500);
        b.last_motion_type = Some(MotionType::Kick);
        b.last_motion_switch_time = Time::from_nanos(10_000_000_000);
        b.world_state.now = b.last_motion_switch_time + elapsed;
        b.world_state.robot.ground_to_field = Some(Default::default());
        b.ball = Some(LastBall {
            position: point![0.5, 0.1],
            velocity: vector![0.2, -0.3],
            age: b.world_state.now,
            field_side: Side::Left,
            source: BallSource::Own,
        });
        b.last_motion_command = MotionCommand::Kick {
            ball_position: point![0.4, -0.1],
            ball_velocity: vector![0.0, 0.0],
            kick_direction: Default::default(),
            head: HeadMotion::LookAround,
            target_speed: 1.5,
            soft: true,
            quick: true,
            strong: false,
        };
        b
    }

    fn kick_settings(b: &Blackboard) -> (bool, bool, bool, f32) {
        let Some(BodyMotion::Kick {
            soft,
            quick,
            strong,
            target_speed,
            ..
        }) = b.body_motion.as_ref()
        else {
            panic!("expected a kick motion")
        };
        (*soft, *quick, *strong, *target_speed)
    }

    #[test]
    fn legacy_kick_retains_settings_within_and_at_the_commitment_boundary() {
        for elapsed in [Duration::from_millis(499), Duration::from_millis(500)] {
            let mut b = legacy_kick_blackboard(elapsed);
            // A stale cached duration must not release an ongoing commitment.
            b.time_since_last_switch = Duration::from_secs(42);
            assert_eq!(kick(&mut b), Status::Success);
            assert_eq!(kick_settings(&b), (true, true, false, 1.5));
            assert_eq!(b.time_since_last_switch, elapsed);
            let Some(BodyMotion::Kick {
                ball_position,
                ball_velocity,
                ..
            }) = b.body_motion
            else {
                panic!("lost kick")
            };
            assert_eq!(ball_position, point![0.5, 0.1]);
            assert_eq!(ball_velocity, vector![0.2, -0.3]);
        }
    }

    #[test]
    fn expired_legacy_kick_uses_configured_settings_despite_a_reset_cached_duration() {
        for elapsed in [501, 580, 2580, 4580] {
            let elapsed = Duration::from_millis(elapsed);
            let mut b = legacy_kick_blackboard(elapsed);
            b.time_since_last_switch = Duration::ZERO;
            assert_eq!(kick(&mut b), Status::Success);
            assert_eq!(kick_settings(&b), (false, false, false, 3.4));
            assert_eq!(b.time_since_last_switch, elapsed);
        }
    }

    #[test]
    fn repeated_legacy_kicks_release_tactical_settings_and_resume_strength_selection() {
        let mut b = legacy_kick_blackboard(Duration::from_millis(100));
        let switched_at = b.last_motion_switch_time;
        let tree = sequence!(
            action!(kick),
            action!(select_kick_target),
            kick_strength_subtree(),
        );
        for elapsed in [100, 500, 501, 580, 2580, 4580] {
            let elapsed = Duration::from_millis(elapsed);
            b.world_state.now = switched_at + elapsed;
            b.ball.as_mut().unwrap().age = b.world_state.now;
            b.time_since_last_switch = Duration::ZERO;
            assert_eq!(tree.tick_with_trace(&mut b).0, Status::Success);
            let expected = if elapsed <= b.parameters.control.allow_switch.kick {
                (true, true, false, 1.5)
            } else {
                (false, false, true, 3.4)
            };
            assert_eq!(kick_settings(&b), expected);
            assert_eq!(b.time_since_last_switch, elapsed);
            assert_eq!(b.last_motion_switch_time, switched_at);
            b.last_motion_command = MotionCommand::from_partial_motions(
                b.body_motion.clone().unwrap(),
                b.head_motion.unwrap(),
            );
        }
    }

    #[test]
    fn expired_legacy_strength_selection_respects_current_range_and_configuration() {
        for (allow_strong, target, expected_strong) in [
            (true, point![4.0, 0.1], true),
            (true, point![1.0, 0.1], false),
            (false, point![4.0, 0.1], false),
        ] {
            let mut b = legacy_kick_blackboard(Duration::from_millis(580));
            b.parameters.kicking.allow_strong_kicks = allow_strong;
            assert_eq!(kick(&mut b), Status::Success);
            assert_eq!(apply_kick_target(&mut b, target), Status::Success);
            assert_eq!(
                kick_strength_subtree().tick_with_trace(&mut b).0,
                Status::Success
            );
            assert_eq!(kick_settings(&b), (false, false, expected_strong, 3.4));
        }
    }

    #[test]
    fn explicit_keeper_strong_disable_still_overrides_retained_and_expired_settings() {
        for elapsed in [Duration::from_millis(100), Duration::from_millis(580)] {
            let mut b = legacy_kick_blackboard(elapsed);
            if let MotionCommand::Kick { strong, .. } = &mut b.last_motion_command {
                *strong = true;
            }
            assert_eq!(kick(&mut b), Status::Success);
            assert_eq!(disable_strong_kick(&mut b), Status::Success);
            let (soft, quick, strong, speed) = kick_settings(&b);
            assert!(!strong);
            if elapsed <= b.parameters.control.allow_switch.kick {
                assert_eq!((soft, quick, speed), (true, true, 1.5));
            } else {
                assert_eq!((soft, quick, speed), (false, false, 3.4));
            }
        }
    }

    #[test]
    fn previous_kick_settings_are_not_retained_after_another_motion_type() {
        let mut b = legacy_kick_blackboard(Duration::from_millis(100));
        b.last_motion_type = Some(MotionType::Walk);
        assert_eq!(kick(&mut b), Status::Success);
        assert_eq!(use_last_kick_settings(&mut b), Status::Failure);
        assert_eq!(kick_settings(&b), (false, false, false, 3.4));
    }

    #[test]
    fn keeper_interception_selects_clearance_only_without_an_explicit_target() {
        for explicit_target in [false, true] {
            let mut b = Blackboard::default();
            b.parameters.kicking.minimum_interception_forward_distance = 0.15;
            b.parameters.ball.interception.maximum_intercept_distance = 0.8;
            let position = point![0.5, 0.4];
            let velocity = vector![-1.0, -2.0];
            b.body_motion = Some(BodyMotion::Kick {
                ball_position: position,
                ball_velocity: velocity,
                kick_direction: Default::default(),
                target_speed: 3.4,
                soft: false,
                quick: false,
                strong: false,
            });
            b.kick_target = explicit_target.then_some(point![3.0, 0.4]);
            assert_eq!(intercept(&mut b), Status::Success);
            let Some(BodyMotion::Kick {
                ball_position,
                ball_velocity,
                kick_direction,
                ..
            }) = b.body_motion
            else {
                panic!("lost kick")
            };
            assert_eq!(ball_position, position);
            assert_eq!(ball_velocity, velocity);
            if explicit_target {
                assert_eq!(kick_direction, Orientation2::default());
                assert_eq!(b.kick_target, Some(point![3.0, 0.4]));
            } else {
                assert!((kick_direction.angle() - 2.0_f32.atan()).abs() < 1e-5);
            }
        }
    }
}
