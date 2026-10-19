use coordinate_systems::Ground;
use linear_algebra::{Orientation2, Point2, Pose2, Vector2, point};
use types::{behavior_tree::Status, motion_command::OrientationMode, motion_type::MotionType};

use crate::{
    kick::passing::is_pass_receiver,
    node::{Blackboard, ball_timeout},
    switch_motion_type::is_allowed_to_switch,
    walk::walk_to,
};

// shortcut: short constant-velocity prediction; add calibrated friction when available.
const MAXIMUM_INTERCEPTION_TIME: f32 = 3.0;

pub(crate) fn is_receiving_ball(blackboard: &Blackboard) -> bool {
    rolling_ball_in_ground(blackboard).is_some()
}

fn rolling_ball_in_ground(blackboard: &Blackboard) -> Option<(Point2<Ground>, Vector2<Ground>)> {
    let ball = blackboard.ball.as_ref()?;
    let transform = blackboard.world_state.robot.ground_to_field?;
    let now = blackboard.world_state.now;
    if ball.age > now
        || now.duration_since(ball.age) >= ball_timeout(&blackboard.parameters, ball.source)
    {
        return None;
    }
    let position = transform.inverse() * ball.position;
    let velocity = transform.inverse() * ball.velocity;
    (velocity.norm()
        > blackboard
            .parameters
            .ball
            .interception
            .minimum_ball_velocity
        && position.coords().dot(&velocity) < 0.0)
        .then_some((position, velocity))
}

pub fn receive_rolling_ball(blackboard: &mut Blackboard) -> Status {
    if blackboard.last_motion_type != Some(MotionType::Walk) && !is_allowed_to_switch(blackboard) {
        return Status::Failure;
    }
    if !blackboard.last_closest_to_ball && !is_pass_receiver(blackboard) {
        return Status::Failure;
    }
    let Some((position, velocity)) = rolling_ball_in_ground(blackboard) else {
        return Status::Failure;
    };
    if position.coords().norm() <= blackboard.parameters.kicking.kick_activation_distance {
        return Status::Failure;
    }
    let transform = blackboard.world_state.robot.ground_to_field.unwrap();
    let goal = transform.inverse() * point![blackboard.field_dimensions.length / 2.0, 0.0];
    let Some(direction) = (goal - position).try_normalize(f32::EPSILON) else {
        return Status::Failure;
    };
    let standoff = blackboard.parameters.kicking.approach_ball_standoff;
    let limits = blackboard.walking_velocity_limits;
    let walk_speed = blackboard.parameters.walking.speed.kicking;
    let speed = limits.forward_velocity_limits[1]
        .min(-limits.forward_velocity_limits[0])
        .min(limits.lateral_velocity_limit)
        * walk_speed;
    // Arrive behind the ball facing goal, ready to kick onward without another turn.
    let Some(target) = reachable_interception(position - direction * standoff, velocity, speed)
    else {
        return Status::Failure;
    };
    let target_in_field = transform * target;
    let field = &blackboard.field_dimensions;
    if target_in_field.x().abs() >= field.length / 2.0 - field.ball_radius
        || target_in_field.y().abs() >= field.width / 2.0 - field.ball_radius
    {
        return Status::Failure;
    }
    let walking = blackboard.parameters.walking.walk_and_stand;
    blackboard.ball_action = None;
    blackboard.kick_target = None;
    walk_to(
        blackboard,
        Pose2::from_parts(target, Orientation2::from_vector(direction)),
        walk_speed,
        OrientationMode::LookTowards {
            direction: Orientation2::from_vector(direction),
            tolerance: walking.orientation_tolerance,
        },
        walking.normal_distance_to_be_aligned,
        walking.hysteresis,
    )
}

fn reachable_interception(
    position: Point2<Ground>,
    velocity: Vector2<Ground>,
    speed: f32,
) -> Option<Point2<Ground>> {
    if !speed.is_finite()
        || speed <= 0.0
        || !position
            .inner
            .iter()
            .chain(velocity.inner.iter())
            .all(|value| value.is_finite())
    {
        return None;
    }
    // First intersection of the rolling trajectory and the robot's reachable circle.
    let a = velocity.norm_squared() - speed * speed;
    let b = 2.0 * position.coords().dot(&velocity);
    let c = position.coords().norm_squared();
    let time = if a.abs() <= f32::EPSILON {
        if b >= 0.0 {
            return None;
        }
        -c / b
    } else {
        let discriminant = b * b - 4.0 * a * c;
        if discriminant < 0.0 {
            return None;
        }
        let root = discriminant.sqrt();
        [(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)]
            .into_iter()
            .filter(|time| time.is_finite() && *time >= 0.0)
            .min_by(f32::total_cmp)?
    };
    (time.is_finite() && (0.0..=MAXIMUM_INTERCEPTION_TIME).contains(&time))
        .then(|| position + velocity * time)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    use linear_algebra::{point, vector};
    use types::{
        field_dimensions::{FieldDimensions, Side},
        motion_command::BodyMotion,
        world_state::{BallSource, BallState},
    };

    use crate::node::LastBall;

    #[test]
    fn walks_to_incoming_ball_and_hands_off_close_ball() {
        let mut blackboard = Blackboard {
            field_dimensions: FieldDimensions::SPL_2025,
            last_closest_to_ball: true,
            ball: Some(LastBall {
                position: point![2.0, 0.3],
                velocity: vector![-2.0, 0.0],
                age: Default::default(),
                field_side: Side::Left,
                source: BallSource::Own,
            }),
            ..Default::default()
        };
        blackboard.world_state.robot.ground_to_field = Some(Default::default());
        blackboard.parameters.ball.last_ball_timeout = Duration::from_secs(1);
        blackboard.parameters.kicking.approach_ball_standoff = 0.2;
        blackboard.parameters.kicking.kick_activation_distance = 0.75;
        blackboard.walking_velocity_limits.forward_velocity_limits = [-0.5, 0.5];
        blackboard.walking_velocity_limits.lateral_velocity_limit = 0.5;
        assert_eq!(receive_rolling_ball(&mut blackboard), Status::Success);
        let Some(BodyMotion::Walk {
            target_orientation,
            orientation_mode: OrientationMode::LookTowards { direction, .. },
            ..
        }) = blackboard.body_motion
        else {
            panic!("expected goal-facing interception")
        };
        let goalward: Orientation2<Ground> = Orientation2::from_vector(vector![2.5, -0.3]);
        assert!((target_orientation.angle() - goalward.angle()).abs() < 1e-6);
        assert_eq!(target_orientation, direction);

        blackboard.ball.as_mut().unwrap().position = point![0.5, 0.0];
        assert_eq!(receive_rolling_ball(&mut blackboard), Status::Failure);
        blackboard.ball.as_mut().unwrap().position = point![2.0, 0.3];
        blackboard.ball.as_mut().unwrap().velocity = vector![2.0, 0.0];
        assert_eq!(receive_rolling_ball(&mut blackboard), Status::Failure);
        blackboard.ball.as_mut().unwrap().velocity = vector![-2.0, 0.0];
        blackboard.world_state.now = blackboard.world_state.now + Duration::from_secs(2);
        assert_eq!(receive_rolling_ball(&mut blackboard), Status::Failure);

        blackboard.ball.as_mut().unwrap().age = blackboard.world_state.now;
        blackboard.last_motion_type = Some(MotionType::Kick);
        blackboard.last_motion_switch_time = blackboard.world_state.now;
        blackboard.parameters.control.allow_switch.kick = Duration::from_secs(1);
        assert_eq!(receive_rolling_ball(&mut blackboard), Status::Failure);
        blackboard.last_motion_type = Some(MotionType::Walk);
        blackboard.last_closest_to_ball = false;
        assert_eq!(receive_rolling_ball(&mut blackboard), Status::Failure);
    }

    #[test]
    fn hands_off_incoming_ball_directly_to_quick_goalward_kick() {
        let mut blackboard = Blackboard {
            field_dimensions: FieldDimensions::SPL_2025,
            last_closest_to_ball: true,
            last_motion_type: Some(MotionType::Walk),
            ball: Some(LastBall {
                position: point![1.0, 0.15],
                velocity: vector![-2.0, 0.0],
                age: Default::default(),
                field_side: Side::Left,
                source: BallSource::Own,
            }),
            ..Default::default()
        };
        blackboard.world_state.robot.ground_to_field = Some(Default::default());
        blackboard.parameters.ball.last_ball_timeout = Duration::from_secs(1);
        blackboard.parameters.kicking.kick_activation_distance = 1.3;
        blackboard.world_state.ball = Some(BallState {
            ball_in_ground: point![1.0, 0.15],
            ball_in_ground_velocity: vector![-2.0, 0.0],
            ..Default::default()
        });
        blackboard.parameters.kicking.strategy.maximum_shot_distance = 6.0;
        blackboard.parameters.control.allow_switch.walk = Duration::from_secs(1);
        assert_eq!(receive_rolling_ball(&mut blackboard), Status::Failure);
        assert_eq!(
            crate::tree::striker_subtree()
                .tick_with_trace(&mut blackboard)
                .0,
            Status::Success
        );
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick { quick: true, ball_velocity, .. })
                if ball_velocity == vector![-2.0, 0.0]
        ));
        assert!(blackboard.kick_target.unwrap().x() > 1.0);
        assert!(blackboard.ball_action.is_some());

        blackboard.last_motion_command = types::motion_command::MotionCommand::from_partial_motions(
            blackboard.body_motion.clone().unwrap(),
            blackboard.head_motion.unwrap(),
        );
        blackboard.last_motion_type = Some(MotionType::Kick);
        blackboard.ball.as_mut().unwrap().velocity = vector![-0.2, 0.0];
        blackboard
            .world_state
            .ball
            .as_mut()
            .unwrap()
            .ball_in_ground_velocity = vector![-0.2, 0.0];
        crate::tree::striker_subtree().tick_with_trace(&mut blackboard);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick { quick: true, .. })
        ));
    }

    #[test]
    fn moves_sideways_to_reachable_point_before_ball_passes() {
        let target = reachable_interception(point![2.0, 0.3], vector![-2.0, 0.0], 0.4).unwrap();
        let time = (2.0 - target.x()) / 2.0;
        assert!(time > 0.0 && time < 1.0);
        assert!((target.coords().norm() - 0.4 * time).abs() < 1e-5);
        assert_eq!(target.y(), 0.3);
    }

    #[test]
    fn rejects_unreachable_receding_and_invalid_trajectories() {
        for (position, velocity, speed) in [
            (point![2.0, 2.0], vector![-2.0, 0.0], 0.3),
            (point![2.0, 0.0], vector![2.0, 0.0], 0.3),
            (point![f32::NAN, 0.0], vector![-2.0, 0.0], 0.3),
            (point![2.0, 0.0], vector![-2.0, 0.0], 0.0),
            (point![10.0, 0.0], vector![-0.1, 0.0], 0.1),
        ] {
            assert!(reachable_interception(position, velocity, speed).is_none());
        }
    }
}
