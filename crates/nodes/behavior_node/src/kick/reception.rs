use std::time::Duration;

use coordinate_systems::Ground;
use linear_algebra::{Orientation2, Point2, Vector2, point};
use types::{
    behavior_tree::Status,
    motion_command::{BodyMotion, OrientationMode},
    motion_type::MotionType,
    path::traits::PathProgress,
    walking_velocity_limits::WalkingVelocityLimits,
};

use crate::{
    kick::kick_ball_in_ground,
    node::{Blackboard, ball_timeout},
    switch_motion_type::is_allowed_to_switch,
    walk::plan_interception,
};

// shortcut: short constant-velocity prediction; add calibrated friction when available.
const MAXIMUM_INTERCEPTION_TIME: f32 = 6.0;
const INTERCEPTION_TIME_STEP: f32 = 0.1;

pub(crate) fn is_receiving_ball(blackboard: &Blackboard) -> bool {
    rolling_ball_in_ground(blackboard).is_some_and(|(position, velocity)| {
        let own_goal = blackboard
            .world_state
            .robot
            .ground_to_field
            .unwrap()
            .inverse()
            * point![-blackboard.field_dimensions.length / 2.0, 0.0];
        should_block_ball(position, velocity, own_goal)
    })
}

pub(crate) fn is_ready_to_receive_kick(blackboard: &Blackboard) -> bool {
    let Some((position, velocity)) = rolling_ball_in_ground(blackboard) else {
        return false;
    };
    if blackboard.world_state.robot.player_number != blackboard.parameters.goalkeeper.player_number
        && position.x() > 0.0
        && position.coords().norm() <= blackboard.parameters.kicking.kick_activation_distance
        && position.coords().dot(&velocity) < 0.0
    {
        return true;
    }
    let standoff = blackboard.parameters.kicking.approach_ball_standoff;
    let half_width = (blackboard.parameters.walking.path_planning.robot_radius / 2.0)
        .max(2.0 * blackboard.field_dimensions.ball_radius);
    let contact = point![standoff, 0.0];
    let contact_time = (contact - position).dot(&velocity) / velocity.norm_squared();
    if position.x() > 0.0
        && position.coords().norm() <= blackboard.parameters.kicking.kick_activation_distance
        && (0.0..=0.25).contains(&contact_time)
        && (position + velocity * contact_time - contact).norm() <= half_width
    {
        return true;
    }
    let closing_speed = -velocity.x();
    // Allow 250 ms for turn/swing preparation, within the policy's 2 m input limit.
    let handoff_distance = if closing_speed > 1.0 {
        (blackboard.parameters.kicking.kick_activation_distance + closing_speed * 0.25).min(2.0)
    } else {
        blackboard.parameters.kicking.kick_activation_distance
    };
    if position.x() <= 0.0
        || position.coords().norm() > handoff_distance
        || Orientation2::from_vector(position.coords()).angle().abs() > 0.45
        || (velocity.x() >= 0.0
            && position.x() > standoff + blackboard.field_dimensions.ball_radius)
    {
        return false;
    }
    let lateral_position = if velocity.x() < -f32::EPSILON && position.x() > standoff {
        position.y() + velocity.y() * (standoff - position.x()) / velocity.x()
    } else {
        position.y()
    };
    lateral_position.abs() <= half_width
}

fn rolling_ball_in_ground(blackboard: &Blackboard) -> Option<(Point2<Ground>, Vector2<Ground>)> {
    let transform = blackboard.world_state.robot.ground_to_field?;
    let (position, velocity) = kick_ball_in_ground(blackboard, &transform)?;
    let now = blackboard.world_state.now;
    let fresh_ball = blackboard.ball.as_ref().is_some_and(|ball| {
        ball.age <= now
            && now.duration_since(ball.age) < ball_timeout(&blackboard.parameters, ball.source)
    });
    let fresh_visual = blackboard
        .visual_kick_ball_position
        .as_ref()
        .is_some_and(|ball| {
            ball.position == position
                && position.coords().norm()
                    <= blackboard.parameters.kicking.kick_activation_distance
                        + blackboard.parameters.kicking.kick_activation_hysteresis
                && ball
                    .age_at(now)
                    .is_some_and(|age| age <= Duration::from_millis(100))
        });
    if !fresh_ball && !fresh_visual {
        return None;
    }
    (position
        .inner
        .iter()
        .chain(velocity.inner.iter())
        .all(|value| value.is_finite())
        && velocity.norm()
            > blackboard
                .parameters
                .ball
                .interception
                .minimum_ball_velocity)
        .then_some((position, velocity))
}

pub fn receive_rolling_ball(blackboard: &mut Blackboard) -> Status {
    if blackboard.last_motion_type == Some(MotionType::Kick)
        || (blackboard.last_motion_type != Some(MotionType::Walk)
            && !is_allowed_to_switch(blackboard))
    {
        return Status::Failure;
    }
    let Some((position, velocity)) = rolling_ball_in_ground(blackboard) else {
        return Status::Failure;
    };
    if is_ready_to_receive_kick(blackboard) {
        return Status::Failure;
    }
    let transform = blackboard.world_state.robot.ground_to_field.unwrap();
    let goal = transform.inverse() * point![-blackboard.field_dimensions.length / 2.0, 0.0];
    super::strategy::select_ball_action(blackboard);
    let kick_orientation = super::execution::resolve_target(blackboard)
        .filter(|_| {
            position.x() > 0.0
                && position.coords().norm()
                    <= blackboard.parameters.kicking.kick_activation_distance
                        + blackboard.parameters.kicking.kick_activation_hysteresis
        })
        .and_then(|target| {
            (transform.inverse() * target - position)
                .try_normalize(f32::EPSILON)
                .map(Orientation2::from_vector)
        });
    let orientation = kick_orientation.unwrap_or_else(|| Orientation2::from_vector(-velocity));
    let standoff = blackboard.parameters.kicking.approach_ball_standoff;
    let limits = blackboard.walking_velocity_limits;
    let walk_speed = blackboard.parameters.walking.speed.kicking;
    let Some(target) = blocking_target(
        position,
        velocity,
        goal,
        standoff,
        limits,
        walk_speed,
        orientation,
    ) else {
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
    if target.coords().norm() <= walking.target_reached_thresholds.x {
        // Keep Walk active while holding the lane so an arriving ball needs no Stand dwell.
        blackboard.body_motion = Some(BodyMotion::WalkWithVelocity {
            velocity: Vector2::zeros(),
            angular_velocity: limits.scale_angular_axis(
                kick_orientation
                    .unwrap_or_else(|| Orientation2::from_vector(position.coords()))
                    .angle(),
            ),
        });
        return Status::Success;
    }
    let path = plan_interception(blackboard, target, transform);
    let forward = path.forward(Point2::origin());
    if forward.x() < 0.0 {
        // Turning and translation are independent; do not back into a rolling ball.
        blackboard.body_motion = Some(BodyMotion::WalkWithVelocity {
            velocity: Vector2::zeros(),
            angular_velocity: limits.scale_angular_axis(Orientation2::from_vector(forward).angle()),
        });
        return Status::Success;
    }
    let kick_orientation =
        kick_orientation.filter(|direction| direction.as_unit_vector().dot(&forward) >= 0.0);
    blackboard.body_motion = Some(BodyMotion::Walk {
        path,
        orientation_mode: match kick_orientation {
            Some(direction) => OrientationMode::LookTowards {
                direction,
                tolerance: walking.orientation_tolerance,
            },
            None => OrientationMode::AlignWithPath,
        },
        target_orientation: kick_orientation.unwrap_or_else(|| Orientation2::from_vector(forward)),
        distance_to_be_aligned: walking.normal_distance_to_be_aligned,
        speed: walk_speed,
    });
    Status::Success
}

fn should_block_ball(
    position: Point2<Ground>,
    velocity: Vector2<Ground>,
    own_goal: Point2<Ground>,
) -> bool {
    position.coords().dot(&velocity) < 0.0 || velocity.dot(&(own_goal - position)) > 0.0
}

fn blocking_target(
    position: Point2<Ground>,
    velocity: Vector2<Ground>,
    own_goal: Point2<Ground>,
    standoff: f32,
    limits: WalkingVelocityLimits,
    walk_speed: f32,
    orientation: Orientation2<Ground>,
) -> Option<Point2<Ground>> {
    if !should_block_ball(position, velocity, own_goal) {
        return None;
    }
    // Arrive behind the contact point facing the outgoing kick, avoiding a late turn.
    let contact_offset = -orientation.as_unit_vector() * standoff;
    for step in 1..=(MAXIMUM_INTERCEPTION_TIME / INTERCEPTION_TIME_STEP) as u32 {
        let time = step as f32 * INTERCEPTION_TIME_STEP;
        let target = position + velocity * time + contact_offset;
        if travel_time(target, orientation, limits, walk_speed) <= time {
            return Some(target);
        }
    }
    // A fast ball can be unreachable. Still occupy its goal lane instead of chasing it.
    let to_goal = own_goal - position;
    if velocity.dot(&to_goal) > 0.0 {
        let fraction = (-position.coords()).dot(&to_goal) / to_goal.norm_squared();
        if fraction > 0.0 {
            return Some(position + to_goal * fraction.min(0.9));
        }
    }
    let time = (-position.coords().dot(&velocity) / velocity.norm_squared())
        .clamp(0.0, MAXIMUM_INTERCEPTION_TIME);
    (time > 0.0).then(|| position + velocity * time + contact_offset)
}

fn travel_time(
    target: Point2<Ground>,
    orientation: Orientation2<Ground>,
    limits: WalkingVelocityLimits,
    walk_speed: f32,
) -> f32 {
    let mut time = orientation.angle().abs() / limits.angular_velocity_limit.max(f32::EPSILON);
    for distance in [
        target.coords(),
        orientation.as_transform::<Ground>().inverse() * target.coords(),
    ] {
        let forward_limit = if distance.x() >= 0.0 {
            limits.forward_velocity_limits[1]
        } else {
            -limits.forward_velocity_limits[0]
        };
        time = time
            .max(distance.x().abs() / forward_limit.min(walk_speed).max(f32::EPSILON))
            .max(
                distance.y().abs()
                    / limits
                        .lateral_velocity_limit
                        .min(walk_speed)
                        .max(f32::EPSILON),
            );
    }
    time
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    use hsl_network_messages::PlayerNumber;
    use linear_algebra::{point, vector};
    use types::{
        field_dimensions::{FieldDimensions, Side},
        motion_command::BodyMotion,
        world_state::{BallSource, BallState},
    };

    use crate::node::LastBall;

    #[test]
    fn field_player_receives_near_approaching_ball_with_kick_policy() {
        let mut blackboard = Blackboard {
            field_dimensions: FieldDimensions::SPL_2025,
            last_motion_type: Some(MotionType::Walk),
            ball: Some(LastBall {
                position: point![0.8, -0.7],
                velocity: vector![0.0, 1.0],
                age: Default::default(),
                field_side: Side::Right,
                source: BallSource::Own,
            }),
            ..Default::default()
        };
        blackboard.world_state.robot.player_number = PlayerNumber::Two;
        blackboard.world_state.robot.ground_to_field = Some(Default::default());
        blackboard.parameters.goalkeeper.player_number = PlayerNumber::One;
        blackboard.parameters.ball.last_ball_timeout = Duration::from_secs(1);
        blackboard.parameters.kicking.kick_activation_distance = 1.3;
        blackboard.parameters.kicking.approach_ball_standoff = 0.3;
        blackboard.parameters.kicking.strategy.maximum_shot_distance = 6.0;
        blackboard.parameters.kicking.target_speed = 3.4;
        blackboard.parameters.kicking.allow_strong_kicks = true;
        blackboard
            .parameters
            .kicking
            .strong_kick_min_target_distance = 0.1;
        assert!(is_ready_to_receive_kick(&blackboard));
        assert_eq!(receive_rolling_ball(&mut blackboard), Status::Failure);
        crate::tree::striker_subtree().tick_with_trace(&mut blackboard);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick {
                ball_position,
                ball_velocity,
                quick: false,
                strong: false,
                soft: false,
                target_speed: 3.4,
                ..
            }) if ball_position == point![0.8, -0.7] && ball_velocity == vector![0.0, 1.0]
        ));
        for (position, velocity) in [
            (point![1.4, 0.0], vector![-0.5, 0.0]),
            (point![0.8, 0.7], vector![0.4, 0.3]),
            (point![-0.5, -0.7], vector![0.3, 0.5]),
        ] {
            let ball = blackboard.ball.as_mut().unwrap();
            ball.position = position;
            ball.velocity = velocity;
            assert!(!is_ready_to_receive_kick(&blackboard));
        }
        blackboard.world_state.robot.player_number = PlayerNumber::One;
        blackboard.ball.as_mut().unwrap().position = point![0.8, -0.7];
        blackboard.ball.as_mut().unwrap().velocity = vector![0.0, 1.0];
        assert!(!is_ready_to_receive_kick(&blackboard));
        blackboard.ball.as_mut().unwrap().position = point![0.3, -0.2];
        assert!(is_ready_to_receive_kick(&blackboard));
    }

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
            orientation_mode: OrientationMode::AlignWithPath,
            ref path,
            ..
        }) = blackboard.body_motion
        else {
            panic!("expected path-facing interception")
        };
        assert!(
            (target_orientation.angle()
                - Orientation2::from_vector(path.forward(Point2::origin())).angle())
            .abs()
                < 1e-6
        );

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
        assert_eq!(receive_rolling_ball(&mut blackboard), Status::Success);
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
        blackboard.parameters.kicking.quick = true;
        blackboard.parameters.walking.path_planning.robot_radius = 0.4;
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
    fn meets_a_diagonal_trajectory_before_closest_approach() {
        let limits = WalkingVelocityLimits {
            forward_velocity_limits: [-0.5, 0.5],
            lateral_velocity_limit: 0.5,
            angular_velocity_limit: 2.0,
        };
        let position = point![2.0, 0.7];
        let velocity = vector![-1.0, -0.3];
        let direction = velocity.normalize();
        let target = blocking_target(
            position,
            velocity,
            point![-4.5, 0.0],
            0.2,
            limits,
            1.0,
            Orientation2::from_vector(-direction),
        )
        .unwrap();
        let time = (target.x() - position.x() - direction.x() * 0.2) / velocity.x();
        assert!(time > 0.0 && time < 2.0);
        assert!((target - (position + velocity * time + direction * 0.2)).norm() < 1e-5);
        assert!(travel_time(target, Orientation2::from_vector(-direction), limits, 1.0) <= time);
    }

    #[test]
    fn occupies_goal_lane_when_a_fast_crossing_ball_is_unreachable() {
        let position = point![2.0, 2.0];
        let goal = point![-4.5, 0.0];
        let limits = WalkingVelocityLimits {
            forward_velocity_limits: [-0.2, 0.2],
            lateral_velocity_limit: 0.2,
            angular_velocity_limit: 1.0,
        };
        let target = blocking_target(
            position,
            vector![-3.0, 0.0],
            goal,
            0.2,
            limits,
            1.0,
            Orientation2::identity(),
        )
        .unwrap();
        let lane = goal - position;
        let fraction = (target - position).dot(&lane) / lane.norm_squared();
        assert!((0.0..1.0).contains(&fraction));
        assert!((target - (position + lane * fraction)).norm() < 1e-5);
        assert!(target.y() < position.y());
    }

    #[test]
    fn gets_goal_side_of_a_slow_receding_ball_when_reachable() {
        let position = point![-1.0, 0.0];
        let limits = WalkingVelocityLimits {
            forward_velocity_limits: [-0.8, 1.8],
            lateral_velocity_limit: 1.0,
            angular_velocity_limit: 2.0,
        };
        let target = blocking_target(
            position,
            vector![-0.4, 0.0],
            point![-4.5, 0.0],
            0.2,
            limits,
            1.8,
            Orientation2::identity(),
        )
        .unwrap();
        assert!(target.x() < position.x());
        assert!(target.x() > -4.5);
        assert!(target.y().abs() < 1e-6);
    }

    #[test]
    fn turns_before_approaching_a_goalward_ball_behind_the_body() {
        let mut blackboard = Blackboard {
            field_dimensions: FieldDimensions::SPL_2025,
            last_motion_type: Some(MotionType::Walk),
            ball: Some(LastBall {
                position: point![-1.5, 0.4],
                velocity: vector![-0.4, 0.0],
                age: Default::default(),
                field_side: Side::Left,
                source: BallSource::Own,
            }),
            walking_velocity_limits: WalkingVelocityLimits {
                forward_velocity_limits: [-0.8, 1.8],
                lateral_velocity_limit: 1.0,
                angular_velocity_limit: 2.0,
            },
            ..Default::default()
        };
        blackboard.world_state.robot.ground_to_field = Some(Default::default());
        blackboard.parameters.ball.last_ball_timeout = Duration::from_secs(1);
        blackboard.parameters.kicking.approach_ball_standoff = 0.3;
        blackboard.parameters.walking.speed.kicking = 1.8;
        assert_eq!(receive_rolling_ball(&mut blackboard), Status::Success);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::WalkWithVelocity { velocity, angular_velocity })
                if velocity == Vector2::zeros() && angular_velocity > 0.0
        ));
    }

    #[test]
    fn rejects_receding_balls_without_a_useful_forward_block() {
        for (position, velocity) in [
            (point![2.0, 0.0], vector![2.0, 0.0]),
            (point![-2.0, 0.0], vector![-2.0, 0.0]),
            (point![f32::NAN, 0.0], vector![-2.0, 0.0]),
        ] {
            assert!(
                blocking_target(
                    position,
                    velocity,
                    point![-4.5, 0.0],
                    0.2,
                    WalkingVelocityLimits::default(),
                    1.0,
                    Orientation2::from_vector(-velocity),
                )
                .is_none()
            );
        }
    }

    #[test]
    fn slow_crossing_ball_keeps_walking_until_facing_its_path() {
        let mut blackboard = Blackboard {
            field_dimensions: FieldDimensions::SPL_2025,
            last_motion_type: Some(MotionType::Walk),
            ball: Some(LastBall {
                position: point![0.8, 0.7],
                velocity: vector![-0.4, -0.3],
                age: Default::default(),
                field_side: Side::Left,
                source: BallSource::Own,
            }),
            walking_velocity_limits: WalkingVelocityLimits {
                forward_velocity_limits: [-0.5, 0.5],
                lateral_velocity_limit: 0.5,
                angular_velocity_limit: 1.0,
            },
            ..Default::default()
        };
        blackboard.world_state.robot.ground_to_field = Some(Default::default());
        blackboard.parameters.ball.last_ball_timeout = Duration::from_secs(1);
        blackboard
            .parameters
            .ball
            .interception
            .minimum_ball_velocity = 0.3;
        blackboard.parameters.kicking.kick_activation_distance = 1.3;
        blackboard.parameters.kicking.approach_ball_standoff = 0.2;
        blackboard.parameters.walking.path_planning.robot_radius = 0.4;
        assert!(is_receiving_ball(&blackboard));
        assert!(!is_ready_to_receive_kick(&blackboard));
        assert_eq!(receive_rolling_ball(&mut blackboard), Status::Success);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Walk { .. })
        ));

        blackboard.ball.as_mut().unwrap().position = point![0.8, 0.05];
        blackboard.ball.as_mut().unwrap().velocity = vector![-0.5, 0.0];
        assert!(is_ready_to_receive_kick(&blackboard));
        assert_eq!(receive_rolling_ball(&mut blackboard), Status::Failure);
    }

    #[test]
    fn approaches_crossing_ball_facing_the_outgoing_shot() {
        let mut blackboard = Blackboard {
            field_dimensions: FieldDimensions::SPL_2025,
            last_motion_type: Some(MotionType::Walk),
            ball: Some(LastBall {
                position: point![0.8, -0.6],
                velocity: vector![0.0, 1.0],
                age: Default::default(),
                field_side: Side::Left,
                source: BallSource::Own,
            }),
            walking_velocity_limits: WalkingVelocityLimits {
                forward_velocity_limits: [-0.5, 0.8],
                lateral_velocity_limit: 0.5,
                angular_velocity_limit: 2.0,
            },
            ..Default::default()
        };
        blackboard.world_state.robot.ground_to_field = Some(Default::default());
        blackboard.parameters.ball.last_ball_timeout = Duration::from_secs(1);
        blackboard.parameters.kicking.strategy.maximum_shot_distance = 6.0;
        blackboard.parameters.kicking.approach_ball_standoff = 0.3;
        blackboard.parameters.kicking.kick_activation_distance = 1.3;
        blackboard.parameters.walking.speed.kicking = 1.0;
        assert_eq!(receive_rolling_ball(&mut blackboard), Status::Success);
        let Some(BodyMotion::Walk {
            orientation_mode: OrientationMode::LookTowards { direction, .. },
            path,
            ..
        }) = blackboard.body_motion
        else {
            panic!("expected outgoing-shot alignment")
        };
        assert!(direction.as_unit_vector().x() > 0.9);
        assert!(direction.as_unit_vector().y() > 0.0);
        let types::path::PathSegment::LineSegment(segment) = path.last_segment() else {
            panic!("expected final contact approach")
        };
        assert!(segment.1.x() < 0.8);
    }

    #[test]
    fn fresh_close_percept_can_receive_without_a_filtered_position() {
        let mut blackboard = Blackboard {
            field_dimensions: FieldDimensions::SPL_2025,
            visual_kick_ball_position: Some(types::ball_position::BallPosition {
                position: point![0.5, 0.05],
                velocity: Vector2::zeros(),
                last_seen: Default::default(),
            }),
            ..Default::default()
        };
        blackboard.world_state.robot.ground_to_field = Some(Default::default());
        blackboard.world_state.ball = Some(BallState {
            ball_in_ground_velocity: vector![-0.5, 0.0],
            ..Default::default()
        });
        blackboard.parameters.kicking.kick_activation_distance = 1.3;
        assert!(is_receiving_ball(&blackboard));
        assert!(is_ready_to_receive_kick(&blackboard));

        blackboard.world_state.now = blackboard.world_state.now + Duration::from_millis(101);
        assert!(!is_receiving_ball(&blackboard));
        assert!(!is_ready_to_receive_kick(&blackboard));
    }

    #[test]
    fn fast_incoming_ball_starts_quick_kick_with_preparation_lead() {
        let mut blackboard = Blackboard {
            field_dimensions: FieldDimensions::SPL_2025,
            last_motion_type: Some(MotionType::Walk),
            ball: Some(LastBall {
                position: point![1.7, 0.05],
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
        blackboard.parameters.kicking.quick = true;
        blackboard.parameters.kicking.approach_ball_standoff = 0.3;
        blackboard.parameters.kicking.strategy.maximum_shot_distance = 6.0;
        blackboard.parameters.walking.path_planning.robot_radius = 0.4;
        assert!(is_ready_to_receive_kick(&blackboard));
        assert_eq!(receive_rolling_ball(&mut blackboard), Status::Failure);
        crate::tree::striker_subtree().tick_with_trace(&mut blackboard);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick { quick: true, ball_position, ball_velocity, .. })
                if ball_position == point![1.7, 0.05] && ball_velocity == vector![-2.0, 0.0]
        ));

        blackboard.ball.as_mut().unwrap().velocity = vector![-0.5, 0.0];
        blackboard.ball.as_mut().unwrap().position = point![1.4, 0.05];
        assert!(!is_ready_to_receive_kick(&blackboard));
        blackboard.ball.as_mut().unwrap().velocity = vector![-6.0, 0.0];
        blackboard.ball.as_mut().unwrap().position = point![2.01, 0.05];
        assert!(!is_ready_to_receive_kick(&blackboard));
        blackboard.ball.as_mut().unwrap().position = point![1.7, 0.6];
        assert!(!is_ready_to_receive_kick(&blackboard));
    }

    #[test]
    fn prepares_for_side_arrival_before_it_crosses_the_feet() {
        let mut blackboard = Blackboard {
            field_dimensions: FieldDimensions::SPL_2025,
            last_motion_type: Some(MotionType::Walk),
            ball: Some(LastBall {
                position: point![0.3, -0.6],
                velocity: vector![0.0, 2.5],
                age: Default::default(),
                field_side: Side::Left,
                source: BallSource::Own,
            }),
            ..Default::default()
        };
        blackboard.world_state.robot.ground_to_field = Some(Default::default());
        blackboard.parameters.ball.last_ball_timeout = Duration::from_secs(1);
        blackboard.parameters.kicking.kick_activation_distance = 1.3;
        blackboard.parameters.kicking.approach_ball_standoff = 0.3;
        blackboard.parameters.kicking.strategy.maximum_shot_distance = 6.0;
        blackboard.parameters.walking.path_planning.robot_radius = 0.4;
        assert!(is_ready_to_receive_kick(&blackboard));
        assert_eq!(receive_rolling_ball(&mut blackboard), Status::Failure);
        crate::tree::striker_subtree().tick_with_trace(&mut blackboard);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick { quick: false, .. })
        ));
        for position in [point![0.8, -0.6], point![0.3, 0.6], point![0.3, -0.9]] {
            blackboard.ball.as_mut().unwrap().position = position;
            assert!(!is_ready_to_receive_kick(&blackboard));
        }
    }
}
