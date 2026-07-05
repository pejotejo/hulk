use coordinate_systems::{Field, Ground};
use linear_algebra::{Isometry2, Orientation2, Point2, Rotation2, point};
use types::{
    behavior_tree::Status,
    field_dimensions::{Half, Side},
    motion_command::{BodyMotion, KickPower, MotionCommand},
    motion_type::MotionType,
};

use crate::{
    action,
    actions::stand,
    behavior_tree::Node,
    condition, negation,
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
            subtree!(kick_power_subtree),
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
    if let (Some(ball), Some(ground_to_field)) = (
        &blackboard.ball,
        &blackboard.world_state.robot.ground_to_field,
    ) {
        let ball_in_ground = ground_to_field.inverse() * ball.position;
        let robot_theta_to_field: Orientation2<Field> = ground_to_field.orientation();

        blackboard.body_motion = Some(BodyMotion::VisualKick {
            ball_position: ball_in_ground,
            kick_direction: Default::default(),
            target_position: Default::default(),
            robot_theta_to_field,
            kick_power: Default::default(),
        });

        Status::Success
    } else {
        Status::Failure
    }
}

pub fn select_kick_target(blackboard: &mut Blackboard) -> Status {
    let (Some(ground_to_field), Some(ball)) = (
        blackboard.world_state.robot.ground_to_field,
        &blackboard.ball,
    ) else {
        return Status::Failure;
    };

    let goal_position = select_kick_target_in_field(blackboard, ground_to_field, ball.position);
    let target_offset_angle = blackboard.parameters.kicking.kick_target_offset_angle;

    apply_visual_kick_target(blackboard, goal_position, target_offset_angle)
}

fn select_kick_target_in_field(
    blackboard: &Blackboard,
    ground_to_field: Isometry2<Ground, Field>,
    ball: Point2<Field>,
) -> Point2<Field> {
    let field_dimensions = blackboard.field_dimensions;
    let kicking = &blackboard.parameters.kicking;

    let goal_x = field_dimensions.length / 2.0;
    let left_post = field_dimensions.goal_post(Half::Opponent, Side::Left);
    let usable_goal_y = (left_post.y().abs()
        - field_dimensions.goal_post_diameter / 2.0
        - kicking.kick_target_goal_post_margin)
        .max(0.0);

    let max_x = field_dimensions.length / 2.0;
    let max_y = field_dimensions.width / 2.0;
    let maximum_distance = kicking
        .kick_target_maximum_distance
        .max(kicking.kick_target_minimum_distance);
    let minimum_distance = kicking.kick_target_minimum_distance.min(maximum_distance);

    let mut candidates = Vec::new();
    for index in 0..10 {
        let y = -usable_goal_y + 2.0 * usable_goal_y * index as f32 / 9.0;
        candidates.push((point![goal_x, y], true));
    }

    for index in 0..20 {
        let side = if index % 2 == 0 { -1.0 } else { 1.0 };
        let distance =
            minimum_distance + (maximum_distance - minimum_distance) * (index / 2) as f32 / 9.0;

        candidates.push((
            point![
                (ball.x() + distance).clamp(-max_x, max_x),
                (ball.y() + side * distance * 0.5).clamp(-max_y, max_y)
            ],
            false,
        ));
    }

    candidates
        .into_iter()
        .max_by(|(left, left_is_goal), (right, right_is_goal)| {
            kick_target_score(blackboard, ground_to_field, ball, *left, *left_is_goal).total_cmp(
                &kick_target_score(blackboard, ground_to_field, ball, *right, *right_is_goal),
            )
        })
        .map(|(target, _)| target)
        .unwrap_or(point![goal_x, 0.0])
}

fn kick_target_score(
    blackboard: &Blackboard,
    ground_to_field: Isometry2<Ground, Field>,
    ball: Point2<Field>,
    target: Point2<Field>,
    is_goal_target: bool,
) -> f32 {
    let mut score = if is_goal_target { 10.0 } else { 0.0 };
    let field_dimensions = blackboard.field_dimensions;

    for obstacle in &blackboard.world_state.obstacles {
        let obstacle_position = ground_to_field * obstacle.position;
        if obstacle_position.x() > field_dimensions.length / 2.0
            && obstacle_position.y().abs() < field_dimensions.goal_inner_width / 2.0
        {
            continue;
        }

        let radius = obstacle.radius_at_foot_height
            + blackboard.parameters.kicking.kick_target_obstacle_clearance;
        let clearance = distance_to_segment(obstacle_position, ball, target) - radius;

        if clearance < 0.0 {
            score -= 1000.0 - clearance * 1000.0;
        } else {
            score += clearance.min(1.0);
        }
    }

    score - (target - ball).norm() * 0.01
}

fn distance_to_segment(point: Point2<Field>, start: Point2<Field>, end: Point2<Field>) -> f32 {
    let segment = end - start;
    let length_squared = segment.norm_squared();
    if length_squared <= f32::EPSILON {
        return (point - start).norm();
    }

    let t = ((point - start).dot(&segment) / length_squared).clamp(0.0, 1.0);
    let closest = start + segment * t;
    (point - closest).norm()
}

pub(super) fn apply_visual_kick_target(
    blackboard: &mut Blackboard,
    target_position_in_field: Point2<Field>,
    target_offset_angle: f32,
) -> Status {
    if let (Some(ground_to_field), Some(ball)) = (
        blackboard.world_state.robot.ground_to_field,
        &blackboard.ball,
    ) {
        let field_to_ground = ground_to_field.inverse();
        let target_position = field_to_ground * target_position_in_field;
        let ball_in_ground = field_to_ground * ball.position;
        let kick_direction = Orientation2::from_vector(target_position - ball_in_ground);

        if let Some(BodyMotion::VisualKick {
            target_position: motion_target_position,
            kick_direction: motion_kick_direction,
            ..
        }) = blackboard.body_motion.as_mut()
        {
            *motion_target_position = Rotation2::new(target_offset_angle) * target_position;
            *motion_kick_direction = kick_direction;

            return Status::Success;
        }
    }

    Status::Failure
}

pub fn kick_power_subtree() -> Node<Blackboard> {
    selection!(
        sequence!(
            condition!(is_last_motion_type, MotionType::Kick),
            action!(use_last_kick_power)
        ),
        sequence!(
            negation!(condition!(is_close_to_target)),
            condition!(allow_schlong),
            action!(use_kick_power, KickPower::Schlong)
        ),
        action!(use_kick_power, KickPower::Rumpelstilzchen)
    )
}

pub fn is_close_to_target(blackboard: &mut Blackboard) -> bool {
    if let Some(BodyMotion::VisualKick {
        target_position, ..
    }) = &blackboard.body_motion
    {
        target_position.coords().norm()
            < blackboard
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

pub fn use_last_kick_power(blackboard: &mut Blackboard) -> Status {
    if let MotionCommand::VisualKick {
        kick_power: last_kick_power,
        ..
    } = blackboard.last_motion_command
        && let Some(BodyMotion::VisualKick {
            kick_power: motion_kick_power,
            ..
        }) = blackboard.body_motion.as_mut()
    {
        *motion_kick_power = last_kick_power;

        return Status::Success;
    }
    Status::Failure
}

pub fn use_kick_power(blackboard: &mut Blackboard, kick_power: KickPower) -> Status {
    if let Some(BodyMotion::VisualKick {
        kick_power: motion_kick_power,
        ..
    }) = blackboard.body_motion.as_mut()
    {
        *motion_kick_power = kick_power;

        return Status::Success;
    }
    Status::Failure
}

pub fn intercept(blackboard: &mut Blackboard) -> Status {
    if let (Some(ball), Some(ground_to_field)) = (
        &blackboard.ball,
        &blackboard.world_state.robot.ground_to_field,
    ) {
        let ball_in_ground = ground_to_field.inverse() * ball.position;
        let velocity = ball.velocity;
        if velocity.norm() < f32::EPSILON {
            return Status::Failure;
        }
        let time_to_closest_approach =
            -ball_in_ground.coords().dot(&velocity) / velocity.norm_squared();
        if time_to_closest_approach < 0.0 {
            return Status::Failure;
        }

        let interception_point = ball_in_ground + velocity * time_to_closest_approach;
        if interception_point.x() < blackboard.parameters.kicking.kick_position_ball_distance {
            return Status::Failure;
        }

        if interception_point.coords().norm()
            > blackboard
                .parameters
                .intercept_ball
                .maximum_intercept_distance
        {
            return Status::Failure;
        }

        let kick_direction = Orientation2::from_vector(ball_in_ground - interception_point);

        if let Some(BodyMotion::VisualKick {
            ball_position: motion_ball_position,
            target_position: motion_target_position,
            kick_direction: motion_kick_direction,
            ..
        }) = blackboard.body_motion.as_mut()
        {
            *motion_ball_position = interception_point;
            *motion_target_position = ball_in_ground;
            *motion_kick_direction = kick_direction;
            return Status::Success;
        }
    }
    Status::Failure
}

pub fn set_kick_target_in_front(blackboard: &mut Blackboard) -> Status {
    if let (Some(ground_to_field), Some(ball)) = (
        blackboard.world_state.robot.ground_to_field,
        &blackboard.ball,
    ) {
        if blackboard.last_motion_type != Some(MotionType::Kick) {
            let kick_target = ground_to_field * point!(3.0, 0.0);
            blackboard.last_kick_target = Some(kick_target);
        }

        if let Some(BodyMotion::VisualKick {
            target_position: motion_target_position,
            kick_direction: motion_kick_direction,
            ..
        }) = blackboard.body_motion.as_mut()
            && let Some(target_in_field) = blackboard.last_kick_target
        {
            let field_to_ground = ground_to_field.inverse();
            let ball_in_ground = field_to_ground * ball.position;
            let target_position = field_to_ground * target_in_field;
            let kick_direction = Orientation2::from_vector(target_position - ball_in_ground);

            *motion_target_position =
                Rotation2::new(blackboard.parameters.kicking.kick_target_offset_angle)
                    * target_position;
            *motion_kick_direction = kick_direction;
            return Status::Success;
        }
    }
    Status::Failure
}
