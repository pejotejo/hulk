use coordinate_systems::Field;
use geometry::{Distance, circle::Circle, line_segment::LineSegment, visibility::visible_segments};
use linear_algebra::{Orientation2, Point2, Pose2, Vector2, point};
use types::{
    behavior_tree::Status,
    motion_command::{BodyMotion, MotionCommand, OrientationMode},
    motion_type::MotionType,
    obstacles::ObstacleKind,
};

use crate::{
    actions::stand, conditions::is_close_to_ball, node::Blackboard,
    switch_motion_type::is_allowed_to_switch, walk::walk_to,
};

use super::{geometry::goal_openings, kick_ball_position_in_ground, strategy::BallAction};

/// Resolve and validate before writing any motion. No corridor means stand.
pub fn execute_ball_action(blackboard: &mut Blackboard) -> Status {
    let Some(target) = resolve_target(blackboard) else {
        blackboard.ball_action = None;
        blackboard.kick_target = None;
        return stand(blackboard);
    };
    let ground_to_field = blackboard.world_state.robot.ground_to_field.unwrap();
    let ball = kick_ball_position_in_ground(blackboard, &ground_to_field).unwrap();
    let target = ground_to_field.inverse() * target;
    blackboard.kick_target = Some(target);
    let direction = (target - ball).normalize();
    let orientation = Orientation2::from_vector(direction);
    let kicking = &blackboard.parameters.kicking;
    let walk = blackboard.parameters.walking.walk_and_stand;
    let dribbling = blackboard.ball_action == Some(BallAction::Dribble);
    let mut strong = !dribbling
        && kicking.allow_strong_kicks
        && (target - ball).norm() >= kicking.strong_kick_min_target_distance;
    if !dribbling
        && let MotionCommand::Kick {
            strong: previous, ..
        } = blackboard.last_motion_command
        && (!previous || kicking.allow_strong_kicks)
    {
        strong = previous;
    }
    let target_speed = kicking.target_speed;
    let soft = dribbling || kicking.soft;
    let quick = !dribbling && kicking.quick;
    let was_kicking = blackboard.last_motion_type == Some(MotionType::Kick);
    let can_switch = is_allowed_to_switch(blackboard);
    if (is_close_to_ball(blackboard) || (was_kicking && !can_switch)) && (was_kicking || can_switch)
    {
        blackboard.body_motion = Some(BodyMotion::Kick {
            ball_position: ball,
            ball_velocity: blackboard
                .world_state
                .ball
                .map_or(Vector2::zeros(), |ball| ball.ball_in_ground_velocity),
            kick_direction: orientation,
            target_speed,
            soft,
            quick,
            strong,
        });
        return Status::Success;
    }
    if blackboard.last_motion_type != Some(MotionType::Walk) && !can_switch {
        // Cancel a kick while waiting for another motion to become available.
        return stand(blackboard);
    }

    // Only get within kick range. The kick policy handles final alignment.
    let Some(toward_ball) = ball.coords().try_normalize(f32::EPSILON) else {
        return stand(blackboard);
    };
    let approach = ball - toward_ball * blackboard.parameters.kicking.approach_ball_standoff;
    walk_to(
        blackboard,
        Pose2::from_parts(approach, Orientation2::from_vector(ball.coords())),
        blackboard.parameters.walking.speed.kicking,
        OrientationMode::AlignWithPath,
        walk.normal_distance_to_be_aligned,
        walk.hysteresis,
    )
}

fn resolve_target(blackboard: &Blackboard) -> Option<Point2<Field>> {
    let action = blackboard.ball_action?;
    let transform = blackboard.world_state.robot.ground_to_field?;
    let ball = transform * kick_ball_position_in_ground(blackboard, &transform)?;
    let field = &blackboard.field_dimensions;
    let parameters = &blackboard.parameters.kicking.strategy;
    if ball.x().abs() >= field.length / 2.0 || ball.y().abs() >= field.width / 2.0 {
        return None;
    }
    let openings = match action {
        BallAction::Shoot { opening: selected } => {
            let dx = field.length / 2.0 - ball.x();
            if parameters.maximum_shot_distance <= dx {
                return None;
            }
            let dy = (parameters.maximum_shot_distance.powi(2) - dx.powi(2)).sqrt();
            goal_openings(
                ball,
                transform,
                &blackboard.world_state.obstacles,
                field,
                parameters.obstacle_clearance,
            )
            .into_iter()
            .filter_map(|opening| {
                let lower = opening.0.y().max(selected.0.y()).max(ball.y() - dy);
                let upper = opening.1.y().min(selected.1.y()).min(ball.y() + dy);
                (lower < upper).then_some(LineSegment(
                    point![opening.0.x(), lower],
                    point![opening.0.x(), upper],
                ))
            })
            .filter(|opening| {
                LineSegment(ball, opening.0).angle(LineSegment(ball, opening.1))
                    >= parameters.minimum_shot_angle
            })
            .collect()
        }
        BallAction::Dribble | BallAction::Clear => {
            let radius = field.ball_radius + parameters.obstacle_clearance;
            let distance = parameters.maximum_shot_distance;
            // A forward cross-section bounds every target to the configured reach.
            let step = distance / std::f32::consts::SQRT_2;
            let x = (ball.x() + step).min(field.length / 2.0 - radius);
            let half_width = field.width / 2.0 - radius;
            let lower = (ball.y() - step).max(-half_width);
            let upper = (ball.y() + step).min(half_width);
            if step <= 0.0 || x <= ball.x() || lower >= upper {
                return None;
            }
            let obstacles: Vec<_> = blackboard
                .world_state
                .obstacles
                .iter()
                .filter(|obstacle| obstacle.kind != ObstacleKind::Ball)
                .map(|obstacle| {
                    Circle::new(
                        transform * obstacle.position,
                        obstacle.radius_at_foot_height + radius,
                    )
                })
                .collect();
            visible_segments(
                ball,
                LineSegment(point![x, lower], point![x, upper]),
                &obstacles,
            )
        }
    };
    let target_x = openings.first()?.0.x();
    let preferred = (action == BallAction::Dribble).then(|| {
        let goal = point![field.length / 2.0, 0.0];
        ball + (goal - ball) * ((target_x - ball.x()) / (goal.x() - ball.x()))
    });
    let opening = openings.into_iter().max_by(|a, b| {
        if let Some(preferred) = preferred {
            let ordering = b
                .squared_distance_to(preferred)
                .total_cmp(&a.squared_distance_to(preferred));
            if !ordering.is_eq() {
                return ordering;
            }
        }
        LineSegment(ball, a.0)
            .angle(LineSegment(ball, a.1))
            .total_cmp(&LineSegment(ball, b.0).angle(LineSegment(ball, b.1)))
    })?;
    // Advance toward goal when that ray is inside a safe opening.
    if let Some(preferred) = preferred
        && opening.projection_factor(preferred) > 0.0
        && opening.projection_factor(preferred) < 1.0
    {
        return Some(preferred);
    }
    // Bisect the angle, not the goal-line length, to maximize angular edge clearance.
    let direction = (opening.0 - ball).normalize() + (opening.1 - ball).normalize();
    let target = ball + direction * ((opening.0.x() - ball.x()) / direction.x());
    (target.x().is_finite() && target.y().is_finite()).then_some(target)
}
