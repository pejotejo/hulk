use coordinate_systems::{Field, Ground};
use geometry::line_segment::LineSegment;
use linear_algebra::{Orientation2, Point2, Pose2, Vector2, point};
use types::{
    behavior_tree::Status,
    motion_command::{BodyMotion, OrientationMode},
    motion_type::MotionType,
    path::traits::PathProgress,
};

use crate::{
    actions::stand, conditions::is_close_to_ball, node::Blackboard,
    switch_motion_type::is_allowed_to_start_kick, walk::walk_to,
};

use super::{
    geometry::{BALL_FIELD_BOUNDARY_TOLERANCE, goal_openings, pass_openings},
    kick_ball_in_ground,
    passing::{is_executing_announced_pass, pass_is_announced, pass_receiver_position},
    strategy::{BallAction, own_non_penalty_restart, select_restart_ball_action},
};

/// Execute a restart pass, revalidating its receiver and corridor before every kick.
/// Ordinary kicking keeps the base kick subtree.
pub fn execute_ball_action(blackboard: &mut Blackboard) -> Status {
    if !matches!(blackboard.ball_action, Some(BallAction::Pass { .. })) {
        return Status::Failure;
    }
    let target = resolve_target(blackboard).or_else(|| {
        // Reselect only while our restart is still active.
        if own_non_penalty_restart(blackboard) {
            blackboard.last_announced_pass = None;
            select_restart_ball_action(blackboard);
            resolve_target(blackboard)
        } else {
            None
        }
    });
    let Some(target_in_field) = target else {
        blackboard.ball_action = None;
        blackboard.kick_target = None;
        blackboard.last_announced_pass = None;
        return stand(blackboard);
    };
    let Some(BallAction::Pass { receiver }) = blackboard.ball_action else {
        return Status::Failure;
    };
    let Some(transform) = blackboard.world_state.robot.ground_to_field else {
        return Status::Failure;
    };
    let Some((ball, ball_velocity)) = kick_ball_in_ground(blackboard, &transform) else {
        return Status::Failure;
    };
    let target = transform.inverse() * target_in_field;
    blackboard.kick_target = Some(target);
    let Some(direction) = (target - ball).try_normalize(f32::EPSILON) else {
        return stand(blackboard);
    };
    let orientation = Orientation2::from_vector(direction);
    let kicking = &blackboard.parameters.kicking;
    let strong = kicking.allow_strong_kicks
        && (super::kickoff::own_kickoff(blackboard)
            || (target - ball).norm()
                >= kicking
                    .strong_kick_min_target_distance
                    .max(blackboard.field_dimensions.penalty_area_width));
    let target_speed = kicking.target_speed;
    let was_kicking = blackboard.last_motion_type == Some(MotionType::Kick);
    let can_switch = is_allowed_to_start_kick(blackboard);
    if ((ball.x() >= 0.0 && is_close_to_ball(blackboard)) || (was_kicking && !can_switch))
        && (was_kicking || can_switch)
    {
        if !pass_is_announced(blackboard, receiver, target_in_field) {
            return stand(blackboard);
        }
        blackboard.body_motion = Some(BodyMotion::Kick {
            ball_position: ball,
            ball_velocity,
            kick_direction: orientation,
            target_speed,
            soft: false,
            quick: false,
            strong,
        });
        blackboard.last_kick_target = Some(target_in_field);
        if was_kicking {
            super::use_last_kick_settings(blackboard);
        }
        return Status::Success;
    }
    approach_ball(blackboard, ball)
}

pub(crate) fn approach_restart_ball(blackboard: &mut Blackboard) -> Status {
    if !own_non_penalty_restart(blackboard) {
        return Status::Failure;
    }
    if is_close_to_ball(blackboard) {
        return stand(blackboard);
    }
    let Some(transform) = blackboard.world_state.robot.ground_to_field else {
        return Status::Failure;
    };
    let Some((ball, _)) = kick_ball_in_ground(blackboard, &transform) else {
        return Status::Failure;
    };
    approach_ball(blackboard, ball)
}

fn approach_ball(blackboard: &mut Blackboard, ball: Point2<Ground>) -> Status {
    if blackboard.last_motion_type != Some(MotionType::Walk)
        && !is_allowed_to_start_kick(blackboard)
    {
        return stand(blackboard);
    }
    // Approach the observed ball; the kick policy handles final alignment.
    let Some(toward_ball) = ball.coords().try_normalize(f32::EPSILON) else {
        return stand(blackboard);
    };
    let approach = ball - toward_ball * blackboard.parameters.kicking.approach_ball_standoff;
    let walking = blackboard.parameters.walking.walk_and_stand;
    let status = walk_to(
        blackboard,
        Pose2::from_parts(approach, Orientation2::from_vector(ball.coords())),
        blackboard.parameters.walking.speed.kicking,
        OrientationMode::LookAt {
            target: ball,
            tolerance: walking.orientation_tolerance,
        },
        walking.normal_distance_to_be_aligned,
        walking.hysteresis,
    );
    if let Some(BodyMotion::Walk { path, speed, .. }) = &mut blackboard.body_motion {
        *speed = blackboard.parameters.walking.speed.kicking;
        if path.forward(Point2::origin()).x() < 0.0 {
            blackboard.body_motion = Some(BodyMotion::WalkWithVelocity {
                velocity: Vector2::zeros(),
                angular_velocity: blackboard
                    .walking_velocity_limits
                    .scale_angular_axis(Orientation2::from_vector(ball.coords()).angle()),
            });
        }
    }
    status
}

pub(crate) fn resolve_target(blackboard: &Blackboard) -> Option<Point2<Field>> {
    let action = blackboard.ball_action?;
    let transform = blackboard.world_state.robot.ground_to_field?;
    let ball = transform * kick_ball_in_ground(blackboard, &transform)?.0;
    let field = &blackboard.field_dimensions;
    let parameters = &blackboard.parameters.kicking.strategy;
    parameters.validate().ok()?;
    if !ball.inner.iter().all(|value| value.is_finite())
        || !field.length.is_finite()
        || !field.width.is_finite()
        || field.length <= 0.0
        || field.width <= 0.0
        || ball.x().abs() > field.length / 2.0 + BALL_FIELD_BOUNDARY_TOLERANCE
        || ball.y().abs() > field.width / 2.0 + BALL_FIELD_BOUNDARY_TOLERANCE
    {
        return None;
    }
    match action {
        BallAction::Pass { receiver } => {
            if !blackboard.has_team_message_budget() {
                return None;
            }
            if let Some(target) = super::kickoff::kickoff_pass_target(blackboard, receiver) {
                return Some(target);
            }
            let executing = is_executing_announced_pass(blackboard, receiver);
            if !own_non_penalty_restart(blackboard) && !executing {
                return None;
            }
            let receiver_position = pass_receiver_position(blackboard, ball, receiver)?;
            if !executing
                && !crate::substates::restart_pass_is_open(blackboard, receiver_position)
                && blackboard
                    .world_state
                    .filtered_game_controller_state
                    .as_ref()
                    .is_some_and(|game| {
                        matches!(
                            game.sub_state,
                            Some(
                                hsl_network_messages::SubState::CornerKick
                                    | hsl_network_messages::SubState::ThrowIn
                            )
                        )
                    })
            {
                return None;
            }
            let mut passing = parameters.passing.clone();
            if executing {
                passing.minimum_distance = 0.0;
            }
            pass_openings(
                ball,
                receiver_position,
                transform,
                &blackboard.world_state.obstacles,
                field,
                &passing,
                parameters.obstacle_clearance,
            )
            .into_iter()
            .filter_map(|opening| {
                let direction = (opening.0 - ball).normalize() + (opening.1 - ball).normalize();
                let target =
                    geometry::line::Line2::from(opening).intersection(&geometry::line::Line2 {
                        point: ball,
                        direction,
                    });
                target
                    .inner
                    .iter()
                    .all(|value| value.is_finite())
                    .then_some((
                        target,
                        LineSegment(ball, opening.0).angle(LineSegment(ball, opening.1)),
                    ))
            })
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(target, _)| target)
        }
        BallAction::Shoot { opening } => {
            let range = parameters.maximum_shot_distance;
            let dx = field.length / 2.0 - ball.x();
            if dx < 0.0 || dx >= range {
                return None;
            }
            let reachable_y = (range.powi(2) - dx.powi(2)).sqrt();
            goal_openings(
                ball,
                transform,
                &blackboard.world_state.obstacles,
                field,
                parameters.obstacle_clearance,
            )
            .into_iter()
            .filter_map(|visible| {
                let lower = opening.0.y().max(visible.0.y()).max(ball.y() - reachable_y);
                let upper = opening.1.y().min(visible.1.y()).min(ball.y() + reachable_y);
                if lower >= upper {
                    return None;
                }
                let refined =
                    LineSegment(point![opening.0.x(), lower], point![opening.1.x(), upper]);
                let angle = LineSegment(ball, refined.0).angle(LineSegment(ball, refined.1));
                (angle >= parameters.minimum_shot_angle).then_some((refined.center(), angle))
            })
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(target, _)| target)
        }
        BallAction::QuickKick { .. } => {
            Some(if blackboard.last_motion_type == Some(MotionType::Kick) {
                blackboard
                    .last_kick_target
                    .unwrap_or(point![field.length / 2.0, 0.0])
            } else {
                point![field.length / 2.0, 0.0]
            })
        }
        BallAction::Clear | BallAction::Dribble => None,
    }
}
