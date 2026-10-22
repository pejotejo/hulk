use coordinate_systems::{Field, Ground};
use geometry::{line::Line2, line_segment::LineSegment};
use linear_algebra::{Isometry2, Orientation2, Point2, Pose2, Vector2, point, vector};
use types::{
    behavior_tree::Status,
    motion_command::{BodyMotion, MotionCommand, OrientationMode},
    motion_type::MotionType,
    path::traits::PathProgress,
};

use crate::{
    actions::stand,
    conditions::is_close_to_ball,
    node::Blackboard,
    switch_motion_type::{is_allowed_to_start_kick, is_allowed_to_switch},
    walk::walk_to,
};

use super::{
    geometry::{BALL_FIELD_BOUNDARY_TOLERANCE, goal_openings, pass_openings, travel_openings},
    kick_ball_in_ground, kick_ball_position_in_ground, pass_receiver_position,
    passing::pass_is_announced,
    strategy::BallAction,
};

/// Revalidate the selected corridor, choosing another safe action if it has disappeared.
pub fn execute_ball_action(blackboard: &mut Blackboard) -> Status {
    if crate::conditions::goalkeeper_clearance_direction(blackboard).is_some() {
        if blackboard.last_motion_type != Some(MotionType::Walk)
            && !is_allowed_to_switch(blackboard)
        {
            blackboard.ball_action = None;
            blackboard.kick_target = None;
            return stand(blackboard);
        }
        return super::passing::yield_goalkeeper_clearance(blackboard);
    }
    let target = resolve_target(blackboard).or_else(|| {
        if super::kickoff::needs_kickoff_updates(blackboard) {
            return None;
        }
        blackboard.last_announced_pass = None;
        super::strategy::select_ball_action(blackboard);
        resolve_target(blackboard)
    });
    let Some(target) = target else {
        blackboard.ball_action = None;
        blackboard.kick_target = None;
        blackboard.last_announced_pass = None;
        return stand(blackboard);
    };
    let ground_to_field = blackboard.world_state.robot.ground_to_field.unwrap();
    let (ball, ball_velocity) = kick_ball_in_ground(blackboard, &ground_to_field).unwrap();
    let pass_ready = match blackboard.ball_action {
        Some(BallAction::Pass { receiver }) => pass_is_announced(blackboard, receiver, target),
        _ => true,
    };
    let target = ground_to_field.inverse() * target;
    blackboard.kick_target = Some(target);
    let direction = (target - ball).normalize();
    let orientation = Orientation2::from_vector(direction);
    let kicking = &blackboard.parameters.kicking;
    let walk = blackboard.parameters.walking.walk_and_stand;
    let controlled = matches!(
        blackboard.ball_action,
        Some(BallAction::Dribble | BallAction::Pass { .. })
    );
    let force_strong = matches!(
        blackboard.ball_action,
        Some(BallAction::Shoot { .. } | BallAction::ShootAtGoalCenter | BallAction::Clear)
    ) || (matches!(blackboard.ball_action, Some(BallAction::Pass { .. }))
        && super::kickoff::own_kickoff(blackboard));
    let strong = kicking.allow_strong_kicks && force_strong;
    let was_kicking = blackboard.last_motion_type == Some(MotionType::Kick);
    let target_speed = if blackboard.ball_action == Some(BallAction::Dribble) {
        kicking.dribble_target_speed.min(kicking.target_speed)
    } else {
        kicking.target_speed
    };
    let receiving = super::reception::is_receiving_ball(blackboard);
    let ready_to_receive = !receiving || super::reception::is_ready_to_receive_kick(blackboard);
    let field_ball = ground_to_field * ball;
    let field = &blackboard.field_dimensions;
    let urgent = (field_ball.x() < -field.length / 2.0 + field.penalty_area_length
        && field_ball.y().abs() < field.penalty_area_width / 2.0)
        || crate::conditions::opponent_near_ball(blackboard, kicking.strategy.pressure_distance);
    let quick = (receiving
        && ready_to_receive
        && blackboard.world_state.robot.player_number
            != blackboard.parameters.goalkeeper.player_number)
        || (!controlled
            && (kicking.quick
                || urgent
                || (was_kicking
                    && matches!(
                        blackboard.last_motion_command,
                        MotionCommand::Kick { quick: true, .. }
                    ))));
    let can_switch = is_allowed_to_start_kick(blackboard);
    let close_to_ball = is_close_to_ball(blackboard);
    if (((was_kicking || (!receiving && ball.x() >= 0.0)) && close_to_ball)
        || (receiving && ready_to_receive)
        || (was_kicking && !can_switch))
        && (was_kicking || can_switch)
    {
        if !pass_ready {
            return stand(blackboard);
        }
        blackboard.body_motion = Some(BodyMotion::Kick {
            ball_position: ball,
            ball_velocity,
            kick_direction: orientation,
            target_speed,
            soft: false,
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
    let projected = ground_to_field.inverse()
        * blackboard
            .projected_ball_position(ground_to_field * ball, ground_to_field * ball_velocity);
    let Some(toward_ball) = projected.coords().try_normalize(f32::EPSILON) else {
        return stand(blackboard);
    };
    let approach = projected - toward_ball * blackboard.parameters.kicking.approach_ball_standoff;
    let status = walk_to(
        blackboard,
        Pose2::from_parts(approach, Orientation2::from_vector(projected.coords())),
        blackboard.parameters.walking.speed.kicking,
        OrientationMode::AlignWithPath,
        walk.normal_distance_to_be_aligned,
        walk.hysteresis,
    );
    if let Some(BodyMotion::Walk { path, speed, .. }) = &mut blackboard.body_motion {
        // The kick policy takes over at range; do not slow the approach several metres early.
        *speed = blackboard.parameters.walking.speed.kicking;
        let forward = path.forward(Point2::origin());
        if forward.x() < 0.0 {
            blackboard.body_motion = Some(BodyMotion::WalkWithVelocity {
                velocity: Vector2::zeros(),
                angular_velocity: blackboard
                    .walking_velocity_limits
                    .scale_angular_axis(Orientation2::from_vector(forward).angle()),
            });
        }
    }
    status
}

pub(super) fn resolve_target(blackboard: &Blackboard) -> Option<Point2<Field>> {
    let action = blackboard.ball_action?;
    let transform = blackboard.world_state.robot.ground_to_field?;
    let ball = transform * kick_ball_position_in_ground(blackboard, &transform)?;
    let field = &blackboard.field_dimensions;
    let parameters = &blackboard.parameters.kicking.strategy;
    parameters.validate().ok()?;
    if !ball.x().is_finite()
        || !ball.y().is_finite()
        || !field.length.is_finite()
        || !field.width.is_finite()
        || field.length <= 0.0
        || field.width <= 0.0
        || ball.x().abs() > field.length / 2.0 + BALL_FIELD_BOUNDARY_TOLERANCE
        || ball.y().abs() > field.width / 2.0 + BALL_FIELD_BOUNDARY_TOLERANCE
    {
        return None;
    }
    resolve_action_target(blackboard, action, transform, ball)
}

fn resolve_action_target(
    blackboard: &Blackboard,
    action: BallAction,
    transform: Isometry2<Ground, Field>,
    ball: Point2<Field>,
) -> Option<Point2<Field>> {
    let field = &blackboard.field_dimensions;
    let parameters = &blackboard.parameters.kicking.strategy;
    let openings = match action {
        BallAction::ShootAtGoalCenter => {
            let target = point![field.length / 2.0, 0.0];
            return ((target - ball).norm() <= parameters.maximum_shot_distance
                && goal_openings(
                    ball,
                    transform,
                    &blackboard.world_state.obstacles,
                    field,
                    parameters.obstacle_clearance,
                )
                .iter()
                .any(|opening| opening.0.y() < 0.0 && opening.1.y() > 0.0))
            .then_some(target);
        }
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
        BallAction::Pass { .. } if !blackboard.has_team_message_budget() => return None,
        BallAction::Pass { receiver } => {
            if let Some(target) = super::kickoff::kickoff_pass_target(blackboard, receiver) {
                return Some(target);
            }
            let mut passing = parameters.passing.clone();
            // Finish a launched pass as its remaining travel shrinks; still check obstacles.
            if super::passing::is_executing_announced_pass(blackboard, receiver) {
                passing.minimum_distance = 0.0;
            }
            pass_openings(
                ball,
                pass_receiver_position(blackboard, ball, receiver)?,
                transform,
                &blackboard.world_state.obstacles,
                field,
                &passing,
                parameters.obstacle_clearance,
            )
        }
        BallAction::Dribble | BallAction::Clear => {
            let distance = if action == BallAction::Dribble {
                parameters.maximum_dribble_distance
            } else {
                parameters.maximum_shot_distance.min(field.length / 2.0)
            };
            travel_openings(
                ball,
                transform,
                &blackboard.world_state.obstacles,
                field,
                distance,
                parameters.obstacle_clearance,
            )
        }
    };
    if action == BallAction::Dribble {
        let goal = point![field.length / 2.0, 0.0];
        return openings
            .into_iter()
            .map(|opening| goalward_target(ball, goal, opening))
            .min_by(|a, b| {
                (a - ball)
                    .angle(&(goal - ball))
                    .total_cmp(&(b - ball).angle(&(goal - ball)))
            });
    }

    openings
        .into_iter()
        .filter_map(|mut opening| {
            if action == BallAction::Clear {
                if opening.0.x().max(opening.1.x()) <= ball.x() {
                    return None;
                }
                if opening.0.x() < ball.x() || opening.1.x() < ball.x() {
                    let forward_boundary = Line2 {
                        point: ball,
                        direction: vector![0.0, 1.0],
                    };
                    let intersection = Line2::from(opening).intersection(&forward_boundary);
                    if opening.0.x() < ball.x() {
                        opening.0 = intersection;
                    } else {
                        opening.1 = intersection;
                    }
                }
            }
            // Bisect the visible angle to maximize angular edge clearance.
            let direction = (opening.0 - ball).normalize() + (opening.1 - ball).normalize();
            let target = Line2::from(opening).intersection(&Line2 {
                point: ball,
                direction,
            });
            (target.x().is_finite()
                && target.y().is_finite()
                && (action != BallAction::Clear || target.x() > ball.x()))
            .then_some((
                target,
                LineSegment(ball, opening.0).angle(LineSegment(ball, opening.1)),
            ))
        })
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(target, _)| target)
}

pub(super) fn goalward_target(
    ball: Point2<Field>,
    goal: Point2<Field>,
    opening: LineSegment<Field>,
) -> Point2<Field> {
    // Stay inside tangent boundaries, including when the preferred ray is blocked.
    let inset = (opening.1 - opening.0) * (0.001 / opening.length()).min(0.5);
    let opening = LineSegment(opening.0 + inset, opening.1 - inset);
    let preferred = Line2::from(opening).intersection(&Line2::from_points(ball, goal));
    if (0.0..=1.0).contains(&opening.projection_factor(preferred))
        && (preferred - ball).dot(&(goal - ball)) > 0.0
    {
        preferred
    } else if (opening.0 - ball).angle(&(goal - ball)) <= (opening.1 - ball).angle(&(goal - ball)) {
        opening.0
    } else {
        opening.1
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::node::LastBall;
    use types::{
        field_dimensions::{FieldDimensions, Side},
        obstacles::Obstacle,
        world_state::BallSource,
    };

    #[test]
    fn walking_hands_off_immediately_and_urgent_shots_and_clears_are_quick() {
        let mut board = Blackboard {
            field_dimensions: FieldDimensions::SPL_2025,
            ball_action: Some(BallAction::ShootAtGoalCenter),
            ball: Some(LastBall {
                position: point![0.5, 0.0],
                velocity: Vector2::zeros(),
                age: Default::default(),
                field_side: Side::Left,
                source: BallSource::Own,
            }),
            last_motion_type: Some(MotionType::Walk),
            ..Default::default()
        };
        board.world_state.robot.ground_to_field = Some(Default::default());
        board.world_state.now = ros_z::time::Time::from_nanos(1_000_000_000);
        board.last_motion_switch_time =
            board.world_state.now - std::time::Duration::from_millis(100);
        board.parameters.control.allow_switch.walk = std::time::Duration::from_millis(500);
        board.parameters.control.allow_switch.stand_up = std::time::Duration::from_millis(500);
        board.parameters.kicking.strategy.maximum_shot_distance = 12.0;
        board.parameters.kicking.strategy.pressure_distance = 0.8;
        board.parameters.kicking.kick_activation_distance = 1.3;
        board.parameters.kicking.allow_strong_kicks = true;
        board.parameters.kicking.strong_kick_min_target_distance = 3.0;
        assert_eq!(execute_ball_action(&mut board), Status::Success);
        assert!(matches!(
            board.body_motion,
            Some(BodyMotion::Kick {
                strong: true,
                quick: false,
                soft: false,
                ..
            })
        ));

        board.world_state.obstacles = vec![Obstacle::robot(point![0.4, 0.6], 0.1, 0.1)];
        execute_ball_action(&mut board);
        assert!(matches!(
            board.body_motion,
            Some(BodyMotion::Kick {
                strong: true,
                quick: true,
                ..
            })
        ));

        board.world_state.obstacles.clear();
        board.world_state.robot.ground_to_field =
            Some(Isometry2::from_parts(vector![-4.2, 0.0], 0.0));
        board.ball.as_mut().unwrap().position = point![-3.7, 0.0];
        board.ball_action = Some(BallAction::Clear);
        execute_ball_action(&mut board);
        assert!(matches!(
            board.body_motion,
            Some(BodyMotion::Kick {
                strong: true,
                quick: true,
                soft: false,
                ..
            })
        ));

        board.ball.as_mut().unwrap().velocity = vector![0.4, 0.0];
        execute_ball_action(&mut board);
        assert!(matches!(
            board.body_motion,
            Some(BodyMotion::Kick {
                strong: true,
                quick: true,
                soft: false,
                ..
            })
        ));

        board.last_motion_type = Some(MotionType::StandUp);
        execute_ball_action(&mut board);
        assert!(matches!(board.body_motion, Some(BodyMotion::Stand)));
    }

    #[test]
    fn goal_shots_are_strong_even_when_close_or_rolling_and_honor_quick() {
        let mut blackboard = Blackboard {
            field_dimensions: FieldDimensions::SPL_2025,
            ball_action: Some(BallAction::ShootAtGoalCenter),
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
        blackboard.field_dimensions.length = 6.0;
        blackboard.parameters.kicking.target_speed = 3.4;
        blackboard.parameters.kicking.strategy.maximum_shot_distance = 12.0;
        blackboard.parameters.kicking.soft = true;
        blackboard.parameters.kicking.allow_strong_kicks = true;
        blackboard
            .parameters
            .kicking
            .strong_kick_min_target_distance = 3.0;
        blackboard.parameters.kicking.kick_activation_distance = 3.0;
        assert_eq!(execute_ball_action(&mut blackboard), Status::Success);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick {
                strong: true,
                soft: false,
                target_speed: 3.4,
                ..
            })
        ));

        blackboard.parameters.ball.last_ball_timeout = std::time::Duration::from_secs(1);
        blackboard.ball.as_mut().unwrap().velocity = vector![-1.5, 0.0];
        blackboard.last_motion_type = Some(MotionType::Walk);
        execute_ball_action(&mut blackboard);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick {
                strong: true,
                quick: false,
                soft: false,
                target_speed: 3.4,
                ..
            })
        ));
        blackboard.parameters.kicking.quick = true;
        execute_ball_action(&mut blackboard);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick { quick: true, .. })
        ));
        blackboard.last_motion_command = MotionCommand::from_partial_motions(
            blackboard.body_motion.clone().unwrap(),
            types::motion_command::HeadMotion::Center {
                image_region_target: Default::default(),
            },
        );
        blackboard.last_motion_type = Some(MotionType::Kick);
        if let MotionCommand::Kick { strong, .. } = &mut blackboard.last_motion_command {
            *strong = false;
        }
        blackboard.parameters.kicking.quick = false;
        blackboard.ball.as_mut().unwrap().velocity = Vector2::zeros();
        execute_ball_action(&mut blackboard);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick {
                strong: true,
                quick: true,
                ..
            })
        ));
        blackboard.last_motion_type = Some(MotionType::Walk);
        blackboard.world_state.now =
            blackboard.world_state.now + std::time::Duration::from_millis(1);
        execute_ball_action(&mut blackboard);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick {
                strong: true,
                quick: false,
                ..
            })
        ));

        blackboard.parameters.kicking.allow_strong_kicks = false;
        assert_eq!(execute_ball_action(&mut blackboard), Status::Success);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick { strong: false, .. })
        ));

        blackboard.ball_action = Some(BallAction::Dribble);
        blackboard.parameters.kicking.allow_strong_kicks = true;
        blackboard
            .parameters
            .kicking
            .strategy
            .maximum_dribble_distance = 1.0;
        blackboard.parameters.kicking.dribble_target_speed = 1.2;
        blackboard.last_motion_type = Some(MotionType::Kick);
        assert!(matches!(
            blackboard.last_motion_command,
            MotionCommand::Kick { quick: true, .. }
        ));
        execute_ball_action(&mut blackboard);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick {
                strong: false,
                quick: false,
                soft: false,
                target_speed: 1.2,
                ..
            })
        ));
    }
}
