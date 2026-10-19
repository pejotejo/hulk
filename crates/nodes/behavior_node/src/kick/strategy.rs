use coordinate_systems::Field;
use geometry::{Distance, line::Line2, line_segment::LineSegment};
use hsl_network_messages::{PlayerNumber, SubState, Team};
use linear_algebra::{Point2, point};
use ros_z::Message;
use serde::{Deserialize, Serialize};
use types::{behavior_tree::Status, obstacles::ObstacleKind};

use crate::node::Blackboard;

use super::{
    geometry::{goal_openings, pass_openings_with_progress, travel_openings},
    kick_ball_position_in_ground,
    passing::has_fresh_pass_acknowledgement,
};

/// Tactical intent; execution revalidates the corridor before choosing a motion.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Message)]
pub enum BallAction {
    Shoot { opening: LineSegment<Field> },
    Pass { receiver: PlayerNumber },
    Dribble,
    Clear,
    Cutback,
    Hold,
}

pub fn select_ball_action(blackboard: &mut Blackboard) -> Status {
    blackboard.ball_action_active = true;
    let previous = blackboard.ball_action;
    let next = choose_ball_action(blackboard);
    // Keep a still-safe intent while the motion policy is committed. Corridor
    // invalidation always wins over this short tactical hysteresis.
    let retain_committed = blackboard.last_motion_type
        == Some(types::motion_type::MotionType::Kick)
        && !crate::switch_motion_type::is_allowed_to_switch(blackboard)
        && previous.is_some_and(|action| {
            blackboard
                .world_state
                .robot
                .ground_to_field
                .and_then(|transform| {
                    kick_ball_position_in_ground(blackboard, &transform).and_then(|ball| {
                        resolve_action_target(blackboard, action, transform * ball)
                    })
                })
                .is_some()
        });
    blackboard.ball_action = if retain_committed { previous } else { next };
    if !matches!(blackboard.ball_action, Some(BallAction::Pass { .. })) {
        blackboard.last_announced_pass = None;
        blackboard.pass_started_at = None;
    } else if blackboard.ball_action != previous {
        blackboard.last_announced_pass = None;
        blackboard.pass_started_at = Some(blackboard.world_state.now);
    }
    if blackboard.ball_action.is_some() {
        Status::Success
    } else {
        Status::Failure
    }
}

fn choose_ball_action(blackboard: &Blackboard) -> Option<BallAction> {
    let world = &blackboard.world_state;
    let field = &blackboard.field_dimensions;
    let parameters = &blackboard.parameters.kicking.strategy;
    parameters.validate().ok()?;
    let transform = world.robot.ground_to_field?;
    let ball_in_ground = kick_ball_position_in_ground(blackboard, &transform)?;
    let ball = transform * ball_in_ground;
    if !valid_ball(blackboard, ball) {
        return None;
    }

    let restart = own_restart(blackboard);
    let inward_restart = matches!(
        restart,
        Some(SubState::CornerKick | SubState::ThrowIn | SubState::IndirectFreeKick)
    );
    let own_goal = LineSegment(
        point![-field.length / 2.0, -field.goal_inner_width / 2.0],
        point![-field.length / 2.0, field.goal_inner_width / 2.0],
    );
    let was_clearing = blackboard.ball_action == Some(BallAction::Clear);
    let clear_margin = if was_clearing {
        parameters.clear_distance_hysteresis
    } else {
        0.0
    };
    let under_pressure = world.obstacles.iter().any(|obstacle| {
        matches!(
            obstacle.kind,
            ObstacleKind::Robot | ObstacleKind::Person | ObstacleKind::Unknown
        ) && (obstacle.position - ball_in_ground).norm() - obstacle.radius_at_foot_height
            < parameters.pressure_distance + clear_margin
    });
    if !inward_restart
        && under_pressure
        && own_goal.distance_to(ball) < parameters.clear_goal_distance + clear_margin
        && resolve_action_target(blackboard, BallAction::Clear, ball).is_some()
    {
        return Some(BallAction::Clear);
    }

    if !inward_restart && let Some(opening) = select_shot_opening(blackboard, ball) {
        return Some(BallAction::Shoot { opening });
    }

    if blackboard.has_team_message_budget()
        && blackboard
            .pass_cooldown_until
            .is_none_or(|until| world.now >= until)
    {
        let mut best: Option<(PlayerNumber, f32)> = None;
        let mut retained = None;
        for (receiver, _) in blackboard.timed_player_states.iter() {
            let Some(position) = pass_receiver_position(blackboard, ball, receiver) else {
                continue;
            };
            let action = BallAction::Pass { receiver };
            let Some(target) = resolve_action_target(blackboard, action, ball) else {
                continue;
            };
            // Preparing a receiver costs possession time. Under immediate
            // pressure, use an action we can execute now unless this pass has
            // already completed its normal announcement/acknowledgement exchange.
            if under_pressure && !has_fresh_pass_acknowledgement(blackboard, receiver, target) {
                continue;
            }
            // Advance in open play; on the attacking goal line reward an inward
            // cutback rather than demanding impossible positive x progress.
            let score = if matches!(restart, Some(SubState::CornerKick)) {
                -((position - attacking_reception_position(blackboard, ball)).norm())
            } else {
                position.x() - ball.x()
            };
            if best.is_none_or(|(_, old_score)| score > old_score) {
                best = Some((receiver, score));
            }
            if blackboard.ball_action == Some(action) {
                retained = Some((receiver, score));
            }
        }
        if let Some((receiver, score)) = best {
            let receiver = retained
                .filter(|(_, old_score)| {
                    score <= old_score + parameters.passing.selection_hysteresis
                })
                .map_or(receiver, |(receiver, _)| receiver);
            return Some(BallAction::Pass { receiver });
        }
    }

    if !inward_restart
        && ball.x() <= 0.0
        && under_pressure
        && resolve_action_target(blackboard, BallAction::Clear, ball).is_some()
    {
        // With no immediate shot or acknowledged pass, release own-half
        // pressure through a validated forward corridor instead of a soft advance.
        return Some(BallAction::Clear);
    }

    if world
        .filtered_game_controller_state
        .as_ref()
        .is_none_or(|game| game.sub_state.is_none())
        && (!under_pressure || was_clearing)
        && let Some(target) = resolve_action_target(blackboard, BallAction::Clear, ball)
        && target.x() - ball.x() > parameters.maximum_dribble_distance
    {
        // Keep the existing regular-kick policy for a validated lane that gains
        // more than the dribble range forward when there is room to start it.
        // Continue an existing clear under pressure rather than oscillating
        // between policies. A long lateral exit alone keeps soft control.
        return Some(BallAction::Clear);
    }

    let action = if inward_restart {
        BallAction::Cutback
    } else {
        BallAction::Dribble
    };
    Some(
        if resolve_action_target(blackboard, action, ball).is_some() {
            action
        } else if ball.x() > field.length / 2.0 - field.penalty_area_length
            && resolve_action_target(blackboard, BallAction::Cutback, ball).is_some()
        {
            // A blocked attacker on the goal line can improve the angle by
            // moving back into the field rather than demanding more x progress.
            BallAction::Cutback
        } else {
            BallAction::Hold
        },
    )
}

fn select_shot_opening(blackboard: &Blackboard, ball: Point2<Field>) -> Option<LineSegment<Field>> {
    let field = &blackboard.field_dimensions;
    let parameters = &blackboard.parameters.kicking.strategy;
    let goal_x = field.length / 2.0;
    let dx = goal_x - ball.x();
    if dx <= 0.0 || dx >= parameters.maximum_shot_distance {
        return None;
    }
    let reachable_y = (parameters.maximum_shot_distance.powi(2) - dx.powi(2)).sqrt();
    let mut best: Option<(LineSegment<Field>, f32)> = None;
    let mut retained = None;
    for opening in goal_openings(
        ball,
        blackboard.world_state.robot.ground_to_field?,
        &blackboard.world_state.obstacles,
        field,
        parameters.obstacle_clearance,
    ) {
        let lower = opening.0.y().max(ball.y() - reachable_y);
        let upper = opening.1.y().min(ball.y() + reachable_y);
        if lower >= upper {
            continue;
        }
        let opening = LineSegment(point![goal_x, lower], point![goal_x, upper]);
        let angle = opening_angle(ball, opening);
        if angle < parameters.minimum_shot_angle {
            continue;
        }
        if best.is_none_or(|(_, old_angle)| angle > old_angle) {
            best = Some((opening, angle));
        }
        if let Some(BallAction::Shoot { opening: previous }) = blackboard.ball_action
            && previous.0.x() == goal_x
            && (lower..upper).contains(&previous.center().y())
        {
            retained = Some((opening, angle));
        }
    }
    best.map(|(opening, angle)| {
        retained
            .filter(|(_, retained_angle)| {
                angle <= retained_angle + parameters.shot_angle_hysteresis
            })
            .map_or(opening, |(opening, _)| opening)
    })
}

/// Recompute the selected corridor using the current measured ball and obstacles.
/// No action falls back to an unchecked goal-center shot.
pub fn resolve_action_target(
    blackboard: &Blackboard,
    action: BallAction,
    ball: Point2<Field>,
) -> Option<Point2<Field>> {
    let transform = blackboard.world_state.robot.ground_to_field?;
    let field = &blackboard.field_dimensions;
    let parameters = &blackboard.parameters.kicking.strategy;
    parameters.validate().ok()?;
    if !valid_ball(blackboard, ball) {
        return None;
    }
    let openings = match action {
        BallAction::Hold => return None,
        BallAction::Shoot { opening: selected } => {
            if matches!(
                own_restart(blackboard),
                Some(SubState::CornerKick | SubState::ThrowIn | SubState::IndirectFreeKick)
            ) {
                return None;
            }
            let dx = field.length / 2.0 - ball.x();
            if dx <= 0.0 || dx >= parameters.maximum_shot_distance {
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
                    point![opening.1.x(), upper],
                ))
            })
            .filter(|opening| opening_angle(ball, *opening) >= parameters.minimum_shot_angle)
            .collect::<Vec<_>>()
        }
        BallAction::Pass { receiver } => {
            if !blackboard.has_team_message_budget() {
                return None;
            }
            let passing = &parameters.passing;
            pass_openings_with_progress(
                ball,
                pass_receiver_position(blackboard, ball, receiver)?,
                transform,
                &blackboard.world_state.obstacles,
                field,
                passing,
                parameters.obstacle_clearance,
                if allows_nonforward_pass(blackboard) {
                    -passing.maximum_distance
                } else {
                    passing.minimum_progress
                },
            )
        }
        BallAction::Dribble | BallAction::Clear | BallAction::Cutback => travel_openings(
            ball,
            transform,
            &blackboard.world_state.obstacles,
            field,
            if action == BallAction::Clear {
                parameters.maximum_shot_distance
            } else {
                parameters.maximum_dribble_distance
            },
            parameters.obstacle_clearance,
        ),
    };
    if action == BallAction::Dribble || action == BallAction::Cutback {
        let preferred = if action == BallAction::Cutback {
            attacking_reception_position(blackboard, ball)
        } else {
            point![field.length / 2.0, 0.0]
        };
        return openings
            .into_iter()
            .filter_map(|opening| goalward_target(ball, preferred, opening))
            .filter_map(|target| {
                bounded_travel_target(
                    blackboard,
                    ball,
                    target,
                    parameters.maximum_dribble_distance,
                )
            })
            .filter(|target| {
                if action == BallAction::Cutback {
                    // A restart must put the ball back inside both field lines.
                    target.x().abs() < field.length / 2.0 - field.ball_radius
                        && target.y().abs() < field.width / 2.0 - field.ball_radius
                } else {
                    target.x() > ball.x() + parameters.minimum_forward_progress
                }
            })
            .min_by(|a, b| {
                (a - preferred)
                    .norm_squared()
                    .total_cmp(&(b - preferred).norm_squared())
            });
    }
    openings
        .into_iter()
        .filter_map(|opening| {
            let opening = if action == BallAction::Clear {
                forward_opening(ball, opening, parameters.minimum_forward_progress)?
            } else {
                opening
            };
            let target = angular_center(ball, opening)?;
            let angle = opening_angle(ball, opening);
            if action == BallAction::Clear {
                let target = bounded_travel_target(
                    blackboard,
                    ball,
                    target,
                    parameters.maximum_shot_distance,
                )?;
                let progress = target.x() - ball.x();
                (progress > parameters.minimum_forward_progress)
                    .then_some((target, angle * progress))
            } else {
                Some((target, angle))
            }
        })
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(target, _)| target)
}

fn forward_opening(
    ball: Point2<Field>,
    mut opening: LineSegment<Field>,
    minimum_progress: f32,
) -> Option<LineSegment<Field>> {
    let minimum_x = ball.x() + minimum_progress;
    if opening.0.x() < minimum_x && opening.1.x() < minimum_x {
        return None;
    }
    if opening.0.x() < minimum_x || opening.1.x() < minimum_x {
        let fraction = (minimum_x - opening.0.x()) / (opening.1.x() - opening.0.x());
        let intersection = opening.0 + (opening.1 - opening.0) * fraction;
        if opening.0.x() < minimum_x {
            opening.0 = intersection;
        } else {
            opening.1 = intersection;
        }
    }
    (opening.length_squared() > f32::EPSILON).then_some(opening)
}

fn bounded_travel_target(
    blackboard: &Blackboard,
    ball: Point2<Field>,
    target: Point2<Field>,
    maximum_distance: f32,
) -> Option<Point2<Field>> {
    let displacement = target - ball;
    let distance = displacement.norm();
    if !distance.is_finite()
        || distance <= f32::EPSILON
        || !maximum_distance.is_finite()
        || maximum_distance <= 0.0
    {
        return None;
    }
    // Shortening a validated finite ball segment preserves obstacle clearance.
    let target = ball + displacement * (maximum_distance / distance).min(1.0);
    let margin = blackboard.field_dimensions.ball_radius
        + blackboard.parameters.kicking.strategy.obstacle_clearance;
    let half_length = blackboard.field_dimensions.length / 2.0 - margin;
    let half_width = blackboard.field_dimensions.width / 2.0 - margin;
    // A restart ball can begin outside these inner bounds. Check again after
    // shortening rather than assuming every point along its entering ray is in.
    (target
        .inner
        .coords
        .iter()
        .all(|coordinate| coordinate.is_finite())
        && target.x().abs() <= half_length
        && target.y().abs() <= half_width)
        .then_some(target)
}

pub(super) fn pass_receiver_position(
    blackboard: &Blackboard,
    ball: Point2<Field>,
    receiver: PlayerNumber,
) -> Option<Point2<Field>> {
    let world = &blackboard.world_state;
    let parameters = &blackboard.parameters.kicking.strategy.passing;
    if parameters.maximum_distance <= 0.0
        || receiver == world.robot.player_number
        || receiver == blackboard.parameters.goalkeeper.player_number
        || world
            .filtered_game_controller_state
            .as_ref()
            .is_some_and(|state| state.penalties[receiver].is_some())
    {
        return None;
    }
    let state = blackboard.timed_player_states[receiver].as_ref()?;
    if state.inner.unavailable
        || state.time > world.now
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
        && position.x().abs() < field.length / 2.0 - field.ball_radius
        && position.y().abs() < field.width / 2.0 - field.ball_radius
        && (allows_nonforward_pass(blackboard)
            || position.x() - ball.x() >= parameters.minimum_progress))
        .then_some(position)
}

fn own_restart(blackboard: &Blackboard) -> Option<SubState> {
    blackboard
        .world_state
        .filtered_game_controller_state
        .as_ref()
        .and_then(|game| {
            (game.kicking_team == Some(Team::Hulks))
                .then_some(game.sub_state)
                .flatten()
        })
}

fn allows_nonforward_pass(blackboard: &Blackboard) -> bool {
    matches!(
        own_restart(blackboard),
        Some(SubState::CornerKick | SubState::ThrowIn)
    )
}

fn valid_ball(blackboard: &Blackboard, ball: Point2<Field>) -> bool {
    let field = &blackboard.field_dimensions;
    ball.x().is_finite()
        && ball.y().is_finite()
        && field.length > 0.0
        && field.width > 0.0
        && ball.x().abs() <= field.length / 2.0 + field.ball_radius
        && ball.y().abs() <= field.width / 2.0 + field.ball_radius
}

fn attacking_reception_position(blackboard: &Blackboard, ball: Point2<Field>) -> Point2<Field> {
    let field = &blackboard.field_dimensions;
    if ball.x() > 0.0 {
        point![field.length / 2.0 - field.penalty_area_length, 0.0]
    } else {
        point![
            ball.x()
                + blackboard
                    .parameters
                    .kicking
                    .strategy
                    .maximum_dribble_distance,
            0.0
        ]
    }
}

fn opening_angle(ball: Point2<Field>, opening: LineSegment<Field>) -> f32 {
    LineSegment(ball, opening.0).angle(LineSegment(ball, opening.1))
}

fn angular_center(ball: Point2<Field>, opening: LineSegment<Field>) -> Option<Point2<Field>> {
    let direction = (opening.0 - ball).try_normalize(f32::EPSILON)?
        + (opening.1 - ball).try_normalize(f32::EPSILON)?;
    let target = Line2::from(opening).intersection(&Line2 {
        point: ball,
        direction,
    });
    (target.x().is_finite() && target.y().is_finite()).then_some(target)
}

fn goalward_target(
    ball: Point2<Field>,
    preferred: Point2<Field>,
    opening: LineSegment<Field>,
) -> Option<Point2<Field>> {
    // Avoid the tangent boundary; exact grazing trajectories are fragile.
    let inset = (opening.1 - opening.0) * (0.01 / opening.length()).min(0.5);
    let opening = LineSegment(opening.0 + inset, opening.1 - inset);
    let preferred_target = Line2::from(opening).intersection(&Line2::from_points(ball, preferred));
    let target = if preferred_target.x().is_finite()
        && preferred_target.y().is_finite()
        && (0.0..=1.0).contains(&opening.projection_factor(preferred_target))
        && (preferred_target - ball).dot(&(preferred - ball)) > 0.0
    {
        preferred_target
    } else if (opening.0 - preferred).norm_squared() < (opening.1 - preferred).norm_squared() {
        opening.0
    } else {
        opening.1
    };
    (target.x().is_finite() && target.y().is_finite()).then_some(target)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::node::LastBall;
    use hsl_network_messages::{Half, PassIntent, PassReady};
    use linear_algebra::{Isometry2, Pose2, vector};
    use ros_z::time::Time;
    use std::time::Duration;
    use types::{
        field_dimensions::{FieldDimensions, Side},
        filtered_game_controller_state::FilteredGameControllerState,
        obstacles::Obstacle,
        parameters::KickStrategyParameters,
        time_wrapper::TimeWrapper,
        world_state::PlayerState,
    };

    fn blackboard(ball: Point2<Field>) -> Blackboard {
        let mut blackboard = Blackboard::default();
        blackboard.field_dimensions = FieldDimensions::SPL_2025;
        blackboard.world_state.robot.ground_to_field = Some(Default::default());
        blackboard.ball = Some(LastBall {
            position: ball,
            velocity: Default::default(),
            age: Default::default(),
            field_side: Side::Left,
            source: types::world_state::BallSource::Own,
        });
        blackboard.parameters.kicking.strategy = KickStrategyParameters {
            maximum_shot_distance: 5.0,
            maximum_dribble_distance: 1.5,
            obstacle_clearance: 0.05,
            minimum_shot_angle: 0.03,
            minimum_forward_progress: 0.1,
            ..Default::default()
        };
        blackboard
    }

    fn small_and_middle_fields() -> [FieldDimensions; 2] {
        let small = FieldDimensions {
            ball_radius: 0.073,
            goal_inner_width: 1.8,
            ..FieldDimensions::SPL_2025
        };
        [
            small,
            FieldDimensions {
                length: 14.0,
                width: 9.0,
                ball_radius: 0.105,
                goal_inner_width: 2.4,
                penalty_area_length: 3.0,
                ..small
            },
        ]
    }

    fn field_advance_blackboard(field: FieldDimensions, ball: Point2<Field>) -> Blackboard {
        let mut b = blackboard(ball);
        b.field_dimensions = field;
        b.parameters.kicking.strategy = KickStrategyParameters::default();
        b.parameters.kicking.target_speed = 3.4;
        b.parameters.kicking.kick_activation_distance = 1.1;
        b.parameters.kicking.kick_activation_hysteresis = 0.2;
        b.parameters.walking.path_planning.robot_radius = 0.4;
        b.parameters.ball.last_ball_timeout = Duration::from_millis(500);
        b.world_state.robot.ground_to_field = Some(Isometry2::from_parts(
            vector![ball.x() - 0.5, ball.y()],
            0.0,
        ));
        b
    }

    #[test]
    fn a_validated_long_field_lane_uses_the_existing_regular_policy_on_both_fields() {
        use types::motion_command::BodyMotion;

        for field in small_and_middle_fields() {
            let ball = point![-2.0, 0.0];
            let mut b = field_advance_blackboard(field, ball);
            assert!(select_shot_opening(&b, ball).is_none());
            let target = resolve_action_target(&b, BallAction::Clear, ball).unwrap();
            assert!(target.x() - ball.x() > b.parameters.kicking.strategy.maximum_dribble_distance);
            assert_eq!(select_ball_action(&mut b), Status::Success);
            assert_eq!(b.ball_action, Some(BallAction::Clear));
            assert_eq!(
                super::super::execution::execute_ball_action(&mut b),
                Status::Success
            );
            assert!(matches!(
                b.body_motion,
                Some(BodyMotion::Kick { soft: false, strong: false, target_speed, .. })
                    if target_speed == b.parameters.kicking.target_speed
            ));
        }
    }

    #[test]
    fn long_field_lanes_preserve_immediate_shot_and_addressed_pass_priorities() {
        for field in small_and_middle_fields() {
            let ball = point![field.length / 2.0 - 3.0, 0.0];
            let b = field_advance_blackboard(field, ball);
            let target = resolve_action_target(&b, BallAction::Clear, ball).unwrap();
            assert!(target.x() - ball.x() > b.parameters.kicking.strategy.maximum_dribble_distance);
            assert!(matches!(
                choose_ball_action(&b),
                Some(BallAction::Shoot { .. })
            ));
        }
        let mut b = own_half_pressure_blackboard();
        acknowledge_pass(&mut b);
        assert_eq!(
            choose_ball_action(&b),
            Some(BallAction::Pass {
                receiver: PlayerNumber::Two
            })
        );
    }

    #[test]
    fn an_attacking_half_challenge_keeps_soft_control_unless_already_clearing() {
        let [small, middle] = small_and_middle_fields();
        // Include the smoke harness's SPL ball, then physical Foundation balls.
        // The angled challenge keeps a long forward exit on both larger balls.
        for (field, lateral) in [
            (FieldDimensions::SPL_2025, -0.1),
            (small, -0.3),
            (middle, -0.3),
        ] {
            let ball = point![1.0, 0.0];
            let mut b = field_advance_blackboard(field, ball);
            let transform = b.world_state.robot.ground_to_field.unwrap();
            let obstacle = Obstacle::robot(transform.inverse() * point![2.0, lateral], 0.3, 0.5);
            let gap = (obstacle.position - transform.inverse() * ball).norm()
                - obstacle.radius_at_foot_height;
            assert!(gap < b.parameters.kicking.strategy.pressure_distance);
            b.world_state.obstacles = vec![obstacle];
            assert!(select_shot_opening(&b, ball).is_none());
            let target = resolve_action_target(&b, BallAction::Clear, ball).unwrap();
            assert!(target.x() - ball.x() > b.parameters.kicking.strategy.maximum_dribble_distance);
            assert!(resolve_action_target(&b, BallAction::Dribble, ball).is_some());
            assert_eq!(choose_ball_action(&b), Some(BallAction::Dribble));

            b.ball_action = Some(BallAction::Clear);
            assert_eq!(choose_ball_action(&b), Some(BallAction::Clear));
        }
    }

    #[test]
    fn short_forward_space_keeps_soft_dribbling_and_a_fully_blocked_ball_holds() {
        use types::motion_command::BodyMotion;

        let field = small_and_middle_fields()[0];
        let ball = point![3.8, 2.6];
        let mut b = field_advance_blackboard(field, ball);
        let transform = b.world_state.robot.ground_to_field.unwrap();
        b.world_state.obstacles = vec![Obstacle::robot(
            transform.inverse() * point![4.25, 0.5],
            0.2,
            0.25,
        )];
        assert!(select_shot_opening(&b, ball).is_none());
        let short_clear = resolve_action_target(&b, BallAction::Clear, ball).unwrap();
        assert!(
            (short_clear - ball).norm() <= b.parameters.kicking.strategy.maximum_dribble_distance
        );
        assert_eq!(select_ball_action(&mut b), Status::Success);
        assert_eq!(b.ball_action, Some(BallAction::Dribble));
        assert_eq!(
            super::super::execution::execute_ball_action(&mut b),
            Status::Success
        );
        assert!(matches!(
            b.body_motion,
            Some(BodyMotion::Kick { soft: true, target_speed, .. })
                if target_speed == b.parameters.kicking.strategy.dribble_target_speed
        ));

        let ball = point![-2.0, 0.0];
        let mut b = field_advance_blackboard(field, ball);
        b.world_state.obstacles = vec![Obstacle::robot(
            b.world_state.robot.ground_to_field.unwrap().inverse() * ball,
            0.2,
            0.25,
        )];
        assert!(resolve_action_target(&b, BallAction::Clear, ball).is_none());
        assert_eq!(choose_ball_action(&b), Some(BallAction::Hold));
    }

    #[test]
    fn a_valid_long_lateral_exit_with_short_forward_gain_keeps_soft_dribbling() {
        use types::motion_command::BodyMotion;

        let ball = point![3.8, 2.6];
        let mut b = field_advance_blackboard(small_and_middle_fields()[0], ball);
        // Isolate the forward-gain requirement from the pressure gate.
        b.parameters.kicking.strategy.pressure_distance = 0.0;
        let transform = b.world_state.robot.ground_to_field.unwrap();
        b.world_state.obstacles = [point![4.3, 2.6], point![4.3, 2.0]]
            .into_iter()
            .map(|position| Obstacle::robot(transform.inverse() * position, 0.2, 0.25))
            .collect();
        assert!(select_shot_opening(&b, ball).is_none());
        let clear = resolve_action_target(&b, BallAction::Clear, ball)
            .expect("the long lateral exit remains valid for an emergency clearance");
        let dribble_range = b.parameters.kicking.strategy.maximum_dribble_distance;
        assert!((clear - ball).norm() > dribble_range, "{clear:?}");
        assert!(clear.x() - ball.x() <= dribble_range);
        assert_eq!(select_ball_action(&mut b), Status::Success);
        assert_eq!(b.ball_action, Some(BallAction::Dribble));
        assert_eq!(
            super::super::execution::execute_ball_action(&mut b),
            Status::Success
        );
        assert!(matches!(
            b.body_motion,
            Some(BodyMotion::Kick { soft: true, target_speed, .. })
                if target_speed == b.parameters.kicking.strategy.dribble_target_speed
        ));
    }

    #[test]
    fn long_lane_availability_does_not_replace_inward_restart_actions() {
        for field in small_and_middle_fields() {
            for (restart, ball) in [
                (
                    SubState::CornerKick,
                    point![field.length / 2.0, field.width / 2.0],
                ),
                (SubState::ThrowIn, point![-2.0, field.width / 2.0]),
                (SubState::IndirectFreeKick, point![-2.0, 0.0]),
            ] {
                let mut b = field_advance_blackboard(field, ball);
                b.world_state.filtered_game_controller_state = Some(FilteredGameControllerState {
                    kicking_team: Some(Team::Hulks),
                    sub_state: Some(restart),
                    ..Default::default()
                });
                assert_eq!(choose_ball_action(&b), Some(BallAction::Cutback));
            }
        }
    }

    #[test]
    fn maximum_travel_ranges_keep_central_and_corner_targets_inside_both_fields() {
        for field in small_and_middle_fields() {
            for (ball, actions) in [
                (
                    Point2::origin(),
                    &[BallAction::Clear, BallAction::Dribble, BallAction::Cutback][..],
                ),
                (
                    point![-field.length / 4.0, field.width / 4.0],
                    &[BallAction::Clear, BallAction::Dribble][..],
                ),
                (
                    point![field.length / 2.0, field.width / 2.0],
                    &[BallAction::Cutback][..],
                ),
                (
                    point![field.length / 2.0, -field.width / 2.0],
                    &[BallAction::Cutback][..],
                ),
            ] {
                let mut b = blackboard(ball);
                b.field_dimensions = field;
                b.parameters.kicking.strategy.maximum_shot_distance = 6.0;
                b.parameters.kicking.strategy.maximum_dribble_distance = 2.0;
                for &action in actions {
                    let target = resolve_action_target(&b, action, ball).unwrap_or_else(|| {
                        panic!("an unobstructed {action:?} needs an in-field target at {ball:?}")
                    });
                    let maximum = if action == BallAction::Clear {
                        6.0
                    } else {
                        2.0
                    };
                    assert!((target - ball).norm() <= maximum + 1e-6);
                    let margin =
                        field.ball_radius + b.parameters.kicking.strategy.obstacle_clearance;
                    assert!(target.x().abs() <= field.length / 2.0 - margin);
                    assert!(target.y().abs() <= field.width / 2.0 - margin);
                    if action == BallAction::Clear || action == BallAction::Dribble {
                        assert!(
                            target.x() - ball.x()
                                > b.parameters.kicking.strategy.minimum_forward_progress
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn clamping_a_restart_ray_must_not_leave_its_target_outside_inner_field_bounds() {
        let b = blackboard(point![4.5, 3.0]);
        assert!(bounded_travel_target(&b, point![4.5, 3.0], point![3.5, 2.0], 0.1).is_none());
        assert!(bounded_travel_target(&b, Point2::origin(), point![f32::NAN, 0.0], 2.0).is_none());
    }

    #[test]
    fn shortening_selected_travel_rays_preserves_robot_collision_clearance() {
        for field in small_and_middle_fields() {
            let mut b = blackboard(Point2::origin());
            b.field_dimensions = field;
            b.parameters.kicking.strategy.maximum_shot_distance = 6.0;
            b.parameters.kicking.strategy.maximum_dribble_distance = 2.0;
            b.world_state.obstacles = vec![Obstacle::robot(point![1.0, 0.0], 0.2, 0.25)];
            for action in [BallAction::Clear, BallAction::Dribble, BallAction::Cutback] {
                let target = resolve_action_target(&b, action, Point2::origin()).unwrap();
                let radius =
                    0.2 + field.ball_radius + b.parameters.kicking.strategy.obstacle_clearance;
                assert!(
                    LineSegment(Point2::origin(), target).distance_to(point![1.0, 0.0])
                        >= radius - 1e-5
                );
            }
            b.world_state.obstacles = vec![Obstacle::robot(Point2::origin(), 0.2, 0.25)];
            assert!(resolve_action_target(&b, BallAction::Clear, Point2::origin()).is_none());
            assert!(resolve_action_target(&b, BallAction::Dribble, Point2::origin()).is_none());
        }
    }

    #[test]
    fn clear_width_and_progress_prefer_a_forward_exit_over_a_nearly_lateral_one() {
        let ball = point![-3.0, 3.0];
        let mut b = blackboard(ball);
        b.field_dimensions = small_and_middle_fields()[1];
        b.parameters.kicking.strategy.maximum_shot_distance = 6.0;
        let target = resolve_action_target(&b, BallAction::Clear, ball).unwrap();
        // The right edge at x=3 is available. Width alone instead selected the
        // bottom edge near [-2.34,-3], gaining less than a metre forward.
        assert!(
            target.x() > 2.0,
            "prefer the usable forward corridor: {target:?}"
        );
        assert!((target - ball).norm() <= 6.0 + 1e-6);
    }

    #[test]
    fn clear_keeps_the_forward_part_of_a_side_region_when_the_front_is_blocked() {
        let mut b = blackboard(Point2::origin());
        b.field_dimensions = small_and_middle_fields()[0];
        b.parameters.kicking.strategy.maximum_shot_distance = 6.0;
        b.world_state.obstacles = [-0.65, 0.0, 0.65]
            .into_iter()
            .map(|y| Obstacle::robot(point![1.2, y], 0.2, 0.25))
            .collect();
        let target = resolve_action_target(&b, BallAction::Clear, Point2::origin())
            .expect("a visible forward portion of the side edge is still usable");
        assert!(target.x() > b.parameters.kicking.strategy.minimum_forward_progress);
        for obstacle in &b.world_state.obstacles {
            assert!(
                LineSegment(Point2::origin(), target)
                    .distance_to(point![obstacle.position.x(), obstacle.position.y()])
                    >= obstacle.radius_at_foot_height
                        + b.field_dimensions.ball_radius
                        + b.parameters.kicking.strategy.obstacle_clearance
                        - 1e-5
            );
        }
    }

    #[test]
    fn corner_on_field_boundary_selects_an_inward_cutback() {
        for y in [-3.0, 3.0] {
            let mut b = blackboard(point![4.5, y]);
            b.world_state.filtered_game_controller_state = Some(
                types::filtered_game_controller_state::FilteredGameControllerState {
                    sub_state: Some(SubState::CornerKick),
                    kicking_team: Some(Team::Hulks),
                    ..Default::default()
                },
            );
            assert_eq!(choose_ball_action(&b), Some(BallAction::Cutback));
            let target = resolve_action_target(&b, BallAction::Cutback, point![4.5, y]).unwrap();
            assert!(target.x() < 4.4 && target.y().abs() < 2.9);
            assert!(
                resolve_action_target(
                    &b,
                    BallAction::Shoot {
                        opening: LineSegment(point![4.5, -0.5], point![4.5, 0.5])
                    },
                    point![4.5, y]
                )
                .is_none()
            );
        }
    }

    #[test]
    fn newly_blocked_shot_has_no_unchecked_center_fallback() {
        let mut b = blackboard(point![2.0, 0.0]);
        let BallAction::Shoot { opening } = choose_ball_action(&b).unwrap() else {
            panic!("expected an open shot")
        };
        b.world_state.obstacles = vec![types::obstacles::Obstacle::robot(
            point![2.5, 0.0],
            0.7,
            0.7,
        )];
        assert!(
            resolve_action_target(&b, BallAction::Shoot { opening }, point![2.0, 0.0]).is_none()
        );
        assert!(!matches!(
            choose_ball_action(&b),
            Some(BallAction::Shoot { .. })
        ));
    }

    #[test]
    fn indirect_free_kick_does_not_select_a_direct_shot() {
        let mut b = blackboard(point![2.5, 0.5]);
        b.world_state.filtered_game_controller_state = Some(
            types::filtered_game_controller_state::FilteredGameControllerState {
                sub_state: Some(SubState::IndirectFreeKick),
                kicking_team: Some(Team::Hulks),
                ..Default::default()
            },
        );
        assert!(matches!(choose_ball_action(&b), Some(BallAction::Cutback)));
    }

    fn passing_blackboard(pressured: bool) -> Blackboard {
        let ball = point![0.22, 0.0];
        let mut b = blackboard(ball);
        b.world_state.now = Time::from_nanos(10_000_000_000);
        b.ball.as_mut().unwrap().age = b.world_state.now;
        b.world_state.robot.player_number = PlayerNumber::Three;
        b.parameters.goalkeeper.player_number = PlayerNumber::One;
        // Keep the goal beyond shooting range so the available pass competes
        // with an executable forward dribble rather than an immediate shot.
        b.parameters.kicking.strategy.maximum_shot_distance = 2.0;
        b.parameters.kicking.strategy.maximum_dribble_distance = 2.0;
        b.world_state.filtered_game_controller_state = Some(FilteredGameControllerState {
            half: Half::Second,
            remaining_time_in_half: Duration::from_secs(100),
            remaining_number_of_messages: 1000,
            ..Default::default()
        });
        b.timed_player_states.two = Some(TimeWrapper {
            time: b.world_state.now,
            inner: PlayerState {
                pose: Pose2::new(point![2.0, 0.0], std::f32::consts::PI),
                ..Default::default()
            },
        });
        if pressured {
            // Match the 0.733 m foot gap from the Small 113 s loss: this
            // opponent applies pressure while leaving the outgoing lane open.
            b.world_state.obstacles = vec![Obstacle::robot(point![ball.x(), 0.933], 0.2, 0.25)];
        }
        b
    }

    fn acknowledge_pass(b: &mut Blackboard) -> Point2<Field> {
        let action = BallAction::Pass {
            receiver: PlayerNumber::Two,
        };
        let target = resolve_action_target(b, action, b.ball.as_ref().unwrap().position).unwrap();
        b.ball_action = Some(action);
        b.pass_started_at = Some(b.world_state.now - Duration::from_millis(200));
        b.last_announced_pass = Some(TimeWrapper {
            time: b.world_state.now - Duration::from_millis(200),
            inner: PassIntent {
                receiver: PlayerNumber::Two,
                target,
            },
        });
        b.last_sent_hsl_message_time = Some(b.world_state.now - Duration::from_millis(100));
        let receiver = b.timed_player_states.two.as_mut().unwrap();
        receiver.time = b.world_state.now - Duration::from_millis(100);
        receiver.inner.pass_ready = Some(PassReady {
            sender: PlayerNumber::Three,
            target,
        });
        target
    }

    #[test]
    fn an_unavailable_receiver_invalidates_even_an_acknowledged_committed_pass() {
        let mut b = passing_blackboard(false);
        let target = acknowledge_pass(&mut b);
        let action = BallAction::Pass {
            receiver: PlayerNumber::Two,
        };
        assert_eq!(choose_ball_action(&b), Some(action));
        b.last_motion_type = Some(types::motion_type::MotionType::Kick);
        b.last_motion_switch_time = b.world_state.now;
        b.parameters.control.allow_switch.kick = Duration::from_millis(500);
        b.timed_player_states
            .two
            .as_mut()
            .unwrap()
            .inner
            .unavailable = true;
        let ball = b.ball.as_ref().unwrap().position;
        assert!(pass_receiver_position(&b, ball, PlayerNumber::Two).is_none());
        assert!(resolve_action_target(&b, action, ball).is_none());
        assert_eq!(select_ball_action(&mut b), Status::Success);
        assert_ne!(b.ball_action, Some(action));
        assert!(b.last_announced_pass.is_none());
        assert!(b.pass_started_at.is_none());

        // A newer ordinary state restores eligibility; availability is not a
        // permanent veto tied to the receiver's player number.
        b.world_state.now = b.world_state.now + Duration::from_millis(20);
        b.ball.as_mut().unwrap().age = b.world_state.now;
        let receiver = b.timed_player_states.two.as_mut().unwrap();
        receiver.time = b.world_state.now;
        receiver.inner.unavailable = false;
        receiver.inner.pass_ready = None;
        assert_eq!(choose_ball_action(&b), Some(action));
        assert_eq!(resolve_action_target(&b, action, ball), Some(target));
    }

    fn own_half_pressure_blackboard() -> Blackboard {
        let mut b = passing_blackboard(true);
        b.field_dimensions = small_and_middle_fields()[0];
        b.ball.as_mut().unwrap().position = point![-0.5, 0.0];
        b.parameters.kicking.strategy.maximum_shot_distance = 6.0;
        b.world_state.obstacles = vec![
            Obstacle::robot(point![-0.5, 0.933], 0.2, 0.25),
            Obstacle::robot(point![2.0, 0.0], 0.7, 0.7),
        ];
        b.timed_player_states.two.as_mut().unwrap().inner.pose = Pose2::new(point![1.5, 1.5], 0.0);
        b
    }

    #[test]
    fn own_half_pressure_uses_the_existing_regular_clearance_after_shot_and_ready_pass_choices() {
        use types::motion_command::BodyMotion;

        let mut b = own_half_pressure_blackboard();
        assert!(select_shot_opening(&b, b.ball.as_ref().unwrap().position).is_none());
        assert_eq!(select_ball_action(&mut b), Status::Success);
        assert_eq!(b.ball_action, Some(BallAction::Clear));
        b.parameters.kicking.kick_activation_distance = 1.1;
        b.parameters.kicking.kick_activation_hysteresis = 0.2;
        b.parameters.kicking.target_speed = 3.4;
        assert_eq!(
            super::super::execution::execute_ball_action(&mut b),
            Status::Success
        );
        assert!(
            matches!(b.body_motion, Some(BodyMotion::Kick { soft: false, target_speed, .. }) if target_speed == b.parameters.kicking.target_speed)
        );

        let mut b = own_half_pressure_blackboard();
        acknowledge_pass(&mut b);
        assert_eq!(
            choose_ball_action(&b),
            Some(BallAction::Pass {
                receiver: PlayerNumber::Two
            })
        );
        b.world_state.obstacles.pop();
        assert!(matches!(
            choose_ball_action(&b),
            Some(BallAction::Shoot { .. })
        ));
    }

    #[test]
    fn own_half_pressure_release_preserves_unpressured_short_space_and_inward_restarts() {
        let mut b = own_half_pressure_blackboard();
        b.world_state.obstacles.remove(0);
        assert_eq!(
            choose_ball_action(&b),
            Some(BallAction::Pass {
                receiver: PlayerNumber::Two
            })
        );
        b.timed_player_states.two = None;
        let ball = b.ball.as_ref().unwrap().position;
        let target = resolve_action_target(&b, BallAction::Clear, ball).unwrap();
        // The blocker leaves short forward progress even along a longer side exit.
        assert!(target.x() - ball.x() <= b.parameters.kicking.strategy.maximum_dribble_distance);
        assert_eq!(choose_ball_action(&b), Some(BallAction::Dribble));

        for restart in [
            SubState::CornerKick,
            SubState::ThrowIn,
            SubState::IndirectFreeKick,
        ] {
            let mut b = own_half_pressure_blackboard();
            let game = b
                .world_state
                .filtered_game_controller_state
                .as_mut()
                .unwrap();
            game.kicking_team = Some(Team::Hulks);
            game.sub_state = Some(restart);
            assert_eq!(choose_ball_action(&b), Some(BallAction::Cutback));
        }
    }

    #[test]
    fn an_unpressured_striker_can_prepare_a_new_pass() {
        let mut b = passing_blackboard(false);
        assert!(b.has_team_message_budget());
        assert_eq!(select_ball_action(&mut b), Status::Success);
        assert_eq!(
            b.ball_action,
            Some(BallAction::Pass {
                receiver: PlayerNumber::Two
            })
        );
        assert!(b.pass_started_at.is_some());
        assert!(b.last_announced_pass.is_none());
    }

    #[test]
    fn a_pressured_striker_uses_an_immediate_action_instead_of_preparing_a_receiver() {
        let mut b = passing_blackboard(true);
        let pass = BallAction::Pass {
            receiver: PlayerNumber::Two,
        };
        let ball = b.ball.as_ref().unwrap().position;
        assert!(resolve_action_target(&b, pass, ball).is_some());
        assert_eq!(select_ball_action(&mut b), Status::Success);
        assert_eq!(b.ball_action, Some(BallAction::Dribble));
        assert!(resolve_action_target(&b, BallAction::Dribble, ball).is_some());
        assert!(b.pass_started_at.is_none());
        assert!(b.last_announced_pass.is_none());
    }

    #[test]
    fn a_freshly_acknowledged_pass_remains_available_under_pressure() {
        let mut b = passing_blackboard(true);
        let target = acknowledge_pass(&mut b);
        let announcement = b.last_announced_pass.as_ref().unwrap().time;
        assert_eq!(select_ball_action(&mut b), Status::Success);
        assert_eq!(
            b.ball_action,
            Some(BallAction::Pass {
                receiver: PlayerNumber::Two
            })
        );
        assert_eq!(b.last_announced_pass.as_ref().unwrap().time, announcement);
        assert!(super::super::passing::pass_is_ready(
            &mut b,
            PlayerNumber::Two,
            target
        ));
    }

    #[test]
    fn pressure_does_not_retain_a_pass_with_a_stale_wrong_or_unannounced_acknowledgement() {
        for invalidation in 0..6 {
            let mut b = passing_blackboard(true);
            acknowledge_pass(&mut b);
            match invalidation {
                0 => {
                    b.timed_player_states.two.as_mut().unwrap().time = b.world_state.now
                        - b.parameters.kicking.strategy.passing.maximum_ready_age
                        - Duration::from_millis(1);
                }
                1 => {
                    b.timed_player_states
                        .two
                        .as_mut()
                        .unwrap()
                        .inner
                        .pass_ready
                        .as_mut()
                        .unwrap()
                        .sender = PlayerNumber::Four;
                }
                2 => {
                    b.timed_player_states
                        .two
                        .as_mut()
                        .unwrap()
                        .inner
                        .pass_ready
                        .as_mut()
                        .unwrap()
                        .target = point![2.0, 1.0];
                }
                3 => {
                    b.timed_player_states.two.as_mut().unwrap().time =
                        b.last_announced_pass.as_ref().unwrap().time;
                }
                4 => b.last_announced_pass = None,
                _ => b.timed_player_states.two.as_mut().unwrap().inner.pass_ready = None,
            }
            assert_eq!(select_ball_action(&mut b), Status::Success);
            assert_eq!(
                b.ball_action,
                Some(BallAction::Dribble),
                "invalid acknowledgement case {invalidation} must not cost possession time"
            );
            assert!(b.last_announced_pass.is_none());
            assert!(b.pass_started_at.is_none());
        }
    }
}
