use std::time::Duration;

use coordinate_systems::Field;
use geometry::line_segment::LineSegment;
use hsl_network_messages::{PassIntent, PlayerNumber, SubState};
use linear_algebra::{Orientation2, Point2, Vector2, vector};
use path_planner::path_planner::PathPlanner;
use types::{
    behavior_tree::Status, motion_command::BodyMotion, motion_type::MotionType, obstacles::Obstacle,
};

use crate::{
    actions::stand,
    conditions::{goalkeeper_clearance_direction, hulks_is_kicking_team},
    kick::reception::receive_rolling_ball,
    node::Blackboard,
    substates::{is_in_sub_state, is_sub_state},
};

// shortcut: readiness is unconfirmed; tune this delay after measuring delivery and turning.
const PREPARATION_TIME: Duration = Duration::from_millis(150);

pub(crate) fn matches_announced_pass(blackboard: &Blackboard, pass: PassIntent) -> bool {
    let now = blackboard.world_state.now;
    let parameters = &blackboard.parameters.kicking.strategy.passing;
    blackboard
        .last_announced_pass
        .as_ref()
        .is_some_and(|announced| {
            announced.time <= now
                && announced.inner.receiver == pass.receiver
                && (announced.inner.target - pass.target).norm() <= parameters.target_half_width
        })
        && blackboard
            .last_sent_hsl_message_time
            .is_some_and(|sent| sent <= now && now.duration_since(sent) < PassIntent::TIMEOUT)
}

pub(super) fn pass_is_announced(
    blackboard: &Blackboard,
    receiver: PlayerNumber,
    target: Point2<Field>,
) -> bool {
    matches_announced_pass(blackboard, PassIntent { receiver, target })
        && blackboard.last_announced_pass.as_ref().is_some_and(|pass| {
            blackboard.world_state.now.duration_since(pass.time) >= PREPARATION_TIME
        })
}

pub(super) fn is_executing_announced_pass(blackboard: &Blackboard, receiver: PlayerNumber) -> bool {
    blackboard.last_motion_type == Some(MotionType::Kick)
        && blackboard.ball_action == Some(super::strategy::BallAction::Pass { receiver })
        && blackboard.last_announced_pass.as_ref().is_some_and(|pass| {
            pass.inner.receiver == receiver && matches_announced_pass(blackboard, pass.inner)
        })
}

fn incoming_pass(blackboard: &Blackboard) -> Option<(PassIntent, Point2<Field>)> {
    let world = &blackboard.world_state;
    let transform = world.robot.ground_to_field?;
    let own_position = transform.as_pose().position();
    let parameters = &blackboard.parameters.kicking.strategy.passing;
    parameters.validate().ok()?;
    if parameters.maximum_distance <= 0.0
        || !own_position.x().is_finite()
        || !own_position.y().is_finite()
    {
        return None;
    }
    world
        .player_states
        .iter()
        .filter_map(|(sender, state)| {
            let state = state.as_ref()?;
            let pass = state.inner.pass?;
            if sender == world.robot.player_number
                || pass.receiver != world.robot.player_number
                || state.time > world.now
                || world.now.duration_since(state.time) >= PassIntent::TIMEOUT
                || world
                    .filtered_game_controller_state
                    .as_ref()
                    .is_some_and(|game| game.penalties[sender].is_some())
                || !pass.target.x().is_finite()
                || !pass.target.y().is_finite()
                || pass.target.x().abs() >= blackboard.field_dimensions.length / 2.0
                || pass.target.y().abs() >= blackboard.field_dimensions.width / 2.0
                || (pass.target - own_position).norm()
                    > parameters.receiver_standoff + 2.0 * parameters.target_half_width
                || !state.inner.pose.position().x().is_finite()
                || !state.inner.pose.position().y().is_finite()
            {
                return None;
            }
            Some((state.time, pass, state.inner.pose.position()))
        })
        .max_by_key(|(time, ..)| *time)
        .map(|(_, pass, sender)| (pass, sender))
}

pub fn is_pass_receiver(blackboard: &mut Blackboard) -> bool {
    incoming_pass(blackboard).is_some()
}

pub fn is_pass_ball_arriving(blackboard: &mut Blackboard) -> bool {
    let parameters = &blackboard.parameters.kicking.strategy.passing;
    let arriving = incoming_pass(blackboard).is_some_and(|(pass, sender)| {
        let own_position = blackboard
            .world_state
            .robot
            .ground_to_field
            .unwrap()
            .as_pose()
            .position();
        blackboard.ball.as_ref().is_some_and(|ball| {
            let projected = blackboard.projected_ball_position(ball.position, ball.velocity);
            (projected - pass.target).norm()
                <= parameters.receiver_standoff + parameters.target_half_width
                && (projected - own_position).norm() < (projected - sender).norm()
        })
    });
    if arriving {
        // The addressed receiver claims possession without the normal entry delay.
        blackboard.last_closest_to_ball = true;
        blackboard.closest_to_ball_entered_area_since = None;
        blackboard.closest_to_ball_left_area_since = None;
    }
    arriving
}

pub fn prepare_to_receive_pass(blackboard: &mut Blackboard) -> Status {
    let Some((_, sender)) = incoming_pass(blackboard) else {
        return Status::Failure;
    };
    if receive_rolling_ball(blackboard) == Status::Success {
        return Status::Success;
    }
    let ball = blackboard.ball.as_ref().map_or(sender, |ball| {
        blackboard.projected_ball_position(ball.position, ball.velocity)
    });
    hold_and_face_ball(blackboard, ball)
}

pub fn prepare_for_restart_pass(blackboard: &mut Blackboard) -> Status {
    if !hulks_is_kicking_team(blackboard)
        || !is_in_sub_state(blackboard)
        || is_sub_state(blackboard, SubState::PenaltyKick)
    {
        return Status::Failure;
    }
    let (Some(ball), Some(transform)) = (
        blackboard.ball.as_ref(),
        blackboard.world_state.robot.ground_to_field,
    ) else {
        return Status::Failure;
    };
    let ball = ball.position;
    let distance = (ball - transform.as_pose().position()).norm();
    let parameters = &blackboard.parameters.kicking.strategy.passing;
    if parameters.validate().is_err()
        || parameters.maximum_distance <= 0.0
        || !distance.is_finite()
        || distance >= parameters.maximum_distance + parameters.receiver_standoff
    {
        return Status::Failure;
    }
    blackboard.ball_action = None;
    blackboard.kick_target = None;
    hold_and_face_ball(blackboard, ball)
}

pub fn yield_goalkeeper_clearance(blackboard: &mut Blackboard) -> Status {
    let transform = blackboard.world_state.robot.ground_to_field.unwrap();
    let ball = blackboard.ball.as_ref().unwrap();
    let position = ball.position;
    let Some(direction) = goalkeeper_clearance_direction(blackboard) else {
        return Status::Failure;
    };
    let lateral = vector![-direction.y(), direction.x()];
    let own_position = transform.as_pose().position();
    let offset = (own_position - position).dot(&lateral);
    let radius = blackboard.parameters.walking.path_planning.robot_radius;
    let clearance = radius
        + blackboard.field_dimensions.ball_radius
        + blackboard.parameters.kicking.strategy.obstacle_clearance;
    blackboard.ball_action = None;
    blackboard.kick_target = None;
    blackboard.last_closest_to_ball = false;
    blackboard.closest_to_ball_entered_area_since = None;
    blackboard.closest_to_ball_left_area_since = None;
    if offset.abs() >= clearance {
        return hold_and_face_ball(blackboard, position);
    }

    let mut planner = PathPlanner::default();
    planner.with_obstacles(&blackboard.world_state.obstacles, radius);
    planner.with_rule_obstacles(
        transform.inverse(),
        &blackboard.world_state.rule_obstacles,
        radius,
    );
    planner.with_ball(
        transform.inverse() * position,
        blackboard.field_dimensions.ball_radius,
        radius,
    );
    planner.with_goal_support_structures(transform.inverse(), &blackboard.field_dimensions);
    for (number, teammate) in blackboard.world_state.player_states.iter() {
        if number != blackboard.world_state.robot.player_number
            && let Some(teammate) = teammate
            && teammate.time <= blackboard.world_state.now
            && blackboard.world_state.now.duration_since(teammate.time)
                <= blackboard
                    .parameters
                    .kicking
                    .strategy
                    .passing
                    .maximum_receiver_age
        {
            planner.with_obstacles(
                &[Obstacle::robot(
                    transform.inverse() * teammate.inner.pose.position(),
                    radius,
                    radius,
                )],
                radius,
            );
        }
    }
    let preferred_side = if offset < 0.0 { -1.0 } else { 1.0 };
    for side in [preferred_side, -preferred_side] {
        // Never cross the rolling ball's line to reach the other side.
        if offset * side < -f32::EPSILON {
            continue;
        }
        let target = own_position + lateral * (side * (clearance + radius) - offset);
        let target = transform.inverse() * target;
        let step = LineSegment(Point2::origin(), target);
        let field_target = transform * target;
        if field_target.x().abs() >= blackboard.field_dimensions.length / 2.0 - radius
            || field_target.y().abs() >= blackboard.field_dimensions.width / 2.0 - radius
            || planner
                .obstacles
                .iter()
                .any(|obstacle| obstacle.shape.intersects_line_segment(step))
        {
            continue;
        }
        let ball = transform.inverse() * position;
        let limits = blackboard.walking_velocity_limits;
        let velocity = target.coords().normalize() * blackboard.parameters.walking.speed.kicking;
        let forward_limit = if velocity.x() < 0.0 {
            -limits.forward_velocity_limits[0]
        } else {
            limits.forward_velocity_limits[1]
        };
        // Scale uniformly to stay on the obstacle-checked segment at any body orientation.
        let scale = (velocity.x().abs() / forward_limit.max(f32::EPSILON))
            .max(velocity.y().abs() / limits.lateral_velocity_limit.max(f32::EPSILON))
            .max(1.0);
        blackboard.path_obstacles_output = planner.obstacles;
        blackboard.body_motion = Some(BodyMotion::WalkWithVelocity {
            velocity: velocity / scale,
            angular_velocity: limits
                .scale_angular_axis(Orientation2::from_vector(ball.coords()).angle()),
        });
        return Status::Success;
    }
    hold_and_face_ball(blackboard, position)
}

fn hold_and_face_ball(blackboard: &mut Blackboard, ball: Point2<Field>) -> Status {
    let transform = blackboard.world_state.robot.ground_to_field.unwrap();
    let direction = transform.inverse() * ball;
    let Some(direction) = direction.coords().try_normalize(f32::EPSILON) else {
        return stand(blackboard);
    };
    let angle = Orientation2::from_vector(direction).angle();
    if angle.abs()
        <= blackboard
            .parameters
            .walking
            .walk_and_stand
            .orientation_tolerance
    {
        return stand(blackboard);
    }
    // Keep the reception position while turning toward the ball.
    blackboard.body_motion = Some(BodyMotion::WalkWithVelocity {
        velocity: Vector2::zeros(),
        angular_velocity: blackboard.walking_velocity_limits.scale_angular_axis(angle),
    });
    Status::Success
}
