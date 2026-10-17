use std::time::Duration;

use coordinate_systems::Field;
use hsl_network_messages::{PassIntent, PlayerNumber};
use linear_algebra::{Orientation2, Point2, Vector2};
use types::{behavior_tree::Status, motion_command::BodyMotion};

use crate::{actions::stand, node::Blackboard};

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
            (ball.position - pass.target).norm()
                <= parameters.receiver_standoff + parameters.target_half_width
                && (ball.position - own_position).norm() < (ball.position - sender).norm()
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
    let transform = blackboard.world_state.robot.ground_to_field.unwrap();
    let ball = blackboard
        .ball
        .as_ref()
        .map_or(sender, |ball| ball.position);
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
    // Hold the advertised receiver position rather than continuing supporter movement.
    blackboard.body_motion = Some(BodyMotion::WalkWithVelocity {
        velocity: Vector2::zeros(),
        angular_velocity: blackboard.walking_velocity_limits.scale_angular_axis(angle),
    });
    Status::Success
}
