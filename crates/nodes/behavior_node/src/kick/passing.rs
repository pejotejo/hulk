use coordinate_systems::Field;
use hsl_network_messages::{PassIntent, PassReady, PlayerNumber};
use linear_algebra::{Orientation2, Point2, Vector2};
use ros_z::{Message, time::Time};
use serde::{Deserialize, Serialize};
use types::{
    behavior_tree::Status,
    fall_detection::Posture,
    motion_command::{BodyMotion, HeadMotion, ImageRegion},
};

use crate::{actions::stand, node::Blackboard};

use super::strategy::BallAction;

/// An addressed pass whose launch was observed locally. Its lifetime is tied to
/// that launch, so repeated state messages cannot hold a receiver indefinitely.
#[derive(Clone, Copy, Debug, Deserialize, Serialize, Message)]
pub struct ReceivingPass {
    pub sender: PlayerNumber,
    pub target: Point2<Field>,
    pub sender_position: Point2<Field>,
    pub launched_at: Time,
    /// Suppress this same offer after completion or a failed reception until
    /// the sender withdraws or changes it. A marker never reserves a receiver.
    #[serde(default)]
    pub released_at: Option<Time>,
}

/// True only while the same intent is carried in normal, budgeted state messages.
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

/// Wait for a fresh addressed acknowledgement; a local announcement alone is
/// insufficient. A bounded attempt prevents a lost peer from freezing a striker.
pub fn pass_is_ready(
    blackboard: &mut Blackboard,
    receiver: PlayerNumber,
    target: Point2<Field>,
) -> bool {
    let now = blackboard.world_state.now;
    let started = *blackboard.pass_started_at.get_or_insert(now);
    let parameters = &blackboard.parameters.kicking.strategy.passing;
    if started > now || now.duration_since(started) >= parameters.preparation_timeout {
        blackboard.pass_cooldown_until = Some(now + parameters.cooldown);
        blackboard.pass_started_at = None;
        blackboard.last_announced_pass = None;
        blackboard.ball_action = Some(BallAction::Hold);
        blackboard.kick_target = None;
        return false;
    }
    has_fresh_pass_acknowledgement(blackboard, receiver, target)
}

/// Selection and execution share the same addressed, post-announcement
/// acknowledgement check. Inspecting readiness never starts or extends a pass.
pub(crate) fn has_fresh_pass_acknowledgement(
    blackboard: &Blackboard,
    receiver: PlayerNumber,
    target: Point2<Field>,
) -> bool {
    let now = blackboard.world_state.now;
    let parameters = &blackboard.parameters.kicking.strategy.passing;
    if !matches_announced_pass(blackboard, PassIntent { receiver, target }) {
        return false;
    }
    let Some(announced) = blackboard.last_announced_pass.as_ref() else {
        return false;
    };
    let Some(receiver_state) = blackboard.timed_player_states[receiver].as_ref() else {
        return false;
    };
    if receiver_state.time <= announced.time
        || receiver_state.time > now
        || now.duration_since(receiver_state.time) > parameters.maximum_ready_age
    {
        return false;
    }
    receiver_state.inner.pass_ready.is_some_and(|ready| {
        ready.sender == blackboard.world_state.robot.player_number
            && (ready.target - target).norm() <= parameters.target_half_width
    })
}

fn announced_pass(blackboard: &Blackboard) -> Option<(PlayerNumber, PassIntent, Point2<Field>)> {
    let world = &blackboard.world_state;
    let own_position = world.robot.ground_to_field?.as_pose().position();
    let parameters = &blackboard.parameters.kicking.strategy.passing;
    parameters.validate().ok()?;
    if parameters.maximum_distance <= 0.0
        || world.robot.player_number == blackboard.parameters.goalkeeper.player_number
        || !finite_position(own_position)
        || world
            .filtered_game_controller_state
            .as_ref()
            .is_some_and(|game| game.penalties[world.robot.player_number].is_some())
    {
        return None;
    }
    blackboard
        .timed_player_states
        .iter()
        .filter_map(|(sender, state)| {
            let state = state.as_ref()?;
            let pass = state.inner.pass?;
            let sender_position = state.inner.pose.position();
            if sender == world.robot.player_number
                || pass.receiver != world.robot.player_number
                || state.time > world.now
                || world.now.duration_since(state.time)
                    >= PassIntent::TIMEOUT.min(parameters.maximum_receiver_age)
                || world
                    .filtered_game_controller_state
                    .as_ref()
                    .is_some_and(|game| game.penalties[sender].is_some())
                || !finite_position(pass.target)
                || !finite_position(sender_position)
                || pass.target.x().abs() >= blackboard.field_dimensions.length / 2.0
                || pass.target.y().abs() >= blackboard.field_dimensions.width / 2.0
                || (pass.target - own_position).norm()
                    > parameters.receiver_standoff + 2.0 * parameters.target_half_width
                || (sender_position - pass.target).norm()
                    > parameters.maximum_distance
                        + blackboard.parameters.kicking.kick_activation_distance
            {
                return None;
            }
            Some((state.time, sender, pass, sender_position))
        })
        .max_by_key(|(time, ..)| *time)
        .map(|(_, sender, pass, position)| (sender, pass, position))
}

fn retained_pass(blackboard: &Blackboard) -> Option<ReceivingPass> {
    let pass = blackboard.receiving_pass?;
    let world = &blackboard.world_state;
    let parameters = &blackboard.parameters.kicking.strategy.passing;
    parameters.validate().ok()?;
    let own_position = world.robot.ground_to_field?.as_pose().position();
    let ball = blackboard.ball.as_ref()?;
    let maximum_age = PassIntent::TIMEOUT.min(parameters.preparation_timeout);
    let direction = (own_position - pass.sender_position).try_normalize(f32::EPSILON)?;
    let relative_ball = ball.position - own_position;
    let corridor_width = parameters.target_half_width
        + blackboard.parameters.walking.path_planning.robot_radius
        + blackboard.field_dimensions.ball_radius;
    let lateral_distance =
        (relative_ball.x() * direction.y() - relative_ball.y() * direction.x()).abs();
    let close = relative_ball.norm()
        <= blackboard
            .parameters
            .kicking
            .kick_activation_distance
            .max(parameters.receiver_standoff + parameters.target_half_width);
    let passed = relative_ball.dot(&direction) > blackboard.field_dimensions.ball_radius;
    if parameters.maximum_distance <= 0.0
        || pass.released_at.is_some()
        || pass.launched_at > world.now
        || world.now.duration_since(pass.launched_at) >= maximum_age
        || ball.age > world.now
        || world.now.duration_since(ball.age)
            > crate::node::ball_timeout(&blackboard.parameters, ball.source)
        || !finite_position(ball.position)
        || !ball.velocity.inner.iter().all(|value| value.is_finite())
        || !finite_position(own_position)
        || !finite_position(pass.target)
        || !finite_position(pass.sender_position)
        || world.robot.player_number == blackboard.parameters.goalkeeper.player_number
        || world
            .filtered_game_controller_state
            .as_ref()
            .is_some_and(|game| game.penalties[world.robot.player_number].is_some())
        || (pass.target - own_position).norm()
            > parameters.receiver_standoff + 2.0 * parameters.target_half_width
        || lateral_distance > corridor_width
        || (!close && !passed && ball.velocity.dot(&(-relative_ball)) <= 0.0)
    {
        return None;
    }
    Some(pass)
}

fn incoming_pass(blackboard: &Blackboard) -> Option<(PlayerNumber, PassIntent, Point2<Field>)> {
    announced_pass(blackboard)
        .filter(|(sender, pass, _)| {
            blackboard.receiving_pass.is_none_or(|receiving| {
                receiving.released_at.is_none()
                    || !same_offer(blackboard, receiving, *sender, *pass)
            })
        })
        .or_else(|| {
            let pass = retained_pass(blackboard)?;
            Some((
                pass.sender,
                PassIntent {
                    receiver: blackboard.world_state.robot.player_number,
                    target: pass.target,
                },
                pass.sender_position,
            ))
        })
}

fn same_offer(
    blackboard: &Blackboard,
    receiving: ReceivingPass,
    sender: PlayerNumber,
    pass: PassIntent,
) -> bool {
    receiving.sender == sender
        && (receiving.target - pass.target).norm()
            <= blackboard
                .parameters
                .kicking
                .strategy
                .passing
                .target_half_width
}

pub fn is_pass_receiver(blackboard: &mut Blackboard) -> bool {
    let announced = announced_pass(blackboard);
    if retained_pass(blackboard).is_none()
        && let Some(mut receiving) = blackboard.receiving_pass
    {
        if announced
            .is_some_and(|(sender, pass, _)| same_offer(blackboard, receiving, sender, pass))
        {
            receiving
                .released_at
                .get_or_insert(blackboard.world_state.now);
            blackboard.receiving_pass = Some(receiving);
        } else {
            blackboard.receiving_pass = None;
        }
    }
    if let Some((sender, pass, sender_position)) = announced
        && blackboard.receiving_pass.is_none_or(|receiving| {
            receiving.released_at.is_none() || !same_offer(blackboard, receiving, sender, pass)
        })
        && let Some(own_position) = blackboard
            .world_state
            .robot
            .ground_to_field
            .map(|transform| transform.as_pose().position())
        && let Some(ball) = blackboard.ball.as_ref()
        && let Some(direction) = (own_position - sender_position).try_normalize(f32::EPSILON)
    {
        let parameters = &blackboard.parameters.kicking.strategy.passing;
        let from_sender = ball.position - sender_position;
        let speed = ball.velocity.norm();
        let lateral = (from_sender.x() * direction.y() - from_sender.y() * direction.x()).abs();
        let reception_width = parameters.target_half_width
            + blackboard.parameters.walking.path_planning.robot_radius
            + blackboard.field_dimensions.ball_radius;
        let closest_time =
            (own_position - ball.position).dot(&ball.velocity) / ball.velocity.norm_squared();
        let launched = finite_position(ball.position)
            && ball.velocity.inner.iter().all(|value| value.is_finite())
            && ball.age <= blackboard.world_state.now
            && blackboard.world_state.now.duration_since(ball.age)
                <= crate::node::ball_timeout(&blackboard.parameters, ball.source)
            && speed > parameters.maximum_reception_speed
            && ball.velocity.dot(&direction) > 0.0
            && closest_time > 0.0
            && (ball.position + ball.velocity * closest_time - own_position).norm()
                <= reception_width
            && from_sender.dot(&direction) > 2.0 * blackboard.field_dimensions.ball_radius
            && lateral <= reception_width;
        if launched
            && blackboard.receiving_pass.is_none_or(|current| {
                current.sender != sender
                    || (current.target - pass.target).norm() > parameters.target_half_width
            })
        {
            blackboard.receiving_pass = Some(ReceivingPass {
                sender,
                target: pass.target,
                sender_position,
                launched_at: blackboard.world_state.now,
                released_at: None,
            });
        }
    }
    incoming_pass(blackboard).is_some()
}

pub fn is_pass_ball_arriving(blackboard: &mut Blackboard) -> bool {
    let arriving = incoming_pass(blackboard).is_some_and(|(_, pass, sender)| {
        let transform = blackboard.world_state.robot.ground_to_field.unwrap();
        let own_position = transform.as_pose().position();
        let parameters = &blackboard.parameters.kicking.strategy.passing;
        blackboard.ball.as_ref().is_some_and(|ball| {
            let relative = ball.position - own_position;
            let captured = ball.velocity.norm() <= parameters.maximum_reception_speed
                && (relative.norm() < blackboard.parameters.kicking.kick_activation_distance
                    || (ball.position - pass.target).norm()
                        <= parameters.receiver_standoff + parameters.target_half_width);
            let passed = retained_pass(blackboard).is_some()
                && (own_position - sender)
                    .try_normalize(f32::EPSILON)
                    .is_some_and(|direction| {
                        relative.dot(&direction) > blackboard.field_dimensions.ball_radius
                    });
            (captured || passed) && relative.norm() < (ball.position - sender).norm()
        })
    });
    if arriving {
        // The addressed receiver takes over without the ordinary ownership delay.
        blackboard.last_closest_to_ball = true;
        blackboard.closest_to_ball_entered_area_since = None;
        blackboard.closest_to_ball_left_area_since = None;
        if let Some(receiving) = &mut blackboard.receiving_pass {
            receiving.released_at = Some(blackboard.world_state.now);
        }
    }
    arriving
}

pub fn prepare_to_receive_pass(blackboard: &mut Blackboard) -> Status {
    blackboard.ready_to_receive_pass = None;
    let Some((sender_number, pass, sender)) = incoming_pass(blackboard) else {
        return Status::Failure;
    };
    let transform = blackboard.world_state.robot.ground_to_field.unwrap();
    let ball = blackboard
        .ball
        .as_ref()
        .map_or(sender, |ball| ball.position);
    let ball_in_ground = transform.inverse() * ball;
    blackboard.head_motion = Some(HeadMotion::LookAt {
        target: ball_in_ground,
        height_above_ground: blackboard.field_dimensions.ball_radius,
        image_region_target: ImageRegion::Center,
    });
    let Some(direction) = ball_in_ground.coords().try_normalize(f32::EPSILON) else {
        return stand(blackboard);
    };
    let angle = Orientation2::from_vector(direction).angle();
    let parameters = &blackboard.parameters.kicking.strategy.passing;
    let upright = blackboard
        .world_state
        .fall_detection
        .as_ref()
        .is_some_and(|fall| {
            fall.is_valid(blackboard.world_state.now) && fall.posture == Posture::Upright
        });
    let in_place = (pass.target - transform.as_pose().position()).norm()
        <= parameters.receiver_standoff + parameters.target_half_width;
    if upright && in_place && angle.abs() <= parameters.receiver_orientation_tolerance {
        blackboard.ready_to_receive_pass = Some(PassReady {
            sender: sender_number,
            target: pass.target,
        });
        return stand(blackboard);
    }
    if !upright {
        return stand(blackboard);
    }
    // The passer selected this receiver's current position. Hold it while turning
    // toward the ball rather than continuing the ordinary support movement.
    blackboard.body_motion = Some(BodyMotion::WalkWithVelocity {
        velocity: Vector2::zeros(),
        angular_velocity: blackboard.walking_velocity_limits.scale_angular_axis(angle),
    });
    Status::Success
}

fn finite_position(position: Point2<Field>) -> bool {
    position
        .inner
        .iter()
        .all(|coordinate| coordinate.is_finite())
}

#[cfg(test)]
mod tests {
    use super::*;
    use linear_algebra::{Isometry2, Pose2, point, vector};
    use std::time::Duration;
    use types::{
        fall_detection::FallDetection,
        field_dimensions::FieldDimensions,
        field_dimensions::Side,
        time_wrapper::TimeWrapper,
        world_state::{BallSource, PlayerState},
    };

    fn blackboard() -> Blackboard {
        let mut b = Blackboard::default();
        b.field_dimensions = FieldDimensions::SPL_2025;
        b.world_state.robot.player_number = PlayerNumber::Two;
        b.parameters.goalkeeper.player_number = PlayerNumber::One;
        let p = &mut b.parameters.kicking.strategy.passing;
        p.maximum_distance = 3.0;
        p.maximum_receiver_age = Duration::from_secs(3);
        p.maximum_ready_age = Duration::from_millis(700);
        p.preparation_timeout = Duration::from_secs(6);
        p.cooldown = Duration::from_secs(2);
        p.target_half_width = 0.2;
        p.receiver_standoff = 0.5;
        p.receiver_orientation_tolerance = 0.5;
        p.maximum_reception_speed = 0.5;
        b
    }

    #[test]
    fn sender_requires_acknowledgement_after_announcement_and_times_out() {
        let mut b = blackboard();
        let target = point![1.5, 0.0];
        b.ball_action = Some(BallAction::Pass {
            receiver: PlayerNumber::Three,
        });
        b.last_announced_pass = Some(TimeWrapper {
            time: b.world_state.now,
            inner: PassIntent {
                receiver: PlayerNumber::Three,
                target,
            },
        });
        b.last_sent_hsl_message_time = Some(b.world_state.now);
        b.timed_player_states.three = Some(TimeWrapper {
            time: b.world_state.now,
            inner: PlayerState {
                pass_ready: Some(PassReady {
                    sender: PlayerNumber::Two,
                    target,
                }),
                ..Default::default()
            },
        });
        assert!(!pass_is_ready(&mut b, PlayerNumber::Three, target));
        b.world_state.now = b.world_state.now + Duration::from_millis(100);
        b.timed_player_states.three.as_mut().unwrap().time = b.world_state.now;
        assert!(pass_is_ready(&mut b, PlayerNumber::Three, target));
        b.timed_player_states
            .three
            .as_mut()
            .unwrap()
            .inner
            .pass_ready
            .as_mut()
            .unwrap()
            .sender = PlayerNumber::Four;
        assert!(!pass_is_ready(&mut b, PlayerNumber::Three, target));
        b.world_state.now = b.world_state.now + Duration::from_secs(6);
        assert!(!pass_is_ready(&mut b, PlayerNumber::Three, target));
        assert_eq!(b.ball_action, Some(BallAction::Hold));
        assert!(b.pass_cooldown_until.unwrap() > b.world_state.now);
        assert!(b.last_announced_pass.is_none());
    }

    #[test]
    fn receiver_acknowledges_only_a_fresh_in_place_upright_facing_intent() {
        let mut b = blackboard();
        b.world_state.robot.ground_to_field = Some(Isometry2::from_parts(
            vector![2.0, 0.0],
            std::f32::consts::PI,
        ));
        b.world_state.fall_detection = Some(FallDetection {
            time: b.world_state.now,
            valid_until: b.world_state.now + Duration::from_secs(10),
            posture: Posture::Upright,
        });
        b.timed_player_states.three = Some(TimeWrapper {
            time: b.world_state.now,
            inner: PlayerState {
                pose: Pose2::new(point![0.0, 0.0], 0.0),
                pass: Some(PassIntent {
                    receiver: PlayerNumber::Two,
                    target: point![1.5, 0.0],
                }),
                ..Default::default()
            },
        });
        prepare_to_receive_pass(&mut b);
        assert_eq!(
            b.ready_to_receive_pass,
            Some(PassReady {
                sender: PlayerNumber::Three,
                target: point![1.5, 0.0]
            })
        );
        b.world_state.robot.ground_to_field = Some(Isometry2::from_parts(vector![2.0, 0.0], 0.0));
        prepare_to_receive_pass(&mut b);
        assert!(b.ready_to_receive_pass.is_none());
        b.world_state.robot.ground_to_field = Some(Isometry2::from_parts(
            vector![2.0, 0.0],
            std::f32::consts::PI,
        ));
        b.world_state.fall_detection.as_mut().unwrap().posture = Posture::Falling;
        prepare_to_receive_pass(&mut b);
        assert!(b.ready_to_receive_pass.is_none());
        b.world_state.now = b.world_state.now + PassIntent::TIMEOUT;
        assert!(!is_pass_receiver(&mut b));
    }

    fn receiving_blackboard() -> Blackboard {
        let mut b = blackboard();
        b.world_state.now = Time::from_nanos(1_000_000_000);
        b.world_state.robot.ground_to_field = Some(Isometry2::from_parts(
            vector![2.0, 0.0],
            std::f32::consts::PI,
        ));
        b.world_state.fall_detection = Some(FallDetection {
            time: b.world_state.now,
            valid_until: b.world_state.now + Duration::from_secs(10),
            posture: Posture::Upright,
        });
        b.parameters.kicking.kick_activation_distance = 0.8;
        b.parameters.walking.path_planning.robot_radius = 0.4;
        b.parameters.ball.last_ball_timeout = Duration::from_millis(500);
        b.timed_player_states.three = Some(TimeWrapper {
            time: b.world_state.now,
            inner: PlayerState {
                pose: Pose2::new(point![0.0, 0.0], 0.0),
                pass: Some(PassIntent {
                    receiver: PlayerNumber::Two,
                    target: point![1.5, 0.0],
                }),
                ..Default::default()
            },
        });
        measured_ball(&mut b, point![0.9, 0.0], vector![1.8, 0.0]);
        b
    }

    fn measured_ball(b: &mut Blackboard, position: Point2<Field>, velocity: Vector2<Field>) {
        b.ball = Some(crate::node::LastBall {
            position,
            velocity,
            age: b.world_state.now,
            field_side: Side::Left,
            source: BallSource::Own,
        });
    }

    #[test]
    fn moving_pass_keeps_receiver_standing_until_the_measured_ball_slows() {
        let mut b = receiving_blackboard();
        assert!(is_pass_receiver(&mut b));
        assert!(b.receiving_pass.is_some());
        assert!(!is_pass_ball_arriving(&mut b));
        assert_eq!(prepare_to_receive_pass(&mut b), Status::Success);
        assert!(matches!(b.body_motion, Some(BodyMotion::Stand)));

        b.world_state.now = b.world_state.now + Duration::from_millis(400);
        measured_ball(&mut b, point![1.714, 0.0], vector![1.64, 0.0]);
        assert!(is_pass_receiver(&mut b));
        assert!(!is_pass_ball_arriving(&mut b));
        prepare_to_receive_pass(&mut b);
        assert!(matches!(b.body_motion, Some(BodyMotion::Stand)));

        measured_ball(&mut b, point![1.8, 0.0], vector![0.2, 0.0]);
        assert!(is_pass_ball_arriving(&mut b));
        assert!(b.last_closest_to_ball);
        assert!(b.receiving_pass.unwrap().released_at.is_some());
        assert!(!is_pass_receiver(&mut b));
    }

    #[test]
    fn sender_withdrawal_after_launch_keeps_reception_and_a_passed_ball_can_be_recovered() {
        let mut b = receiving_blackboard();
        assert!(is_pass_receiver(&mut b));
        let launched_at = b.receiving_pass.unwrap().launched_at;
        b.world_state.now = b.world_state.now + Duration::from_millis(300);
        b.timed_player_states.three.as_mut().unwrap().inner.pass = None;
        b.timed_player_states.three.as_mut().unwrap().time = b.world_state.now;
        measured_ball(&mut b, point![1.7, 0.0], vector![1.5, 0.0]);
        assert!(is_pass_receiver(&mut b));
        assert_eq!(b.receiving_pass.unwrap().launched_at, launched_at);
        assert!(!is_pass_ball_arriving(&mut b));
        prepare_to_receive_pass(&mut b);
        assert!(matches!(b.body_motion, Some(BodyMotion::Stand)));

        measured_ball(&mut b, point![2.2, 0.0], vector![1.0, 0.0]);
        assert!(is_pass_receiver(&mut b));
        assert!(is_pass_ball_arriving(&mut b));
        assert!(b.receiving_pass.unwrap().released_at.is_some());
        assert!(!is_pass_receiver(&mut b));
        assert!(b.receiving_pass.is_none());
    }

    #[test]
    fn cancelled_or_stale_offers_before_launch_never_latch_reception() {
        let mut b = receiving_blackboard();
        measured_ball(&mut b, point![0.2, 0.0], Vector2::zeros());
        assert!(is_pass_receiver(&mut b));
        assert!(b.receiving_pass.is_none());
        b.timed_player_states.three.as_mut().unwrap().inner.pass = None;
        assert!(!is_pass_receiver(&mut b));

        let mut b = receiving_blackboard();
        b.world_state.now = b.world_state.now + PassIntent::TIMEOUT;
        assert!(!is_pass_receiver(&mut b));
        assert!(b.receiving_pass.is_none());
    }

    #[test]
    fn retained_reception_expires_and_rejects_a_clear_miss_or_stale_ball() {
        for invalidation in 0..3 {
            let mut b = receiving_blackboard();
            assert!(is_pass_receiver(&mut b));
            b.timed_player_states.three = None;
            match invalidation {
                0 => {
                    b.world_state.now = b.world_state.now + PassIntent::TIMEOUT;
                    measured_ball(&mut b, point![1.2, 0.0], vector![1.0, 0.0]);
                }
                1 => measured_ball(&mut b, point![1.0, 1.5], vector![1.0, 0.0]),
                _ => b.world_state.now = b.world_state.now + Duration::from_secs(1),
            }
            assert!(!is_pass_receiver(&mut b));
            assert!(b.receiving_pass.is_none());
        }
    }

    #[test]
    fn repeated_offer_cannot_restart_a_missed_or_expired_reception() {
        for expired in [false, true] {
            let mut b = receiving_blackboard();
            assert!(is_pass_receiver(&mut b));
            if expired {
                b.world_state.now = b.world_state.now + PassIntent::TIMEOUT;
                b.timed_player_states.three.as_mut().unwrap().time = b.world_state.now;
                measured_ball(&mut b, point![1.2, 0.0], vector![1.0, 0.0]);
            } else {
                measured_ball(&mut b, point![1.0, 1.5], vector![1.0, 0.0]);
            }
            assert!(!is_pass_receiver(&mut b));
            let released_at = b.receiving_pass.unwrap().released_at.unwrap();
            b.world_state.now = b.world_state.now + Duration::from_millis(100);
            b.timed_player_states.three.as_mut().unwrap().time = b.world_state.now;
            measured_ball(&mut b, point![1.7, 0.0], vector![1.0, 0.0]);
            assert!(!is_pass_receiver(&mut b));
            assert_eq!(b.receiving_pass.unwrap().released_at, Some(released_at));
            b.timed_player_states.three.as_mut().unwrap().inner.pass = None;
            assert!(!is_pass_receiver(&mut b));
            assert!(b.receiving_pass.is_none());
        }
    }
}
