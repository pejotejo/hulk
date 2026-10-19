use coordinate_systems::Field;
use filtering::hysteresis::{
    greater_than_with_relative_hysteresis, less_than_with_relative_hysteresis,
};
use geometry::{Distance, line_segment::LineSegment};
use hsl_network_messages::PlayerNumber;
use linear_algebra::point;
use ros_z::Message;
use serde::{Deserialize, Serialize};
use types::{behavior_tree::Status, obstacles::ObstacleKind};

use crate::node::Blackboard;

use super::{
    geometry::{goal_openings, pass_openings},
    kick_ball_position_in_ground, pass_receiver_position,
};

/// Tactical intent only. Execution chooses the target, approach, and physical motion.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Message)]
pub enum BallAction {
    Shoot {
        opening: LineSegment<Field>,
    },
    Pass {
        receiver: PlayerNumber,
    },
    Dribble,
    Clear,
    /// Last resort; does not require an open corridor or a reachable goal.
    ShootAtGoalCenter,
}

/// Select ordinary striker intent. Clear `ball_action` when leaving that context.
/// Execution still needs to validate clear/dribble corridors and kick availability.
pub fn select_ball_action(blackboard: &mut Blackboard) -> Status {
    blackboard.ball_action = choose_ball_action(blackboard);
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
    let previous = blackboard.ball_action;
    let ground_to_field = world.robot.ground_to_field?;
    let ball_in_ground = kick_ball_position_in_ground(blackboard, &ground_to_field)?;
    let ball = ground_to_field * ball_in_ground;
    let goal_x = field.length / 2.0;
    if ball.x().abs() >= goal_x || ball.y().abs() >= field.width / 2.0 {
        return None;
    }

    let was_clearing = matches!(previous, Some(BallAction::Clear));
    let own_goal = LineSegment(
        point![-goal_x, -field.goal_inner_width / 2.0],
        point![-goal_x, field.goal_inner_width / 2.0],
    );
    // Robot affiliation is unknown; nearby robots conservatively count as pressure.
    let under_pressure = world.obstacles.iter().any(|obstacle| {
        matches!(
            obstacle.kind,
            ObstacleKind::Robot | ObstacleKind::Person | ObstacleKind::Unknown
        ) && less_than_with_relative_hysteresis(
            was_clearing,
            (obstacle.position - ball_in_ground).norm() - obstacle.radius_at_foot_height,
            parameters.pressure_distance,
            0.0..=parameters.clear_distance_hysteresis,
        )
    });
    if under_pressure
        && less_than_with_relative_hysteresis(
            was_clearing,
            own_goal.distance_to(ball),
            parameters.clear_goal_distance,
            0.0..=parameters.clear_distance_hysteresis,
        )
    {
        return Some(BallAction::Clear);
    }

    let range = parameters.maximum_shot_distance;
    if range.is_finite() && range > goal_x - ball.x() {
        // Clip regions to reachable goal points before rating their angular width.
        let reachable_y = (range.powi(2) - (goal_x - ball.x()).powi(2)).sqrt();
        let mut best = None;
        let mut retained = None;
        for opening in goal_openings(
            ball,
            ground_to_field,
            &world.obstacles,
            field,
            parameters.obstacle_clearance,
        ) {
            let lower = opening.0.y().max(ball.y() - reachable_y);
            let upper = opening.1.y().min(ball.y() + reachable_y);
            if lower >= upper {
                continue;
            }
            let opening = LineSegment(point![goal_x, lower], point![goal_x, upper]);
            let angle = LineSegment(ball, opening.0).angle(LineSegment(ball, opening.1));
            if angle < parameters.minimum_shot_angle {
                continue;
            }
            if best.is_none_or(|(_, best_angle)| angle > best_angle) {
                best = Some((opening, angle));
            }
            if let Some(BallAction::Shoot { opening: previous }) = previous
                && previous.0.x() == goal_x
                && (lower..upper).contains(&previous.center().y())
            {
                retained = Some((opening, angle));
            }
        }
        if let Some((opening, angle)) = best
            && greater_than_with_relative_hysteresis(
                matches!(previous, Some(BallAction::Shoot { .. })),
                angle,
                parameters.minimum_shot_angle,
                0.0..=parameters.shot_angle_hysteresis,
            )
        {
            let opening = retained
                .filter(|(_, old_angle)| angle <= old_angle + parameters.shot_angle_hysteresis)
                .map_or(opening, |(opening, _)| opening);
            return Some(BallAction::Shoot { opening });
        }
    }

    if !blackboard.has_team_message_budget() {
        return Some(BallAction::Dribble);
    }
    let mut best = None;
    let mut retained = None;
    for (receiver, _) in world.player_states.iter() {
        let Some(position) = pass_receiver_position(blackboard, ball, receiver) else {
            continue;
        };
        if pass_openings(
            ball,
            position,
            ground_to_field,
            &world.obstacles,
            field,
            &parameters.passing,
            parameters.obstacle_clearance,
        )
        .is_empty()
        {
            continue;
        }
        let progress = position.x() - ball.x();
        if best.is_none_or(|(_, best_progress)| progress > best_progress) {
            best = Some((receiver, progress));
        }
        if previous == Some(BallAction::Pass { receiver }) {
            retained = Some((receiver, progress));
        }
    }
    if let Some((receiver, progress)) = best {
        let receiver = retained
            .filter(|(_, old_progress)| {
                progress <= old_progress + parameters.passing.selection_hysteresis
            })
            .map_or(receiver, |(receiver, _)| receiver);
        return Some(BallAction::Pass { receiver });
    }

    Some(BallAction::Dribble)
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use hsl_network_messages::Penalty;
    use linear_algebra::Pose2;
    use types::{
        ball_position::BallPosition,
        fall_detection::{FallDetection, Posture},
        field_dimensions::{FieldDimensions, Half, Side},
        motion_command::BodyMotion,
        obstacles::Obstacle,
        parameters::{KickStrategyParameters, PassingParameters},
        time_wrapper::TimeWrapper,
        world_state::{BallState, PlayerState},
    };

    use crate::{
        kick::{
            execution::execute_ball_action,
            passing::{is_pass_ball_arriving, is_pass_receiver},
        },
        node::LastBall,
        tree::{create_tree, striker_subtree},
    };

    use super::*;

    fn blackboard() -> Blackboard {
        let mut blackboard = Blackboard {
            field_dimensions: FieldDimensions::SPL_2025,
            ball: Some(LastBall {
                position: point![2.0, 0.0],
                velocity: Default::default(),
                age: Default::default(),
                field_side: Side::Left,
                source: types::world_state::BallSource::Own,
            }),
            ..Default::default()
        };
        blackboard.world_state.robot.ground_to_field = Some(Default::default());
        blackboard.parameters.kicking.strategy = KickStrategyParameters {
            maximum_shot_distance: 3.0,
            maximum_dribble_distance: 3.0,
            obstacle_clearance: 0.05,
            minimum_shot_angle: 0.02,
            shot_angle_hysteresis: 0.05,
            pressure_distance: 0.8,
            clear_goal_distance: 1.5,
            clear_distance_hysteresis: 0.2,
            passing: Default::default(),
        };
        blackboard
    }

    fn passing_blackboard() -> Blackboard {
        let mut blackboard = blackboard();
        blackboard.world_state.robot.primary_state = types::primary_state::PrimaryState::Playing;
        blackboard.world_state.filtered_game_controller_state = Some(
            types::filtered_game_controller_state::FilteredGameControllerState {
                remaining_number_of_messages: 100,
                remaining_time_in_half: Duration::from_millis(4800),
                ..Default::default()
            },
        );
        blackboard
            .parameters
            .network
            .remaining_amount_of_messages_to_stop_sending = 20;
        blackboard.ball.as_mut().unwrap().position = point![0.0, 0.0];
        blackboard.parameters.kicking.strategy.passing = PassingParameters {
            maximum_distance: 3.0,
            minimum_progress: 0.5,
            receiver_standoff: 0.5,
            target_half_width: 0.2,
            selection_hysteresis: 0.25,
            maximum_receiver_age: Duration::from_millis(500),
        };
        blackboard.parameters.kicking.kick_activation_distance = 3.0;
        blackboard.parameters.kicking.target_speed = 1.0;
        blackboard.world_state.player_states.two = Some(TimeWrapper {
            time: blackboard.world_state.now,
            inner: PlayerState {
                pose: Pose2::new(point![2.0, 0.0], 0.0),
                ..Default::default()
            },
        });
        blackboard.world_state.obstacles = vec![Obstacle::robot(point![2.0, 0.0], 0.2, 0.25)];
        blackboard
    }

    #[test]
    fn pass_refines_receiver_region_and_revalidates_before_kicking() {
        let mut blackboard = passing_blackboard();
        let transform =
            linear_algebra::Isometry2::from_parts(linear_algebra::vector![0.3, -0.2], 0.6);
        blackboard.world_state.robot.ground_to_field = Some(transform);
        blackboard.world_state.obstacles = [
            Obstacle::robot(transform.inverse() * point![2.0, 0.0], 0.2, 0.25),
            Obstacle::robot(transform.inverse() * point![0.8, -0.04], 0.01, 0.01),
        ]
        .to_vec();
        blackboard.parameters.kicking.allow_strong_kicks = true;
        blackboard
            .parameters
            .kicking
            .strong_kick_min_target_distance = 0.1;
        blackboard.parameters.kicking.quick = true;
        select_ball_action(&mut blackboard);
        assert_eq!(
            blackboard.ball_action,
            Some(BallAction::Pass {
                receiver: PlayerNumber::Two
            })
        );
        execute_ball_action(&mut blackboard);
        let target = transform * blackboard.kick_target.unwrap();
        assert!((target.x() - 1.5).abs() < 1e-5);
        assert!(target.y() > 0.0 && target.y() < 0.2);
        assert_eq!(blackboard.body_motion, Some(BodyMotion::Stand));
        assert!(blackboard.try_sending_state_message().is_some());
        blackboard.world_state.now = blackboard.world_state.now + Duration::from_millis(150);
        execute_ball_action(&mut blackboard);
        let Some(BodyMotion::Kick {
            kick_direction,
            soft: true,
            strong: false,
            quick: false,
            target_speed: 1.0,
            ..
        }) = blackboard.body_motion
        else {
            panic!("expected controlled pass")
        };
        let direction = transform * kick_direction.as_unit_vector();
        assert!((direction - target.coords().normalize()).norm() < 1e-5);
        for obstacle in &blackboard.world_state.obstacles {
            assert!(
                LineSegment(point![0.0, 0.0], target).distance_to(transform * obstacle.position)
                    > obstacle.radius_at_foot_height + 0.1
            );
        }

        let receiver = point![2.0, 0.4];
        blackboard
            .world_state
            .player_states
            .two
            .as_mut()
            .unwrap()
            .inner
            .pose = Pose2::new(receiver, 0.0);
        blackboard.world_state.obstacles[0].position = transform.inverse() * receiver;
        execute_ball_action(&mut blackboard);
        let refined = transform * blackboard.kick_target.unwrap();
        assert!(refined.y() > target.y());
        assert!((refined - receiver).norm() <= 0.54);

        blackboard.world_state.obstacles.push(Obstacle::robot(
            transform.inverse() * point![0.8, 0.0],
            0.4,
            0.4,
        ));
        execute_ball_action(&mut blackboard);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick { .. })
        ));
        assert!((transform * blackboard.kick_target.unwrap() - point![4.5, 0.0]).norm() < 1e-5);
        assert_eq!(blackboard.ball_action, Some(BallAction::ShootAtGoalCenter));
        assert!(blackboard.last_announced_pass.is_none());
    }

    #[test]
    fn pass_rejects_unavailable_receivers_and_unsafe_corridors() {
        type InvalidateReceiver = fn(&mut Blackboard);
        let cases: [(&str, InvalidateReceiver); 12] = [
            ("goalkeeper", |b| {
                b.parameters.goalkeeper.player_number = PlayerNumber::Two
            }),
            ("message budget", |b| {
                b.world_state
                    .filtered_game_controller_state
                    .as_mut()
                    .unwrap()
                    .remaining_number_of_messages = 19;
            }),
            ("disabled", |b| {
                b.parameters.kicking.strategy.passing.maximum_distance = 0.0
            }),
            ("missing", |b| b.world_state.player_states.two = None),
            ("self", |b| {
                b.world_state.robot.player_number = PlayerNumber::Two
            }),
            ("stale", |b| {
                b.world_state.now = b.world_state.now + Duration::from_millis(500)
            }),
            ("future", |b| {
                b.world_state.player_states.two.as_mut().unwrap().time =
                    b.world_state.now + Duration::from_nanos(1)
            }),
            ("penalized", |b| {
                let mut game =
                    types::filtered_game_controller_state::FilteredGameControllerState::default();
                game.penalties.two = Some(Penalty::PickUp {
                    remaining: Duration::from_secs(1),
                });
                b.world_state.filtered_game_controller_state = Some(game);
            }),
            ("behind", |b| {
                b.world_state.player_states.two.as_mut().unwrap().inner.pose =
                    Pose2::new(point![-1.0, 0.0], 0.0)
            }),
            ("range", |b| {
                b.world_state.player_states.two.as_mut().unwrap().inner.pose =
                    Pose2::new(point![4.1, 0.0], 0.0)
            }),
            ("blocked", |b| {
                b.world_state
                    .obstacles
                    .push(Obstacle::robot(point![0.7, 0.0], 0.4, 0.4))
            }),
            ("opponent near receiver", |b| {
                b.world_state
                    .obstacles
                    .push(Obstacle::robot(point![1.65, 0.0], 0.2, 0.2))
            }),
        ];
        for (name, invalidate) in cases {
            let mut blackboard = passing_blackboard();
            invalidate(&mut blackboard);
            select_ball_action(&mut blackboard);
            assert_eq!(blackboard.ball_action, Some(BallAction::Dribble), "{name}");
            blackboard.ball_action = Some(BallAction::Pass {
                receiver: PlayerNumber::Two,
            });
            execute_ball_action(&mut blackboard);
            assert!(
                matches!(blackboard.body_motion, Some(BodyMotion::Kick { .. })),
                "{name}"
            );
            assert_eq!(blackboard.kick_target, Some(point![4.5, 0.0]), "{name}");
            assert_eq!(
                blackboard.ball_action,
                Some(BallAction::ShootAtGoalCenter),
                "{name}"
            );
        }
    }

    #[test]
    fn pass_waits_for_normal_message_and_preparation_without_acknowledgement() {
        let mut blackboard = passing_blackboard();
        blackboard.last_sent_hsl_message_time = Some(blackboard.world_state.now);
        select_ball_action(&mut blackboard);
        execute_ball_action(&mut blackboard);
        assert_eq!(blackboard.body_motion, Some(BodyMotion::Stand));
        assert!(blackboard.try_sending_state_message().is_none());

        blackboard.world_state.now = blackboard.world_state.now + Duration::from_millis(301);
        execute_ball_action(&mut blackboard);
        assert_eq!(blackboard.body_motion, Some(BodyMotion::Stand));
        let Some(types::messages::OutgoingMessage::Hsl(hsl_network_messages::HulkMessage::State(
            message,
        ))) = blackboard.try_sending_state_message()
        else {
            panic!("expected regular state message")
        };
        let pass = message.pass.unwrap();
        assert_eq!(pass.receiver, PlayerNumber::Two);
        let transform = blackboard.world_state.robot.ground_to_field.unwrap();
        assert_eq!(
            Some(pass.target),
            blackboard.kick_target.map(|target| transform * target)
        );
        execute_ball_action(&mut blackboard);
        assert_eq!(blackboard.body_motion, Some(BodyMotion::Stand));
        blackboard.world_state.now = blackboard.world_state.now + Duration::from_millis(149);
        execute_ball_action(&mut blackboard);
        assert_eq!(blackboard.body_motion, Some(BodyMotion::Stand));
        blackboard.world_state.now = blackboard.world_state.now + Duration::from_millis(1);
        execute_ball_action(&mut blackboard);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick { .. })
        ));
        assert!(blackboard.try_sending_state_message().is_none());

        // A materially different reception target needs its own ordinary update.
        let receiver = point![2.0, 0.8];
        let state = blackboard.world_state.player_states.two.as_mut().unwrap();
        state.inner.pose = receiver.into();
        state.time = blackboard.world_state.now;
        blackboard.world_state.obstacles[0].position = transform.inverse() * receiver;
        execute_ball_action(&mut blackboard);
        assert_eq!(blackboard.body_motion, Some(BodyMotion::Stand));
        blackboard.world_state.now = blackboard.world_state.now + Duration::from_millis(151);
        assert!(blackboard.try_sending_state_message().is_some());
        execute_ball_action(&mut blackboard);
        assert_eq!(blackboard.body_motion, Some(BodyMotion::Stand));
        blackboard.world_state.now = blackboard.world_state.now + Duration::from_millis(150);
        execute_ball_action(&mut blackboard);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick { .. })
        ));

        blackboard.ball_action = Some(BallAction::Dribble);
        blackboard.world_state.now = blackboard.world_state.now + Duration::from_millis(151);
        let Some(types::messages::OutgoingMessage::Hsl(hsl_network_messages::HulkMessage::State(
            message,
        ))) = blackboard.try_sending_state_message()
        else {
            panic!("expected cancellation in regular update")
        };
        assert_eq!(message.pass, None);
        assert!(blackboard.last_announced_pass.is_none());
    }

    #[test]
    fn receiver_prepares_for_stationary_ball_then_claims_arrival_and_expires_intent() {
        let mut receiver = passing_blackboard();
        receiver.world_state.robot.player_number = PlayerNumber::Two;
        receiver.parameters.goalkeeper.player_number = PlayerNumber::One;
        receiver.world_state.robot.ground_to_field = Some(linear_algebra::Isometry2::from_parts(
            linear_algebra::vector![2.0, 0.0],
            0.0,
        ));
        receiver.world_state.obstacles.clear();
        receiver.world_state.player_states = Default::default();
        receiver.world_state.player_states.three = Some(TimeWrapper {
            time: receiver.world_state.now,
            inner: PlayerState {
                pass: Some(hsl_network_messages::PassIntent {
                    receiver: PlayerNumber::Two,
                    target: point![1.5, 0.0],
                }),
                ..Default::default()
            },
        });
        receiver.world_state.fall_detection = Some(FallDetection {
            time: receiver.world_state.now,
            valid_until: receiver.world_state.now + Duration::from_secs(10),
            posture: Posture::Upright,
        });
        receiver.walking_velocity_limits.angular_velocity_limit = 1.0;
        receiver.parameters.voronoi.grid_resolution = 0.1;
        receiver.parameters.ball.closest_to_ball.enter_duration = Duration::from_millis(300);
        receiver.world_state.ball = Some(BallState {
            ball_in_ground: point![-2.0, 0.0],
            ..Default::default()
        });
        let tree = create_tree();
        assert!(is_pass_receiver(&mut receiver));
        assert!(!is_pass_ball_arriving(&mut receiver));
        tree.tick_with_trace(&mut receiver);
        let Some(BodyMotion::WalkWithVelocity {
            velocity,
            angular_velocity,
        }) = &receiver.body_motion
        else {
            panic!("receiver must face ball before it moves")
        };
        assert_eq!(velocity.norm(), 0.0);
        assert_eq!(angular_velocity.abs(), 1.0);
        assert!(receiver.kick_target.is_none());

        receiver.ball.as_mut().unwrap().position = point![0.4, 0.0];
        receiver.ball.as_mut().unwrap().velocity = linear_algebra::vector![2.0, 0.0];
        receiver.world_state.ball.as_mut().unwrap().ball_in_ground = point![-1.6, 0.0];
        receiver.world_state.ball.as_mut().unwrap().ball_in_field = point![0.4, 0.0];
        receiver
            .world_state
            .ball
            .as_mut()
            .unwrap()
            .ball_in_ground_velocity = linear_algebra::vector![2.0, 0.0];
        assert!(!is_pass_ball_arriving(&mut receiver));
        receiver.parameters.ball.pursuit_lookahead = Duration::from_millis(500);
        tree.tick_with_trace(&mut receiver);
        assert!(receiver.last_closest_to_ball);
        assert!(matches!(
            receiver.body_motion,
            Some(BodyMotion::Kick { .. })
        ));

        // A fast ball beside the receiver must not project beyond the reception area.
        receiver.ball.as_mut().unwrap().position = point![1.6, 0.0];
        receiver.ball.as_mut().unwrap().velocity = linear_algebra::vector![4.0, 0.0];
        let ball = receiver.world_state.ball.as_mut().unwrap();
        ball.ball_in_ground = point![-0.4, 0.0];
        ball.ball_in_field = point![1.6, 0.0];
        ball.ball_in_ground_velocity = linear_algebra::vector![4.0, 0.0];
        tree.tick_with_trace(&mut receiver);
        assert!(
            matches!(receiver.body_motion, Some(BodyMotion::Kick { ball_position, .. })
            if (ball_position - point![-0.4, 0.0]).norm() < 1e-6)
        );

        receiver
            .world_state
            .player_states
            .three
            .as_mut()
            .unwrap()
            .inner
            .pass = None;
        assert!(!is_pass_receiver(&mut receiver));
        receiver.kick_target = None;
        tree.tick_with_trace(&mut receiver);
        assert!(matches!(
            receiver.body_motion,
            Some(BodyMotion::Kick { .. })
        ));
        receiver
            .world_state
            .player_states
            .three
            .as_mut()
            .unwrap()
            .inner
            .pass = Some(hsl_network_messages::PassIntent {
            receiver: PlayerNumber::Two,
            target: point![1.5, 0.0],
        });
        receiver.world_state.now = receiver.world_state.now + Duration::from_secs(1);
        assert!(!is_pass_receiver(&mut receiver));
    }

    #[test]
    fn pursuit_projection_is_optional_bounded_and_ignores_invalid_velocity() {
        let mut blackboard = blackboard();
        let position = point![1.0, 0.5];
        let velocity = linear_algebra::vector![2.0, -1.0];
        assert_eq!(
            blackboard.projected_ball_position(position, velocity),
            position
        );
        blackboard.parameters.ball.pursuit_lookahead = Duration::from_millis(500);
        assert_eq!(
            blackboard.projected_ball_position(position, velocity),
            point![2.0, 0.0]
        );
        assert_eq!(
            blackboard.projected_ball_position(position, linear_algebra::Vector2::zeros()),
            position
        );
        assert_eq!(
            blackboard.projected_ball_position(position, linear_algebra::vector![f32::NAN, 1.0]),
            position
        );
        let projected =
            blackboard.projected_ball_position(position, linear_algebra::vector![20.0, -20.0]);
        assert_eq!(projected, point![4.45, -2.95]);
        let outside = point![4.6, 0.0];
        assert_eq!(
            blackboard.projected_ball_position(outside, -velocity),
            outside
        );
        blackboard.world_state.robot.ground_to_field = Some(linear_algebra::Isometry2::from_parts(
            linear_algebra::vector![0.5, 0.5],
            1.0,
        ));
        assert_eq!(
            blackboard.projected_ball_position(position, velocity),
            position
        );
    }

    #[test]
    fn pass_retains_receiver_until_improvement_or_corridor_closure() {
        let mut blackboard = passing_blackboard();
        blackboard
            .world_state
            .player_states
            .two
            .as_mut()
            .unwrap()
            .inner
            .pose = Pose2::new(point![2.0, 1.0], 0.0);
        let fourth = TimeWrapper {
            time: blackboard.world_state.now,
            inner: PlayerState {
                pose: Pose2::new(point![1.9, -1.0], 0.0),
                ..Default::default()
            },
        };
        blackboard.world_state.player_states.four = Some(fourth);
        blackboard.world_state.obstacles.clear();
        for (x, expected) in [
            (1.9, PlayerNumber::Two),
            (2.1, PlayerNumber::Two),
            (2.4, PlayerNumber::Four),
        ] {
            blackboard
                .world_state
                .player_states
                .four
                .as_mut()
                .unwrap()
                .inner
                .pose = Pose2::new(point![x, -1.0], 0.0);
            select_ball_action(&mut blackboard);
            assert_eq!(
                blackboard.ball_action,
                Some(BallAction::Pass { receiver: expected })
            );
        }
        blackboard
            .world_state
            .obstacles
            .push(Obstacle::robot(point![0.8, -0.4], 0.2, 0.2));
        select_ball_action(&mut blackboard);
        assert_eq!(
            blackboard.ball_action,
            Some(BallAction::Pass {
                receiver: PlayerNumber::Two
            })
        );
    }

    #[test]
    fn shots_and_emergency_clears_take_priority_over_passes() {
        let mut blackboard = passing_blackboard();
        blackboard.ball.as_mut().unwrap().position = point![2.0, 0.0];
        blackboard
            .world_state
            .player_states
            .two
            .as_mut()
            .unwrap()
            .inner
            .pose = Pose2::new(point![3.3, 1.0], 0.0);
        blackboard.world_state.obstacles = vec![Obstacle::robot(point![3.3, 1.0], 0.2, 0.2)];
        select_ball_action(&mut blackboard);
        assert!(matches!(
            blackboard.ball_action,
            Some(BallAction::Shoot { .. })
        ));

        blackboard
            .world_state
            .obstacles
            .push(Obstacle::robot(point![3.6, 0.0], 0.4, 0.4));
        select_ball_action(&mut blackboard);
        assert_eq!(
            blackboard.ball_action,
            Some(BallAction::Pass {
                receiver: PlayerNumber::Two
            })
        );

        blackboard.ball.as_mut().unwrap().position = point![-3.8, 0.0];
        blackboard
            .world_state
            .player_states
            .two
            .as_mut()
            .unwrap()
            .inner
            .pose = Pose2::new(point![-1.8, 0.0], 0.0);
        blackboard.world_state.obstacles = vec![Obstacle::robot(point![-3.8, 0.7], 0.2, 0.2)];
        select_ball_action(&mut blackboard);
        assert_eq!(blackboard.ball_action, Some(BallAction::Clear));
    }

    #[test]
    fn selects_intent_from_blackboard_and_clears_it_without_ball() {
        let mut blackboard = blackboard();
        assert_eq!(select_ball_action(&mut blackboard), Status::Success);
        assert!(matches!(
            blackboard.ball_action,
            Some(BallAction::Shoot { .. })
        ));
        blackboard.world_state.obstacles = vec![Obstacle::robot(point![3.0, 0.0], 0.5, 0.5)];
        select_ball_action(&mut blackboard);
        assert_eq!(blackboard.ball_action, Some(BallAction::Dribble));
        blackboard.ball.as_mut().unwrap().position = point![-3.8, 0.0];
        blackboard.world_state.obstacles = vec![Obstacle::robot(point![-3.2, 0.0], 0.2, 0.2)];
        select_ball_action(&mut blackboard);
        assert_eq!(blackboard.ball_action, Some(BallAction::Clear));
        blackboard.ball = None;
        assert_eq!(select_ball_action(&mut blackboard), Status::Failure);
        assert_eq!(blackboard.ball_action, None);
    }

    #[test]
    fn dribble_advances_toward_goal_through_a_clear_corridor() {
        let mut blackboard = blackboard();
        let ball = point![1.0, 1.0];
        blackboard.ball.as_mut().unwrap().position = ball;
        blackboard.parameters.kicking.kick_activation_distance = 10.0;
        select_ball_action(&mut blackboard);
        assert_eq!(blackboard.ball_action, Some(BallAction::Dribble));
        execute_ball_action(&mut blackboard);
        let goal = point![blackboard.field_dimensions.length / 2.0, 0.0];
        let target =
            blackboard.world_state.robot.ground_to_field.unwrap() * blackboard.kick_target.unwrap();
        assert!(target.x() > ball.x() && target.x() < goal.x());
        assert!(LineSegment(ball, target).angle(LineSegment(ball, goal)) < 1e-6);

        let obstacle = Obstacle::robot(point![2.0, 0.714], 0.2, 0.2);
        blackboard.world_state.obstacles = vec![obstacle];
        execute_ball_action(&mut blackboard);
        let target =
            blackboard.world_state.robot.ground_to_field.unwrap() * blackboard.kick_target.unwrap();
        assert!(
            LineSegment(ball, target).distance_to(
                blackboard.world_state.robot.ground_to_field.unwrap() * obstacle.position,
            ) > obstacle.radius_at_foot_height
                + blackboard.field_dimensions.ball_radius
                + blackboard.parameters.kicking.strategy.obstacle_clearance
        );
        // Detour just around the blocker instead of aiming at the opening's midpoint.
        assert!(LineSegment(ball, target).angle(LineSegment(ball, goal)) < 0.4);

        // Strategy and execution must use the same observed ball, not disagree on range.
        blackboard.world_state.obstacles.clear();
        blackboard.visual_kick_ball_position = Some(BallPosition {
            position: point![2.0, 0.0],
            velocity: Default::default(),
            last_seen: Default::default(),
        });
        select_ball_action(&mut blackboard);
        assert!(matches!(
            blackboard.ball_action,
            Some(BallAction::Shoot { .. })
        ));
        execute_ball_action(&mut blackboard);
        assert_eq!(blackboard.kick_target.unwrap().x(), goal.x());
    }

    #[test]
    fn dribble_recovers_from_h2_end_line_position_without_shortening_travel() {
        let mut blackboard = blackboard();
        blackboard.field_dimensions = FieldDimensions {
            length: 8.95,
            width: 5.95,
            goal_inner_width: 2.4,
            ball_radius: 0.105,
            line_width: 0.06,
            ..FieldDimensions::SPL_2025
        };
        let field = blackboard.field_dimensions;
        let ball = point![4.219_695, -2.053_933_6];
        let transform =
            linear_algebra::Isometry2::from_parts(linear_algebra::vector![3.99, -1.86], -0.43);
        blackboard.world_state.robot.ground_to_field = Some(transform);
        blackboard.ball.as_mut().unwrap().position = ball;
        blackboard.parameters.kicking.kick_activation_distance = 3.0;
        blackboard.world_state.obstacles = [Half::Own, Half::Opponent]
            .into_iter()
            .flat_map(|half| {
                [Side::Left, Side::Right].map(move |side| {
                    Obstacle::goal_post(
                        transform.inverse() * field.goal_post(half, side),
                        field.goal_post_diameter / 2.0,
                    )
                })
            })
            .chain(
                [
                    point![4.278_642_7, -1.109_891_7],
                    point![-4.045, -0.018],
                    point![-1.616, 0.0],
                ]
                .map(|position| Obstacle::robot(transform.inverse() * position, 0.2, 0.25)),
            )
            .collect();

        select_ball_action(&mut blackboard);
        assert_eq!(blackboard.ball_action, Some(BallAction::Dribble));
        execute_ball_action(&mut blackboard);
        let target = transform * blackboard.kick_target.unwrap();
        let travel = blackboard
            .parameters
            .kicking
            .strategy
            .maximum_dribble_distance;
        let margin = field.ball_radius + blackboard.parameters.kicking.strategy.obstacle_clearance;
        assert!((target - ball).norm() >= travel - 1e-5);
        assert!(target.x() < ball.x() && target.y() > ball.y());
        assert!(target.x().abs() <= field.length / 2.0 - margin);
        assert!(target.y().abs() <= field.width / 2.0 - margin);
        for obstacle in &blackboard.world_state.obstacles {
            assert!(
                LineSegment(ball, target).distance_to(transform * obstacle.position)
                    > obstacle.radius_at_foot_height + margin
            );
        }
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick {
                soft: true,
                strong: false,
                ..
            })
        ));
    }

    #[test]
    fn dribble_falls_back_to_goal_center_but_invalid_parameters_still_stop() {
        let mut blackboard = blackboard();
        blackboard.ball.as_mut().unwrap().position = point![0.0, 0.0];
        blackboard.field_dimensions.length = 1.0;
        blackboard.field_dimensions.width = 1.0;
        blackboard.parameters.kicking.kick_activation_distance = 1.0;
        for distance in [3.0, 0.0, -1.0, f32::NAN, f32::INFINITY] {
            blackboard.ball_action = Some(BallAction::Dribble);
            blackboard
                .parameters
                .kicking
                .strategy
                .maximum_dribble_distance = distance;
            execute_ball_action(&mut blackboard);
            if distance.is_finite() && distance >= 0.0 {
                assert!(matches!(
                    blackboard.body_motion,
                    Some(BodyMotion::Kick { .. })
                ));
                assert_eq!(blackboard.kick_target, Some(point![0.5, 0.0]));
                assert_eq!(blackboard.ball_action, Some(BallAction::ShootAtGoalCenter));
            } else {
                assert_eq!(blackboard.body_motion, Some(BodyMotion::Stand));
                assert_eq!(blackboard.kick_target, None);
                assert_eq!(blackboard.ball_action, None);
            }
        }
    }

    #[test]
    fn invalid_strategy_parameters_stop_selection_and_execution() {
        for invalid in [-0.01, f32::NAN, f32::INFINITY] {
            let mut blackboard = blackboard();
            blackboard.parameters.kicking.strategy.shot_angle_hysteresis = invalid;
            assert_eq!(select_ball_action(&mut blackboard), Status::Failure);
            assert_eq!(blackboard.ball_action, None);
            blackboard.ball_action = Some(BallAction::Dribble);
            execute_ball_action(&mut blackboard);
            assert_eq!(blackboard.body_motion, Some(BodyMotion::Stand));
            assert_eq!(blackboard.kick_target, None);
        }
    }

    #[test]
    fn goal_center_fallback_stops_without_valid_ball_or_localization() {
        for case in 0..5 {
            let mut blackboard = blackboard();
            blackboard.ball_action = Some(BallAction::ShootAtGoalCenter);
            match case {
                0 => blackboard.ball = None,
                1 => blackboard.world_state.robot.ground_to_field = None,
                2 => blackboard.ball.as_mut().unwrap().position = point![4.5, 0.0],
                3 => blackboard.ball.as_mut().unwrap().position = point![0.0, 3.0],
                _ => blackboard.ball.as_mut().unwrap().position = point![f32::NAN, 0.0],
            }
            execute_ball_action(&mut blackboard);
            assert_eq!(
                blackboard.body_motion,
                Some(BodyMotion::Stand),
                "case {case}"
            );
            assert_eq!(blackboard.kick_target, None, "case {case}");
        }
    }

    #[test]
    fn clear_uses_forward_part_of_a_sideways_opening() {
        let mut blackboard = blackboard();
        let ball = point![-3.8, -0.8];
        blackboard.ball.as_mut().unwrap().position = ball;
        blackboard.parameters.kicking.kick_activation_distance = 5.0;
        blackboard.world_state.obstacles = [
            point![-3.123_852, -0.981_173],
            point![-3.193_782, -0.45],
            point![-3.45, -0.193_782],
        ]
        .map(|position| Obstacle::robot(position, 0.2, 0.2))
        .to_vec();
        select_ball_action(&mut blackboard);
        assert_eq!(blackboard.ball_action, Some(BallAction::Clear));
        execute_ball_action(&mut blackboard);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick { .. })
        ));
        let transform = blackboard.world_state.robot.ground_to_field.unwrap();
        let target = transform * blackboard.kick_target.unwrap();
        assert!(target.x() > ball.x());
        assert!((target - ball).norm() >= 3.0);
        for obstacle in &blackboard.world_state.obstacles {
            assert!(
                LineSegment(ball, target).distance_to(transform * obstacle.position)
                    > obstacle.radius_at_foot_height
                        + blackboard.field_dimensions.ball_radius
                        + blackboard.parameters.kicking.strategy.obstacle_clearance
            );
        }
    }

    #[test]
    fn kick_handoff_uses_the_selected_visual_ball() {
        let mut blackboard = blackboard();
        blackboard.parameters.kicking.kick_activation_distance = 1.0;
        blackboard.visual_kick_ball_position = Some(BallPosition {
            position: point![0.5, 0.0],
            velocity: Default::default(),
            last_seen: Default::default(),
        });
        select_ball_action(&mut blackboard);
        execute_ball_action(&mut blackboard);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick { ball_position, .. }) if ball_position == point![0.5, 0.0]
        ));

        blackboard.ball.as_mut().unwrap().position = point![0.5, 0.0];
        blackboard
            .visual_kick_ball_position
            .as_mut()
            .unwrap()
            .position = point![2.0, 0.0];
        select_ball_action(&mut blackboard);
        execute_ball_action(&mut blackboard);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Walk { .. })
        ));
    }

    #[test]
    fn retains_shot_side_under_small_changes_but_rejects_blocked_opening() {
        let mut blackboard = blackboard();
        blackboard.world_state.obstacles = vec![Obstacle::robot(point![3.5, -0.02], 0.1, 0.1)];
        for y in [-0.02, 0.02] {
            blackboard.world_state.obstacles[0].position = point![3.5, y];
            select_ball_action(&mut blackboard);
            let Some(BallAction::Shoot { opening }) = blackboard.ball_action else {
                panic!("expected shot")
            };
            assert!(opening.center().y() > 0.0);
        }
        blackboard.world_state.obstacles[0].radius_at_foot_height = 1.0;
        select_ball_action(&mut blackboard);
        assert_eq!(blackboard.ball_action, Some(BallAction::Dribble));
    }

    #[test]
    fn execution_approaches_ball_revalidates_and_kicks_to_advance() {
        let mut blackboard = blackboard();
        blackboard.parameters.kicking.kick_activation_distance = 1.0;
        blackboard.parameters.kicking.approach_ball_standoff = 0.3;
        blackboard
            .parameters
            .walking
            .walk_and_stand
            .target_reached_thresholds = nalgebra::vector![0.05, 0.1];
        blackboard.parameters.walking.path_planning.robot_radius = 0.4;
        blackboard
            .parameters
            .walking
            .path_planning
            .ball_obstacle_radius = 0.05;
        let transform =
            linear_algebra::Isometry2::from_parts(linear_algebra::vector![0.2, -0.1], 0.3);
        blackboard.world_state.robot.ground_to_field = Some(transform);
        blackboard.parameters.ball.pursuit_lookahead = Duration::from_millis(500);
        blackboard.ball.as_mut().unwrap().velocity = linear_algebra::vector![0.0, 1.0];
        let observed = transform.inverse() * point![2.0, 0.0];
        let velocity = transform.inverse() * linear_algebra::vector![0.0, 1.0];
        blackboard.world_state.ball = Some(BallState {
            ball_in_ground: observed,
            ball_in_field: point![2.0, 0.0],
            ball_in_ground_velocity: velocity,
            ..Default::default()
        });
        blackboard.world_state.obstacles = vec![Obstacle::robot(
            transform.inverse() * point![3.5, 0.0],
            0.1,
            0.1,
        )];
        let tree = striker_subtree();
        assert_eq!(tree.tick_with_trace(&mut blackboard).0, Status::Success);
        let target = transform * blackboard.kick_target.unwrap();
        let Some(BodyMotion::Walk {
            target_orientation,
            ref path,
            ..
        }) = blackboard.body_motion
        else {
            panic!("expected approach")
        };
        let ball_in_ground = transform.inverse() * point![2.0, 0.5];
        let facing_ball = linear_algebra::Orientation2::from_vector(ball_in_ground.coords());
        assert!((target_orientation.angle() - facing_ball.angle()).abs() < 1e-6);
        let expected_approach = ball_in_ground - ball_in_ground.coords().normalize() * 0.3;
        let types::path::PathSegment::LineSegment(last) = path.last_segment() else {
            panic!("expected final straight approach")
        };
        assert!((last.1 - expected_approach).norm() < 1e-5);
        let expected = linear_algebra::Orientation2::from_vector(
            transform.inverse() * (target - point![2.0, 0.0]),
        );
        blackboard.parameters.kicking.kick_activation_distance = 3.0;
        tree.tick_with_trace(&mut blackboard);
        let Some(BodyMotion::Kick {
            kick_direction,
            ball_position,
            ball_velocity,
            ..
        }) = blackboard.body_motion
        else {
            panic!("expected shot")
        };
        assert!((blackboard.kick_target.unwrap() - transform.inverse() * target).norm() < 1e-6);
        assert!((kick_direction.angle() - expected.angle()).abs() < 1e-6);
        assert_eq!(ball_position, observed);
        assert_eq!(ball_velocity, velocity);
        blackboard.world_state.obstacles[0].radius_at_foot_height = 1.0;
        execute_ball_action(&mut blackboard);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick { .. })
        ));
        assert!((transform * blackboard.kick_target.unwrap() - point![4.5, 0.0]).norm() < 1e-5);
        assert_eq!(blackboard.ball_action, Some(BallAction::ShootAtGoalCenter));

        blackboard.world_state.robot.ground_to_field = Some(Default::default());
        blackboard.ball.as_mut().unwrap().position = point![-3.8, 0.0];
        blackboard.world_state.obstacles = vec![Obstacle::robot(point![-3.8, 0.7], 0.2, 0.2)];
        blackboard.parameters.kicking.kick_activation_distance = 5.0;
        tree.tick_with_trace(&mut blackboard);
        assert_eq!(blackboard.ball_action, Some(BallAction::Clear));
        assert!(blackboard.kick_target.unwrap().x() > -3.8);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick { .. })
        ));

        // The recorded stall had 6 cm lateral error; kick-policy alignment must handle it.
        blackboard.parameters.kicking.kick_activation_distance = 2.0;
        blackboard.ball.as_mut().unwrap().position = point![0.51, -0.06];
        blackboard.world_state.obstacles.clear();
        blackboard.parameters.kicking.allow_strong_kicks = true;
        blackboard
            .parameters
            .kicking
            .strong_kick_min_target_distance = 0.1;
        blackboard.last_motion_command = types::motion_command::MotionCommand::from_partial_motions(
            BodyMotion::Kick {
                ball_position: point![0.51, -0.06],
                ball_velocity: Default::default(),
                target_speed: 1.0,
                soft: false,
                quick: false,
                kick_direction: Default::default(),
                strong: true,
            },
            types::motion_command::HeadMotion::Center {
                image_region_target: Default::default(),
            },
        );
        tree.tick_with_trace(&mut blackboard);
        assert_eq!(blackboard.ball_action, Some(BallAction::Dribble));
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick {
                strong: false,
                soft: true,
                ..
            })
        ));
        assert!(blackboard.kick_target.unwrap().x() > 0.51);
        blackboard.world_state.obstacles = vec![Obstacle::robot(point![0.6, 0.0], 0.3, 0.3)];
        tree.tick_with_trace(&mut blackboard);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick { .. })
        ));
        assert_eq!(blackboard.ball_action, Some(BallAction::ShootAtGoalCenter));
        assert_eq!(blackboard.kick_target, Some(point![4.5, 0.0]));
    }
}
