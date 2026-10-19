use hsl_network_messages::SubState;
use linear_algebra::{Rotation2, point};
use types::{
    behavior_tree::Status, field_dimensions::Side,
    filtered_game_controller_state::FilteredGameControllerState,
};

use crate::{
    action,
    behavior_tree::Node,
    condition,
    conditions::hulks_is_kicking_team,
    kick::{execution::execute_ball_action, strategy::select_ball_action},
    node::Blackboard,
    penalty_shootout::penalty_kick_striker_subtree,
    selection, sequence, subtree,
    walk::walk_to_block_position,
};

pub fn sub_state_subtree() -> Node<Blackboard> {
    selection!(
        sequence!(
            condition!(hulks_is_kicking_team),
            selection!(
                sequence!(
                    condition!(is_sub_state, SubState::PenaltyKick),
                    subtree!(penalty_kick_striker_subtree),
                ),
                sequence!(action!(select_ball_action), action!(execute_ball_action)),
                action!(crate::actions::stand),
            )
        ),
        sequence!(
            selection!(
                sequence!(
                    selection!(
                        condition!(is_sub_state, SubState::ThrowIn),
                        condition!(is_sub_state, SubState::IndirectFreeKick),
                        condition!(is_sub_state, SubState::DirectFreeKick),
                        condition!(is_sub_state, SubState::GoalKick),
                    ),
                    action!(set_block_position_field),
                ),
                sequence!(
                    condition!(is_sub_state, SubState::CornerKick),
                    action!(set_block_position_corner),
                ),
                sequence!(
                    condition!(is_sub_state, SubState::PenaltyKick),
                    action!(set_block_position_penalty_kick),
                )
            ),
            action!(walk_to_block_position)
        )
    )
}

pub fn is_in_sub_state(blackboard: &mut Blackboard) -> bool {
    matches!(
        blackboard.world_state.filtered_game_controller_state,
        Some(FilteredGameControllerState {
            sub_state: Some(_),
            kicking_team: Some(_),
            ..
        })
    )
}

pub fn is_sub_state(blackboard: &mut Blackboard, sub_state: SubState) -> bool {
    matches!(
        blackboard.world_state.filtered_game_controller_state,
        Some(FilteredGameControllerState {
            sub_state: Some(current_sub_state),
            ..
        }) if current_sub_state == sub_state
    )
}

pub fn set_block_position_field(blackboard: &mut Blackboard) -> Status {
    if let (Some(ball), Some(ground_to_field)) = (
        &blackboard.ball,
        &blackboard.world_state.robot.ground_to_field,
    ) {
        let ball_in_ground = ground_to_field.inverse() * ball.position;
        let goal_position =
            ground_to_field.inverse() * point!(-blackboard.field_dimensions.length / 2.0, 0.0);
        let direction = (goal_position - ball_in_ground).normalize();

        let distance_to_ball = (blackboard.field_dimensions.center_circle_diameter / 2.0
            + blackboard.parameters.substates.blocking_distance_offset)
            .max(
                blackboard.field_dimensions.center_circle_diameter / 2.0
                    + blackboard.parameters.walking.path_planning.robot_radius,
            );

        blackboard.walk_position = Some(ball_in_ground + (direction * distance_to_ball));

        Status::Success
    } else {
        Status::Failure
    }
}

pub fn set_block_position_corner(blackboard: &mut Blackboard) -> Status {
    if let (Some(ball), Some(ground_to_field)) = (
        &blackboard.ball,
        &blackboard.world_state.robot.ground_to_field,
    ) {
        let ball_position = ground_to_field.inverse() * ball.position;
        let goal_position =
            ground_to_field.inverse() * point!(-blackboard.field_dimensions.length / 2.0, 0.0);
        let angle_direction = match ball.field_side {
            Side::Left => 1.0,
            Side::Right => -1.0,
        };

        let parameters = &blackboard.parameters.substates;

        let direction = Rotation2::new(angle_direction * parameters.corner_kick_blocking_angle)
            * (goal_position - ball_position).normalize();

        let distance_to_ball = (blackboard.field_dimensions.center_circle_diameter / 2.0
            + parameters.blocking_distance_offset)
            .max(
                blackboard.field_dimensions.center_circle_diameter / 2.0
                    + blackboard.parameters.walking.path_planning.robot_radius,
            );

        blackboard.walk_position = Some(ball_position + direction * distance_to_ball);

        Status::Success
    } else {
        Status::Failure
    }
}

pub fn set_block_position_penalty_kick(blackboard: &mut Blackboard) -> Status {
    if let Some(ground_to_field) = &blackboard.world_state.robot.ground_to_field {
        let field_dimensions = blackboard.field_dimensions;
        let penalty_area_x = -field_dimensions.length / 2.0 + field_dimensions.penalty_area_length;
        let distance_to_ball_along_x =
            field_dimensions.penalty_area_length - field_dimensions.penalty_marker_distance;
        let distance_to_ball = (blackboard.field_dimensions.center_circle_diameter / 2.0
            + blackboard.parameters.substates.blocking_distance_offset)
            .max(blackboard.field_dimensions.center_circle_diameter / 2.0);

        let line_position = (distance_to_ball.powi(2) - distance_to_ball_along_x.powi(2))
            .max(0.0)
            .sqrt();

        let side_sign = (ground_to_field * point!(0.0, 0.0)).y().signum();

        blackboard.walk_position =
            Some(ground_to_field.inverse() * point!(penalty_area_x, side_sign * line_position));

        Status::Success
    } else {
        Status::Failure
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use hsl_network_messages::{PlayerNumber, Team};
    use linear_algebra::{Point2, Pose2};
    use types::{
        fall_detection::{FallDetection, Posture},
        field_dimensions::FieldDimensions,
        filtered_game_state::FilteredGameState,
        messages::OutgoingMessage,
        motion_command::{BodyMotion, MotionCommand},
        motion_type::MotionType,
        obstacles::Obstacle,
        parameters::{KickStrategyParameters, PassingParameters},
        primary_state::PrimaryState,
        time_wrapper::TimeWrapper,
        world_state::PlayerState,
    };

    use crate::{kick::strategy::BallAction, node::LastBall};

    use super::*;

    fn restart(state: SubState, ball: Point2<coordinate_systems::Field>) -> Blackboard {
        let mut board = Blackboard {
            field_dimensions: FieldDimensions::SPL_2025,
            last_motion_type: Some(MotionType::Walk),
            ball: Some(LastBall {
                position: ball,
                velocity: Default::default(),
                age: Default::default(),
                field_side: Side::Left,
                source: Default::default(),
            }),
            ..Default::default()
        };
        board.world_state.now = board.world_state.now + Duration::from_secs(1);
        board.world_state.fall_detection = Some(FallDetection {
            time: board.world_state.now,
            valid_until: board.world_state.now + Duration::from_secs(10),
            posture: Posture::Upright,
        });
        board.world_state.robot.primary_state = PrimaryState::Playing;
        board.world_state.robot.ground_to_field =
            Some(Pose2::new(ball - linear_algebra::vector![0.6, 0.0], 0.7).as_transform());
        board.world_state.filtered_game_controller_state = Some(FilteredGameControllerState {
            kicking_team: Some(Team::Hulks),
            sub_state: Some(state),
            game_state: FilteredGameState::Playing {
                ball_is_free: false,
                kick_off: false,
            },
            remaining_number_of_messages: 1000,
            remaining_time_in_half: Duration::from_secs(100),
            ..Default::default()
        });
        board.parameters.goalkeeper.player_number = PlayerNumber::One;
        board.parameters.voronoi.grid_resolution = 0.25;
        board.parameters.voronoi.padding = 0.5;
        board
            .parameters
            .walking
            .walk_and_stand
            .orientation_tolerance = 0.05;
        board.parameters.kicking.kick_activation_distance = 1.3;
        board.parameters.kicking.allow_strong_kicks = true;
        board.parameters.kicking.strong_kick_min_target_distance = 3.0;
        board.parameters.kicking.strategy = KickStrategyParameters {
            maximum_shot_distance: 12.0,
            maximum_dribble_distance: 3.0,
            obstacle_clearance: 0.05,
            minimum_shot_angle: 0.02,
            passing: PassingParameters {
                maximum_distance: 8.0,
                minimum_progress: 0.25,
                receiver_standoff: 0.5,
                target_half_width: 0.2,
                maximum_receiver_age: Duration::from_secs(3),
                ..Default::default()
            },
            ..Default::default()
        };
        board
    }

    #[test]
    fn own_restarts_pass_from_touchlines_and_without_goal_alignment() {
        for (state, ball, receiver) in [
            (SubState::GoalKick, point![-3.5, 0.0], point![-1.5, 0.0]),
            (SubState::ThrowIn, point![0.0, 3.0], point![-1.0, 1.0]),
            (SubState::CornerKick, point![4.5, 3.0], point![2.5, 1.0]),
            (
                SubState::IndirectFreeKick,
                point![0.0, 0.0],
                point![2.0, 1.0],
            ),
        ] {
            let mut board = restart(state, ball);
            board.world_state.player_states.two = Some(TimeWrapper {
                time: board.world_state.now,
                inner: PlayerState {
                    pose: Pose2::new(receiver, 0.0),
                    ..Default::default()
                },
            });
            let tree = sub_state_subtree();
            assert_eq!(
                tree.tick_with_trace(&mut board).0,
                Status::Success,
                "{state:?}"
            );
            assert_eq!(
                board.ball_action,
                Some(BallAction::Pass {
                    receiver: PlayerNumber::Two
                }),
                "{state:?}"
            );
            assert_eq!(board.body_motion, Some(BodyMotion::Stand));
            assert!(board.try_sending_state_message().is_some());
            board.world_state.now = board.world_state.now + Duration::from_millis(150);
            tree.tick_with_trace(&mut board);
            assert!(
                matches!(
                    board.body_motion,
                    Some(BodyMotion::Kick {
                        soft: false,
                        strong: false,
                        ..
                    })
                ),
                "{state:?}"
            );
        }
    }

    #[test]
    fn own_restart_without_receiver_shoots_or_plays_infield() {
        let mut board = restart(SubState::GoalKick, point![-3.5, 0.0]);
        sub_state_subtree().tick_with_trace(&mut board);
        assert!(matches!(board.ball_action, Some(BallAction::Shoot { .. })));
        assert!(matches!(
            board.body_motion,
            Some(BodyMotion::Kick {
                strong: true,
                soft: false,
                ..
            })
        ));

        for state in [SubState::ThrowIn, SubState::IndirectFreeKick] {
            let mut board = restart(state, point![0.0, 3.0]);
            sub_state_subtree().tick_with_trace(&mut board);
            assert_eq!(board.ball_action, Some(BallAction::Dribble), "{state:?}");
            assert!(
                matches!(
                    board.body_motion,
                    Some(BodyMotion::Kick {
                        strong: false,
                        soft: false,
                        ..
                    })
                ),
                "{state:?}"
            );
        }
    }

    #[test]
    fn addressed_restart_receiver_holds_position_until_taker_kicks() {
        for (state, ball, position) in [
            (SubState::CornerKick, point![4.5, 3.0], point![2.5, 1.0]),
            (SubState::ThrowIn, point![0.0, 3.0], point![-1.0, 1.0]),
        ] {
            let mut taker = restart(state, ball);
            taker.world_state.robot.player_number = PlayerNumber::Two;
            let pose = Pose2::new(
                position,
                (ball.y() - position.y()).atan2(ball.x() - position.x()),
            );
            taker.world_state.player_states.three = Some(TimeWrapper {
                time: taker.world_state.now,
                inner: PlayerState {
                    pose,
                    ..Default::default()
                },
            });
            let tree = crate::tree::create_tree();
            tree.tick_with_trace(&mut taker);
            assert_eq!(
                taker.ball_action,
                Some(BallAction::Pass {
                    receiver: PlayerNumber::Three
                }),
                "{state:?}"
            );
            assert_eq!(taker.body_motion, Some(BodyMotion::Stand));
            let Some(OutgoingMessage::Hsl(hsl_network_messages::HulkMessage::State(message))) =
                taker.try_sending_state_message()
            else {
                panic!("restart pass must be announced");
            };

            let mut receiver = taker.clone();
            receiver.world_state.robot.player_number = PlayerNumber::Three;
            receiver.world_state.robot.ground_to_field = Some(pose.as_transform());
            receiver.world_state.player_states = Default::default();
            receiver.world_state.player_states.two = Some(TimeWrapper {
                time: receiver.world_state.now,
                inner: PlayerState {
                    pose: message.pose,
                    pass: message.pass,
                    ..Default::default()
                },
            });
            receiver.ball_action = None;
            receiver.kick_target = None;
            receiver.body_motion = None;
            tree.tick_with_trace(&mut receiver);
            assert_eq!(receiver.body_motion, Some(BodyMotion::Stand), "{state:?}");

            taker.world_state.now = taker.world_state.now + Duration::from_millis(150);
            let state = taker.world_state.player_states.three.as_mut().unwrap();
            state.time = taker.world_state.now;
            state.inner.pose = receiver
                .world_state
                .robot
                .ground_to_field
                .unwrap()
                .as_pose();
            taker.kick_target = None;
            taker.body_motion = None;
            tree.tick_with_trace(&mut taker);
            assert!(matches!(
                taker.body_motion,
                Some(BodyMotion::Kick {
                    strong: false,
                    soft: false,
                    ..
                })
            ));

            let transform = taker.world_state.robot.ground_to_field.unwrap();
            let target = transform * taker.kick_target.unwrap();
            taker.last_motion_command = MotionCommand::from_partial_motions(
                taker.body_motion.clone().unwrap(),
                taker.head_motion.unwrap(),
            );
            taker.last_motion_type = Some(MotionType::Kick);
            taker.last_motion_switch_time = taker.world_state.now;
            // GameController releases the restart on the first small ball movement.
            taker
                .world_state
                .filtered_game_controller_state
                .as_mut()
                .unwrap()
                .sub_state = None;
            taker.ball.as_mut().unwrap().position += (target - ball).normalize() * 0.1;
            taker.kick_target = None;
            taker.body_motion = None;
            tree.tick_with_trace(&mut taker);
            assert_eq!(
                taker.ball_action,
                Some(BallAction::Pass {
                    receiver: PlayerNumber::Three
                })
            );
            assert!(matches!(
                taker.body_motion,
                Some(BodyMotion::Kick {
                    strong: false,
                    soft: false,
                    ..
                })
            ));

            let mut blocked = taker.clone();
            let ball = blocked.ball.as_ref().unwrap().position;
            blocked.world_state.obstacles = vec![Obstacle::robot(
                transform.inverse() * (ball + (target - ball) * 0.5),
                0.4,
                0.4,
            )];
            select_ball_action(&mut blocked);
            assert!(!matches!(
                blocked.ball_action,
                Some(BallAction::Pass { .. })
            ));

            let mut finished = taker.clone();
            finished.last_motion_type = Some(MotionType::Walk);
            select_ball_action(&mut finished);
            assert!(!matches!(
                finished.ball_action,
                Some(BallAction::Pass { .. })
            ));

            taker.world_state.now = taker.last_sent_hsl_message_time.unwrap()
                + hsl_network_messages::PassIntent::TIMEOUT;
            taker.world_state.player_states.three.as_mut().unwrap().time = taker.world_state.now;
            select_ball_action(&mut taker);
            assert!(!matches!(taker.ball_action, Some(BallAction::Pass { .. })));
        }
    }

    #[test]
    fn restart_supporter_waits_in_pass_range_before_an_announcement() {
        for (state, ball, position) in [
            (SubState::ThrowIn, point![0.0, 3.0], point![-1.0, 1.0]),
            (SubState::GoalKick, point![-3.5, 0.0], point![-1.5, 1.0]),
        ] {
            let mut receiver = restart(state, ball);
            let taker_pose = receiver
                .world_state
                .robot
                .ground_to_field
                .unwrap()
                .as_pose();
            let pose = Pose2::new(
                position,
                (ball.y() - position.y()).atan2(ball.x() - position.x()),
            );
            receiver.world_state.robot.ground_to_field = Some(pose.as_transform());
            receiver.world_state.player_states.two = Some(TimeWrapper {
                time: receiver.world_state.now,
                inner: PlayerState {
                    pose: taker_pose,
                    ..Default::default()
                },
            });
            let tree = crate::tree::create_tree();
            assert!(!crate::kick::passing::is_pass_receiver(&mut receiver));
            tree.tick_with_trace(&mut receiver);
            assert_eq!(receiver.body_motion, Some(BodyMotion::Stand), "{state:?}");

            for case in 0..3 {
                let mut moving = receiver.clone();
                match case {
                    0 => moving.parameters.kicking.strategy.passing.maximum_distance = 0.5,
                    1 => {
                        moving
                            .world_state
                            .filtered_game_controller_state
                            .as_mut()
                            .unwrap()
                            .kicking_team = Some(Team::Opponent)
                    }
                    _ => {
                        moving
                            .world_state
                            .filtered_game_controller_state
                            .as_mut()
                            .unwrap()
                            .sub_state = Some(SubState::PenaltyKick)
                    }
                }
                moving.body_motion = None;
                tree.tick_with_trace(&mut moving);
                assert!(
                    matches!(moving.body_motion, Some(BodyMotion::Walk { .. })),
                    "{state:?}, case {case}"
                );
            }

            let mut taker = restart(state, ball);
            taker.world_state.robot.player_number = PlayerNumber::Two;
            taker.world_state.player_states.three = Some(TimeWrapper {
                time: receiver.world_state.now,
                inner: PlayerState {
                    pose,
                    ..Default::default()
                },
            });
            // A held receiver's older report agrees with its current obstacle position.
            taker.world_state.now = taker.world_state.now + Duration::from_millis(300);
            taker.world_state.obstacles = vec![Obstacle::robot(
                taker_pose.as_transform().inverse() * position,
                0.2,
                0.2,
            )];
            tree.tick_with_trace(&mut taker);
            assert_eq!(
                taker.ball_action,
                Some(BallAction::Pass {
                    receiver: PlayerNumber::Three
                })
            );
            assert!(taker.try_sending_state_message().is_some());
            taker.world_state.now = taker.world_state.now + Duration::from_millis(150);
            taker.kick_target = None;
            taker.body_motion = None;
            tree.tick_with_trace(&mut taker);
            assert!(matches!(
                taker.body_motion,
                Some(BodyMotion::Kick {
                    strong: false,
                    soft: false,
                    ..
                })
            ));
        }
    }

    #[test]
    fn restart_boundary_roundoff_is_accepted_but_real_outside_ball_stops() {
        let mut board = restart(SubState::CornerKick, point![7.000_000_5, 4.500_000_5]);
        board.field_dimensions.length = 14.0;
        board.field_dimensions.width = 9.0;
        board.world_state.player_states.two = Some(TimeWrapper {
            time: board.world_state.now,
            inner: PlayerState {
                pose: Pose2::new(point![4.0, 2.0], 0.0),
                ..Default::default()
            },
        });
        assert_eq!(select_ball_action(&mut board), Status::Success);
        execute_ball_action(&mut board);
        assert_eq!(
            board.ball_action,
            Some(BallAction::Pass {
                receiver: PlayerNumber::Two
            })
        );
        assert!(board.try_sending_state_message().is_some());
        board.world_state.now = board.world_state.now + Duration::from_millis(150);
        execute_ball_action(&mut board);
        assert!(matches!(
            board.body_motion,
            Some(BodyMotion::Kick {
                soft: false,
                strong: false,
                ..
            })
        ));

        for ball in [
            point![7.001, 4.5],
            point![7.0, 4.501],
            point![-7.001, 0.0],
            point![0.0, -4.501],
        ] {
            board.ball.as_mut().unwrap().position = ball;
            assert_eq!(select_ball_action(&mut board), Status::Failure);
            execute_ball_action(&mut board);
            assert_eq!(board.body_motion, Some(BodyMotion::Stand));
            assert_eq!(board.kick_target, None);
        }
    }
}
