use hsl_network_messages::SubState;
use types::{
    controller_input::Button, motion_command::KickPower, motion_type::MotionType,
    primary_state::PrimaryState,
};

use crate::{
    action,
    actions::{damping, injected_motion_command, prepare, remote_control, stand, stand_up},
    behavior_tree::Node,
    condition,
    conditions::{
        has_ball_position, hulks_is_kicking_team, is_ball_free, is_closest_to_ball,
        is_controller_connected, is_fallen, is_finishing_striker_kick, is_goalkeeper, is_kickoff,
        is_last_hulk_standing, is_primary_state, is_ready_for_stand_up, is_remote_control_enabled,
        is_remote_kick_mode, is_simple, requires_damping,
        should_yield_goalkeeper_clearance,
    },
    goalkeeper::goalkeeper_subtree,
    head::{look_around, look_at_ball_subtree, look_straight_ahead, search_for_lost_ball_subtree},
    kick::{
        execution::execute_ball_action,
        kick,
        kickoff::kickoff_subtree,
        passing::{
            is_pass_ball_arriving, is_pass_receiver, prepare_for_restart_pass,
            prepare_to_receive_pass, yield_goalkeeper_clearance,
        },
        reception::receive_rolling_ball,
        set_kick_target_beyond_ball,
        strategy::select_ball_action,
        use_kick_power,
    },
    negation,
    node::Blackboard,
    penalty_shootout::{is_penalty_shootout, penalty_shootout_subtree},
    search::leuchtturm,
    selection, sequence,
    substates::{is_in_sub_state, is_sub_state, sub_state_subtree},
    subtree,
    switch_motion_type::switch_motion_type,
    voronoi::{calculate_voronoi_grid, calculate_voronoi_grid_without_goalkeeper},
    walk::{
        block_opponent_with_ball, walk_alternatives_subtree, walk_to_kickoff_pose,
        walk_to_voronoi_position,
    },
};

pub fn create_tree() -> Node<Blackboard> {
    selection!(
        sequence!(
            condition!(is_primary_state, PrimaryState::Damping),
            action!(damping)
        ),
        sequence!(
            condition!(is_primary_state, PrimaryState::Prepare),
            switch_motion_type(
                MotionType::Prepare,
                action!(prepare),
                sequence!(action!(look_straight_ahead), action!(stand))
            )
        ),
        sequence!(condition!(requires_damping), action!(damping)),
        sequence!(
            condition!(is_primary_state, PrimaryState::Stop),
            action!(stand)
        ),
        subtree!(remote_control_subtree),
        action!(injected_motion_command),
        sequence!(
            condition!(is_primary_state, PrimaryState::Finished),
            action!(stand)
        ),
        sequence!(
            selection!(
                condition!(is_primary_state, PrimaryState::Finished),
                condition!(is_primary_state, PrimaryState::Penalized),
            ),
            action!(stand)
        ),
        sequence!(
            condition!(is_primary_state, PrimaryState::Initial),
            action!(look_around),
            action!(stand)
        ),
        sequence!(condition!(is_ready_for_stand_up), action!(stand_up)),
        sequence!(condition!(is_fallen), action!(prepare)),
        sequence!(
            condition!(is_primary_state, PrimaryState::Set),
            sequence!(subtree!(look_at_ball_subtree), action!(stand))
        ),
        sequence!(
            condition!(is_primary_state, PrimaryState::Ready),
            subtree!(ready_subtree)
        ),
        sequence!(
            condition!(is_primary_state, PrimaryState::Playing),
            subtree!(playing_subtree)
        ),
        Node::Failure
    )
}

fn ready_subtree() -> Node<Blackboard> {
    sequence!(action!(walk_to_kickoff_pose), action!(look_around),)
}

fn playing_subtree() -> Node<Blackboard> {
    selection!(
        sequence!(
            condition!(is_kickoff),
            negation!(condition!(is_ball_free)),
            negation!(condition!(hulks_is_kicking_team)),
            subtree!(look_at_ball_subtree),
            action!(stand)
        ),
        sequence!(
            condition!(is_penalty_shootout),
            subtree!(penalty_shootout_subtree)
        ),
        subtree!(kickoff_subtree),
        sequence!(
            condition!(should_yield_goalkeeper_clearance),
            subtree!(look_at_ball_subtree),
            switch_motion_type(
                MotionType::Walk,
                action!(yield_goalkeeper_clearance),
                action!(stand),
            ),
        ),
        sequence!(
            selection!(condition!(is_simple), condition!(is_last_hulk_standing)),
            selection!(
                sequence!(
                    negation!(condition!(has_ball_position)),
                    subtree!(search_subtree)
                ),
                subtree!(striker_subtree)
            )
        ),
        sequence!(condition!(is_goalkeeper), subtree!(goalkeeper_subtree)),
        sequence!(
            negation!(condition!(is_in_sub_state)),
            condition!(is_finishing_striker_kick),
            subtree!(striker_subtree),
        ),
        sequence!(
            selection!(
                negation!(condition!(is_in_sub_state)),
                sequence!(
                    condition!(hulks_is_kicking_team),
                    negation!(condition!(is_sub_state, SubState::PenaltyKick)),
                ),
            ),
            condition!(is_pass_receiver),
            subtree!(look_at_ball_subtree),
            selection!(
                sequence!(condition!(is_pass_ball_arriving), subtree!(striker_subtree)),
                switch_motion_type(
                    MotionType::Walk,
                    action!(prepare_to_receive_pass),
                    action!(stand)
                ),
            ),
        ),
        sequence!(
            negation!(condition!(has_ball_position)),
            subtree!(search_subtree)
        ),
        sequence!(
            selection!(
                sequence!(
                    condition!(hulks_is_kicking_team),
                    condition!(is_in_sub_state),
                    negation!(condition!(is_sub_state, SubState::GoalKick)),
                    action!(calculate_voronoi_grid_without_goalkeeper),
                ),
                action!(calculate_voronoi_grid),
            ),
            condition!(is_closest_to_ball),
            subtree!(striker_subtree),
        ),
        subtree!(supporter_subtree),
    )
}

pub fn search_subtree() -> Node<Blackboard> {
    sequence!(
        subtree!(search_for_lost_ball_subtree),
        switch_motion_type(
            MotionType::Walk,
            // selection!(
            // condition!(has_suggested_search_position),
            // action!(walk_to_search_position),
            action!(leuchtturm),
            // ),
            subtree!(walk_alternatives_subtree),
        )
    )
}

pub(crate) fn striker_subtree() -> Node<Blackboard> {
    sequence!(
        subtree!(look_at_ball_subtree),
        selection!(
            sequence!(condition!(is_in_sub_state), subtree!(sub_state_subtree),),
            sequence!(
                negation!(condition!(is_in_sub_state)),
                selection!(
                    action!(receive_rolling_ball),
                    action!(block_opponent_with_ball),
                    sequence!(action!(select_ball_action), action!(execute_ball_action)),
                    action!(stand),
                )
            )
        )
    )
}

fn supporter_subtree() -> Node<Blackboard> {
    sequence!(
        subtree!(look_at_ball_subtree),
        selection!(
            action!(prepare_for_restart_pass),
            action!(walk_to_voronoi_position),
            action!(stand)
        ),
    )
}

fn remote_control_subtree() -> Node<Blackboard> {
    sequence!(
        condition!(is_remote_control_enabled),
        selection!(
            sequence!(condition!(is_ready_for_stand_up), action!(stand_up)),
            sequence!(condition!(is_fallen), action!(prepare)),
            sequence!(
                condition!(is_controller_connected),
                selection!(
                    subtree!(
                        remote_kick_subtree,
                        Button::LeftTrigger,
                        KickPower::Rumpelstilzchen
                    ),
                    subtree!(
                        remote_kick_subtree,
                        Button::RightTrigger,
                        KickPower::Schlong
                    ),
                    action!(remote_control)
                )
            ),
            action!(stand)
        )
    )
}

fn remote_kick_subtree(button: Button, kick_power: KickPower) -> Node<Blackboard> {
    sequence!(
        condition!(is_remote_kick_mode, button),
        subtree!(look_at_ball_subtree),
        action!(kick),
        action!(set_kick_target_beyond_ball),
        action!(use_kick_power, kick_power),
    )
}

#[cfg(test)]
mod clearance_tests {
    use std::time::Duration;

    use geometry::circle::Circle;
    use hsl_network_messages::{PassIntent, Penalty, PlayerNumber, Team};
    use linear_algebra::{Pose2, point, vector};
    use types::{
        behavior_tree::{NodeTrace, Status},
        fall_detection::{FallDetection, Posture},
        field_dimensions::{FieldDimensions, Side},
        filtered_game_controller_state::FilteredGameControllerState,
        filtered_game_state::FilteredGameState,
        motion_command::BodyMotion,
        obstacles::Obstacle,
        parameters::{KickStrategyParameters, PassingParameters},
        rule_obstacles::RuleObstacle,
        time_wrapper::TimeWrapper,
        walking_velocity_limits::WalkingVelocityLimits,
        world_state::{BallSource, PlayerState},
    };

    use super::*;
    use crate::{kick::strategy::BallAction, node::LastBall};

    fn three_players() -> Blackboard {
        let mut board = Blackboard {
            field_dimensions: FieldDimensions {
                length: 14.0,
                width: 9.0,
                penalty_area_length: 3.0,
                penalty_area_width: 6.0,
                ball_radius: 0.105,
                ..FieldDimensions::SPL_2025
            },
            last_motion_type: Some(MotionType::Walk),
            last_closest_to_ball: true,
            walking_velocity_limits: WalkingVelocityLimits {
                forward_velocity_limits: [-1.0, 1.0],
                lateral_velocity_limit: 0.6,
                angular_velocity_limit: 2.0,
            },
            ..Default::default()
        };
        board.world_state.now = board.world_state.now + Duration::from_secs(10);
        board.world_state.robot.primary_state = PrimaryState::Playing;
        board.world_state.robot.player_number = PlayerNumber::Two;
        board.world_state.robot.ground_to_field =
            Some(Pose2::new(point![-3.6, 0.0], 0.0).as_transform());
        board.world_state.filtered_game_controller_state = Some(FilteredGameControllerState {
            game_state: FilteredGameState::Playing {
                ball_is_free: true,
                kick_off: false,
            },
            kicking_team: Some(Team::Hulks),
            ..Default::default()
        });
        board.world_state.fall_detection = Some(FallDetection {
            time: board.world_state.now,
            valid_until: board.world_state.now + Duration::from_secs(10),
            posture: Posture::Upright,
        });
        for (number, position) in [
            (PlayerNumber::One, point![-6.6, 0.0]),
            (PlayerNumber::Two, point![-3.6, 0.0]),
            (PlayerNumber::Three, point![-1.5, -2.0]),
        ] {
            board.world_state.player_states[number] = Some(TimeWrapper {
                time: board.world_state.now,
                inner: PlayerState {
                    pose: Pose2::new(position, 0.0),
                    ..Default::default()
                },
            });
        }
        board.ball = Some(LastBall {
            position: point![-5.5, 0.0],
            velocity: vector![3.4, 0.0],
            age: board.world_state.now,
            field_side: Side::Left,
            source: BallSource::Own,
        });
        board.parameters.goalkeeper.player_number = PlayerNumber::One;
        board
            .parameters
            .goalkeeper
            .kick_away_ball_maximum_robot_distance = 1.5;
        board.parameters.ball.last_ball_timeout = Duration::from_secs(1);
        board.parameters.ball.pursuit_lookahead = Duration::from_millis(500);
        board.parameters.ball.interception.minimum_ball_velocity = 0.3;
        board
            .parameters
            .ball
            .interception
            .minimum_ball_velocity_towards_own_half = 0.7;
        board.parameters.ball.closest_to_ball.exit_duration = Duration::from_millis(150);
        board.parameters.voronoi.grid_resolution = 0.25;
        board.parameters.voronoi.padding = 0.5;
        board.parameters.walking.path_planning.robot_radius = 0.4;
        board.parameters.walking.speed.kicking = 1.8;
        board.parameters.walking.speed.blocking = 0.8;
        board.parameters.walking.speed.velocity_fade_distance = 3.0;
        board
            .parameters
            .walking
            .walk_and_stand
            .orientation_tolerance = 0.05;
        board.parameters.kicking.target_speed = 3.4;
        board.parameters.kicking.kick_activation_distance = 1.3;
        board.parameters.kicking.approach_ball_standoff = 0.3;
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

    fn succeeded(trace: &NodeTrace, name: &str) -> bool {
        (trace.name == name && trace.status == Status::Success)
            || trace.children.iter().any(|child| succeeded(child, name))
    }

    #[test]
    fn three_players_clear_then_yield_even_after_projected_possession_claim() {
        let tree = create_tree();
        let mut keeper = three_players();
        for y in [-1e-7, 1e-7] {
            assert!(
                crate::goalkeeper::goalkeeper_clearance_target(
                    point![-6.0, y],
                    &keeper.field_dimensions
                )
                .y() > 0.0
            );
        }
        assert!(
            crate::goalkeeper::goalkeeper_clearance_target(
                point![-6.0, -0.5],
                &keeper.field_dimensions
            )
            .y() < 0.0
        );
        keeper.world_state.robot.player_number = PlayerNumber::One;
        let keeper_transform = keeper
            .world_state
            .player_states
            .one
            .as_ref()
            .unwrap()
            .inner
            .pose
            .as_transform();
        keeper.world_state.robot.ground_to_field = Some(keeper_transform);
        keeper.ball.as_mut().unwrap().position = point![-6.0, 0.0];
        keeper.ball.as_mut().unwrap().velocity = vector![0.0, 0.0];
        keeper.world_state.obstacles = vec![Obstacle::robot(
            keeper_transform.inverse() * point![-4.0, 1.0],
            0.2,
            0.2,
        )];
        tree.tick_with_trace(&mut keeper);
        assert!(matches!(
            keeper.body_motion,
            Some(BodyMotion::Kick {
                strong: true,
                soft: false,
                ..
            })
        ));

        // Prepare before launch, while the keeper winds up and support could walk into its lane.
        for number in [PlayerNumber::Two, PlayerNumber::Three] {
            let mut preparing = three_players();
            preparing.ball.as_mut().unwrap().position = point![-6.0, 0.0];
            preparing.ball.as_mut().unwrap().velocity = vector![0.0, 0.0];
            preparing
                .world_state
                .player_states
                .two
                .as_mut()
                .unwrap()
                .inner
                .pose = Pose2::new(point![-3.8, 0.825], 0.0);
            let transform = preparing.world_state.player_states[number]
                .as_ref()
                .unwrap()
                .inner
                .pose
                .as_transform();
            preparing.world_state.robot.player_number = number;
            preparing.world_state.robot.ground_to_field = Some(transform);
            preparing.world_state.obstacles = vec![Obstacle::robot(
                transform.inverse() * point![-2.5, 2.5],
                0.2,
                0.2,
            )];
            let (_, trace) = tree.tick_with_trace(&mut preparing);
            assert!(succeeded(&trace, "yield_goalkeeper_clearance"));
            assert!(preparing.ball_action.is_none());
            assert!(preparing.kick_target.is_none());
            let Some(BodyMotion::WalkWithVelocity { velocity, .. }) = preparing.body_motion else {
                panic!("field players must prepare the clearance lane before launch");
            };
            if number == PlayerNumber::Two {
                assert!(velocity.norm() > 0.0);
                let direction = vector![6.0, 2.25].normalize();
                assert!((transform * velocity).dot(&direction).abs() < 1e-6);
                for case in 0..3 {
                    let mut ordinary = preparing.clone();
                    match case {
                        0 => ordinary.world_state.obstacles.clear(),
                        1 => {
                            ordinary
                                .world_state
                                .player_states
                                .three
                                .as_mut()
                                .unwrap()
                                .inner
                                .pose = Pose2::new(point![-5.7, 0.1], 0.0)
                        }
                        _ => {
                            ordinary.ball.as_mut().unwrap().position = point![-4.0, 0.0];
                            ordinary.ball.as_mut().unwrap().velocity = vector![0.5, 0.0];
                        }
                    }
                    let (_, trace) = tree.tick_with_trace(&mut ordinary);
                    assert!(
                        !succeeded(&trace, "yield_goalkeeper_clearance"),
                        "prelaunch case {case}"
                    );
                }
            } else {
                assert_eq!(velocity.norm(), 0.0);
            }
        }

        let mut supporter = three_players();
        calculate_voronoi_grid(&mut supporter);
        assert!(is_closest_to_ball(&mut supporter));
        supporter.ball_action = Some(BallAction::Dribble);
        supporter.kick_target = Some(point![1.0, 0.0]);
        let (_, trace) = tree.tick_with_trace(&mut supporter);
        assert!(succeeded(&trace, "yield_goalkeeper_clearance"));
        let Some(BodyMotion::WalkWithVelocity { velocity, .. }) = supporter.body_motion else {
            panic!("supporter must leave clearance lane");
        };
        assert!(velocity.y() >= 0.6);
        assert!(velocity.x().abs() < 1e-6);
        assert!(supporter.ball_action.is_none());
        assert!(supporter.kick_target.is_none());
        assert!(!supporter.last_closest_to_ball);

        // A real strong clearance slows below half its requested speed before passing support.
        supporter.ball.as_mut().unwrap().velocity = vector![1.1, 0.0];
        let (_, trace) = tree.tick_with_trace(&mut supporter);
        assert!(succeeded(&trace, "yield_goalkeeper_clearance"));

        // The third robot is already clear and must not chase the projected ball.
        let mut third = three_players();
        third.world_state.robot.player_number = PlayerNumber::Three;
        third.world_state.robot.ground_to_field = Some(
            third
                .world_state
                .player_states
                .three
                .as_ref()
                .unwrap()
                .inner
                .pose
                .as_transform(),
        );
        let (_, trace) = tree.tick_with_trace(&mut third);
        assert!(succeeded(&trace, "yield_goalkeeper_clearance"));
        assert!(
            matches!(third.body_motion, Some(BodyMotion::WalkWithVelocity { velocity, .. }) if velocity.norm() == 0.0)
        );

        supporter.ball.as_mut().unwrap().position = point![-3.0, 0.0];
        let (_, trace) = tree.tick_with_trace(&mut supporter);
        assert!(!succeeded(&trace, "yield_goalkeeper_clearance"));
    }

    #[test]
    fn clearance_yield_preserves_pass_reception_and_incoming_shot_defense() {
        let tree = create_tree();
        let mut receiver = three_players();
        receiver
            .world_state
            .player_states
            .one
            .as_mut()
            .unwrap()
            .inner
            .pass = Some(PassIntent {
            receiver: PlayerNumber::Two,
            target: point![-4.1, 0.0],
        });
        let (_, trace) = tree.tick_with_trace(&mut receiver);
        assert!(!succeeded(&trace, "yield_goalkeeper_clearance"));
        assert!(succeeded(&trace, "is_pass_receiver"));
        assert!(succeeded(&trace, "receive_rolling_ball"));

        let mut defender = three_players();
        defender.ball.as_mut().unwrap().position = point![-1.5, 0.0];
        defender.ball.as_mut().unwrap().velocity = vector![-3.4, 0.0];
        let (_, trace) = tree.tick_with_trace(&mut defender);
        assert!(!succeeded(&trace, "yield_goalkeeper_clearance"));
        assert!(succeeded(&trace, "receive_rolling_ball"));
    }

    #[test]
    fn clearance_yield_releases_for_uncertain_origin_or_changed_ball() {
        let tree = create_tree();
        for case in 0..13 {
            let mut board = three_players();
            match case {
                0 => board.ball.as_mut().unwrap().velocity = vector![0.5, 0.0],
                1 => board.ball.as_mut().unwrap().velocity = vector![0.0, 3.4],
                2 => board.ball.as_mut().unwrap().velocity = vector![-3.4, 0.0],
                3 => board.ball.as_mut().unwrap().age = Default::default(),
                4 => {
                    board.world_state.player_states.one.as_mut().unwrap().time = Default::default()
                }
                5 => {
                    board.world_state.player_states.one.as_mut().unwrap().time =
                        board.world_state.now + Duration::from_secs(1)
                }
                6 => {
                    board
                        .world_state
                        .player_states
                        .one
                        .as_mut()
                        .unwrap()
                        .inner
                        .pose = Pose2::new(point![-3.0, 0.0], 0.0)
                }
                7 => {
                    board
                        .world_state
                        .player_states
                        .one
                        .as_mut()
                        .unwrap()
                        .inner
                        .pose = Pose2::new(point![-6.6, 1.0], 0.0)
                }
                8 => {
                    board
                        .world_state
                        .filtered_game_controller_state
                        .as_mut()
                        .unwrap()
                        .penalties
                        .one = Some(Penalty::PickUp {
                        remaining: Duration::from_secs(30),
                    })
                }
                9 => board.last_motion_type = Some(MotionType::Kick),
                10 => {
                    board
                        .world_state
                        .filtered_game_controller_state
                        .as_mut()
                        .unwrap()
                        .sub_state = Some(SubState::GoalKick)
                }
                11 => board.ball.as_mut().unwrap().position = point![0.1, 0.0],
                _ => {
                    board.last_motion_type = Some(MotionType::StandUp);
                    board.last_motion_switch_time = board.world_state.now;
                    board.parameters.control.allow_switch.stand_up = Duration::from_secs(1);
                }
            }
            let (_, trace) = tree.tick_with_trace(&mut board);
            assert!(
                !succeeded(&trace, "yield_goalkeeper_clearance"),
                "case {case}"
            );
            if case == 12 {
                assert!(succeeded(&trace, "should_yield_goalkeeper_clearance"));
                assert_eq!(board.body_motion, Some(BodyMotion::Stand));
            }
        }
    }

    #[test]
    fn clearance_step_respects_teammate_bodies_rules_and_current_side() {
        let tree = create_tree();
        let mut rotated = three_players();
        rotated.walking_velocity_limits.lateral_velocity_limit = 1.5;
        let transform = Pose2::new(point![-3.6, 0.0], std::f32::consts::FRAC_PI_4).as_transform();
        rotated.world_state.robot.ground_to_field = Some(transform);
        tree.tick_with_trace(&mut rotated);
        let Some(BodyMotion::WalkWithVelocity { velocity, .. }) = rotated.body_motion else {
            panic!("rotated supporter must leave clearance lane");
        };
        let field_velocity = transform * velocity;
        assert!(field_velocity.x().abs() < 1e-6);
        assert!(field_velocity.y() > 0.0);
        assert!(velocity.x() <= rotated.walking_velocity_limits.forward_velocity_limits[1]);
        assert!(velocity.y().abs() <= 1.0);

        let mut board = three_players();
        board
            .world_state
            .player_states
            .three
            .as_mut()
            .unwrap()
            .inner
            .pose = Pose2::new(point![-3.6, 0.9], 0.0);
        tree.tick_with_trace(&mut board);
        assert!(
            matches!(board.body_motion, Some(BodyMotion::WalkWithVelocity { velocity, .. }) if velocity.y() < 0.0)
        );

        board.world_state.rule_obstacles =
            vec![RuleObstacle::Circle(Circle::new(point![-3.6, -0.8], 0.3))];
        tree.tick_with_trace(&mut board);
        assert!(
            matches!(board.body_motion, Some(BodyMotion::WalkWithVelocity { velocity, .. }) if velocity.norm() == 0.0)
        );

        // Already above the line: a blocked exit must not send us through the ball lane.
        board.world_state.rule_obstacles.clear();
        board.world_state.robot.ground_to_field =
            Some(Pose2::new(point![-3.6, 0.1], 0.0).as_transform());
        tree.tick_with_trace(&mut board);
        assert!(
            matches!(board.body_motion, Some(BodyMotion::WalkWithVelocity { velocity, .. }) if velocity.norm() == 0.0)
        );
    }
}
