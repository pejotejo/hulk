use std::time::Duration;

use behavior_node::{
    goalkeeper::walk_to_goalkeeper_default_position,
    node::{Blackboard, LastBall},
    tree::create_tree,
    walk::walk_to_voronoi_position,
};
use coordinate_systems::Field;
use hsl_network_messages::{PlayerNumber, SubState, Team};
use linear_algebra::{Point2, Pose2, point};
use ros_z::time::Time;
use types::{
    behavior_tree::{NodeTrace, Status},
    fall_detection::{FallDetection, Posture},
    field_dimensions::{FieldDimensions, Side},
    filtered_game_controller_state::FilteredGameControllerState,
    filtered_game_state::FilteredGameState,
    motion_command::{BodyMotion, OrientationMode},
    parameters::BehaviorParameters,
    players::Players,
    primary_state::PrimaryState,
    world_state::{BallSource, BallState, PlayerState, RobotState, WorldState},
};
use voronoi::{Ownership, VoronoiGrid};

fn blackboard(player: PlayerNumber, ball: Point2<Field>) -> Blackboard {
    let mut parameters = BehaviorParameters::default();
    parameters.ball.closest_to_ball.enter_duration = Duration::from_millis(300);
    parameters.ball.closest_to_ball.exit_duration = Duration::from_millis(150);
    parameters.kicking.kick_activation_distance = 1.3;
    parameters.kicking.kick_activation_hysteresis = 0.3;
    parameters.kicking.approach_ball_standoff = 0.3;
    parameters.goalkeeper.player_number = PlayerNumber::One;
    parameters.goalkeeper.x_offset = 0.3;
    parameters.goalkeeper.passive_distance = 4.5;
    parameters.goalkeeper.striker_distance = 3.5;
    parameters.goalkeeper.kick_away_ball_maximum_robot_distance = 1.5;
    parameters.goalkeeper.active_defense_maximum_robot_distance = 2.0;
    parameters.goalkeeper.distance_to_goalpost = 0.15;
    parameters.voronoi.grid_resolution = 0.2;
    parameters.voronoi.padding = 0.5;
    parameters.walking.path_planning.robot_radius = 0.4;
    parameters.walking.walk_and_stand.target_reached_thresholds = nalgebra::Vector2::new(0.05, 0.1);

    let player_states = Players {
        one: Some(PlayerState {
            pose: Pose2::new(point![-4.2, 0.0], 0.0),
            ..Default::default()
        }),
        two: Some(PlayerState {
            pose: Pose2::new(point![2.8, -2.0], 0.0),
            ..Default::default()
        }),
        three: Some(PlayerState {
            pose: Pose2::new(point![3.2, 0.0], 0.0),
            ..Default::default()
        }),
        four: Some(PlayerState {
            pose: Pose2::new(point![2.8, 2.0], 0.0),
            ..Default::default()
        }),
        five: Some(PlayerState {
            pose: Pose2::new(point![4.0, 0.0], 0.0),
            ..Default::default()
        }),
    };
    let ground_to_field = player_states[player].unwrap().pose.as_transform();
    let now = Time::from_nanos(1_000_000_000);
    let last_ball = LastBall {
        position: ball,
        velocity: Default::default(),
        age: now,
        field_side: Side::Left,
        source: BallSource::Own,
    };
    Blackboard {
        field_dimensions: FieldDimensions::SPL_2025,
        head_yaw: 0.0,
        parameters,
        walking_velocity_limits: Default::default(),
        world_state: WorldState {
            robot: RobotState {
                player_number: player,
                primary_state: PrimaryState::Playing,
                ground_to_field: Some(ground_to_field),
            },
            ball: Some(BallState {
                ball_in_field: ball,
                ball_in_ground: ground_to_field.inverse() * ball,
                ..Default::default()
            }),
            player_states,
            now,
            fall_detection: Some(FallDetection {
                time: now,
                valid_until: now + Duration::from_secs(10),
                posture: Posture::Upright,
            }),
            filtered_game_controller_state: Some(FilteredGameControllerState {
                game_state: FilteredGameState::Playing {
                    ball_is_free: true,
                    kick_off: false,
                },
                ..Default::default()
            }),
            ..Default::default()
        },
        controller_input: None,
        remote_control_enabled: false,
        path_obstacles_output: Vec::new(),
        time_since_last_switch: Duration::ZERO,
        direction_difference: 0.0,
        voronoi_inputs: Vec::new(),
        ball: Some(last_ball.clone()),
        visual_kick_ball_position: None,
        last_ball: Some(last_ball),
        last_close_enough_to_kick: false,
        kick_target: None,
        last_kick_target: None,
        last_motion_command: Default::default(),
        last_motion_switch_time: Time::zero(),
        last_motion_type: None,
        last_sent_game_controller_return_message_time: None,
        last_sent_hsl_message_time: None,
        last_closest_to_ball: false,
        closest_to_ball_entered_area_since: None,
        closest_to_ball_left_area_since: None,
        is_injected_motion_command: false,
        walk_position: None,
        body_motion: None,
        head_motion: None,
        voronoi_map: None,
        ..Default::default()
    }
}

fn tick_after_ownership_hysteresis(blackboard: &mut Blackboard) -> NodeTrace {
    let tree = create_tree();
    assert_eq!(tree.tick_with_trace(blackboard).0, Status::Success);
    blackboard.world_state.now = blackboard.world_state.now + Duration::from_millis(320);
    blackboard.body_motion = None;
    blackboard.head_motion = None;
    blackboard.voronoi_inputs.clear();
    blackboard.voronoi_map = None;
    let (status, trace) = tree.tick_with_trace(blackboard);
    assert_eq!(status, Status::Success);
    trace
}

fn succeeded(trace: &NodeTrace, name: &str) -> bool {
    trace.name == name && trace.status == Status::Success
        || trace.children.iter().any(|child| succeeded(child, name))
}

#[test]
fn field_players_pursue_balls_outside_the_goalkeeper_striker_region() {
    // The keeper keeps its support region even when a field player must pursue
    // the ball, including at the keeper's strict striker boundary.
    for (ball_x, keeper_is_striker) in [(-2.0, true), (-1.01, true), (-1.0, false), (-0.8, false)] {
        let ball = point![ball_x, 0.0];
        let mut field_player = blackboard(PlayerNumber::Three, ball);
        let trace = tick_after_ownership_hysteresis(&mut field_player);
        assert_eq!(
            field_player
                .voronoi_map
                .as_ref()
                .unwrap()
                .ownership_at(ball),
            Some(Ownership::Robot(PlayerNumber::One)),
            "ball x = {ball_x}"
        );
        assert!(
            field_player
                .voronoi_inputs
                .contains(&field_player.world_state.player_states.one.unwrap().pose)
        );
        assert_eq!(
            succeeded(&trace, "subtree_striker_subtree"),
            !keeper_is_striker,
            "field player, ball x = {ball_x}"
        );

        let mut goalkeeper = blackboard(PlayerNumber::One, ball);
        let trace = tick_after_ownership_hysteresis(&mut goalkeeper);
        assert_eq!(
            succeeded(&trace, "subtree_striker_subtree"),
            keeper_is_striker,
            "goalkeeper, ball x = {ball_x}"
        );
    }
}

#[test]
fn restart_ownership_matches_goalkeeper_restart_behavior() {
    for (team, sub_state, keeper_is_striker) in [
        (Team::Hulks, SubState::GoalKick, true),
        (Team::Hulks, SubState::ThrowIn, false),
        (Team::Hulks, SubState::DirectFreeKick, false),
        (Team::Opponent, SubState::GoalKick, false),
        (Team::Opponent, SubState::DirectFreeKick, false),
    ] {
        for player in [PlayerNumber::One, PlayerNumber::Three] {
            let mut blackboard = blackboard(player, point![-2.0, 0.0]);
            let state = blackboard
                .world_state
                .filtered_game_controller_state
                .as_mut()
                .unwrap();
            state.kicking_team = Some(team);
            state.sub_state = Some(sub_state);
            let trace = tick_after_ownership_hysteresis(&mut blackboard);
            assert_eq!(
                succeeded(&trace, "subtree_striker_subtree"),
                keeper_is_striker == (player == PlayerNumber::One),
                "{team:?} {sub_state:?}, {player:?}"
            );
            if player == PlayerNumber::Three {
                let support_owner = if team == Team::Hulks && sub_state != SubState::GoalKick {
                    PlayerNumber::Three
                } else {
                    PlayerNumber::One
                };
                assert_eq!(
                    blackboard
                        .voronoi_map
                        .as_ref()
                        .unwrap()
                        .ownership_at(point![-2.0, 0.0]),
                    Some(Ownership::Robot(support_owner)),
                    "support map for {team:?} {sub_state:?}"
                );
            }
        }
    }
}

#[test]
fn field_players_take_over_when_no_goalkeeper_state_is_available() {
    for restart in [None, Some(SubState::GoalKick)] {
        let mut blackboard = blackboard(PlayerNumber::Three, point![-2.0, 0.0]);
        blackboard.world_state.player_states.one = None;
        if let Some(sub_state) = restart {
            let state = blackboard
                .world_state
                .filtered_game_controller_state
                .as_mut()
                .unwrap();
            state.kicking_team = Some(Team::Hulks);
            state.sub_state = Some(sub_state);
        }
        let trace = tick_after_ownership_hysteresis(&mut blackboard);
        assert!(succeeded(&trace, "subtree_striker_subtree"));
        assert_eq!(
            blackboard
                .voronoi_map
                .as_ref()
                .unwrap()
                .ownership_at(point![-2.0, 0.0]),
            Some(Ownership::Robot(PlayerNumber::Three))
        );
    }
}

#[test]
fn blocked_orphan_ball_is_assigned_without_changing_the_support_map() {
    use types::obstacles::Obstacle;

    let ball = point![-0.8, 0.0];
    let mut blackboard = blackboard(PlayerNumber::Three, ball);
    let ground_to_field = blackboard.world_state.robot.ground_to_field.unwrap();
    blackboard.world_state.obstacles = vec![Obstacle::robot(
        ground_to_field.inverse() * ball,
        0.25,
        0.25,
    )];
    let trace = tick_after_ownership_hysteresis(&mut blackboard);
    assert!(succeeded(&trace, "subtree_striker_subtree"));
    let map = blackboard.voronoi_map.as_ref().unwrap();
    assert_eq!(map.ownership_at(ball), Some(Ownership::Blocked));
    assert_eq!(
        map.nearest_non_blocked_ownership(ball),
        Some(Ownership::Robot(PlayerNumber::One))
    );
    assert!(
        blackboard
            .voronoi_inputs
            .contains(&blackboard.world_state.player_states.one.unwrap().pose)
    );
}

#[test]
fn orphan_ball_election_preserves_obstacle_aware_travel_costs() {
    use geometry::rectangle::Rectangle;
    use types::rule_obstacles::RuleObstacle;

    let ball = point![-0.8, 0.0];
    let mut blackboard = blackboard(PlayerNumber::Two, ball);
    // Three is closer in a straight line, but Two can go around the barrier
    // sooner. The temporary ownership map must use the same blocked cells.
    blackboard.world_state.rule_obstacles = vec![RuleObstacle::Rectangle(Rectangle {
        min: point![0.0, -1.0],
        max: point![2.0, 3.0],
    })];
    let trace = tick_after_ownership_hysteresis(&mut blackboard);
    assert!(succeeded(&trace, "subtree_striker_subtree"));
    assert_eq!(
        blackboard.voronoi_map.as_ref().unwrap().ownership_at(ball),
        Some(Ownership::Robot(PlayerNumber::One))
    );
}

#[test]
fn retreating_goalkeeper_looks_at_cached_own_half_ball_in_ground_coordinates() {
    let mut blackboard = blackboard(PlayerNumber::One, point![-1.5, 1.0]);
    let ground_to_field = Pose2::new(point![-3.0, 0.5], 0.6).as_transform();
    blackboard.world_state.robot.ground_to_field = Some(ground_to_field);
    blackboard.world_state.ball = None;
    assert_eq!(
        walk_to_goalkeeper_default_position(&mut blackboard),
        Status::Success
    );
    let Some(BodyMotion::Walk {
        orientation_mode: OrientationMode::LookAt { target, .. },
        ..
    }) = blackboard.body_motion
    else {
        panic!("keeper must keep looking at the cached ball while returning home")
    };
    let expected = ground_to_field.inverse() * point![-1.5, 1.0];
    approx::assert_relative_eq!(target.x(), expected.x(), epsilon = 1e-6);
    approx::assert_relative_eq!(target.y(), expected.y(), epsilon = 1e-6);

    blackboard.ball.as_mut().unwrap().position = point![1.5, 1.0];
    assert_eq!(
        walk_to_goalkeeper_default_position(&mut blackboard),
        Status::Success
    );
    assert!(matches!(
        blackboard.body_motion,
        Some(BodyMotion::Walk {
            orientation_mode: OrientationMode::AlignWithPath,
            ..
        })
    ));
}

fn set_robot_pose(blackboard: &mut Blackboard, position: Point2<Field>) {
    let pose = Pose2::new(position, 0.6);
    blackboard.world_state.robot.ground_to_field = Some(pose.as_transform());
    blackboard.world_state.player_states[blackboard.world_state.robot.player_number] =
        Some(PlayerState {
            pose,
            ..Default::default()
        });
    if let Some(ball) = &mut blackboard.world_state.ball {
        ball.ball_in_ground = pose.as_transform().inverse() * ball.ball_in_field;
    }
}

fn walk_target_in_field(blackboard: &Blackboard) -> Point2<Field> {
    let Some(BodyMotion::Walk {
        path,
        target_orientation,
        ..
    }) = &blackboard.body_motion
    else {
        panic!("supporter must walk back onto the field")
    };
    assert!(target_orientation.angle().is_finite());
    let types::path::PathSegment::LineSegment(last) = path.last_segment() else {
        panic!("expected a final straight path segment")
    };
    blackboard.world_state.robot.ground_to_field.unwrap() * last.1
}

#[test]
fn stranded_supporters_reenter_when_all_field_player_sites_are_outside_the_grid() {
    for position in [
        point![-1.64, -3.55],
        point![0.32, 3.94],
        point![-5.3, 0.0],
        point![5.3, 0.0],
        point![5.3, 3.8],
    ] {
        // At an own throw-in the keeper is correctly excluded, leaving no
        // Voronoi owners if all three field players have crossed the grid edge.
        let mut blackboard = blackboard(PlayerNumber::Two, point![-1.6, -3.0]);
        let game = blackboard
            .world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap();
        game.kicking_team = Some(Team::Hulks);
        game.sub_state = Some(SubState::ThrowIn);
        blackboard
            .world_state
            .player_states
            .three
            .as_mut()
            .unwrap()
            .pose = Pose2::new(point![0.32, 3.94], 0.0);
        blackboard
            .world_state
            .player_states
            .four
            .as_mut()
            .unwrap()
            .pose = Pose2::new(point![-0.5, -3.72], 0.0);
        blackboard.world_state.player_states.five = None;
        set_robot_pose(&mut blackboard, position);

        let trace = tick_after_ownership_hysteresis(&mut blackboard);
        assert!(succeeded(&trace, "subtree_supporter_subtree"));
        assert!(
            blackboard
                .voronoi_map
                .as_ref()
                .unwrap()
                .cells()
                .all(|(_, owner)| !matches!(owner, Ownership::Robot(_)))
        );
        let target = walk_target_in_field(&blackboard);
        assert!(target.x().abs() <= 4.1 + 1e-5, "{position:?} -> {target:?}");
        assert!(target.y().abs() <= 2.6 + 1e-5, "{position:?} -> {target:?}");
        assert!((target - position).norm() > 0.4);
    }
}

#[test]
fn supporter_reentry_avoids_blocked_rule_regions() {
    use geometry::{direction::AngleTo, rectangle::Rectangle};
    use linear_algebra::Orientation2;
    use types::rule_obstacles::RuleObstacle;

    let mut blackboard = blackboard(PlayerNumber::Two, point![0.0, 0.0]);
    set_robot_pose(&mut blackboard, point![-5.3, 0.0]);
    let restricted = RuleObstacle::Rectangle(Rectangle {
        min: point![-4.5, -1.5],
        max: point![-3.0, 1.5],
    });
    let mut map = VoronoiGrid::new(point![-5.0, -3.5], point![5.0, 3.5], 0.2);
    blackboard.world_state.rule_obstacles = vec![restricted];
    map.initialize_obstacles(
        &[],
        &blackboard.world_state.rule_obstacles,
        blackboard.world_state.robot.ground_to_field.unwrap(),
    );
    blackboard.voronoi_map = Some(map);

    let status = walk_to_voronoi_position(&mut blackboard);
    if status == Status::Failure {
        // This pose can be enclosed by the goal structure and expanded rule
        // region. Standing is preferable to a direct shortcut through it.
        assert!(blackboard.body_motion.is_none());
        return;
    }
    assert_eq!(status, Status::Success);
    let target = walk_target_in_field(&blackboard);
    assert!(!restricted.contains(target));
    assert_ne!(
        blackboard
            .voronoi_map
            .as_ref()
            .unwrap()
            .ownership_at(target),
        Some(Ownership::Blocked)
    );
    let Some(BodyMotion::Walk { path, .. }) = &blackboard.body_motion else {
        panic!("expected a planned reentry path")
    };
    let ground_to_field = blackboard.world_state.robot.ground_to_field.unwrap();
    for segment in &path.segments {
        for step in 0..=100 {
            let fraction = step as f32 / 100.0;
            let position = match segment {
                types::path::PathSegment::LineSegment(line) => {
                    line.0 + (line.1 - line.0) * fraction
                }
                types::path::PathSegment::Arc(arc) => {
                    let angle = arc.start.angle()
                        + arc.start.angle_to(arc.end, arc.direction)
                            * arc.direction.angle_sign::<f32>()
                            * fraction;
                    arc.circle.point_at_angle(Orientation2::new(angle))
                }
            };
            assert!(!restricted.contains(ground_to_field * position));
        }
    }
}

#[test]
fn supporter_reentry_requires_an_outside_pose_and_an_unblocked_destination() {
    use geometry::rectangle::Rectangle;
    use types::rule_obstacles::RuleObstacle;

    let mut blackboard = blackboard(PlayerNumber::Two, point![0.0, 0.0]);
    set_robot_pose(&mut blackboard, point![0.0, 1.0]);
    blackboard.voronoi_map = Some(VoronoiGrid::new(point![-5.0, -3.5], point![5.0, 3.5], 0.2));
    assert_eq!(walk_to_voronoi_position(&mut blackboard), Status::Failure);
    assert!(blackboard.body_motion.is_none());

    set_robot_pose(&mut blackboard, point![0.0, 3.8]);
    blackboard
        .voronoi_map
        .as_mut()
        .unwrap()
        .initialize_obstacles(
            &[],
            &[RuleObstacle::Rectangle(Rectangle {
                min: point![-6.0, -4.0],
                max: point![6.0, 4.0],
            })],
            blackboard.world_state.robot.ground_to_field.unwrap(),
        );
    assert_eq!(walk_to_voronoi_position(&mut blackboard), Status::Failure);
    assert!(blackboard.body_motion.is_none());
}

#[test]
fn supporter_reentry_orientation_is_finite_when_its_destination_is_the_ball() {
    let mut blackboard = blackboard(PlayerNumber::Two, point![4.0, 0.0]);
    set_robot_pose(&mut blackboard, point![5.3, 0.0]);
    blackboard.voronoi_map = Some(VoronoiGrid::new(point![-5.0, -3.5], point![5.0, 3.5], 0.2));
    assert_eq!(walk_to_voronoi_position(&mut blackboard), Status::Success);
    let target = walk_target_in_field(&blackboard);
    approx::assert_relative_eq!(target.x(), 4.0, epsilon = 1e-5);
    approx::assert_relative_eq!(target.y(), 0.0, epsilon = 1e-5);
}

#[test]
fn supporter_reentry_does_not_use_a_direct_path_through_an_enclosing_rule_region() {
    use geometry::rectangle::Rectangle;
    use types::rule_obstacles::RuleObstacle;

    let mut blackboard = blackboard(PlayerNumber::Two, point![0.0, 0.0]);
    set_robot_pose(&mut blackboard, point![-5.8, 0.0]);
    blackboard.world_state.rule_obstacles = [
        (point![-5.5, -4.0], point![-4.5, 4.0]),
        (point![4.5, -4.0], point![5.5, 4.0]),
        (point![-5.5, -4.0], point![5.5, -3.0]),
        (point![-5.5, 3.0], point![5.5, 4.0]),
    ]
    .map(|(min, max)| RuleObstacle::Rectangle(Rectangle { min, max }))
    .to_vec();
    let mut map = VoronoiGrid::new(point![-6.0, -4.5], point![6.0, 4.5], 0.2);
    map.initialize_obstacles(
        &[],
        &blackboard.world_state.rule_obstacles,
        blackboard.world_state.robot.ground_to_field.unwrap(),
    );
    blackboard.voronoi_map = Some(map);
    assert_eq!(walk_to_voronoi_position(&mut blackboard), Status::Failure);
    assert!(blackboard.body_motion.is_none());
}
