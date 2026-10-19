use std::time::Duration;

use behavior_node::{
    conditions::is_closest_to_ball,
    node::{Blackboard, LastBall},
    tree::create_tree,
    voronoi::calculate_voronoi_grid,
    walk::walk_to_voronoi_position,
};
use coordinate_systems::Field;
use geometry::{Distance, circle::Circle, line_segment::LineSegment, rectangle::Rectangle};
use hsl_network_messages::{Penalty, PlayerNumber, Team};
use linear_algebra::{Point2, Pose2, point};
use ros_z::time::Time;
use types::{
    behavior_tree::{NodeTrace, Status},
    fall_detection::{FallDetection, Posture},
    field_dimensions::{FieldDimensions, Side},
    filtered_game_controller_state::FilteredGameControllerState,
    filtered_game_state::FilteredGameState,
    motion_command::{BodyMotion, HeadMotion},
    obstacles::Obstacle,
    path::traits::EndPoints,
    primary_state::PrimaryState,
    rule_obstacles::RuleObstacle,
    time_wrapper::TimeWrapper,
    world_state::{BallSource, BallState, PlayerState},
};
use voronoi::{Ownership, VoronoiGrid};

fn blackboard(player: PlayerNumber, position: Point2<Field>) -> Blackboard {
    let mut blackboard = Blackboard::default();
    blackboard.field_dimensions = FieldDimensions::SPL_2025;
    blackboard.world_state.now = Time::from_nanos(10_000_000_000);
    blackboard.world_state.robot.player_number = player;
    blackboard.world_state.robot.ground_to_field = Some(Pose2::new(position, 0.0).as_transform());
    blackboard.world_state.filtered_game_controller_state =
        Some(FilteredGameControllerState::default());
    blackboard.ball = Some(LastBall {
        position: point!(0.0, 0.0),
        velocity: Default::default(),
        age: blackboard.world_state.now,
        field_side: Side::Left,
        source: BallSource::Own,
    });
    blackboard.last_ball = blackboard.ball.clone();
    let parameters = &mut blackboard.parameters;
    parameters.ball.closest_to_ball.enter_duration = Duration::ZERO;
    parameters.ball.closest_to_ball.exit_duration = Duration::from_millis(150);
    parameters.kicking.kick_activation_distance = 0.8;
    parameters.kicking.kick_activation_hysteresis = 0.2;
    parameters.kicking.approach_ball_standoff = 0.3;
    parameters.goalkeeper.player_number = PlayerNumber::One;
    parameters.goalkeeper.striker_distance = 3.5;
    parameters.voronoi.grid_resolution = 0.2;
    parameters.voronoi.padding = 0.5;
    parameters.voronoi.ball_support_distance = 1.5;
    parameters.voronoi.ball_support_sigma = 0.5;
    parameters.voronoi.ball_weight = 1.0;
    parameters.voronoi.centroid_anchor_weight = 0.1;
    parameters.voronoi.centroid_anchor_sigma = 1.0;
    parameters.voronoi.teammate_pose_max_age = Duration::from_millis(1500);
    parameters.voronoi.receiving_lane_weight = 0.25;
    parameters.voronoi.shot_lane_weight = 0.35;
    parameters.walking.path_planning.robot_radius = 0.28;
    parameters.walking.path_planning.ball_obstacle_radius = 0.05;
    parameters
        .walking
        .path_planning
        .obstacle_escape_spline_segments = 5;
    parameters.walking.walk_and_stand.target_reached_thresholds = nalgebra::Vector2::new(0.05, 0.1);
    parameters.walking.speed.kicking = 1.8;
    parameters.walking.speed.minimum_speed = 0.2;
    parameters.walking.speed.velocity_fade_distance = 3.0;
    blackboard
}

fn teammate(
    blackboard: &mut Blackboard,
    player: PlayerNumber,
    position: Point2<Field>,
    time: Time,
) {
    let state = PlayerState {
        pose: Pose2::new(position, 0.0),
        ..Default::default()
    };
    blackboard.world_state.player_states[player] = Some(state);
    blackboard.timed_player_states[player] = Some(TimeWrapper { time, inner: state });
}

fn claim(blackboard: &mut Blackboard) -> bool {
    assert_eq!(calculate_voronoi_grid(blackboard), Status::Success);
    is_closest_to_ball(blackboard)
}

#[test]
fn delayed_reciprocal_poses_leave_one_ball_pursuer_even_during_exit_hysteresis() {
    let mut two = blackboard(PlayerNumber::Two, point!(-0.5, 0.15));
    let mut three = blackboard(PlayerNumber::Three, point!(-0.45, -0.2));
    let time = Time::from_nanos(9_500_000_000);
    teammate(&mut two, PlayerNumber::Three, point!(-0.7, -0.2), time);
    teammate(&mut three, PlayerNumber::Two, point!(-0.75, 0.15), time);
    two.last_closest_to_ball = true;
    three.last_closest_to_ball = true;

    assert!(claim(&mut two));
    assert!(!claim(&mut three));
    // Each current pose beats the delayed peer in its own territorial map. The
    // local yield resolves that ambiguity without changing either support map.
    for blackboard in [&two, &three] {
        assert_eq!(
            blackboard
                .voronoi_map
                .as_ref()
                .unwrap()
                .ownership_at(point!(0.0, 0.0)),
            Some(Ownership::Robot(blackboard.world_state.robot.player_number))
        );
    }
    assert!(!three.last_closest_to_ball);
    assert!(three.closest_to_ball_entered_area_since.is_none());
}

#[test]
fn stale_future_and_penalized_teammates_do_not_force_local_yielding() {
    for time in [
        Time::from_nanos(8_000_000_000),
        Time::from_nanos(10_100_000_000),
    ] {
        let mut three = blackboard(PlayerNumber::Three, point!(-0.45, -0.2));
        teammate(&mut three, PlayerNumber::Two, point!(-0.75, 0.15), time);
        assert!(claim(&mut three));
    }
    let mut three = blackboard(PlayerNumber::Three, point!(-0.45, -0.2));
    teammate(
        &mut three,
        PlayerNumber::Two,
        point!(-0.75, 0.15),
        Time::from_nanos(9_500_000_000),
    );
    three
        .world_state
        .filtered_game_controller_state
        .as_mut()
        .unwrap()
        .penalties
        .two = Some(Penalty::MotionInSet {
        remaining: Duration::from_secs(10),
    });
    assert!(claim(&mut three));
}

#[test]
fn clearly_closer_player_keeps_ball_despite_higher_number() {
    let mut two = blackboard(PlayerNumber::Two, point!(-0.85, 0.0));
    let mut three = blackboard(PlayerNumber::Three, point!(-0.15, 0.0));
    let now = two.world_state.now;
    teammate(&mut two, PlayerNumber::Three, point!(-0.15, 0.0), now);
    teammate(&mut three, PlayerNumber::Two, point!(-0.85, 0.0), now);
    two.last_closest_to_ball = true;
    assert!(!claim(&mut two));
    assert!(claim(&mut three));
}

#[test]
fn ineligible_goalkeeper_does_not_win_a_local_contest() {
    let mut three = blackboard(PlayerNumber::Three, point!(0.1, 0.0));
    let now = three.world_state.now;
    teammate(&mut three, PlayerNumber::One, point!(-0.4, 0.0), now);
    assert!(claim(&mut three));
}

fn support_target(blackboard: &mut Blackboard) -> Point2<Field> {
    let player = blackboard.world_state.robot.player_number;
    let ground_to_field = blackboard.world_state.robot.ground_to_field.unwrap();
    let mut map = VoronoiGrid::new(point!(0.2, -2.0), point!(3.0, 2.0), 0.2);
    map.multi_source_dijkstra(&[(ground_to_field.as_pose(), player)]);
    blackboard.voronoi_map = Some(map);
    assert_eq!(walk_to_voronoi_position(blackboard), Status::Success);
    let Some(BodyMotion::Walk { path, .. }) = &blackboard.body_motion else {
        panic!("supporter did not walk")
    };
    ground_to_field * path.segments.last().unwrap().end_point()
}

#[test]
fn supporters_leave_shooting_lane_and_keep_their_owned_region() {
    let mut supporter = blackboard(PlayerNumber::Three, point!(1.8, -1.5));
    let target = support_target(&mut supporter);
    assert!(
        target.y().abs() >= 0.5,
        "target {target:?} crowds the shooting lane"
    );
    assert!((target - point!(0.0, 0.0)).norm() >= 0.86);
    assert_eq!(
        supporter.voronoi_map.as_ref().unwrap().ownership_at(target),
        Some(Ownership::Robot(PlayerNumber::Three))
    );
}

#[test]
fn receiving_target_clears_robots_teammates_rules_and_the_ball_lane() {
    let mut supporter = blackboard(PlayerNumber::Three, point!(1.8, -1.5));
    let transform = supporter.world_state.robot.ground_to_field.unwrap();
    let blocker = point!(0.8, 0.0);
    supporter
        .world_state
        .obstacles
        .push(Obstacle::robot(transform.inverse() * blocker, 0.2, 0.25));
    let peer = point!(1.3, -1.1);
    let now = supporter.world_state.now;
    teammate(&mut supporter, PlayerNumber::Two, peer, now);
    let rule = Rectangle {
        min: point!(2.0, -0.4),
        max: point!(2.8, 0.4),
    };
    supporter
        .world_state
        .rule_obstacles
        .push(RuleObstacle::Rectangle(rule));
    let target = support_target(&mut supporter);
    assert!((target - blocker).norm() > 0.53);
    assert!((target - peer).norm() > 0.56);
    assert!(target.x() < 1.72 || target.y().abs() > 0.68);
    assert!(
        LineSegment::new(point!(0.0, 0.0), target).distance_to(blocker) > 0.3,
        "blocked receiving lane toward {target:?}"
    );
}

#[test]
fn consistent_nearby_poses_elect_one_pursuer_even_when_lower_number_is_farther() {
    let mut two = blackboard(PlayerNumber::Two, point![-0.6, 0.0]);
    let mut three = blackboard(PlayerNumber::Three, point![-0.2, 0.0]);
    let now = two.world_state.now;
    teammate(&mut two, PlayerNumber::Three, point![-0.2, 0.0], now);
    teammate(&mut three, PlayerNumber::Two, point![-0.6, 0.0], now);
    assert!(claim(&mut two));
    assert!(!claim(&mut three));
}

#[test]
fn rule_separated_contenders_keep_the_obstacle_aware_territorial_pursuer() {
    for rule in [
        RuleObstacle::Rectangle(Rectangle {
            min: point![-0.5, -4.0],
            max: point![-0.4, 4.0],
        }),
        RuleObstacle::Circle(Circle::new(point![-0.4, 0.0], 0.05)),
    ] {
        let mut two = blackboard(PlayerNumber::Two, point![-0.6, 0.0]);
        let mut three = blackboard(PlayerNumber::Three, point![-0.2, 0.0]);
        let now = two.world_state.now;
        teammate(&mut two, PlayerNumber::Three, point![-0.2, 0.0], now);
        teammate(&mut three, PlayerNumber::Two, point![-0.6, 0.0], now);
        two.world_state.rule_obstacles.push(rule.clone());
        three.world_state.rule_obstacles.push(rule);
        assert!(!claim(&mut two));
        assert!(claim(&mut three));
        assert_eq!(
            two.voronoi_map
                .as_ref()
                .unwrap()
                .ownership_at(point![0.0, 0.0]),
            Some(Ownership::Robot(PlayerNumber::Three))
        );
    }
}

#[test]
fn an_intervening_robot_keeps_the_clearer_territorial_route() {
    let two_position = point![-0.6, 0.6];
    let three_position = point![-0.6, -0.6];
    let mut two = blackboard(PlayerNumber::Two, two_position);
    let mut three = blackboard(PlayerNumber::Three, three_position);
    let now = two.world_state.now;
    teammate(&mut two, PlayerNumber::Three, three_position, now);
    teammate(&mut three, PlayerNumber::Two, two_position, now);
    for blackboard in [&mut two, &mut three] {
        let transform = blackboard.world_state.robot.ground_to_field.unwrap();
        blackboard.world_state.obstacles.push(Obstacle::robot(
            transform.inverse() * point![-0.3, 0.2],
            0.25,
            0.25,
        ));
    }
    assert!(!claim(&mut two));
    assert!(claim(&mut three));
}

#[test]
fn observed_endpoint_robots_do_not_disable_the_local_tie_break() {
    let two_position = point![-0.6, 0.0];
    let three_position = point![-0.2, 0.0];
    let mut two = blackboard(PlayerNumber::Two, two_position);
    let mut three = blackboard(PlayerNumber::Three, three_position);
    let now = two.world_state.now;
    teammate(&mut two, PlayerNumber::Three, three_position, now);
    teammate(&mut three, PlayerNumber::Two, two_position, now);
    for blackboard in [&mut two, &mut three] {
        let transform = blackboard.world_state.robot.ground_to_field.unwrap();
        let other_position = if blackboard.world_state.robot.player_number == PlayerNumber::Two {
            three_position
        } else {
            two_position
        };
        blackboard.world_state.obstacles.push(Obstacle::robot(
            transform.inverse() * other_position,
            0.2,
            0.25,
        ));
    }
    assert!(claim(&mut two));
    assert!(!claim(&mut three));
}

#[test]
fn distant_supporters_retreat_toward_defensive_play_on_both_sides_without_losing_clearance() {
    let mut targets = Vec::new();
    for side in [-1.0, 1.0] {
        let mut supporter = blackboard(PlayerNumber::Three, point![2.0, side]);
        supporter.ball.as_mut().unwrap().position = point![-3.0, side * 1.2];
        supporter.parameters.voronoi.centroid_anchor_weight = 0.6;
        supporter.parameters.voronoi.forward_weight = 0.2;
        supporter.parameters.walking.path_planning.robot_radius = 0.4;
        let peer = point![0.2, side];
        let now = supporter.world_state.now;
        teammate(&mut supporter, PlayerNumber::Two, peer, now);
        let rule = Rectangle {
            min: point![0.4, side - 0.4],
            max: point![1.0, side + 0.4],
        };
        supporter
            .world_state
            .rule_obstacles
            .push(RuleObstacle::Rectangle(rule));
        let target = support_target(&mut supporter);
        // The ball's preferred ring lies outside this entire owned region.
        // Staying near its forward centroid would leave this player unable to
        // support either the defender or a pass out of our own half.
        assert!(
            target.x() <= 0.4,
            "distant supporter stayed upfield at {target:?}"
        );
        assert!((target - peer).norm() > 0.8);
        assert!(
            target.x() < 0.0
                || target.x() > 1.4
                || target.y() < side - 0.8
                || target.y() > side + 0.8
        );
        assert_eq!(
            supporter.voronoi_map.as_ref().unwrap().ownership_at(target),
            Some(Ownership::Robot(PlayerNumber::Three))
        );
        targets.push(target);
    }
    approx::assert_relative_eq!(targets[0].x(), targets[1].x(), epsilon = 0.0001);
    approx::assert_relative_eq!(targets[0].y(), -targets[1].y(), epsilon = 0.0001);
}

fn kickoff_player(player: PlayerNumber) -> Blackboard {
    let poses = [
        (PlayerNumber::One, point![-4.0, 0.0]),
        (PlayerNumber::Two, point![-0.65, 0.0]),
        (PlayerNumber::Three, point![-2.0, 1.4]),
        (PlayerNumber::Four, point![-2.0, -1.4]),
    ];
    let position = poses
        .iter()
        .find(|(number, _)| *number == player)
        .unwrap()
        .1;
    let mut b = blackboard(player, position);
    let now = b.world_state.now;
    b.world_state.robot.primary_state = PrimaryState::Playing;
    b.world_state.fall_detection = Some(FallDetection {
        time: now,
        valid_until: now + Duration::from_secs(10),
        posture: Posture::Upright,
    });
    b.world_state.filtered_game_controller_state = Some(FilteredGameControllerState {
        game_state: FilteredGameState::Playing {
            ball_is_free: true,
            kick_off: true,
        },
        kicking_team: Some(Team::Hulks),
        ..Default::default()
    });
    for (number, pose) in poses {
        if number != player {
            teammate(&mut b, number, pose, now);
        }
    }
    b.parameters.ball.last_ball_timeout = Duration::from_secs(1);
    b.parameters.goalkeeper.x_offset = 0.3;
    b.parameters.walking.speed.blocking = 1.0;
    set_kickoff_ball(&mut b, Point2::origin());
    b
}

fn set_kickoff_ball(b: &mut Blackboard, position: Point2<Field>) {
    b.world_state.ball = Some(BallState {
        ball_in_field: position,
        ball_in_ground: b.world_state.robot.ground_to_field.unwrap().inverse() * position,
        last_seen_ball: b.world_state.now.to_wallclock(),
        source: BallSource::Own,
        ..Default::default()
    });
    b.ball.as_mut().unwrap().position = position;
    b.ball.as_mut().unwrap().age = b.world_state.now;
}

fn succeeded(trace: &NodeTrace, name: &str) -> bool {
    trace.name == name && trace.status == Status::Success
        || trace.children.iter().any(|child| succeeded(child, name))
}

#[test]
fn kickoff_support_holds_its_legal_pose_then_resumes_after_the_ball_moves() {
    let mut supporter = kickoff_player(PlayerNumber::Three);
    let tree = create_tree();
    let (status, trace) = tree.tick_with_trace(&mut supporter);
    assert_eq!(status, Status::Success);
    assert!(matches!(supporter.body_motion, Some(BodyMotion::Stand)));
    assert!(matches!(
        supporter.head_motion,
        Some(HeadMotion::LookAt { .. })
    ));
    assert!(!succeeded(&trace, "subtree_striker_subtree"));
    assert!(!succeeded(&trace, "walk_to_voronoi_position"));

    supporter.world_state.now = supporter.world_state.now + Duration::from_millis(20);
    set_kickoff_ball(&mut supporter, point![0.31, 0.0]);
    supporter.body_motion = None;
    supporter.voronoi_map = None;
    supporter.voronoi_inputs.clear();
    let (status, trace) = tree.tick_with_trace(&mut supporter);
    assert_eq!(status, Status::Success);
    assert!(supporter.kickoff_support.as_ref().unwrap().released);
    assert!(succeeded(&trace, "walk_to_voronoi_position"));
    assert!(matches!(
        supporter.body_motion,
        Some(BodyMotion::Walk { .. })
    ));

    // A missed sighting cannot make a non-taker leave its legal kickoff pose.
    let mut blind_supporter = kickoff_player(PlayerNumber::Three);
    blind_supporter.world_state.ball = None;
    blind_supporter.ball = None;
    let (status, trace) = tree.tick_with_trace(&mut blind_supporter);
    assert_eq!(status, Status::Success);
    assert!(matches!(
        blind_supporter.body_motion,
        Some(BodyMotion::Stand)
    ));
    assert!(!succeeded(&trace, "leuchtturm"));
}

#[test]
fn kickoff_support_hold_preserves_the_elected_taker_and_keeper_branches() {
    let tree = create_tree();
    let mut taker = kickoff_player(PlayerNumber::Two);
    let (status, trace) = tree.tick_with_trace(&mut taker);
    assert_eq!(status, Status::Success);
    assert!(succeeded(&trace, "subtree_striker_subtree"));
    assert!(matches!(
        taker.body_motion,
        Some(BodyMotion::Kick { .. } | BodyMotion::Walk { .. })
    ));

    let mut keeper = kickoff_player(PlayerNumber::One);
    let (status, trace) = tree.tick_with_trace(&mut keeper);
    assert_eq!(status, Status::Success);
    assert!(succeeded(&trace, "subtree_goalkeeper_subtree"));
    assert!(!succeeded(&trace, "is_waiting_for_kickoff_movement"));
}
