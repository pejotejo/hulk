use coordinate_systems::{Field, Ground};
use filtering::hysteresis::less_than_with_relative_hysteresis;
use hsl_network_messages::{PlayerNumber, Team};
use linear_algebra::{Isometry2, Orientation2, Point, Point2, Pose2, point};
use path_planner::path_planner::PathPlanner;
use types::{
    behavior_tree::Status,
    field_dimensions::FieldDimensions,
    filtered_game_controller_state::FilteredGameControllerState,
    motion_command::{BodyMotion, MotionCommand, OrientationMode},
    motion_type::MotionType,
    obstacles::ObstacleKind,
    parameters::{KickoffParameters, VoronoiParameters},
    path::{Path, direct_path},
};
use voronoi::{Ownership, VoronoiGrid};

use crate::{
    action,
    actions::stand,
    behavior_tree::Node,
    condition,
    conditions::{hulks_is_kicking_team, opponent_near_ball},
    kick::{
        kick,
        kickoff::{kickoff_receiver, kickoff_receiver_position, kickoff_support_position},
        select_kick_target, use_last_kick_settings,
    },
    node::Blackboard,
    selection, sequence, subtree,
    switch_motion_type::{is_allowed_to_switch, is_last_motion_type, switch_motion_type},
};

pub fn plan(
    blackboard: &mut Blackboard,
    target_in_ground: Point2<Ground>,
    ground_to_field: Isometry2<Ground, Field>,
) -> Path {
    plan_with_ball_obstacle(blackboard, target_in_ground, ground_to_field, true)
}

pub(crate) fn plan_interception(
    blackboard: &mut Blackboard,
    target_in_ground: Point2<Ground>,
    ground_to_field: Isometry2<Ground, Field>,
) -> Path {
    plan_with_ball_obstacle(blackboard, target_in_ground, ground_to_field, false)
}

fn plan_with_ball_obstacle(
    blackboard: &mut Blackboard,
    target_in_ground: Point2<Ground>,
    ground_to_field: Isometry2<Ground, Field>,
    avoid_ball: bool,
) -> Path {
    let parameters: &types::parameters::PathPlanningParameters =
        &blackboard.parameters.walking.path_planning;
    let field_dimensions = blackboard.field_dimensions;

    let mut planner = PathPlanner {
        obstacle_escape_spline_segments: parameters.obstacle_escape_spline_segments,
        ..Default::default()
    };
    planner.with_last_motion(
        blackboard.last_motion_command.as_ref(),
        parameters.rotation_penalty_factor,
    );
    let target_in_field = ground_to_field * target_in_ground;
    let own_goal_x = -field_dimensions.length / 2.0;
    let keeper_defending = blackboard.world_state.robot.player_number
        == blackboard.parameters.goalkeeper.player_number
        && (own_goal_x..=own_goal_x + field_dimensions.goal_box_area_length)
            .contains(&target_in_field.x())
        && target_in_field.y().abs() <= field_dimensions.goal_inner_width / 2.0;
    for mut obstacle in blackboard.world_state.obstacles.iter().copied() {
        let position = ground_to_field * obstacle.position;
        let defending_own_post = keeper_defending
            && obstacle.kind == ObstacleKind::GoalPost
            && (position.x() - own_goal_x).abs() <= field_dimensions.goal_post_diameter
            && (position.y().abs()
                - (field_dimensions.goal_inner_width + field_dimensions.goal_post_diameter) / 2.0)
                .abs()
                <= field_dimensions.goal_post_diameter;
        let robot_radius = if defending_own_post {
            // Preserve body/post clearance without the maneuvering margin around other obstacles.
            obstacle.radius_at_hip_height = field_dimensions.goal_post_diameter / 2.0;
            parameters.robot_radius * 0.75
        } else {
            parameters.robot_radius
        };
        planner.with_obstacles(&[obstacle], robot_radius);
    }
    planner.with_rule_obstacles(
        ground_to_field.inverse(),
        &blackboard.world_state.rule_obstacles,
        parameters.robot_radius,
    );
    planner.with_field_borders(
        ground_to_field,
        field_dimensions.length,
        field_dimensions.width,
        field_dimensions.border_strip_width,
        parameters.field_border_weight,
    );
    planner.with_goal_support_structures(ground_to_field.inverse(), &field_dimensions);
    let ball_obstacle = blackboard.world_state.ball.map(|ball| ball.ball_in_ground);

    if avoid_ball && let Some(ball_position) = ball_obstacle {
        planner.with_ball(
            ball_position,
            parameters.ball_obstacle_radius,
            parameters.robot_radius,
        );
    }

    let x_max = field_dimensions.length / 2.0 + field_dimensions.border_strip_width;
    let y_max = field_dimensions.width / 2.0 + field_dimensions.border_strip_width;
    let clamped_target_in_robot = ground_to_field.inverse()
        * point![
            target_in_field.x().clamp(-x_max, x_max),
            target_in_field.y().clamp(-y_max, y_max)
        ];

    let path = planner
        .plan(Point::origin(), clamped_target_in_robot)
        .unwrap();
    blackboard.path_obstacles_output = planner.obstacles;
    path.unwrap_or_else(|| direct_path(Point::origin(), target_in_ground))
}

pub fn walk_to(
    blackboard: &mut Blackboard,
    target_pose: Pose2<Ground>,
    maximal_walk_speed: f32,
    orientation_mode: OrientationMode,
    distance_to_be_aligned: f32,
    hysteresis: nalgebra::Vector2<f32>,
) -> Status {
    if let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field {
        let parameters = &blackboard.parameters.walking.walk_and_stand;
        let distance_to_walk = target_pose.position().coords().norm();
        let angle_to_walk = target_pose.orientation().angle();
        let was_standing_last_cycle = matches!(
            blackboard.last_motion_command,
            Some(MotionCommand::Stand { .. })
        );
        let is_reached = less_than_with_relative_hysteresis(
            was_standing_last_cycle,
            distance_to_walk,
            parameters.target_reached_thresholds.x,
            0.0..=hysteresis.x,
        ) && less_than_with_relative_hysteresis(
            was_standing_last_cycle,
            angle_to_walk.abs(),
            parameters.target_reached_thresholds.y,
            0.0..=hysteresis.y,
        );

        let minimal_walk_speed = blackboard.parameters.walking.speed.minimum_speed;
        let velocity_fade_distance = blackboard.parameters.walking.speed.velocity_fade_distance;

        // Desmos: https://www.desmos.com/calculator/ss94dje2ke
        let walk_speed = maximal_walk_speed
            - (maximal_walk_speed - minimal_walk_speed)
                * (-(2.0 * distance_to_walk / velocity_fade_distance).powf(2.0)).exp();

        if is_reached {
            blackboard.body_motion = Some(BodyMotion::Stand);
            Status::Success
        } else {
            let path = plan(blackboard, target_pose.position(), ground_to_field);
            blackboard.body_motion = Some(BodyMotion::Walk {
                path,
                orientation_mode,
                target_orientation: target_pose.orientation(),
                distance_to_be_aligned,
                speed: walk_speed,
            });
            Status::Success
        }
    } else {
        Status::Failure
    }
}

pub fn walk_to_ball(blackboard: &mut Blackboard) -> Status {
    if let (Some(ball), Some(ground_to_field)) = (
        &blackboard.last_ball,
        &blackboard.world_state.robot.ground_to_field,
    ) {
        let field_to_ground = ground_to_field.inverse();
        let ball_in_ground =
            field_to_ground * blackboard.projected_ball_position(ball.position, ball.velocity);
        let goal_position = field_to_ground * point!(blackboard.field_dimensions.length / 2.0, 0.0);
        let orientation = Orientation2::from_vector(goal_position - ball_in_ground);
        let walk_and_stand = blackboard.parameters.walking.walk_and_stand;
        let kicking_speed = blackboard.parameters.walking.speed.kicking;

        let target_position = ball_in_ground
            - (goal_position - ball_in_ground).normalize()
                * blackboard.parameters.kicking.kick_position_ball_distance;
        walk_to(
            blackboard,
            Pose2::from_parts(target_position, orientation),
            kicking_speed,
            OrientationMode::AlignWithPath,
            walk_and_stand.normal_distance_to_be_aligned,
            walk_and_stand.hysteresis,
        )
    } else {
        Status::Failure
    }
}

pub fn block_opponent_with_ball(blackboard: &mut Blackboard) -> Status {
    if blackboard.last_motion_type == Some(MotionType::Kick)
        || (blackboard.last_motion_type != Some(MotionType::Walk)
            && !is_allowed_to_switch(blackboard))
        || !opponent_near_ball(
            blackboard,
            blackboard.parameters.kicking.strategy.pressure_distance,
        )
    {
        return Status::Failure;
    }
    let (Some(ball), Some(transform)) = (
        blackboard.ball.as_ref(),
        blackboard.world_state.robot.ground_to_field,
    ) else {
        return Status::Failure;
    };
    // A striker behind the attacker should approach from our goal side.
    if transform.translation().x() <= ball.position.x() + blackboard.field_dimensions.ball_radius {
        return Status::Failure;
    }
    let Some(toward_goal) = (point![-blackboard.field_dimensions.length / 2.0, 0.0]
        - ball.position)
        .try_normalize(f32::EPSILON)
    else {
        return Status::Failure;
    };
    let standoff = blackboard.parameters.kicking.approach_ball_standoff.max(
        blackboard.parameters.walking.path_planning.robot_radius
            + blackboard
                .parameters
                .walking
                .path_planning
                .ball_obstacle_radius
                .max(blackboard.field_dimensions.ball_radius)
            + 0.05,
    );
    let target = transform.inverse() * (ball.position + toward_goal * standoff);
    let ball = transform.inverse() * ball.position;
    let walking = blackboard.parameters.walking.walk_and_stand;
    blackboard.ball_action = None;
    blackboard.kick_target = None;
    walk_to(
        blackboard,
        Pose2::from_parts(target, Orientation2::from_vector(ball - target)),
        blackboard.parameters.walking.speed.blocking,
        OrientationMode::LookAt {
            target: ball,
            tolerance: walking.orientation_tolerance,
        },
        walking.normal_distance_to_be_aligned,
        walking.hysteresis,
    )
}

pub fn walk_to_ball_subtree() -> Node<Blackboard> {
    switch_motion_type(
        MotionType::Walk,
        action!(walk_to_ball),
        subtree!(walk_alternatives_subtree),
    )
}

pub fn walk_alternatives_subtree() -> Node<Blackboard> {
    selection!(
        sequence!(
            condition!(is_last_motion_type, MotionType::Kick),
            sequence!(
                action!(kick),
                action!(select_kick_target),
                action!(use_last_kick_power),
            )
        ),
        action!(stand)
    )
}

pub fn walk_to_block_position(blackboard: &mut Blackboard) -> Status {
    if let (Some(block_position), Some(ball), Some(ground_to_field)) = (
        &blackboard.walk_position,
        &blackboard.last_ball,
        blackboard.world_state.robot.ground_to_field,
    ) {
        let ball_position = ground_to_field.inverse() * ball.position;
        let orientation = Orientation2::from_vector(ball_position - *block_position);
        let walk_and_stand = blackboard.parameters.walking.walk_and_stand;
        let blocking_speed = blackboard.parameters.walking.speed.blocking;

        walk_to(
            blackboard,
            Pose2::from_parts(*block_position, orientation),
            blocking_speed,
            OrientationMode::LookAt {
                target: ball_position,
                tolerance: walk_and_stand.orientation_tolerance,
            },
            walk_and_stand.normal_distance_to_be_aligned,
            walk_and_stand.goalkeeper_hysteresis,
        )
    } else {
        Status::Failure
    }
}

pub fn walk_to_kickoff_pose(blackboard: &mut Blackboard) -> Status {
    let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field else {
        return Status::Failure;
    };

    let Some(game_controller_state) = blackboard
        .world_state
        .filtered_game_controller_state
        .as_ref()
    else {
        return Status::Failure;
    };

    let player_number = blackboard.player_number;

    if game_controller_state.penalties[player_number].is_some() {
        return Status::Failure;
    }

    let kickoff_pose = select_kickoff_pose(
        player_number,
        game_controller_state,
        blackboard.parameters.goalkeeper.player_number,
        &blackboard.parameters.kickoff,
        &blackboard.field_dimensions,
    );

    let walk_and_stand_parameters = blackboard.parameters.walking.walk_and_stand;

    walk_to(
        blackboard,
        ground_to_field.inverse() * kickoff_pose,
        blackboard.parameters.walking.speed.walk_to_kickoff,
        OrientationMode::AlignWithPath,
        walk_and_stand_parameters.normal_distance_to_be_aligned,
        walk_and_stand_parameters.hysteresis,
    )
}

fn select_kickoff_pose(
    player_number: PlayerNumber,
    game_controller_state: &FilteredGameControllerState,
    goalkeeper_player_number: PlayerNumber,
    kickoff_parameters: &KickoffParameters,
    field_dimensions: &FieldDimensions,
) -> Pose2<Field> {
    let field_center = point![0.0, 0.0];
    if player_number == goalkeeper_player_number {
        let relative_position = kickoff_parameters.relative_goalkeeper_position;
        let position = point![
            relative_position.x() * field_dimensions.length / 2.0,
            relative_position.y() * field_dimensions.width / 2.0
        ];
        let orientation = Orientation2::from_vector(field_center - position);
        return Pose2::from_parts(position, orientation);
    }

    let field_player_rank = game_controller_state
        .penalties
        .iter()
        .filter(|(number, penalty)| {
            *number > player_number && *number != goalkeeper_player_number && penalty.is_none()
        })
        .count();

    let relative_position = if game_controller_state.kicking_team == Some(Team::Hulks) {
        match field_player_rank {
            0 => kickoff_parameters.relative_striker_position,
            rank => kickoff_parameters.relative_aggressive_positions[rank - 1],
        }
    } else {
        kickoff_parameters.relative_defensive_positions[field_player_rank]
    };

    let position = point![
        relative_position.x() * field_dimensions.length / 2.0,
        relative_position.y() * field_dimensions.width / 2.0
    ];
    let orientation = Orientation2::from_vector(field_center - position);
    Pose2::from_parts(position, orientation)
}

pub fn walk_to_voronoi_position(blackboard: &mut Blackboard) -> Status {
    if let (Some(ground_to_field), Some(map)) = (
        blackboard.world_state.robot.ground_to_field,
        &blackboard.voronoi_map,
    ) && let Some(target_position) = target_player_position(
        map,
        blackboard.player_number,
        blackboard.ball.as_ref().map(|ball| ball.position),
        &blackboard.field_dimensions,
        &blackboard.parameters.voronoi,
    ) {
        let walk_and_stand = blackboard.parameters.walking.walk_and_stand;
        let kicking_speed = blackboard.parameters.walking.speed.kicking;
        let orientation_mode = if let Some(ball) = &blackboard.ball {
            OrientationMode::LookAt {
                target: ground_to_field.inverse() * ball.position,
                tolerance: walk_and_stand.orientation_tolerance,
            }
        } else {
            OrientationMode::AlignWithPath
        };

        let target_orientation =
            blackboard
                .ball
                .as_ref()
                .map_or_else(Orientation2::default, |ball| {
                    Orientation2::from_vector(
                        ground_to_field.inverse() * (ball.position - target_position),
                    )
                });
        walk_to(
            blackboard,
            Pose2::from_parts(
                ground_to_field.inverse() * target_position,
                target_orientation,
            ),
            kicking_speed,
            orientation_mode,
            walk_and_stand.normal_distance_to_be_aligned,
            walk_and_stand.hysteresis,
        )
    } else {
        Status::Failure
    }
}

fn target_player_position(
    map: &VoronoiGrid,
    player: PlayerNumber,
    ball_position: Option<Point2<Field>>,
    field_dimensions: &FieldDimensions,
    parameters: &VoronoiParameters,
) -> Option<Point2<Field>> {
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut count = 0;
    let mut candidates = Vec::new();

    for (point, ownership) in map.cells() {
        if ownership != Ownership::Robot(player)
            || point.x().abs() > field_dimensions.length / 2.0
            || point.y().abs() > field_dimensions.width / 2.0
        {
            continue;
        }

        candidates.push(point);

        sum_x += point.x();
        sum_y += point.y();
        count += 1;
    }

    if count == 0 {
        return None;
    }

    let inv_count = 1.0 / count as f32;
    let centroid: Point2<Field> = point![sum_x * inv_count, sum_y * inv_count];

    let Some(ball_position) = ball_position else {
        return Some(centroid);
    };

    let half_length = field_dimensions.length / 2.0 + parameters.padding;
    let ball_x = ball_position.x();
    let ball_y = ball_position.y();
    let side_factor = (ball_x / half_length).clamp(-1.0, 1.0);

    let resolution = map.resolution();

    let support_distance = (field_dimensions.center_circle_diameter / 2.0).max(resolution);
    let support_sigma =
        (parameters.relative_ball_support_sigma * field_dimensions.width).max(resolution);
    let inv_two_support_sigma_sq = 1.0 / (2.0 * support_sigma * support_sigma);

    let centroid_sigma =
        (parameters.relative_centroid_anchor_sigma * field_dimensions.width).max(resolution);

    let mut best_target = None;

    for point in candidates {
        let forward_norm = point.x() / half_length;
        let forward_term = parameters.forward_weight * side_factor * forward_norm;

        let dx_ball = point.x() - ball_x;
        let dy_ball = point.y() - ball_y;
        let ball_distance = (dx_ball * dx_ball + dy_ball * dy_ball).sqrt();
        let support_distance_error = ball_distance - support_distance;

        let ball_term = if dx_ball <= 0.0 {
            parameters.ball_weight
                * (-(support_distance_error * support_distance_error) * inv_two_support_sigma_sq)
                    .exp()
        } else {
            0.0
        };

        let dx_centroid = point.x() - centroid.x();
        let dy_centroid = point.y() - centroid.y();
        let centroid_penalty = parameters.centroid_anchor_weight
            * (dx_centroid * dx_centroid + dy_centroid * dy_centroid).sqrt()
            / centroid_sigma;

        let score = forward_term + ball_term - centroid_penalty;
        if best_target.is_none_or(|(best_score, _)| score > best_score) {
            best_target = Some((score, point));
        }
    }

    best_target.map(|(_, point)| point)
}

#[cfg(test)]
mod ball_support_tests {
    use super::*;

    #[test]
    fn defending_keeper_keeps_post_clearance_without_reducing_other_margins() {
        use crate::{goalkeeper::set_goalkeeper_active_defense_position, node::LastBall};
        use types::{
            field_dimensions::Side, obstacles::Obstacle, path::traits::EndPoints,
            world_state::BallSource,
        };
        for side in [-1.0, 1.0] {
            for post_radius in [0.05, 0.2] {
                for mapped_offset in [0.0, 0.025] {
                    let dimensions = FieldDimensions {
                        length: 14.0,
                        width: 9.0,
                        goal_inner_width: 2.4,
                        ..FieldDimensions::SPL_2025
                    };
                    let transform =
                        Isometry2::from_parts(linear_algebra::vector![-6.7, side * 0.6], 0.0);
                    let mut board = Blackboard {
                        field_dimensions: dimensions,
                        ball: Some(LastBall {
                            position: point![-6.8, side * 2.8],
                            velocity: Default::default(),
                            age: Default::default(),
                            field_side: Side::Left,
                            source: BallSource::Own,
                        }),
                        ..Default::default()
                    };
                    board.world_state.robot.ground_to_field = Some(transform);
                    board.parameters.goalkeeper.player_number = PlayerNumber::One;
                    board.world_state.robot.player_number = PlayerNumber::One;
                    board.parameters.goalkeeper.distance_to_goalpost = 0.15;
                    board
                        .parameters
                        .goalkeeper
                        .active_defense_maximum_robot_distance = 2.0;
                    board.parameters.walking.path_planning.robot_radius = 0.4;
                    let post = point![-7.0 - mapped_offset, side * 1.25];
                    board.world_state.obstacles = vec![
                        Obstacle::goal_post(transform.inverse() * post, post_radius),
                        Obstacle::goal_post(
                            transform.inverse() * point![7.0, side * 1.25],
                            post_radius,
                        ),
                        Obstacle::robot(point![2.0, 2.0], 0.2, 0.2),
                    ];
                    assert_eq!(
                        set_goalkeeper_active_defense_position(&mut board),
                        Status::Success
                    );
                    let target = board.walk_position.unwrap();
                    let path = plan(&mut board, target, transform);
                    let endpoint = transform * path.end_point();
                    assert!((endpoint - post).norm() >= 0.35 - 1e-5);
                    assert!((endpoint - transform * target).norm() < 0.15);
                    approx::assert_relative_eq!(
                        board.path_obstacles_output[1]
                            .shape
                            .as_circle()
                            .unwrap()
                            .radius,
                        post_radius + 0.4
                    );
                    approx::assert_relative_eq!(
                        board.path_obstacles_output[2]
                            .shape
                            .as_circle()
                            .unwrap()
                            .radius,
                        0.6
                    );

                    plan(
                        &mut board,
                        transform.inverse() * point![-5.0, 0.5],
                        transform,
                    );
                    approx::assert_relative_eq!(
                        board.path_obstacles_output[0]
                            .shape
                            .as_circle()
                            .unwrap()
                            .radius,
                        post_radius + 0.4,
                        epsilon = 1e-5
                    );

                    board.world_state.robot.player_number = PlayerNumber::Two;
                    let fielder_path = plan(&mut board, target, transform);
                    assert!((fielder_path.end_point() - target).norm() > 0.15);
                    approx::assert_relative_eq!(
                        board.path_obstacles_output[0]
                            .shape
                            .as_circle()
                            .unwrap()
                            .radius,
                        post_radius + 0.4
                    );
                }
            }
        }
    }

    #[test]
    fn approaches_contested_ball_from_own_goal_side() {
        use crate::node::LastBall;
        use types::{
            field_dimensions::Side,
            obstacles::{Obstacle, ObstacleKind},
            path::PathSegment,
            world_state::BallSource,
        };
        let mut blackboard = Blackboard {
            field_dimensions: FieldDimensions::SPL_2025,
            ..Default::default()
        };
        blackboard.world_state.robot.ground_to_field = Some(Default::default());
        blackboard.ball = Some(LastBall {
            position: point![-1.0, 0.0],
            velocity: Default::default(),
            age: Default::default(),
            field_side: Side::Left,
            source: BallSource::Own,
        });
        blackboard.parameters.kicking.strategy.pressure_distance = 0.8;
        blackboard.parameters.walking.path_planning.robot_radius = 0.4;
        blackboard.world_state.obstacles = vec![Obstacle {
            kind: ObstacleKind::Robot,
            position: point![-0.9, 0.1],
            radius_at_foot_height: 0.2,
            radius_at_hip_height: 0.2,
        }];
        assert_eq!(block_opponent_with_ball(&mut blackboard), Status::Success);
        let Some(BodyMotion::Walk {
            path,
            orientation_mode: OrientationMode::LookAt { target, .. },
            ..
        }) = &blackboard.body_motion
        else {
            panic!("expected goal-side approach")
        };
        let PathSegment::LineSegment(segment) = path.last_segment() else {
            panic!("expected path to goal-side position")
        };
        assert!(segment.1.x() < -1.4);
        assert_eq!(*target, point![-1.0, 0.0]);
        let opponent = blackboard.world_state.obstacles.pop().unwrap();
        assert_eq!(block_opponent_with_ball(&mut blackboard), Status::Failure);
        blackboard.world_state.obstacles.push(opponent);
        blackboard.last_motion_type = Some(MotionType::Kick);
        assert_eq!(block_opponent_with_ball(&mut blackboard), Status::Failure);
        blackboard.last_motion_type = Some(MotionType::Walk);
        blackboard.world_state.robot.ground_to_field = Some(Isometry2::from_parts(
            linear_algebra::vector![-1.5, 0.0],
            0.0,
        ));
        assert_eq!(block_opponent_with_ball(&mut blackboard), Status::Failure);
    }

    #[test]
    fn more_ball_weight_moves_support_toward_ball_ring() {
        let mut map = VoronoiGrid::new(point![-4.5, -3.0], point![4.5, 3.0], 0.2);
        map.multi_source_dijkstra(&[(Pose2::new(point![-2.0, 0.0], 0.0), PlayerNumber::Two)]);
        let ball = point![3.8, 0.0];
        let mut parameters = VoronoiParameters {
            ball_weight: 1.0,
            ball_support_distance: 1.5,
            ball_support_sigma: 0.5,
            centroid_anchor_weight: 0.6,
            centroid_anchor_sigma: 1.0,
            ..Default::default()
        };
        let before = target_player_position(
            &map,
            PlayerNumber::Two,
            Some(ball),
            &FieldDimensions::SPL_2025,
            &parameters,
        )
        .unwrap();
        parameters.ball_weight = 3.0;
        let after = target_player_position(
            &map,
            PlayerNumber::Two,
            Some(ball),
            &FieldDimensions::SPL_2025,
            &parameters,
        )
        .unwrap();
        assert!((after - ball).norm() < (before - ball).norm());
        assert!(((after - ball).norm() - parameters.ball_support_distance).abs() < 0.5);
    }
}
