use coordinate_systems::{Field, Ground};
use filtering::hysteresis::less_than_with_relative_hysteresis;
use geometry::{Distance, line_segment::LineSegment};
use hsl_network_messages::PlayerNumber;
use linear_algebra::{Isometry2, Orientation2, Point, Point2, Pose2, point};
use path_planner::path_planner::PathPlanner;
use types::{
    behavior_tree::Status,
    motion_command::{BodyMotion, MotionCommand, OrientationMode},
    motion_type::MotionType,
    obstacles::{Obstacle, ObstacleKind},
    path::{Path, direct_path},
    rule_obstacles::RuleObstacle,
};
use voronoi::{Ownership, VoronoiGrid};

use crate::{
    action,
    actions::stand,
    behavior_tree::Node,
    condition,
    conditions::hulks_is_kicking_team,
    kick::{kick, select_kick_target, use_last_kick_settings},
    node::Blackboard,
    selection, sequence, subtree,
    switch_motion_type::{is_last_motion_type, switch_motion_type},
    voronoi::fresh_teammate_poses,
};

pub fn plan(
    blackboard: &mut Blackboard,
    target_in_ground: Point2<Ground>,
    ground_to_field: Isometry2<Ground, Field>,
) -> Path {
    try_plan(blackboard, target_in_ground, ground_to_field)
        .unwrap_or_else(|| direct_path(Point::origin(), target_in_ground))
}

pub(crate) fn try_plan(
    blackboard: &mut Blackboard,
    target_in_ground: Point2<Ground>,
    ground_to_field: Isometry2<Ground, Field>,
) -> Option<Path> {
    let parameters: &types::parameters::PathPlanningParameters =
        &blackboard.parameters.walking.path_planning;
    let field_dimensions = blackboard.field_dimensions;

    let mut planner = PathPlanner {
        obstacle_escape_spline_segments: parameters.obstacle_escape_spline_segments,
        ..Default::default()
    };
    planner.with_last_motion(
        &blackboard.last_motion_command,
        parameters.rotation_penalty_factor,
    );
    planner.with_obstacles(&blackboard.world_state.obstacles, parameters.robot_radius);
    let teammate_obstacles: Vec<_> = fresh_teammate_poses(blackboard)
        .map(|(_, pose)| ground_to_field.inverse() * pose.position())
        .filter(|position| {
            !blackboard.world_state.obstacles.iter().any(|obstacle| {
                obstacle.kind == ObstacleKind::Robot
                    && (obstacle.position - position).norm() <= parameters.robot_radius
            })
        })
        .map(|position| Obstacle::robot(position, parameters.robot_radius, parameters.robot_radius))
        .collect();
    planner.with_obstacles(&teammate_obstacles, parameters.robot_radius);
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

    if let Some(ball_position) = ball_obstacle {
        planner.with_ball(
            ball_position,
            parameters.ball_obstacle_radius,
            parameters.robot_radius,
        );
    }

    let target_in_field = ground_to_field * target_in_ground;
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
    path
}

pub fn walk_to(
    blackboard: &mut Blackboard,
    target_pose: Pose2<Ground>,
    maximal_walk_speed: f32,
    orientation_mode: OrientationMode,
    distance_to_be_aligned: f32,
    hysteresis: nalgebra::Vector2<f32>,
) -> Status {
    walk_to_with_path(
        blackboard,
        target_pose,
        maximal_walk_speed,
        orientation_mode,
        distance_to_be_aligned,
        hysteresis,
        None,
    )
}

pub(crate) fn walk_to_with_path(
    blackboard: &mut Blackboard,
    target_pose: Pose2<Ground>,
    maximal_walk_speed: f32,
    orientation_mode: OrientationMode,
    distance_to_be_aligned: f32,
    hysteresis: nalgebra::Vector2<f32>,
    planned_path: Option<Path>,
) -> Status {
    if let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field {
        let parameters = &blackboard.parameters.walking.walk_and_stand;
        let distance_to_walk = target_pose.position().coords().norm();
        let angle_to_walk = target_pose.orientation().angle();
        let was_standing_last_cycle =
            matches!(blackboard.last_motion_command, MotionCommand::Stand { .. });
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
            let path = planned_path
                .unwrap_or_else(|| plan(blackboard, target_pose.position(), ground_to_field));
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
    let target = blackboard
        .world_state
        .robot
        .ground_to_field
        .zip(blackboard.kick_target)
        .map(|(ground_to_field, target)| ground_to_field * target)
        .unwrap_or_else(|| point!(blackboard.field_dimensions.length / 2.0, 0.0));
    crate::kick::execution::walk_to_ball_action(blackboard, target)
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
                action!(use_last_kick_settings),
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
    if let (Some(ground_to_field), player_number) = (
        blackboard.world_state.robot.ground_to_field,
        blackboard.world_state.robot.player_number,
    ) {
        let field_to_ground = ground_to_field.inverse();
        let kickoff = &blackboard.parameters.kickoff;
        let standard_pose = kickoff.standard_positions[player_number];
        let striker_position = kickoff.striker_position;
        let walk_and_stand = blackboard.parameters.walking.walk_and_stand;
        let walk_to_kickoff_speed = blackboard.parameters.walking.speed.walk_to_kickoff;

        let mut target_position = standard_pose.position;

        if hulks_is_kicking_team(blackboard) && player_number == PlayerNumber::Three {
            target_position = striker_position;
        }

        let kickoff_pose_in_field =
            Pose2::from_parts(target_position, Orientation2::new(standard_pose.rotation));

        let kickoff_pose_in_ground = field_to_ground * kickoff_pose_in_field;

        walk_to(
            blackboard,
            kickoff_pose_in_ground,
            walk_to_kickoff_speed,
            OrientationMode::AlignWithPath,
            walk_and_stand.normal_distance_to_be_aligned,
            walk_and_stand.hysteresis,
        );
        Status::Success
    } else {
        Status::Failure
    }
}

pub fn walk_to_voronoi_position(blackboard: &mut Blackboard) -> Status {
    if let (Some(ground_to_field), Some(map)) = (
        blackboard.world_state.robot.ground_to_field,
        &blackboard.voronoi_map,
    ) && let Some(target_position) = target_player_position(blackboard, map).or_else(|| {
        // A robot beyond the grid has no support region. Re-enter the field
        // so it can participate in the next ownership election.
        let position = ground_to_field.as_pose().position();
        let half_length = blackboard.field_dimensions.length / 2.0;
        let half_width = blackboard.field_dimensions.width / 2.0;
        if position.x().abs() <= half_length && position.y().abs() <= half_width {
            return None;
        }
        let margin = blackboard.parameters.walking.path_planning.robot_radius;
        let target_x = half_length - margin;
        let target_y = half_width - margin;
        map.cells()
            .filter(|(point, ownership)| {
                *ownership != Ownership::Blocked
                    && point.x().abs() <= target_x
                    && point.y().abs() <= target_y
                    && blackboard
                        .world_state
                        .rule_obstacles
                        .iter()
                        .all(|obstacle| match obstacle {
                            RuleObstacle::Circle(circle) => {
                                (point - circle.center).norm_squared()
                                    > (circle.radius + margin).powi(2)
                            }
                            RuleObstacle::Rectangle(rectangle) => {
                                !(rectangle.min.x() - margin..=rectangle.max.x() + margin)
                                    .contains(&point.x())
                                    || !(rectangle.min.y() - margin..=rectangle.max.y() + margin)
                                        .contains(&point.y())
                            }
                        })
            })
            .min_by(|(a, _), (b, _)| {
                (a - position)
                    .norm_squared()
                    .total_cmp(&(b - position).norm_squared())
            })
            .map(|(point, _)| point)
    }) {
        let Some(path) = try_plan(
            blackboard,
            ground_to_field.inverse() * target_position,
            ground_to_field,
        ) else {
            // A supporter must not take a direct shortcut through teammates or
            // a rule area when the planned route is unavailable.
            return Status::Failure;
        };
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
        walk_to_with_path(
            blackboard,
            Pose2::from_parts(
                ground_to_field.inverse() * target_position,
                target_orientation,
            ),
            kicking_speed,
            orientation_mode,
            walk_and_stand.normal_distance_to_be_aligned,
            walk_and_stand.hysteresis,
            Some(path),
        )
    } else {
        Status::Failure
    }
}

fn target_player_position(blackboard: &Blackboard, map: &VoronoiGrid) -> Option<Point2<Field>> {
    let player = blackboard.world_state.robot.player_number;
    let field_dimensions = &blackboard.field_dimensions;
    let parameters = &blackboard.parameters.voronoi;
    let ball_position = blackboard.ball.as_ref().map(|ball| ball.position);
    let ground_to_field = blackboard.world_state.robot.ground_to_field?;
    let robot_radius = blackboard.parameters.walking.path_planning.robot_radius;
    let obstacles: Vec<_> = blackboard
        .world_state
        .obstacles
        .iter()
        .map(|obstacle| {
            (
                ground_to_field * obstacle.position,
                obstacle
                    .radius_at_foot_height
                    .max(obstacle.radius_at_hip_height),
            )
        })
        .chain(fresh_teammate_poses(blackboard).map(|(_, pose)| (pose.position(), robot_radius)))
        .collect();
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

    let half_length = field_dimensions.length / 2.0 + parameters.padding;
    let ball_x = ball_position.map_or(0.0, |position| position.x());
    let side_factor = (ball_x / half_length).clamp(-1.0, 1.0);

    let resolution = map.resolution();

    let support_distance = parameters.ball_support_distance.max(resolution);
    let support_sigma = parameters.ball_support_sigma.max(resolution);
    let inv_two_support_sigma_sq = 1.0 / (2.0 * support_sigma * support_sigma);

    let centroid_sigma = parameters.centroid_anchor_sigma.max(resolution);

    let mut best_target = None;

    for point in candidates {
        // Ownership cells account for obstacle centers. A walking destination
        // also needs room for this robot's footprint, including unseen teammates.
        if point.x().abs() > field_dimensions.length / 2.0 - robot_radius
            || point.y().abs() > field_dimensions.width / 2.0 - robot_radius
            || obstacles.iter().any(|(position, radius)| {
                (point - position).norm_squared() <= (radius + robot_radius).powi(2)
            })
            || !has_rule_clearance(blackboard, point, robot_radius)
        {
            continue;
        }

        let ball_term = if let Some(ball_position) = ball_position {
            let ball_distance = (point - ball_position).norm();
            let ball_workspace =
                2.0 * robot_radius + blackboard.parameters.kicking.approach_ball_standoff;
            if ball_distance < ball_workspace {
                continue;
            }
            let support_distance_error = ball_distance - support_distance;
            // A Gaussian alone becomes negligible throughout a distant player's
            // region. Keep a pull toward useful support range even there,
            // rather than letting its old territorial centroid hold it upfield.
            let distant_support_penalty = support_distance_error.max(0.0) / support_sigma;
            parameters.ball_weight
                * ((-(support_distance_error * support_distance_error) * inv_two_support_sigma_sq)
                    .exp()
                    - distant_support_penalty)
        } else {
            0.0
        };

        let forward_norm = point.x() / half_length;
        let forward_term = parameters.forward_weight * side_factor * forward_norm;

        let dx_centroid = point.x() - centroid.x();
        let dy_centroid = point.y() - centroid.y();
        let centroid_penalty = parameters.centroid_anchor_weight
            * (dx_centroid * dx_centroid + dy_centroid * dy_centroid).sqrt()
            / centroid_sigma;

        let lane_score = ball_position.map_or(0.0, |ball_position| {
            let receiving_lane = LineSegment::new(ball_position, point);
            let receiving_clearance = obstacles
                .iter()
                .filter(|(position, _)| {
                    (0.1..0.9).contains(&receiving_lane.projection_factor(*position))
                })
                .map(|(position, radius)| {
                    ((receiving_lane.distance_to(*position)
                        - radius
                        - field_dimensions.ball_radius)
                        / robot_radius.max(resolution))
                    .clamp(0.0, 1.0)
                })
                .fold(1.0_f32, f32::min);
            let goal_center = point!(field_dimensions.length / 2.0, 0.0);
            let shot_vector = goal_center - ball_position;
            let shot_blocking = if shot_vector.norm_squared() > f32::EPSILON {
                let shot_lane = LineSegment::new(ball_position, goal_center);
                let along_lane = shot_lane.projection_factor(point);
                if (0.0..1.0).contains(&along_lane) {
                    (1.0 - shot_lane.distance_to(point) / (2.0 * robot_radius).max(resolution))
                        .clamp(0.0, 1.0)
                } else {
                    0.0
                }
            } else {
                0.0
            };

            parameters.receiving_lane_weight * receiving_clearance
                - parameters.shot_lane_weight * shot_blocking
        });
        let score = forward_term + ball_term - centroid_penalty + lane_score;
        if best_target.is_none_or(|(best_score, _)| score > best_score) {
            best_target = Some((score, point));
        }
    }

    best_target.map(|(_, point)| point)
}

fn has_rule_clearance(blackboard: &Blackboard, point: Point2<Field>, margin: f32) -> bool {
    has_rule_path_clearance(blackboard, LineSegment(point, point), margin)
}

/// Check the original rule regions, before any planner destination projection.
pub(crate) fn has_rule_path_clearance(
    blackboard: &Blackboard,
    path: LineSegment<Field>,
    margin: f32,
) -> bool {
    blackboard
        .world_state
        .rule_obstacles
        .iter()
        .all(|obstacle| match obstacle {
            RuleObstacle::Circle(circle) => {
                let distance_squared = if path.length_squared() <= f32::EPSILON {
                    (path.0 - circle.center).norm_squared()
                } else {
                    path.squared_distance_to(circle.center)
                };
                distance_squared > (circle.radius + margin).powi(2)
            }
            RuleObstacle::Rectangle(rectangle) => {
                let min = point![rectangle.min.x() - margin, rectangle.min.y() - margin];
                let max = point![rectangle.max.x() + margin, rectangle.max.y() + margin];
                let outside = |point: Point2<Field>| {
                    !(min.x()..=max.x()).contains(&point.x())
                        || !(min.y()..=max.y()).contains(&point.y())
                };
                let corners = [min, point![min.x(), max.y()], max, point![max.x(), min.y()]];
                outside(path.0)
                    && outside(path.1)
                    && (path.length_squared() <= f32::EPSILON
                        || (0..4).all(|i| {
                            !path.intersects_line_segment(LineSegment(
                                corners[i],
                                corners[(i + 1) % 4],
                            ))
                        }))
            }
        })
}
