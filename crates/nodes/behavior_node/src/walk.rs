use coordinate_systems::{Field, Ground};
use filtering::hysteresis::less_than_with_relative_hysteresis;
use hsl_network_messages::PlayerNumber;
use linear_algebra::{Isometry2, Orientation2, Point, Point2, Pose2, point};
use path_planner::path_planner::PathPlanner;
use types::{
    behavior_tree::Status,
    field_dimensions::FieldDimensions,
    motion_command::{BodyMotion, MotionCommand, OrientationMode},
    motion_type::MotionType,
    parameters::VoronoiParameters,
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
    planner.with_obstacles(&blackboard.world_state.obstacles, parameters.robot_radius);
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
                * blackboard.parameters.kicking.approach_ball_standoff;
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
        blackboard.player_number,
    ) {
        let field_to_ground = ground_to_field.inverse();
        let kickoff = &blackboard.parameters.kickoff;
        let standard_pose = kickoff.standard_positions[player_number];
        let striker_position = kickoff.striker_position;
        let walk_and_stand = blackboard.parameters.walking.walk_and_stand;
        let walk_to_kickoff_speed = blackboard.parameters.walking.speed.walk_to_kickoff;

        let mut target_position = standard_pose.position;

        let mut target_orientation = Orientation2::new(standard_pose.rotation);
        let own_kickoff = hulks_is_kicking_team(blackboard);
        if own_kickoff && player_number != blackboard.parameters.goalkeeper.player_number {
            if player_number == PlayerNumber::Three {
                target_position = striker_position;
            } else if Some(player_number) == kickoff_receiver(blackboard) {
                target_position = kickoff_receiver_position(blackboard);
                // Receive with an open stance so the next shot needs only a small turn.
                target_orientation = Orientation2::from_vector(
                    (-target_position.coords()).normalize()
                        + (point![blackboard.field_dimensions.length / 2.0, 0.0] - target_position)
                            .normalize(),
                );
                let position = ground_to_field.as_pose().position();
                let radius = blackboard.parameters.walking.path_planning.robot_radius;
                if blackboard.world_state.robot.primary_state
                    == types::primary_state::PrimaryState::Ready
                    && position.y() * target_position.y().signum() < 2.0 * radius + 0.05
                {
                    // Enter the pass-side lane behind the striker before walking forward.
                    let staging_x = (striker_position.x() - 2.0 * radius - 0.05).min(
                        -blackboard.field_dimensions.center_circle_diameter / 2.0 - radius - 0.05,
                    );
                    target_position =
                        if position.x() > staging_x + walk_and_stand.target_reached_thresholds.x {
                            point![staging_x, position.y()]
                        } else {
                            point![position.x(), target_position.y()]
                        };
                }
            }
        }
        if player_number != blackboard.parameters.goalkeeper.player_number
            && !(own_kickoff && player_number == PlayerNumber::Three)
        {
            target_position = kickoff_support_position(blackboard, target_position);
        }

        let kickoff_pose_in_field = Pose2::from_parts(target_position, target_orientation);

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

    let support_distance = parameters.ball_support_distance.max(resolution);
    let support_sigma = parameters.ball_support_sigma.max(resolution);
    let inv_two_support_sigma_sq = 1.0 / (2.0 * support_sigma * support_sigma);

    let centroid_sigma = parameters.centroid_anchor_sigma.max(resolution);

    let mut best_target = None;

    for point in candidates {
        let forward_norm = point.x() / half_length;
        let forward_term = parameters.forward_weight * side_factor * forward_norm;

        let dx_ball = point.x() - ball_x;
        let dy_ball = point.y() - ball_y;
        let ball_distance = (dx_ball * dx_ball + dy_ball * dy_ball).sqrt();
        let support_distance_error = ball_distance - support_distance;
        let ball_term = parameters.ball_weight
            * (-(support_distance_error * support_distance_error) * inv_two_support_sigma_sq).exp();

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
