use coordinate_systems::Field;
use filtering::hysteresis::less_than_with_hysteresis;
use geometry::{Distance, line_segment::LineSegment, rectangle::Rectangle};
use hsl_network_messages::Team;
use linear_algebra::{Point2, point, vector};
use types::{
    controller_input::Button, fall_detection::Posture,
    filtered_game_controller_state::FilteredGameControllerState,
    filtered_game_state::FilteredGameState, obstacles::ObstacleKind, primary_state::PrimaryState,
    rule_obstacles::RuleObstacle,
};
use voronoi::{Ownership, VoronoiGrid};

use crate::{
    goalkeeper::is_goalkeeper_striker_candidate,
    node::Blackboard,
    voronoi::{available_teammate_poses, create_voronoi_grid_without_goalkeeper},
};

pub fn is_ball_interception_candidate(blackboard: &mut Blackboard) -> bool {
    if let (Some(ball), Some(ground_to_field)) = (
        &blackboard.ball,
        &blackboard.world_state.robot.ground_to_field,
    ) {
        let parameters = &blackboard.parameters.ball.interception;

        let ball_in_ground = ground_to_field.inverse() * ball.position;
        let ball_is_in_front_of_robot = ball_in_ground.coords().norm()
            < parameters.maximum_ball_distance
            && ball_in_ground.x() > 0.0;
        let ball_is_moving_towards_robot = (ground_to_field.inverse() * ball.velocity).x()
            < -parameters.minimum_ball_velocity_towards_robot;

        let ball_in_field_velocity = ball.velocity;
        let ball_is_moving = ball_in_field_velocity.norm() > parameters.minimum_ball_velocity;
        let ball_is_moving_towards_own_half =
            ball_in_field_velocity.x() < -parameters.minimum_ball_velocity_towards_own_half;

        ball_is_in_front_of_robot
            && ball_is_moving
            && ball_is_moving_towards_robot
            && ball_is_moving_towards_own_half
    } else {
        false
    }
}

pub fn is_close_to_ball(blackboard: &mut Blackboard) -> bool {
    let mut is_close = false;
    if let (Some(ball), Some(ground_to_field)) = (
        &blackboard.ball,
        &blackboard.world_state.robot.ground_to_field,
    ) {
        let distance_to_ball = (ground_to_field.inverse() * ball.position).coords().norm();
        let parameters = &blackboard.parameters.kicking;
        is_close = less_than_with_hysteresis(
            blackboard.last_close_enough_to_kick,
            distance_to_ball,
            parameters.kick_activation_distance,
            parameters.kick_activation_hysteresis,
        );
        blackboard.last_close_enough_to_kick = is_close;
    }

    is_close
}

pub fn is_close_to_ball_aligned(blackboard: &mut Blackboard) -> bool {
    let mut is_close_and_aligned = false;
    if let (Some(ball), Some(ground_to_field)) = (
        &blackboard.ball,
        &blackboard.world_state.robot.ground_to_field,
    ) {
        let ball_in_ground = ground_to_field.inverse() * ball.position;
        let goal_position =
            ground_to_field.inverse() * point!(blackboard.field_dimensions.length / 2.0, 0.0);
        let target_kick_position = ball_in_ground
            - (goal_position - ball_in_ground).normalize()
                * blackboard.parameters.kicking.approach_ball_standoff;

        let parameters = &blackboard.parameters.substates;
        let distance_to_ball = target_kick_position.coords().norm();
        let is_close = less_than_with_hysteresis(
            blackboard.last_close_enough_to_kick,
            distance_to_ball,
            parameters.distance_for_kick,
            parameters.distance_for_kick_hysteresis,
        );

        let direction = (goal_position - target_kick_position).normalize();
        let robot_facing_direction = vector!(1.0, 0.0);
        let is_aligned =
            direction.angle(&robot_facing_direction) < parameters.alignment_angle_threshold;

        blackboard.direction_difference = direction.angle(&robot_facing_direction);

        is_close_and_aligned = is_close && (is_aligned || blackboard.last_close_enough_to_kick);
        blackboard.last_close_enough_to_kick = is_close_and_aligned;
    }

    is_close_and_aligned
}

pub fn is_closest_to_ball(blackboard: &mut Blackboard) -> bool {
    let own_player_number = blackboard.world_state.robot.player_number;

    let ownership = if let (Some(ball_position), Some(voronoi_map)) = (
        blackboard.ball.as_ref().map(|ball| ball.position),
        &blackboard.voronoi_map,
    ) {
        let ownership = resolved_ownership(voronoi_map, ball_position);
        let ownership = if ownership
            == Some(Ownership::Robot(
                blackboard.parameters.goalkeeper.player_number,
            ))
            && !is_goalkeeper_striker_candidate(blackboard)
        {
            // Elect a field player without moving the goalkeeper's support region.
            create_voronoi_grid_without_goalkeeper(blackboard)
                .and_then(|map| resolved_ownership(&map, ball_position))
        } else {
            ownership
        };
        ownership
    } else {
        None
    };
    let territorial_owner = match ownership {
        Some(Ownership::Robot(player)) => Some(player),
        _ => None,
    };
    let local_owner = local_ball_owner(blackboard, territorial_owner);
    // Resolve nearby ambiguous claims in both directions: the winner must take
    // ownership as well as the loser yielding, or consistent poses can leave
    // both robots waiting. Preserve the territorial support map.
    if local_owner.is_some_and(|player| player != own_player_number) {
        blackboard.closest_to_ball_entered_area_since = None;
        blackboard.closest_to_ball_left_area_since = Some(blackboard.world_state.now);
        blackboard.last_closest_to_ball = false;
        return false;
    }
    let raw_is_closest = local_owner.or(territorial_owner) == Some(own_player_number);

    let now = blackboard.world_state.now;
    if raw_is_closest {
        blackboard.closest_to_ball_entered_area_since =
            blackboard.closest_to_ball_entered_area_since.or(Some(now));
        blackboard.closest_to_ball_left_area_since = None;
    } else {
        blackboard.closest_to_ball_left_area_since =
            blackboard.closest_to_ball_left_area_since.or(Some(now));
        blackboard.closest_to_ball_entered_area_since = None;
    }

    let parameters = blackboard.parameters.ball.closest_to_ball;
    let is_closest = if blackboard.last_closest_to_ball {
        raw_is_closest
            || blackboard
                .closest_to_ball_left_area_since
                .is_some_and(|since| now.duration_since(since) < parameters.exit_duration)
    } else {
        raw_is_closest
            && blackboard
                .closest_to_ball_entered_area_since
                .is_some_and(|since| now.duration_since(since) >= parameters.enter_duration)
    };

    blackboard.last_closest_to_ball = is_closest;
    is_closest
}

fn local_ball_owner(
    blackboard: &mut Blackboard,
    territorial_owner: Option<hsl_network_messages::PlayerNumber>,
) -> Option<hsl_network_messages::PlayerNumber> {
    let ball_position = blackboard.ball.as_ref()?.position;
    let own_position = blackboard
        .world_state
        .robot
        .ground_to_field?
        .as_pose()
        .position();
    // Team elections use the measured shared point, never the distance-dependent
    // prediction used for each robot's approach.
    let own_distance = (own_position - ball_position).norm();
    let parameters = &blackboard.parameters;
    let tie_distance = 2.0 * parameters.walking.path_planning.robot_radius;
    let local_distance = parameters.voronoi.ball_support_distance.max(tie_distance);
    let workspace = local_distance.max(
        parameters.kicking.kick_activation_distance + parameters.kicking.kick_activation_hysteresis,
    );
    if own_distance > workspace {
        return None;
    }
    let own_player = blackboard.world_state.robot.player_number;
    let goalkeeper = parameters.goalkeeper.player_number;
    let keeper_eligible = is_goalkeeper_striker_candidate(blackboard);
    let mut candidates = vec![(own_player, own_distance)];
    candidates.extend(
        available_teammate_poses(blackboard).filter_map(|(player, pose)| {
            let distance = (pose.position() - ball_position).norm();
            (distance <= workspace
                && (pose.position() - own_position).norm() <= local_distance
                && (player != goalkeeper || keeper_eligible)
                && local_contest_is_unseparated(blackboard, own_position, pose.position()))
            .then_some((player, distance))
        }),
    );
    // Do not override an obstacle-aware territorial owner outside this contest.
    if candidates.len() < 2
        || !candidates
            .iter()
            .any(|(player, _)| Some(*player) == territorial_owner)
    {
        return None;
    }
    let nearest = candidates
        .iter()
        .map(|(_, distance)| *distance)
        .min_by(f32::total_cmp)?;
    candidates
        .into_iter()
        .filter(|(_, distance)| *distance <= nearest + tie_distance)
        .map(|(player, _)| player)
        .min()
}

fn local_contest_is_unseparated(
    blackboard: &Blackboard,
    own_position: Point2<Field>,
    teammate_position: Point2<Field>,
) -> bool {
    let corridor = LineSegment::new(own_position, teammate_position);
    let radius = blackboard.parameters.walking.path_planning.robot_radius;
    let distance_to_corridor = |position: Point2<Field>| {
        if corridor.length_squared() <= f32::EPSILON {
            (position - own_position).norm()
        } else {
            corridor.distance_to(position)
        }
    };
    if blackboard
        .world_state
        .rule_obstacles
        .iter()
        .any(|obstacle| match obstacle {
            RuleObstacle::Circle(circle) => {
                distance_to_corridor(circle.center) <= circle.radius + radius
            }
            RuleObstacle::Rectangle(rectangle) => {
                let expanded = Rectangle {
                    min: rectangle.min - vector!(radius, radius),
                    max: rectangle.max + vector!(radius, radius),
                };
                let bottom_right = point!(expanded.max.x(), expanded.min.y());
                let top_left = point!(expanded.min.x(), expanded.max.y());
                expanded.contains(own_position)
                    || expanded.contains(teammate_position)
                    || [
                        LineSegment::new(expanded.min, bottom_right),
                        LineSegment::new(bottom_right, expanded.max),
                        LineSegment::new(expanded.max, top_left),
                        LineSegment::new(top_left, expanded.min),
                    ]
                    .into_iter()
                    .any(|edge| corridor.intersects_line_segment(edge))
            }
        })
    {
        return false;
    }
    let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field else {
        return false;
    };
    blackboard.world_state.obstacles.iter().all(|obstacle| {
        if obstacle.kind == ObstacleKind::Ball {
            return true;
        }
        let position = ground_to_field * obstacle.position;
        // The contenders themselves are observed as robots. They must not be
        // mistaken for an intervening obstacle when deciding which one yields.
        if obstacle.kind == ObstacleKind::Robot
            && ((position - own_position).norm() <= radius
                || (position - teammate_position).norm() <= radius)
        {
            return true;
        }
        distance_to_corridor(position)
            > obstacle
                .radius_at_foot_height
                .max(obstacle.radius_at_hip_height)
                + radius
    })
}

fn resolved_ownership(map: &VoronoiGrid, position: Point2<Field>) -> Option<Ownership> {
    match map.ownership_at(position) {
        Some(Ownership::Blocked) => map.nearest_non_blocked_ownership(position),
        ownership => ownership,
    }
}

fn posture(blackboard: &Blackboard) -> Option<Posture> {
    blackboard
        .world_state
        .fall_detection
        .filter(|fall_detection| fall_detection.is_valid(blackboard.world_state.now))
        .map(|fall_detection| fall_detection.posture)
}

pub fn should_do_fall_protection(blackboard: &mut Blackboard) -> bool {
    matches!(posture(blackboard), None | Some(Posture::Falling))
}

pub fn is_fallen(blackboard: &mut Blackboard) -> bool {
    matches!(posture(blackboard), Some(Posture::Fallen { .. }))
}

pub fn is_ready_for_stand_up(blackboard: &mut Blackboard) -> bool {
    matches!(
        posture(blackboard),
        Some(
            Posture::Fallen {
                is_ready_for_standup: true
            } | Posture::StandingUp
        )
    )
}

pub fn is_goalkeeper(blackboard: &mut Blackboard) -> bool {
    blackboard.world_state.robot.player_number == blackboard.parameters.goalkeeper.player_number
}

pub fn is_primary_state(blackboard: &mut Blackboard, primary_state: PrimaryState) -> bool {
    blackboard.world_state.robot.primary_state == primary_state
}

pub fn is_remote_control_enabled(blackboard: &mut Blackboard) -> bool {
    blackboard.remote_control_enabled
}

pub fn is_controller_connected(blackboard: &mut Blackboard) -> bool {
    blackboard
        .controller_input
        .as_ref()
        .is_some_and(|controller| controller.connected)
}

pub fn is_remote_kick_mode(blackboard: &mut Blackboard, button: Button) -> bool {
    blackboard
        .controller_input
        .as_ref()
        .is_some_and(|controller| controller.is_pressed(button))
}

pub fn has_ball_position(blackboard: &mut Blackboard) -> bool {
    blackboard.ball.is_some()
}

pub fn has_new_ball_position(blackboard: &mut Blackboard) -> bool {
    blackboard.world_state.ball.is_some()
}

pub fn has_hypothetical_ball_position(blackboard: &mut Blackboard) -> bool {
    !blackboard
        .world_state
        .hypothetical_ball_positions
        .is_empty()
}

pub fn hulks_is_kicking_team(blackboard: &mut Blackboard) -> bool {
    matches!(
        blackboard.world_state.filtered_game_controller_state,
        Some(FilteredGameControllerState {
            kicking_team: Some(Team::Hulks),
            ..
        })
    )
}

pub fn is_ball_free(blackboard: &mut Blackboard) -> bool {
    matches!(
        blackboard.world_state.filtered_game_controller_state,
        Some(FilteredGameControllerState {
            game_state: FilteredGameState::Playing {
                ball_is_free: true,
                ..
            },
            ..
        })
    )
}

pub fn is_kickoff(blackboard: &mut Blackboard) -> bool {
    matches!(
        blackboard.world_state.filtered_game_controller_state,
        Some(FilteredGameControllerState {
            game_state: FilteredGameState::Playing { kick_off: true, .. },
            ..
        })
    )
}

pub fn is_last_hulk_standing(blackboard: &mut Blackboard) -> bool {
    let own_player_number = blackboard.world_state.robot.player_number;

    blackboard
        .world_state
        .player_states
        .iter()
        .all(|(player_number, player_state)| {
            player_number == own_player_number || player_state.is_none_or(|state| state.unavailable)
        })
}

pub fn is_simple(blackboard: &mut Blackboard) -> bool {
    blackboard.parameters.control.is_simple
}
