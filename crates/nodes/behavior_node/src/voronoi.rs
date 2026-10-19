use coordinate_systems::{Field, Ground};
use hsl_network_messages::PlayerNumber;
use linear_algebra::{Isometry2, Pose2, point};
use types::behavior_tree::Status;
use voronoi::VoronoiGrid;

use crate::node::Blackboard;

/// Teammate poses suitable for immediate coordination, rather than long-lived
/// territorial estimates. Future and expired packets cannot reserve space.
pub(crate) fn fresh_teammate_poses(
    blackboard: &Blackboard,
) -> impl Iterator<Item = (PlayerNumber, Pose2<Field>)> + '_ {
    let now = blackboard.world_state.now;
    let own_player_number = blackboard.world_state.robot.player_number;
    let maximum_age = blackboard.parameters.voronoi.teammate_pose_max_age;

    blackboard
        .timed_player_states
        .iter()
        .filter_map(move |(player_number, state)| {
            let state = state.as_ref()?;
            if player_number == own_player_number
                || state.time > now
                || now.duration_since(state.time) > maximum_age
                || blackboard
                    .world_state
                    .filtered_game_controller_state
                    .as_ref()
                    .is_some_and(|game| game.penalties[player_number].is_some())
                || !state
                    .inner
                    .pose
                    .position()
                    .inner
                    .iter()
                    .all(|value| value.is_finite())
            {
                return None;
            }

            Some((player_number, state.inner.pose))
        })
}

/// Fresh teammates that can accept a ball role. Unavailable teammates remain
/// present in `fresh_teammate_poses` so their bodies still reserve space.
pub(crate) fn available_teammate_poses(
    blackboard: &Blackboard,
) -> impl Iterator<Item = (PlayerNumber, Pose2<Field>)> + '_ {
    fresh_teammate_poses(blackboard).filter(move |(player, _)| {
        blackboard.timed_player_states[*player]
            .as_ref()
            .is_some_and(|state| !state.inner.unavailable)
    })
}

pub fn calculate_voronoi_grid(blackboard: &mut Blackboard) -> Status {
    let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field else {
        return Status::Failure;
    };

    let sites = collect_sites(blackboard, ground_to_field.as_pose());
    calculate_voronoi_grid_from_sites(blackboard, ground_to_field, &sites)
}

pub fn calculate_voronoi_grid_without_goalkeeper(blackboard: &mut Blackboard) -> Status {
    let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field else {
        return Status::Failure;
    };

    let mut sites = collect_sites(blackboard, ground_to_field.as_pose());
    let goalkeeper = blackboard.parameters.goalkeeper.player_number;
    sites.retain(|(_, player_number)| *player_number != goalkeeper);

    calculate_voronoi_grid_from_sites(blackboard, ground_to_field, &sites)
}

fn calculate_voronoi_grid_from_sites(
    blackboard: &mut Blackboard,
    ground_to_field: Isometry2<Ground, Field>,
    sites: &[(Pose2<Field>, PlayerNumber)],
) -> Status {
    for (pose, _) in sites {
        blackboard.voronoi_inputs.push(*pose);
    }

    blackboard.voronoi_map = Some(create_voronoi_grid(blackboard, ground_to_field, sites));
    Status::Success
}

pub(crate) fn create_voronoi_grid_without_goalkeeper(
    blackboard: &Blackboard,
) -> Option<VoronoiGrid> {
    let ground_to_field = blackboard.world_state.robot.ground_to_field?;
    let mut sites = collect_sites(blackboard, ground_to_field.as_pose());
    let goalkeeper = blackboard.parameters.goalkeeper.player_number;
    sites.retain(|(_, player_number)| *player_number != goalkeeper);

    Some(create_voronoi_grid(blackboard, ground_to_field, &sites))
}

fn create_voronoi_grid(
    blackboard: &Blackboard,
    ground_to_field: Isometry2<Ground, Field>,
    sites: &[(Pose2<Field>, PlayerNumber)],
) -> VoronoiGrid {
    let field_dimensions = &blackboard.field_dimensions;
    let voronoi_parameters = &blackboard.parameters.voronoi;
    let obstacles = &blackboard.world_state.obstacles;
    let rule_obstacles = &blackboard.world_state.rule_obstacles;

    let length_half = field_dimensions.length / 2.0;
    let width_half = field_dimensions.width / 2.0;
    let padding = voronoi_parameters.padding;

    let grid_min = point!(-length_half - padding, -width_half - padding);
    let grid_max = point!(length_half + padding, width_half + padding);

    let mut map = VoronoiGrid::new(grid_min, grid_max, voronoi_parameters.grid_resolution);
    map.initialize_obstacles(obstacles, rule_obstacles, ground_to_field);
    map.multi_source_dijkstra(sites);
    map
}

fn collect_sites(
    blackboard: &Blackboard,
    robot_pose: Pose2<Field>,
) -> Vec<(Pose2<Field>, PlayerNumber)> {
    let robot_player_number = blackboard.world_state.robot.player_number;
    let mut sites = vec![(robot_pose, robot_player_number)];

    for (player_number, player_state) in blackboard.world_state.player_states.iter() {
        if let Some(player_state) = player_state
            && player_number != robot_player_number
            && !player_state.unavailable
        {
            sites.push((player_state.pose, player_number));
        }
    }

    sites
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    use crate::{
        conditions::is_closest_to_ball,
        kick::{execution::execute_ball_action, strategy::BallAction},
        node::LastBall,
    };
    use ros_z::time::Time;
    use types::{
        field_dimensions::{FieldDimensions, Side},
        motion_command::BodyMotion,
        primary_state::PrimaryState,
        time_wrapper::TimeWrapper,
        world_state::{BallSource, PlayerState},
    };
    use voronoi::Ownership;

    fn blackboard(own_position: linear_algebra::Point2<Field>) -> Blackboard {
        let mut b = Blackboard::default();
        b.field_dimensions = FieldDimensions::SPL_2025;
        b.world_state.now = Time::from_nanos(10_000_000_000);
        b.world_state.robot.player_number = PlayerNumber::Three;
        b.world_state.robot.primary_state = PrimaryState::Playing;
        b.world_state.robot.ground_to_field = Some(Pose2::new(own_position, 0.0).as_transform());
        b.parameters.goalkeeper.player_number = PlayerNumber::One;
        b.parameters.voronoi.grid_resolution = 0.2;
        b.parameters.voronoi.padding = 0.5;
        b.parameters.voronoi.teammate_pose_max_age = Duration::from_millis(1500);
        b.parameters.walking.path_planning.robot_radius = 0.4;
        b.parameters.ball.closest_to_ball.enter_duration = Duration::ZERO;
        b.parameters.ball.closest_to_ball.exit_duration = Duration::ZERO;
        b
    }

    fn teammate(
        b: &mut Blackboard,
        player: PlayerNumber,
        position: linear_algebra::Point2<Field>,
        unavailable: bool,
    ) {
        let state = PlayerState {
            pose: Pose2::new(position, 0.0),
            unavailable,
            ..Default::default()
        };
        b.world_state.player_states[player] = Some(state);
        b.timed_player_states[player] = Some(TimeWrapper {
            time: b.world_state.now - Duration::from_millis(60),
            inner: state,
        });
    }

    fn set_ball(b: &mut Blackboard, position: linear_algebra::Point2<Field>) {
        b.ball = Some(LastBall {
            position,
            velocity: Default::default(),
            age: b.world_state.now,
            field_side: Side::Left,
            source: BallSource::Own,
        });
    }

    #[test]
    fn stationary_ball_is_reassigned_when_nearby_owner_is_unavailable() {
        // The measured Small defense shape at 43 s: H2 is nearest but unable
        // to stand up, while both upright field players remain in support.
        let ball = point![3.625348, 0.002947];
        let mut b = blackboard(point![2.420794, 0.742351]);
        set_ball(&mut b, ball);
        teammate(
            &mut b,
            PlayerNumber::One,
            point![-4.144783, -0.029957],
            false,
        );
        teammate(&mut b, PlayerNumber::Two, point![4.094954, 0.650741], false);
        teammate(
            &mut b,
            PlayerNumber::Four,
            point![2.447441, -0.781851],
            false,
        );
        assert_eq!(calculate_voronoi_grid(&mut b), Status::Success);
        assert_eq!(
            b.voronoi_map.as_ref().unwrap().ownership_at(ball),
            Some(Ownership::Robot(PlayerNumber::Two))
        );
        assert!(!is_closest_to_ball(&mut b));

        let fallen_position = point![4.094954, 0.650741];
        teammate(&mut b, PlayerNumber::Two, fallen_position, true);
        b.voronoi_inputs.clear();
        assert_eq!(calculate_voronoi_grid(&mut b), Status::Success);
        assert_eq!(
            b.voronoi_map.as_ref().unwrap().ownership_at(ball),
            Some(Ownership::Robot(PlayerNumber::Three))
        );
        assert!(is_closest_to_ball(&mut b));
        assert!(
            !b.voronoi_inputs
                .iter()
                .any(|pose| pose.position() == fallen_position)
        );
        assert!(fresh_teammate_poses(&b).any(|(player, _)| player == PlayerNumber::Two));
        assert!(!available_teammate_poses(&b).any(|(player, _)| player == PlayerNumber::Two));

        // A fresh available update restores both territorial and local claims.
        b.world_state.now = b.world_state.now + Duration::from_millis(100);
        teammate(&mut b, PlayerNumber::Two, fallen_position, false);
        assert_eq!(calculate_voronoi_grid(&mut b), Status::Success);
        assert_eq!(
            b.voronoi_map.as_ref().unwrap().ownership_at(ball),
            Some(Ownership::Robot(PlayerNumber::Two))
        );
        assert!(!is_closest_to_ball(&mut b));
    }

    #[test]
    fn unavailable_lower_number_cannot_reclaim_a_local_contest_but_keeps_body_clearance() {
        let mut b = blackboard(point![0.0, 0.0]);
        set_ball(&mut b, point![0.7, 0.0]);
        teammate(&mut b, PlayerNumber::Two, point![0.45, 0.0], true);
        assert_eq!(calculate_voronoi_grid(&mut b), Status::Success);
        assert!(is_closest_to_ball(&mut b));

        b.parameters.kicking.kick_activation_distance = 1.1;
        b.parameters.kicking.kick_activation_hysteresis = 0.2;
        b.parameters.ball.last_ball_timeout = Duration::from_secs(1);
        b.ball_action = Some(BallAction::Clear);
        assert_eq!(execute_ball_action(&mut b), Status::Success);
        assert!(!matches!(b.body_motion, Some(BodyMotion::Kick { .. })));
        assert!(fresh_teammate_poses(&b).any(|(player, _)| player == PlayerNumber::Two));

        let mut clear_body_route = b.clone();
        clear_body_route.world_state.player_states.two = None;
        clear_body_route.timed_player_states.two = None;
        clear_body_route.body_motion = None;
        assert_eq!(execute_ball_action(&mut clear_body_route), Status::Success);
        assert!(matches!(
            clear_body_route.body_motion,
            Some(BodyMotion::Kick { .. })
        ));

        teammate(&mut b, PlayerNumber::Two, point![0.45, 0.0], false);
        assert_eq!(calculate_voronoi_grid(&mut b), Status::Success);
        assert!(!is_closest_to_ball(&mut b));
    }
}
