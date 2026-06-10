use std::time::{Duration, SystemTime};

use coordinate_systems::Field;
use hsl_network_messages::PlayerNumber;
use linear_algebra::{Point2, distance};
use serde::{Deserialize, Serialize};
use types::world_state::WorldState;

const STRIKER_OWNERSHIP_HYSTERESIS: f32 = 0.25;

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize)]
pub struct StrikerMemory {
    pub owner: Option<PlayerNumber>,
}

pub fn select_striker(
    world_state: &WorldState,
    ball: Point2<Field>,
    now: SystemTime,
    freshness_timeout: Duration,
    goal_keeper_number: PlayerNumber,
    previous: StrikerMemory,
) -> Option<PlayerNumber> {
    let own_player_number = world_state.robot.player_number;
    let mut best: Option<(PlayerNumber, f32)> = None;
    let mut previous_owner_candidate: Option<(PlayerNumber, f32)> = None;

    for (player_number, player_state) in world_state.player_states.iter() {
        if player_number == goal_keeper_number {
            continue;
        }

        let position = if player_number == own_player_number {
            world_state
                .robot
                .ground_to_field
                .as_ref()
                .map(|ground_to_field| ground_to_field.translation())
        } else {
            let Some(player_state) = player_state else {
                continue;
            };
            let age = now
                .duration_since(player_state.last_seen)
                .unwrap_or(Duration::ZERO);
            if age > freshness_timeout {
                continue;
            }
            Some(player_state.pose.position())
        };
        let Some(position) = position else {
            continue;
        };
        let distance_to_ball = distance(position, ball);

        if previous.owner == Some(player_number) {
            previous_owner_candidate = Some((player_number, distance_to_ball));
        }

        match best {
            Some((_, best_distance)) if distance_to_ball >= best_distance => {}
            _ => best = Some((player_number, distance_to_ball)),
        }
    }

    let (best_owner, best_distance) = best?;
    if let Some((previous_owner, previous_distance)) = previous_owner_candidate
        && previous_distance <= best_distance + STRIKER_OWNERSHIP_HYSTERESIS
    {
        return Some(previous_owner);
    }

    Some(best_owner)
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, SystemTime};

    use coordinate_systems::{Field, Ground};
    use hsl_network_messages::PlayerNumber;
    use linear_algebra::{Isometry2, Pose2, point, vector};
    use types::{players::Players, world_state::PlayerState};

    use super::*;

    fn world_state_with_own_pose(
        player_number: PlayerNumber,
        position: Point2<Field>,
    ) -> WorldState {
        let mut world_state = WorldState::default();
        world_state.robot.player_number = player_number;
        world_state.robot.ground_to_field = Some(ground_to_field(position.x(), position.y()));
        world_state
    }

    fn ground_to_field(x: f32, y: f32) -> Isometry2<Ground, Field> {
        Isometry2::from_parts(vector![x, y], 0.0)
    }

    fn player_state(x: f32, y: f32, last_seen: SystemTime) -> PlayerState {
        PlayerState {
            pose: Pose2::from(point![x, y]),
            ball_position: None,
            coordination: None,
            last_seen,
        }
    }

    #[test]
    fn own_robot_wins_when_closest() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let mut world_state = world_state_with_own_pose(PlayerNumber::Two, point![0.0, 0.0]);
        world_state.player_states = Players {
            three: Some(player_state(2.0, 0.0, now)),
            ..Players::new(None)
        };

        let owner = select_striker(
            &world_state,
            point![0.2, 0.0],
            now,
            Duration::from_secs(1),
            PlayerNumber::One,
            StrikerMemory::default(),
        );

        assert_eq!(owner, Some(PlayerNumber::Two));
    }

    #[test]
    fn fresh_teammate_closer_wins() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let mut world_state = world_state_with_own_pose(PlayerNumber::Two, point![2.0, 0.0]);
        world_state.player_states = Players {
            four: Some(player_state(0.1, 0.0, now)),
            ..Players::new(None)
        };

        let owner = select_striker(
            &world_state,
            point![0.0, 0.0],
            now,
            Duration::from_secs(1),
            PlayerNumber::One,
            StrikerMemory::default(),
        );

        assert_eq!(owner, Some(PlayerNumber::Four));
    }

    #[test]
    fn stale_teammate_is_ignored() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let mut world_state = world_state_with_own_pose(PlayerNumber::Two, point![1.0, 0.0]);
        world_state.player_states = Players {
            three: Some(player_state(0.1, 0.0, now - Duration::from_secs(2))),
            ..Players::new(None)
        };

        let owner = select_striker(
            &world_state,
            point![0.0, 0.0],
            now,
            Duration::from_secs(1),
            PlayerNumber::One,
            StrikerMemory::default(),
        );

        assert_eq!(owner, Some(PlayerNumber::Two));
    }

    #[test]
    fn previous_owner_is_kept_when_within_hysteresis_of_best() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let mut world_state = world_state_with_own_pose(PlayerNumber::Two, point![1.0, 0.0]);
        world_state.player_states = Players {
            three: Some(player_state(1.2, 0.0, now)),
            ..Players::new(None)
        };

        let owner = select_striker(
            &world_state,
            point![0.0, 0.0],
            now,
            Duration::from_secs(1),
            PlayerNumber::One,
            StrikerMemory {
                owner: Some(PlayerNumber::Three),
            },
        );

        assert_eq!(owner, Some(PlayerNumber::Three));
    }

    #[test]
    fn goalkeeper_closest_is_ignored_for_striker_selection() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let mut world_state = world_state_with_own_pose(PlayerNumber::Two, point![1.0, 0.0]);
        world_state.player_states = Players {
            one: Some(player_state(0.1, 0.0, now)),
            ..Players::new(None)
        };

        let owner = select_striker(
            &world_state,
            point![0.0, 0.0],
            now,
            Duration::from_secs(1),
            PlayerNumber::One,
            StrikerMemory::default(),
        );

        assert_eq!(owner, Some(PlayerNumber::Two));
    }
}
