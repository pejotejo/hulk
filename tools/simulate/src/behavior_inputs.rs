//! Ground-truth substitutes for perception/localization, with real downstream composers.
use std::f32::consts::PI;

use color_eyre::Result;
use coordinate_systems::{Field, Ground, Odometry};
use hsl_network_messages::PlayerNumber;
use linear_algebra::{Isometry2, Point2, Vector2, point, vector};
use ros_z::{Message, prelude::*, time::Time};
use serde::{Deserialize, Serialize};
use types::{
    ball_position::BallPosition,
    field_dimensions::{GlobalFieldSide, Half, Side},
    multivariate_normal_distribution::MultivariateNormalDistribution,
    obstacles::Obstacle,
    tracked_objects::{
        HumanoidRole, HumanoidTrack, TeammateProbability, TrackStatus, TrackedObjects,
    },
};

use crate::{observations::Frame, profiles::Profile, reference::Reference, team::TeamId};

/// Physical truth of another robot, as seen from the robot publishing it.
#[derive(Clone, Debug, Deserialize, Serialize, Message)]
pub struct HumanoidTruth {
    pub team: TruthTeam,
    pub player_number: u8,
    /// Whether the robot is on the observer's team.
    pub is_teammate: bool,
    pub position_in_ground: Point2<Ground>,
    pub position_in_field: Point2<Field>,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize, Message)]
pub enum TruthTeam {
    Hulks,
    Opponents,
}

pub struct BehaviorInputs {
    pose: Reference<Isometry2<Ground, Field>>,
    ball: Reference<Option<BallPosition<Ground>>>,
    visual_ball: Reference<Option<BallPosition<Ground>>>,
    obstacles: Reference<Vec<Obstacle>>,
    humanoids: Reference<Vec<HumanoidTruth>>,
    humanoid_tracks: Reference<TrackedObjects<HumanoidTrack>>,
    interest: Publisher<Point2<Ground>>,
    selected_ball: ros_z::cache::Cache<Option<BallPosition<Ground>>>,
}

impl BehaviorInputs {
    pub async fn new(node: &Node, profile: Profile) -> Result<Self> {
        Ok(Self {
            pose: Reference::new(node, "ground_to_field", profile).await?,
            ball: Reference::new(node, "ball_filter/ball_position", profile).await?,
            visual_ball: Reference::new(node, "visual_kick/ball_position", profile).await?,
            obstacles: Reference::new(node, "obstacles", profile).await?,
            humanoids: Reference::new(node, "humanoids", profile).await?,
            humanoid_tracks: Reference::new(node, "obstacle_filter/humanoid_tracks", profile)
                .await?,
            selected_ball: node
                .subscriber("ball_filter/ball_position")
                .cache(1)
                .build()
                .await?,
            interest: node.publisher("position_of_interest").build().await?,
        })
    }

    pub async fn publish(
        &self,
        frame: &Frame,
        side: GlobalFieldSide,
        ground_to_odometry: Isometry2<Ground, Odometry>,
    ) -> Result<()> {
        let ground_to_world = frame.sample.ground_to_world;
        let time = frame.time;
        let pose = ground_to_field(ground_to_world, side);
        let ball = frame
            .balls
            .first()
            .map(|ball| ball_in_ground(ground_to_world, ball.position, ball.velocity, time));
        let obstacles_needed = self.obstacles.needed();
        let mut obstacles = Vec::new();
        if obstacles_needed {
            let goal_post_obstacles = goal_posts_in_ground(&frame.field, side, ground_to_world);
            obstacles.extend(goal_post_obstacles);
        }
        if obstacles_needed {
            let inverse = ground_to_world.inverse();
            let robot_obstacles: Vec<_> = frame
                .robots
                .iter()
                .map(|position| {
                    let p = inverse * nalgebra::Point3::from(*position);
                    Obstacle::robot(point![p.x, p.y], 0.2, 0.25)
                })
                .collect();
            obstacles.extend(robot_obstacles);
        }
        if self.humanoids.needed() || self.humanoid_tracks.needed() {
            let inverse = ground_to_world.inverse();
            let world_to_field = world_to_field(side);
            let humanoids: Vec<_> = frame
                .robots
                .iter()
                .zip(&frame.robot_ids)
                .map(|(position, id)| {
                    let ground = inverse * nalgebra::Point3::from(*position);
                    let field = world_to_field * nalgebra::point![position[0], position[1]];
                    HumanoidTruth {
                        team: match id.team {
                            TeamId::Hulks => TruthTeam::Hulks,
                            TeamId::Opponents => TruthTeam::Opponents,
                        },
                        player_number: id.number,
                        is_teammate: id.team == frame.observer.team,
                        position_in_ground: point![ground.x, ground.y],
                        position_in_field: point![field.x, field.y],
                    }
                })
                .collect();
            if self.humanoid_tracks.needed() {
                self.humanoid_tracks
                    .publish(
                        &identified_tracks(&humanoids, ground_to_odometry, pose, time),
                        time,
                    )
                    .await?;
            }
            self.humanoids.publish(&humanoids, time).await?;
        }
        self.pose.publish(&pose, time).await?;
        self.ball.publish(&ball, time).await?;
        self.visual_ball.publish(&ball, time).await?;
        if obstacles_needed {
            self.obstacles.publish(&obstacles, time).await?;
        }
        self.interest
            .publish_with_source_time(
                &self
                    .selected_ball
                    .get_latest()
                    .and_then(|ball| *ball)
                    .map_or(point![1.0, 0.0], |b| b.position),
                time,
            )
            .await?;
        Ok(())
    }
}

fn identified_tracks(
    humanoids: &[HumanoidTruth],
    ground_to_odometry: Isometry2<Ground, Odometry>,
    ground_to_field: Isometry2<Ground, Field>,
    time: Time,
) -> TrackedObjects<HumanoidTrack> {
    let players = [
        PlayerNumber::One,
        PlayerNumber::Two,
        PlayerNumber::Three,
        PlayerNumber::Four,
        PlayerNumber::Five,
    ];
    let tracks = humanoids
        .iter()
        .map(|humanoid| {
            let teammate = humanoid
                .player_number
                .checked_sub(1)
                .and_then(|index| players.get(usize::from(index)))
                .copied()
                .filter(|_| humanoid.is_teammate);
            let position = ground_to_odometry * humanoid.position_in_ground;
            HumanoidTrack {
                id: ((humanoid.team as u64) << 8) | u64::from(humanoid.player_number),
                existence: 1.0,
                status: TrackStatus::Confirmed,
                robot_probability: 1.0,
                human_probability: 0.0,
                role: teammate.map_or(HumanoidRole::Opponent, HumanoidRole::Teammate),
                teammate_probabilities: teammate
                    .into_iter()
                    .map(|player_number| TeammateProbability {
                        player_number,
                        probability: 1.0,
                    })
                    .collect(),
                remote_only: false,
                detection_probability: 1.0,
                time_since_detection: Some(std::time::Duration::ZERO),
                // shortcut: truth supplies positions only; add velocity when behavior consumes it.
                state: MultivariateNormalDistribution {
                    mean: nalgebra::vector![position.x(), position.y(), 0.0, 0.0],
                    covariance: nalgebra::Matrix4::zeros(),
                },
                position_in_ground: humanoid.position_in_ground,
                velocity_in_ground: Vector2::zeros(),
                position_in_field: Some(humanoid.position_in_field),
            }
        })
        .collect();
    TrackedObjects {
        time,
        ground_to_odometry,
        ground_to_field: Some(ground_to_field),
        tracks,
    }
}

fn goal_posts_in_ground(
    field: &types::field_dimensions::FieldDimensions,
    side: GlobalFieldSide,
    ground_to_world: nalgebra::Isometry3<f32>,
) -> Vec<Obstacle> {
    let field_to_world = world_to_field(side).inverse();
    let world_to_ground = ground_to_world.inverse();
    [Half::Own, Half::Opponent]
        .into_iter()
        .flat_map(|half| {
            [Side::Left, Side::Right].map(|field_side| {
                let position = field.goal_post(half, field_side);
                let world = field_to_world * nalgebra::point![position.x(), position.y()];
                let ground = world_to_ground * nalgebra::point![world.x, world.y, 0.0];
                Obstacle::goal_post(point![ground.x, ground.y], field.goal_post_diameter / 2.0)
            })
        })
        .collect()
}

fn world_to_field(side: GlobalFieldSide) -> nalgebra::Isometry2<f32> {
    let rotation = if side == GlobalFieldSide::Home {
        0.0
    } else {
        PI
    };
    nalgebra::Isometry2::new(nalgebra::Vector2::zeros(), rotation)
}

fn ground_to_field(
    pose: nalgebra::Isometry3<f32>,
    side: GlobalFieldSide,
) -> Isometry2<Ground, Field> {
    let yaw = pose.rotation.euler_angles().2;
    let ground = nalgebra::Isometry2::new(
        nalgebra::vector![pose.translation.x, pose.translation.y],
        yaw,
    );
    let rotation = if side == GlobalFieldSide::Home {
        0.0
    } else {
        PI
    };
    Isometry2::wrap(nalgebra::Isometry2::new(nalgebra::Vector2::zeros(), rotation) * ground)
}

fn ball_in_ground(
    pose: nalgebra::Isometry3<f32>,
    position: [f64; 3],
    velocity: [f64; 3],
    time: Time,
) -> BallPosition<Ground> {
    let inverse = pose.inverse();
    let position = inverse * nalgebra::Point3::from(position.map(|v| v as f32));
    let velocity = inverse.rotation * nalgebra::Vector3::from(velocity.map(|v| v as f32));
    BallPosition {
        position: point![position.x, position.y],
        velocity: vector![velocity.x, velocity.y],
        last_seen: time,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identified_truth_keeps_roles_ids_time_and_odometry_frame() {
        let time = Time::from_nanos(42);
        let ground_to_odometry = Isometry2::from_parts(vector![2.0, -1.0], PI / 2.0);
        let ground_to_field = Isometry2::from_parts(vector![-3.0, 2.0], PI);
        let position = point![1.0, 0.5];
        let humanoids = [
            HumanoidTruth {
                team: TruthTeam::Hulks,
                player_number: 2,
                is_teammate: true,
                position_in_ground: position,
                position_in_field: ground_to_field * position,
            },
            HumanoidTruth {
                team: TruthTeam::Opponents,
                player_number: 2,
                is_teammate: false,
                position_in_ground: position,
                position_in_field: ground_to_field * position,
            },
        ];
        let tracks = identified_tracks(&humanoids, ground_to_odometry, ground_to_field, time);
        assert_eq!(tracks.time, time);
        assert_eq!(
            tracks.tracks[0].role,
            HumanoidRole::Teammate(PlayerNumber::Two)
        );
        assert_eq!(tracks.tracks[1].role, HumanoidRole::Opponent);
        assert_ne!(tracks.tracks[0].id, tracks.tracks[1].id);
        for track in &tracks.tracks {
            let in_odometry = ground_to_odometry * position;
            assert!(
                (track.state.mean.fixed_rows::<2>(0) - in_odometry.coords().inner).norm() < 1e-5
            );
            assert_eq!(track.position_in_ground, position);
            assert_eq!(track.position_in_field, Some(ground_to_field * position));
            assert_eq!(track.status, TrackStatus::Confirmed);
            assert_eq!(track.time_since_detection, Some(std::time::Duration::ZERO));
        }
        assert!(
            identified_tracks(&[], ground_to_odometry, ground_to_field, time)
                .tracks
                .is_empty()
        );
    }

    #[test]
    fn ground_truth_rotates_positions_velocities_and_field_side() {
        let pose = nalgebra::Isometry3::new(
            nalgebra::vector![2.0, 3.0, 0.0],
            nalgebra::vector![0.0, 0.0, PI / 2.0],
        );
        let time = Time::from_nanos(42);
        let ball = ball_in_ground(pose, [2.0, 4.0, 0.1], [0.0, 2.0, 0.0], time);
        assert!((ball.position - point![1.0, 0.0]).norm() < 1e-5);
        assert!((ball.velocity - vector![2.0, 0.0]).norm() < 1e-5);
        assert_eq!(ball.last_seen, time);
        for (side, sign) in [(GlobalFieldSide::Home, 1.0), (GlobalFieldSide::Away, -1.0)] {
            let field = ground_to_field(pose, side) * ball.position;
            assert!((field - point![2.0 * sign, 4.0 * sign]).norm() < 1e-5);
        }
    }

    #[test]
    fn goal_post_truth_agrees_with_the_field_pose_on_both_sides() {
        let field = types::field_dimensions::FieldDimensions::SPL_2025;
        let pose = nalgebra::Isometry3::new(
            nalgebra::vector![2.0, 3.0, 0.0],
            nalgebra::vector![0.0, 0.0, PI / 2.0],
        );
        for side in [GlobalFieldSide::Home, GlobalFieldSide::Away] {
            let ground_to_field = ground_to_field(pose, side);
            let mut in_field: Vec<_> = goal_posts_in_ground(&field, side, pose)
                .into_iter()
                .map(|post| ground_to_field * post.position)
                .collect();
            in_field.sort_by(|a, b| (a.x(), a.y()).partial_cmp(&(b.x(), b.y())).unwrap());
            let x = field.length / 2.0;
            let y = (field.goal_inner_width + field.goal_post_diameter) / 2.0;
            for (post, expected) in
                in_field
                    .iter()
                    .zip([point![-x, -y], point![-x, y], point![x, -y], point![x, y]])
            {
                assert!((*post - expected).norm() < 1e-4, "{side:?}: {post:?}");
            }
        }
    }
}
