use coordinate_systems::Ground;
use framework::AdditionalOutput;
use linear_algebra::{Pose2, Vector2};
use types::{
    ball_position::BallPosition,
    motion_command::{HeadMotion, ImageRegion, MotionCommand, OrientationMode, WalkSpeed},
    path_obstacles::PathObstacle,
};

use crate::behavior::walk_to_pose::WalkAndStand;

pub fn execute(
    ball_position: Option<BallPosition<Ground>>,
    path_obstacles_output: &mut AdditionalOutput<Vec<PathObstacle>>,
    walk_and_stand: &WalkAndStand,
) -> Option<MotionCommand> {
    match ball_position {
        Some(ball_position) => walk_and_stand.execute(
            Pose2::from(ball_position.position),
            HeadMotion::LookAt {
                target: ball_position.position,
                image_region_target: ImageRegion::Top,
            },
            path_obstacles_output,
            WalkSpeed::Normal,
            OrientationMode::AlignWithPath,
            0.0,
            walk_and_stand.parameters.hysteresis,
        ),
        None => Some(MotionCommand::WalkWithVelocity {
            velocity: Vector2::zeros(),
            angular_velocity: 0.0,
            head: HeadMotion::Center {
                image_region_target: ImageRegion::Top,
            },
        }),
    }
}
