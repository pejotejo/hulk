use coordinate_systems::{Ground, UpcomingSupport};
use linear_algebra::{Isometry2, Pose2};
use types::{
    motion_command::{ArmMotion, HeadMotion, ImageRegion, MotionCommand},
    parameters::{InWalkKickInfoParameters, InWalkKicksParameters},
    primary_state::{PrimaryState, RampDirection},
    world_state::WorldState,
};


#[allow(clippy::too_many_arguments)]
pub fn execute(
    world_state: &WorldState,
    in_walk_kicks: &InWalkKicksParameters,
) -> Option<MotionCommand> {
    //     let ball_position = world_state.ball?.ball_in_ground;
    // let distance_to_ball = ball_position.coords().norm();
    // let head = if distance_to_ball < parameters.distance_to_look_directly_at_the_ball {
    //     HeadMotion::LookAt {
    //         target: ball_position,
    //         image_region_target: ImageRegion::Center,
    //         camera: Some(CameraPosition::Bottom),
    //     }
    // } else {
    //     HeadMotion::LookLeftAndRightOf {
    //         target: ball_position,
    //     }
    // };
    dbg!("Phillip");
    dbg!(world_state.ball);
    dbg!(world_state.robot.primary_state);
    match (world_state.robot.primary_state, world_state.ball) {
        (PrimaryState::KickingRollingBall{ramp_direction: RampDirection::Right}, None) => Some(MotionCommand::Stand {
            head: HeadMotion::SearchRight,
        }),

        (PrimaryState::KickingRollingBall{ramp_direction: RampDirection::Left}, None) => Some(MotionCommand::Stand {
            head: HeadMotion::SearchLeft,
        }),
        (PrimaryState::KickingRollingBall{..}, _) => {
            let head = HeadMotion::LookAt {
                target: world_state.ball?.ball_in_ground,
                image_region_target: ImageRegion::Center,
                camera: None,
            };
            let kick_decisions = world_state.kick_decisions.as_ref()?;
            let instant_kick_decisions = world_state.instant_kick_decisions.as_ref()?;

            let available_kick = kick_decisions
                .iter()
                .chain(instant_kick_decisions.iter())
                .find(|decision| {
                    is_kick_pose_reached(
                        decision.kick_pose,
                        &in_walk_kicks[decision.variant],
                        world_state.robot.ground_to_upcoming_support,
                    )
                });
            if let Some(kick) = available_kick {
                let command = MotionCommand::InWalkKick {
                    head,
                    kick: kick.variant,
                    kicking_side: kick.kicking_side,
                    strength: kick.strength,
                    left_arm: ArmMotion::Swing,
                    right_arm: ArmMotion::Swing,
                };
                return Some(command);
            }
            Some(MotionCommand::Stand { head })
        }
        _ => None
    }
}

fn is_kick_pose_reached(
    kick_pose: Pose2<Ground>,
    kick_info: &InWalkKickInfoParameters,
    ground_to_upcoming_support: Isometry2<Ground, UpcomingSupport>,
) -> bool {
    let upcoming_kick_pose = ground_to_upcoming_support * kick_pose;
    let is_x_reached = kick_info
        .reached_x
        .contains(&upcoming_kick_pose.position().x());
    let is_y_reached = kick_info
        .reached_y
        .contains(&upcoming_kick_pose.position().y());
    let is_orientation_reached = kick_info
        .reached_turn
        .contains(&upcoming_kick_pose.orientation().angle());
    is_x_reached && is_y_reached && is_orientation_reached
}
