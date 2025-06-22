use std::time::Duration;

use types::{
    motion_command::{ArmMotion, HeadMotion, ImageRegion, KickVariant, MotionCommand},
    primary_state::{PrimaryState, RampDirection},
    support_foot::Side,
    world_state::WorldState,
};

#[allow(clippy::too_many_arguments)]
pub fn execute(
    world_state: &WorldState,
    time_to_reach_foot: &Duration,
    step_duration: &Duration,
    kick_strength: &f32,
    kick_start_threshold: f32,
) -> Option<MotionCommand> {
    match (world_state.robot.primary_state, world_state.ball) {
        (
            PrimaryState::KickingRollingBall {
                ramp_direction: RampDirection::Right,
            },
            None,
        ) => Some(MotionCommand::Stand {
            head: HeadMotion::SearchRight,
        }),
        (
            PrimaryState::KickingRollingBall {
                ramp_direction: RampDirection::Left,
            },
            None,
        ) => Some(MotionCommand::Stand {
            head: HeadMotion::SearchLeft,
        }),
        (PrimaryState::KickingRollingBall { ramp_direction }, _) => {
            let image_region_target = match ramp_direction {
                RampDirection::Left => ImageRegion::TopLeft,
                RampDirection::Right => ImageRegion::TopRight,
            };
            let head = HeadMotion::LookAt {
                target: world_state.ball?.ball_in_ground,
                image_region_target,
                camera: None,
            };
            if time_to_reach_foot.as_secs_f32() - step_duration.as_secs_f32() < kick_start_threshold
            {
                let kicking_side = match ramp_direction {
                    RampDirection::Left => Side::Right,
                    RampDirection::Right => Side::Left,
                };

                let command = MotionCommand::InWalkKick {
                    head,
                    kick: KickVariant::Forward,
                    kicking_side,
                    strength: *kick_strength,
                    left_arm: ArmMotion::Swing,
                    right_arm: ArmMotion::Swing,
                };
                return Some(command);
            }
            Some(MotionCommand::Stand { head })
        }
        _ => None,
    }
}
