use types::{
    motion_command::{HeadMotion, ImageRegion, MotionCommand},
    primary_state::PrimaryState,
    world_state::WorldState,
};


    pub fn execute(world_state: &WorldState) -> Option<MotionCommand> {
        match (world_state.robot.primary_state, world_state.ball) {
            (PrimaryState::KickingRollingBall, None) => Some(MotionCommand::Stand {
                head: HeadMotion::SearchRight,
            }),
            (PrimaryState::KickingRollingBall, _) => Some(MotionCommand::Stand {
                head: HeadMotion::LookAt {
                    target: world_state.ball?.ball_in_ground,
                    image_region_target: ImageRegion::Center,
                    camera: None,
                },
            }),
            _ => None,
        }
    }
