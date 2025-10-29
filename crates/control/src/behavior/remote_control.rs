use linear_algebra::vector;
use types::motion_command::{HeadMotion, MotionCommand};

use crate::behavior::node::RemoteControlParameters;



pub fn execute(remote_control_parameters: &RemoteControlParameters) -> Option<MotionCommand> {
    Some(MotionCommand::WalkWithVelocity {
        head: HeadMotion::Center,
        velocity: vector!(
            remote_control_parameters.walk.forward,
            remote_control_parameters.walk.left,
            remote_control_parameters.walk.turn
        ),
    })
}
