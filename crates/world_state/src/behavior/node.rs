use color_eyre::Result;
use serde::{Deserialize, Serialize};

use context_attribute::context;
use coordinate_systems::Ground;
use framework::{AdditionalOutput, MainOutput};
use types::{
    action::Action, ball_position::BallPosition, field_dimensions::FieldDimensions,
    motion_command::MotionCommand, parameters::BehaviorParameters, path_obstacles::PathObstacle,
    primary_state::PrimaryState, world_state::WorldState,
};

use crate::behavior::{
    finish, initial, look_around, penalize, remote_control, safe, stand_up, walk_to_ball_two,
    walk_to_pose::{WalkAndStand, WalkPathPlanner},
};

#[derive(Deserialize, Serialize)]
pub struct Behavior {}

#[context]
pub struct CreationContext {}

#[context]
pub struct CycleContext {
    ball_position: Input<Option<BallPosition<Ground>>, "ball_position?">,
    world_state: Input<WorldState, "world_state">,

    field_dimensions: Parameter<FieldDimensions, "field_dimensions">,
    parameters: Parameter<BehaviorParameters, "behavior">,

    active_action: AdditionalOutput<Action, "active_action">,
    path_obstacles_output: AdditionalOutput<Vec<PathObstacle>, "path_obstacles">,

    last_motion_command: CyclerState<MotionCommand, "last_motion_command">,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    pub motion_command: MainOutput<MotionCommand>,
}

impl Behavior {
    pub fn new(_context: CreationContext) -> Result<Self> {
        Ok(Self {})
    }

    pub fn cycle(&mut self, mut context: CycleContext) -> Result<MainOutputs> {
        let world_state = context.world_state;

        if let Some(command) = &context.parameters.injected_motion_command {
            return Ok(MainOutputs {
                motion_command: command.clone().into(),
            });
        }

        let mut actions = vec![
            // Action::Safe,
            Action::Finish,
            Action::Penalize,
            Action::Initial,
        ];

        if context.parameters.remote_control.enable {
            actions.insert(0, Action::RemoteControl);
        }

        if world_state.robot.primary_state == PrimaryState::Playing {
            actions.push(Action::WalkToBall);
        }

        let walk_path_planner = WalkPathPlanner::new(
            context.field_dimensions,
            &world_state.obstacles,
            &context.parameters.path_planning,
            context.last_motion_command,
        );

        let walk_and_stand = WalkAndStand::new(
            world_state,
            &context.parameters.walk_and_stand,
            &walk_path_planner,
            context.last_motion_command,
        );

        let (action, motion_command) = actions
            .iter()
            .find_map(|action| {
                let motion_command = match action {
                    Action::Safe => safe::execute(world_state),
                    Action::Penalize => penalize::execute(world_state),
                    Action::Initial => initial::execute(world_state),
                    Action::Finish => finish::execute(world_state),
                    Action::StandUp => stand_up::execute(world_state),
                    Action::LookAround => look_around::execute(world_state),
                    Action::RemoteControl => {
                        remote_control::execute(&context.parameters.remote_control)
                    }
                    Action::WalkToBall => walk_to_ball_two::execute(
                        context.ball_position.copied(),
                        &mut context.path_obstacles_output,
                        &walk_and_stand,
                    ),
                }?;
                Some((action, motion_command))
            })
            .unwrap_or_else(|| panic!("there has to be at least one action available",));
        context.active_action.fill_if_subscribed(|| *action);

        *context.last_motion_command = motion_command.clone();

        Ok(MainOutputs {
            motion_command: motion_command.into(),
        })
    }
}
