use color_eyre::Result;
use linear_algebra::{Point2, point};
use serde::{Deserialize, Serialize};

use context_attribute::context;
use coordinate_systems::{Field, Ground};
use framework::{AdditionalOutput, MainOutput};
use types::{
    action::Action,
    ball_position::BallPosition,
    dribble_path_plan::DribblePathPlan,
    field_dimensions::{FieldDimensions, Side},
    kick_decision::DecisionParameters,
    motion_command::{MotionCommand, WalkSpeed},
    parameters::{
        BehaviorParameters, InWalkKicksParameters, LostBallParameters, WalkSpeedParameters,
    },
    path_obstacles::PathObstacle,
    primary_state::PrimaryState,
    world_state::WorldState,
};

use crate::behavior::lost_ball;

use super::{
    defend::defend::{Defend, DefendMode},
    dribble, finish,
    head::LookAction,
    initial, look_around, penalize, remote_control, safe, stand_during_penalty_kick, stand_up,
    walk_to_ball, walk_to_kick_off, walk_to_penalty_kick,
    walk_to_pose::{WalkAndStand, WalkPathPlanner},
};

#[derive(Deserialize, Serialize)]
pub struct Behavior {
    last_defender_mode: DefendMode,
    last_known_ball_position: Point2<Field>,
}

#[context]
pub struct CreationContext {}

#[context]
pub struct CycleContext {
    ball_position: Input<Option<BallPosition<Ground>>, "ball_position?">,
    dribble_path_plan: Input<Option<DribblePathPlan>, "dribble_path_plan?">,
    world_state: Input<WorldState, "world_state">,

    defend_walk_speed: Parameter<WalkSpeed, "walk_speed.defend">,
    field_dimensions: Parameter<FieldDimensions, "field_dimensions">,
    in_walk_kicks: Parameter<InWalkKicksParameters, "in_walk_kicks">, // TODO: kommt durch kick_selector
    kick_decision_parameters: Parameter<DecisionParameters, "kick_selector">, // TODO: kommt durch kick_selector
    lost_ball_parameters: Parameter<LostBallParameters, "behavior.lost_ball">,
    parameters: Parameter<BehaviorParameters, "behavior">,
    walk_speed: Parameter<WalkSpeedParameters, "walk_speed">,

    path_obstacles_output: AdditionalOutput<Vec<PathObstacle>, "path_obstacles">, // TODO: kommt durch path_planner
    active_action: AdditionalOutput<Action, "active_action">,

    last_motion_command: CyclerState<MotionCommand, "last_motion_command">,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    pub motion_command: MainOutput<MotionCommand>,
}

impl Behavior {
    pub fn new(_context: CreationContext) -> Result<Self> {
        Ok(Self {
            last_defender_mode: DefendMode::Passive,
            last_known_ball_position: point![0.0, 0.0],
        })
    }

    pub fn cycle(&mut self, mut context: CycleContext) -> Result<MainOutputs> {
        let world_state = context.world_state;

        if let Some(command) = &context.parameters.injected_motion_command {
            return Ok(MainOutputs {
                motion_command: command.clone().into(),
            });
        }

        let mut actions = vec![
            Action::Safe,
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
        let look_action = LookAction::new(world_state);
        let mut defend = Defend::new(
            world_state,
            context.field_dimensions,
            &context.parameters.role_positions,
            &walk_and_stand,
            &look_action,
            &mut self.last_defender_mode,
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

                    Action::DefendGoal => defend.goal(
                        &mut context.path_obstacles_output,
                        *context.defend_walk_speed,
                        context
                            .parameters
                            .walk_and_stand
                            .defender_distance_to_be_aligned,
                    ),
                    Action::DefendKickOff => defend.kick_off(
                        &mut context.path_obstacles_output,
                        *context.defend_walk_speed,
                        context
                            .parameters
                            .walk_and_stand
                            .defender_distance_to_be_aligned,
                    ),
                    Action::DefendLeft => defend.left(
                        &mut context.path_obstacles_output,
                        *context.defend_walk_speed,
                        context
                            .parameters
                            .walk_and_stand
                            .defender_distance_to_be_aligned,
                    ),
                    Action::DefendPenaltyKick => defend.penalty_kick(
                        &mut context.path_obstacles_output,
                        *context.defend_walk_speed,
                        context
                            .parameters
                            .walk_and_stand
                            .defender_distance_to_be_aligned,
                    ),
                    Action::DefendOpponentCornerKick { side: Side::Left } => defend
                        .opponent_corner_kick(
                            &mut context.path_obstacles_output,
                            *context.defend_walk_speed,
                            Side::Left,
                            context
                                .parameters
                                .walk_and_stand
                                .defender_distance_to_be_aligned,
                        ),
                    Action::DefendOpponentCornerKick { side: Side::Right } => defend
                        .opponent_corner_kick(
                            &mut context.path_obstacles_output,
                            *context.defend_walk_speed,
                            Side::Right,
                            context
                                .parameters
                                .walk_and_stand
                                .defender_distance_to_be_aligned,
                        ),

                    Action::StandDuringPenaltyKick => stand_during_penalty_kick::execute(
                        world_state,
                        context.field_dimensions,
                        &context.world_state.robot.role,
                    ),
                    Action::Dribble => dribble::execute(
                        world_state,
                        &walk_path_planner,
                        context.in_walk_kicks,
                        &context.parameters.dribbling,
                        context.dribble_path_plan.cloned(),
                        context.walk_speed.dribble,
                        context.parameters.dribbling.distance_to_be_aligned,
                    ),

                    Action::SearchForLostBall => lost_ball::execute(
                        world_state,
                        self.last_known_ball_position,
                        &walk_path_planner,
                        context.lost_ball_parameters,
                        &mut context.path_obstacles_output,
                        context.walk_speed.lost_ball,
                        context
                            .parameters
                            .walk_and_stand
                            .normal_distance_to_be_aligned,
                    ),

                    Action::WalkToKickOff => walk_to_kick_off::execute(
                        world_state,
                        &walk_and_stand,
                        &look_action,
                        &mut context.path_obstacles_output,
                        context.parameters.role_positions.striker_kickoff_position,
                        context.kick_decision_parameters.kick_off_angle,
                        context.walk_speed.walk_to_kickoff,
                        context
                            .parameters
                            .walk_and_stand
                            .normal_distance_to_be_aligned,
                    ),
                    Action::WalkToPenaltyKick => walk_to_penalty_kick::execute(
                        world_state,
                        &walk_and_stand,
                        &look_action,
                        &mut context.path_obstacles_output,
                        context.field_dimensions,
                        context.walk_speed.walk_to_penalty_kick,
                        context
                            .parameters
                            .walk_and_stand
                            .normal_distance_to_be_aligned,
                    ),

                    Action::WalkToBall => walk_to_ball::execute(
                        context.ball_position.copied(),
                        context.parameters.walk_with_velocity.clone(),
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
