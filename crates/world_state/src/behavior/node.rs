use std::net::SocketAddr;
use std::time::{Duration, SystemTime};

use color_eyre::Result;

use context_attribute::context;
use coordinate_systems::{Field, Ground};
use framework::{AdditionalOutput, MainOutput};
use hardware::NetworkInterface;
use hsl_network_messages::HulkMessage;
use linear_algebra::{Point2, Pose2, Vector2};
use serde::{Deserialize, Serialize};
use types::{
    behavior_tree::NodeTrace,
    field_dimensions::{FieldDimensions, Side},
    motion_command::{BodyMotion, HeadMotion, MotionCommand},
    motion_type::MotionType,
    parameters::{BehaviorParameters, HslNetworkParameters},
    path_obstacles::PathObstacle,
    world_state::WorldState,
};
use voronoi::VoronoiGrid;

use crate::behavior::{
    behavior_tree::Node,
    kick_selector::{CycleIntent, KickMemory, outgoing_coordination_intent, select_cycle_intent},
    motion_assembler::assemble_motion_command,
    role_arbitration::{StrikerMemory, select_striker},
    tree::create_tree,
};

fn create_tree_default() -> Node<Blackboard> {
    create_tree()
}

fn create_static_layout_default() -> NodeTrace {
    create_tree().static_layout_trace()
}

fn persistent_kick_target(
    remote_kick_mode: bool,
    cycle_intent: CycleIntent,
    last_kick_target: Option<Point2<Field>>,
) -> Option<Point2<Field>> {
    if remote_kick_mode {
        last_kick_target
    } else {
        cycle_intent
            .kick
            .map(|kick| kick.target)
            .or(last_kick_target)
    }
}

#[derive(Serialize, Deserialize)]
pub struct Behavior {
    pub ball: Option<LastBall>,
    pub last_ball: Option<LastBall>,
    pub last_close_enough_to_kick: bool,
    pub last_kick_target: Option<Point2<Field>>,
    #[serde(default)]
    pub kick_memory: KickMemory,
    #[serde(default)]
    pub striker_memory: StrikerMemory,
    pub last_motion_switch_time: SystemTime,
    pub last_motion_type: Option<MotionType>,
    #[serde(skip, default = "create_tree_default")]
    pub tree: Node<Blackboard>,
    #[serde(skip, default = "create_static_layout_default")]
    pub static_layout: NodeTrace,
    pub last_sent_game_controller_return_message_time: Option<SystemTime>,
    pub last_sent_hsl_message_time: Option<SystemTime>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LastBall {
    pub position: Point2<Field>,
    pub velocity: Vector2<Ground>,
    pub age: SystemTime,
    pub field_side: Side,
}

#[derive(Debug, Clone, Serialize)]
pub struct Blackboard {
    pub field_dimensions: FieldDimensions,
    pub free_kick_obstacle_radius: f32,
    pub pass_intent_timeout: Duration,
    pub parameters: BehaviorParameters,
    pub world_state: WorldState,

    pub path_obstacles_output: Vec<PathObstacle>,
    pub time_since_last_switch: Duration,
    pub direction_difference: f32,
    pub voronoi_inputs: Vec<Pose2<Field>>,

    pub ball: Option<LastBall>,
    pub last_ball: Option<LastBall>,
    pub last_close_enough_to_kick: bool,
    pub last_kick_target: Option<Point2<Field>>,
    pub cycle_intent: CycleIntent,
    pub kick_memory: KickMemory,
    pub last_motion_command: MotionCommand,
    pub last_motion_switch_time: SystemTime,
    pub last_motion_type: Option<MotionType>,

    pub is_injected_motion_command: bool,
    pub walk_position: Option<Point2<Ground>>,
    pub body_motion: Option<BodyMotion>,
    pub head_motion: Option<HeadMotion>,
    pub voronoi_map: Option<VoronoiGrid>,
}

#[context]
pub struct CreationContext {}

#[context]
pub struct CycleContext {
    game_controller_address: Input<Option<SocketAddr>, "game_controller_address?">,
    remaining_amount_of_messages:
        Input<Option<u16>, "game_controller_state?.hulks_team.remaining_amount_of_messages">,
    world_state: Input<WorldState, "world_state">,

    field_dimensions: Parameter<FieldDimensions, "field_dimensions">,
    hsl_network_parameters: Parameter<HslNetworkParameters, "hsl_network">,
    parameters: Parameter<BehaviorParameters, "behavior">,
    free_kick_obstacle_radius: Parameter<f32, "rule_obstacles.free_kick_obstacle_radius">,

    behavior_trace: AdditionalOutput<NodeTrace, "behavior.trace">,
    behavior_tree_layout: AdditionalOutput<NodeTrace, "behavior.tree_layout">,
    last_sent_message: AdditionalOutput<HulkMessage, "last_sent_message">,
    path_obstacles_output: AdditionalOutput<Vec<PathObstacle>, "path_obstacles">,
    time_since_last_switch: AdditionalOutput<Duration, "behavior.time_since_last_switch">,
    direction_difference: AdditionalOutput<f32, "behavior.direction_difference">,
    walk_position: AdditionalOutput<Option<Point2<Ground>>, "behavior.walk_position">,
    voronoi_map: AdditionalOutput<Option<VoronoiGrid>, "behavior.voronoi_map">,
    voronoi_inputs: AdditionalOutput<Vec<Pose2<Field>>, "behavior.voronoi_inputs">,

    last_motion_command: CyclerState<MotionCommand, "last_motion_command">,

    hardware: HardwareInterface,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    pub motion_command: MainOutput<MotionCommand>,
}

impl Behavior {
    pub fn new(_context: CreationContext) -> Result<Self> {
        let tree = create_tree();
        let static_layout = tree.static_layout_trace();

        Ok(Self {
            ball: None,
            last_ball: None,
            last_close_enough_to_kick: false,
            last_kick_target: None,
            kick_memory: KickMemory::default(),
            striker_memory: StrikerMemory::default(),
            last_motion_switch_time: SystemTime::UNIX_EPOCH,
            last_motion_type: None,
            tree,
            static_layout,
            last_sent_game_controller_return_message_time: None,
            last_sent_hsl_message_time: None,
        })
    }

    pub fn cycle(
        &mut self,
        mut context: CycleContext<impl NetworkInterface>,
    ) -> Result<MainOutputs> {
        context
            .behavior_tree_layout
            .fill_if_subscribed(|| self.static_layout.clone());

        if let Some(ball) = context.world_state.ball {
            self.ball = Some(LastBall {
                position: ball.ball_in_field,
                velocity: ball.ball_in_ground_velocity,
                age: context.world_state.now,
                field_side: ball.field_side,
            });
            self.last_ball = self.ball.clone();
        } else if let Some(last_ball) = &self.ball
            && context
                .world_state
                .now
                .duration_since(last_ball.age)
                .unwrap_or(Duration::from_secs(0))
                >= context.parameters.last_ball_timeout
        {
            self.ball = None;
        }

        self.kick_memory.target = self.last_kick_target;
        self.kick_memory.close_enough_to_kick = self.last_close_enough_to_kick;

        let selected_striker = self.ball.as_ref().and_then(|ball| {
            select_striker(
                &context.world_state,
                ball.position,
                context.world_state.now,
                context
                    .hsl_network_parameters
                    .hsl_striker_message_receive_timeout,
                context.parameters.goal_keeper_number,
                self.striker_memory,
            )
        });
        self.striker_memory.owner = selected_striker;

        let mut blackboard = Blackboard {
            field_dimensions: *context.field_dimensions,
            free_kick_obstacle_radius: *context.free_kick_obstacle_radius,
            pass_intent_timeout: context
                .hsl_network_parameters
                .hsl_striker_message_receive_timeout,
            parameters: context.parameters.clone(),
            world_state: context.world_state.clone(),

            path_obstacles_output: Vec::new(),
            time_since_last_switch: Duration::ZERO,
            direction_difference: 0.0,
            voronoi_inputs: Vec::new(),

            ball: self.ball.clone(),
            last_ball: self.last_ball.clone(),
            last_close_enough_to_kick: self.kick_memory.close_enough_to_kick,
            last_kick_target: self.kick_memory.target,
            cycle_intent: CycleIntent::default(),
            kick_memory: self.kick_memory,
            last_motion_command: context.last_motion_command.clone(),
            last_motion_switch_time: self.last_motion_switch_time,
            last_motion_type: self.last_motion_type,

            is_injected_motion_command: false,
            walk_position: None,
            body_motion: None,
            head_motion: None,
            voronoi_map: None,
        };
        blackboard.cycle_intent =
            select_cycle_intent(&blackboard, self.kick_memory, selected_striker);
        let (status, trace) = self.tree.tick_with_trace(&mut blackboard);

        let motion_command: MotionCommand = assemble_motion_command(&blackboard, status)?;
        blackboard.kick_memory.target = persistent_kick_target(
            context.parameters.remote_control.enable
                && context.parameters.remote_control.kick_mode_toggle,
            blackboard.cycle_intent,
            blackboard.last_kick_target,
        );
        blackboard.kick_memory.close_enough_to_kick = blackboard.last_close_enough_to_kick;

        self.kick_memory = blackboard.kick_memory;
        self.last_kick_target = self.kick_memory.target;
        self.last_close_enough_to_kick = self.kick_memory.close_enough_to_kick;
        *context.last_motion_command = motion_command.clone();

        let motion_type = match motion_command.clone() {
            MotionCommand::VisualKick { .. } => Some(MotionType::Kick),
            MotionCommand::Walk { .. } => Some(MotionType::Walk),
            MotionCommand::Stand { .. } => Some(MotionType::Stand),
            MotionCommand::StandUp => Some(MotionType::StandUp),
            MotionCommand::Prepare => Some(MotionType::Prepare),
            _ => None,
        };

        self.send_game_controller_return_message(
            context.world_state,
            context.game_controller_address,
            context.hsl_network_parameters,
            context.hardware,
        )?;

        self.send_state_message(
            context.world_state,
            context.hsl_network_parameters,
            context.remaining_amount_of_messages,
            outgoing_coordination_intent(context.world_state.now, blackboard.cycle_intent),
            &mut context.last_sent_message,
            context.hardware,
        )?;

        if motion_type != self.last_motion_type {
            self.last_motion_switch_time = context.world_state.now;
            self.last_motion_type = motion_type;
        }

        context.behavior_trace.fill_if_subscribed(|| trace);
        let path_obstacles_output = blackboard.path_obstacles_output;
        context
            .path_obstacles_output
            .fill_if_subscribed(|| path_obstacles_output);
        context
            .time_since_last_switch
            .fill_if_subscribed(|| blackboard.time_since_last_switch);
        context
            .direction_difference
            .fill_if_subscribed(|| blackboard.direction_difference);
        context
            .walk_position
            .fill_if_subscribed(|| blackboard.walk_position);
        context
            .voronoi_map
            .fill_if_subscribed(|| blackboard.voronoi_map);
        context
            .voronoi_inputs
            .fill_if_subscribed(|| blackboard.voronoi_inputs);

        Ok(MainOutputs {
            motion_command: motion_command.into(),
        })
    }
}

#[cfg(test)]
mod tests {
    use linear_algebra::point;

    use types::motion_command::KickPower;

    use crate::behavior::kick_selector::{KickIntent, KickKind};

    use super::*;

    #[test]
    fn remote_kick_mode_persists_last_kick_target() {
        let autonomous_target = point!(4.5, 0.0);
        let remote_target = point!(3.0, 0.0);
        let cycle_intent = CycleIntent {
            kick: Some(KickIntent {
                kind: KickKind::Shoot,
                ball_position: point!(0.0, 0.0),
                target: autonomous_target,
                power: KickPower::Schlong,
                receiver: None,
            }),
            ..Default::default()
        };

        assert_eq!(
            persistent_kick_target(false, cycle_intent, Some(remote_target)),
            Some(autonomous_target)
        );
        assert_eq!(
            persistent_kick_target(true, cycle_intent, Some(remote_target)),
            Some(remote_target)
        );
    }
}
