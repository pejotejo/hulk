use std::{
    convert::Into,
    mem::take,
    sync::{mpsc, Arc},
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use bevy::{
    ecs::{
        component::Component,
        event::Event,
        system::{Query, Res, ResMut, Resource},
    },
    time::Time,
};
use color_eyre::{eyre::WrapErr, Result};

use buffered_watch::{Receiver, Sender};
use control::localization::generate_initial_pose;
use coordinate_systems::{Field, Ground, Head};
use framework::{future_queue, Producer, RecordingTrigger};
use geometry::{direction::Rotate90Degrees, line_segment::LineSegment};
use hula_types::hardware::Ids;
use linear_algebra::{vector, Isometry2, Orientation2, Point2, Rotation2, Vector2};
use parameters::directory::deserialize;
use projection::camera_matrix::CameraMatrix;
use spl_network_messages::{HulkMessage, PlayerNumber};
use types::{
    ball_position::BallPosition,
    filtered_whistle::FilteredWhistle,
    messages::{IncomingMessage, OutgoingMessage},
    motion_command::{HeadMotion, KickVariant, MotionCommand, OrientationMode},
    motion_selection::MotionSafeExits,
    planned_path::PathSegment,
    pose_kinds::PoseKind,
    support_foot::Side,
};

use crate::{
    ball::BallResource,
    cyclers::control::{Cycler, CyclerInstance, Database},
    game_controller::GameController,
    interfake::{FakeDataInterface, Interfake},
    structs::Parameters,
    visual_referee::VisualRefereeResource,
    whistle::WhistleResource,
};

#[derive(Component)]
pub struct Robot {
    pub interface: Arc<Interfake>,
    pub database: Database,
    pub parameters: Parameters,
    pub last_kick_time: Duration,
    pub ball_last_seen: Option<SystemTime>,
    pub simulator_parameters: SimulatedRobotParameters,

    pub cycler: Cycler<Interfake>,
    control_receiver: Receiver<(SystemTime, Database)>,
    parameters_sender: Sender<(SystemTime, Parameters)>,
    spl_network_sender: Producer<crate::structs::spl_network::MainOutputs>,
    object_detection_top_sender: Producer<crate::structs::object_detection::MainOutputs>,
}

impl Robot {
    pub fn new(player_number: PlayerNumber) -> Self {
        Self::try_new(player_number).expect("failed to create robot")
    }

    pub fn try_new(player_number: PlayerNumber) -> Result<Self> {
        let mut parameters: Parameters = deserialize(
            "etc/parameters",
            &Ids {
                body_id: format!("behavior_simulator.{}", from_player_number(player_number)),
                head_id: format!("behavior_simulator.{}", from_player_number(player_number)),
            },
            true,
        )
        .wrap_err("could not load initial parameters")?;
        parameters.player_number = player_number;

        let interface: Arc<_> = Interfake::default().into();

        let (control_sender, control_receiver) =
            buffered_watch::channel((UNIX_EPOCH, Database::default()));
        let (mut subscriptions_sender, subscriptions_receiver) =
            buffered_watch::channel(Default::default());
        let (mut parameters_sender, parameters_receiver) =
            buffered_watch::channel((UNIX_EPOCH, Default::default()));
        let (spl_network_sender, spl_network_consumer) = future_queue();
        let (recording_sender, _recording_receiver) = mpsc::sync_channel(0);
        let (object_detection_top_sender, object_detection_top_consumer) = future_queue();

        *parameters_sender.borrow_mut() = (SystemTime::now(), parameters.clone());

        let mut cycler = Cycler::new(
            CyclerInstance::Control,
            interface.clone(),
            control_sender,
            subscriptions_receiver,
            parameters_receiver,
            spl_network_consumer,
            object_detection_top_consumer,
            recording_sender,
            RecordingTrigger::new(0),
        )?;
        cycler.cycler_state.motion_safe_exits = MotionSafeExits::fill(true);

        let mut database = Database::default();

        database.main_outputs.ground_to_field = Some(
            generate_initial_pose(
                &parameters.localization.initial_poses[player_number],
                &parameters.field_dimensions,
            )
            .as_transform(),
        );
        database.main_outputs.has_ground_contact = true;
        database.main_outputs.buttons.is_chest_button_pressed_once = true;
        database.main_outputs.is_localization_converged = true;

        subscriptions_sender
            .borrow_mut()
            .insert("additional_outputs".to_string());

        let simulator_parameters = SimulatedRobotParameters {
            ball_view_range: 3.0,
            ball_timeout_factor: 0.1,
        };

        Ok(Self {
            interface,
            database,
            parameters,
            last_kick_time: Duration::default(),
            ball_last_seen: None,
            simulator_parameters,
            cycler,
            control_receiver,
            parameters_sender,
            spl_network_sender,
            object_detection_top_sender,
        })
    }

    pub fn cycle(
        &mut self,
        messages: &[Message],
        referee_pose_kind: &Option<PoseKind>,
    ) -> Result<()> {
        for Message { sender, payload } in messages {
            let source_is_other = *sender != self.parameters.player_number;
            let message = IncomingMessage::Spl(*payload);
            self.spl_network_sender.announce();
            self.spl_network_sender
                .finalize(crate::structs::spl_network::MainOutputs {
                    filtered_message: source_is_other.then(|| message.clone()),
                    message,
                });
        }

        self.object_detection_top_sender.announce();
        self.object_detection_top_sender
            .finalize(crate::structs::object_detection::MainOutputs {
                referee_pose_kind: referee_pose_kind.clone(),
                ..Default::default()
            });

        buffered_watch::Sender::<_>::borrow_mut(
            &mut self.interface.get_last_database_sender().lock(),
        )
        .main_outputs = self.database.main_outputs.clone();
        *self.parameters_sender.borrow_mut() = (SystemTime::now(), self.parameters.clone());

        self.cycler.cycle()?;

        let (_, database) = &*self.control_receiver.borrow_and_mark_as_seen();
        self.database.main_outputs = database.main_outputs.clone();
        self.database.additional_outputs = database.additional_outputs.clone();
        Ok(())
    }

    pub fn field_of_view(&self) -> f32 {
        let image_size = vector![640.0, 480.0];
        let focal_lengths = self
            .parameters
            .camera_matrix_parameters
            .vision_top
            .focal_lengths;
        let focal_lengths_scaled = image_size.inner.cast().component_mul(&focal_lengths);
        let field_of_view = CameraMatrix::calculate_field_of_view(focal_lengths_scaled, image_size);

        field_of_view.x
    }

    pub fn ground_to_field(&self) -> Isometry2<Ground, Field> {
        self.database
            .main_outputs
            .ground_to_field
            .expect("simulated robots should always have a ground to field")
    }

    pub fn ground_to_field_mut(&mut self) -> &mut Isometry2<Ground, Field> {
        self.database
            .main_outputs
            .ground_to_field
            .as_mut()
            .expect("simulated robots should always have a ground to field")
    }

    pub fn whistle_mut(&mut self) -> &mut FilteredWhistle {
        &mut self.database.main_outputs.filtered_whistle
    }
}

pub fn to_player_number(value: usize) -> Result<PlayerNumber, String> {
    let number = match value {
        1 => PlayerNumber::One,
        2 => PlayerNumber::Two,
        3 => PlayerNumber::Three,
        4 => PlayerNumber::Four,
        5 => PlayerNumber::Five,
        6 => PlayerNumber::Six,
        7 => PlayerNumber::Seven,
        number => return Err(format!("invalid player number: {number}")),
    };

    Ok(number)
}

pub fn from_player_number(val: PlayerNumber) -> usize {
    match val {
        PlayerNumber::One => 1,
        PlayerNumber::Two => 2,
        PlayerNumber::Three => 3,
        PlayerNumber::Four => 4,
        PlayerNumber::Five => 5,
        PlayerNumber::Six => 6,
        PlayerNumber::Seven => 7,
    }
}

pub fn move_robots(mut robots: Query<&mut Robot>, mut ball: ResMut<BallResource>, time: Res<Time>) {
    for mut robot in &mut robots {
        if let Some(ball) = robot.database.main_outputs.ball_position.as_mut() {
            ball.position += ball.velocity * time.delta_secs();
            ball.velocity *= 0.98
        }

        let parameters = &robot.parameters;
        let mut ground_to_field_update: Option<Isometry2<Ground, Field>> = None;

        let head_motion = match robot.database.main_outputs.motion_command.clone() {
            MotionCommand::Walk {
                head,
                path,
                orientation_mode,
                ..
            } => {
                let steps_per_second = 1.0 / 0.35;
                let steps_this_cycle = steps_per_second * time.delta_secs();
                let max_step = parameters.step_planner.max_step_size;

                let target = match path[0] {
                    PathSegment::LineSegment(LineSegment(_start, end)) => end.coords(),
                    PathSegment::Arc(arc) => (arc.start.as_unit_vector() * arc.circle.radius)
                        .rotate_90_degrees(arc.direction),
                };

                let orientation = match orientation_mode {
                    OrientationMode::AlignWithPath => {
                        if target.norm_squared() < f32::EPSILON {
                            Orientation2::identity()
                        } else {
                            Orientation2::from_vector(target)
                        }
                    }
                    OrientationMode::Override(orientation) => orientation,
                };
                let step = target.cap_magnitude(max_step.forward * steps_this_cycle);

                let rotation = orientation.angle().clamp(
                    -max_step.turn * steps_this_cycle,
                    max_step.turn * steps_this_cycle,
                );
                let movement = Isometry2::from_parts(step.as_point().coords(), rotation);
                let old_ground_to_field = robot.ground_to_field();
                let new_ground_to_field = old_ground_to_field * movement;
                ground_to_field_update = Some(new_ground_to_field);

                for obstacle in &mut robot.database.main_outputs.obstacles {
                    let obstacle_in_field = old_ground_to_field * obstacle.position;
                    obstacle.position = new_ground_to_field.inverse() * obstacle_in_field;
                }
                if let Some(ball) = robot.database.main_outputs.ball_position.as_mut() {
                    ball.velocity = movement.inverse() * ball.velocity;
                    ball.position = movement.inverse() * ball.position;
                }

                head
            }
            MotionCommand::InWalkKick {
                head,
                kick,
                kicking_side,
                strength,
                ..
            } => {
                if let Some(ball) = ball.state.as_mut() {
                    let side = match kicking_side {
                        Side::Left => -1.0,
                        Side::Right => 1.0,
                    };

                    let in_range =
                        (robot.ground_to_field().as_pose().position() - ball.position).norm() < 0.3;
                    let previous_kick_finished =
                        (time.elapsed() - robot.last_kick_time).as_secs_f32() > 1.0;
                    if in_range && previous_kick_finished {
                        let direction = match kick {
                            KickVariant::Forward => vector![1.0, 0.0],
                            KickVariant::Turn => vector![0.707, 0.707 * side],
                            KickVariant::Side => vector![0.0, 1.0 * -side],
                        };
                        ball.velocity += robot.ground_to_field() * direction * strength * 2.5;
                        robot.last_kick_time = time.elapsed();
                    };
                }
                head
            }
            MotionCommand::SitDown { head } => head,
            MotionCommand::Stand { head, .. } => head,
            _ => HeadMotion::Center,
        };

        let desired_head_yaw = match head_motion {
            HeadMotion::ZeroAngles => 0.0,
            HeadMotion::Center => 0.0,
            HeadMotion::LookAround | HeadMotion::SearchForLostBall => {
                robot.database.main_outputs.look_around.yaw
            }
            HeadMotion::LookAt { target, .. } => Orientation2::from_vector(target.coords()).angle(),
            HeadMotion::LookAtReferee { .. } => {
                if let Some(ground_to_field) = robot.database.main_outputs.ground_to_field {
                    let expected_referee_position = ground_to_field.inverse()
                        * robot
                            .database
                            .main_outputs
                            .expected_referee_position
                            .unwrap_or_default();
                    Orientation2::from_vector(expected_referee_position.coords()).angle()
                } else {
                    0.0
                }
            }
            HeadMotion::LookLeftAndRightOf { target } => {
                let glance_factor = 0.0; //self.time_elapsed.as_secs_f32().sin();
                target.coords().angle(&Vector2::x_axis())
                    + glance_factor * robot.parameters.look_at.glance_angle
            }
            HeadMotion::Unstiff => 0.0,
            HeadMotion::Animation { .. } => 0.0,
        };

        let max_head_rotation_per_cycle =
            robot.parameters.head_motion.maximum_velocity.yaw * time.delta_secs();
        let diff = desired_head_yaw - robot.database.main_outputs.sensor_data.positions.head.yaw;
        let movement = diff.clamp(-max_head_rotation_per_cycle, max_head_rotation_per_cycle);

        robot.database.main_outputs.sensor_data.positions.head.yaw += movement;
        if let Some(new_ground_to_field) = ground_to_field_update {
            *robot.ground_to_field_mut() = new_ground_to_field;
        }
    }
}

#[derive(Event, Clone, Copy)]
pub struct Message {
    pub sender: PlayerNumber,
    pub payload: HulkMessage,
}

#[derive(Resource, Default)]
pub struct Messages {
    pub messages: Vec<Message>,
}

#[allow(clippy::too_many_arguments)]
pub fn cycle_robots(
    mut robots: Query<&mut Robot>,
    ball: Res<BallResource>,
    whistle: Res<WhistleResource>,
    visual_referee: Res<VisualRefereeResource>,
    mut game_controller: ResMut<GameController>,
    time: Res<Time>,
    mut messages: ResMut<Messages>,
) {
    let messages_sent_last_cycle = take(&mut messages.messages);
    let now = SystemTime::UNIX_EPOCH + time.elapsed();

    for mut robot in &mut robots {
        robot.database.main_outputs.cycle_time.start_time = now;

        let ball_visible = ball.state.as_ref().is_some_and(|ball| {
            let ball_in_ground = robot.ground_to_field().inverse() * ball.position;
            let head_to_ground =
                Rotation2::new(robot.database.main_outputs.sensor_data.positions.head.yaw);
            let ball_in_head: Point2<Head> = head_to_ground.inverse() * ball_in_ground;
            let field_of_view = robot.field_of_view();
            let angle_to_ball = ball_in_head.coords().angle(&Vector2::x_axis());

            angle_to_ball.abs() < field_of_view / 2.0
                && ball_in_head.coords().norm() < robot.simulator_parameters.ball_view_range
        });
        if ball_visible {
            robot.ball_last_seen = Some(now);
            robot.database.main_outputs.ball_position =
                ball.state.as_ref().map(|ball| BallPosition {
                    position: robot.ground_to_field().inverse() * ball.position,
                    velocity: robot.ground_to_field().inverse() * ball.velocity,
                    last_seen: now,
                });
        }
        if !robot.ball_last_seen.is_some_and(|last_seen| {
            now.duration_since(last_seen).expect("time ran backwards")
                < robot
                    .parameters
                    .ball_filter
                    .hypothesis_timeout
                    .mul_f32(robot.simulator_parameters.ball_timeout_factor)
        }) {
            robot.database.main_outputs.ball_position = None
        };
        *robot.whistle_mut() = FilteredWhistle {
            is_detected: Some(time.elapsed()) == whistle.last_whistle,
            last_detection: whistle
                .last_whistle
                .map(|last_whistle| SystemTime::UNIX_EPOCH + last_whistle),
        };
        let visual_referee_pose_kind = if matches!(
            robot.database.main_outputs.motion_command.head_motion(),
            Some(HeadMotion::LookAtReferee { .. })
        ) {
            visual_referee.pose_kind.clone()
        } else {
            None
        };
        robot.database.main_outputs.game_controller_state = Some(game_controller.state.clone());
        robot.cycler.cycler_state.ground_to_field = robot.ground_to_field();
        robot.interface.set_time(now);
        robot
            .cycle(&messages_sent_last_cycle, &visual_referee_pose_kind)
            .unwrap();

        for message in robot.interface.take_outgoing_messages() {
            if let OutgoingMessage::Spl(message) = message {
                messages.messages.push(Message {
                    sender: robot.parameters.player_number,
                    payload: message,
                });
                game_controller
                    .state
                    .hulks_team
                    .remaining_amount_of_messages -= 1
            }
        }
    }
}

pub struct SimulatedRobotParameters {
    pub ball_view_range: f32,
    pub ball_timeout_factor: f32,
}
