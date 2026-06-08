use std::{
    f32::consts::{PI, TAU},
    mem::take,
    sync::{Arc, mpsc},
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use bevy::{
    ecs::{
        component::Component,
        event::Event,
        resource::Resource,
        system::{Query, Res, ResMut},
    },
    time::Time,
};
use color_eyre::{Result, eyre::WrapErr};

use buffered_watch::{Receiver, Sender};
use coordinate_systems::{Field, Ground};
use framework::RecordingTrigger;
use hsl_network_messages::{HulkMessage, PlayerNumber};
use hula_types::hardware::Ids;
use linear_algebra::{Isometry2, Orientation2, Pose2, point, vector};
use parameters::directory::deserialize;
use projection::intrinsic::Intrinsic;
use types::{
    ball_position::BallPosition,
    field_dimensions::{FieldDimensions, Side},
    messages::OutgoingMessage,
    motion_command::{HeadMotion, KickPower, MotionCommand, OrientationMode},
    parameters::RLWalkingParameters,
    path::{
        Path, direct_path,
        traits::{Length, PathProgress},
    },
};

use crate::{
    ball::BallResource,
    cyclers::world_state::{Cycler, CyclerInstance, Database},
    game_controller::GameController,
    interfake::{FakeDataInterface, Interfake},
    structs::Parameters,
    whistle::WhistleResource,
};

const VISUAL_KICK_APPROACH_DISTANCE: f32 = 0.3;
const VISUAL_KICK_APPROACH_TOLERANCE: f32 = 0.05;
const VISUAL_KICK_ALIGNMENT_TOLERANCE: f32 = 0.2;
const VISUAL_KICK_CLOSE_DISTANCE: f32 = 0.06;
const VISUAL_KICK_BALL_POSITION_TOLERANCE: f32 = 0.2;
const SIMULATOR_TICK_SECONDS: f32 = 0.012;
const LOWER_KICK_ROLLOUT_DISTANCE: f32 = 2.0;
const HIGHER_KICK_ROLLOUT_DISTANCE: f32 = 4.0;

#[derive(Component)]
pub struct Robot {
    pub interface: Arc<Interfake>,
    pub database: Database,
    pub parameters: Parameters,
    pub last_kick_time: Duration,
    pub head_yaw: f32,
    pub simulator_parameters: SimulatedRobotParameters,

    pub cycler: Cycler<Interfake>,
    control_receiver: Receiver<(SystemTime, Database)>,
    parameters_sender: Sender<(SystemTime, Parameters)>,
}

impl Robot {
    pub fn new(player_number: PlayerNumber) -> Self {
        Self::try_new(player_number).expect("failed to create robot")
    }

    pub fn try_new(player_number: PlayerNumber) -> Result<Self> {
        let mut parameters: Parameters = deserialize(
            "etc/parameters",
            &Ids {
                robot_id: format!("behavior_simulator.{}", from_player_number(player_number)),
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
        let (recording_sender, _recording_receiver) = mpsc::sync_channel(0);

        *parameters_sender.borrow_mut() = (SystemTime::now(), parameters.clone());

        let cycler = Cycler::new(
            CyclerInstance::WorldState,
            interface.clone(),
            control_sender,
            subscriptions_receiver,
            parameters_receiver,
            recording_sender,
            RecordingTrigger::new(0),
        )?;
        let mut database = Database::default();

        database.main_outputs.ground_to_field = Some(initial_ground_to_field(
            player_number,
            &parameters.field_dimensions,
        ));
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
            head_yaw: 0.0,
            simulator_parameters,

            cycler,
            control_receiver,
            parameters_sender,
        })
    }

    pub fn cycle(&mut self, _messages: &[Message]) -> Result<()> {
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
        let focal_lengths = nalgebra::vector![0.96666, 1.28574];
        let focal_lengths_scaled = image_size.inner.cast().component_mul(&focal_lengths);
        let field_of_view = Intrinsic::calculate_field_of_view(focal_lengths_scaled, image_size);

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
}

pub fn to_player_number(value: usize) -> Result<PlayerNumber, String> {
    let number = match value {
        1 => PlayerNumber::One,
        2 => PlayerNumber::Two,
        3 => PlayerNumber::Three,
        4 => PlayerNumber::Four,
        5 => PlayerNumber::Five,
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
    }
}

fn initial_ground_to_field(
    player_number: PlayerNumber,
    field_dimensions: &FieldDimensions,
) -> Isometry2<Ground, Field> {
    let (center_line_offset_x, side) = match player_number {
        PlayerNumber::One => (-3.0, Side::Right),
        PlayerNumber::Two => (-3.0, Side::Left),
        PlayerNumber::Three => (-1.0, Side::Right),
        PlayerNumber::Four => (-2.0, Side::Left),
        PlayerNumber::Five => (-1.0, Side::Left),
    };

    match side {
        Side::Left => Pose2::new(
            point![center_line_offset_x, field_dimensions.width * 0.5],
            -std::f32::consts::FRAC_PI_2,
        )
        .as_transform(),
        Side::Right => Pose2::new(
            point![center_line_offset_x, -field_dimensions.width * 0.5],
            std::f32::consts::FRAC_PI_2,
        )
        .as_transform(),
    }
}

pub fn move_robots(mut robots: Query<&mut Robot>, mut ball: ResMut<BallResource>, time: Res<Time>) {
    for mut robot in &mut robots {
        match robot.database.main_outputs.motion_command.clone() {
            MotionCommand::Walk {
                path,
                orientation_mode,
                target_orientation,
                distance_to_be_aligned,
                speed,
                ..
            } => {
                if let Some(step) = walk_step(
                    &path,
                    orientation_mode,
                    target_orientation,
                    distance_to_be_aligned,
                    speed,
                    &robot.parameters.rl_walking,
                    time.delta_secs(),
                ) {
                    apply_ground_frame_step(&mut robot, step);
                }
            }
            MotionCommand::WalkWithVelocity {
                velocity,
                angular_velocity,
                ..
            } => {
                if let Some(step) = velocity_step(velocity, angular_velocity, time.delta_secs()) {
                    apply_ground_frame_step(&mut robot, step);
                }
            }
            MotionCommand::VisualKick {
                ball_position,
                kick_direction,
                kick_power,
                ..
            } => {
                let ball_friction_coefficient = ball.friction_coefficient;
                let Some(ball_state) = ball.state.as_mut() else {
                    continue;
                };
                let ball_in_ground = robot.ground_to_field().inverse() * ball_state.position;
                let is_aligned = kick_direction.angle().abs() < VISUAL_KICK_ALIGNMENT_TOLERANCE;
                let ball_matches_command =
                    (ball_in_ground - ball_position).norm() < VISUAL_KICK_BALL_POSITION_TOLERANCE;
                if ball_in_ground.coords().norm() < VISUAL_KICK_CLOSE_DISTANCE
                    && is_aligned
                    && ball_matches_command
                    && time.elapsed().saturating_sub(robot.last_kick_time)
                        > Duration::from_millis(500)
                {
                    ball_state.velocity = robot.ground_to_field()
                        * (kick_direction.as_unit_vector()
                            * kick_speed(kick_power, ball_friction_coefficient));
                    robot.last_kick_time = time.elapsed();
                    continue;
                }

                let approach_position =
                    ball_position - kick_direction.as_unit_vector() * VISUAL_KICK_APPROACH_DISTANCE;
                let destination = if ball_position.coords().norm()
                    > VISUAL_KICK_APPROACH_DISTANCE + VISUAL_KICK_APPROACH_TOLERANCE
                    || !is_aligned
                {
                    approach_position
                } else {
                    ball_position
                };
                let path = direct_path(point![0.0, 0.0], destination);
                if let Some(step) = walk_step(
                    &path,
                    OrientationMode::Unspecified,
                    kick_direction,
                    0.0,
                    robot.parameters.behavior.walk_speed.kicking,
                    &robot.parameters.rl_walking,
                    time.delta_secs(),
                ) {
                    apply_ground_frame_step(&mut robot, step);
                }
            }
            _ => {}
        }
    }
}

fn apply_ground_frame_step(robot: &mut Robot, step: Isometry2<Ground, Ground>) {
    let new_ground_to_field = robot.ground_to_field() * step;
    let ground_frame_step = step.inverse();
    if let Some(ball) = robot.database.main_outputs.ball_position.as_mut() {
        ball.position = ground_frame_step * ball.position;
        ball.velocity = ground_frame_step * ball.velocity;
    }
    for ball in &mut robot.database.main_outputs.hypothetical_ball_positions {
        ball.position = ground_frame_step * ball.position;
    }
    for obstacle in &mut robot.database.main_outputs.obstacles {
        obstacle.position = ground_frame_step * obstacle.position;
    }
    *robot.ground_to_field_mut() = new_ground_to_field;
}

fn velocity_step(
    velocity: linear_algebra::Vector2<Ground>,
    angular_velocity: f32,
    delta_seconds: f32,
) -> Option<Isometry2<Ground, Ground>> {
    let translation = velocity * delta_seconds;
    let rotation = angular_velocity * delta_seconds;

    if translation.norm() <= f32::EPSILON && rotation.abs() <= f32::EPSILON {
        return None;
    }

    Some(Isometry2::from_parts(translation, rotation))
}

fn update_head_yaw(robot: &mut Robot, delta_seconds: f32, elapsed_seconds: f32) {
    let desired_head_yaw = match robot.database.main_outputs.motion_command.head_motion() {
        Some(
            HeadMotion::ZeroAngles
            | HeadMotion::Center { .. }
            | HeadMotion::LookAtReferee { .. }
            | HeadMotion::Unstiff,
        )
        | None => 0.0,
        Some(HeadMotion::LookAround | HeadMotion::SearchForLostBall) => {
            elapsed_seconds.sin() * robot.parameters.head_motion.maximum_yaw
        }
        Some(HeadMotion::LookAt { target, .. }) => direction_angle(target.coords(), robot.head_yaw),
        Some(HeadMotion::LookLeftAndRightOf { target }) => {
            direction_angle(target.coords(), robot.head_yaw)
                + elapsed_seconds.sin() * robot.parameters.behavior.look_action.angle_threshold
        }
    }
    .clamp(
        robot.parameters.head_motion.minimum_yaw,
        robot.parameters.head_motion.maximum_yaw,
    );

    let max_movement = robot.parameters.head_motion.maximum_velocity.yaw * delta_seconds;
    let movement =
        normalize_angle(desired_head_yaw - robot.head_yaw).clamp(-max_movement, max_movement);
    robot.head_yaw = normalize_angle(robot.head_yaw + movement);
}

fn ball_is_visible(
    ball_in_ground: linear_algebra::Point2<Ground>,
    head_yaw: f32,
    field_of_view: f32,
    range: f32,
) -> bool {
    let ball_vector = ball_in_ground.coords();
    if ball_vector.norm() > range {
        return false;
    }

    let angle_to_ball = direction_angle(ball_vector, head_yaw);
    normalize_angle(angle_to_ball - head_yaw).abs() < field_of_view * 0.5
}

fn direction_angle(direction: linear_algebra::Vector2<Ground>, fallback: f32) -> f32 {
    if direction.norm() <= f32::EPSILON {
        fallback
    } else {
        Orientation2::from_vector(direction).angle()
    }
}

fn normalize_angle(angle: f32) -> f32 {
    (angle + PI).rem_euclid(TAU) - PI
}

fn kick_speed(kick_power: KickPower, friction_coefficient: f32) -> f32 {
    kick_rollout_distance(kick_power) * (1.0 - friction_coefficient) / SIMULATOR_TICK_SECONDS
}

fn kick_rollout_distance(kick_power: KickPower) -> f32 {
    match kick_power {
        KickPower::Rumpelstilzchen => LOWER_KICK_ROLLOUT_DISTANCE,
        KickPower::Schlong => HIGHER_KICK_ROLLOUT_DISTANCE,
    }
}

fn walk_step(
    path: &Path,
    orientation_mode: OrientationMode,
    target_orientation: Orientation2<Ground>,
    distance_to_be_aligned: f32,
    speed: f32,
    walking_parameters: &RLWalkingParameters,
    delta_seconds: f32,
) -> Option<Isometry2<Ground, Ground>> {
    let distance = path.length();
    let forward = path.forward(point![0.0, 0.0]);
    let translation = if distance > f32::EPSILON && forward.norm() > f32::EPSILON {
        let step_distance = distance.min(speed.max(0.0) * delta_seconds);
        forward.normalize() * step_distance
    } else {
        vector![0.0, 0.0]
    };
    let orientation = walk_step_orientation(
        path,
        orientation_mode,
        target_orientation,
        distance_to_be_aligned,
        walking_parameters.hybrid_align_distance,
    );
    let rotation =
        orientation.as_unit_vector().y() * walking_parameters.max_alignment_rate * delta_seconds;

    if translation.norm() <= f32::EPSILON && rotation.abs() <= f32::EPSILON {
        return None;
    }

    Some(Isometry2::from_parts(translation, rotation))
}

fn walk_step_orientation(
    path: &Path,
    orientation_mode: OrientationMode,
    target_orientation: Orientation2<Ground>,
    distance_to_be_aligned: f32,
    hybrid_align_distance: f32,
) -> Orientation2<Ground> {
    let origin = point![0.0, 0.0];
    let path_forward = path.forward(origin);
    let walk_orientation = match orientation_mode {
        OrientationMode::Unspecified => target_orientation,
        OrientationMode::AlignWithPath => direction_orientation(path_forward, target_orientation),
        OrientationMode::LookTowards { direction, .. } => direction,
        OrientationMode::LookAt { target, .. } => {
            direction_orientation(target - origin, target_orientation)
        }
    };

    let importance = target_alignment_importance(
        distance_to_be_aligned.max(0.0),
        hybrid_align_distance.max(0.0),
        path.length(),
    );
    walk_orientation.slerp(target_orientation, importance)
}

fn target_alignment_importance(
    distance_to_be_aligned: f32,
    hybrid_align_distance: f32,
    distance_to_target: f32,
) -> f32 {
    if distance_to_target < distance_to_be_aligned {
        1.0
    } else if hybrid_align_distance > f32::EPSILON
        && distance_to_target < distance_to_be_aligned + hybrid_align_distance
    {
        (1.0 + f32::cos(PI * (distance_to_target - distance_to_be_aligned) / hybrid_align_distance))
            * 0.5
    } else {
        0.0
    }
}

fn direction_orientation(
    direction: linear_algebra::Vector2<Ground>,
    fallback: Orientation2<Ground>,
) -> Orientation2<Ground> {
    if direction.norm() <= f32::EPSILON {
        fallback
    } else {
        Orientation2::from_vector(direction)
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
    mut game_controller: ResMut<GameController>,
    time: Res<Time>,
    mut messages: ResMut<Messages>,
) {
    let messages_sent_last_cycle = take(&mut messages.messages);
    let now = SystemTime::UNIX_EPOCH + time.elapsed();

    for mut robot in &mut robots {
        robot.database.main_outputs.cycle_time.start_time = now;
        robot.database.main_outputs.cycle_time.last_cycle_duration = time.delta();

        update_head_yaw(&mut robot, time.delta_secs(), time.elapsed_secs());

        if let Some(ball_state) = ball.state {
            let ball_in_ground = robot.ground_to_field().inverse() * ball_state.position;
            if ball_is_visible(
                ball_in_ground,
                robot.head_yaw,
                robot.field_of_view(),
                robot.simulator_parameters.ball_view_range,
            ) {
                robot.database.main_outputs.ball_position = Some(BallPosition {
                    position: ball_in_ground,
                    velocity: robot.ground_to_field().inverse() * ball_state.velocity,
                    last_seen: now,
                });
            }
        }
        let ball_timeout = robot
            .parameters
            .ball_filter
            .hypothesis_timeout
            .mul_f32(robot.simulator_parameters.ball_timeout_factor);
        if !robot
            .database
            .main_outputs
            .ball_position
            .is_some_and(|ball_position| {
                now.duration_since(ball_position.last_seen)
                    .is_ok_and(|age| age < ball_timeout)
            })
        {
            robot.database.main_outputs.ball_position = None;
        }

        let _ = &whistle;
        robot.database.main_outputs.game_controller_state = Some(game_controller.state.clone());
        robot.interface.set_time(now);
        robot.cycle(&messages_sent_last_cycle).unwrap();

        for message in robot.interface.take_outgoing_messages() {
            if let OutgoingMessage::Hsl(message) = message {
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

#[cfg(test)]
mod tests {
    use std::{f32::consts::FRAC_PI_2, time::Duration};

    use bevy::{app::App, prelude::Update, time::Time};
    use linear_algebra::Orientation2;
    use types::{
        ball_position::SimulatorBallState,
        motion_command::ImageRegion,
        motion_command::{HeadMotion, MotionCommand, OrientationMode},
        path::direct_path,
    };

    use super::*;

    fn walking_parameters() -> RLWalkingParameters {
        RLWalkingParameters {
            hybrid_align_distance: 0.2,
            max_alignment_rate: 2.0,
            ..Default::default()
        }
    }

    #[test]
    fn walk_step_moves_toward_path_end_by_speed_times_delta() {
        let path = direct_path(point![0.0, 0.0], point![1.0, 0.0]);

        let step = walk_step(
            &path,
            OrientationMode::AlignWithPath,
            Orientation2::identity(),
            0.0,
            0.5,
            &walking_parameters(),
            0.1,
        )
        .expect("expected walking step");

        assert!((step.translation().x() - 0.05).abs() < 0.0001);
        assert!(step.translation().y().abs() < 0.0001);
    }

    #[test]
    fn walk_step_integrates_rotation_velocity_instead_of_snapping_to_heading() {
        let path = direct_path(point![0.0, 0.0], point![0.0, 1.0]);

        let step = walk_step(
            &path,
            OrientationMode::AlignWithPath,
            Orientation2::identity(),
            0.0,
            0.5,
            &walking_parameters(),
            0.1,
        )
        .expect("expected walking step");

        assert!(step.orientation().angle() > 0.0);
        assert!(step.orientation().angle() < FRAC_PI_2 * 0.5);
    }

    #[test]
    fn remembered_ball_moves_into_new_ground_frame_when_robot_walks() {
        let mut app = App::new();
        let mut time = Time::<()>::default();
        time.advance_by(Duration::from_millis(100));
        app.insert_resource(time)
            .insert_resource(BallResource::default())
            .add_systems(Update, move_robots);

        let mut robot = Robot::new(PlayerNumber::One);
        robot.database.main_outputs.ground_to_field = Some(Isometry2::identity());
        robot.database.main_outputs.ball_position = Some(BallPosition {
            position: point![1.0, 0.0],
            velocity: vector![0.0, 0.0],
            last_seen: SystemTime::UNIX_EPOCH,
        });
        robot.database.main_outputs.motion_command = MotionCommand::Walk {
            head: HeadMotion::ZeroAngles,
            path: direct_path(point![0.0, 0.0], point![1.0, 0.0]),
            orientation_mode: OrientationMode::AlignWithPath,
            target_orientation: Orientation2::identity(),
            distance_to_be_aligned: 0.0,
            speed: 1.0,
        };
        app.world_mut().spawn(robot);

        app.update();

        let mut robots = app.world_mut().query::<&Robot>();
        let robot = robots.single(app.world()).expect("expected one robot");
        let ball_position = robot
            .database
            .main_outputs
            .ball_position
            .expect("expected remembered ball");
        assert!((ball_position.position.x() - 0.9).abs() < 0.0001);
        assert!(ball_position.position.y().abs() < 0.0001);
    }

    #[test]
    fn walk_with_velocity_moves_and_rotates_by_velocity_components() {
        let mut app = App::new();
        let mut time = Time::<()>::default();
        time.advance_by(Duration::from_millis(100));
        app.insert_resource(time)
            .insert_resource(BallResource::default())
            .add_systems(Update, move_robots);

        let mut robot = Robot::new(PlayerNumber::One);
        robot.database.main_outputs.ground_to_field = Some(Isometry2::identity());
        robot.database.main_outputs.motion_command = MotionCommand::WalkWithVelocity {
            head: HeadMotion::ZeroAngles,
            velocity: vector![1.0, 0.5],
            angular_velocity: 2.0,
        };
        app.world_mut().spawn(robot);

        app.update();

        let mut robots = app.world_mut().query::<&Robot>();
        let robot = robots.single(app.world()).expect("expected one robot");
        let ground_to_field = robot.ground_to_field();
        assert!((ground_to_field.translation().x() - 0.1).abs() < 0.0001);
        assert!((ground_to_field.translation().y() - 0.05).abs() < 0.0001);
        assert!((ground_to_field.orientation().angle() - 0.2).abs() < 0.0001);
    }

    #[test]
    fn visual_kick_does_not_kick_from_approach_distance() {
        let mut app = App::new();
        let mut time = Time::<()>::default();
        time.advance_by(Duration::from_secs(1));
        app.insert_resource(time)
            .insert_resource(BallResource {
                state: Some(SimulatorBallState {
                    position: point![0.3, 0.0],
                    velocity: vector![0.0, 0.0],
                }),
                ..Default::default()
            })
            .add_systems(Update, move_robots);

        let mut robot = Robot::new(PlayerNumber::One);
        robot.database.main_outputs.ground_to_field = Some(Isometry2::identity());
        robot.database.main_outputs.motion_command = MotionCommand::VisualKick {
            head: HeadMotion::ZeroAngles,
            ball_position: point![0.3, 0.0],
            kick_direction: Orientation2::identity(),
            target_position: point![1.0, 0.0],
            robot_theta_to_field: Orientation2::identity(),
            kick_power: KickPower::Rumpelstilzchen,
        };
        app.world_mut().spawn(robot);

        app.update();

        let ball = app.world().resource::<BallResource>().state.unwrap();
        assert_eq!(ball.velocity, vector![0.0, 0.0]);
        let mut robots = app.world_mut().query::<&Robot>();
        let robot = robots.single(app.world()).expect("expected one robot");
        assert!(robot.ground_to_field().translation().x() > 0.0);
    }

    #[test]
    fn visual_kick_keeps_closing_in_after_reaching_approach_distance() {
        let mut app = App::new();
        let mut time = Time::<()>::default();
        time.advance_by(Duration::from_secs(1));
        app.insert_resource(time)
            .insert_resource(BallResource {
                state: Some(SimulatorBallState {
                    position: point![0.24, 0.0],
                    velocity: vector![0.0, 0.0],
                }),
                ..Default::default()
            })
            .add_systems(Update, move_robots);

        let mut robot = Robot::new(PlayerNumber::One);
        robot.database.main_outputs.ground_to_field = Some(Isometry2::identity());
        robot.database.main_outputs.motion_command = MotionCommand::VisualKick {
            head: HeadMotion::ZeroAngles,
            ball_position: point![0.24, 0.0],
            kick_direction: Orientation2::identity(),
            target_position: point![1.0, 0.0],
            robot_theta_to_field: Orientation2::identity(),
            kick_power: KickPower::Rumpelstilzchen,
        };
        app.world_mut().spawn(robot);

        app.update();

        let ball = app.world().resource::<BallResource>().state.unwrap();
        assert_eq!(ball.velocity, vector![0.0, 0.0]);
        let mut robots = app.world_mut().query::<&Robot>();
        let robot = robots.single(app.world()).expect("expected one robot");
        assert!(robot.ground_to_field().translation().x() > 0.0);
    }

    #[test]
    fn visual_kick_kicks_when_close_to_ball_and_aligned() {
        let mut app = App::new();
        let mut time = Time::<()>::default();
        time.advance_by(Duration::from_secs(1));
        app.insert_resource(time)
            .insert_resource(BallResource {
                state: Some(SimulatorBallState {
                    position: point![0.02, 0.0],
                    velocity: vector![0.0, 0.0],
                }),
                ..Default::default()
            })
            .add_systems(Update, move_robots);

        let mut robot = Robot::new(PlayerNumber::One);
        robot.database.main_outputs.ground_to_field = Some(Isometry2::identity());
        robot.database.main_outputs.motion_command = MotionCommand::VisualKick {
            head: HeadMotion::ZeroAngles,
            ball_position: point![0.02, 0.0],
            kick_direction: Orientation2::identity(),
            target_position: point![1.0, 0.0],
            robot_theta_to_field: Orientation2::identity(),
            kick_power: KickPower::Rumpelstilzchen,
        };
        app.world_mut().spawn(robot);

        app.update();

        let ball = app.world().resource::<BallResource>().state.unwrap();
        assert!(ball.velocity.x() > 0.0);
        assert!(ball.velocity.y().abs() < 0.0001);
    }

    #[test]
    fn ball_behind_centered_head_is_not_visible() {
        let mut app = App::new();
        let mut time = Time::<()>::default();
        time.advance_by(Duration::from_millis(12));
        app.insert_resource(time)
            .insert_resource(BallResource {
                state: Some(SimulatorBallState {
                    position: point![-1.0, 0.0],
                    velocity: vector![0.0, 0.0],
                }),
                ..Default::default()
            })
            .insert_resource(WhistleResource::default())
            .insert_resource(GameController::default())
            .insert_resource(Messages::default())
            .add_systems(Update, cycle_robots);

        let mut robot = Robot::new(PlayerNumber::One);
        robot.database.main_outputs.ground_to_field = Some(Isometry2::identity());
        robot.database.main_outputs.motion_command = MotionCommand::Stand {
            head: HeadMotion::Center {
                image_region_target: ImageRegion::Center,
            },
        };
        app.world_mut().spawn(robot);

        app.update();

        let mut robots = app.world_mut().query::<&Robot>();
        let robot = robots.single(app.world()).expect("expected one robot");
        assert!(robot.database.main_outputs.ball_position.is_none());
    }

    #[test]
    fn remembered_ball_timeout_uses_ball_filter_hypothesis_timeout_factor() {
        let mut app = App::new();
        let mut time = Time::<()>::default();
        time.advance_by(Duration::from_secs(1));
        app.insert_resource(time)
            .insert_resource(BallResource::default())
            .insert_resource(WhistleResource::default())
            .insert_resource(GameController::default())
            .insert_resource(Messages::default())
            .add_systems(Update, cycle_robots);

        let mut robot = Robot::new(PlayerNumber::One);
        robot.database.main_outputs.ground_to_field = Some(Isometry2::identity());
        robot.database.main_outputs.ball_position = Some(BallPosition {
            position: point![1.0, 0.0],
            velocity: vector![0.0, 0.0],
            last_seen: SystemTime::UNIX_EPOCH,
        });
        app.world_mut().spawn(robot);

        app.update();

        let mut robots = app.world_mut().query::<&Robot>();
        let robot = robots.single(app.world()).expect("expected one robot");
        assert!(robot.database.main_outputs.ball_position.is_some());
    }

    #[test]
    fn lower_kick_rolls_about_two_meters_with_default_friction() {
        assert!((default_rollout_distance(KickPower::Rumpelstilzchen) - 2.0).abs() < 0.01);
    }

    #[test]
    fn higher_kick_rolls_about_four_meters_with_default_friction() {
        assert!((default_rollout_distance(KickPower::Schlong) - 4.0).abs() < 0.01);
    }

    fn default_rollout_distance(kick_power: KickPower) -> f32 {
        kick_speed(kick_power, 0.98) * 0.012 / (1.0 - 0.98)
    }
}
