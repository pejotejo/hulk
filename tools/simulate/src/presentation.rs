//! View-only camera direction and pose replay; the physics worker remains live.
mod replay;

use bevy::prelude::*;
use eframe::egui::{self, Color32, Ui};

use crate::{
    bevy_mujoco::{MujocoBody, MujocoPoseUpdateSet, MujocoRenderFrame},
    parameters::CurrentSimulatorParameters,
    scene::{
        ball::{Ball, SpawnedBalls},
        object::ObjectPart,
        robot::RobotHead,
    },
    team::Team,
};
use replay::{History, Replay, interpolate};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum Shot {
    #[default]
    Drone,
    Sideline,
    Player,
}

impl Shot {
    fn label(self) -> &'static str {
        match self {
            Self::Drone => "Drone",
            Self::Sideline => "Sideline",
            Self::Player => "Player",
        }
    }
}

#[derive(Clone, Copy, Default, PartialEq, Eq)]
enum Subject {
    #[default]
    Ball,
    Player,
}

#[derive(Default, Resource)]
pub(crate) struct Presentation {
    history: History,
    replay: Option<Replay>,
    cinematic: bool,
    shot: Shot,
    subject: Subject,
    manual_pose: Option<Transform>,
}

impl Presentation {
    pub(crate) fn is_replaying(&self) -> bool {
        self.replay.is_some()
    }

    pub(super) fn cinematic(&self) -> bool {
        self.cinematic
    }

    pub(super) fn badge(&self) -> String {
        if let Some(replay) = &self.replay {
            format!("REPLAY  {:.1} / {:.1} s", replay.elapsed, replay.duration())
        } else {
            "LIVE".to_owned()
        }
    }

    fn set_cinematic(&mut self, enabled: bool, camera: &mut Transform) {
        if enabled == self.cinematic {
            return;
        }
        if enabled {
            self.manual_pose = Some(*camera);
        } else if let Some(pose) = self.manual_pose.take() {
            *camera = pose;
        }
        self.cinematic = enabled;
    }
}

pub(super) struct PresentationPlugin;

impl Plugin for PresentationPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Presentation>().add_systems(
            Update,
            (present_poses, direct_camera)
                .chain()
                .after(MujocoPoseUpdateSet),
        );
    }
}

pub(super) fn set_cinematic(world: &mut World, enabled: bool) {
    world.resource_scope(|world, mut presentation: Mut<Presentation>| {
        let mut cameras = world.query_filtered::<&mut Transform, With<Camera3d>>();
        if let Ok(mut camera) = cameras.single_mut(world) {
            presentation.set_cinematic(enabled, &mut camera);
        }
    });
}

pub(super) fn toolbar(ui: &mut Ui, world: &mut World) {
    let cinematic = world.resource::<Presentation>().cinematic;
    if ui
        .selectable_label(cinematic, "Cinematic")
        .on_hover_text("Smooth tracking shots. Switching off restores your manual camera.")
        .clicked()
    {
        set_cinematic(world, !cinematic);
    }
    let mut presentation = world.resource_mut::<Presentation>();
    ui.add_enabled_ui(presentation.cinematic, |ui| {
        egui::ComboBox::from_id_salt("simulator_camera_shot")
            .width(80.0)
            .selected_text(presentation.shot.label())
            .show_ui(ui, |ui| {
                for shot in [Shot::Drone, Shot::Sideline, Shot::Player] {
                    ui.selectable_value(&mut presentation.shot, shot, shot.label());
                }
            });
        ui.selectable_value(&mut presentation.subject, Subject::Ball, "Ball");
        ui.selectable_value(&mut presentation.subject, Subject::Player, "Player")
            .on_hover_text("Follow the player selected in Twix.");
    });
    ui.separator();
    if presentation.is_replaying() {
        if ui.button("Return live").clicked() {
            presentation.replay = None;
        }
    } else {
        let duration = presentation.history.duration();
        if ui
            .add_enabled(duration >= 0.1, egui::Button::new("Replay"))
            .on_hover_text(format!(
                "Replay the last {duration:.1} simulation seconds (up to 10 s).\n\
                 Physics, robotics and match controls remain live."
            ))
            .clicked()
        {
            presentation.replay = presentation.history.clip();
        }
    }
    let replaying = presentation.is_replaying();
    ui.colored_label(
        if replaying {
            Color32::from_rgb(255, 192, 92)
        } else {
            Color32::from_rgb(100, 210, 165)
        },
        presentation.badge(),
    )
    .on_hover_text(if replaying {
        "Pose replay only. The live simulation and match desk continue running."
    } else {
        "Showing the live simulation."
    });
}

fn present_poses(
    frame: Res<MujocoRenderFrame>,
    time: Res<Time<Real>>,
    mut presentation: ResMut<Presentation>,
    mut bodies: Query<
        (Entity, &mut Transform),
        (With<MujocoBody>, Or<(With<Ball>, With<ObjectPart>)>),
    >,
) {
    // Always capture the live transforms before applying the historical ones.
    if presentation.history.record(
        frame.time,
        frame.generation,
        bodies.iter().map(|(entity, pose)| (entity, *pose)),
    ) {
        presentation.replay = None;
    }
    if let Some(replay) = &mut presentation.replay
        && !replay.advance(time.delta_secs_f64())
    {
        presentation.replay = None;
    }
    if let Some(replay) = &presentation.replay {
        replay.for_each_pose(|entity, pose| {
            if let Ok((_, mut transform)) = bodies.get_mut(entity) {
                *transform = pose;
            }
        });
    }
}

fn direct_camera(
    presentation: Res<Presentation>,
    time: Res<Time<Real>>,
    team: Res<Team>,
    parameters: Res<CurrentSimulatorParameters>,
    balls: Res<SpawnedBalls>,
    bodies: Query<
        (&Transform, Option<&ObjectPart>, Has<RobotHead>),
        (With<MujocoBody>, Without<Camera3d>),
    >,
    mut cameras: Query<(&mut Transform, &Projection), (With<Camera3d>, Without<MujocoBody>)>,
) {
    if !presentation.cinematic {
        return;
    }
    let ball = balls
        .0
        .iter()
        .find_map(|entity| bodies.get(*entity).ok())
        .map(|(pose, ..)| pose.translation);
    let selected = team.selected().and_then(|member| member.entity);
    let player = bodies
        .iter()
        .filter(|(_, part, head)| *head && part.is_some())
        .min_by(|(a, part_a, _), (b, part_b, _)| {
            if presentation.subject == Subject::Player {
                let a_selected = part_a.is_some_and(|part| Some(part.0) == selected);
                let b_selected = part_b.is_some_and(|part| Some(part.0) == selected);
                if a_selected != b_selected {
                    return b_selected.cmp(&a_selected);
                }
            }
            a.translation
                .distance_squared(ball.unwrap_or(Vec3::ZERO))
                .total_cmp(&b.translation.distance_squared(ball.unwrap_or(Vec3::ZERO)))
        })
        .map(|(pose, ..)| *pose);
    let focus = match presentation.subject {
        Subject::Ball => ball.or(player.map(|pose| pose.translation)),
        Subject::Player => player.map(|pose| pose.translation).or(ball),
    }
    .unwrap_or(Vec3::ZERO);
    for (mut camera, projection) in &mut cameras {
        let aspect = match projection {
            Projection::Perspective(projection) => projection.aspect_ratio,
            _ => 1.0,
        };
        let desired = shot_pose(
            presentation.shot,
            focus,
            player,
            ball,
            parameters.field_dimensions.width,
            aspect,
        );
        *camera = interpolate(*camera, desired, 1.0 - (-3.5 * time.delta_secs()).exp());
    }
}

fn shot_pose(
    shot: Shot,
    focus: Vec3,
    player: Option<Transform>,
    ball: Option<Vec3>,
    field_width: f32,
    aspect: f32,
) -> Transform {
    let framing = aspect.max(0.25).recip().clamp(1.0, 2.0);
    let target = focus.with_y(focus.y.max(0.25));
    let (eye, target) = match shot {
        Shot::Drone => {
            let distance = (field_width * 0.8).max(4.5) * framing;
            (target + Vec3::new(-0.3, 1.0, 0.8) * distance, target)
        }
        Shot::Sideline => (
            Vec3::new(
                focus.x * 0.65,
                (field_width * 0.35).max(2.3) * framing,
                field_width / 2.0 + 2.0 * framing,
            ),
            target,
        ),
        Shot::Player => {
            let player = player.unwrap_or(Transform::from_translation(target));
            let forward = (ball.unwrap_or(target + player.rotation * Vec3::X) - player.translation)
                .with_y(0.0)
                .try_normalize()
                .unwrap_or(Vec3::X);
            let side = forward.cross(Vec3::Y);
            let eye = player.translation - forward * 2.2 * framing + side * 0.65 + Vec3::Y * 0.55;
            (
                eye.with_y(eye.y.max(1.2)),
                player.translation.lerp(target, 0.6),
            )
        }
    };
    Transform::from_translation(eye).looking_at(target, Vec3::Y)
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use bevy::time::TimeUpdateStrategy;

    use super::*;

    #[test]
    fn replay_overrides_live_sync_before_propagation_and_returns_to_live_on_scene_change() {
        fn live_sync(
            mut frame: ResMut<MujocoRenderFrame>,
            mut bodies: Query<&mut Transform, With<MujocoBody>>,
        ) {
            frame.time += 1.0;
            for mut pose in &mut bodies {
                pose.translation.x = frame.time as f32;
            }
        }

        let mut app = App::new();
        app.add_plugins((MinimalPlugins, TransformPlugin))
            .init_resource::<Presentation>()
            .init_resource::<MujocoRenderFrame>()
            .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_millis(
                100,
            )))
            .add_systems(Update, live_sync.in_set(MujocoPoseUpdateSet))
            .add_systems(Update, present_poses.after(MujocoPoseUpdateSet));
        let ball = app.world_mut().spawn_empty().id();
        app.world_mut().entity_mut(ball).insert((
            Ball,
            MujocoBody::new(ball, "ball"),
            Transform::default(),
            GlobalTransform::default(),
        ));
        app.update();
        app.update();
        {
            let mut presentation = app.world_mut().resource_mut::<Presentation>();
            presentation.replay = presentation.history.clip();
        }
        app.update();
        let displayed = app.world().get::<Transform>(ball).unwrap().translation.x;
        assert!((displayed - 1.1).abs() < 1e-5);
        assert_eq!(app.world().resource::<MujocoRenderFrame>().time, 3.0);
        assert_eq!(
            app.world()
                .get::<GlobalTransform>(ball)
                .unwrap()
                .translation()
                .x,
            displayed
        );
        let presentation = app.world().resource::<Presentation>();
        let mut latest_live = presentation.history.clip().unwrap();
        latest_live.elapsed = latest_live.duration();
        latest_live.for_each_pose(|_, pose| assert_eq!(pose.translation.x, 3.0));

        app.world_mut()
            .resource_mut::<MujocoRenderFrame>()
            .generation += 1;
        app.update();
        assert!(!app.world().resource::<Presentation>().is_replaying());
        assert_eq!(
            app.world().get::<Transform>(ball).unwrap().translation.x,
            4.0
        );
    }

    #[test]
    fn manual_camera_survives_shot_changes_and_cinematic_toggle() {
        let mut presentation = Presentation::default();
        let manual =
            Transform::from_xyz(3.0, 4.0, 5.0).looking_at(Vec3::new(1.0, 0.0, 2.0), Vec3::Y);
        let mut camera = manual;
        presentation.set_cinematic(true, &mut camera);
        camera = shot_pose(Shot::Sideline, Vec3::ZERO, None, None, 6.0, 1.0);
        presentation.set_cinematic(true, &mut camera);
        presentation.set_cinematic(false, &mut camera);
        assert_eq!(camera, manual);
    }

    #[test]
    fn shots_stay_above_ground_and_frame_their_target_without_a_ball_or_player() {
        for shot in [Shot::Drone, Shot::Sideline, Shot::Player] {
            for width in [6.0, 9.0, 14.0] {
                for aspect in [0.5, 1.0, 16.0 / 9.0] {
                    let pose = shot_pose(shot, Vec3::ZERO, None, None, width, aspect);
                    assert!(pose.translation.is_finite());
                    assert!(pose.translation.y >= 1.2);
                    assert!(pose.rotation.is_finite());
                    assert!(pose.forward().dot(-pose.translation.normalize()) > 0.95);
                }
            }
        }
    }

    #[test]
    fn camera_smoothing_is_independent_of_frame_rate() {
        let from = Transform::from_xyz(0.0, 3.0, 10.0);
        let target = Transform::from_xyz(2.0, 6.0, 4.0).with_rotation(Quat::from_rotation_y(0.7));
        let mut camera = from;
        for _ in 0..60 {
            camera = interpolate(camera, target, 1.0 - (-3.5_f32 / 60.0).exp());
        }
        let once = interpolate(from, target, 1.0 - (-3.5_f32).exp());
        assert!(camera.translation.abs_diff_eq(once.translation, 1e-5));
        assert!(camera.rotation.abs_diff_eq(once.rotation, 1e-4));
    }
}
