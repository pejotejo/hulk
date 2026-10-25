//! Score-driven presentation only. Neither the particle pool nor its clock writes physics.
use std::sync::{Arc, Mutex};

use bevy::prelude::*;
use game_controller_core::types::{Phase, Side, SideMapping};

use crate::{
    bevy_mujoco::MujocoRenderFrame,
    presentation::{Presentation, PresentationUpdateSet},
    team::{Member, Team},
};

pub(crate) const PARTICLE_COUNT: usize = 128;
const LIFETIME: f64 = 3.5;
pub(crate) const CONFETTI_COLORS: [[f32; 4]; 4] = [
    [0.98, 0.80, 0.16, 1.0],
    [0.10, 0.80, 0.55, 1.0],
    [0.98, 0.98, 0.94, 1.0],
    [0.13, 0.63, 0.93, 1.0],
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Score {
    pub goals: [u16; 2],
    pub shootout: bool,
    pub home_defends_negative: bool,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct GoalBurst {
    pub time: f64,
    pub goal_x: f32,
    pub goal_width: f32,
}

#[derive(Default)]
struct Tracker {
    score: Option<Score>,
    time: f64,
    burst: Option<GoalBurst>,
}

#[derive(Clone, Default, Resource)]
pub(crate) struct MatchEffects(Arc<Mutex<Tracker>>);

impl MatchEffects {
    pub(crate) fn goal(&self) -> Option<GoalBurst> {
        self.0.lock().unwrap().burst
    }

    pub(crate) fn burst(&self, time: f64) -> Option<GoalBurst> {
        self.goal()
            .filter(|burst| time >= burst.time && time - burst.time < LIFETIME)
    }

    fn observe(&self, time: f64, score: Score, length: f32, goal_width: f32) {
        if !time.is_finite() {
            return;
        }
        let mut state = self.0.lock().unwrap();
        if time < state.time
            || state.score.is_some_and(|old| {
                old.home_defends_negative != score.home_defends_negative
                    || old.shootout != score.shootout
                    || old
                        .goals
                        .iter()
                        .zip(score.goals)
                        .any(|(old, new)| new < *old)
            })
        {
            state.burst = None;
            state.score = None;
        }
        if let Some(old) = state.score {
            let scorer = match (
                score.goals[0].checked_sub(old.goals[0]),
                score.goals[1].checked_sub(old.goals[1]),
            ) {
                (Some(1), Some(0)) => Some(0),
                (Some(0), Some(1)) => Some(1),
                _ => None,
            };
            if let Some(scorer) = scorer {
                let positive = (scorer == 0) == score.home_defends_negative;
                state.burst = Some(GoalBurst {
                    time,
                    goal_x: if positive {
                        length / 2.0
                    } else {
                        -length / 2.0
                    },
                    goal_width,
                });
            } else if score.goals != old.goals {
                state.burst = None;
            }
        }
        state.score = Some(score);
        state.time = time;
    }
}

/// Called at sensor-frame cadence, including paused match-desk changes.
pub(crate) fn observe_score(team: &Team, members: &[Member], time: f64) {
    let score = if let Some(referee) = team.referee() {
        let engine = referee.engine.lock().unwrap();
        let game = engine.core.get_game(false);
        let shootout = game.phase == Phase::PenaltyShootout;
        Some(Score {
            goals: [Side::Home, Side::Away].map(|side| {
                if shootout {
                    game.teams[side].penalty_shot_mask.count_ones() as u16
                } else {
                    u16::from(game.teams[side].score)
                }
            }),
            shootout,
            home_defends_negative: game.sides == SideMapping::HomeDefendsLeftGoal,
        })
    } else {
        members
            .iter()
            .find(|member| member.id.team == crate::TeamId::Hulks)
            .and_then(|member| member.io.presentation_score())
    };
    if let Some(score) = score {
        let field = &team
            .configuration()
            .field_configuration
            .as_ref()
            .unwrap()
            .dimensions;
        team.effects()
            .observe(time, score, field.length, field.goal_inner_width);
    }
}

/// Same deterministic trajectory in Bevy and the headless visual-only user scene.
pub(crate) fn particles(burst: GoalBurst, time: f64) -> impl Iterator<Item = (Transform, usize)> {
    let age = (time - burst.time) as f32;
    let active = (0.0..LIFETIME as f32).contains(&age);
    (0..if active { PARTICLE_COUNT } else { 0 }).map(move |index| {
        let fraction = ((index * 73 % PARTICLE_COUNT) as f32 + 0.5) / PARTICLE_COUNT as f32;
        let side = if index % 2 == 0 { -1.0 } else { 1.0 };
        let start = Vec3::new(burst.goal_x, 0.6, side * (burst.goal_width / 2.0 + 0.35));
        let velocity = Vec3::new(
            -burst.goal_x.signum() * (0.15 + fraction * 1.3),
            2.6 + fraction * 1.8,
            -side * (0.35 + (index % 11) as f32 * 0.12),
        );
        let position = (start + velocity * age - Vec3::Y * (1.25 * age * age))
            .with_y((start.y + velocity.y * age - 1.25 * age * age).max(0.04));
        let spin = index as f32 * 1.71 + age * (2.0 + fraction * 4.0);
        (
            Transform::from_translation(position).with_rotation(Quat::from_euler(
                EulerRot::XYZ,
                spin,
                spin * 0.7,
                spin * 0.3,
            )),
            index % CONFETTI_COLORS.len(),
        )
    })
}

#[derive(Component)]
struct Confetti;

#[derive(Default, Resource)]
struct Pool {
    entities: Vec<Entity>,
    visible: bool,
}

pub(crate) struct MatchEffectsPlugin;
impl Plugin for MatchEffectsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Pool>()
            .add_systems(Update, draw_confetti.after(PresentationUpdateSet));
    }
}

fn draw_confetti(
    mut commands: Commands,
    presentation: Res<Presentation>,
    frame: Res<MujocoRenderFrame>,
    effects: Res<MatchEffects>,
    mut pool: ResMut<Pool>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut query: Query<(&mut Transform, &mut Visibility), With<Confetti>>,
) {
    let burst = (!presentation.is_replaying())
        .then(|| effects.burst(frame.time))
        .flatten();
    if let Some(burst) = burst {
        if pool.entities.is_empty() {
            let mesh = meshes.add(Rectangle::new(0.075, 0.035));
            let colors: Vec<_> = CONFETTI_COLORS
                .iter()
                .map(|color| {
                    materials.add(StandardMaterial {
                        base_color: Color::srgba(color[0], color[1], color[2], color[3]),
                        unlit: true,
                        cull_mode: None,
                        ..default()
                    })
                })
                .collect();
            for (pose, color) in particles(burst, frame.time) {
                pool.entities.push(
                    commands
                        .spawn((
                            Confetti,
                            Mesh3d(mesh.clone()),
                            MeshMaterial3d(colors[color].clone()),
                            pose,
                            Visibility::Visible,
                        ))
                        .id(),
                );
            }
        } else {
            for (entity, (pose, _)) in pool.entities.iter().zip(particles(burst, frame.time)) {
                if let Ok((mut transform, mut visibility)) = query.get_mut(*entity) {
                    *transform = pose;
                    *visibility = Visibility::Visible;
                }
            }
        }
        pool.visible = true;
    } else if pool.visible {
        for (_, mut visibility) in &mut query {
            *visibility = Visibility::Hidden;
        }
        pool.visible = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn score_changes_celebrate_once_and_undo_reset_and_new_sessions_do_not_celebrate() {
        let effects = MatchEffects::default();
        let score = |goals| Score {
            goals,
            shootout: false,
            home_defends_negative: true,
        };
        effects.observe(0.0, score([0, 0]), 14.0, 2.6);
        assert!(effects.goal().is_none());
        effects.observe(1.0, score([1, 0]), 14.0, 2.6);
        assert_eq!(effects.burst(1.0).unwrap().goal_x, 7.0);
        effects.observe(2.0, score([1, 0]), 14.0, 2.6);
        assert_eq!(effects.goal().unwrap().time, 1.0);
        assert!(effects.burst(5.0).is_none());
        effects.observe(2.0, score([0, 0]), 14.0, 2.6);
        assert!(effects.goal().is_none());
        effects.observe(3.0, score([0, 1]), 14.0, 2.6);
        assert_eq!(effects.burst(3.0).unwrap().goal_x, -7.0);
        let shootout = |goals| Score {
            goals,
            shootout: true,
            home_defends_negative: true,
        };
        effects.observe(4.0, shootout([0, 0]), 14.0, 2.6);
        assert!(effects.goal().is_none());
        effects.observe(4.5, shootout([1, 0]), 14.0, 2.6);
        assert_eq!(effects.burst(4.5).unwrap().goal_x, 7.0);
        effects.observe(4.5, shootout([0, 0]), 14.0, 2.6);
        assert!(effects.goal().is_none());
        effects.observe(0.0, score([4, 4]), 14.0, 2.6);
        assert!(effects.goal().is_none());
        let new = MatchEffects::default();
        new.observe(0.0, score([2, 0]), 14.0, 2.6);
        assert!(new.goal().is_none());
    }

    #[test]
    fn particles_are_bounded_deterministic_and_expire_in_simulation_time() {
        let burst = GoalBurst {
            time: 2.0,
            goal_x: 7.0,
            goal_width: 2.6,
        };
        assert_eq!(particles(burst, 3.0).count(), PARTICLE_COUNT);
        assert!(particles(burst, 3.0).eq(particles(burst, 3.0)));
        assert_eq!(particles(burst, 1.0).count(), 0);
        assert_eq!(particles(burst, 5.5).count(), 0);
        for (pose, color) in particles(burst, 4.0) {
            assert!(pose.translation.is_finite() && pose.translation.y >= 0.04);
            assert!(color < CONFETTI_COLORS.len());
        }
    }

    #[test]
    fn rendered_pool_is_reused_hidden_after_expiry_and_never_contains_physics_bodies() {
        let effects = MatchEffects::default();
        let score = |goals| Score {
            goals,
            shootout: false,
            home_defends_negative: true,
        };
        effects.observe(0.0, score([0, 0]), 14.0, 2.6);
        let mut app = App::new();
        app.add_plugins(MinimalPlugins)
            .init_resource::<Presentation>()
            .init_resource::<MujocoRenderFrame>()
            .init_resource::<Assets<Mesh>>()
            .init_resource::<Assets<StandardMaterial>>()
            .insert_resource(effects.clone())
            .add_plugins(MatchEffectsPlugin);
        app.update();
        assert!(app.world().resource::<Pool>().entities.is_empty());
        effects.observe(1.0, score([1, 0]), 14.0, 2.6);
        app.world_mut().resource_mut::<MujocoRenderFrame>().time = 1.5;
        app.update();
        let original = app.world().resource::<Pool>().entities.clone();
        assert_eq!(original.len(), PARTICLE_COUNT);
        assert!(original.iter().all(|entity| {
            app.world()
                .get::<crate::bevy_mujoco::MujocoBody>(*entity)
                .is_none()
        }));
        app.world_mut().resource_mut::<MujocoRenderFrame>().time = 5.0;
        app.update();
        assert!(
            original
                .iter()
                .all(|entity| app.world().get::<Visibility>(*entity) == Some(&Visibility::Hidden))
        );
        effects.observe(6.0, score([2, 0]), 14.0, 2.6);
        app.world_mut().resource_mut::<MujocoRenderFrame>().time = 6.5;
        app.update();
        assert_eq!(app.world().resource::<Pool>().entities, original);
        assert_eq!(app.world().resource::<Assets<Mesh>>().len(), 1);
        assert_eq!(
            app.world().resource::<Assets<StandardMaterial>>().len(),
            CONFETTI_COLORS.len()
        );
        effects.observe(6.5, score([1, 0]), 14.0, 2.6);
        app.update();
        assert!(
            original
                .iter()
                .all(|entity| app.world().get::<Visibility>(*entity) == Some(&Visibility::Hidden))
        );
    }
}
