use bevy::prelude::*;
use types::field_dimensions::FieldDimensions;

const MIN_DWELL: f64 = 2.4;
const FALLBACK_PERIOD: f64 = 6.0;
const GOAL_HOLD: f64 = 3.5;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum Shot {
    #[default]
    Auto,
    Drone,
    Sideline,
    Player,
    Endline,
    Goal,
    Chase,
    Orbit,
    Wide,
}

impl Shot {
    pub(super) const ALL: [Self; 9] = [
        Self::Auto,
        Self::Drone,
        Self::Sideline,
        Self::Player,
        Self::Endline,
        Self::Goal,
        Self::Chase,
        Self::Orbit,
        Self::Wide,
    ];

    pub(super) fn label(self) -> &'static str {
        match self {
            Self::Auto => "Auto director",
            Self::Drone => "Drone",
            Self::Sideline => "Sideline",
            Self::Player => "Player",
            Self::Endline => "Endline",
            Self::Goal => "Goal",
            Self::Chase => "Chase",
            Self::Orbit => "Orbit",
            Self::Wide => "Wide",
        }
    }
}

#[derive(Default)]
pub(super) struct Director {
    previous: Option<(f64, Vec3)>,
    clock: Option<(f64, bool)>,
    shot: Shot,
    since: f64,
    fast: bool,
    close: bool,
    danger: bool,
    goal: Option<(f64, Vec3)>,
}

impl Director {
    pub fn goal_focus(&self) -> Option<Vec3> {
        self.goal.map(|(_, focus)| focus)
    }

    pub fn update(
        &mut self,
        time: f64,
        replaying: bool,
        ball: Option<Vec3>,
        player: Option<Vec3>,
        field: &FieldDimensions,
        goal_height: f32,
        live_goal: Option<(f64, f32)>,
    ) -> Shot {
        if !time.is_finite() {
            return Shot::Wide;
        }
        let reset = self
            .clock
            .is_none_or(|(last, was_replaying)| time < last || replaying != was_replaying);
        if reset {
            *self = Self::default();
        }
        let advanced = self.clock != Some((time, replaying));
        self.clock = Some((time, replaying));
        let ball = ball.filter(|ball| ball.is_finite());
        let velocity = self
            .previous
            .zip(ball)
            .filter(|((last, _), _)| time > *last && time - last <= 0.5)
            .map_or(Vec3::ZERO, |((last, previous), ball)| {
                (ball - previous) / (time - last) as f32
            });
        // Teleports are not kicks; retain the shot but discard their velocity spike.
        let velocity = if velocity.length_squared() <= 25.0_f32.powi(2) {
            velocity
        } else {
            Vec3::ZERO
        };
        let speed = velocity.with_y(0.0).length();
        if advanced || ball.is_none() {
            self.fast = speed > if self.fast { 1.8 } else { 2.8 };
        }
        self.close = ball.zip(player).is_some_and(|(ball, player)| {
            ball.with_y(0.0).distance(player.with_y(0.0)) < if self.close { 1.65 } else { 1.1 }
        });
        self.danger = ball.is_some_and(|ball| {
            let margin = if self.danger { 2.7 } else { 2.0 };
            ball.x.abs() > field.length / 2.0 - margin
                && ball.z.abs() < field.goal_inner_width / 2.0 + 1.0
                && (self.danger
                    || velocity.x * ball.x.signum() > 0.35
                    || ball.x.abs() > field.length / 2.0 - 0.6)
        });
        // A replay can infer a goal view from its poses, but cannot see live match events.
        let historical_goal = replaying
            && ball.is_some_and(|ball| {
                ball.x.abs() > field.length / 2.0
                    && ball.x.abs() < field.length / 2.0 + field.goal_depth + 0.5
                    && ball.z.abs() < field.goal_inner_width / 2.0
                    && ball.y < goal_height
            });
        let goal = if replaying {
            historical_goal.then(|| {
                let sign = ball.unwrap().x.signum();
                (time, sign * field.length / 2.0)
            })
        } else {
            live_goal
                .filter(|(goal, x)| x.is_finite() && (0.0..=GOAL_HOLD).contains(&(time - goal)))
        };
        let cancelled_goal = !replaying && self.goal.is_some() && goal.is_none();
        if cancelled_goal {
            self.goal = None;
        }
        if let Some((goal_time, goal_x)) = goal
            && self
                .goal
                .is_none_or(|(previous, focus)| goal_time > previous || goal_x != focus.x)
        {
            self.goal = Some((goal_time, Vec3::new(goal_x, 0.4, 0.0)));
        }
        if self.goal.is_some_and(|(goal, _)| time - goal > GOAL_HOLD) {
            self.goal = None;
        }
        if self.previous.is_none_or(|(last, _)| time > last) {
            self.previous = ball.map(|ball| (time, ball));
        }
        let desired = if self.goal.is_some() {
            Shot::Goal
        } else if self.danger {
            Shot::Endline
        } else if self.fast {
            Shot::Chase
        } else if self.close {
            Shot::Player
        } else {
            let fallback = [Shot::Wide, Shot::Drone, Shot::Sideline, Shot::Orbit];
            fallback[(time.max(0.0) / FALLBACK_PERIOD) as usize % fallback.len()]
        };
        if reset || cancelled_goal || (desired != self.shot && time - self.since >= MIN_DWELL) {
            self.shot = desired;
            self.since = time;
        }
        self.shot
    }
}

/// Bevy coordinates: X along the field, Y up, Z across the field.
/// `Auto` has no event state here and produces an establishing wide shot.
pub(crate) fn shot_pose(
    shot: Shot,
    focus: Vec3,
    player: Option<Transform>,
    ball: Option<Vec3>,
    field: &FieldDimensions,
    goal_height: f32,
    aspect: f32,
    time: f64,
) -> Transform {
    let framing = aspect.max(0.25).recip().clamp(1.0, 2.0);
    let focus = if focus.is_finite() { focus } else { Vec3::ZERO };
    let target = Vec3::new(
        focus.x.clamp(-field.length / 2.0, field.length / 2.0),
        focus.y.clamp(0.25, goal_height + 3.0),
        focus.z.clamp(-field.width / 2.0, field.width / 2.0),
    );
    let sign = if focus.x < 0.0 { -1.0 } else { 1.0 };
    let player = player
        .filter(|pose| pose.translation.is_finite() && pose.rotation.is_finite())
        .unwrap_or(Transform::from_translation(target));
    let forward = (ball.filter(|ball| ball.is_finite()).unwrap_or(target) - player.translation)
        .with_y(0.0)
        .try_normalize()
        .unwrap_or(
            (player.rotation * Vec3::X)
                .with_y(0.0)
                .normalize_or(Vec3::X),
        );
    let side = forward.cross(Vec3::Y);
    let (eye, target) = match shot {
        Shot::Auto | Shot::Wide => (
            Vec3::new(
                -field.length * 0.4,
                field.length.max(field.width) * 0.95 * framing,
                field.width * 0.8,
            ),
            target.lerp(Vec3::new(0.0, 0.25, 0.0), 0.7),
        ),
        Shot::Drone => {
            let distance = (field.width * 0.8).max(4.5) * framing;
            (target + Vec3::new(-0.3, 1.0, 0.8) * distance, target)
        }
        Shot::Sideline => (
            Vec3::new(
                focus.x * 0.65,
                (field.width * 0.35).max(2.3) * framing,
                field.width / 2.0 + 1.2,
            ),
            target,
        ),
        Shot::Player => (
            player.translation - forward * 2.2 * framing + side * 0.65 + Vec3::Y * 0.55,
            player.translation.lerp(target, 0.6),
        ),
        Shot::Endline => (
            Vec3::new(
                sign * (field.length / 2.0 + field.goal_depth + 0.5),
                2.3 * framing,
                field.goal_inner_width / 2.0 + 1.3,
            ),
            target,
        ),
        Shot::Goal => (
            Vec3::new(
                sign * (field.length / 2.0 - 3.5 * framing),
                2.4 * framing,
                field.goal_inner_width / 2.0 + 1.5 * framing,
            ),
            if (focus.x.abs() - field.length / 2.0).abs() < 0.75 {
                target.with_y(goal_height * 0.5 + 0.3)
            } else {
                target
            },
        ),
        Shot::Chase => (
            target - forward * 3.0 * framing + side * 0.5 + Vec3::Y * 1.8,
            target + forward * 0.8,
        ),
        Shot::Orbit => {
            let angle = if time.is_finite() {
                (time * 0.18).rem_euclid(std::f64::consts::TAU) as f32
            } else {
                0.0
            };
            let radius = field.width.max(4.0) * 0.55 * framing;
            (
                target + Vec3::new(angle.cos() * radius, 2.8 * framing, angle.sin() * radius),
                target,
            )
        }
    };
    let eye = safe_eye(eye, field, goal_height);
    let target = if eye.distance_squared(target) < 0.01 {
        eye + Vec3::X
    } else {
        target
    };
    Transform::from_translation(eye).looking_at(target, Vec3::Y)
}

pub(super) fn safe_eye(eye: Vec3, field: &FieldDimensions, goal_height: f32) -> Vec3 {
    let eye = if eye.is_finite() { eye } else { Vec3::Y * 3.0 };
    // Stay inside the perimeter furniture, including during interpolated transitions.
    let end = (field.length / 2.0 + field.border_strip_width.max(field.goal_depth) - 0.35).max(0.1);
    let side = (field.width / 2.0 + field.border_strip_width - 0.35).max(0.1);
    let mut eye = Vec3::new(
        eye.x.clamp(-end, end),
        eye.y.max(1.2),
        eye.z.clamp(-side, side),
    );
    let clearance = Vec2::new(
        (field.length / 2.0 - 0.35 - eye.x.abs()).max(0.0),
        (eye.z.abs() - field.goal_inner_width / 2.0 - 0.4).max(0.0),
    )
    .length();
    let near_goal = 1.0 - (clearance / 0.8).clamp(0.0, 1.0);
    let near_goal = near_goal * near_goal * (3.0 - 2.0 * near_goal);
    eye.y = eye
        .y
        .max((1.0 - near_goal) * 1.2 + near_goal * (goal_height + 0.65).max(1.2));
    eye
}

#[cfg(test)]
mod tests {
    use super::*;

    fn update(director: &mut Director, time: f64, ball: Vec3) -> Shot {
        director.update(
            time,
            false,
            Some(ball),
            None,
            &FieldDimensions::SPL_2025,
            1.1,
            None,
        )
    }

    #[test]
    fn fast_ball_respects_dwell_hysteresis_and_pauses() {
        let mut director = Director::default();
        assert_eq!(update(&mut director, 0.0, Vec3::ZERO), Shot::Wide);
        assert_eq!(update(&mut director, 0.1, Vec3::X * 0.4), Shot::Wide);
        assert!(director.fast);
        update(&mut director, 2.3, Vec3::ZERO);
        assert_eq!(update(&mut director, 2.4, Vec3::X * 0.4), Shot::Chase);
        assert_eq!(update(&mut director, 2.5, Vec3::X * 0.62), Shot::Chase);
        assert!(
            director.fast,
            "speed between entry and exit thresholds stays fast"
        );
        update(&mut director, 2.5, Vec3::X * 0.62);
        assert!(
            director.fast,
            "rendering a paused frame must not change the shot state"
        );
        assert_eq!(update(&mut director, 2.6, Vec3::X * 0.79), Shot::Chase);
        assert!(!director.fast);
        assert_eq!(update(&mut director, 4.81, Vec3::X * 0.79), Shot::Wide);
    }

    #[test]
    fn approaching_either_goal_and_player_proximity_choose_different_views() {
        for sign in [-1.0, 1.0] {
            let mut director = Director::default();
            update(&mut director, 0.0, Vec3::new(sign * 2.0, 0.1, 0.0));
            update(&mut director, 2.4, Vec3::new(sign * 2.3, 0.1, 0.0));
            assert_eq!(
                update(&mut director, 2.5, Vec3::new(sign * 2.7, 0.1, 0.0)),
                Shot::Endline
            );
        }
        let mut director = Director::default();
        let mut near = |time, distance| {
            director.update(
                time,
                false,
                Some(Vec3::ZERO),
                Some(Vec3::new(distance, 0.8, 0.0)),
                &FieldDimensions::SPL_2025,
                1.1,
                None,
            )
        };
        assert_eq!(near(0.0, 0.9), Shot::Player);
        assert_eq!(near(2.5, 1.4), Shot::Player);
        assert_eq!(near(2.6, 1.7), Shot::Wide);
    }

    #[test]
    fn replay_ignores_live_goals_and_resets_velocity_and_dwell() {
        let field = FieldDimensions::SPL_2025;
        let mut director = Director::default();
        assert_eq!(
            director.update(
                9.0,
                false,
                Some(Vec3::ZERO),
                None,
                &field,
                1.1,
                Some((9.0, -4.5))
            ),
            Shot::Goal
        );
        assert_eq!(
            director.update(
                1.0,
                true,
                Some(Vec3::ZERO),
                None,
                &field,
                1.1,
                Some((9.0, -4.5))
            ),
            Shot::Wide
        );
        assert!(director.goal_focus().is_none());
        assert_eq!(
            director.update(
                1.1,
                false,
                Some(Vec3::ZERO),
                None,
                &field,
                1.1,
                Some((9.0, -4.5))
            ),
            Shot::Wide,
            "future events cannot affect a rewound simulation"
        );
        assert_eq!(
            director.update(
                0.0,
                true,
                Some(Vec3::new(-field.length / 2.0 - 0.1, 0.1, 0.0)),
                None,
                &field,
                1.1,
                Some((9.0, 4.5)),
            ),
            Shot::Goal,
            "replay goal framing comes from historical ball poses"
        );
        assert!(director.goal_focus().unwrap().x < 0.0);
    }

    #[test]
    fn live_goal_focus_survives_ball_reset_and_undo_cancels_the_hold_while_paused() {
        let field = FieldDimensions::SPL_2025;
        let mut director = Director::default();
        assert_eq!(
            director.update(
                1.0,
                false,
                Some(Vec3::ZERO),
                None,
                &field,
                1.1,
                Some((1.0, -field.length / 2.0)),
            ),
            Shot::Goal
        );
        assert_eq!(director.goal_focus().unwrap().x, -field.length / 2.0);
        assert_eq!(
            director.update(1.0, false, Some(Vec3::ZERO), None, &field, 1.1, None),
            Shot::Wide
        );
        assert!(director.goal_focus().is_none());
    }

    #[test]
    fn fallback_is_deterministic_and_teleports_do_not_trigger_chase() {
        let mut director = Director::default();
        for (time, expected) in [
            (0.0, Shot::Wide),
            (6.0, Shot::Drone),
            (12.0, Shot::Sideline),
            (18.0, Shot::Orbit),
            (24.0, Shot::Wide),
        ] {
            assert_eq!(update(&mut director, time, Vec3::ZERO), expected);
        }
        assert_eq!(update(&mut director, 24.01, Vec3::Z * 10.0), Shot::Wide);
        assert!(!director.fast);
        assert_eq!(update(&mut director, 0.0, Vec3::ZERO), Shot::Wide);
        assert!(director.goal_focus().is_none());
    }

    #[test]
    fn camera_eyes_and_intermediate_poses_clear_goal_and_perimeter() {
        let field = FieldDimensions::SPL_2025;
        let height = 2.8;
        let mut previous = Transform::from_xyz(-field.length, -1.0, 0.0);
        for shot in Shot::ALL {
            for sign in [-1.0, 1.0] {
                for focus in [
                    Vec3::new(sign * field.length / 2.0, 0.1, 0.0),
                    Vec3::new(sign * field.length, 0.1, field.width),
                    Vec3::ZERO,
                    Vec3::NAN,
                ] {
                    let pose = shot_pose(
                        shot,
                        focus,
                        Some(Transform::from_translation(focus)),
                        Some(focus),
                        &field,
                        height,
                        0.4,
                        8.0,
                    );
                    assert!(pose.translation.is_finite());
                    assert!(pose.rotation.is_finite());
                    for fraction in [0.0, 0.25, 0.5, 0.75, 1.0] {
                        let eye = safe_eye(
                            previous.translation.lerp(pose.translation, fraction),
                            &field,
                            height,
                        );
                        assert!(eye.is_finite());
                        assert!(eye.y >= 1.2);
                        assert!(eye.x.abs() < field.length / 2.0 + field.border_strip_width);
                        assert!(eye.z.abs() < field.width / 2.0 + field.border_strip_width);
                        if eye.x.abs() >= field.length / 2.0 - 0.35
                            && eye.z.abs() < field.goal_inner_width / 2.0 + 0.4
                        {
                            assert!(eye.y >= height + 0.65);
                        }
                    }
                    previous = pose;
                }
            }
        }
        let entry = field.length / 2.0 - 0.35;
        let before = safe_eye(Vec3::new(entry - 0.0001, 1.2, 0.0), &field, height);
        let after = safe_eye(Vec3::new(entry + 0.0001, 1.2, 0.0), &field, height);
        assert!(
            before.abs_diff_eq(after, 0.001),
            "goal clearance must not snap vertically"
        );
    }
}
