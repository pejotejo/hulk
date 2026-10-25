use std::collections::VecDeque;

use bevy::prelude::*;

const WINDOW: f64 = 10.0;
const SAMPLE_PERIOD: f64 = 1.0 / 30.0;
const MAX_FRAMES: usize = 301;

#[derive(Clone, Default)]
struct Frame {
    time: f64,
    poses: Vec<(Entity, Transform)>,
}

#[derive(Default)]
pub(super) struct History {
    // shortcut: sample rendered poses; capture on the worker if background replay is needed.
    frames: VecDeque<Frame>,
    generation: Option<u64>,
}

impl History {
    /// Returns true when the scene or clock changed and an old clip is invalid.
    pub fn record(
        &mut self,
        time: f64,
        generation: u64,
        poses: impl Iterator<Item = (Entity, Transform)>,
    ) -> bool {
        if !time.is_finite() {
            return false;
        }
        let reset = self.generation != Some(generation)
            || self.frames.back().is_some_and(|last| time < last.time);
        if reset {
            self.frames.clear();
            self.generation = Some(generation);
        }
        if self
            .frames
            .back()
            .is_some_and(|last| time - last.time < SAMPLE_PERIOD - 1e-9)
        {
            return reset;
        }

        // Reuse the evicted pose allocation once the rolling window is full.
        let mut frame = if self.frames.len() >= MAX_FRAMES
            || self
                .frames
                .front()
                .is_some_and(|first| time - first.time > WINDOW)
        {
            self.frames.pop_front().unwrap()
        } else {
            Frame::default()
        };
        while self
            .frames
            .front()
            .is_some_and(|first| time - first.time > WINDOW)
        {
            self.frames.pop_front();
        }
        frame.time = time;
        frame.poses.clear();
        frame.poses.extend(poses);
        frame
            .poses
            .sort_unstable_by_key(|(entity, _)| entity.to_bits());
        self.frames.push_back(frame);
        reset
    }

    pub fn duration(&self) -> f64 {
        self.frames
            .front()
            .zip(self.frames.back())
            .map_or(0.0, |(first, last)| last.time - first.time)
    }

    pub fn clip(&self) -> Option<Replay> {
        (self.duration() >= 0.1).then(|| Replay {
            // Copy poses once per replay; live history keeps recording independently.
            frames: self.frames.iter().cloned().collect(),
            elapsed: 0.0,
        })
    }
}

pub(super) struct Replay {
    frames: Vec<Frame>,
    pub elapsed: f64,
}

impl Replay {
    pub fn timestamp(&self) -> f64 {
        self.frames[0].time + self.elapsed.clamp(0.0, self.duration())
    }

    pub fn duration(&self) -> f64 {
        self.frames.last().unwrap().time - self.frames[0].time
    }

    pub fn advance(&mut self, seconds: f64) -> bool {
        self.elapsed += seconds.max(0.0);
        self.elapsed < self.duration()
    }

    pub fn for_each_pose(&self, mut apply: impl FnMut(Entity, Transform)) {
        let (before, after, fraction) = self.sample();
        for (entity, pose) in &before.poses {
            let Ok(index) = after
                .poses
                .binary_search_by_key(&entity.to_bits(), |(entity, _)| entity.to_bits())
            else {
                continue;
            };
            apply(*entity, interpolate(*pose, after.poses[index].1, fraction));
        }
    }

    pub fn pose(&self, entity: Entity) -> Option<Transform> {
        let (before, after, fraction) = self.sample();
        let lookup = |frame: &Frame| {
            frame
                .poses
                .binary_search_by_key(&entity.to_bits(), |(entity, _)| entity.to_bits())
                .ok()
                .map(|index| frame.poses[index].1)
        };
        Some(interpolate(lookup(before)?, lookup(after)?, fraction))
    }

    fn sample(&self) -> (&Frame, &Frame, f32) {
        let time = self.timestamp();
        let upper = self
            .frames
            .partition_point(|frame| frame.time <= time)
            .min(self.frames.len() - 1);
        let before = &self.frames[upper.saturating_sub(1)];
        let after = &self.frames[upper];
        let fraction = if after.time > before.time {
            ((time - before.time) / (after.time - before.time)).clamp(0.0, 1.0) as f32
        } else {
            0.0
        };
        (before, after, fraction)
    }
}

pub(super) fn interpolate(from: Transform, to: Transform, fraction: f32) -> Transform {
    Transform {
        translation: from.translation.lerp(to.translation, fraction),
        rotation: from.rotation.slerp(to.rotation, fraction),
        scale: from.scale.lerp(to.scale, fraction),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entity() -> Entity {
        World::new().spawn_empty().id()
    }

    #[test]
    fn history_is_bounded_by_simulation_time_and_pauses_do_not_add_frames() {
        let mut history = History::default();
        let entity = entity();
        for step in 0..6000 {
            history.record(
                step as f64 / 120.0,
                1,
                std::iter::once((entity, Transform::from_xyz(step as f32, 0.0, 0.0))),
            );
        }
        assert!((9.9..=WINDOW).contains(&history.duration()));
        assert!(history.frames.len() <= MAX_FRAMES);
        let length = history.frames.len();
        let time = history.frames.back().unwrap().time;
        for _ in 0..1000 {
            history.record(time, 1, std::iter::empty());
        }
        assert_eq!(history.frames.len(), length);
        assert_eq!(history.frames.back().unwrap().poses.len(), 1);
    }

    #[test]
    fn scene_changes_and_clock_rewinds_invalidate_history() {
        let mut history = History::default();
        assert!(history.record(1.0, 1, std::iter::empty()));
        assert!(!history.record(2.0, 1, std::iter::empty()));
        assert!(history.record(2.0, 2, std::iter::empty()));
        assert!(history.clip().is_none());
        history.record(3.0, 2, std::iter::empty());
        assert!(history.record(0.0, 2, std::iter::empty()));
        assert_eq!(history.duration(), 0.0);
    }

    #[test]
    fn replay_interpolates_articulated_poses_and_keeps_its_original_clip() {
        let mut world = World::new();
        let ball = world.spawn_empty().id();
        let head = world.spawn_empty().id();
        let mut history = History::default();
        for time in [0.0, 1.0] {
            history.record(
                time,
                1,
                [
                    (ball, Transform::from_xyz(time as f32 * 2.0, 0.1, 0.0)),
                    (
                        head,
                        Transform::from_xyz(0.0, 0.8, time as f32)
                            .with_rotation(Quat::from_rotation_y(time as f32)),
                    ),
                ]
                .into_iter(),
            );
        }
        let mut replay = history.clip().unwrap();
        history.record(20.0, 1, std::iter::empty());
        assert!(replay.advance(0.5));
        let mut count = 0;
        replay.for_each_pose(|entity, pose| {
            count += 1;
            if entity == ball {
                assert!(pose.translation.abs_diff_eq(Vec3::new(1.0, 0.1, 0.0), 1e-5));
            } else {
                assert!(pose.translation.abs_diff_eq(Vec3::new(0.0, 0.8, 0.5), 1e-5));
                assert!(pose.rotation.abs_diff_eq(Quat::from_rotation_y(0.5), 1e-5));
            }
        });
        assert_eq!(count, 2);
        assert_eq!(replay.duration(), 1.0);
        assert!(!replay.advance(0.5));
    }

    #[test]
    fn replay_timestamp_and_subject_poses_use_the_captured_clock() {
        let mut world = World::new();
        let ball = world.spawn_empty().id();
        let live_only = world.spawn_empty().id();
        let mut history = History::default();
        for time in [12.0, 13.0] {
            history.record(
                time,
                1,
                std::iter::once((ball, Transform::from_xyz(time as f32, 0.1, 0.0))),
            );
        }
        let mut replay = history.clip().unwrap();
        assert!(replay.advance(0.25));
        assert_eq!(replay.timestamp(), 12.25);
        assert_eq!(replay.pose(ball).unwrap().translation.x, 12.25);
        assert!(replay.pose(live_only).is_none());
        history.record(99.0, 1, std::iter::empty());
        assert_eq!(replay.timestamp(), 12.25);
        replay.advance(50.0);
        assert_eq!(replay.timestamp(), 13.0);
    }
}
