use std::{
    f32::consts::{FRAC_PI_2, PI, TAU},
    sync::Arc,
    time::Duration,
};

use color_eyre::Report;
use coordinate_systems::{Camera, Field, Pixel};
use eframe::egui::{Color32, Stroke};
use linear_algebra::{Isometry3, Point2, Point3, point};
use projection::{camera_matrix::CameraMatrix, intrinsic::Intrinsic};
use ros_z::{
    qos::{QosDurability, QosProfile},
    time::Time,
};
use ros_z_debug::{
    ObservationPolicy, SampleRecord,
    query::{TemporalMatch, TimeProjection, plan::Selection},
};
use twix_visualization::twix_painter::TwixPainter;
use types::{
    field_dimensions::{FieldDimensions, Half, Side},
    localization::{LocalizationEstimate, LocalizationStatus},
    time_wrapper::TimeWrapper,
    visual_localization::VisualLocalizationFrame,
};

use super::{
    super::{
        image_overlay::ImageOverlay,
        inputs::{OverlayInput, OverlayInputs},
    },
    camera::{interpolate_transform, projected_camera_at},
};

const NEAR_Z: f32 = 1.0e-4;
const FIELD_STROKE: Stroke = Stroke {
    width: 2.0,
    color: Color32::from_rgb(80, 220, 255),
};
const RESIDUAL_STROKE: Stroke = Stroke {
    width: 2.0,
    color: Color32::from_rgb(255, 80, 200),
};

#[derive(Default)]
pub(in crate::panels::image) struct ProjectedFieldLinesOverlay {
    sample: Option<ProjectedFieldSample>,
}

impl ImageOverlay for ProjectedFieldLinesOverlay {
    fn inputs(&self) -> Vec<OverlayInput> {
        let latched = ObservationPolicy::default().with_subscriber_qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        });
        let mut dimensions =
            OverlayInput::new::<FieldDimensions>("field_dimensions", Selection::latest());
        dimensions.source.projection = TimeProjection::SourceTime;
        dimensions.source.live_policy = latched;
        let mut status =
            OverlayInput::new::<LocalizationStatus>("localization/status", Selection::latest());
        status.source.live_policy = latched;
        vec![
            OverlayInput::new::<TimeWrapper<CameraMatrix>>(
                "camera_matrix",
                Selection::Bracket {
                    max_gap: Duration::from_millis(100),
                },
            ),
            OverlayInput::new::<LocalizationEstimate>(
                "localization/estimate",
                Selection::Bracket {
                    max_gap: Duration::from_millis(100),
                },
            ),
            status,
            dimensions,
            OverlayInput::new::<TimeWrapper<VisualLocalizationFrame>>(
                "field_mark_association/visual_localization_local",
                Selection::Exact,
            ),
        ]
    }

    fn prepare(&mut self, time: Time, inputs: &mut OverlayInputs) -> Result<(), Report> {
        self.sample = None;
        let Some(status) = inputs.decode::<LocalizationStatus>(2, time)? else {
            return Ok(());
        };
        let status = &status.last().value;
        let Some(poses) = inputs.decode::<LocalizationEstimate>(1, time)? else {
            return Ok(());
        };
        let (before, after) = match &poses {
            TemporalMatch::Exact(record) => (&record.value, &record.value),
            TemporalMatch::Bracket { before, after } => (&before.value, &after.value),
            TemporalMatch::Before(_) => return Ok(()),
        };
        if [before, after]
            .iter()
            .any(|pose| pose.epoch != status.epoch || pose.generation != status.generation)
        {
            return Ok(());
        }
        let (Some(before_pose), Some(after_pose)) = (before.robot_to_field, after.robot_to_field)
        else {
            return Ok(());
        };
        if !before_pose
            .pose
            .inner
            .to_homogeneous()
            .iter()
            .chain(after_pose.pose.inner.to_homogeneous().iter())
            .all(|v| v.is_finite())
        {
            return Ok(());
        }
        let gap = after.time.duration_since(before.time);
        let fraction = if gap.is_zero() {
            0.0
        } else {
            time.duration_since(before.time).div_duration_f32(gap)
        };
        let field_to_robot = interpolate_transform(
            Isometry3::wrap(before_pose.pose.inner.cast::<f32>().inverse()),
            Isometry3::wrap(after_pose.pose.inner.cast::<f32>().inverse()),
            fraction,
        );
        let Some(camera) = inputs
            .decode::<TimeWrapper<CameraMatrix>>(0, time)?
            .and_then(|samples| projected_camera_at(&samples, time))
        else {
            return Ok(());
        };
        let dimensions = inputs
            .decode::<FieldDimensions>(3, time)
            .ok()
            .flatten()
            .map_or(FieldDimensions::SPL_2025, |samples| samples.last().value);
        let associations = inputs
            .decode::<TimeWrapper<VisualLocalizationFrame>>(4, time)
            .ok()
            .flatten()
            .filter(|samples| matches!(samples, TemporalMatch::Exact(_)))
            .map(|samples| samples.last().clone())
            .filter(|record| {
                record.value.inner.epoch == status.epoch
                    && record.value.inner.generation == status.generation
            });
        self.sample = Some(ProjectedFieldSample {
            projection: FieldProjection {
                field_to_camera: camera.robot_to_camera * field_to_robot,
                intrinsics: camera.intrinsics,
            },
            dimensions,
            associations,
        });
        Ok(())
    }

    fn paint(&self, painter: &TwixPainter<Pixel>) {
        let Some(sample) = &self.sample else { return };
        sample.projection.draw_field(painter, &sample.dimensions);
        if let Some(frame) = &sample.associations {
            for association in &frame.value.inner.associations {
                if !association.detection.inner.iter().all(|v| v.is_finite()) {
                    continue;
                }
                if let Some(projected) = sample.projection.project(association.field_point) {
                    painter.line_segment(association.detection, projected, RESIDUAL_STROKE);
                    painter.circle_filled(projected, 3.5, RESIDUAL_STROKE.color);
                }
            }
        }
    }
}

struct ProjectedFieldSample {
    projection: FieldProjection,
    dimensions: FieldDimensions,
    associations: Option<Arc<SampleRecord<TimeWrapper<VisualLocalizationFrame>>>>,
}

struct FieldProjection {
    field_to_camera: Isometry3<Field, Camera>,
    intrinsics: Intrinsic,
}

impl FieldProjection {
    fn project(&self, point: Point3<Field>) -> Option<Point2<Pixel>> {
        self.project_camera(self.field_to_camera * point)
    }

    fn project_camera(&self, point: Point3<Camera>) -> Option<Point2<Pixel>> {
        if !point.inner.iter().all(|v| v.is_finite())
            || point.z() < NEAR_Z
            || !self.intrinsics.is_valid()
        {
            return None;
        }
        let pixel = self.intrinsics.project(point.coords());
        pixel.inner.iter().all(|v| v.is_finite()).then_some(pixel)
    }

    fn segment(&self, start: Point2<Field>, end: Point2<Field>) -> Option<[Point2<Pixel>; 2]> {
        let mut start = self.field_to_camera * start.extend(0.0);
        let mut end = self.field_to_camera * end.extend(0.0);
        if !start
            .inner
            .iter()
            .chain(end.inner.iter())
            .all(|v| v.is_finite())
            || (start.z() < NEAR_Z && end.z() < NEAR_Z)
        {
            return None;
        }
        if start.z() < NEAR_Z || end.z() < NEAR_Z {
            let t = (NEAR_Z - start.z()) / (end.z() - start.z());
            let mut clipped = start + (end - start) * t;
            // Roundoff in interpolation must not put the clipped endpoint behind the plane.
            clipped.inner.z = NEAR_Z;
            if start.z() < NEAR_Z {
                start = clipped;
            } else {
                end = clipped;
            }
        }
        Some([self.project_camera(start)?, self.project_camera(end)?])
    }

    fn draw_field(&self, painter: &TwixPainter<Pixel>, d: &FieldDimensions) {
        let line = |a, b| {
            if let Some([a, b]) = self.segment(a, b) {
                painter.line_segment(a, b, FIELD_STROKE);
            }
        };
        line(d.t_crossing(Side::Left), d.t_crossing(Side::Right));
        for side in [Side::Left, Side::Right] {
            line(d.corner(Half::Own, side), d.corner(Half::Opponent, side));
        }
        self.arc(
            painter,
            d.center(),
            d.center_circle_diameter / 2.0,
            0.0,
            TAU,
        );
        for half in [Half::Own, Half::Opponent] {
            line(d.corner(half, Side::Left), d.corner(half, Side::Right));
            line(
                d.goal_box_corner(half, Side::Left),
                d.goal_box_corner(half, Side::Right),
            );
            line(
                d.penalty_box_corner(half, Side::Left),
                d.penalty_box_corner(half, Side::Right),
            );
            let spot = d.penalty_spot(half);
            let r = d.penalty_marker_size / 2.0;
            line(point![spot.x() - r, 0.0], point![spot.x() + r, 0.0]);
            line(point![spot.x(), -r], point![spot.x(), r]);
            for side in [Side::Left, Side::Right] {
                line(
                    d.goal_box_corner(half, side),
                    d.goal_box_goal_line_intersection(half, side),
                );
                line(
                    d.penalty_box_corner(half, side),
                    d.penalty_box_goal_line_intersection(half, side),
                );
                let start = match (half, side) {
                    (Half::Own, Side::Left) => -FRAC_PI_2,
                    (Half::Own, Side::Right) => 0.0,
                    (Half::Opponent, Side::Left) => PI,
                    (Half::Opponent, Side::Right) => FRAC_PI_2,
                };
                self.arc(
                    painter,
                    d.corner(half, side),
                    d.corner_arc_radius,
                    start,
                    start + FRAC_PI_2,
                );
            }
        }
    }

    fn arc(
        &self,
        painter: &TwixPainter<Pixel>,
        center: Point2<Field>,
        radius: f32,
        start: f32,
        end: f32,
    ) {
        if !radius.is_finite() || radius <= 0.0 {
            return;
        }
        let count = ((end - start).abs() / TAU * 256.0).ceil().clamp(1.0, 256.0) as usize;
        let at = |angle: f32| {
            self.project(point![
                center.x() + radius * angle.cos(),
                center.y() + radius * angle.sin(),
                0.0
            ])
        };
        let mut previous = at(start);
        for index in 1..=count {
            let next = at(start + (end - start) * index as f32 / count as f32);
            if let (Some(a), Some(b)) = (previous, next) {
                painter.line_segment(a, b, FIELD_STROKE);
            }
            previous = next;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn query_inputs_interpolate_matching_epochs_and_clear_invalid_geometry() {
        use crate::panels::image::tests::raw_sample;
        use ros_z_debug::query::plan::State;
        use types::localization::{LocalizationState, PoseEstimate};

        let estimate = |nanos, x, epoch| LocalizationEstimate {
            time: Time::from_nanos(nanos),
            epoch,
            generation: 2,
            robot_to_local: PoseEstimate {
                pose: Isometry3::identity(),
                covariance: nalgebra::SMatrix::identity(),
            },
            robot_to_field: Some(PoseEstimate {
                pose: Isometry3::from_translation(x, 0.0, 0.0),
                covariance: nalgebra::SMatrix::identity(),
            }),
        };
        let before = estimate(10, 2.0, 1);
        let mut after = estimate(30, 6.0, 1);
        let camera = TimeWrapper {
            time: Time::from_nanos(20),
            inner: CameraMatrix::default(),
        };
        let status = LocalizationStatus {
            time: Time::from_nanos(10),
            epoch: 1,
            generation: 2,
            state: LocalizationState::Tracking,
            heading: None,
        };
        let states = |after: &LocalizationEstimate| {
            vec![
                State::Ready(vec![raw_sample("/camera_matrix", 1, 20, 20, &camera)]),
                State::Ready(vec![
                    raw_sample("/localization/estimate", 2, 10, 10, &before),
                    raw_sample("/localization/estimate", 3, 30, 30, after),
                ]),
                State::Ready(vec![raw_sample("/localization/status", 4, 10, 10, &status)]),
                State::Ready(vec![]),
                State::Pending,
            ]
        };
        let time = Time::from_nanos(20);
        let mut inputs = OverlayInputs::default();
        let mut overlay = ProjectedFieldLinesOverlay::default();
        inputs.update(&states(&after));
        overlay.prepare(time, &mut inputs).unwrap();
        let sample = overlay
            .sample
            .as_ref()
            .expect("optional residual and dimensions must not block geometry");
        assert_eq!(sample.projection.field_to_camera.inner.translation.x, -4.0);
        assert!(sample.associations.is_none());
        after.epoch = 3;
        inputs.update(&states(&after));
        overlay.prepare(time, &mut inputs).unwrap();
        assert!(overlay.sample.is_none(), "never interpolate across epochs");
        after.epoch = 1;
        after.robot_to_field = None;
        inputs.update(&states(&after));
        overlay.prepare(time, &mut inputs).unwrap();
        assert!(
            overlay.sample.is_none(),
            "invalid localization clears old projection"
        );
    }

    #[test]
    fn projection_rejects_invalid_depth_and_clips_crossing_segments() {
        let projection = FieldProjection {
            field_to_camera: Isometry3::from_rotation(linear_algebra::vector![
                0.0, -FRAC_PI_2, 0.0
            ]),
            intrinsics: Intrinsic::default(),
        };
        assert!(projection.project(point![f32::NAN, 0.0, 0.0]).is_none());
        assert!(
            projection
                .segment(point![-2.0, 1.0], point![-1.0, 1.0])
                .is_none()
        );
        let segment = projection
            .segment(point![-1.0, 1.0], point![1.0, 1.0])
            .unwrap();
        assert!(
            segment
                .iter()
                .all(|p| p.inner.iter().all(|v| v.is_finite()))
        );
        assert_eq!(
            projection
                .segment(point![1.0, 1.0], point![-1.0, 1.0])
                .unwrap(),
            [segment[1], segment[0]]
        );
    }
}
