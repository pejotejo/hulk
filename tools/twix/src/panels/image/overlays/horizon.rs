use std::{sync::Arc, time::Duration};

use color_eyre::Result;
use coordinate_systems::Pixel;
use eframe::epaint::{Color32, Stroke};
use linear_algebra::point;
use projection::camera_matrix::CameraMatrix;
use types::time_wrapper::TimeWrapper;

use crate::{
    backend::TwixBackend, panels::image::overlay::Overlay, twix_painter::TwixPainter,
    value_buffer::BufferHandle,
};

pub struct Horizon {
    camera_matrix: BufferHandle<TimeWrapper<CameraMatrix>>,
}

impl Overlay for Horizon {
    const NAME: &'static str = "Horizon";

    fn new(backend: Arc<TwixBackend>) -> Self {
        Self {
            camera_matrix: backend.subscribe_value("camera_matrix", Duration::ZERO),
        }
    }

    fn paint(&self, painter: &TwixPainter<Pixel>) -> Result<()> {
        let Some(horizon) = self
            .camera_matrix
            .get_last_value()?
            .and_then(|wrapper| wrapper.inner.horizon)
        else {
            return Ok(());
        };

        let left_horizon_height = horizon.y_at_x(0.0);
        let right_horizon_height = horizon.y_at_x(640.0);

        painter.line_segment(
            point![0.0, left_horizon_height],
            point![640.0, right_horizon_height],
            Stroke::new(3.0_f32, Color32::GREEN),
        );

        painter.circle_stroke(
            horizon.vanishing_point,
            5.0,
            Stroke::new(3.0_f32, Color32::GREEN),
        );

        Ok(())
    }
}
