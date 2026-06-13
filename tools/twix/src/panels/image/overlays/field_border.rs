use std::{sync::Arc, time::Duration};

use color_eyre::Result;
use coordinate_systems::Pixel;
use eframe::epaint::{Color32, Stroke};
use linear_algebra::Point2;
use types::{field_border::FieldBorder as DetectedFieldBorder, time_wrapper::TimeWrapper};

use crate::{
    backend::TwixBackend, panels::image::overlay::Overlay, twix_painter::TwixPainter,
    value_buffer::BufferHandle,
};

pub struct FieldBorder {
    border: BufferHandle<TimeWrapper<Option<DetectedFieldBorder>>>,
    candidates: BufferHandle<Vec<Point2<Pixel>>>,
}

impl Overlay for FieldBorder {
    const NAME: &'static str = "Field Border";

    fn new(backend: Arc<TwixBackend>) -> Self {
        Self {
            border: backend.subscribe_value("field_border", Duration::ZERO),
            candidates: backend.subscribe_value("field_border_points", Duration::ZERO),
        }
    }

    fn paint(&self, painter: &TwixPainter<Pixel>) -> Result<()> {
        for point in self.candidates.get_last_value()?.unwrap_or_default() {
            painter.circle_filled(point, 2.0, Color32::BLUE);
        }

        let Some(border) = self
            .border
            .get_last_value()?
            .and_then(|wrapper| wrapper.inner)
        else {
            return Ok(());
        };
        for line in border.border_lines {
            painter.line_segment(
                line.0,
                line.1,
                Stroke::new(3.0_f32, Color32::from_rgb(255, 0, 240)),
            );
        }

        Ok(())
    }
}
