use std::{sync::Arc, time::Duration};

use color_eyre::Result;
use coordinate_systems::Pixel;
use eframe::epaint::{Color32, Stroke};
use geometry::circle::Circle;

use crate::{
    backend::TwixBackend, panels::image::overlay::Overlay, twix_painter::TwixPainter,
    value_buffer::BufferHandle,
};

pub struct BallDetection {
    filtered_balls: BufferHandle<Vec<Circle<Pixel>>>,
}

impl Overlay for BallDetection {
    const NAME: &'static str = "Ball Detection";

    fn new(backend: Arc<TwixBackend>) -> Self {
        Self {
            filtered_balls: backend
                .subscribe_value("ball_filter/filtered_balls_in_image", Duration::ZERO),
        }
    }

    fn paint(&self, painter: &TwixPainter<Pixel>) -> Result<()> {
        if let Some(filtered_balls) = self.filtered_balls.get_last_value()? {
            for circle in &filtered_balls {
                painter.circle_stroke(
                    circle.center,
                    circle.radius,
                    Stroke::new(3.0_f32, Color32::RED),
                );
            }
        }

        Ok(())
    }
}
