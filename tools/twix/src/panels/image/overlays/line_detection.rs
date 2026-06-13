use std::{sync::Arc, time::Duration};

use color_eyre::Result;
use eframe::epaint::{Color32, Stroke};

use coordinate_systems::Pixel;
use geometry::line_segment::LineSegment;
use types::{
    image_segments::{EdgeType, GenericSegment},
    line_data::{DiscardedLine, LineDiscardReason},
};

use crate::{
    backend::TwixBackend, panels::image::overlay::Overlay, twix_painter::TwixPainter,
    value_buffer::BufferHandle,
};

fn edge_type_to_color(edge_type: EdgeType) -> Color32 {
    match edge_type {
        EdgeType::Rising => Color32::RED,
        EdgeType::Falling => Color32::BLUE,
        EdgeType::ImageBorder => Color32::GOLD,
        EdgeType::LimbBorder => Color32::BLACK,
    }
}

pub struct LineDetection {
    lines_in_image: BufferHandle<Vec<LineSegment<Pixel>>>,
    discarded_lines: BufferHandle<Vec<DiscardedLine>>,
    filtered_segments: BufferHandle<Vec<GenericSegment>>,
}

impl Overlay for LineDetection {
    const NAME: &'static str = "Line Detection";

    fn new(backend: Arc<TwixBackend>) -> Self {
        Self {
            lines_in_image: backend
                .subscribe_value("line_detection/lines_in_image", Duration::ZERO),
            discarded_lines: backend
                .subscribe_value("line_detection/discarded_lines", Duration::ZERO),
            filtered_segments: backend
                .subscribe_value("line_detection/filtered_segments", Duration::ZERO),
        }
    }

    fn paint(&self, painter: &TwixPainter<Pixel>) -> Result<()> {
        let Some(lines_in_image) = self.lines_in_image.get_last_value()? else {
            return Ok(());
        };
        let Some(discarded_lines) = self.discarded_lines.get_last_value()? else {
            return Ok(());
        };
        let Some(filtered_segments) = self.filtered_segments.get_last_value()? else {
            return Ok(());
        };
        for segment in filtered_segments {
            painter.line_segment(
                segment.start.cast(),
                segment.end.cast(),
                Stroke::new(1.0_f32, Color32::BLACK),
            );
            painter.circle_stroke(
                segment.center().cast(),
                2.0,
                Stroke::new(1.0_f32, Color32::RED),
            );
            painter.circle_filled(
                segment.start.cast(),
                1.0,
                edge_type_to_color(segment.start_edge_type),
            );
            painter.circle_filled(
                segment.end.cast(),
                1.0,
                edge_type_to_color(segment.end_edge_type),
            );
        }
        for discarded_line in discarded_lines {
            let color = match discarded_line.discard_reason {
                LineDiscardReason::TooFewPoints => Color32::YELLOW,
                LineDiscardReason::LineTooShort => Color32::GRAY,
                LineDiscardReason::LineTooLong => Color32::BROWN,
                LineDiscardReason::TooFarAway => Color32::BLACK,
            };
            painter.line_segment(
                discarded_line.line.0,
                discarded_line.line.1,
                Stroke::new(3.0_f32, color),
            );
        }
        for line in lines_in_image {
            painter.line_segment(line.0, line.1, Stroke::new(3.0_f32, Color32::ORANGE));
        }
        Ok(())
    }
}
