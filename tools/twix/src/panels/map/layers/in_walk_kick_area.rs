use std::sync::Arc;

use color_eyre::{eyre::Ok, Result};
use eframe::{
    egui::accesskit::Point,
    epaint::{Color32, Stroke},
};

use coordinate_systems::Ground;
use linear_algebra::{point, Point2};
use types::{
    ball_position::{self, BallPosition},
    field_dimensions::FieldDimensions,
    parameters::InWalkKickInfoParameters,
    support_foot::{Side, SupportFoot},
    world_state::{self, WorldState},
};
use walking_engine::{mode::kicking::Kicking, Engine};

use crate::{
    nao::Nao,
    panels::map::{layer::Layer, layers::kick_decisions},
    twix_painter::TwixPainter,
    value_buffer::BufferHandle,
};

pub struct KickThreshold {
    pub kick_threshold: BufferHandle<InWalkKickInfoParameters>,
    pub walking_engine: BufferHandle<Option<Engine>>,
    pub ball_position: BufferHandle<Option<BallPosition<Ground>>>,
}

struct Corners {
    min: Point2<Ground>,
    max: Point2<Ground>,
}
impl Corners {
    fn contains(&self, position: Point2<Ground>) -> bool {
        position.x() >= self.min.x()
            && position.x() <= self.max.x()
            && position.y() >= self.min.y()
            && position.y() <= self.max.y()
    }
}
impl Layer<Ground> for KickThreshold {
    const NAME: &'static str = "In Walk Kick Area";

    fn new(nao: Arc<Nao>) -> Self {
        let kick_threshold = nao.subscribe_value("parameters.in_walk_kicks.forward");
        let walking_engine = nao.subscribe_value("Control.additional_outputs.walking.engine");
        let ball_position = nao.subscribe_value("Control.main_outputs.ball_position");
        Self {
            kick_threshold,
            walking_engine,
            ball_position,
        }
    }

    fn paint(
        &self,
        painter: &TwixPainter<Ground>,
        _field_dimensions: &FieldDimensions,
    ) -> Result<()> {
        let Some(walking_engine) = self.walking_engine.get_last_value()?.flatten() else {
            return Ok(());
        };
        let Some(ball_position) = self.ball_position.get_last_value()?.flatten() else {
            return Ok(());
        };
        if let Some(kick_threshold) = self.kick_threshold.get_last_value()? {
            let corners_left = Corners {
                min: point!(
                    kick_threshold.reached_x.start - kick_threshold.position.x,
                    kick_threshold.reached_y.start - kick_threshold.position.y
                ),
                max: point!(
                    kick_threshold.reached_x.end - kick_threshold.position.x,
                    kick_threshold.reached_y.end - kick_threshold.position.y
                ),
            };
            let corners_right = Corners {
                min: point!(
                    kick_threshold.reached_x.start - kick_threshold.position.x,
                    kick_threshold.reached_y.start + kick_threshold.position.y
                ),
                max: point!(
                    kick_threshold.reached_x.end - kick_threshold.position.x,
                    kick_threshold.reached_y.end + kick_threshold.position.y
                ),
            };
            painter.rect_stroke(
                corners_left.min,
                corners_left.max,
                Stroke::new(0.005, Color32::MAGENTA),
            );
            painter.rect_stroke(
                corners_right.min,
                corners_right.max,
                Stroke::new(0.005, Color32::MAGENTA),
            );

            let side = match walking_engine.mode {
                walking_engine::mode::Mode::Kicking(kicking) => kicking.kick.side,
                walking_engine::mode::Mode::Walking(walking) => {
                    walking.step.plan.support_side.opposite()
                }
                _ => return Ok(()),
            };
            match side {
                Side::Left => painter.rect_stroke(
                    corners_left.min,
                    corners_left.max,
                    Stroke::new(0.01, Color32::BLUE),
                ),
                Side::Right => painter.rect_stroke(
                    corners_right.min,
                    corners_right.max,
                    Stroke::new(0.01, Color32::BLUE),
                ),
            };

            if corners_left.contains(ball_position.position) {
                painter.rect_stroke(
                    corners_left.min,
                    corners_left.max,
                    Stroke::new(0.01, Color32::GREEN),
                );
            } else if corners_right.contains(ball_position.position) {
                painter.rect_stroke(
                    corners_right.min,
                    corners_right.max,
                    Stroke::new(0.01, Color32::GREEN),
                );
            }
        }
        Ok(())
    }
}
