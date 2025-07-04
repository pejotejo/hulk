use std::collections::BTreeMap;

use eframe::egui::{
    pos2, vec2, Color32, CornerRadius, Painter, PointerButton, Pos2, Rect, Response, Sense, Stroke,
    Ui, Vec2, Widget,
};

use framework::Timing;

use crate::{
    controls::Controls,
    coordinate_systems::{
        AbsoluteScreen, AbsoluteTime, FrameRange, PanAndZoom, RelativeTime, ScreenRange,
        ViewportRange,
    },
    user_data::BookmarkCollection,
};

pub struct Frames<'state> {
    controls: &'state Controls,
    indices: &'state BTreeMap<String, Vec<Timing>>,
    frame_range: &'state FrameRange,
    viewport_range: &'state mut ViewportRange,
    position: &'state mut RelativeTime,
    item_spacing: Vec2,
    bookmarks: &'state mut BookmarkCollection,
}

impl<'state> Frames<'state> {
    pub fn new(
        controls: &'state Controls,
        indices: &'state BTreeMap<String, Vec<Timing>>,
        frame_range: &'state FrameRange,
        viewport_range: &'state mut ViewportRange,
        position: &'state mut RelativeTime,
        item_spacing: Vec2,
        bookmarks: &'state mut BookmarkCollection,
    ) -> Self {
        Self {
            controls,
            indices,
            frame_range,
            viewport_range,
            position,
            item_spacing,
            bookmarks,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn interact(
        &mut self,
        double_clicked: bool,
        cursor_position: Option<Pos2>,
        cursor_down: bool,
        scroll_delta: Vec2,
        shift_down: bool,
        keys: Keys,
        screen_range: &ScreenRange,
    ) -> bool {
        let original_position = *self.position;

        if double_clicked {
            *self.viewport_range = ViewportRange::from_frame_range(self.frame_range);
            return false;
        }

        let cursor_position =
            AbsoluteScreen::new(cursor_position.map_or(0.0, |position| position.x))
                .map_to_relative_screen(screen_range);

        let cursor_position = cursor_position.map_to_relative_time(self.viewport_range);
        let position_changed = cursor_down && cursor_position != *self.position;
        if position_changed {
            *self.position = cursor_position;
        }

        let zoom_factor = 0.99_f32.powf(scroll_delta.y);
        let pan_offset =
            AbsoluteScreen::new(scroll_delta.x + if shift_down { scroll_delta.y } else { 0.0 })
                .scale_to_relative_screen(screen_range)
                .scale_to_relative_time(self.viewport_range);

        let transform = PanAndZoom::from_shift(cursor_position)
            * PanAndZoom::new(zoom_factor, pan_offset)
            * PanAndZoom::from_shift(-cursor_position);
        *self.viewport_range = transform * self.viewport_range.clone();

        if keys.jump_backward_large {
            *self.position -= RelativeTime::new(10.0);
        }
        if keys.jump_forward_large {
            *self.position += RelativeTime::new(10.0);
        }
        if keys.jump_backward_small {
            *self.position -= RelativeTime::new(1.0);
        }
        if keys.jump_forward_small {
            *self.position += RelativeTime::new(1.0);
        }
        if keys.step_backward {
            *self.position -= RelativeTime::new(0.01);
        }
        if keys.step_forward {
            *self.position += RelativeTime::new(0.01);
        }
        if keys.jump_to_next_bookmark {
            if let Some((next_bookmark_time, _)) = self
                .bookmarks
                .next_after(&self.position.map_to_absolute_time(self.frame_range))
            {
                *self.position = next_bookmark_time.map_to_relative_time(self.frame_range);
            }
        };
        if keys.jump_to_previous_bookmark {
            if let Some((previous_bookmark_time, _)) = self
                .bookmarks
                .previous_before(&self.position.map_to_absolute_time(self.frame_range))
            {
                *self.position = previous_bookmark_time.map_to_relative_time(self.frame_range);
            }
        };

        original_position != *self.position
    }

    fn show_cyclers(&self, painter: &Painter, color: Color32, screen_range: &ScreenRange) {
        let spacing = self.item_spacing.y;
        let total_spacing = spacing * (self.indices.len() - 1) as f32;
        let row_height = (painter.clip_rect().height() - total_spacing) / self.indices.len() as f32;

        for (index, recording_index) in self.indices.values().enumerate() {
            let top_left =
                painter.clip_rect().left_top() + vec2(0.0, (row_height + spacing) * index as f32);
            let mut painter = painter.clone();
            painter.set_clip_rect(Rect::from_min_max(
                top_left,
                pos2(painter.clip_rect().right(), top_left.y + row_height),
            ));
            self.show_cycler(recording_index, painter, color, screen_range);
        }
    }

    fn show_cycler(
        &self,
        index: &[Timing],
        painter: Painter,
        color: Color32,
        screen_range: &ScreenRange,
    ) {
        for frame in index {
            self.show_frame(frame, &painter, color, screen_range);
        }
    }

    fn show_frame(
        &self,
        frame: &Timing,
        painter: &Painter,
        color: Color32,
        screen_range: &ScreenRange,
    ) {
        let left = AbsoluteTime::new(frame.timestamp)
            .map_to_relative_time(self.frame_range)
            .map_to_relative_screen(self.viewport_range)
            .map_to_absolute_screen(screen_range);
        let right = AbsoluteTime::new(frame.timestamp + frame.duration)
            .map_to_relative_time(self.frame_range)
            .map_to_relative_screen(self.viewport_range)
            .map_to_absolute_screen(screen_range);

        let mut rect = painter.clip_rect();
        rect.set_left(left.inner());
        rect.set_right(right.inner());

        painter.rect_filled(rect, CornerRadius::ZERO, color);
    }

    fn show_position(&self, painter: &Painter, color: Color32, screen_range: &ScreenRange) {
        let clip_rect = painter.clip_rect();
        let x = self
            .position
            .map_to_relative_screen(self.viewport_range)
            .map_to_absolute_screen(screen_range);

        painter.line_segment(
            [
                pos2(x.inner(), clip_rect.top()),
                pos2(x.inner(), clip_rect.bottom()),
            ],
            Stroke::new(2.0, color),
        );
    }
}

impl Widget for Frames<'_> {
    fn ui(mut self, ui: &mut Ui) -> Response {
        let (mut response, painter) =
            ui.allocate_painter(ui.available_size(), Sense::click_and_drag());

        let screen_range = ScreenRange::new(
            AbsoluteScreen::new(painter.clip_rect().left()),
            AbsoluteScreen::new(painter.clip_rect().right()),
        );

        let (double_clicked, cursor_position, cursor_down, scroll_delta, shift_down, keys) = ui
            .input_mut(|input| {
                (
                    input.pointer.button_double_clicked(PointerButton::Primary),
                    input.pointer.interact_pos(),
                    input.pointer.button_down(PointerButton::Primary),
                    input.smooth_scroll_delta,
                    input.modifiers.shift,
                    Keys {
                        jump_backward_large: input
                            .consume_shortcut(&self.controls.jump_large.backward),
                        jump_forward_large: input
                            .consume_shortcut(&self.controls.jump_large.forward),
                        jump_backward_small: input
                            .consume_shortcut(&self.controls.jump_small.backward),
                        jump_forward_small: input
                            .consume_shortcut(&self.controls.jump_small.forward),
                        step_backward: input.consume_shortcut(&self.controls.step.backward),
                        step_forward: input.consume_shortcut(&self.controls.step.forward),
                        jump_to_previous_bookmark: input
                            .consume_shortcut(&self.controls.bookmark.backward),
                        jump_to_next_bookmark: input
                            .consume_shortcut(&self.controls.bookmark.forward),
                    },
                )
            });

        if self.interact(
            double_clicked,
            cursor_position,
            cursor_down && response.hovered(),
            scroll_delta,
            shift_down,
            keys,
            &screen_range,
        ) {
            response.mark_changed();
        }

        self.show_cyclers(&painter, ui.visuals().strong_text_color(), &screen_range);
        self.show_position(&painter, Color32::GREEN, &screen_range);

        response
    }
}

struct Keys {
    jump_backward_large: bool,
    jump_forward_large: bool,
    jump_backward_small: bool,
    jump_forward_small: bool,
    step_backward: bool,
    step_forward: bool,
    jump_to_next_bookmark: bool,
    jump_to_previous_bookmark: bool,
}
