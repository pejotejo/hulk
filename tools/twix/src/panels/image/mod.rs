use std::{env::temp_dir, fs::create_dir_all, path::PathBuf, sync::Arc};

use chrono::{DateTime, Utc};
use color_eyre::{eyre::eyre, Result};
use coordinate_systems::Pixel;
use eframe::egui::{ColorImage, Response, SizeHint, TextureOptions, Ui, UiBuilder, Widget};
use geometry::rectangle::Rectangle;
use image::RgbImage;
use linear_algebra::{point, vector};
use log::{info, warn};
use serde_json::{json, Value};

use types::{jpeg::JpegImage, ycbcr422_image::YCbCr422Image};

use crate::{
    nao::Nao,
    panel::Panel,
    twix_painter::{Orientation, TwixPainter},
    value_buffer::BufferHandle,
    zoom_and_pan::ZoomAndPanTransform,
};

use self::{
    cycler_selector::{VisionCycler, VisionCyclerSelector},
    overlay::Overlays,
};

pub mod cycler_selector;
pub mod overlay;
mod overlays;

enum RawOrJpeg {
    Raw(BufferHandle<YCbCr422Image>),
    Jpeg(BufferHandle<JpegImage>),
}

pub struct ImagePanel {
    nao: Arc<Nao>,
    image_buffer: RawOrJpeg,
    cycler: VisionCycler,
    overlays: Overlays,
    zoom_and_pan: ZoomAndPanTransform,
}

impl Panel for ImagePanel {
    const NAME: &'static str = "Image";

    fn new(nao: Arc<Nao>, value: Option<&Value>) -> Self {
        let cycler = value
            .and_then(|value| {
                let string = value.get("cycler")?.as_str()?;
                VisionCycler::try_from(string).ok()
            })
            .unwrap_or(VisionCycler::Top);
        let cycler_path = cycler.as_path();

        let is_jpeg = value
            .and_then(|value| value.get("is_jpeg"))
            .and_then(|value| value.as_bool())
            .unwrap_or(true);

        let image_buffer = if is_jpeg {
            let path = format!("{cycler_path}.main_outputs.image.jpeg");
            RawOrJpeg::Jpeg(nao.subscribe_value(path))
        } else {
            let path = format!("{cycler_path}.main_outputs.image");
            RawOrJpeg::Raw(nao.subscribe_value(path))
        };

        let overlays = Overlays::new(
            nao.clone(),
            value.and_then(|value| value.get("overlays")),
            cycler,
        );
        Self {
            nao,
            image_buffer,
            cycler,
            overlays,
            zoom_and_pan: ZoomAndPanTransform::default(),
        }
    }

    fn save(&self) -> Value {
        let overlays = self.overlays.save();

        json!({
            "is_jpeg": matches!(self.image_buffer, RawOrJpeg::Jpeg(_)),
            "cycler": self.cycler.as_path(),
            "overlays": overlays,
        })
    }
}

fn save_jpeg_image(buffer: &BufferHandle<JpegImage>, path: PathBuf) -> Result<()> {
    let buffer = buffer
        .get_last_value()?
        .ok_or_else(|| eyre!("no image available"))?;
    buffer.save_to_jpeg_file(&path)?;
    info!("image saved to '{}'", path.display());
    Ok(())
}

fn save_raw_image(buffer: &BufferHandle<YCbCr422Image>, path: PathBuf) -> Result<()> {
    let buffer = buffer
        .get_last_value()?
        .ok_or_else(|| eyre!("no image available"))?;
    buffer.save_to_ycbcr_444_file(&path)?;
    info!("image saved to '{}'", path.display());
    Ok(())
}

impl Widget for &mut ImagePanel {
    fn ui(self, ui: &mut Ui) -> Response {
        ui.horizontal(|ui| {
            let mut jpeg = matches!(self.image_buffer, RawOrJpeg::Jpeg(_));
            let mut cycler_selector = VisionCyclerSelector::new(&mut self.cycler);
            if cycler_selector.ui(ui).changed() {
                self.resubscribe(jpeg);
                self.overlays.update_cycler(self.cycler);
            }
            self.overlays.combo_box(ui, self.cycler);
            if ui.checkbox(&mut jpeg, "JPEG").changed() {
                self.resubscribe(jpeg);
            }
            let maybe_timestamp = match &self.image_buffer {
                RawOrJpeg::Raw(buffer) => buffer.get_last_timestamp(),
                RawOrJpeg::Jpeg(buffer) => buffer.get_last_timestamp(),
            };
            if let Ok(Some(timestamp)) = maybe_timestamp {
                let date: DateTime<Utc> = timestamp.into();
                ui.label(date.format("%T%.3f").to_string());
            }
            if ui.button("Save").clicked() {
                let time_stamp = Utc::now().format("%H:%M:%S%.3f").to_string();
                let directory = temp_dir().join("twix");
                if let Err(error) = create_dir_all(&directory) {
                    warn!("failed to create temporary folder /tmp/twix: {error}");
                } else {
                    let cycler_name = format!("{:?}", self.cycler);
                    let path = directory.join(format!("image_{cycler_name}_{time_stamp}.png"));
                    let result = match &self.image_buffer {
                        RawOrJpeg::Raw(buffer) => save_raw_image(buffer, path),
                        RawOrJpeg::Jpeg(buffer) => {
                            save_jpeg_image(buffer, path.with_extension("jpeg"))
                        }
                    };
                    if let Err(error) = result {
                        warn!("failed to save image: {error}");
                    }
                }
            }
        });
        let (response, mut painter) = TwixPainter::allocate(
            ui,
            vector![640.0, 480.0],
            point![0.0, 0.0],
            Orientation::LeftHanded,
        );
        self.zoom_and_pan.apply(ui, &mut painter, &response);

        if let Err(error) = self.show_image(&painter) {
            ui.scope_builder(UiBuilder::new().max_rect(response.rect), |ui| {
                ui.label(format!("{error}"))
            });
        };

        self.overlays.paint(&painter);

        match response.hover_pos() {
            Some(position) => {
                let pixel_position = painter.transform_pixel_to_world(position);
                response.on_hover_text_at_pointer(format!(
                    "x: {:.1}, y: {:.1}",
                    pixel_position.x(),
                    pixel_position.y()
                ))
            }
            _ => response,
        }
    }
}

impl ImagePanel {
    fn resubscribe(&mut self, jpeg: bool) {
        let cycler_path = self.cycler.as_path();
        self.image_buffer = if jpeg {
            RawOrJpeg::Jpeg(
                self.nao
                    .subscribe_value(format!("{cycler_path}.main_outputs.image.jpeg")),
            )
        } else {
            RawOrJpeg::Raw(
                self.nao
                    .subscribe_value(format!("{cycler_path}.main_outputs.image")),
            )
        };
    }

    fn show_image(&self, painter: &TwixPainter<Pixel>) -> Result<()> {
        let context = painter.context();

        let image_identifier = format!("bytes://image-{:?}", self.cycler);
        let image = match &self.image_buffer {
            RawOrJpeg::Raw(buffer) => {
                let ycbcr = buffer
                    .get_last_value()?
                    .ok_or_else(|| eyre!("no image available"))?;
                let image = ColorImage::from_rgb(
                    [ycbcr.width() as usize, ycbcr.height() as usize],
                    RgbImage::from(ycbcr).as_raw(),
                );
                context
                    .load_texture(&image_identifier, image, TextureOptions::NEAREST)
                    .id()
            }
            RawOrJpeg::Jpeg(buffer) => {
                let jpeg = buffer
                    .get_last_value()?
                    .ok_or_else(|| eyre!("no image available"))?;
                context.forget_image(&image_identifier);
                context.include_bytes(image_identifier.clone(), jpeg.data);
                context
                    .try_load_texture(
                        &image_identifier,
                        TextureOptions::NEAREST,
                        SizeHint::Size(640, 480),
                    )?
                    .texture_id()
                    .unwrap()
            }
        };

        painter.image(
            image,
            Rectangle {
                min: point!(0.0, 0.0),
                max: point!(640.0, 480.0),
            },
        );
        Ok(())
    }
}
