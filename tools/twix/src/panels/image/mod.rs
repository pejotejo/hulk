use std::{
    env::temp_dir,
    fs::create_dir_all,
    path::PathBuf,
    sync::Arc,
    time::{Duration, SystemTime},
};

use chrono::{DateTime, Utc};
use color_eyre::{
    Result,
    eyre::{bail, eyre},
};
use eframe::egui::{
    ColorImage, ComboBox, Context, Response, TextureHandle, TextureId, TextureOptions, Ui, Widget,
};
use geometry::rectangle::Rectangle;
use image::{EncodableLayout, RgbImage};
use linear_algebra::{point, vector};
use log::{info, warn};
use ros2::sensor_msgs::image::Image;
use serde_json::{Value, json};

use types::{time_wrapper::TimeWrapper, ycbcr422_image::YCbCr422Image};

use crate::{
    backend::TwixBackend,
    panel::{Panel, PanelCreationContext},
    twix_painter::{Orientation, TwixPainter},
    value_buffer::BufferHandle,
    zoom_and_pan::ZoomAndPanTransform,
};

use self::overlay::Overlays;

pub mod overlay;
mod overlays;

enum ImageBuffer {
    Raw(BufferHandle<TimeWrapper<Image>>),
    YCbCr(BufferHandle<TimeWrapper<YCbCr422Image>>),
}

struct LoadedTexture {
    timestamp: SystemTime,
    dimensions: (u32, u32),
    handle: TextureHandle,
}

pub struct ImagePanel {
    backend: Arc<TwixBackend>,
    image_buffer: ImageBuffer,
    overlays: Overlays,
    zoom_and_pan: ZoomAndPanTransform,
    last_image_path: String,
    current_image_path: String,
    current_image_label: String,
    texture: Option<LoadedTexture>,
}

fn subscribe_image(backend: &Arc<TwixBackend>, image_path: &str) -> ImageBuffer {
    if image_path.ends_with("ycbcr422_image") {
        ImageBuffer::YCbCr(backend.subscribe_value(image_path.to_string(), Duration::ZERO))
    } else {
        ImageBuffer::Raw(backend.subscribe_value(image_path.to_string(), Duration::ZERO))
    }
}

impl<'a> Panel<'a> for ImagePanel {
    const NAME: &'static str = "Image";

    fn new(context: PanelCreationContext) -> Self {
        let default_image_path = context
            .value
            .and_then(|value| value.get("topic").or_else(|| value.get("path")))
            .and_then(Value::as_str)
            .unwrap_or("inputs/left_image")
            .to_string();
        let default_image_label = image_label(&default_image_path).to_string();

        let image_buffer = subscribe_image(&context.backend, &default_image_path);

        let overlays = Overlays::new(
            context.backend.clone(),
            context.value.and_then(|value| value.get("overlays")),
        );
        Self {
            backend: context.backend,
            image_buffer,
            overlays,
            zoom_and_pan: ZoomAndPanTransform::default(),
            current_image_path: default_image_path.clone(),
            last_image_path: default_image_path,
            current_image_label: default_image_label,
            texture: None,
        }
    }

    fn save(&self) -> Value {
        let overlays = self.overlays.save();

        json!({
            "topic": self.current_image_path.clone(),
            "overlays": overlays,
        })
    }
}

fn image_label(topic: &str) -> &'static str {
    match topic {
        "inputs/left_image" => "Left Image",
        "inputs/right_image" => "Right Image",
        "inputs/ycbcr422_image" => "YCbCr422 Image",
        _ => "Image",
    }
}

fn save_raw_image(buffer: &BufferHandle<TimeWrapper<Image>>, path: PathBuf) -> Result<()> {
    let buffer = buffer
        .get_last_value()?
        .ok_or_else(|| eyre!("no image available"))?;
    buffer.inner.save_to_file(&path)?;
    info!("image saved to '{}'", path.display());
    Ok(())
}

fn save_ycbcr422_image(
    buffer: &BufferHandle<TimeWrapper<YCbCr422Image>>,
    path: PathBuf,
) -> Result<()> {
    let buffer = buffer
        .get_last_value()?
        .ok_or_else(|| eyre!("no image available"))?;
    buffer.inner.save_to_ycbcr_444_file(&path)?;
    info!("image saved to '{}'", path.display());
    Ok(())
}

impl Widget for &mut ImagePanel {
    fn ui(self, ui: &mut Ui) -> Response {
        ui.horizontal(|ui| {
            if self.current_image_path == "inputs/right_image" {
                ui.label("Overlays unavailable for right image");
            } else {
                self.overlays.combo_box(ui);
            }

            ComboBox::from_label("Image Topic")
                .selected_text(self.current_image_label.clone())
                .show_ui(ui, |ui| {
                    let mut selectable_item = |value: &str, label: &str| {
                        let is_selected = self.current_image_path == value;

                        if ui.selectable_label(is_selected, label).clicked() {
                            self.current_image_path = value.to_string();
                            self.current_image_label = label.to_string();
                        }
                    };

                    selectable_item("inputs/left_image", "Left Image");
                    selectable_item("inputs/right_image", "Right Image");
                    selectable_item("inputs/ycbcr422_image", "YCbCr422 Image");
                });
            if self.last_image_path != self.current_image_path {
                self.resubscribe();
                self.last_image_path = self.current_image_path.clone();
            }

            if let Some(timestamp) = self.current_image_timestamp() {
                let date: DateTime<Utc> = timestamp.into();
                ui.label(date.format("%T%.3f").to_string());
            }
            if ui.button("Save").clicked() {
                let time_stamp = Utc::now().format("%H:%M:%S%.3f").to_string();
                let directory = temp_dir().join("twix");
                if let Err(error) = create_dir_all(&directory) {
                    warn!("failed to create temporary folder /tmp/twix: {error}");
                } else {
                    let path = directory.join(format!("image_vision_{time_stamp}.png"));
                    let result = match &self.image_buffer {
                        ImageBuffer::Raw(buffer) => save_raw_image(buffer, path),
                        ImageBuffer::YCbCr(buffer) => save_ycbcr422_image(buffer, path),
                    };
                    if let Err(error) = result {
                        warn!("failed to save image: {error}");
                    }
                }
            }
        });

        let (texture_id, (width, height)) = match self.load_latest_texture(ui.ctx()) {
            Ok(result) => result,
            Err(error) => {
                return ui.scope(|ui| ui.label(format!("{error}"))).response;
            }
        };

        let (response, mut painter) = TwixPainter::allocate(
            ui,
            vector![width as f32, height as f32],
            point![0.0, 0.0],
            Orientation::LeftHanded,
        );
        self.zoom_and_pan.apply(ui, &mut painter, &response);
        painter.image(
            texture_id,
            Rectangle {
                min: point!(0.0, 0.0),
                max: point!(width as f32, height as f32),
            },
        );

        if self.current_image_path != "inputs/right_image" {
            self.overlays.paint(&painter);
        }

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
    fn resubscribe(&mut self) {
        self.image_buffer = subscribe_image(&self.backend, &self.current_image_path);
        self.texture = None;
    }

    fn current_image_timestamp(&self) -> Option<std::time::SystemTime> {
        match &self.image_buffer {
            ImageBuffer::Raw(buffer) => buffer.get_last_timestamp(),
            ImageBuffer::YCbCr(buffer) => buffer.get_last_timestamp(),
        }
        .ok()
        .flatten()
    }

    fn load_latest_texture(&mut self, context: &Context) -> Result<(TextureId, (u32, u32))> {
        let latest_timestamp = match &self.image_buffer {
            ImageBuffer::Raw(buffer) => buffer.get_last_timestamp()?,
            ImageBuffer::YCbCr(buffer) => buffer.get_last_timestamp()?,
        }
        .ok_or_else(|| eyre!("no image available"))?;

        if let Some(texture) = &self.texture
            && texture.timestamp == latest_timestamp
        {
            return Ok((texture.handle.id(), texture.dimensions));
        }

        let (image, timestamp, dimensions) = match &self.image_buffer {
            ImageBuffer::Raw(buffer) => {
                let ros_image = buffer
                    .get_last()?
                    .ok_or_else(|| eyre!("no image available"))?;
                let timestamp = ros_image.timestamp;
                let ros_image = ros_image.value;
                let ros_image = ros_image.inner;
                if ros_image.height == 0 || ros_image.width == 0 {
                    bail!(
                        "Image has no pixels. Dimensions: {}x{}",
                        ros_image.width,
                        ros_image.height
                    );
                }

                let rgb_image: RgbImage = ros_image
                    .try_into()
                    .map_err(|e: image::ImageError| eyre!(e))?;

                let image = ColorImage::from_rgb(
                    [rgb_image.width() as usize, rgb_image.height() as usize],
                    rgb_image.as_bytes(),
                );

                (image, timestamp, (rgb_image.width(), rgb_image.height()))
            }
            ImageBuffer::YCbCr(buffer) => {
                let image = buffer
                    .get_last()?
                    .ok_or_else(|| eyre!("no image available"))?;
                let timestamp = image.timestamp;
                let image = image.value.inner;
                if image.height() == 0 || image.width() == 0 {
                    bail!(
                        "Image has no pixels. Dimensions: {}x{}",
                        image.width(),
                        image.height()
                    );
                }

                let rgb_image: RgbImage = image.into();

                let image = ColorImage::from_rgb(
                    [rgb_image.width() as usize, rgb_image.height() as usize],
                    rgb_image.as_bytes(),
                );

                (image, timestamp, (rgb_image.width(), rgb_image.height()))
            }
        };

        Ok((
            self.update_texture(context, image, timestamp, dimensions),
            dimensions,
        ))
    }

    fn update_texture(
        &mut self,
        context: &Context,
        image: ColorImage,
        timestamp: SystemTime,
        dimensions: (u32, u32),
    ) -> TextureId {
        let texture = match self.texture.as_mut() {
            Some(texture) => {
                texture.handle.set(image, TextureOptions::NEAREST);
                texture.timestamp = timestamp;
                texture.dimensions = dimensions;
                texture
            }
            None => {
                self.texture = Some(LoadedTexture {
                    timestamp,
                    dimensions,
                    handle: context.load_texture(
                        "bytes://image-vision",
                        image,
                        TextureOptions::NEAREST,
                    ),
                });
                self.texture.as_mut().expect("texture was just created")
            }
        };

        texture.handle.id()
    }
}
