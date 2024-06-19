use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};

use crate::color::{Intensity, Rgb, YCbCr444};

#[derive(
    Copy,
    Clone,
    Debug,
    Default,
    Deserialize,
    Serialize,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
)]
pub struct FieldColor {
    pub red_chromaticity_threshold: f32,
    pub blue_chromaticity_threshold: f32,
    pub green_chromaticity_threshold: f32,
    pub green_luminance_threshold: f32,
    pub luminance_threshold: f32,
}

impl FieldColor {
    pub fn get_intensity(&self, color: YCbCr444) -> Intensity {
        let rgb = Rgb::from(color);
        let chromaticity = rgb.convert_to_rgchromaticity();
        if (chromaticity.red > self.red_chromaticity_threshold
            || chromaticity.blue > self.blue_chromaticity_threshold
            || chromaticity.green < self.green_chromaticity_threshold
            || (rgb.green as f32) < self.green_luminance_threshold)
            && (rgb.get_luminance() as f32) > self.luminance_threshold
        {
            Intensity::Low
        } else {
            Intensity::High
        }
    }
}
