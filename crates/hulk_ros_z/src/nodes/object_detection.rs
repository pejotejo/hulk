use std::{env, sync::Arc};

use color_eyre::{
    Result,
    eyre::{Context, bail},
};
use geometry::rectangle::Rectangle;
use image::RgbImage;
use linear_algebra::point;
use ndarray::{ArrayView3, ArrayView4, Axis, s};
use ort::{
    execution_providers::TensorRTExecutionProvider,
    inputs,
    session::{Session, builder::GraphOptimizationLevel},
    value::TensorRef,
};
use ros_z::{Builder, context::ZContext};
use ros_z_config::prelude::*;
use ros2::sensor_msgs::image::Image;
use tokio::time::Instant;
use types::{
    bounding_box::BoundingBox,
    object_detection::{Detection, Detections, NaoLabelPartyObjectDetectionLabel},
};

use crate::{IntoEyreResultExt, config::ObjectDetectionConfig, msgs::ObjectDetectionStatus};

pub async fn run(ctx: Arc<ZContext>) -> Result<()> {
    let node = ctx
        .create_node("object_detection")
        .with_type_description_service()
        .with_extended_type_description_service()
        .build()
        .into_eyre()?;

    let config = node
        .bind_config_with_metadata_as::<ObjectDetectionConfig>("object_detection")
        .into_eyre()?;

    config
        .add_validation_hook(move |cfg: &ObjectDetectionConfig| {
            if cfg.inputs.maximum_intersection_over_union > 1.0
                || cfg.inputs.maximum_intersection_over_union < 0.0
            {
                return Err(
                    "object_detection.inputs.maximum_intersection_over_union must be in [0,1]"
                        .to_owned(),
                );
            }
            if cfg.inputs.confidence_threshold > 1.0 || cfg.inputs.confidence_threshold < 0.0 {
                return Err(
                    "object_detection.inputs.confidence_threshold must be in [0,1]".to_owned(),
                );
            }
            if !env::current_dir()
                .unwrap()
                .clone()
                .join(&cfg.inputs.rgb_neural_network_path)
                .exists()
            {
                return Err(format!(
                    "object_detection.inputs.neural_network_path: {} must exist inside pwd: {}",
                    cfg.inputs.rgb_neural_network_path.display(),
                    env::current_dir().unwrap().display()
                ));
            }
            if !env::current_dir()
                .unwrap()
                .clone()
                .join(&cfg.inputs.nv12_neural_network_path)
                .exists()
            {
                return Err(format!(
                    "object_detection.inputs.neural_network_path: {} must exist inside pwd: {}",
                    cfg.inputs.nv12_neural_network_path.display(),
                    env::current_dir().unwrap().display()
                ));
            }
            Ok(())
        })
        .into_eyre()?;

    let cfg = config.snapshot().typed().clone();

    let image_sub = node
        .create_sub::<Image>("robot_hw/left_image")
        .build()
        .into_eyre()?;

    let detection_pub = node
        .create_pub::<Detections<NaoLabelPartyObjectDetectionLabel>>("object_detection/detections")
        .build()
        .into_eyre()?;

    let status_pub = node
        .create_pub::<ObjectDetectionStatus>("object_detection/object_detection_status")
        .build()
        .into_eyre()?;

    let rgb_neural_network_path = env::current_dir()
        .unwrap()
        .join(cfg.inputs.rgb_neural_network_path);

    let nv12_neural_network_path = env::current_dir()
        .unwrap()
        .join(cfg.inputs.nv12_neural_network_path);

    let tensor_rt = TensorRTExecutionProvider::default()
        .with_device_id(0)
        .with_fp16(true)
        .with_engine_cache(true)
        .with_engine_cache_path(rgb_neural_network_path.parent().unwrap().display())
        .build();

    let mut rgb_session = Session::builder()?
        .with_execution_providers([tensor_rt.clone()])?
        .with_optimization_level(GraphOptimizationLevel::Level3)?
        .with_intra_threads(2)?
        .commit_from_file(rgb_neural_network_path)?;

    let mut nv12_session = Session::builder()?
        .with_execution_providers([tensor_rt])?
        .with_optimization_level(GraphOptimizationLevel::Level3)?
        .with_intra_threads(2)?
        .commit_from_file(nv12_neural_network_path)?;

    loop {
        let cfg = config.snapshot().typed().clone();

        tokio::select! {
            msg = image_sub.async_recv() => {
                let image = msg.into_eyre()?;
                let (detections, status) = if image.encoding == "nv12" {
                    do_inference(&cfg, &mut nv12_session, &image)?
                } else {
                    do_inference(&cfg, &mut rgb_session, &image)?
                };

                detection_pub.async_publish(&detections).await.into_eyre()?;
                status_pub.async_publish(&status).await.into_eyre()?;
            }
        }
    }
}

pub fn do_inference(
    cfg: &ObjectDetectionConfig,
    session: &mut Session,
    image: &Image,
) -> Result<(
    Detections<NaoLabelPartyObjectDetectionLabel>,
    ObjectDetectionStatus,
)> {
    if image.encoding != "rgb8" && image.encoding != "nv12" {
        bail!("unsupported image encoding: {}", image.encoding);
    }

    if image.width % 32 != 0 || image.height % 32 != 0 {
        bail!(
            "image dimensions must be multiples of 32 (got {}x{})",
            image.width,
            image.height
        );
    }
    let height = image.height as usize;
    let width = image.width as usize;

    let inference_start = Instant::now();
    let output = if image.encoding == "nv12" {
        let nv12_data = ArrayView3::from_shape(
            [image.height as usize / 2, image.width as usize / 2, 6],
            image.data.as_slice(),
        )
        .wrap_err("failed to view nv12 data")?;

        let outputs =
            session.run(inputs!["raw_bytes_input" => TensorRef::from_array_view(nv12_data)?])?;
        outputs["network_detections"]
            .try_extract_array::<f32>()?
            .t()
            .into_owned()
    } else {
        let rgb_image: RgbImage = image.clone().try_into()?;

        let rgb_u8_view = ArrayView4::from_shape([1, 3, height, width], rgb_image.as_raw())
            .wrap_err("failed to view rgb8 data as NCHW float tensor")?;

        let rgb_f32 = rgb_u8_view.mapv(|byte| byte as f32 / 255.0);

        let outputs = session.run(inputs!["images" => TensorRef::from_array_view(&rgb_f32)?])?;
        outputs["detection_output"]
            .try_extract_array::<f32>()?
            .t()
            .into_owned()
    };

    let output = output.slice(s![.., .., 0]);

    let last_inference_duration = inference_start.elapsed();
    let post_processing_start = Instant::now();

    let mut candidate_detections: Vec<Detection<NaoLabelPartyObjectDetectionLabel>> = output
        .axis_iter(Axis(1))
        .filter_map(|row| {
            let confidence = row[4usize];
            let class_id = row[5usize] as usize;
            if confidence < cfg.inputs.confidence_threshold {
                return None;
            }
            let label = NaoLabelPartyObjectDetectionLabel::from_index(class_id);
            Some(Detection {
                bounding_box: BoundingBox {
                    area: Rectangle {
                        min: point!(row[0usize], row[1usize]),
                        max: point!(row[2usize], row[3usize]),
                    },
                    confidence,
                },
                label,
            })
        })
        .collect();

    candidate_detections.sort_by(|detection1, detection2| {
        detection1
            .bounding_box
            .confidence
            .total_cmp(&detection2.bounding_box.confidence)
    });

    let last_post_processing_duration = post_processing_start.elapsed();
    let non_maxiumum_suppression_start = Instant::now();

    let detections = non_maximum_suppression(
        candidate_detections,
        cfg.inputs.maximum_intersection_over_union,
    );

    let last_non_maximum_suppression_duration = non_maxiumum_suppression_start.elapsed();

    Ok((
        Detections { detections },
        ObjectDetectionStatus {
            last_inference_duration,
            last_post_processing_duration,
            last_non_maximum_suppression_duration,
        },
    ))
}

fn non_maximum_suppression<T>(
    mut sorted_candidate_detections: Vec<Detection<T>>,
    maximum_intersection_over_union: f32,
) -> Vec<Detection<T>> {
    let mut detections = Vec::new();

    while let Some(detection) = sorted_candidate_detections.pop() {
        sorted_candidate_detections.retain(|detection_candidate| {
            detection
                .bounding_box
                .intersection_over_union(&detection_candidate.bounding_box)
                < maximum_intersection_over_union
        });

        detections.push(detection)
    }

    detections
}
