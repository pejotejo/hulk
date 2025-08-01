use coordinate_systems::{Camera, Ground, Pixel};
use linear_algebra::{point, vector, Isometry3, Point2, Vector2, Vector3};
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};

use crate::intrinsic::Intrinsic;

#[derive(
    Clone,
    Copy,
    Debug,
    Default,
    Deserialize,
    PartialEq,
    Serialize,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
)]
pub struct Horizon {
    pub vanishing_point: Point2<Pixel>,
    pub normal: Vector2<Pixel>,
}

impl Horizon {
    pub const ABOVE_IMAGE: Horizon = Self {
        vanishing_point: point![0.0, 0.0],
        normal: vector![0.0, -1.0],
    };

    pub fn is_above_with_margin(&self, point: Point2<Pixel>, margin: f32) -> bool {
        let horizon_y = self.y_at_x(point.x());
        horizon_y + margin < point.y()
    }

    pub fn horizon_y_maximum(&self) -> f32 {
        self.y_at_x(0.0).max(self.y_at_x(640.0))
    }

    pub fn horizon_y_minimum(&self) -> f32 {
        self.y_at_x(0.0).min(self.y_at_x(640.0))
    }

    pub fn y_at_x(&self, x: f32) -> f32 {
        -self.normal.x() * (x - self.vanishing_point.x()) / self.normal.y()
            + self.vanishing_point.y()
    }

    fn find_vanishing_point(
        ground_to_camera: Isometry3<Ground, Camera>,
        intrinsics: &Intrinsic,
    ) -> Option<Point2<Pixel>> {
        let camera_front = Vector3::z_axis();
        let ground_front = ground_to_camera.inverse() * camera_front;
        let ground_front = vector![ground_front.x(), ground_front.y(), 0.0].try_normalize(0.001)?;

        let vanishing_point = ground_to_camera * ground_front;
        let vanishing_point_image = intrinsics.transform(vanishing_point);

        Some(Vector2::wrap(vanishing_point_image.xy().inner).as_point())
    }

    fn find_horizon_normal(
        ground_to_camera: Isometry3<Ground, Camera>,
        intrinsics: &Intrinsic,
    ) -> Option<Vector2<Pixel>> {
        let up = Vector3::z_axis();
        let up_in_camera = ground_to_camera * up;
        let horizon_normal_camera: Vector3<Camera> =
            vector![up_in_camera.x(), up_in_camera.y(), 0.0].try_normalize(0.001)?;
        let horizon_normal_image = intrinsics.transform(horizon_normal_camera).inner;

        Some(Vector2::wrap(horizon_normal_image.xy()))
    }

    pub fn from_parameters(
        ground_to_camera: Isometry3<Ground, Camera>,
        intrinsics: &Intrinsic,
    ) -> Option<Self> {
        let vanishing_point = Self::find_vanishing_point(ground_to_camera, intrinsics)?;
        let normal = Self::find_horizon_normal(ground_to_camera, intrinsics)?;
        Some(Self {
            vanishing_point,
            normal,
        })
    }
}
