use nalgebra::{Complex, RealField, SimdRealField};

use crate::{vector, Framed, Rotation2, Rotation3, Vector2, Vector3};

pub type Orientation2<Frame, T = f32> = Framed<Frame, nalgebra::UnitComplex<T>>;
pub type Orientation3<Frame, T = f32> = Framed<Frame, nalgebra::UnitQuaternion<T>>;

impl<Frame, T> Orientation2<Frame, T>
where
    T: SimdRealField + Copy,
    T::Element: SimdRealField,
{
    pub fn new(angle: T) -> Self {
        Self::wrap(nalgebra::UnitComplex::new(angle))
    }

    pub fn identity() -> Self {
        Self::wrap(nalgebra::UnitComplex::identity())
    }

    pub fn mirror(&self) -> Self {
        Self::wrap(self.inner.inverse())
    }

    pub fn from_cos_sin_unchecked(cos: T, sin: T) -> Self {
        Self::wrap(nalgebra::UnitComplex::from_cos_sin_unchecked(cos, sin))
    }

    pub fn from_vector(direction: Vector2<Frame, T>) -> Self
    where
        T: RealField,
    {
        Self::wrap(nalgebra::UnitComplex::rotation_between(
            &nalgebra::Vector2::x_axis(),
            &direction.inner,
        ))
    }

    pub fn as_transform<From>(&self) -> Rotation2<From, Frame, T> {
        Rotation2::wrap(self.inner)
    }

    pub fn angle(&self) -> T {
        self.inner.angle()
    }

    pub fn rotation_to(&self, other: Self) -> Rotation2<Frame, Frame, T> {
        Rotation2::wrap(self.inner.rotation_to(&other.inner))
    }

    pub fn slerp(&self, other: Self, t: T) -> Self {
        Self::wrap(self.inner.slerp(&other.inner, t))
    }

    pub fn as_unit_vector(&self) -> Vector2<Frame, T> {
        let Complex { re, im } = self.inner.as_ref();

        vector![*re, *im]
    }
}

impl<Frame, T> Orientation3<Frame, T>
where
    T: SimdRealField + RealField,
    T::Element: SimdRealField,
{
    pub fn new(axis_angle: Vector3<Frame, T>) -> Self {
        Self::wrap(nalgebra::UnitQuaternion::new(axis_angle.inner))
    }

    pub fn from_euler_angles(roll: T, pitch: T, yaw: T) -> Self {
        Self::wrap(nalgebra::UnitQuaternion::from_euler_angles(
            roll, pitch, yaw,
        ))
    }

    pub fn mirror(&self) -> Self {
        Self::wrap(self.inner.inverse())
    }

    pub fn angle_to(&self, other: Self) -> T {
        self.inner.angle_to(&other.inner)
    }

    pub fn rotation_to(&self, other: Self) -> Rotation3<Frame, Frame, T> {
        Rotation3::wrap(self.inner.rotation_to(&other.inner))
    }

    pub fn rotation<From>(self) -> Rotation3<From, Frame, T> {
        Rotation3::wrap(self.inner)
    }

    pub fn slerp(&self, other: Self, t: T) -> Self {
        Self::wrap(self.inner.slerp(&other.inner, t))
    }

    pub fn euler_angles(&self) -> (T, T, T) {
        self.inner.euler_angles()
    }
}
