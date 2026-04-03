use std::sync::{Arc, OnceLock};

use nalgebra::{
    Isometry2, Isometry3, Point2, Point3, Rotation2, Rotation3, Translation2, Translation3,
    UnitComplex, UnitQuaternion, Vector2, Vector3,
};

use crate::{
    FieldTypeInfo,
    dynamic::{FieldSchema, FieldType, MessageSchema},
};

macro_rules! impl_vector_field_type {
    ($ty:ty, $field_type:expr, $len:expr) => {
        impl FieldTypeInfo for $ty {
            fn field_type() -> FieldType {
                FieldType::Array(Box::new($field_type), $len)
            }
        }
    };
}

impl_vector_field_type!(Vector2<f32>, FieldType::Float32, 2);
impl_vector_field_type!(Vector2<f64>, FieldType::Float64, 2);
impl_vector_field_type!(Vector3<f32>, FieldType::Float32, 3);
impl_vector_field_type!(Vector3<f64>, FieldType::Float64, 3);
impl_vector_field_type!(Point2<f32>, FieldType::Float32, 2);
impl_vector_field_type!(Point2<f64>, FieldType::Float64, 2);
impl_vector_field_type!(Point3<f32>, FieldType::Float32, 3);
impl_vector_field_type!(Point3<f64>, FieldType::Float64, 3);
impl_vector_field_type!(Translation2<f32>, FieldType::Float32, 2);
impl_vector_field_type!(Translation2<f64>, FieldType::Float64, 2);
impl_vector_field_type!(Translation3<f32>, FieldType::Float32, 3);
impl_vector_field_type!(Translation3<f64>, FieldType::Float64, 3);
impl_vector_field_type!(Rotation2<f32>, FieldType::Float32, 4);
impl_vector_field_type!(Rotation2<f64>, FieldType::Float64, 4);
impl_vector_field_type!(Rotation3<f32>, FieldType::Float32, 9);
impl_vector_field_type!(Rotation3<f64>, FieldType::Float64, 9);
impl_vector_field_type!(UnitComplex<f32>, FieldType::Float32, 2);
impl_vector_field_type!(UnitComplex<f64>, FieldType::Float64, 2);
impl_vector_field_type!(UnitQuaternion<f32>, FieldType::Float32, 4);
impl_vector_field_type!(UnitQuaternion<f64>, FieldType::Float64, 4);

fn isometry_schema(
    type_name: &str,
    rotation: FieldType,
    translation: FieldType,
) -> Arc<MessageSchema> {
    Arc::new(MessageSchema {
        type_name: type_name.to_string(),
        package: "nalgebra".to_string(),
        name: type_name
            .rsplit('/')
            .next()
            .unwrap_or(type_name)
            .to_string(),
        fields: vec![
            FieldSchema::new("rotation", rotation),
            FieldSchema::new("translation", translation),
        ],
        type_hash: None,
    })
}

impl FieldTypeInfo for Isometry2<f32> {
    fn field_type() -> FieldType {
        static SCHEMA: OnceLock<Arc<MessageSchema>> = OnceLock::new();
        FieldType::Message(
            SCHEMA
                .get_or_init(|| {
                    isometry_schema(
                        "nalgebra/msg/Isometry2F32",
                        <UnitComplex<f32> as FieldTypeInfo>::field_type(),
                        <Translation2<f32> as FieldTypeInfo>::field_type(),
                    )
                })
                .clone(),
        )
    }
}

impl FieldTypeInfo for Isometry2<f64> {
    fn field_type() -> FieldType {
        static SCHEMA: OnceLock<Arc<MessageSchema>> = OnceLock::new();
        FieldType::Message(
            SCHEMA
                .get_or_init(|| {
                    isometry_schema(
                        "nalgebra/msg/Isometry2F64",
                        <UnitComplex<f64> as FieldTypeInfo>::field_type(),
                        <Translation2<f64> as FieldTypeInfo>::field_type(),
                    )
                })
                .clone(),
        )
    }
}

impl FieldTypeInfo for Isometry3<f32> {
    fn field_type() -> FieldType {
        static SCHEMA: OnceLock<Arc<MessageSchema>> = OnceLock::new();
        FieldType::Message(
            SCHEMA
                .get_or_init(|| {
                    isometry_schema(
                        "nalgebra/msg/Isometry3F32",
                        <UnitQuaternion<f32> as FieldTypeInfo>::field_type(),
                        <Translation3<f32> as FieldTypeInfo>::field_type(),
                    )
                })
                .clone(),
        )
    }
}

impl FieldTypeInfo for Isometry3<f64> {
    fn field_type() -> FieldType {
        static SCHEMA: OnceLock<Arc<MessageSchema>> = OnceLock::new();
        FieldType::Message(
            SCHEMA
                .get_or_init(|| {
                    isometry_schema(
                        "nalgebra/msg/Isometry3F64",
                        <UnitQuaternion<f64> as FieldTypeInfo>::field_type(),
                        <Translation3<f64> as FieldTypeInfo>::field_type(),
                    )
                })
                .clone(),
        )
    }
}
