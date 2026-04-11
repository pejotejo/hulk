use sha2::Digest;

use crate::dynamic::FieldType;
use crate::entity::{TypeHash, TypeInfo};

/// Convert a canonical ROS type name such as `std_msgs/msg/String` into the
/// corresponding DDS-facing name such as `std_msgs::msg::dds_::String_`.
pub fn canonical_type_name_to_dds(type_name: &str) -> String {
    type_name
        .replace("/msg/", "::msg::dds_::")
        .replace("/srv/", "::srv::dds_::")
        .replace("/action/", "::action::dds_::")
        + "_"
}

/// Convert a DDS-facing type name such as `std_msgs::msg::dds_::String_` into
/// the canonical ROS form `std_msgs/msg/String`.
pub fn dds_type_name_to_canonical(type_name: &str) -> String {
    type_name
        .replace("::msg::dds_::", "/msg/")
        .replace("::srv::dds_::", "/srv/")
        .replace("::action::dds_::", "/action/")
        .trim_end_matches('_')
        .to_string()
}

fn sanitize_generic_name_fragment(fragment: &str) -> String {
    let mut sanitized = String::with_capacity(fragment.len());
    let mut previous_was_underscore = false;

    for ch in fragment.chars() {
        let normalized = if ch.is_ascii_alphanumeric() { ch } else { '_' };
        if normalized == '_' {
            if !previous_was_underscore {
                sanitized.push('_');
                previous_was_underscore = true;
            }
        } else {
            sanitized.push(normalized.to_ascii_lowercase());
            previous_was_underscore = false;
        }
    }

    sanitized.trim_matches('_').to_string()
}

fn short_stable_hash(value: &str) -> String {
    let digest = sha2::Sha256::digest(value.as_bytes());
    let mut hash = String::with_capacity(12);
    for byte in &digest[..6] {
        use std::fmt::Write;
        let _ = write!(&mut hash, "{:02x}", byte);
    }
    hash
}

fn field_type_generic_arg_name(field_type: &FieldType) -> String {
    match field_type {
        FieldType::Bool => "bool".to_string(),
        FieldType::Int8 => "i8".to_string(),
        FieldType::Int16 => "i16".to_string(),
        FieldType::Int32 => "i32".to_string(),
        FieldType::Int64 => "i64".to_string(),
        FieldType::Uint8 => "u8".to_string(),
        FieldType::Uint16 => "u16".to_string(),
        FieldType::Uint32 => "u32".to_string(),
        FieldType::Uint64 => "u64".to_string(),
        FieldType::Float32 => "f32".to_string(),
        FieldType::Float64 => "f64".to_string(),
        FieldType::String => "string".to_string(),
        FieldType::BoundedString(capacity) => format!("string_{}", capacity),
        FieldType::Message(schema) => schema.type_name.clone(),
        FieldType::Optional(inner) => {
            format!("option_{}", field_type_generic_arg_name(inner.as_ref()))
        }
        FieldType::Enum(schema) => schema.type_name.clone(),
        FieldType::Array(inner, len) => {
            format!(
                "array_{}_{}",
                len,
                field_type_generic_arg_name(inner.as_ref())
            )
        }
        FieldType::Sequence(inner) => {
            format!("vec_{}", field_type_generic_arg_name(inner.as_ref()))
        }
        FieldType::BoundedSequence(inner, max) => {
            format!(
                "vec_{}_{}",
                max,
                field_type_generic_arg_name(inner.as_ref())
            )
        }
    }
}

pub fn format_generic_message_type_name(
    base_type_name: &str,
    generic_arg_names: &[String],
) -> String {
    if generic_arg_names.is_empty() {
        return base_type_name.to_string();
    }

    let mut parts = base_type_name.rsplitn(2, '/');
    let leaf_name = parts.next().unwrap_or(base_type_name);
    let prefix = parts.next().unwrap_or_default();

    let mut suffix = generic_arg_names
        .iter()
        .map(|name| sanitize_generic_name_fragment(name))
        .filter(|name| !name.is_empty())
        .collect::<Vec<_>>()
        .join("__");

    if suffix.is_empty() {
        suffix = short_stable_hash(base_type_name);
    } else if suffix.len() > 96 {
        let hash = short_stable_hash(&suffix);
        suffix.truncate(72);
        suffix.push_str("__");
        suffix.push_str(&hash);
    }

    let qualified_leaf = format!("{}__{}", leaf_name, suffix);
    if prefix.is_empty() {
        qualified_leaf
    } else {
        format!("{}/{}", prefix, qualified_leaf)
    }
}

pub trait FieldTypeInfo {
    fn field_type() -> crate::dynamic::FieldType;

    fn generic_arg_name() -> String {
        field_type_generic_arg_name(&Self::field_type())
    }
}

/// Trait for ROS messages that provides message metadata
/// This trait supports both compile-time (static) and runtime (dynamic) type information.
///
/// ## Static Methods (Compile-time)
/// For generated Rust types where type info is known at compile time (e.g., ros-z-msgs).
///
/// ## Dynamic Methods (Runtime)
/// For wrapper types where type info must be queried at runtime (e.g., rcl-z RosMessage).
/// Default implementations delegate to static methods for backward compatibility.
pub trait MessageTypeInfo {
    /// Returns the canonical ROS message type name (e.g., `geometry_msgs/msg/Vector3`).
    /// Static method for compile-time known types
    fn type_name() -> &'static str;

    /// Returns the DDS-facing type name derived from [`Self::type_name()`].
    fn dds_type_name() -> String {
        canonical_type_name_to_dds(Self::type_name())
    }

    /// Returns the type hash (RIHS01 for ROS2, MD5 for ROS1)
    /// Static method for compile-time known types
    fn type_hash() -> TypeHash;

    /// Returns complete TypeInfo combining name and hash
    /// Static method for compile-time known types
    fn type_info() -> TypeInfo {
        TypeInfo::with_hash(Self::type_name(), Self::type_hash())
    }

    /// Returns the package name (extracted from type name)
    /// Static method for compile-time known types
    fn package_name() -> &'static str {
        Self::type_name().split('/').next().unwrap_or("unknown")
    }

    /// Returns whether this message has a fixed size (for optimization)
    /// Static method for compile-time known types
    fn is_fixed_size() -> bool {
        false
    }

    /// Returns the runtime schema for this message type, if available.
    ///
    /// This enables static publishers created via [`crate::node::ZNode::create_pub`]
    /// to auto-register schemas with a node's TypeDescription service.
    ///
    /// Default implementation returns `None` for backward compatibility.
    /// Generated ROS2 message types can override this to return `Some(schema)`.
    fn message_schema() -> Option<std::sync::Arc<crate::dynamic::MessageSchema>> {
        None
    }

    /// Returns the runtime field shape used when this type is nested inside another schema.
    fn field_type() -> crate::dynamic::FieldType {
        crate::dynamic::FieldType::Message(
            Self::message_schema()
                .expect("nested message fields require MessageTypeInfo::message_schema()"),
        )
    }

    /// Register any non-standard schema discovery hooks for this type on the node.
    ///
    /// Core ros-z keeps the standard type-description path separate, so the
    /// default implementation is a no-op. Extended schema derives override this
    /// to register with ros-z's parallel extended type description service.
    fn register_type_extensions(_node: &crate::node::ZNode) -> std::result::Result<(), String> {
        Ok(())
    }

    // === Dynamic Methods (Runtime) ===

    /// Returns the canonical ROS message type name at runtime.
    /// Override this for types that need to query type info dynamically
    fn type_name_dyn(&self) -> String {
        Self::type_name().to_string()
    }

    /// Returns the DDS-facing type name at runtime.
    fn dds_type_name_dyn(&self) -> String {
        canonical_type_name_to_dds(&self.type_name_dyn())
    }

    /// Returns the type hash at runtime
    /// Override this for types that need to query type info dynamically
    fn type_hash_dyn(&self) -> TypeHash {
        Self::type_hash()
    }

    /// Returns complete TypeInfo at runtime
    /// Override this for types that need to query type info dynamically
    fn type_info_dyn(&self) -> TypeInfo {
        TypeInfo::with_hash(&self.type_name_dyn(), self.type_hash_dyn())
    }

    /// Returns the package name at runtime
    fn package_name_dyn(&self) -> String {
        self.type_name_dyn()
            .split('/')
            .next()
            .unwrap_or("unknown")
            .to_string()
    }
}

/// Backward compatibility alias for existing code
pub trait WithTypeInfo: MessageTypeInfo {}

impl<T: MessageTypeInfo> FieldTypeInfo for T {
    fn field_type() -> crate::dynamic::FieldType {
        <T as MessageTypeInfo>::field_type()
    }

    fn generic_arg_name() -> String {
        <T as MessageTypeInfo>::type_name().to_string()
    }
}

macro_rules! impl_primitive_field_type_info {
    ($ty:ty, $field_type:expr, $generic_arg_name:expr) => {
        impl FieldTypeInfo for $ty {
            fn field_type() -> crate::dynamic::FieldType {
                $field_type
            }

            fn generic_arg_name() -> String {
                $generic_arg_name.to_string()
            }
        }
    };
}

impl_primitive_field_type_info!(bool, FieldType::Bool, "bool");
impl_primitive_field_type_info!(i8, FieldType::Int8, "i8");
impl_primitive_field_type_info!(u8, FieldType::Uint8, "u8");
impl_primitive_field_type_info!(i16, FieldType::Int16, "i16");
impl_primitive_field_type_info!(u16, FieldType::Uint16, "u16");
impl_primitive_field_type_info!(i32, FieldType::Int32, "i32");
impl_primitive_field_type_info!(u32, FieldType::Uint32, "u32");
impl_primitive_field_type_info!(i64, FieldType::Int64, "i64");
impl_primitive_field_type_info!(u64, FieldType::Uint64, "u64");
impl_primitive_field_type_info!(f32, FieldType::Float32, "f32");
impl_primitive_field_type_info!(f64, FieldType::Float64, "f64");
impl_primitive_field_type_info!(String, FieldType::String, "string");

impl<T: FieldTypeInfo> FieldTypeInfo for Vec<T> {
    fn field_type() -> crate::dynamic::FieldType {
        FieldType::Sequence(Box::new(T::field_type()))
    }
}

impl<T: FieldTypeInfo> FieldTypeInfo for Option<T> {
    fn field_type() -> crate::dynamic::FieldType {
        FieldType::Optional(Box::new(T::field_type()))
    }
}

impl<T: FieldTypeInfo, const N: usize> FieldTypeInfo for [T; N] {
    fn field_type() -> crate::dynamic::FieldType {
        FieldType::Array(Box::new(T::field_type()), N)
    }
}

/// Trait for ROS service types that provides service-level type information
/// This trait supports both compile-time (static) and runtime (dynamic) type information.
///
/// For services, the type name should be based on the service name (not Request/Response)
/// and the hash should be the composite service hash (not just request or response hash).
///
/// The service hash in ROS2 is computed from a composite type that includes:
/// - request_message (the Request type)
/// - response_message (the Response type)
/// - event_message (a virtual Event type containing ServiceEventInfo, request[], and response[])
///
/// ## Static Methods (Compile-time)
/// For generated Rust service types where type info is known at compile time (e.g., ros-z-msgs).
///
/// ## Dynamic Methods (Runtime)
/// For wrapper types where type info must be queried at runtime (e.g., rcl-z RosService).
/// Default implementations delegate to static methods for backward compatibility.
pub trait ServiceTypeInfo {
    /// Returns the service type info (type name and hash for the service)
    /// Static method for compile-time known types
    fn service_type_info() -> TypeInfo;

    /// Returns the service type info at runtime
    /// Override this for types that need to query type info dynamically
    fn service_type_info_dyn(&self) -> TypeInfo {
        Self::service_type_info()
    }
}

/// Trait for ROS action types that provides action-level type information
/// This trait supports compile-time (static) type information.
///
/// For actions, the type name should be based on the action name and the hash should be
/// the composite action hash.
///
/// ## Static Methods (Compile-time)
/// For generated Rust action types where type info is known at compile time (e.g., ros-z-msgs).
pub trait ActionTypeInfo {
    /// Returns the action type info (type name and hash for the action)
    /// Static method for compile-time known types
    fn action_type_info() -> TypeInfo;

    /// Returns the action type info at runtime
    /// Override this for types that need to query type info dynamically
    fn action_type_info_dyn(&self) -> TypeInfo {
        Self::action_type_info()
    }
}

pub mod srv {

    use crate::msg::{ZMessage, ZService};

    #[allow(non_snake_case)]
    pub mod AddTwoInts {
        use serde::{Deserialize, Serialize};

        pub type Service = (Request, Response);

        #[derive(Debug, Serialize, Deserialize, Default, Clone)]
        pub struct Request {
            pub a: i64,
            pub b: i64,
        }

        #[derive(Debug, Serialize, Deserialize, Default, Clone)]
        pub struct Response {
            pub sum: i64,
        }
    }

    pub enum ZSrv<L, R> {
        L(L),
        R(R),
    }
    impl<RQ: ZMessage, RP: ZMessage> ZService for ZSrv<RQ, RP> {
        type Request = RQ;
        type Response = RP;
    }
    impl<RQ: ZMessage, RP: ZMessage> ZService for (RQ, RP) {
        type Request = RQ;
        type Response = RP;
    }
}
