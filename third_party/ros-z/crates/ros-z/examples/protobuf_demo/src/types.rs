/// Shared protobuf type definitions and trait implementations
use ros_z::{
    MessageTypeInfo, ServiceTypeInfo,
    entity::TypeHash,
    msg::{ProtobufSerdes, ZMessage, ZService},
};

// Include protobuf messages generated from sensor_data.proto
pub mod generated {
    include!(concat!(env!("OUT_DIR"), "/examples.rs"));
}

pub use generated::{CalculateRequest, CalculateResponse, SensorData};

// ========== SensorData Trait Implementations ==========

impl MessageTypeInfo for SensorData {
    fn type_name() -> &'static str {
        "examples/msg/SensorData"
    }

    fn type_hash() -> TypeHash {
        TypeHash::zero() // For custom messages without ROS type support
    }
}

impl ZMessage for SensorData {
    type Serdes = ros_z::msg::SerdeCdrSerdes<SensorData>;
}

// SensorData uses serde/CDR for backward compatibility with the original pub/sub demo

// ========== CalculateRequest Trait Implementations ==========

impl MessageTypeInfo for CalculateRequest {
    fn type_name() -> &'static str {
        "examples/srv/Calculate_Request"
    }

    fn type_hash() -> TypeHash {
        TypeHash::zero()
    }
}

// Explicitly implement ZMessage to use ProtobufSerdes for pure protobuf serialization
impl ZMessage for CalculateRequest {
    type Serdes = ProtobufSerdes<CalculateRequest>;
}

// ========== CalculateResponse Trait Implementations ==========

impl MessageTypeInfo for CalculateResponse {
    fn type_name() -> &'static str {
        "examples/srv/Calculate_Response"
    }

    fn type_hash() -> TypeHash {
        TypeHash::zero()
    }
}

// Explicitly implement ZMessage to use ProtobufSerdes for pure protobuf serialization
impl ZMessage for CalculateResponse {
    type Serdes = ProtobufSerdes<CalculateResponse>;
}

// ========== Calculate Service Definition ==========

pub struct Calculate;

impl ServiceTypeInfo for Calculate {
    fn service_type_info() -> ros_z::entity::TypeInfo {
        ros_z::entity::TypeInfo::new("examples/srv/Calculate", None)
    }
}

impl ZService for Calculate {
    type Request = CalculateRequest;
    type Response = CalculateResponse;
}
