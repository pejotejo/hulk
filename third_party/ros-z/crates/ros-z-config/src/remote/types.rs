use serde::{Deserialize, Serialize};

use ros_z::{
    MessageTypeInfo, ServiceTypeInfo, WithTypeInfo,
    entity::{TypeHash, TypeInfo},
    msg::{SerdeCdrSerdes, ZMessage, ZService},
};

use crate::{ConfigKey, LayerPath, snapshot::ConfigTimestamp};

/// JSON payload embedded as a UTF-8 string inside CDR-encoded wire messages.
pub type JsonPayload = String;

/// Origin of a committed config change.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[repr(u8)]
pub enum NodeConfigChangeSource {
    #[default]
    LocalWrite = 0,
    RemoteWrite = 1,
    Reload = 2,
}

/// Request for the full effective config snapshot of one node.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GetNodeConfigSnapshotRequest {}

/// Full effective config snapshot plus per-layer overlays.
///
/// `committed_at` is reported on the node's active clock timeline and is not
/// guaranteed to be host wallclock time when the node uses a logical clock.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GetNodeConfigSnapshotResponse {
    pub success: bool,
    pub message: String,
    pub node_fqn: String,
    pub config_key: ConfigKey,
    pub revision: u64,
    pub committed_at: ConfigTimestamp,
    pub layers: Vec<LayerPath>,
    pub value_json: JsonPayload,
    pub layer_overlays_json: Vec<JsonPayload>,
}

/// Request for the effective value at one field path.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GetNodeConfigValueRequest {
    pub path: String,
}

/// Response containing the effective JSON value for one field path.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GetNodeConfigValueResponse {
    pub success: bool,
    pub message: String,
    pub revision: u64,
    pub path: String,
    pub effective_source_layer: LayerPath,
    pub value_json: JsonPayload,
}

/// Request to set one JSON value in one target layer.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SetNodeConfigRequest {
    pub path: String,
    pub value_json: JsonPayload,
    pub target_layer: LayerPath,
    pub expected_revision: Option<u64>,
}

/// Result of a single-path remote write.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SetNodeConfigResponse {
    pub success: bool,
    pub message: String,
    pub committed_revision: u64,
    pub changed_paths: Vec<String>,
}

/// One JSON write used in a remote atomic batch request.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NodeConfigWriteJson {
    pub path: String,
    pub value_json: JsonPayload,
    pub target_layer: LayerPath,
}

/// Request to apply several JSON writes atomically.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SetNodeConfigAtomicallyRequest {
    pub writes: Vec<NodeConfigWriteJson>,
    pub expected_revision: Option<u64>,
}

/// Result of a remote atomic write batch.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SetNodeConfigAtomicallyResponse {
    pub success: bool,
    pub message: String,
    pub committed_revision: u64,
    pub changed_paths: Vec<String>,
}

/// Request to remove one layer-local override.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ResetNodeConfigRequest {
    pub path: String,
    pub target_layer: LayerPath,
    pub expected_revision: Option<u64>,
}

/// Result of a remote reset operation.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ResetNodeConfigResponse {
    pub success: bool,
    pub message: String,
    pub committed_revision: u64,
    pub changed_paths: Vec<String>,
}

/// Request to reload the node's overlays from disk.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ReloadNodeConfigRequest {}

/// Result of a remote reload request.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ReloadNodeConfigResponse {
    pub success: bool,
    pub message: String,
    pub committed_revision: u64,
    pub changed_paths: Vec<String>,
}

/// Request to list metadata-backed field paths.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ListNodeConfigPathsRequest {
    pub prefixes: Vec<String>,
    pub depth: u64,
    pub writable_only: bool,
}

/// Response containing metadata-backed field paths.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ListNodeConfigPathsResponse {
    pub success: bool,
    pub message: String,
    pub revision: u64,
    pub paths: Vec<String>,
}

/// Request to fetch metadata for selected paths or all paths.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GetNodeConfigMetadataRequest {
    pub paths: Vec<String>,
}

/// Wire representation of one metadata-backed field.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NodeConfigFieldMetadataWire {
    pub path: String,
    pub type_name: String,
    pub description: String,
    pub writable: bool,
    pub min: Option<f64>,
    pub max: Option<f64>,
    pub effective_source_layer: LayerPath,
}

/// Response containing metadata for one or more field paths.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GetNodeConfigMetadataResponse {
    pub success: bool,
    pub message: String,
    pub revision: u64,
    pub metadata: Vec<NodeConfigFieldMetadataWire>,
}

/// One changed field in a published config event.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NodeConfigChange {
    pub path: String,
    pub effective_source_layer: LayerPath,
    pub old_value_json: JsonPayload,
    pub new_value_json: JsonPayload,
}

/// Published config event on `~config/events`.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NodeConfigEvent {
    pub node_fqn: String,
    pub config_key: ConfigKey,
    pub previous_revision: u64,
    pub revision: u64,
    pub source: NodeConfigChangeSource,
    pub changed_paths: Vec<String>,
    pub changes: Vec<NodeConfigChange>,
}

macro_rules! impl_zmessage {
    ($($ty:ty),* $(,)?) => {
        $(impl ZMessage for $ty {
            type Serdes = SerdeCdrSerdes<Self>;
        })*
    };
}

impl_zmessage!(
    GetNodeConfigSnapshotRequest,
    GetNodeConfigSnapshotResponse,
    GetNodeConfigValueRequest,
    GetNodeConfigValueResponse,
    SetNodeConfigRequest,
    SetNodeConfigResponse,
    NodeConfigWriteJson,
    SetNodeConfigAtomicallyRequest,
    SetNodeConfigAtomicallyResponse,
    ResetNodeConfigRequest,
    ResetNodeConfigResponse,
    ReloadNodeConfigRequest,
    ReloadNodeConfigResponse,
    ListNodeConfigPathsRequest,
    ListNodeConfigPathsResponse,
    GetNodeConfigMetadataRequest,
    NodeConfigFieldMetadataWire,
    GetNodeConfigMetadataResponse,
    NodeConfigChange,
    NodeConfigEvent,
);

impl MessageTypeInfo for NodeConfigEvent {
    fn type_name() -> &'static str {
        "ros_z_config/msg/NodeConfigEvent"
    }

    fn type_hash() -> TypeHash {
        TypeHash::zero()
    }
}

impl WithTypeInfo for NodeConfigEvent {}

macro_rules! impl_service {
    ($srv:ident, $req:ty, $res:ty, $name:literal) => {
        pub struct $srv;

        impl ZService for $srv {
            type Request = $req;
            type Response = $res;
        }

        impl ServiceTypeInfo for $srv {
            fn service_type_info() -> TypeInfo {
                TypeInfo::new($name, None)
            }
        }
    };
}

impl_service!(
    GetNodeConfigSnapshotSrv,
    GetNodeConfigSnapshotRequest,
    GetNodeConfigSnapshotResponse,
    "ros_z_config/srv/GetNodeConfigSnapshot"
);
impl_service!(
    GetNodeConfigValueSrv,
    GetNodeConfigValueRequest,
    GetNodeConfigValueResponse,
    "ros_z_config/srv/GetNodeConfigValue"
);
impl_service!(
    SetNodeConfigSrv,
    SetNodeConfigRequest,
    SetNodeConfigResponse,
    "ros_z_config/srv/SetNodeConfig"
);
impl_service!(
    SetNodeConfigAtomicallySrv,
    SetNodeConfigAtomicallyRequest,
    SetNodeConfigAtomicallyResponse,
    "ros_z_config/srv/SetNodeConfigAtomically"
);
impl_service!(
    ResetNodeConfigSrv,
    ResetNodeConfigRequest,
    ResetNodeConfigResponse,
    "ros_z_config/srv/ResetNodeConfig"
);
impl_service!(
    ReloadNodeConfigSrv,
    ReloadNodeConfigRequest,
    ReloadNodeConfigResponse,
    "ros_z_config/srv/ReloadNodeConfig"
);
impl_service!(
    ListNodeConfigPathsSrv,
    ListNodeConfigPathsRequest,
    ListNodeConfigPathsResponse,
    "ros_z_config/srv/ListNodeConfigPaths"
);
impl_service!(
    GetNodeConfigMetadataSrv,
    GetNodeConfigMetadataRequest,
    GetNodeConfigMetadataResponse,
    "ros_z_config/srv/GetNodeConfigMetadata"
);
