use color_eyre::eyre::{Result, eyre};
use ros_z_config::{
    ConfigTimestamp, GetNodeConfigMetadataResponse, GetNodeConfigSnapshotResponse,
    GetNodeConfigValueResponse, NodeConfigChange, NodeConfigChangeSource, NodeConfigEvent,
};
use serde::Serialize;
use serde_json::Value;

use crate::support::config::config_scope_name;

#[derive(Debug, Clone, Serialize)]
pub struct ConfigSnapshotView {
    pub node: String,
    pub revision: u64,
    pub committed_at: ConfigTimestamp,
    pub location: String,
    pub robot: String,
    pub effective: Value,
    pub overlays: ConfigSnapshotOverlaysView,
}

#[derive(Debug, Clone, Serialize)]
pub struct ConfigSnapshotOverlaysView {
    pub default: Value,
    pub location: Value,
    pub robot: Value,
}

impl ConfigSnapshotView {
    pub fn from_response(response: GetNodeConfigSnapshotResponse) -> Result<Self> {
        Ok(Self {
            node: response.node_fqn,
            revision: response.revision,
            committed_at: response.committed_at,
            location: response.location,
            robot: response.robot,
            effective: parse_json_field("effective config", &response.value_json)?,
            overlays: ConfigSnapshotOverlaysView {
                default: parse_json_field("default overlay", &response.default_overlay_json)?,
                location: parse_json_field("location overlay", &response.location_overlay_json)?,
                robot: parse_json_field("robot overlay", &response.robot_overlay_json)?,
            },
        })
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct ConfigValueView {
    pub node: String,
    pub path: String,
    pub revision: u64,
    pub effective_source_scope: String,
    pub value: Value,
}

impl ConfigValueView {
    pub fn from_response(node: String, response: GetNodeConfigValueResponse) -> Result<Self> {
        Ok(Self {
            node,
            path: response.path,
            revision: response.revision,
            effective_source_scope: config_scope_name(response.effective_source_scope).to_string(),
            value: parse_json_field("config value", &response.value_json)?,
        })
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct ConfigMutationView {
    pub node: String,
    pub operation: String,
    pub path: Option<String>,
    pub target_scope: Option<String>,
    pub committed_revision: u64,
    pub changed_paths: Vec<String>,
    pub successful: bool,
}

impl ConfigMutationView {
    pub fn new(
        node: String,
        operation: impl Into<String>,
        path: Option<String>,
        target_scope: Option<String>,
        committed_revision: u64,
        changed_paths: Vec<String>,
        successful: bool,
    ) -> Self {
        Self {
            node,
            operation: operation.into(),
            path,
            target_scope,
            committed_revision,
            changed_paths,
            successful,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct ConfigPathsView {
    pub node: String,
    pub revision: u64,
    pub prefixes: Vec<String>,
    pub depth: u64,
    pub writable_only: bool,
    pub paths: Vec<String>,
}

impl ConfigPathsView {
    pub fn new(
        node: String,
        revision: u64,
        prefixes: Vec<String>,
        depth: u64,
        writable_only: bool,
        paths: Vec<String>,
    ) -> Self {
        Self {
            node,
            revision,
            prefixes,
            depth,
            writable_only,
            paths,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct ConfigMetadataView {
    pub node: String,
    pub revision: u64,
    pub requested_paths: Vec<String>,
    pub metadata: Vec<ConfigMetadataFieldView>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ConfigMetadataFieldView {
    pub path: String,
    pub type_name: String,
    pub description: String,
    pub writable: bool,
    pub allowed_scopes: Vec<String>,
    pub min: Option<f64>,
    pub max: Option<f64>,
    pub effective_source_scope: String,
}

impl ConfigMetadataView {
    pub fn from_response(
        node: String,
        requested_paths: Vec<String>,
        response: GetNodeConfigMetadataResponse,
    ) -> Self {
        Self {
            node,
            revision: response.revision,
            requested_paths,
            metadata: response
                .metadata
                .into_iter()
                .map(|field| ConfigMetadataFieldView {
                    path: field.path,
                    type_name: field.type_name,
                    description: field.description,
                    writable: field.writable,
                    allowed_scopes: field
                        .allowed_scopes
                        .into_iter()
                        .map(|scope| config_scope_name(scope).to_string())
                        .collect(),
                    min: field.min,
                    max: field.max,
                    effective_source_scope: config_scope_name(field.effective_source_scope)
                        .to_string(),
                })
                .collect(),
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct ConfigWatchEventView {
    pub node: String,
    pub previous_revision: u64,
    pub revision: u64,
    pub source: String,
    pub changed_paths: Vec<String>,
    pub changes: Vec<ConfigWatchChangeView>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ConfigWatchChangeView {
    pub path: String,
    pub effective_source_scope: String,
    pub old_value: Value,
    pub new_value: Value,
}

impl ConfigWatchEventView {
    pub fn from_event(event: NodeConfigEvent) -> Result<Self> {
        Ok(Self {
            node: event.node_fqn,
            previous_revision: event.previous_revision,
            revision: event.revision,
            source: change_source_name(event.source).to_string(),
            changed_paths: event.changed_paths,
            changes: event
                .changes
                .into_iter()
                .map(ConfigWatchChangeView::from_change)
                .collect::<Result<Vec<_>>>()?,
        })
    }
}

impl ConfigWatchChangeView {
    fn from_change(change: NodeConfigChange) -> Result<Self> {
        Ok(Self {
            path: change.path,
            effective_source_scope: config_scope_name(change.effective_source_scope).to_string(),
            old_value: parse_json_field("old config value", &change.old_value_json)?,
            new_value: parse_json_field("new config value", &change.new_value_json)?,
        })
    }
}

fn parse_json_field(label: &str, value: &str) -> Result<Value> {
    serde_json::from_str(value).map_err(|err| eyre!("failed to parse {label}: {err}"))
}

fn change_source_name(source: NodeConfigChangeSource) -> &'static str {
    match source {
        NodeConfigChangeSource::LocalWrite => "local_write",
        NodeConfigChangeSource::RemoteWrite => "remote_write",
        NodeConfigChangeSource::Reload => "reload",
    }
}
