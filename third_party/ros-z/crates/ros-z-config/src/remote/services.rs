use std::sync::Arc;

use ros_z::{
    Builder,
    attachment::Attachment,
    msg::ZMessage,
    node::ZNode,
    pubsub::ZPub,
    qos::{QosDurability, QosHistory, QosProfile, QosReliability},
    service::ZServer,
};
use serde::Serialize;
use serde::de::DeserializeOwned;
use zenoh::Wait;

use crate::{
    ConfigError, ConfigFieldMetadata, NodeConfig,
    node_config::{CommitOutcome, NodeConfigInner},
    remote::types::*,
};

pub struct RemoteConfigServices<T> {
    event_publisher: ZPub<NodeConfigEvent>,
    _get_snapshot: Arc<ZServer<GetNodeConfigSnapshotSrv, ()>>,
    _get_value: Arc<ZServer<GetNodeConfigValueSrv, ()>>,
    _set: Arc<ZServer<SetNodeConfigSrv, ()>>,
    _set_atomic: Arc<ZServer<SetNodeConfigAtomicallySrv, ()>>,
    _reset: Arc<ZServer<ResetNodeConfigSrv, ()>>,
    _reload: Arc<ZServer<ReloadNodeConfigSrv, ()>>,
    _list_paths: Option<Arc<ZServer<ListNodeConfigPathsSrv, ()>>>,
    _get_metadata: Option<Arc<ZServer<GetNodeConfigMetadataSrv, ()>>>,
    _phantom: std::marker::PhantomData<T>,
}

impl<T> RemoteConfigServices<T>
where
    T: Serialize + DeserializeOwned + Send + Sync + 'static,
{
    pub fn register(
        node: &ZNode,
        inner: Arc<NodeConfigInner<T>>,
        metadata: Option<Arc<Vec<ConfigFieldMetadata>>>,
    ) -> crate::Result<Self> {
        let event_publisher = node
            .create_pub::<NodeConfigEvent>("~config/events")
            .with_qos(QosProfile {
                reliability: QosReliability::Reliable,
                durability: QosDurability::TransientLocal,
                history: QosHistory::KeepLast(std::num::NonZeroUsize::new(64).expect("non-zero")),
                ..Default::default()
            })
            .build()
            .map_err(|err| ConfigError::RemoteError {
                message: err.to_string(),
            })?;

        let get_snapshot =
            register_server::<GetNodeConfigSnapshotSrv>(node, "~config/get_snapshot", {
                let inner = inner.clone();
                move |query| handle_get_snapshot::<T>(&inner, query)
            })?;

        let get_value = register_server::<GetNodeConfigValueSrv>(node, "~config/get_value", {
            let inner = inner.clone();
            move |query| handle_get_value::<T>(&inner, query)
        })?;

        let set = register_server::<SetNodeConfigSrv>(node, "~config/set", {
            let inner = inner.clone();
            move |query| handle_set::<T>(&inner, query)
        })?;

        let set_atomic =
            register_server::<SetNodeConfigAtomicallySrv>(node, "~config/set_atomic", {
                let inner = inner.clone();
                move |query| handle_set_atomic::<T>(&inner, query)
            })?;

        let reset = register_server::<ResetNodeConfigSrv>(node, "~config/reset", {
            let inner = inner.clone();
            move |query| handle_reset::<T>(&inner, query)
        })?;

        let reload = register_server::<ReloadNodeConfigSrv>(node, "~config/reload", {
            let inner = inner.clone();
            move |query| handle_reload::<T>(&inner, query)
        })?;

        let list_paths = if metadata.is_some() {
            Some(Arc::new(register_server::<ListNodeConfigPathsSrv>(
                node,
                "~config/list_paths",
                {
                    let inner = inner.clone();
                    move |query| handle_list_paths::<T>(&inner, query)
                },
            )?))
        } else {
            None
        };

        let get_metadata = if metadata.is_some() {
            Some(Arc::new(register_server::<GetNodeConfigMetadataSrv>(
                node,
                "~config/get_metadata",
                {
                    let inner = inner.clone();
                    move |query| handle_get_metadata::<T>(&inner, query)
                },
            )?))
        } else {
            None
        };

        Ok(Self {
            event_publisher,
            _get_snapshot: Arc::new(get_snapshot),
            _get_value: Arc::new(get_value),
            _set: Arc::new(set),
            _set_atomic: Arc::new(set_atomic),
            _reset: Arc::new(reset),
            _reload: Arc::new(reload),
            _list_paths: list_paths,
            _get_metadata: get_metadata,
            _phantom: std::marker::PhantomData,
        })
    }

    pub fn publish_event(&self, event: &NodeConfigEvent) -> crate::Result<()> {
        self.event_publisher
            .publish(event)
            .map_err(|err| ConfigError::RemoteError {
                message: err.to_string(),
            })
    }
}

fn register_server<S>(
    node: &ZNode,
    name: &str,
    handler: impl Fn(&zenoh::query::Query) + Send + Sync + 'static,
) -> crate::Result<ZServer<S, ()>>
where
    S: ros_z::msg::ZService + ros_z::ServiceTypeInfo,
{
    node.create_service::<S>(name)
        .build_with_callback(move |query| handler(&query))
        .map_err(|err| ConfigError::RemoteError {
            message: err.to_string(),
        })
}

fn handle_get_snapshot<T>(inner: &Arc<NodeConfigInner<T>>, query: &zenoh::query::Query)
where
    T: Serialize + DeserializeOwned + Send + Sync + 'static,
{
    let config = NodeConfig {
        inner: inner.clone(),
    };
    let snapshot = config.snapshot();
    let response = GetNodeConfigSnapshotResponse {
        success: true,
        message: String::new(),
        node_fqn: snapshot.node_fqn.clone(),
        config_key: snapshot.config_key.clone(),
        revision: snapshot.revision,
        committed_at: snapshot.committed_at,
        layers: snapshot.layers.clone(),
        value_json: to_json(&snapshot.effective),
        layer_overlays_json: snapshot.layer_overlays.iter().map(to_json).collect(),
    };
    reply(query, &response);
}

fn handle_get_value<T>(inner: &Arc<NodeConfigInner<T>>, query: &zenoh::query::Query)
where
    T: Serialize + DeserializeOwned + Send + Sync + 'static,
{
    let config = NodeConfig {
        inner: inner.clone(),
    };
    let request = decode_request::<GetNodeConfigValueRequest>(query);
    let response = match request {
        Ok(request) => {
            let snapshot = config.snapshot();
            match config.get_json(&request.path) {
                Ok(value) => GetNodeConfigValueResponse {
                    success: true,
                    message: String::new(),
                    revision: snapshot.revision,
                    path: request.path.clone(),
                    effective_source_layer: snapshot
                        .effective_source_layer(&request.path)
                        .unwrap_or_default(),
                    value_json: to_json(&value),
                },
                Err(err) => GetNodeConfigValueResponse {
                    success: false,
                    message: err.to_string(),
                    revision: snapshot.revision,
                    path: request.path,
                    effective_source_layer: String::new(),
                    value_json: "null".to_string(),
                },
            }
        }
        Err(message) => GetNodeConfigValueResponse {
            success: false,
            message,
            revision: 0,
            path: String::new(),
            effective_source_layer: String::new(),
            value_json: "null".to_string(),
        },
    };
    reply(query, &response);
}

fn handle_set<T>(inner: &Arc<NodeConfigInner<T>>, query: &zenoh::query::Query)
where
    T: Serialize + DeserializeOwned + Send + Sync + 'static,
{
    let config = NodeConfig {
        inner: inner.clone(),
    };
    let request = decode_request::<SetNodeConfigRequest>(query);
    let response = match request {
        Ok(request) => match serde_json::from_str(&request.value_json) {
            Ok(value) => match config.commit(
                &[crate::ConfigJsonWrite {
                    path: request.path,
                    value,
                    target_layer: request.target_layer,
                }],
                &[],
                request.expected_revision,
                NodeConfigChangeSource::RemoteWrite,
            ) {
                Ok(outcome) => write_response(outcome),
                Err(err) => error_write_response(err),
            },
            Err(err) => error_write_response(ConfigError::RemoteError {
                message: err.to_string(),
            }),
        },
        Err(message) => SetNodeConfigResponse {
            success: false,
            message,
            committed_revision: 0,
            changed_paths: Vec::new(),
        },
    };
    reply(query, &response);
}

fn handle_set_atomic<T>(inner: &Arc<NodeConfigInner<T>>, query: &zenoh::query::Query)
where
    T: Serialize + DeserializeOwned + Send + Sync + 'static,
{
    let config = NodeConfig {
        inner: inner.clone(),
    };
    let request = decode_request::<SetNodeConfigAtomicallyRequest>(query);
    let response = match request {
        Ok(request) => {
            let mut writes = Vec::with_capacity(request.writes.len());
            let mut parse_error = None;
            for write in request.writes {
                match serde_json::from_str(&write.value_json) {
                    Ok(value) => writes.push(crate::ConfigJsonWrite {
                        path: write.path,
                        value,
                        target_layer: write.target_layer,
                    }),
                    Err(err) => {
                        parse_error = Some(err.to_string());
                        break;
                    }
                }
            }

            if let Some(message) = parse_error {
                error_write_response(ConfigError::RemoteError { message })
            } else {
                match config.commit(
                    &writes,
                    &[],
                    request.expected_revision,
                    NodeConfigChangeSource::RemoteWrite,
                ) {
                    Ok(outcome) => write_response(outcome),
                    Err(err) => error_write_response(err),
                }
            }
        }
        Err(message) => SetNodeConfigAtomicallyResponse {
            success: false,
            message,
            committed_revision: 0,
            changed_paths: Vec::new(),
        },
    };
    reply(query, &response);
}

fn handle_reset<T>(inner: &Arc<NodeConfigInner<T>>, query: &zenoh::query::Query)
where
    T: Serialize + DeserializeOwned + Send + Sync + 'static,
{
    let config = NodeConfig {
        inner: inner.clone(),
    };
    let request = decode_request::<ResetNodeConfigRequest>(query);
    let response = match request {
        Ok(request) => match config.commit(
            &[],
            &[(request.path, request.target_layer)],
            request.expected_revision,
            NodeConfigChangeSource::RemoteWrite,
        ) {
            Ok(outcome) => write_response(outcome),
            Err(err) => error_write_response(err),
        },
        Err(message) => ResetNodeConfigResponse {
            success: false,
            message,
            committed_revision: 0,
            changed_paths: Vec::new(),
        },
    };
    reply(query, &response);
}

fn handle_reload<T>(inner: &Arc<NodeConfigInner<T>>, query: &zenoh::query::Query)
where
    T: Serialize + DeserializeOwned + Send + Sync + 'static,
{
    let config = NodeConfig {
        inner: inner.clone(),
    };
    let response = match config.reload_with_source(NodeConfigChangeSource::Reload) {
        Ok(outcome) => ReloadNodeConfigResponse {
            success: true,
            message: String::new(),
            committed_revision: outcome.committed_revision,
            changed_paths: outcome.changed_paths,
        },
        Err(err) => ReloadNodeConfigResponse {
            success: false,
            message: err.to_string(),
            committed_revision: 0,
            changed_paths: Vec::new(),
        },
    };
    reply(query, &response);
}

fn handle_list_paths<T>(inner: &Arc<NodeConfigInner<T>>, query: &zenoh::query::Query)
where
    T: Serialize + DeserializeOwned + Send + Sync + 'static,
{
    let config = NodeConfig {
        inner: inner.clone(),
    };
    let request = decode_request::<ListNodeConfigPathsRequest>(query).unwrap_or_default();
    let response = match config.list_paths() {
        Ok(paths) => {
            let filtered = paths
                .into_iter()
                .filter(|path| {
                    request.prefixes.is_empty()
                        || request
                            .prefixes
                            .iter()
                            .any(|prefix| path.starts_with(prefix))
                })
                .filter(|path| {
                    if request.depth == 0 {
                        true
                    } else {
                        path.split('.').count() as u64 <= request.depth
                    }
                })
                .filter(|path| {
                    if !request.writable_only {
                        true
                    } else {
                        config
                            .get_metadata(path)
                            .map(|meta| meta.writable)
                            .unwrap_or(false)
                    }
                })
                .collect();
            ListNodeConfigPathsResponse {
                success: true,
                message: String::new(),
                revision: config.snapshot().revision,
                paths: filtered,
            }
        }
        Err(err) => ListNodeConfigPathsResponse {
            success: false,
            message: err.to_string(),
            revision: config.snapshot().revision,
            paths: Vec::new(),
        },
    };
    reply(query, &response);
}

fn handle_get_metadata<T>(inner: &Arc<NodeConfigInner<T>>, query: &zenoh::query::Query)
where
    T: Serialize + DeserializeOwned + Send + Sync + 'static,
{
    let config = NodeConfig {
        inner: inner.clone(),
    };
    let request = decode_request::<GetNodeConfigMetadataRequest>(query).unwrap_or_default();
    let snapshot = config.snapshot();
    let selected_paths = if request.paths.is_empty() {
        config.list_paths().unwrap_or_default()
    } else {
        request.paths
    };

    let mut metadata = Vec::new();
    for path in selected_paths {
        if let Ok(field) = config.get_metadata(&path) {
            metadata.push(NodeConfigFieldMetadataWire {
                path: field.path,
                type_name: field.type_name,
                description: field.description,
                writable: field.writable,
                min: field.min,
                max: field.max,
                effective_source_layer: snapshot.effective_source_layer(&path).unwrap_or_default(),
            });
        }
    }

    let response = GetNodeConfigMetadataResponse {
        success: true,
        message: String::new(),
        revision: snapshot.revision,
        metadata,
    };
    reply(query, &response);
}

fn write_response<T>(outcome: CommitOutcome) -> T
where
    T: From<(bool, String, u64, Vec<String>)>,
{
    T::from((
        true,
        String::new(),
        outcome.committed_revision,
        outcome.changed_paths,
    ))
}

fn error_write_response<T>(err: ConfigError) -> T
where
    T: From<(bool, String, u64, Vec<String>)>,
{
    T::from((false, err.to_string(), 0, Vec::new()))
}

impl From<(bool, String, u64, Vec<String>)> for SetNodeConfigResponse {
    fn from(value: (bool, String, u64, Vec<String>)) -> Self {
        Self {
            success: value.0,
            message: value.1,
            committed_revision: value.2,
            changed_paths: value.3,
        }
    }
}

impl From<(bool, String, u64, Vec<String>)> for SetNodeConfigAtomicallyResponse {
    fn from(value: (bool, String, u64, Vec<String>)) -> Self {
        Self {
            success: value.0,
            message: value.1,
            committed_revision: value.2,
            changed_paths: value.3,
        }
    }
}

impl From<(bool, String, u64, Vec<String>)> for ResetNodeConfigResponse {
    fn from(value: (bool, String, u64, Vec<String>)) -> Self {
        Self {
            success: value.0,
            message: value.1,
            committed_revision: value.2,
            changed_paths: value.3,
        }
    }
}

fn decode_request<T>(query: &zenoh::query::Query) -> std::result::Result<T, String>
where
    T: ZMessage,
    for<'a> <T as ZMessage>::Serdes: ros_z::msg::ZDeserializer<Output = T, Input<'a> = &'a [u8]>,
{
    let payload = query
        .payload()
        .ok_or_else(|| "missing request payload".to_string())?;
    T::deserialize(payload.to_bytes().as_ref()).map_err(|err| err.to_string())
}

fn reply<T>(query: &zenoh::query::Query, response: &T)
where
    T: ZMessage,
{
    let bytes = response.serialize();
    let mut reply = query.reply(query.key_expr().clone(), bytes);
    if let Some(att_bytes) = query.attachment()
        && let Ok(att) = Attachment::try_from(att_bytes)
    {
        reply = reply.attachment(att);
    }
    if let Err(err) = reply.wait() {
        tracing::warn!("[CFG] Failed to send config reply: {err}");
    }
}

fn to_json(value: &serde_json::Value) -> String {
    serde_json::to_string(value).unwrap_or_else(|_| "null".to_string())
}
