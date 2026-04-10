use std::{
    collections::BTreeSet,
    thread,
    sync::{Arc, Mutex},
    sync::atomic::{AtomicU64, Ordering},
    time::Duration,
};

use color_eyre::eyre::eyre;
use log::error;
use ros_z::{
    Builder,
    context::{ZContext, ZContextBuilder},
    dynamic::DynamicMessage,
    graph::Graph,
    node::ZNode,
    pubsub::Received,
    time::ZTime,
};
use ros_z_config::{
    ConfigScope,
    GetNodeConfigMetadataResponse,
    GetNodeConfigSnapshotResponse,
    GetNodeConfigValueResponse,
    ListNodeConfigPathsResponse,
    RemoteConfigClient,
    ResetNodeConfigResponse,
    SetNodeConfigResponse,
};
use serde_json::Value;
use tokio::{
    runtime::{Builder as RuntimeBuilder, Runtime},
    sync::watch,
};

use crate::{
    backend::{
        BackendCapability, BackendConnectionStatus, BackendError, BackendResult,
        ConfigNodeDescriptor, ConfigNodeListState, TopicDescriptor, TopicListState, TwixTime,
    },
    change_buffer::{Change, ChangeBuffer, ChangeBufferHandle},
    dynamic_json::dynamic_message_to_json,
    value_buffer::{Buffer, BufferHandle, Datum},
};

type ChangeCallback = Arc<dyn Fn() + Send + Sync + 'static>;
const DYNAMIC_SCHEMA_DISCOVERY_TIMEOUT: Duration = Duration::from_secs(10);

struct ConnectedBackend {
    generation: u64,
    _context: Arc<ZContext>,
    node: Arc<ZNode>,
}

pub struct Robot {
    runtime: Runtime,
    endpoint: Arc<Mutex<String>>,
    current_backend: Arc<Mutex<Option<Arc<ConnectedBackend>>>>,
    backend_tx: watch::Sender<Option<Arc<ConnectedBackend>>>,
    status_tx: watch::Sender<BackendConnectionStatus>,
    callbacks: Arc<Mutex<Vec<ChangeCallback>>>,
    generation: AtomicU64,
}

impl Robot {
    pub fn new(endpoint: String, _repository: Option<repository::Repository>) -> Self {
        let runtime = RuntimeBuilder::new_multi_thread().enable_all().build().unwrap();
        let (backend_tx, _) = watch::channel(None);
        let (status_tx, _) = watch::channel(BackendConnectionStatus::Disconnected);

        Self {
            runtime,
            endpoint: Arc::new(Mutex::new(endpoint)),
            current_backend: Arc::new(Mutex::new(None)),
            backend_tx,
            status_tx,
            callbacks: Arc::new(Mutex::new(Vec::new())),
            generation: AtomicU64::new(1),
        }
    }

    pub fn connect(&self) {
        if !matches!(
            *self.status_tx.borrow(),
            BackendConnectionStatus::Disconnected
        ) {
            return;
        }

        let endpoint = self.endpoint.lock().unwrap().clone();
        let current_backend = self.current_backend.clone();
        let backend_tx = self.backend_tx.clone();
        let status_tx = self.status_tx.clone();
        let callbacks = self.callbacks.clone();
        let generation = self.generation.fetch_add(1, Ordering::Relaxed);

        status_tx.send_replace(BackendConnectionStatus::Connecting);
        trigger_callbacks(&callbacks);

        self.runtime.spawn(async move {
            match connect_backend(&endpoint, generation) {
                Ok(backend) => {
                    let backend = Arc::new(backend);
                    *current_backend.lock().unwrap() = Some(backend.clone());
                    backend_tx.send_replace(Some(backend));
                    status_tx.send_replace(BackendConnectionStatus::Connected);
                }
                Err(error) => {
                    error!("failed to connect ros-z backend: {error}");
                    *current_backend.lock().unwrap() = None;
                    backend_tx.send_replace(None);
                    status_tx.send_replace(BackendConnectionStatus::Disconnected);
                }
            }
            trigger_callbacks(&callbacks);
        });
    }

    pub fn disconnect(&self) {
        *self.current_backend.lock().unwrap() = None;
        self.backend_tx.send_replace(None);
        self.status_tx.send_replace(BackendConnectionStatus::Disconnected);
        trigger_callbacks(&self.callbacks);
    }

    pub fn connection_status(&self) -> BackendConnectionStatus {
        *self.status_tx.borrow()
    }

    pub fn set_address(&self, endpoint: String) {
        *self.endpoint.lock().unwrap() = endpoint;
    }

    pub fn endpoint(&self) -> String {
        self.endpoint.lock().unwrap().clone()
    }

    pub fn topic_list_state(&self) -> TopicListState {
        let Some(backend) = self.current_backend.lock().unwrap().clone() else {
            return TopicListState {
                discovering: matches!(
                    self.connection_status(),
                    BackendConnectionStatus::Connecting
                ),
                topics: Vec::new(),
            };
        };

        let mut topics = backend
            .context_graph()
            .get_topic_names_and_types()
            .into_iter()
            .map(|(name, graph_type)| TopicDescriptor { name, graph_type })
            .collect::<Vec<_>>();
        topics.sort_by(|left, right| left.name.cmp(&right.name));

        TopicListState {
            discovering: topics.is_empty(),
            topics,
        }
    }

    pub fn config_node_list_state(&self) -> ConfigNodeListState {
        let Some(backend) = self.current_backend.lock().unwrap().clone() else {
            return ConfigNodeListState {
                discovering: matches!(
                    self.connection_status(),
                    BackendConnectionStatus::Connecting
                ),
                nodes: Vec::new(),
            };
        };

        let services = service_names(backend.context_graph());
        let nodes = config_nodes_from_services(&services);
        ConfigNodeListState {
            discovering: nodes.is_empty(),
            nodes,
        }
    }

    pub fn has_capability(&self, capability: BackendCapability) -> bool {
        matches!(
            capability,
            BackendCapability::TopicDiscovery
                | BackendCapability::DynamicInspection
                | BackendCapability::TypedSubscription
                | BackendCapability::NodeConfigRead
                | BackendCapability::NodeConfigMetadata
                | BackendCapability::NodeConfigWrite
        )
    }

    pub fn subscribe_json(&self, topic: impl Into<String>) -> BufferHandle<Value> {
        self.subscribe_buffered_json(topic, Duration::ZERO)
    }

    pub fn subscribe_buffered_json(
        &self,
        topic: impl Into<String>,
        history: Duration,
    ) -> BufferHandle<Value> {
        let topic = topic.into();
        let (buffer, handle) = Buffer::new(history);
        let mut backend_rx = self.backend_tx.subscribe();
        let callbacks = self.callbacks.clone();

        self.runtime.spawn(async move {
            subscribe_dynamic_json_loop(topic, buffer, &mut backend_rx, callbacks).await;
        });

        handle
    }

    pub fn subscribe_changes_json(&self, topic: impl Into<String>) -> ChangeBufferHandle<Value> {
        let topic = topic.into();
        let (buffer, handle) = ChangeBuffer::new();
        let mut backend_rx = self.backend_tx.subscribe();
        let callbacks = self.callbacks.clone();

        self.runtime.spawn(async move {
            subscribe_dynamic_change_loop(topic, buffer, &mut backend_rx, callbacks).await;
        });

        handle
    }

    pub fn subscribe_value<T>(&self, logical_path: impl Into<String>) -> BufferHandle<T>
    {
        self.subscribe_buffered_value(logical_path, Duration::ZERO)
    }

    pub fn subscribe_buffered_value<T>(
        &self,
        logical_path: impl Into<String>,
        history: Duration,
    ) -> BufferHandle<T> {
        let logical_path = logical_path.into();
        let (buffer, handle) = Buffer::new(history);
        buffer.push_error(eyre!(BackendError::UnmappedLogicalPath { path: logical_path }));
        handle
    }

    pub fn subscribe_topic_value<T>(
        &self,
        topic: impl Into<String>,
        history: Duration,
    ) -> BufferHandle<T>
    where
        T: ros_z::msg::ZMessage + ros_z::WithTypeInfo + Send + Sync + 'static,
        for<'de> T: serde::Deserialize<'de>,
        for<'a> <T as ros_z::msg::ZMessage>::Serdes:
            ros_z::msg::ZDeserializer<Output = T, Input<'a> = &'a [u8]>,
    {
        let topic = topic.into();
        let (buffer, handle) = Buffer::new(history);
        let backend_rx = self.backend_tx.subscribe();
        let callbacks = self.callbacks.clone();

        thread::spawn(move || {
            subscribe_typed_value_loop::<T>(topic, buffer, backend_rx, callbacks);
        });
        handle
    }

    pub fn write(&self, _path: impl Into<String>, _value: Value) -> BackendResult<()> {
        Err(BackendError::UnsupportedCapability { operation: "write" })
    }

    pub fn on_change(&self, callback: impl Fn() + Send + Sync + 'static) {
        self.callbacks.lock().unwrap().push(Arc::new(callback));
    }

    pub fn get_config_snapshot(
        &self,
        selector: &str,
    ) -> BackendResult<GetNodeConfigSnapshotResponse> {
        let client = self.config_client(selector)?;
        self.runtime
            .block_on(client.get_snapshot())
            .map_err(|error| BackendError::Operation {
                operation: "config.get_snapshot",
                message: error.to_string(),
            })
    }

    pub fn get_config_value(
        &self,
        selector: &str,
        path: &str,
    ) -> BackendResult<GetNodeConfigValueResponse> {
        let client = self.config_client(selector)?;
        self.runtime
            .block_on(client.get_value(path))
            .map_err(|error| BackendError::Operation {
                operation: "config.get_value",
                message: error.to_string(),
            })
    }

    pub fn list_config_paths(
        &self,
        selector: &str,
        writable_only: bool,
    ) -> BackendResult<ListNodeConfigPathsResponse> {
        let client = self.config_client(selector)?;
        self.runtime
            .block_on(client.list_paths(Vec::new(), 0, writable_only))
            .map_err(|error| BackendError::Operation {
                operation: "config.list_paths",
                message: error.to_string(),
            })
    }

    pub fn get_config_metadata(
        &self,
        selector: &str,
        paths: Vec<String>,
    ) -> BackendResult<GetNodeConfigMetadataResponse> {
        let client = self.config_client(selector)?;
        self.runtime
            .block_on(client.get_metadata(paths))
            .map_err(|error| BackendError::Operation {
                operation: "config.get_metadata",
                message: error.to_string(),
            })
    }

    pub fn set_config_json(
        &self,
        selector: &str,
        path: &str,
        value: &Value,
        scope: ConfigScope,
        expected_revision: Option<u64>,
    ) -> BackendResult<SetNodeConfigResponse> {
        let client = self.config_client(selector)?;
        self.runtime
            .block_on(client.set_json(path, value, scope, expected_revision))
            .map_err(|error| BackendError::Operation {
                operation: "config.set_json",
                message: error.to_string(),
            })
    }

    pub fn reset_config(
        &self,
        selector: &str,
        path: &str,
        scope: ConfigScope,
        expected_revision: Option<u64>,
    ) -> BackendResult<ResetNodeConfigResponse> {
        let client = self.config_client(selector)?;
        self.runtime
            .block_on(client.reset(path, scope, expected_revision))
            .map_err(|error| BackendError::Operation {
                operation: "config.reset",
                message: error.to_string(),
            })
    }

    fn config_client(&self, selector: &str) -> BackendResult<RemoteConfigClient> {
        let backend = self
            .current_backend
            .lock()
            .unwrap()
            .clone()
            .ok_or(BackendError::NotConnected)?;
        let services = service_names(backend.context_graph());
        let node_fqn = resolve_config_node_selector(&services, selector)?;
        RemoteConfigClient::new(backend.node.clone(), node_fqn).map_err(|error| {
            BackendError::Operation {
                operation: "config.client",
                message: error.to_string(),
            }
        })
    }
}

impl ConnectedBackend {
    fn context_graph(&self) -> &Graph {
        self._context.graph().as_ref()
    }
}

fn connect_backend(endpoint: &str, generation: u64) -> color_eyre::Result<ConnectedBackend> {
    let context = Arc::new(
        ZContextBuilder::default()
            .with_router_endpoint(endpoint.to_string())
            .map_err(|error| eyre!(error.to_string()))?
            .build()
            .map_err(|error| eyre!(error.to_string()))?,
    );
    let node = Arc::new(
        context
            .create_node("twix")
            .build()
            .map_err(|error| eyre!(error.to_string()))?,
    );
    Ok(ConnectedBackend {
        generation,
        _context: context,
        node,
    })
}

async fn subscribe_dynamic_json_loop(
    topic: String,
    buffer: Buffer<Value, color_eyre::Report>,
    backend_rx: &mut watch::Receiver<Option<Arc<ConnectedBackend>>>,
    callbacks: Arc<Mutex<Vec<ChangeCallback>>>,
) {
    loop {
        let Some(backend) = wait_for_backend(backend_rx).await else {
            return;
        };
        let generation = backend.generation;
        let subscriber = match backend
            .node
            .create_dyn_sub_auto(&topic, DYNAMIC_SCHEMA_DISCOVERY_TIMEOUT)
            .await
            .map_err(|error| BackendError::Operation {
                operation: "dynamic.subscribe",
                message: error.to_string(),
            })
            .and_then(|builder| {
                builder.build().map_err(|error| BackendError::Operation {
                    operation: "dynamic.subscribe",
                    message: error.to_string(),
                })
            })
        {
            Ok(subscriber) => subscriber,
            Err(error) => {
                buffer.push_error(eyre!(error));
                tokio::time::sleep(Duration::from_secs(1)).await;
                continue;
            }
        };

        loop {
            tokio::select! {
                changed = backend_rx.changed() => {
                    if changed.is_err() {
                        return;
                    }
                    if backend_rx.borrow().as_ref().map(|backend| backend.generation) != Some(generation) {
                        break;
                    }
                }
                result = subscriber.async_recv_with_metadata() => {
                    match result {
                        Ok(received) => {
                            if let Some(datum) = dynamic_received_to_datum(received) {
                                buffer.push(datum).await;
                                trigger_callbacks(&callbacks);
                            }
                        }
                        Err(error) => {
                            buffer.push_error(eyre!(BackendError::Operation {
                                operation: "dynamic.recv",
                                message: error.to_string(),
                            }));
                            break;
                        }
                    }
                }
            }
        }
    }
}

async fn subscribe_dynamic_change_loop(
    topic: String,
    buffer: ChangeBuffer<Value, color_eyre::Report>,
    backend_rx: &mut watch::Receiver<Option<Arc<ConnectedBackend>>>,
    callbacks: Arc<Mutex<Vec<ChangeCallback>>>,
) {
    loop {
        let Some(backend) = wait_for_backend(backend_rx).await else {
            return;
        };
        let generation = backend.generation;
        let subscriber = match backend
            .node
            .create_dyn_sub_auto(&topic, DYNAMIC_SCHEMA_DISCOVERY_TIMEOUT)
            .await
            .map_err(|error| BackendError::Operation {
                operation: "dynamic.subscribe",
                message: error.to_string(),
            })
            .and_then(|builder| {
                builder.build().map_err(|error| BackendError::Operation {
                    operation: "dynamic.subscribe",
                    message: error.to_string(),
                })
            })
        {
            Ok(subscriber) => subscriber,
            Err(error) => {
                buffer.push_error(eyre!(error));
                tokio::time::sleep(Duration::from_secs(1)).await;
                continue;
            }
        };

        loop {
            tokio::select! {
                changed = backend_rx.changed() => {
                    if changed.is_err() {
                        return;
                    }
                    if backend_rx.borrow().as_ref().map(|backend| backend.generation) != Some(generation) {
                        break;
                    }
                }
                result = subscriber.async_recv_with_metadata() => {
                    match result {
                        Ok(received) => {
                            if let Some(change) = dynamic_received_to_change(received) {
                                buffer.push(change);
                                trigger_callbacks(&callbacks);
                            }
                        }
                        Err(error) => {
                            buffer.push_error(eyre!(BackendError::Operation {
                                operation: "dynamic.recv",
                                message: error.to_string(),
                            }));
                            break;
                        }
                    }
                }
            }
        }
    }
}

fn dynamic_received_to_datum(received: Received<DynamicMessage>) -> Option<Datum<Value>> {
    let timestamp = received.transport_time.or(received.source_time)?;
    Some(Datum {
        timestamp: twix_time(timestamp),
        source_timestamp: received.source_time.map(twix_time),
        value: dynamic_message_to_json(&received.message),
    })
}

fn dynamic_received_to_change(received: Received<DynamicMessage>) -> Option<Change<Value>> {
    let timestamp = received.transport_time.or(received.source_time)?;
    Some(Change {
        timestamp: twix_time(timestamp),
        source_timestamp: received.source_time.map(twix_time),
        value: dynamic_message_to_json(&received.message),
    })
}

fn received_to_datum<T>(received: Received<T>) -> Option<Datum<T>> {
    let timestamp = received.transport_time.or(received.source_time)?;
    Some(Datum {
        timestamp: twix_time(timestamp),
        source_timestamp: received.source_time.map(twix_time),
        value: received.message,
    })
}

fn twix_time(time: ZTime) -> TwixTime {
    TwixTime::from_nanos(time.as_nanos())
}

fn subscribe_typed_value_loop<T>(
    topic: String,
    buffer: Buffer<T, color_eyre::Report>,
    backend_rx: watch::Receiver<Option<Arc<ConnectedBackend>>>,
    callbacks: Arc<Mutex<Vec<ChangeCallback>>>,
) where
    T: ros_z::msg::ZMessage + ros_z::WithTypeInfo + Send + Sync + 'static,
    for<'de> T: serde::Deserialize<'de>,
    for<'a> <T as ros_z::msg::ZMessage>::Serdes:
        ros_z::msg::ZDeserializer<Output = T, Input<'a> = &'a [u8]>,
{
    loop {
        let Some(backend) = backend_rx.borrow().clone() else {
            thread::sleep(Duration::from_millis(100));
            continue;
        };
        let generation = backend.generation;
        let subscriber = match backend.node.create_sub::<T>(&topic).build() {
            Ok(subscriber) => subscriber,
            Err(error) => {
                buffer.push_error(eyre!(BackendError::Operation {
                    operation: "typed.subscribe",
                    message: error.to_string(),
                }));
                thread::sleep(Duration::from_secs(1));
                continue;
            }
        };

        loop {
            if backend_rx.borrow().as_ref().map(|backend| backend.generation) != Some(generation) {
                break;
            }
            match subscriber.recv_timeout_with_metadata(Duration::from_millis(200)) {
                Ok(received) => {
                    if let Some(datum) = received_to_datum(received) {
                        buffer.blocking_push(datum);
                        trigger_callbacks(&callbacks);
                    }
                }
                Err(error) if error.to_string().contains("timed out") => continue,
                Err(error) => {
                    buffer.push_error(eyre!(BackendError::Operation {
                        operation: "typed.recv",
                        message: error.to_string(),
                    }));
                    break;
                }
            }
        }
    }
}

async fn wait_for_backend(
    backend_rx: &mut watch::Receiver<Option<Arc<ConnectedBackend>>>,
) -> Option<Arc<ConnectedBackend>> {
    loop {
        if let Some(backend) = backend_rx.borrow().clone() {
            return Some(backend);
        }
        if backend_rx.changed().await.is_err() {
            return None;
        }
    }
}

fn trigger_callbacks(callbacks: &Arc<Mutex<Vec<ChangeCallback>>>) {
    let callbacks = callbacks.lock().unwrap().clone();
    for callback in callbacks {
        callback();
    }
}

fn service_names(graph: &Graph) -> BTreeSet<String> {
    graph
        .get_service_names_and_types()
        .into_iter()
        .map(|(name, _)| name)
        .collect()
}

fn config_nodes_from_services(services: &BTreeSet<String>) -> Vec<ConfigNodeDescriptor> {
    const SNAPSHOT_SUFFIX: &str = "/config/get_snapshot";
    services
        .iter()
        .filter_map(|service| {
            let node_fqn = service.strip_suffix(SNAPSHOT_SUFFIX)?;
            Some(ConfigNodeDescriptor {
                node_fqn: node_fqn.to_string(),
                metadata_capable: has_config_metadata(services, node_fqn),
            })
        })
        .collect()
}

fn resolve_config_node_selector(
    services: &BTreeSet<String>,
    selector: &str,
) -> BackendResult<String> {
    let nodes = config_nodes_from_services(services)
        .into_iter()
        .map(|node| node.node_fqn)
        .collect::<Vec<_>>();
    if selector.starts_with('/') {
        return nodes
            .into_iter()
            .find(|node_fqn| node_fqn == selector)
            .ok_or_else(|| BackendError::Operation {
                operation: "config.resolve_node",
                message: format!("node not found: {selector}"),
            });
    }

    let matches = nodes
        .into_iter()
        .filter(|node_fqn| node_fqn.rsplit('/').next().is_some_and(|name| name == selector))
        .collect::<Vec<_>>();
    match matches.as_slice() {
        [] => Err(BackendError::Operation {
            operation: "config.resolve_node",
            message: format!("node not found: {selector}"),
        }),
        [node_fqn] => Ok(node_fqn.clone()),
        _ => Err(BackendError::Operation {
            operation: "config.resolve_node",
            message: format!("node name '{selector}' is ambiguous: {}", matches.join(", ")),
        }),
    }
}

fn has_config_metadata(services: &BTreeSet<String>, node_fqn: &str) -> bool {
    let list_paths = format!("{node_fqn}/config/list_paths");
    let get_metadata = format!("{node_fqn}/config/get_metadata");
    services.contains(&list_paths) && services.contains(&get_metadata)
}
