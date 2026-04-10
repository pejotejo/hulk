use std::{
    collections::BTreeSet,
    path::PathBuf,
    sync::{Arc, OnceLock},
};

use arc_swap::ArcSwap;
use parking_lot::Mutex;
use ros_z::node::ZNode;
use serde::{Serialize, de::DeserializeOwned};
use serde_json::Value;
use tokio::sync::watch;

use crate::{
    ConfigError, ConfigFieldMetadata, ConfigKey, ConfigMetadata, ConfigTimestamp, FieldPath,
    LayerPath, NodeConfigSnapshot, Result,
    loader::load_json5_object_or_empty,
    merge::{
        RecursiveDiffEntry, get_value_at_path as get_from_value, merge_layers, provenance_for_path,
        recursive_diff, remove_value_at_path, set_value_at_path,
    },
    persistence::write_pretty_json,
    remote::{
        RemoteConfigServices,
        types::{NodeConfigChange, NodeConfigChangeSource, NodeConfigEvent},
    },
};

/// Validation hook run against the fully typed candidate config before commit.
pub type ValidateHook<T> = Arc<dyn Fn(&T) -> std::result::Result<(), String> + Send + Sync>;

/// One JSON write operation used by [`NodeConfig::set_json_atomically`].
#[derive(Debug, Clone)]
pub struct ConfigJsonWrite {
    pub path: FieldPath,
    pub value: Value,
    pub target_layer: LayerPath,
}

/// Typed node-local config handle.
///
/// A `NodeConfig<T>` is cheap to clone and can be shared across tasks. It owns
/// the runtime APIs for reading snapshots, applying JSON writes, reloading from
/// disk, and optionally exposing metadata for metadata-enabled bindings.
#[derive(Clone)]
pub struct NodeConfig<T> {
    pub(crate) inner: Arc<NodeConfigInner<T>>,
}

pub struct NodeConfigInner<T> {
    pub(crate) node_fqn: String,
    pub(crate) config_key: ConfigKey,
    pub(crate) layers: Vec<PathBuf>,
    clock: ros_z::time::ZClock,
    commit_lock: Mutex<()>,
    hooks: Mutex<Vec<ValidateHook<T>>>,
    current: ArcSwap<NodeConfigSnapshot<T>>,
    tx: watch::Sender<Arc<NodeConfigSnapshot<T>>>,
    binding_state: Arc<parking_lot::Mutex<bool>>,
    metadata: Option<Arc<Vec<ConfigFieldMetadata>>>,
    remote: OnceLock<RemoteConfigServices<T>>,
}

impl<T> Drop for NodeConfigInner<T> {
    fn drop(&mut self) {
        *self.binding_state.lock() = false;
    }
}

/// Extension methods that bind typed config to a `ros-z` node.
pub trait NodeConfigExt {
    /// Bind a typed config handle to the node using `config_key` as the
    /// filename stem inside the active config layers.
    ///
    /// This is provided by `ros_z_config::prelude::*`, not as an inherent
    /// method on `ros_z::node::ZNode`.
    fn bind_config_as<T>(&self, config_key: impl Into<ConfigKey>) -> Result<NodeConfig<T>>
    where
        T: Serialize + DeserializeOwned + Send + Sync + 'static;

    /// Bind a typed config handle with metadata support enabled using
    /// `config_key` as the filename stem inside the active config layers.
    ///
    /// This requires `T` to implement [`crate::ConfigMetadata`], typically via
    /// `#[derive(ros_z_config::ConfigMetadata)]`.
    fn bind_config_with_metadata_as<T>(
        &self,
        config_key: impl Into<ConfigKey>,
    ) -> Result<NodeConfig<T>>
    where
        T: Serialize + DeserializeOwned + Send + Sync + ConfigMetadata + 'static;
}

impl NodeConfigExt for ZNode {
    fn bind_config_as<T>(&self, config_key: impl Into<ConfigKey>) -> Result<NodeConfig<T>>
    where
        T: Serialize + DeserializeOwned + Send + Sync + 'static,
    {
        let config_key = config_key.into();
        validate_config_key(&config_key)?;
        let mut bound = self.config_binding_state().lock();
        if *bound {
            return Err(ConfigError::AlreadyBound {
                node_fqn: node_fqn(self),
            });
        }
        *bound = true;

        match bind_config_inner(self, config_key) {
            Ok(config) => Ok(config),
            Err(err) => {
                *bound = false;
                Err(err)
            }
        }
    }

    fn bind_config_with_metadata_as<T>(
        &self,
        config_key: impl Into<ConfigKey>,
    ) -> Result<NodeConfig<T>>
    where
        T: Serialize + DeserializeOwned + Send + Sync + ConfigMetadata + 'static,
    {
        let config_key = config_key.into();
        validate_config_key(&config_key)?;
        let mut bound = self.config_binding_state().lock();
        if *bound {
            return Err(ConfigError::AlreadyBound {
                node_fqn: node_fqn(self),
            });
        }
        *bound = true;

        match bind_config_inner_with_metadata(
            self,
            config_key,
            Some(Arc::new(T::config_metadata())),
        ) {
            Ok(config) => Ok(config),
            Err(err) => {
                *bound = false;
                Err(err)
            }
        }
    }
}

fn bind_config_inner<T>(node: &ZNode, config_key: ConfigKey) -> Result<NodeConfig<T>>
where
    T: Serialize + DeserializeOwned + Send + Sync + 'static,
{
    bind_config_inner_with_metadata(node, config_key, None)
}

fn bind_config_inner_with_metadata<T>(
    node: &ZNode,
    config_key: ConfigKey,
    metadata: Option<Arc<Vec<ConfigFieldMetadata>>>,
) -> Result<NodeConfig<T>>
where
    T: Serialize + DeserializeOwned + Send + Sync + 'static,
{
    let layers = node.runtime_config_inputs().config_layers.clone();
    if layers.is_empty() {
        return Err(ConfigError::EmptyLayerList);
    }

    let node_fqn = node_fqn(node);
    let snapshot = load_snapshot::<T>(&node_fqn, &config_key, &layers, node.clock(), 0)?;
    let snapshot = Arc::new(snapshot);
    let (tx, _rx) = watch::channel(snapshot.clone());

    let binding_state = self_binding_state(node);
    let current = ArcSwap::from(snapshot);
    let inner = Arc::new(NodeConfigInner {
        node_fqn: node_fqn.clone(),
        config_key,
        layers,
        clock: node.clock().clone(),
        commit_lock: Mutex::new(()),
        hooks: Mutex::new(Vec::new()),
        current,
        tx,
        binding_state,
        metadata: metadata.clone(),
        remote: OnceLock::new(),
    });

    let remote = RemoteConfigServices::register(node, inner.clone(), metadata)?;
    let _ = inner.remote.set(remote);

    Ok(NodeConfig { inner })
}

fn self_binding_state(node: &ZNode) -> Arc<parking_lot::Mutex<bool>> {
    node.config_binding_state().clone()
}

fn node_fqn(node: &ZNode) -> String {
    if node.namespace().is_empty() || node.namespace() == "/" {
        format!("/{}", node.name())
    } else {
        format!(
            "/{}/{}",
            node.namespace().trim_start_matches('/'),
            node.name()
        )
    }
}

fn validate_config_key(config_key: &str) -> Result<()> {
    if config_key.is_empty()
        || !config_key
            .chars()
            .all(|ch| ch.is_ascii_alphanumeric() || ch == '_' || ch == '-')
    {
        return Err(ConfigError::InvalidConfigKey {
            key: config_key.to_string(),
        });
    }

    Ok(())
}

fn layer_path(path: &std::path::Path) -> LayerPath {
    path.to_string_lossy().into_owned()
}

fn load_snapshot<T>(
    node_fqn: &str,
    config_key: &str,
    layers: &[PathBuf],
    clock: &ros_z::time::ZClock,
    revision: u64,
) -> Result<NodeConfigSnapshot<T>>
where
    T: Serialize + DeserializeOwned + Send + Sync + 'static,
{
    let layer_overlays = layers
        .iter()
        .map(|layer| load_json5_object_or_empty(&layer.join(format!("{config_key}.json5"))))
        .collect::<Result<Vec<_>>>()?;

    snapshot_from_parts(
        node_fqn,
        config_key,
        layers,
        clock,
        revision,
        layer_overlays,
    )
}

fn snapshot_from_parts<T>(
    node_fqn: &str,
    config_key: &str,
    layers: &[PathBuf],
    clock: &ros_z::time::ZClock,
    revision: u64,
    layer_overlays: Vec<Value>,
) -> Result<NodeConfigSnapshot<T>>
where
    T: Serialize + DeserializeOwned + Send + Sync + 'static,
{
    let layers = layers
        .iter()
        .map(|path| layer_path(path))
        .collect::<Vec<_>>();
    let merged = merge_layers(
        &layers
            .iter()
            .cloned()
            .zip(layer_overlays.iter().cloned())
            .collect::<Vec<_>>(),
    )?;
    let typed: T = serde_json::from_value(merged.effective.clone()).map_err(|err| {
        ConfigError::DeserializationError {
            message: err.to_string(),
        }
    })?;

    Ok(NodeConfigSnapshot {
        node_fqn: node_fqn.to_string(),
        config_key: config_key.to_string(),
        typed: Arc::new(typed),
        effective: merged.effective,
        layers,
        layer_overlays,
        provenance: Arc::new(merged.provenance),
        revision,
        committed_at: ConfigTimestamp::now_from(clock),
    })
}

impl<T> NodeConfig<T>
where
    T: Serialize + DeserializeOwned + Send + Sync + 'static,
{
    /// Return the latest committed snapshot for this node.
    pub fn snapshot(&self) -> Arc<NodeConfigSnapshot<T>> {
        self.inner.current.load_full()
    }

    /// Read the effective JSON value at `path`.
    pub fn get_json(&self, path: &str) -> Result<Value> {
        get_from_value(&self.snapshot().effective, path)?.ok_or_else(|| ConfigError::PathError {
            path: path.to_string(),
            reason: "path not found".to_string(),
        })
    }

    /// Set one path in one target layer using a JSON value.
    pub fn set_json(
        &self,
        path: &str,
        value: Value,
        target_layer: impl Into<LayerPath>,
    ) -> Result<()> {
        self.commit(
            &[ConfigJsonWrite {
                path: path.to_string(),
                value,
                target_layer: target_layer.into(),
            }],
            &[],
            None,
            NodeConfigChangeSource::LocalWrite,
        )
        .map(|_| ())
    }

    /// Apply several JSON writes atomically.
    ///
    /// If `expected_revision` is provided, the commit fails unless it matches
    /// the current snapshot revision.
    pub fn set_json_atomically(
        &self,
        changes: Vec<ConfigJsonWrite>,
        expected_revision: Option<u64>,
    ) -> Result<()> {
        self.commit(
            &changes,
            &[],
            expected_revision,
            NodeConfigChangeSource::LocalWrite,
        )
        .map(|_| ())
    }

    /// Remove one layer-local override.
    ///
    /// Reset removes the key from the target layer overlay rather than writing
    /// `null`. If the key is absent in the target layer, reset succeeds as a
    /// no-op.
    pub fn reset(&self, path: &str, target_layer: impl Into<LayerPath>) -> Result<()> {
        self.commit(
            &[],
            &[(path.to_string(), target_layer.into())],
            None,
            NodeConfigChangeSource::LocalWrite,
        )
        .map(|_| ())
    }

    /// Reload overlays from disk and attempt a new commit.
    pub fn reload(&self) -> Result<()> {
        self.reload_with_source(NodeConfigChangeSource::Reload)
            .map(|_| ())
    }

    pub(crate) fn reload_with_source(
        &self,
        source: NodeConfigChangeSource,
    ) -> Result<CommitOutcome> {
        let _commit_guard = self.inner.commit_lock.lock();
        let current = self.snapshot();

        let candidate = load_snapshot::<T>(
            &self.inner.node_fqn,
            &self.inner.config_key,
            &self.inner.layers,
            &self.inner.clock,
            current.revision + 1,
        )?;
        self.run_hooks(candidate.typed.as_ref())?;
        let diff = recursive_diff(&current.effective, &candidate.effective);
        let changed_paths = diff.iter().map(|entry| entry.path.clone()).collect();
        let snapshot = Arc::new(candidate);
        self.inner.current.store(snapshot.clone());
        let _ = self.inner.tx.send(snapshot.clone());
        if let Err(err) = self.publish_event(&current, &snapshot, diff, source) {
            tracing::warn!("[CFG] Failed to publish config event: {err}");
        }
        Ok(CommitOutcome {
            committed_revision: snapshot.revision,
            changed_paths,
        })
    }

    /// Subscribe to the latest committed snapshot using watch semantics.
    pub fn subscribe(&self) -> crate::ConfigSubscription<T> {
        self.inner.tx.subscribe()
    }

    /// List metadata-backed field paths for a metadata-enabled binding.
    pub fn list_paths(&self) -> Result<Vec<String>> {
        let metadata =
            self.inner
                .metadata
                .as_ref()
                .ok_or_else(|| ConfigError::ValidationError {
                    message: "metadata support not enabled for this binding".to_string(),
                })?;
        Ok(metadata.iter().map(|field| field.path.clone()).collect())
    }

    /// Return metadata for one field path.
    pub fn get_metadata(&self, path: &str) -> Result<ConfigFieldMetadata> {
        let metadata =
            self.inner
                .metadata
                .as_ref()
                .ok_or_else(|| ConfigError::ValidationError {
                    message: "metadata support not enabled for this binding".to_string(),
                })?;
        metadata
            .iter()
            .find(|field| field.path == path)
            .cloned()
            .ok_or_else(|| ConfigError::PathError {
                path: path.to_string(),
                reason: "metadata path not found".to_string(),
            })
    }

    /// Register a validation hook.
    ///
    /// Adding a hook validates the current committed snapshot immediately. If
    /// the current snapshot violates the new hook, registration fails and the
    /// hook is not installed.
    pub fn add_validation_hook(&self, hook: ValidateHook<T>) -> Result<()> {
        let _commit_guard = self.inner.commit_lock.lock();
        self.run_hook(self.snapshot().typed.as_ref(), &hook)?;
        self.inner.hooks.lock().push(hook);
        Ok(())
    }

    fn run_hooks(&self, candidate: &T) -> Result<()> {
        for hook in self.inner.hooks.lock().iter() {
            self.run_hook(candidate, hook)?;
        }
        Ok(())
    }

    fn run_hook(&self, candidate: &T, hook: &ValidateHook<T>) -> Result<()> {
        hook(candidate).map_err(|message| ConfigError::ValidationError { message })
    }

    pub(crate) fn commit(
        &self,
        writes: &[ConfigJsonWrite],
        resets: &[(FieldPath, LayerPath)],
        expected_revision: Option<u64>,
        source: NodeConfigChangeSource,
    ) -> Result<CommitOutcome> {
        let _commit_guard = self.inner.commit_lock.lock();
        let current = self.snapshot();
        if let Some(expected) = expected_revision
            && expected != current.revision
        {
            return Err(ConfigError::RevisionMismatch {
                expected,
                actual: current.revision,
            });
        }

        let mut layer_overlays = current.layer_overlays.clone();
        let active_layers = self
            .inner
            .layers
            .iter()
            .map(|path| layer_path(path))
            .collect::<Vec<_>>();
        let mut touched = BTreeSet::new();

        for write in writes {
            let index = active_layers
                .iter()
                .position(|layer| layer == &write.target_layer)
                .ok_or_else(|| ConfigError::LayerNotActive {
                    layer: write.target_layer.clone(),
                })?;
            let overlay = &mut layer_overlays[index];
            set_value_at_path(overlay, &write.path, write.value.clone())?;
            touched.insert(index);
        }

        for (path, target_layer) in resets {
            let index = active_layers
                .iter()
                .position(|layer| layer == target_layer)
                .ok_or_else(|| ConfigError::LayerNotActive {
                    layer: target_layer.clone(),
                })?;
            let overlay = &mut layer_overlays[index];
            if remove_value_at_path(overlay, path)? {
                touched.insert(index);
            }
        }

        let candidate = snapshot_from_parts::<T>(
            &self.inner.node_fqn,
            &self.inner.config_key,
            &self.inner.layers,
            &self.inner.clock,
            current.revision + 1,
            layer_overlays,
        )?;
        self.run_hooks(candidate.typed.as_ref())?;

        for index in &touched {
            let path = self.inner.layers[*index].join(format!("{}.json5", self.inner.config_key));
            let value = &candidate.layer_overlays[*index];
            write_pretty_json(&path, value)?;
        }

        let diff = recursive_diff(&current.effective, &candidate.effective);
        let changed_paths = diff.iter().map(|entry| entry.path.clone()).collect();
        let snapshot = Arc::new(candidate);
        self.inner.current.store(snapshot.clone());
        let _ = self.inner.tx.send(snapshot.clone());
        if let Err(err) = self.publish_event(&current, &snapshot, diff, source) {
            tracing::warn!("[CFG] Failed to publish config event: {err}");
        }

        Ok(CommitOutcome {
            committed_revision: snapshot.revision,
            changed_paths,
        })
    }

    fn publish_event(
        &self,
        previous: &Arc<NodeConfigSnapshot<T>>,
        current: &Arc<NodeConfigSnapshot<T>>,
        diff: Vec<RecursiveDiffEntry>,
        source: NodeConfigChangeSource,
    ) -> Result<()> {
        let changes = diff
            .into_iter()
            .map(|entry| NodeConfigChange {
                effective_source_layer: provenance_for_path(&current.provenance, &entry.path)
                    .unwrap_or_default(),
                path: entry.path,
                old_value_json: serde_json::to_string(&entry.old_value)
                    .unwrap_or_else(|_| "null".to_string()),
                new_value_json: serde_json::to_string(&entry.new_value)
                    .unwrap_or_else(|_| "null".to_string()),
            })
            .collect::<Vec<_>>();

        let event = NodeConfigEvent {
            node_fqn: self.inner.node_fqn.clone(),
            config_key: self.inner.config_key.clone(),
            previous_revision: previous.revision,
            revision: current.revision,
            source,
            changed_paths: changes.iter().map(|change| change.path.clone()).collect(),
            changes,
        };
        if let Some(remote) = self.inner.remote.get() {
            remote.publish_event(&event)?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone)]
pub(crate) struct CommitOutcome {
    pub committed_revision: u64,
    pub changed_paths: Vec<String>,
}

impl<T> std::fmt::Debug for NodeConfig<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NodeConfig")
            .field("node_fqn", &self.inner.node_fqn)
            .field("config_key", &self.inner.config_key)
            .finish_non_exhaustive()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn commit_outcome_is_debuggable() {
        let outcome = CommitOutcome {
            committed_revision: 1,
            changed_paths: vec!["a".into()],
        };
        let text = format!("{outcome:?}");
        assert!(text.contains("committed_revision"));
    }
}
