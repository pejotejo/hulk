use std::sync::Arc;

use serde::{Deserialize, Serialize};
use serde_json::Value;
use tokio::sync::watch;

use crate::{ConfigKey, LayerPath, ProvenanceMap};

/// Transport-friendly commit timestamp.
///
/// This value is recorded on the node's active [`ros_z::time::ZClock`]
/// timeline. When the node uses a logical clock, `sec`/`nanosec` represent a
/// logical instant rather than host wallclock time.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct ConfigTimestamp {
    pub sec: i64,
    pub nanosec: u32,
}

impl ConfigTimestamp {
    /// Build a timestamp from a `ros-z` clock.
    ///
    /// The resulting timestamp stays on the clock's current timeline instead of
    /// forcing a wallclock interpretation.
    pub fn now_from(clock: &ros_z::time::ZClock) -> Self {
        let now = clock.now().as_nanos();
        let sec = now.div_euclid(1_000_000_000);
        let nanosec = now.rem_euclid(1_000_000_000) as u32;
        Self { sec, nanosec }
    }
}

/// Immutable committed config state for one node.
///
/// Readers receive snapshots through [`crate::NodeConfig::snapshot`] and
/// [`crate::ConfigSubscription`]. Each snapshot contains both the typed config
/// value and the raw effective/overlay JSON trees used for operator-facing
/// introspection and debugging.
///
/// `committed_at` is expressed on the node's active clock timeline.
#[derive(Debug, Clone)]
pub struct NodeConfigSnapshot<T> {
    pub node_fqn: String,
    pub config_key: ConfigKey,
    pub typed: Arc<T>,
    pub effective: Value,
    pub layers: Vec<LayerPath>,
    pub layer_overlays: Vec<Value>,
    pub provenance: Arc<ProvenanceMap>,
    pub revision: u64,
    pub committed_at: ConfigTimestamp,
}

impl<T> NodeConfigSnapshot<T> {
    /// Return the typed config value.
    pub fn typed(&self) -> &T {
        self.typed.as_ref()
    }

    /// Return the layer that currently contributes the effective value at `path`.
    pub fn effective_source_layer(&self, path: &str) -> Option<LayerPath> {
        self.provenance.get(path).cloned()
    }
}

/// Watch-style subscription to the latest committed snapshot.
///
/// New subscribers immediately see the current snapshot. Slow subscribers may
/// skip intermediate revisions, which makes this suitable for "latest state"
/// consumers rather than audit-style event processing.
pub type ConfigSubscription<T> = watch::Receiver<Arc<NodeConfigSnapshot<T>>>;
