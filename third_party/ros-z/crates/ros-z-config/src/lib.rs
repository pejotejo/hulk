//! Rust-first node-local configuration for `ros-z`.
//!
//! `ros-z-config` provides a node-local configuration subsystem intended for
//! native Rust applications built on top of `ros-z`.
//!
//! Core model:
//!
//! - each `ZNode` owns one config space
//! - process-wide startup selects an ordered list of config layers
//! - config files are JSON5 on read and pretty JSON on writeback
//! - later config layers override earlier ones
//! - application code reads typed Rust config snapshots
//! - runtime mutation happens through JSON values addressed by dot-separated paths
//!
//! To use the binding methods, import the extension trait prelude:
//!
//! ```no_run
//! use ros_z::{Builder, context::ZContextBuilder};
//! use ros_z_config::prelude::*;
//! use serde::{Deserialize, Serialize};
//!
//! #[derive(Debug, Clone, Serialize, Deserialize)]
//! #[serde(deny_unknown_fields)]
//! struct VisionConfig {
//!     enabled: bool,
//!     threshold: f64,
//! }
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
//! let ctx = ZContextBuilder::default()
//!     .with_config_layers([
//!         "./config/base",
//!         "./config/location/lab-a",
//!         "./config/robot/robot-01",
//!     ])
//!     .build()?;
//!
//! let node = ctx
//!     .create_node("ball_detector")
//!     .with_namespace("vision")
//!     .build()?;
//!
//! let config = node.bind_config_as::<VisionConfig>("ball_detector")?;
//! let snapshot = config.snapshot();
//! let cfg = snapshot.typed();
//! assert!(cfg.enabled);
//! # Ok(())
//! # }
//! ```
//!
//! Runtime writes use JSON values and are validated by deserializing the merged
//! candidate config back into `T`:
//!
//! ```no_run
//! use ros_z::{Builder, context::ZContextBuilder};
//! use ros_z_config::{ConfigJsonWrite, prelude::*};
//! use serde::{Deserialize, Serialize};
//!
//! #[derive(Debug, Clone, Serialize, Deserialize)]
//! #[serde(deny_unknown_fields)]
//! struct VisionConfig {
//!     enabled: bool,
//!     threshold: f64,
//! }
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
//! let ctx = ZContextBuilder::default()
//!     .with_config_layers([
//!         "./config/base",
//!         "./config/location/lab-a",
//!         "./config/robot/robot-01",
//!     ])
//!     .build()?;
//! let node = ctx.create_node("ball_detector").with_namespace("vision").build()?;
//! let config = node.bind_config_as::<VisionConfig>("ball_detector")?;
//!
//! config.set_json("threshold", serde_json::json!(0.72), "./config/robot/robot-01")?;
//! config.set_json_atomically(
//!     vec![ConfigJsonWrite {
//!         path: "enabled".into(),
//!         value: serde_json::json!(true),
//!         target_layer: "./config/robot/robot-01".into(),
//!     }],
//!     Some(config.snapshot().revision),
//! )?;
//! config.reset("threshold", "./config/robot/robot-01")?;
//! # Ok(())
//! # }
//! ```
//!
//! Local consumers can watch the latest committed snapshot with watch-style
//! semantics:
//!
//! ```no_run
//! use ros_z::{Builder, context::ZContextBuilder};
//! use ros_z_config::prelude::*;
//! use serde::{Deserialize, Serialize};
//!
//! #[derive(Debug, Clone, Serialize, Deserialize)]
//! #[serde(deny_unknown_fields)]
//! struct VisionConfig {
//!     threshold: f64,
//! }
//!
//! # async fn demo() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
//! let ctx = ZContextBuilder::default()
//!     .with_config_layers([
//!         "./config/base",
//!         "./config/location/lab-a",
//!         "./config/robot/robot-01",
//!     ])
//!     .build()?;
//! let node = ctx.create_node("ball_detector").with_namespace("vision").build()?;
//! let config = node.bind_config_as::<VisionConfig>("ball_detector")?;
//! let mut updates = config.subscribe();
//! updates.changed().await?;
//! let snapshot = updates.borrow().clone();
//! let _threshold = snapshot.typed().threshold;
//! # Ok(())
//! # }
//! ```
//!
//! Metadata support is opt-in. Use `bind_config_with_metadata_as::<T>(...)` with a
//! type deriving [`ConfigMetadata`] when you want local and remote metadata
//! introspection.
//!
//! ```no_run
//! use ros_z::{Builder, context::ZContextBuilder};
//! use ros_z_config::prelude::*;
//! use serde::{Deserialize, Serialize};
//!
//! #[derive(Debug, Clone, Serialize, Deserialize, ros_z_config::ConfigMetadata)]
//! #[serde(deny_unknown_fields)]
//! struct WalkConfig {
//!     #[config(doc = "Forward speed", min = -1.0, max = 1.0)]
//!     linear_x: f64,
//! }
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
//! let ctx = ZContextBuilder::default()
//!     .with_config_layers([
//!         "./config/base",
//!         "./config/location/lab-a",
//!         "./config/robot/robot-01",
//!     ])
//!     .build()?;
//! let node = ctx.create_node("walk_publisher").with_namespace("motion").build()?;
//! let config = node.bind_config_with_metadata_as::<WalkConfig>("walk_publisher")?;
//! let paths = config.list_paths()?;
//! let meta = config.get_metadata("linear_x")?;
//! config.set_json("linear_x", serde_json::json!(0.25), "./config/robot/robot-01")?;
//! assert!(paths.contains(&"linear_x".to_string()));
//! assert_eq!(meta.min, Some(-1.0));
//! # Ok(())
//! # }
//! ```

mod error;
mod loader;
mod merge;
mod metadata;
mod node_config;
mod persistence;
mod snapshot;
mod types;

pub mod remote;

pub use error::{ConfigError, Result};
pub use metadata::{ConfigFieldMetadata, ConfigMetadata};
pub use node_config::{ConfigJsonWrite, NodeConfig, NodeConfigExt, ValidateHook};
pub use remote::{RemoteConfigClient, types::*};
pub use ros_z_derive::ConfigMetadata;
pub use snapshot::{ConfigSubscription, ConfigTimestamp, NodeConfigSnapshot};
pub use types::{ConfigKey, FieldPath, LayerPath, ProvenanceMap};

/// Prelude for the extension traits needed to bind config to `ros-z` nodes.
pub mod prelude {
    pub use crate::node_config::NodeConfigExt;
}
