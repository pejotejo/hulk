//! Rust-first node-local configuration for `ros-z`.
//!
//! `ros-z-config` provides a node-local configuration subsystem intended for
//! native Rust applications built on top of `ros-z`.
//!
//! Core model:
//!
//! - each `ZNode` owns one config space
//! - process-wide bootstrap selection chooses `config_root`, `location`, and `robot`
//! - config files are JSON5 on read and pretty JSON on writeback
//! - overlay precedence is `default < location < robot`
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
//!     .with_config_root("./config")
//!     .with_location("lab-a")
//!     .with_robot("robot-01")
//!     .build()?;
//!
//! let node = ctx
//!     .create_node("ball_detector")
//!     .with_namespace("vision")
//!     .build()?;
//!
//! let config = node.bind_config::<VisionConfig>()?;
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
//! use ros_z_config::{ConfigJsonWrite, ConfigScope, prelude::*};
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
//!     .with_config_root("./config")
//!     .with_location("lab-a")
//!     .with_robot("robot-01")
//!     .build()?;
//! let node = ctx.create_node("ball_detector").with_namespace("vision").build()?;
//! let config = node.bind_config::<VisionConfig>()?;
//!
//! config.set_json("threshold", serde_json::json!(0.72), ConfigScope::Robot)?;
//! config.set_json_atomically(
//!     vec![ConfigJsonWrite {
//!         path: "enabled".into(),
//!         value: serde_json::json!(true),
//!         target_scope: ConfigScope::Robot,
//!     }],
//!     Some(config.snapshot().revision),
//! )?;
//! config.reset("threshold", ConfigScope::Robot)?;
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
//!     .with_config_root("./config")
//!     .with_location("lab-a")
//!     .with_robot("robot-01")
//!     .build()?;
//! let node = ctx.create_node("ball_detector").with_namespace("vision").build()?;
//! let config = node.bind_config::<VisionConfig>()?;
//! let mut updates = config.subscribe();
//! updates.changed().await?;
//! let snapshot = updates.borrow().clone();
//! let _threshold = snapshot.typed().threshold;
//! # Ok(())
//! # }
//! ```
//!
//! Metadata support is opt-in. Use `bind_config_with_metadata::<T>()` with a
//! type deriving [`ConfigMetadata`] when you want local and remote metadata
//! introspection.
//!
//! ```no_run
//! use ros_z::{Builder, context::ZContextBuilder};
//! use ros_z_config::{ConfigScope, prelude::*};
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
//!     .with_config_root("./config")
//!     .with_location("lab-a")
//!     .with_robot("robot-01")
//!     .build()?;
//! let node = ctx.create_node("walk_publisher").with_namespace("motion").build()?;
//! let config = node.bind_config_with_metadata::<WalkConfig>()?;
//! let paths = config.list_paths()?;
//! let meta = config.get_metadata("linear_x")?;
//! config.set_json("linear_x", serde_json::json!(0.25), ConfigScope::Robot)?;
//! assert!(paths.contains(&"linear_x".to_string()));
//! assert_eq!(meta.min, Some(-1.0));
//! # Ok(())
//! # }
//! ```

mod bootstrap;
mod error;
mod loader;
mod merge;
mod metadata;
mod node_config;
mod persistence;
mod scope;
mod snapshot;

pub mod remote;

pub use bootstrap::{BootstrapFile, ConfigFilePatterns, NodeConfigPaths, ResolvedBootstrap};
pub use error::{ConfigError, Result};
pub use metadata::{ConfigFieldMetadata, ConfigMetadata};
pub use node_config::{ConfigJsonWrite, NodeConfig, NodeConfigExt, ValidateHook};
pub use remote::RemoteConfigClient;
pub use remote::types::*;
pub use ros_z_derive::ConfigMetadata;
pub use scope::ConfigScope;
pub use snapshot::{ConfigSubscription, ConfigTimestamp, NodeConfigSnapshot, NodeScopeOverlays};

/// Prelude for the extension traits needed to bind config to `ros-z` nodes.
pub mod prelude {
    pub use crate::node_config::NodeConfigExt;
}
