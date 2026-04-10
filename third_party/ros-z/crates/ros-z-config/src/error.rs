use std::{fmt, path::PathBuf};

use crate::{ConfigKey, LayerPath};

/// Errors produced by the node-local config subsystem.
///
/// Internal APIs use this typed error surface directly. Remote request/response
/// APIs collapse errors into human-readable message strings.
#[derive(Debug, Clone)]
pub enum ConfigError {
    FileNotFound { path: PathBuf },
    FileReadError { path: PathBuf, message: String },
    ParseError { path: PathBuf, message: String },
    MergeError { message: String },
    DeserializationError { message: String },
    ValidationError { message: String },
    RevisionMismatch { expected: u64, actual: u64 },
    PersistenceError { path: PathBuf, message: String },
    PathError { path: String, reason: String },
    EmptyLayerList,
    InvalidConfigKey { key: ConfigKey },
    LayerNotActive { layer: LayerPath },
    AlreadyBound { node_fqn: String },
    RemoteError { message: String },
}

/// Convenience result type for config operations.
pub type Result<T> = std::result::Result<T, ConfigError>;

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::FileNotFound { path } => write!(f, "config file not found: {}", path.display()),
            Self::FileReadError { path, message } => {
                write!(
                    f,
                    "failed to read config file {}: {message}",
                    path.display()
                )
            }
            Self::ParseError { path, message } => {
                write!(
                    f,
                    "failed to parse config file {}: {message}",
                    path.display()
                )
            }
            Self::MergeError { message } => write!(f, "merge error: {message}"),
            Self::DeserializationError { message } => {
                write!(f, "typed config deserialization failed: {message}")
            }
            Self::ValidationError { message } => write!(f, "config validation failed: {message}"),
            Self::RevisionMismatch { expected, actual } => {
                write!(f, "revision mismatch: expected {expected}, actual {actual}")
            }
            Self::PersistenceError { path, message } => {
                write!(f, "failed to persist {}: {message}", path.display())
            }
            Self::PathError { path, reason } => write!(f, "invalid path '{path}': {reason}"),
            Self::EmptyLayerList => write!(f, "config layer list must not be empty"),
            Self::InvalidConfigKey { key } => write!(f, "invalid config key '{key}'"),
            Self::LayerNotActive { layer } => {
                write!(f, "target layer is not active for this node: {layer}")
            }
            Self::AlreadyBound { node_fqn } => {
                write!(f, "config already bound for node {node_fqn}")
            }
            Self::RemoteError { message } => write!(f, "remote config error: {message}"),
        }
    }
}

impl std::error::Error for ConfigError {}

impl From<serde_json::Error> for ConfigError {
    fn from(value: serde_json::Error) -> Self {
        Self::DeserializationError {
            message: value.to_string(),
        }
    }
}
