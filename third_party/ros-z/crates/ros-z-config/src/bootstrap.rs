use std::{
    fs,
    path::{Path, PathBuf},
};

use serde::{Deserialize, Serialize};

use crate::{ConfigError, Result};

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct BootstrapFile {
    #[serde(default)]
    pub version: u64,
    #[serde(default)]
    pub active: BootstrapActive,
    #[serde(default)]
    pub files: Option<ConfigFilePatterns>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct BootstrapActive {
    pub location: Option<String>,
    pub robot: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConfigFilePatterns {
    pub default: String,
    pub location: String,
    pub robot: String,
}

impl Default for ConfigFilePatterns {
    fn default() -> Self {
        Self {
            default: "default/{node}.json5".to_string(),
            location: "location/{location}/{node}.json5".to_string(),
            robot: "robot/{robot}/{node}.json5".to_string(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct ResolvedBootstrap {
    pub config_root: PathBuf,
    pub location: String,
    pub robot: String,
    pub files: ConfigFilePatterns,
}

#[derive(Debug, Clone)]
pub struct NodeConfigPaths {
    pub default: PathBuf,
    pub location: PathBuf,
    pub robot: PathBuf,
}

impl ResolvedBootstrap {
    pub fn resolve_node_paths(&self, node_fqn: &str) -> Result<NodeConfigPaths> {
        let node = node_fqn_to_relative_path(node_fqn)?;
        Ok(NodeConfigPaths {
            default: self.config_root.join(render_pattern(
                &self.files.default,
                &node,
                &self.location,
                &self.robot,
            )),
            location: self.config_root.join(render_pattern(
                &self.files.location,
                &node,
                &self.location,
                &self.robot,
            )),
            robot: self.config_root.join(render_pattern(
                &self.files.robot,
                &node,
                &self.location,
                &self.robot,
            )),
        })
    }
}

pub fn resolve_bootstrap(
    inputs: &ros_z::context::RuntimeConfigInputs,
) -> Result<ResolvedBootstrap> {
    let config_root = inputs
        .config_root
        .clone()
        .or_else(|| std::env::var_os("ROS_Z_CONFIG_ROOT").map(PathBuf::from))
        .unwrap_or_else(|| PathBuf::from("./config"));

    let bootstrap_path = config_root.join("bootstrap.json5");
    let bootstrap = if bootstrap_path.exists() {
        let raw =
            fs::read_to_string(&bootstrap_path).map_err(|err| ConfigError::FileReadError {
                path: bootstrap_path.clone(),
                message: err.to_string(),
            })?;
        let bootstrap =
            json5::from_str::<BootstrapFile>(&raw).map_err(|err| ConfigError::ParseError {
                path: bootstrap_path.clone(),
                message: err.to_string(),
            })?;
        if bootstrap.version != 0 && bootstrap.version != 1 {
            return Err(ConfigError::ParseError {
                path: bootstrap_path.clone(),
                message: format!("unsupported bootstrap version: {}", bootstrap.version),
            });
        }
        bootstrap
    } else {
        BootstrapFile {
            version: 1,
            ..BootstrapFile::default()
        }
    };

    let location = inputs
        .location
        .clone()
        .or_else(|| std::env::var("ROS_Z_LOCATION").ok())
        .or(bootstrap.active.location)
        .ok_or(ConfigError::MissingSelection { field: "location" })?;

    let robot = inputs
        .robot
        .clone()
        .or_else(|| std::env::var("ROS_Z_ROBOT").ok())
        .or(bootstrap.active.robot)
        .ok_or(ConfigError::MissingSelection { field: "robot" })?;

    Ok(ResolvedBootstrap {
        config_root,
        location,
        robot,
        files: bootstrap.files.unwrap_or_default(),
    })
}

pub fn node_fqn_to_relative_path(node_fqn: &str) -> Result<String> {
    if !node_fqn.starts_with('/') {
        return Err(ConfigError::PathError {
            path: node_fqn.to_string(),
            reason: "node FQN must start with '/'".to_string(),
        });
    }

    let trimmed = node_fqn.trim_start_matches('/');
    if trimmed.is_empty() {
        return Err(ConfigError::PathError {
            path: node_fqn.to_string(),
            reason: "node FQN must not be root".to_string(),
        });
    }

    Ok(trimmed.to_string())
}

fn render_pattern(pattern: &str, node: &str, location: &str, robot: &str) -> PathBuf {
    Path::new(
        &pattern
            .replace("{node}", node)
            .replace("{location}", location)
            .replace("{robot}", robot),
    )
    .to_path_buf()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn node_fqn_to_relative_file_path() {
        assert_eq!(
            node_fqn_to_relative_path("/localization").unwrap(),
            "localization"
        );
        assert_eq!(
            node_fqn_to_relative_path("/vision/ball_detector").unwrap(),
            "vision/ball_detector"
        );
    }
}
