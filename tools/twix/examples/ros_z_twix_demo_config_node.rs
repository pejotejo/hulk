use std::{
    fs,
    path::{Path, PathBuf},
    sync::Arc,
};

use clap::Parser;
use color_eyre::eyre::{Result, eyre};
use ros_z::{Builder, context::ZContextBuilder};
use ros_z_config::{ConfigMetadata, prelude::*};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
struct TwixDemoConfig {
    #[config(doc = "Enable the synthetic demo node")]
    enabled: bool,

    #[config(doc = "Forward speed command", min = -1.0, max = 1.0)]
    linear_x: f64,

    #[config(doc = "Turn speed command", min = -2.0, max = 2.0)]
    angular_z: f64,

    #[config(doc = "UI label shown by the demo node")]
    label: String,
}

#[derive(Debug, Parser)]
struct Args {
    #[arg(long, default_value = "tcp/127.0.0.1:7447")]
    endpoint: String,

    #[arg(long)]
    config_root: Option<PathBuf>,

    #[arg(long, default_value = "lab-a")]
    location: String,

    #[arg(long, default_value = "robot-01")]
    robot: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    color_eyre::install()?;
    let args = Args::parse();
    let config_root = args.config_root.unwrap_or_else(default_config_root);
    seed_config(&config_root)?;

    let ctx = ZContextBuilder::default()
        .with_router_endpoint(args.endpoint)
        .map_err(|error| eyre!(error.to_string()))?
        .with_config_layers([
            config_root.join("base"),
            config_root.join(format!("location/{}", args.location)),
            config_root.join(format!("robot/{}", args.robot)),
        ])
        .build()
        .map_err(|error| eyre!(error.to_string()))?;
    let node = ctx
        .create_node("twix_demo_config")
        .with_namespace("motion")
        .build()
        .map_err(|error| eyre!(error.to_string()))?;
    let config = node.bind_config_with_metadata_as::<TwixDemoConfig>("twix_demo")?;

    config.add_validation_hook(Arc::new(|candidate: &TwixDemoConfig| {
        if candidate.label.trim().is_empty() {
            return Err("label must not be empty".to_string());
        }
        Ok(())
    }))?;

    println!("node_fqn=/motion/twix_demo_config");
    println!("config_root={}", config_root.display());
    println!("available_paths={:?}", config.list_paths()?);

    std::future::pending::<()>().await;
    Ok(())
}

fn default_config_root() -> PathBuf {
    std::env::temp_dir().join("twix_ros_z_demo_config")
}

fn seed_config(root: &Path) -> Result<()> {
    let path = root.join("base/twix_demo.json5");
    fs::create_dir_all(path.parent().expect("base layer parent"))?;

    let contents = r#"{
  enabled: true,
  linear_x: 0.2,
  angular_z: 0.5,
  label: "Twix Demo"
}
"#;
    fs::write(path, contents)?;
    Ok(())
}
