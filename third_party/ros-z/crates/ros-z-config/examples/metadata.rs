use ros_z::{Builder, context::ZContextBuilder};
use ros_z_config::{ConfigMetadata, prelude::*};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
struct WalkPublisherConfig {
    #[config(doc = "Topic used for velocity commands", writable = false)]
    cmd_vel_topic: String,

    #[config(doc = "Publishing frequency in Hz", min = 1.0, max = 200.0)]
    publish_hz: f64,

    #[config(doc = "Forward walking speed in m/s", min = -1.0, max = 1.0)]
    linear_x: f64,

    #[config(doc = "Enable or disable publishing at runtime")]
    enabled: bool,
}

#[tokio::main(flavor = "multi_thread", worker_threads = 1)]
async fn main() -> std::result::Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let ctx = ZContextBuilder::default()
        .with_config_layers([
            "./config/base",
            "./config/location/lab-a",
            "./config/robot/robot-01",
        ])
        .build()?;

    let node = ctx
        .create_node("walk_publisher")
        .with_namespace("motion")
        .build()?;

    let config = node.bind_config_with_metadata_as::<WalkPublisherConfig>("walk_publisher")?;

    config.add_validation_hook(|cfg: &WalkPublisherConfig| {
        if cfg.publish_hz <= 0.0 {
            return Err("publish_hz must be > 0".into());
        }
        Ok(())
    })?;

    for path in config.list_paths()? {
        let meta = config.get_metadata(&path)?;
        println!(
            "{}: writable={} doc={}",
            meta.path, meta.writable, meta.description
        );
    }

    config.set_json(
        "linear_x",
        serde_json::json!(0.25),
        "./config/robot/robot-01",
    )?;
    if std::env::var_os("ROSZ_CONFIG_EXAMPLE_HOLD").is_some() {
        std::future::pending::<()>().await;
    }
    Ok(())
}
