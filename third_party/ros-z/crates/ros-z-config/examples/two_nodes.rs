use std::{sync::Arc, time::Duration};

use ros_z::{Builder, context::ZContextBuilder};
use ros_z_config::{ConfigScope, NodeConfigSnapshot, prelude::*};
use ros_z_msgs::geometry_msgs::{Twist, Vector3};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct WalkPublisherConfig {
    cmd_vel_topic: String,
    publish_hz: f64,
    linear_x: f64,
    angular_z: f64,
    enabled: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct WalkMonitorConfig {
    cmd_vel_topic: String,
    max_linear_x: f64,
    max_angular_z: f64,
    warn_only: bool,
}

#[tokio::main(flavor = "multi_thread", worker_threads = 1)]
async fn main() -> std::result::Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let ctx = ZContextBuilder::default()
        .with_config_root("./config")
        .with_location("lab-a")
        .with_robot("robot-01")
        .build()?;

    let pub_node = ctx
        .create_node("walk_publisher")
        .with_namespace("motion")
        .build()?;
    let sub_node = ctx
        .create_node("walk_monitor")
        .with_namespace("safety")
        .build()?;

    let pub_cfg = pub_node.bind_config::<WalkPublisherConfig>()?;
    let sub_cfg = sub_node.bind_config::<WalkMonitorConfig>()?;

    pub_cfg.add_validation_hook(Arc::new(|cfg: &WalkPublisherConfig| {
        if cfg.publish_hz <= 0.0 {
            return Err("publish_hz must be > 0".into());
        }
        Ok(())
    }))?;

    let topic = pub_cfg.snapshot().typed().cmd_vel_topic.clone();
    let zpub = pub_node.create_pub::<Twist>(&topic).build()?;
    let zsub = sub_node.create_sub::<Twist>(&topic).build()?;

    let pub_cfg_task = pub_cfg.clone();
    tokio::spawn(async move {
        loop {
            let snapshot: Arc<NodeConfigSnapshot<WalkPublisherConfig>> = pub_cfg_task.snapshot();
            let cfg = snapshot.typed();
            if cfg.enabled {
                let msg = Twist {
                    linear: Vector3 {
                        x: cfg.linear_x,
                        y: 0.0,
                        z: 0.0,
                    },
                    angular: Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: cfg.angular_z,
                    },
                };
                let _ = zpub.async_publish(&msg).await;
            }
            tokio::time::sleep(Duration::from_secs_f64(1.0 / cfg.publish_hz)).await;
        }
    });

    let sub_cfg_task = sub_cfg.clone();
    tokio::spawn(async move {
        while let Ok(msg) = zsub.async_recv().await {
            let snapshot: Arc<NodeConfigSnapshot<WalkMonitorConfig>> = sub_cfg_task.snapshot();
            let cfg = snapshot.typed();
            let linear_ok = msg.linear.x.abs() <= cfg.max_linear_x;
            let angular_ok = msg.angular.z.abs() <= cfg.max_angular_z;
            if !linear_ok || !angular_ok {
                eprintln!(
                    "cmd_vel limit violation: linear.x={:.2}, angular.z={:.2}",
                    msg.linear.x, msg.angular.z
                );
            }
        }
    });

    pub_cfg.set_json("linear_x", serde_json::json!(0.25), ConfigScope::Robot)?;

    std::future::pending::<()>().await;
    Ok(())
}
