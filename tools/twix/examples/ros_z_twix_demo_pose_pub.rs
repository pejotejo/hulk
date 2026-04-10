use std::{thread, time::Duration};

use clap::Parser;
use color_eyre::eyre::{Result, eyre};
use ros_z::{Builder, MessageTypeInfo, context::ZContextBuilder};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, Default, MessageTypeInfo)]
#[ros_msg(type_name = "twix_demo/msg/RobotPose")]
struct RobotPose {
    x: f64,
    y: f64,
    theta: f64,
    confidence: f64,
    state: String,
}

impl ros_z::msg::ZMessage for RobotPose {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}

#[derive(Debug, Parser)]
struct Args {
    #[arg(long, default_value = "tcp/127.0.0.1:7447")]
    endpoint: String,

    #[arg(long, default_value = "/twix_demo/robot_pose")]
    topic: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    color_eyre::install()?;
    let args = Args::parse();

    let ctx = ZContextBuilder::default()
        .with_router_endpoint(args.endpoint)
        .map_err(|error| eyre!(error.to_string()))?
        .build()
        .map_err(|error| eyre!(error.to_string()))?;
    let node = ctx
        .create_node("twix_demo_pose_publisher")
        .with_namespace("tools")
        .with_type_description_service()
        .build()
        .map_err(|error| eyre!(error.to_string()))?;
    let publisher = node
        .create_pub::<RobotPose>(&args.topic)
        .build()
        .map_err(|error| eyre!(error.to_string()))?;

    let mut tick = 0_u64;
    loop {
        let phase = tick as f64 / 20.0;
        let pose = RobotPose {
            x: phase.cos() * 2.5,
            y: phase.sin() * 1.5,
            theta: phase,
            confidence: 0.6 + ((phase / 2.0).sin() + 1.0) * 0.2,
            state: if tick % 80 < 40 {
                "tracking".to_string()
            } else {
                "recovering".to_string()
            },
        };

        publisher
            .async_publish(&pose)
            .await
            .map_err(|error| eyre!(error.to_string()))?;
        tick += 1;
        thread::sleep(Duration::from_millis(100));
    }
}
