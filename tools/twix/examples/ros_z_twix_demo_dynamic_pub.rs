use std::{thread, time::Duration};

use clap::Parser;
use color_eyre::eyre::{Result, eyre};
use ros_z::{Builder, MessageTypeInfo, context::ZContextBuilder};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, Default, MessageTypeInfo)]
#[ros_msg(type_name = "twix_demo/msg/BatteryMetrics")]
struct BatteryMetrics {
    battery_percentage: f64,
    motor_temperature_celsius: f64,
}

impl ros_z::msg::ZMessage for BatteryMetrics {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}

#[derive(Debug, Clone, Serialize, Deserialize, Default, MessageTypeInfo)]
#[ros_msg(type_name = "twix_demo/msg/Velocity2D")]
struct Velocity2D {
    x: f64,
    y: f64,
}

impl ros_z::msg::ZMessage for Velocity2D {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}

#[derive(Debug, Clone, Serialize, Deserialize, Default, MessageTypeInfo)]
#[ros_msg(type_name = "twix_demo/msg/DynamicStatus")]
struct DynamicStatus {
    robot_id: String,
    tick: u64,
    state: String,
    metrics: BatteryMetrics,
    velocity: Velocity2D,
    recent_scores: Vec<f64>,
}

impl ros_z::msg::ZMessage for DynamicStatus {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}

#[derive(Debug, Parser)]
struct Args {
    #[arg(long, default_value = "tcp/127.0.0.1:7447")]
    endpoint: String,

    #[arg(long, default_value = "/twix_demo/status")]
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
        .create_node("twix_demo_dynamic_publisher")
        .with_namespace("tools")
        .with_type_description_service()
        .build()
        .map_err(|error| eyre!(error.to_string()))?;
    let publisher = node
        .create_pub::<DynamicStatus>(&args.topic)
        .build()
        .map_err(|error| eyre!(error.to_string()))?;

    let states = ["idle", "walking", "kicking", "recovering"];
    let mut tick = 0_u64;

    loop {
        let phase = tick as f64 / 10.0;
        let message = DynamicStatus {
            robot_id: "twix-demo".to_string(),
            tick,
            state: states[(tick as usize) % states.len()].to_string(),
            metrics: BatteryMetrics {
                battery_percentage: 100.0 - (tick % 60) as f64 * 0.75,
                motor_temperature_celsius: 42.0 + phase.sin() * 8.0,
            },
            velocity: Velocity2D {
                x: phase.sin(),
                y: (phase / 2.0).cos() * 0.5,
            },
            recent_scores: vec![phase.sin(), phase.cos(), (phase * 0.5).sin()],
        };

        publisher
            .async_publish(&message)
            .await
            .map_err(|error| eyre!(error.to_string()))?;
        tick += 1;
        thread::sleep(Duration::from_millis(250));
    }
}
