use std::{path::PathBuf, sync::Arc};

use clap::Parser;
use color_eyre::Result;
use hulk_ros_z::{into_eyre, namespacing, stack};
use ros_z::{Builder, context::ZContextBuilder};
use tokio_util::sync::CancellationToken;
use tracing_subscriber::EnvFilter;

#[derive(Debug, Parser)]
struct Args {
    #[arg(long)]
    robot: String,
    #[arg(long)]
    location: String,
    #[arg(long, default_value = "config/ros_z")]
    config_root: PathBuf,
    #[arg(long, default_value_t = 0)]
    domain: usize,
    #[arg(long)]
    router: Option<String>,
}

#[tokio::main]
async fn main() -> Result<()> {
    color_eyre::install()?;
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .init();

    let args = Args::parse();
    let namespace = namespacing::normalize_robot_namespace(&args.robot);
    let config_layers = stack::derive_config_layers(&args.config_root, &args.location, &args.robot);

    let mut builder = ZContextBuilder::default()
        .with_namespace(&namespace)
        .with_domain_id(args.domain)
        .with_config_layers(config_layers);

    builder = match args.router {
        Some(router) => into_eyre(builder.with_router_endpoint(router))?,
        None => builder
            .with_mode("router")
            .disable_multicast_scouting()
            .with_connect_endpoints(std::iter::empty::<&str>())
            .with_listen_endpoints(["tcp/127.0.0.1:7447"]),
    };

    let ctx = Arc::new(into_eyre(builder.build())?);
    let shutdown = CancellationToken::new();
    let stack = Arc::new(stack::StackContext {
        ros_z: ctx.clone(),
        shutdown: shutdown.clone(),
        robot: Arc::from(args.robot),
        namespace: Arc::from(namespace),
    });

    let mut running = stack::spawn_all(stack).await?;

    tokio::select! {
        result = stack::monitor(&mut running.join_set, shutdown.clone()) => {
            result?;
        }
        _ = tokio::signal::ctrl_c() => {
            shutdown.cancel();
        }
    }

    stack::shutdown_and_await(
        &ctx,
        shutdown,
        &mut running.join_set,
        running.shutdown_grace_ms,
    )
    .await?;
    Ok(())
}
