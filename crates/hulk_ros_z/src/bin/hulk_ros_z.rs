use std::{path::PathBuf, sync::Arc};

use clap::Parser;
use color_eyre::{Result, eyre::eyre};
use hulk_ros_z::{IntoEyreResultExt, nodes};
use ros_z::{Builder, context::ZContextBuilder};
use tokio::task::JoinSet;
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

struct RunningStack {
    join_set: JoinSet<Result<()>>,
}

#[tokio::main]
async fn main() -> Result<()> {
    color_eyre::install()?;
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .init();

    let args = Args::parse();
    let namespace = if args.robot.starts_with('/') {
        args.robot.clone()
    } else {
        format!("/{}", args.robot)
    };
    let config_layers = derive_config_layers(&args.config_root, &args.location, &args.robot);

    let mut builder = ZContextBuilder::default()
        .with_namespace(&namespace)
        .with_domain_id(args.domain)
        .with_config_layers(config_layers);

    builder = match args.router {
        Some(router) => builder
            .with_mode("client")
            .with_router_endpoint(router)
            .into_eyre()?,
        None => builder
            .with_mode("router")
            .disable_multicast_scouting()
            .with_connect_endpoints(std::iter::empty::<&str>())
            .with_listen_endpoints(["tcp/127.0.0.1:7447"]),
    };

    let ctx = Arc::new(builder.build().into_eyre()?);
    let mut running = spawn_all(ctx.clone()).await?;

    let result = tokio::select! {
        result = monitor(&mut running.join_set) => result,
        _ = tokio::signal::ctrl_c() => {
            Ok(())
        }
    };

    running.join_set.abort_all();
    ctx.shutdown().into_eyre()?;
    result
}

fn derive_config_layers(
    config_root: &std::path::Path,
    location: &str,
    robot: &str,
) -> Vec<PathBuf> {
    vec![
        config_root.join("base"),
        config_root.join("location").join(location),
        config_root.join("robot").join(robot),
    ]
}

async fn spawn_all(ctx: Arc<ros_z::context::ZContext>) -> Result<RunningStack> {
    let mut join_set = JoinSet::new();
    join_set.spawn(nodes::sim_driver::run(ctx.clone()));
    join_set.spawn(nodes::state_estimator::run(ctx.clone()));
    join_set.spawn(nodes::behavior::run(ctx.clone()));
    join_set.spawn(nodes::motion::run(ctx.clone()));
    join_set.spawn(nodes::robot_hw::run(ctx.clone()));
    join_set.spawn(nodes::vision::run(ctx.clone()));
    join_set.spawn(nodes::object_detection::run(ctx));

    Ok(RunningStack { join_set })
}

async fn monitor(join_set: &mut JoinSet<Result<()>>) -> Result<()> {
    while let Some(result) = join_set.join_next().await {
        match result {
            Ok(Ok(())) => {}
            Ok(Err(error)) => return Err(error),
            Err(join_error) => return Err(eyre!("monitor join failed: {join_error}")),
        }
    }

    Ok(())
}
