use std::{path::PathBuf, sync::Arc, time::Duration};

use color_eyre::{Result, eyre::eyre};
use tokio::{task::JoinSet, time::Instant};
use tokio_util::sync::CancellationToken;

use crate::{
    bringup_manager::{BringupManagerConfig, spawn_bringup_manager},
    into_eyre, nodes,
};

pub type NodeTaskHandle = tokio::task::JoinHandle<Result<()>>;

#[derive(Debug)]
pub struct StackContext {
    pub ros_z: Arc<ros_z::context::ZContext>,
    pub shutdown: CancellationToken,
    pub robot: Arc<str>,
    pub namespace: Arc<str>,
}

pub struct RunningStack {
    pub join_set: JoinSet<Result<()>>,
    pub shutdown_grace_ms: u64,
}

pub fn derive_config_layers(
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

pub async fn spawn_all(stack: Arc<StackContext>) -> Result<RunningStack> {
    let handles = vec![
        nodes::sim_driver::spawn(stack.clone())?,
        nodes::state_estimator::spawn(stack.clone())?,
        nodes::behavior::spawn(stack.clone())?,
        nodes::motion::spawn(stack.clone())?,
        nodes::vision::spawn(stack.clone())?,
        spawn_bringup_manager(
            stack,
            BringupManagerConfig {
                shutdown_grace_ms: 2_000,
            },
        )?,
    ];

    let mut join_set = JoinSet::new();
    for handle in handles {
        join_set.spawn(async move { handle.await.map_err(|error| eyre!(error.to_string()))? });
    }

    Ok(RunningStack {
        join_set,
        shutdown_grace_ms: 2_000,
    })
}

pub async fn monitor(
    join_set: &mut JoinSet<Result<()>>,
    shutdown: CancellationToken,
) -> Result<()> {
    while let Some(result) = join_set.join_next().await {
        match result {
            Ok(Ok(())) => {}
            Ok(Err(error)) => {
                shutdown.cancel();
                return Err(error);
            }
            Err(join_error) => {
                shutdown.cancel();
                return Err(eyre!("monitor join failed: {join_error}"));
            }
        }
    }

    Ok(())
}

pub async fn shutdown_and_await(
    ctx: &ros_z::context::ZContext,
    shutdown: CancellationToken,
    join_set: &mut JoinSet<Result<()>>,
    shutdown_grace_ms: u64,
) -> Result<()> {
    shutdown.cancel();

    let deadline = Instant::now() + Duration::from_millis(shutdown_grace_ms);
    while !join_set.is_empty() {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Err(eyre!(
                "shutdown grace period elapsed before all tasks exited"
            ));
        }

        match tokio::time::timeout(remaining, join_set.join_next()).await {
            Ok(Some(Ok(Ok(())))) => {}
            Ok(Some(Ok(Err(error)))) => return Err(error),
            Ok(Some(Err(join_error))) => {
                return Err(eyre!("task join failed during shutdown: {join_error}"));
            }
            Ok(None) => break,
            Err(_) => {
                return Err(eyre!(
                    "shutdown grace period elapsed before all tasks exited"
                ));
            }
        }
    }

    into_eyre(ctx.shutdown())?;
    Ok(())
}
