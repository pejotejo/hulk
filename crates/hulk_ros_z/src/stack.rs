use std::{path::PathBuf, sync::Arc};

use color_eyre::{Result, eyre::eyre};
use tokio::task::JoinSet;

use crate::nodes;

pub type NodeTaskHandle = tokio::task::JoinHandle<Result<()>>;

pub struct RunningStack {
    pub join_set: JoinSet<Result<()>>,
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

pub async fn spawn_all(ctx: Arc<ros_z::context::ZContext>) -> Result<RunningStack> {
    let handles = vec![
        nodes::sim_driver::spawn(ctx.clone())?,
        nodes::state_estimator::spawn(ctx.clone())?,
        nodes::behavior::spawn(ctx.clone())?,
        nodes::motion::spawn(ctx.clone())?,
        nodes::vision::spawn(ctx.clone())?,
    ];

    let mut join_set = JoinSet::new();
    for handle in handles {
        join_set.spawn(async move { handle.await.map_err(|error| eyre!(error.to_string()))? });
    }

    Ok(RunningStack { join_set })
}

pub async fn monitor(join_set: &mut JoinSet<Result<()>>) -> Result<()> {
    while let Some(result) = join_set.join_next().await {
        match result {
            Ok(Ok(())) => {}
            Ok(Err(error)) => return Err(error),
            Err(join_error) => return Err(eyre!("monitor join failed: {join_error}")),
        }
    }

    Ok(())
}
