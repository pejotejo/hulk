use std::{sync::Arc, time::Duration};

use color_eyre::Result;
use tracing::info;

use crate::stack::{NodeTaskHandle, StackContext};

#[derive(Debug, Clone, Copy)]
pub struct BringupManagerConfig {
    pub shutdown_grace_ms: u64,
}

pub fn spawn_bringup_manager(
    stack: Arc<StackContext>,
    config: BringupManagerConfig,
) -> Result<NodeTaskHandle> {
    Ok(tokio::spawn(async move {
        stack.shutdown.cancelled().await;
        info!(
            shutdown_grace_ms = config.shutdown_grace_ms,
            "bringup_manager observed shutdown request"
        );
        tokio::time::sleep(Duration::from_millis(config.shutdown_grace_ms)).await;
        Ok(())
    }))
}
