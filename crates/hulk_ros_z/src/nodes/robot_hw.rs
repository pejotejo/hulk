use std::sync::Arc;

use color_eyre::Result;

use crate::stack::{NodeTaskHandle, StackContext};

pub fn spawn(_stack: Arc<StackContext>) -> Result<NodeTaskHandle> {
    Ok(tokio::spawn(async { Ok(()) }))
}
