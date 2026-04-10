use std::sync::Arc;

use color_eyre::Result;

use crate::stack::NodeTaskHandle;

pub fn spawn(_ctx: Arc<ros_z::context::ZContext>) -> Result<NodeTaskHandle> {
    Ok(tokio::spawn(async { Ok(()) }))
}
