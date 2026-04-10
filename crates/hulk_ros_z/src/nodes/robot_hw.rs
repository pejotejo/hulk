use std::sync::Arc;

use color_eyre::Result;
use ros_z::context::ZContext;

pub async fn run(_ctx: Arc<ZContext>) -> Result<()> {
    std::future::pending::<()>().await;
    #[allow(unreachable_code)]
    Ok(())
}
