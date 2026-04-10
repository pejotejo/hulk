use std::sync::Arc;

use color_eyre::Result;

pub async fn run(_ctx: Arc<ros_z::context::ZContext>) -> Result<()> {
    std::future::pending::<()>().await;
    #[allow(unreachable_code)]
    Ok(())
}
