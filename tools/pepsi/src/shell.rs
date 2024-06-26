use clap::Args;
use color_eyre::{eyre::WrapErr, Result};

use nao::Nao;

use argument_parsers::NaoAddress;

#[derive(Args)]
pub struct Arguments {
    /// The NAO to connect to e.g. 20w or 10.1.24.22
    #[arg(required = true)]
    pub nao: NaoAddress,
}

pub async fn shell(arguments: Arguments) -> Result<()> {
    let nao = Nao::try_new_with_ping(arguments.nao.ip).await?;

    nao.execute_shell()
        .await
        .wrap_err_with(|| format!("failed to execute shell on {}", arguments.nao))
}
