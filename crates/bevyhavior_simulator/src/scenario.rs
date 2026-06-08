use clap::{Parser, Subcommand};

use crate::simulator::Mode;

#[derive(Parser)]
pub struct Arguments {
    #[command(subcommand)]
    command: Option<Command>,
}

impl Arguments {
    pub fn mode(&self) -> Mode {
        self.command.unwrap_or(Command::Serve).into()
    }
}

#[derive(Copy, Clone, Subcommand)]
enum Command {
    /// Run the simulation without the Twix timeline server
    Run,
    /// Run the simulation with the Twix timeline server
    Serve,
}

impl From<Command> for Mode {
    fn from(command: Command) -> Self {
        match command {
            Command::Run => Self::Run,
            Command::Serve => Self::Serve,
        }
    }
}

#[cfg(test)]
mod tests {
    use clap::Parser;

    use super::*;

    #[test]
    fn parses_run_command_as_headless_mode() {
        let args = Arguments::parse_from(["scenario", "run"]);

        assert_eq!(args.mode(), Mode::Run);
    }

    #[test]
    fn parses_serve_command_as_timeline_mode() {
        let args = Arguments::parse_from(["scenario", "serve"]);

        assert_eq!(args.mode(), Mode::Serve);
    }

    #[test]
    fn defaults_to_serve_mode() {
        let args = Arguments::parse_from(["scenario"]);

        assert_eq!(args.mode(), Mode::Serve);
    }

    #[test]
    fn rejects_removed_run_flag() {
        assert!(Arguments::try_parse_from(["scenario", "--run"]).is_err());
    }
}
