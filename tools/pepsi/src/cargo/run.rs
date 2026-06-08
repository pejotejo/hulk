use std::{ffi::OsStr, path::Path};

use clap::{ArgAction, Parser};
use repository::cargo::Cargo;

use super::common::CommonOptions;
use super::{CargoCommand, heading};

// roughly based on https://github.com/messense/cargo-options

#[derive(Clone, Debug, Default, Parser)]
#[command(display_order = 1)]
pub struct Arguments {
    #[command(flatten)]
    pub common: CommonOptions,

    /// Build artifacts in release mode, with optimizations
    #[arg(short = 'r', long, help_heading = heading::COMPILATION_OPTIONS)]
    pub release: bool,

    /// Ignore `rust-version` specification in packages
    #[arg(long)]
    pub ignore_rust_version: bool,

    /// Output build graph in JSON (unstable)
    #[arg(long, help_heading = heading::COMPILATION_OPTIONS)]
    pub unit_graph: bool,

    /// Package to run (see `cargo help pkgid`)
    #[arg(
        short = 'p',
        long = "package",
        value_name = "SPEC",
        action = ArgAction::Append,
        num_args=0..=1,
        help_heading = heading::PACKAGE_SELECTION,
    )]
    pub packages: Vec<String>,

    /// Run the specified binary
    #[arg(
        long,
        value_name = "NAME",
        action = ArgAction::Append,
        num_args=0..=1,
        help_heading = heading::TARGET_SELECTION,
    )]
    pub bin: Vec<String>,

    /// Run the specified example
    #[arg(
        long,
        value_name = "NAME",
        action = ArgAction::Append,
        num_args=0..=1,
        help_heading = heading::TARGET_SELECTION,
    )]
    pub example: Vec<String>,

    /// Arguments for the binary to run
    #[arg(value_name = "args", trailing_var_arg = true, num_args = 0..)]
    pub args: Vec<String>,
}

impl CargoCommand for Arguments {
    const SUB_COMMAND: &'static str = "run";

    fn apply(&self, cargo: &mut Cargo) {
        self.apply_cargo_options(cargo);
        self.apply_target_selection(cargo);
        apply_binary_arguments(cargo, &self.args);
    }

    fn apply_for_manifest(&self, cargo: &mut Cargo, manifest: Option<&OsStr>) {
        if let Some(scenario) = bevyhavior_scenario_path_arguments(manifest, &self.args) {
            self.apply_cargo_options(cargo);
            cargo.arg("--bin").arg(scenario.bin);
            apply_binary_arguments(cargo, &scenario.args);
        } else {
            self.apply(cargo);
        }
    }

    fn profile(&self) -> &str {
        self.common.profile.as_deref().unwrap_or("dev")
    }
}

impl Arguments {
    fn apply_cargo_options(&self, cargo: &mut Cargo) {
        self.common.apply(cargo);

        if self.release {
            cargo.arg("--release");
        }
        if self.ignore_rust_version {
            cargo.arg("--ignore-rust-version");
        }
        if self.unit_graph {
            cargo.arg("--unit-graph");
        }
        for pkg in &self.packages {
            cargo.arg("--package").arg(pkg);
        }
    }

    fn apply_target_selection(&self, cargo: &mut Cargo) {
        for bin in &self.bin {
            cargo.arg("--bin").arg(bin);
        }
        for example in &self.example {
            cargo.arg("--example").arg(example);
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
struct ScenarioPathArguments {
    bin: String,
    args: Vec<String>,
}

fn bevyhavior_scenario_path_arguments(
    manifest: Option<&OsStr>,
    args: &[String],
) -> Option<ScenarioPathArguments> {
    if !manifest.is_some_and(is_bevyhavior_simulator_manifest) {
        return None;
    }
    let [command, scenario_path, remaining @ ..] = args else {
        return None;
    };
    if !matches!(command.as_str(), "run" | "serve") {
        return None;
    }
    let scenario_path = Path::new(scenario_path);
    if scenario_path.extension().and_then(OsStr::to_str) != Some("rs") {
        return None;
    }
    let bin = scenario_path.file_stem()?.to_str()?.to_string();
    let mut args = vec![command.clone()];
    args.extend(remaining.iter().cloned());

    Some(ScenarioPathArguments { bin, args })
}

fn is_bevyhavior_simulator_manifest(manifest: &OsStr) -> bool {
    let manifest = manifest.to_string_lossy();
    matches!(
        manifest.as_ref(),
        "bevyhavior_simulator"
            | "crates/bevyhavior_simulator"
            | "crates/bevyhavior_simulator/Cargo.toml"
    )
}

fn apply_binary_arguments(cargo: &mut Cargo, args: &[String]) {
    if !args.is_empty() {
        cargo.arg("--");
        cargo.args(args);
    }
}

#[cfg(test)]
mod tests {
    use std::ffi::OsStr;

    use super::*;

    #[test]
    fn translates_bevyhavior_scenario_path_to_bin_and_run_command() {
        let args = ["run".to_string(), "/bin/vanishing_ball.rs".to_string()];

        assert_eq!(
            bevyhavior_scenario_path_arguments(Some(OsStr::new("bevyhavior_simulator")), &args),
            Some(ScenarioPathArguments {
                bin: "vanishing_ball".to_string(),
                args: vec!["run".to_string()],
            }),
        );
    }

    #[test]
    fn translates_bevyhavior_scenario_path_to_bin_and_serve_command() {
        let args = ["serve".to_string(), "/bin/vanishing_ball.rs".to_string()];

        assert_eq!(
            bevyhavior_scenario_path_arguments(Some(OsStr::new("bevyhavior_simulator")), &args),
            Some(ScenarioPathArguments {
                bin: "vanishing_ball".to_string(),
                args: vec!["serve".to_string()],
            }),
        );
    }

    #[test]
    fn rejects_legacy_scenario_path_without_mode() {
        let args = ["/bin/vanishing_ball.rs".to_string()];

        assert_eq!(
            bevyhavior_scenario_path_arguments(Some(OsStr::new("bevyhavior_simulator")), &args),
            None,
        );
    }

    #[test]
    fn keeps_normal_run_arguments_unchanged() {
        let args = ["--run".to_string()];

        assert_eq!(
            bevyhavior_scenario_path_arguments(Some(OsStr::new("bevyhavior_simulator")), &args),
            None,
        );
    }
}
