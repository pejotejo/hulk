# Bevyhavior Simulator Twix Live Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore Twix live timeline serving for behavior simulator scenarios while keeping an explicit headless run mode.

**Architecture:** The simulator gets a small `Mode` enum and `SimulatorPlugin` stores the selected mode. Scenario binaries parse positional `run` or `serve` commands; no legacy `--run` flag remains. Pepsi translates `./pepsi run bevyhavior_simulator -- <mode> /bin/<scenario>.rs` into `cargo run --bin <scenario> -- <mode>`.

**Tech Stack:** Rust, Bevy, Clap, Tokio communication server, Pepsi cargo wrapper.

---

## Files

- Modify: `crates/bevyhavior_simulator/src/scenario.rs` for mode parsing and tests.
- Modify: `crates/scenario/src/lib.rs` for proc-macro generated main/test wiring.
- Modify: `crates/bevyhavior_simulator/src/simulator.rs` for `Mode`, mode constructors, recorder wiring, and recording join.
- Modify: `crates/bevyhavior_simulator/src/lib.rs` to compile `recorder` and `server` modules.
- Modify: `tools/pepsi/src/cargo/run.rs` for `run|serve /bin/*.rs` translation and tests.
- Modify: `docs/tooling/behavior_simulator.md` for new commands.

## Task 1: Scenario CLI Mode Parsing

**Files:**
- Modify: `crates/bevyhavior_simulator/src/scenario.rs`
- Modify: `crates/bevyhavior_simulator/src/simulator.rs`

- [ ] **Step 1: Write failing parser tests**

Add tests that describe the desired CLI and prove `--run` is gone:

```rust
#[cfg(test)]
mod tests {
    use clap::Parser;

    use super::*;
    use crate::simulator::Mode;

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
```

- [ ] **Step 2: Run tests and verify RED**

Run: `cargo test -p bevyhavior_simulator scenario::tests`

Expected: FAIL because `Arguments::mode` and `Mode` do not exist and `--run` is still accepted.

- [ ] **Step 3: Implement minimal mode type and parser**

Add the shared mode type to `crates/bevyhavior_simulator/src/simulator.rs`:

```rust
#[derive(Default, Debug, Copy, Clone, Eq, PartialEq)]
pub enum Mode {
    Serve,
    #[default]
    Run,
}
```

Replace the old boolean flag with a positional subcommand:

```rust
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
```

- [ ] **Step 4: Run tests and verify GREEN**

Run: `cargo test -p bevyhavior_simulator scenario::tests`

Expected: PASS.

## Task 2: Simulator Mode Wiring

**Files:**
- Modify: `crates/bevyhavior_simulator/src/lib.rs`
- Modify: `crates/bevyhavior_simulator/src/simulator.rs`
- Modify: `crates/scenario/src/lib.rs`

- [ ] **Step 1: Write failing mode tests**

Add focused unit tests in `crates/bevyhavior_simulator/src/simulator.rs`:

```rust
#[cfg(test)]
mod tests {
    use bevy::app::App;

    use super::*;
    use crate::recorder::Recording;

    #[test]
    fn run_mode_does_not_install_recording() {
        let mut app = App::new();

        app.add_plugins(SimulatorPlugin::run());

        assert!(!app.world().contains_resource::<Recording>());
    }

    #[test]
    fn default_plugin_is_headless_for_tests() {
        assert_eq!(SimulatorPlugin::default().mode(), Mode::Run);
    }
}
```

- [ ] **Step 2: Run tests and verify RED**

Run: `cargo test -p bevyhavior_simulator simulator::tests`

Expected: FAIL because `SimulatorPlugin::run`, `SimulatorPlugin::mode`, and `Mode` do not exist.

- [ ] **Step 3: Export recorder/server modules**

Add private modules to `crates/bevyhavior_simulator/src/lib.rs`:

```rust
mod recorder;
mod server;
```

- [ ] **Step 4: Implement simulator mode**

Update `crates/bevyhavior_simulator/src/simulator.rs` with:

```rust
use crate::recorder::{recording_plugin, Recording};

#[derive(Copy, Clone)]
pub struct SimulatorPlugin {
    mode: Mode,
}

impl SimulatorPlugin {
    pub fn new(mode: Mode) -> Self {
        Self { mode }
    }

    pub fn run() -> Self {
        Self::new(Mode::Run)
    }

    pub fn serve() -> Self {
        Self::new(Mode::Serve)
    }

    pub fn mode(&self) -> Mode {
        self.mode
    }
}

impl Default for SimulatorPlugin {
    fn default() -> Self {
        Self::run()
    }
}
```

In `Plugin::build`, add recorder only for serve mode:

```rust
if self.mode == Mode::Serve {
    app.add_plugins(recording_plugin);
}
```

In `run_to_completion`, after soft-error validation, join the server when recording exists:

```rust
if let Some(recording) = self.world_mut().remove_resource::<Recording>() {
    recording.join()?;
}
```

- [ ] **Step 5: Wire scenario generated main**

In `crates/scenario/src/lib.rs`, generated main should use CLI mode:

```rust
let args = bevyhavior_simulator::scenario::Arguments::parse();

App::new()
    .add_plugins(SimulatorPlugin::new(args.mode()))
    .add_plugins(#function_name)
    .run_to_completion()
```

Generated tests should remain headless:

```rust
bevy::app::App::new()
    .add_plugins(SimulatorPlugin::run())
    .add_plugins(super::#function_name)
    .run_to_completion()
```

- [ ] **Step 6: Run tests and verify GREEN**

Run: `cargo test -p bevyhavior_simulator scenario::tests`

Expected: PASS.

Run: `cargo test -p bevyhavior_simulator simulator::tests`

Expected: PASS.

## Task 3: Pepsi Scenario Translation

**Files:**
- Modify: `tools/pepsi/src/cargo/run.rs`

- [ ] **Step 1: Write failing translation tests**

Replace the old `--run` expectation and add serve coverage:

```rust
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
```

- [ ] **Step 2: Run tests and verify RED**

Run: `cargo test -p pepsi cargo::run::tests`

Expected: FAIL because `run` currently emits `--run`, `serve` is not translated, and the old test name/expectation is stale.

- [ ] **Step 3: Implement translation**

Update `bevyhavior_scenario_path_arguments`:

```rust
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
```

- [ ] **Step 4: Run tests and verify GREEN**

Run: `cargo test -p pepsi cargo::run::tests`

Expected: PASS.

## Task 4: Documentation And Verification

**Files:**
- Modify: `docs/tooling/behavior_simulator.md`

- [ ] **Step 1: Update docs**

Document the two Pepsi commands and direct cargo equivalents:

```markdown
## Usage

Run headless:

```sh
./pepsi run bevyhavior_simulator -- run /bin/vanishing_ball.rs
```

Serve a live Twix timeline:

```sh
./pepsi run bevyhavior_simulator -- serve /bin/vanishing_ball.rs
```
```

- [ ] **Step 2: Verify all targeted tests**

Run: `cargo test -p bevyhavior_simulator scenario::tests`

Expected: PASS.

Run: `cargo test -p bevyhavior_simulator simulator::tests`

Expected: PASS.

Run: `cargo test -p pepsi cargo::run::tests`

Expected: PASS.

Run: `cargo check -p bevyhavior_simulator`

Expected: PASS with only the existing generated-code unused-variable warning.

Run: `./pepsi run bevyhavior_simulator -- run /bin/vanishing_ball.rs`

Expected: scenario prints `Done` and exits successfully.

## Self-Review

- Spec coverage: The plan covers explicit `run|serve`, Twix recorder/server wiring, Pepsi shim translation, and docs.
- Deferred-work scan: No deferred implementation steps remain.
- Type consistency: `Mode`, `SimulatorPlugin::new`, `SimulatorPlugin::run`, and `Arguments::mode` are used consistently across tasks.
- Commit policy: No commit steps are included because the user did not explicitly request commits.
