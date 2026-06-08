# Bevyhavior Simulator Twix Live Design

## Goal

Restore live timeline viewing in Twix while the behavior simulator runs, using an explicit `serve` mode. Keep a headless `run` mode. Remove the deprecated `--run` compatibility flag from user-facing simulator invocation.

## Current State

The simulator currently runs headless only and errors if recording/server mode is requested. The `recorder` and `server` modules exist and already implement frame capture and a communication server at `:1337`, but they are not wired into the simulator plugin. Twix expects `BehaviorSimulator` and `simulator.*` sources to be available for live timeline viewing.

## Approach (Recommended)

Re-enable the existing recorder/server pipeline and make simulator mode explicit via CLI. `serve` starts the timeline server and enables recording; `run` remains headless. Pepsi should support `./pepsi run bevyhavior_simulator -- serve /bin/<scenario>.rs` and `... -- run /bin/<scenario>.rs` by translating these to the corresponding simulator flags.

## CLI/UX Design

- Simulator arguments:
  - `serve` for Twix live timeline (default if no mode passed)
  - `run` for headless
  - `--run` is rejected
- Pepsi shim:
  - `./pepsi run bevyhavior_simulator -- serve /bin/<scenario>.rs` → `cargo run ... --bin <scenario> -- serve`
  - `./pepsi run bevyhavior_simulator -- run /bin/<scenario>.rs` → `cargo run ... --bin <scenario> -- run`
- Documentation:
  - Show both modes; live timeline uses `serve`

## Runtime Design

- `SimulatorPlugin` takes a `Mode` enum `{ Serve, Run }`.
- If `Serve`, add `recording_plugin` and retain `Recording` join logic in `run_to_completion`.
- If `Run`, no server/recording.
- The server continues to expose:
  - `BehaviorSimulator` source for frame timeline
  - `WorldState` source for selected robot state
  - `parameters` source for robot params
  - `simulator` source/sink for timeline controls

## Testing/Verification

- `cargo test -p pepsi` for the shim mapping
- `./pepsi run bevyhavior_simulator -- serve /bin/vanishing_ball.rs` should run and be viewable in Twix
- `./pepsi run bevyhavior_simulator -- run /bin/vanishing_ball.rs` should run headless
