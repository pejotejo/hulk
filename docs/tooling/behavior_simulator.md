# Bevyhavior Simulator

A simplified simulator which can be used for manual or automatic testing of behavior in a defined scenario.

## Usage

Run a scenario headless:

```sh
./pepsi run bevyhavior_simulator -- run /bin/vanishing_ball.rs
```

This is equivalent to `cargo run -p bevyhavior_simulator --bin vanishing_ball -- run`.

Serve a live Twix timeline:

```sh
./pepsi run bevyhavior_simulator -- serve /bin/vanishing_ball.rs
```

This is equivalent to `cargo run -p bevyhavior_simulator --bin vanishing_ball -- serve`.

The restored simulator uses the code-generation framework path. It does not use the zenoh/`ros-z` node framework, the removed visual-referee support, or the separate MuJoCo/Bevy simulator.

In `serve` mode, open [Twix](./twix.md) and use the `BehaviorSimulator` panel and the map panel's `Behavior Simulator` layer while the scenario process keeps running. Stop the scenario process when you are done viewing the timeline.
The simulator returns an error in `run` mode if the robotics code encountered a problem or if the scenario generated an error.

## Scenario Development

The restored crate currently builds these scenario binaries:

- `vanishing_ball`
- `golden_goal`
- `golden_goal_opponent_kickoff`
- `step_planning_test`
- `penalty_shootout_attacking`
- `flickering_ball`
- `goal_kicks`
- `ingame_penalty_kick_opponent`
- `ingame_penalty_kick_opponent_with_kick`
- `ingame_penalty_kick`
- `ball_search`
- `defender_positioning`
- `standing_searcher`
- `quantum_ball`
- `hulks_vs_ghosts`
- `walk_around_ball`
- `oscillating_obstacle`
- `mpc_step_planning_optimizer`
- `kicking_team_filtering`
- `kick_in`
- `ingame_penalty_kick_striker_penalized`
- `demonstration`
- `intercept_ball`
- `striker_dies`
- `golden_goal_striker_penalized`

Use the scenario file path with Pepsi, for example:

```sh
./pepsi run bevyhavior_simulator -- serve /bin/golden_goal.rs
```
