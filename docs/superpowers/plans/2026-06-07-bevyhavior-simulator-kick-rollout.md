# Bevyhavior Simulator Kick Rollout Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make simulated VisualKick rollouts travel about 2m for `KickPower::Rumpelstilzchen` and 4m for `KickPower::Schlong` under active simulator friction.

**Architecture:** Keep ball movement natural through the existing velocity plus friction model. Calibrate the initial kick velocity from desired rollout distance, active ball friction, and the simulator tick duration instead of hard-capping ball position.

**Tech Stack:** Rust, Bevy systems, `types::motion_command::KickPower`, `BallResource` friction model.

---

### Task 1: Calibrate Kick Rollout Distance

**Files:**
- Modify: `crates/bevyhavior_simulator/src/robot.rs`
- Test/verify: `cargo test -p bevyhavior_simulator robot::tests::kick -- --nocapture`

- [ ] **Step 1: Write rollout-distance tests**

Add tests in `crates/bevyhavior_simulator/src/robot.rs` test module that verify the kick-speed helper maps the lower kick to about 2m and the higher kick to about 4m under default friction:

```rust
#[test]
fn lower_kick_rolls_about_two_meters_with_default_friction() {
    assert!(
        (default_rollout_distance(KickPower::Rumpelstilzchen) - 2.0).abs() < 0.01
    );
}

#[test]
fn higher_kick_rolls_about_four_meters_with_default_friction() {
    assert!((default_rollout_distance(KickPower::Schlong) - 4.0).abs() < 0.01);
}

fn default_rollout_distance(kick_power: KickPower) -> f32 {
    kick_speed(kick_power, 0.98) * 0.012 / (1.0 - 0.98)
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p bevyhavior_simulator robot::tests::kick -- --nocapture`

Expected: new rollout-distance tests fail because the current hard-coded kick speeds roll out farther than 2m and 4m.

- [ ] **Step 3: Implement calibrated speeds**

In `crates/bevyhavior_simulator/src/robot.rs`, replace hard-coded kick speeds with desired rollout distances converted to initial speed:

```rust
const SIMULATOR_TICK_SECONDS: f32 = 0.012;
const LOWER_KICK_ROLLOUT_DISTANCE: f32 = 2.0;
const HIGHER_KICK_ROLLOUT_DISTANCE: f32 = 4.0;

fn kick_speed(kick_power: KickPower, friction_coefficient: f32) -> f32 {
    kick_rollout_distance(kick_power) * (1.0 - friction_coefficient) / SIMULATOR_TICK_SECONDS
}

fn kick_rollout_distance(kick_power: KickPower) -> f32 {
    match kick_power {
        KickPower::Rumpelstilzchen => LOWER_KICK_ROLLOUT_DISTANCE,
        KickPower::Schlong => HIGHER_KICK_ROLLOUT_DISTANCE,
    }
}
```

- [ ] **Step 4: Run targeted verification**

Run: `cargo test -p bevyhavior_simulator robot::tests::kick -- --nocapture`

Expected: the rollout-distance tests pass, and existing VisualKick tests still pass.

- [ ] **Step 5: Run affected scenario verification**

Run: `cargo test -p bevyhavior_simulator --bin golden_goal --bin ingame_penalty_kick --bin penalty_shootout_attacking -- --nocapture`

Expected: scenarios either pass or reveal scenarios that depended on the old long kick distance and need local position/test-threshold updates.

### Task 2: Repair Scenarios That Assumed Long Kicks

**Files:**
- Modify only the scenario files that fail after Task 1.
- Test/verify: failing scenario command from Task 1.

- [ ] **Step 1: Inspect each failing scenario**

For each failing scenario, find the ball start position, target, and success criterion. Confirm whether failure is caused by the new 4m maximum rollout rather than broken kick execution.

- [ ] **Step 2: Apply minimal scenario adjustment**

If a scenario expects a goal from too far away, move the ball or robot close enough that a 4m strong kick can still satisfy the scenario intent. Do not change simulator kick physics to satisfy old scenario geometry.

- [ ] **Step 3: Re-run each repaired scenario**

Run the exact failing scenario command again.

Expected: the scenario passes for the same behavioral reason with geometry compatible with 2m/4m kick rollouts.

### Task 3: Final Verification

**Files:**
- Verify only.

- [ ] **Step 1: Format and check whitespace**

Run: `cargo fmt`

Run: `git diff --check`

Expected: both commands exit 0.

- [ ] **Step 2: Run simulator tests**

Run: `cargo test -p bevyhavior_simulator`

Expected: all simulator tests pass.

- [ ] **Step 3: Run simulator check**

Run: `cargo check -p bevyhavior_simulator`

Expected: compile succeeds. Existing generated-code warning for `databases` may remain.

- [ ] **Step 4: Smoke exposed scenarios through Pepsi if scenario geometry changed**

Run: `./pepsi run bevyhavior_simulator -- run /bin/golden_goal.rs`

Expected: scenario exits successfully.
