# Behavior simulator evaluation — 2026-10-18

The candidate is based on `alex/feat/bhuman-opponent-interface` at
`d99ce43652833ffef688dcb88706f7e84965e89e` and lives on `codex/hulks-behavior-improvements`.
It retains the existing HULKs behavior tree and motion policies.

## Retained changes

- Raise regular-kick `kicking.target_speed` from 1.0 to 3.4 m/s, within the existing motion policy's configured range. Strong kicks stay disabled.
- When the ball's Voronoi owner is the goalkeeper but its pursuit gate rejects that ball, elect a field player using the same obstacle-aware costs and ownership hysteresis. Preserve the persistent support map. Use the same keeper eligibility gate during restarts.
- Recover supporters that have left the field and lost their Voronoi region. Choose a nearby unblocked field cell with rule-area clearance, require a planned route, and stand if no route exists. This fixes a game trace where all field-player seeds were outside the grid and all field players stood indefinitely.
- Face the cached own-half ball when the keeper returns home without a current ball observation, using the correct field-to-ground transform.

The keeper election bug and supporter recovery do not depend on opponent identity.
No motion, perception, B-Human behavior, or production simulator changes are retained.

## Checks

`cargo +1.98.0 test -p behavior_node -j 2`: **11 passed**.
The same tests against the original behavior reproduce **7 failures**; four guard cases already pass there.
Coverage includes the strict keeper pursuit boundary, both teams' restarts, missing keeper state, blocked ball cells, obstacle-aware travel costs, cached-ball transforms, field reentry, rule regions, unavailable routes, and coincident ball/target orientation.
Formatting checks for changed Rust files and `git diff --check` pass. Targeted runtime and semantic reviews have no remaining findings.

## Isolated recovery fixture

Both versions start with all three field players outside the ownership grid, a stationary ball at midfield, and stationary keeper/opponents. Real teammate messages settle before releasing the field players.
After 20 simulated seconds, the original has **0/3** players returned; the candidate has **3/3**.
All three candidate players are inside by the first two-second sample and remain inside at the end. The original three remain standing outside throughout.
`reentry-results.json` records positions and first observed return times; both runs include initial/final scene renders.

## Games

Each result is one fresh **120 simulated second** run after the whistle, with automatic refereeing and the `motion_behavior` profile.
Small uses `hsl_small`, 4v4, Small Foundation; Middle uses `hsl_middle`, 3v3, Middle Foundation.
Initial poses and per-team speeds are recorded by the runner. Middle cases reflect the Y coordinates; the kickoff team and field side are not swapped.
Scores are HULKs:opponent, read from recent GameController messages.

| Opponent | Field / players per team | HULKs target speed | Opponent target speed | Original score | Candidate score |
| --- | --- | ---: | ---: | --- | --- |
| HULKs | Small / 4 | 1.0 | 1.0 | 1:0 | 1:0 |
| HULKs | Small / 4 | 3.4 | 1.0 | 1:0 | 2:0 |
| HULKs | Middle, Y-reflected / 3 | 3.4 | 3.4 | 1:0 | 1:1 |
| B-Human | Small / 4 | 1.0 | native | 0:2 | 0:2 |
| B-Human | Small / 4 | 3.4 | native | 0:0 | 2:0 |
| B-Human | Middle, Y-reflected / 3 | 3.4 | native | 0:1 | 0:1 |

The 1.0-speed rows are controls; the candidate default is 3.4.
HULKs self-play uses the selected executable for **both** teams, so those rows are regression screens rather than comparisons against a frozen old HULKs opponent.
The native opponent is the unchanged B-Human worker pinned at `2e3374ba896da8288607f9a1881b876642fe18b8`.
These short, unseeded games are exploratory; they do not establish a win-rate improvement.

## Experiments left out

The kick/pass branch `pejotejo/ktsl-passing-motion-simulator` at `8f25983997c20cae6555457cb33cc550c38171f5` informed an obstacle-aware goal-opening experiment.
It did not show a reliable physical-game benefit, including a central-defender fixture, and was removed.
A broader keeper exclusion also changed support regions and was narrowed to the conditional striker election above.
Neither passing protocol changes nor B-Human tactics were imported.

An early native run advertised more active GameController players than the spawned roster, causing the pinned native formation code to wait/fail. Those failed runs are excluded; the final foundation configurations advertise exactly the spawned roster.
One interrupted parallel HULKs run and an earlier small-field fast native baseline that overlapped it are also excluded. The fast native baseline was repeated alone before this table was finalized.
Archived intermediate candidates and excluded runs remain in the artifact directory with explicit prefixes.

## Reproduction and artifacts

Local artifacts are in `logs/behavior-eval-20261018/`: `final-results.json`, `runtime.json`, the runners, parameter snapshots, binary hashes, per-run `snapshots.jsonl`, logs, test output, and the source patch.
Only directories named `baseline-<case>` and `candidate-<case>` enter `final-results.json`; the summarizer verifies binary hashes, rosters, profile, speeds, duration, sample counts, and final GameController score.

The experiment binaries were built with:

```sh
MUJOCO_DOWNLOAD_DIR=/tmp/hulk-mujoco cargo +1.98.0 build -p simulate --features bhuman --bin simulator-mcp -j 2
```

MuJoCo 3.9.0 and the pinned worker's ONNX Runtime library were used.
The saved runner sets `LD_LIBRARY_PATH`, `ORT_DYLIB_PATH`, and `LD_PRELOAD`, disables MCAP in a copied parameter tree, establishes observations, then advances owned MCP sessions.
Its local repository/cache paths are explicit at the top of the saved scripts.
The MCP client validates tool responses against the repository contract.

The base MCP server only selects HULKs opponents. The saved experimental binaries include the small adapter in `native-mcp-experiment.patch`, selecting the unchanged native worker through `HULK_EXPERIMENT_BHUMAN` and `BHUMAN_SOURCE`. That adapter was restored out of the final source diff.
`python3 run_behavior_matrix.py both` replays the matrix with those saved binaries, skipping completed directories; move existing run directories first to force fresh runs.
`python3 final_behavior_results.py` validates and summarizes only the final matrix.

Candidate experimental binary SHA-256: `e142feb4733fec52d9f6da7e848425b8da3769cde34a8449f56bb4c159d65b47`.
Baseline experimental binary SHA-256: `73f2ba5dc1ac21659c6e90f18b0670e5716288ac07923912b1fc600b4eb3776c`.

## Limits

Motion and behavior run against ideal field pose, ball, and obstacle inputs in this profile.
The cached-ball change is covered by regression tests; these full-scene game inputs do not exercise ball visibility loss.
Physics and ROS-Z scheduling are asynchronous and replay is not deterministic. Five-second snapshots miss brief transitions, and referee pauses take a substantial part of short runs.
No hardware, vision, or localization validation was performed. The 3.4 default still needs real-robot checking before treating simulated kick outcomes as field performance.
