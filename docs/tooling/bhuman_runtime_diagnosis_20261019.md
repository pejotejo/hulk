# B-Human simulator timing and ball-input check — 2026-10-19

No native timing or ball-velocity regression was reproduced. Nine existing
integration tests pass. Ten isolated physical probes produce byte-identical
contact and trajectory logs across playback speeds within each scene.

The user's earlier launch settings are unknown, so these results do not identify
the cause of that particular observation or establish user misconfiguration.
Production behavior, physics, models and the native adapter remain unchanged.

## What was checked

The source base is HULKs commit `11ee05b9e31272522854c2ef3d6546718c376e3d`.
The native worker uses B-Human `2e3374ba896da8288607f9a1881b876642fe18b8` and has
SHA256 `08437e521a209d4fe1809c307550626cdf1251bfb0c6b2dd5d5340fb2097ea03`.
That worker hash matches the original saved evaluation manifest. The GUI default
and evaluation ONNX Runtime paths resolve to the same 1.22.0 library in this setup.

The native scheduling, sensors, coordinate conversion and physics stepping code
have not changed since the initial B-Human integration `bc8fd0146`. Later changes
in this area concern launching, caching, linker selection and labels.

- Physics and native motion advance by 2 ms. Cognition executes every 20 ms.
  Native request ordering rejects skipped or repeated frames.
- Playback speed changes wall-time pacing. The shared physics worker waits for
  native actuation before committing the next physics step; slow execution
  reduces actual playback speed rather than increasing the native timestep.
- Ball velocity comes from MuJoCo's free-joint translation velocity in m/s.
  Field-side and robot-yaw rotations apply consistently to position and velocity;
  the adapter converts velocity to mm/s once for native `BallModel`.
- The native kick-policy input converts velocity back to SI and applies its
  upstream scaling/clamping. No missing ego-velocity subtraction was found.

Relevant implementations are the [physics loop](../../tools/simulate/src/simulation.rs),
[B-Human host bridge](../../tools/simulate/src/bhuman/simulator.rs),
[native worker](../../tools/simulate/bhuman/main.cpp) and
[MuJoCo ball observation](../../tools/simulate/src/observations.rs).

## Physical playback comparison

Each probe runs one native K1 and one physical ball for ten simulated seconds,
with the same actuator mapping, MuJoCo model and ball contact parameters as the
simulator. Native input and physics time advance synchronously; deterministic
GameController packets replace asynchronous transport. HULKs nodes and GUI
rendering are absent.

The stationary ball starts at (1.3, 0) m. The lateral rolling ball starts at
(1.1, −0.45) m and receives velocity (0, 0.8, 0) m/s after one simulated second.
The robot begins at the origin. Every run verifies all 5,000 physics steps and
the exact native cognition cadence. Trajectories are recorded every 20 ms.

Contacts are actual MuJoCo ball–robot contacts with normal force at least 0.01 N,
including the contacting body name. They are not inferred from center distances.

| Field / ball radius | Scene | ¼× first contact | 1× first contact | 4× first contact |
| --- | --- | --- | --- | --- |
| `hsl_small` / 0.073 m | Stationary | 2.392 s | 2.392 s | 2.392 s |
| `hsl_small` / 0.073 m | Rolling | 2.254 s | 2.254 s | 2.254 s |
| `incheon_small` / 0.105 m | Stationary | Not run | 2.388 s | 2.388 s |
| `incheon_small` / 0.105 m | Rolling | Not run | 2.342 s | 2.342 s |

For each row, both the complete contact log and the complete trajectory log are
byte-identical across its tested speeds. All ten probes stay upright under the
probe's geometric thresholds. Actual playback is about 0.24×, 0.94–0.96× and
3.48–3.54× because execution and sleeping add overhead. That wall-time difference
does not alter the physical trajectories in these probes.

The stationary and rolling scenes produce different outgoing paths. These logs
do not export the native requested kick target, so a ball trajectory alone is
not a measurement of kick-target error. In particular, repeatability does not
establish general rolling-ball precision or realistic contact physics.

Full matches can still diverge across runs or speeds because HULKs inference,
ROS transport, GameController delivery and GUI load introduce asynchronous work.
The existing mixed-team test exercises the shared physics path, native frame
counting and pause handling; the isolated comparison does not replace a
reproduction of the user's original GUI session.

## A concrete configuration difference

The original GUI default is `incheon_small`, with a 0.105 m ball radius and
2.4 m goal opening. The evaluated Small matches used `hsl_small`, with a
0.073 m ball radius and 1.8 m goal opening. These are different kicking tasks.
The GUI also preserves the selectable ball override and profile.

Both ball sizes use the same kick networks. Radius is available to higher-level
native approach geometry but is not a direct network input. Ball-size selection
changes the sphere radius while retaining the shared mass, friction, damping and
contact settings. The source does not establish the networks' training radius.
This can change contact behavior; it is not proof of the reported regression.

Fresh GUI sessions default to 1×; the evaluation requested 4×. Saved custom worker
and source paths can override launcher defaults, and an existing
`ORT_DYLIB_PATH` overrides the launcher's library choice. No readable saved GUI
settings were available here, so none of these possibilities is asserted as the
user's actual setup.

For a direct comparison, select the same field, ball override, Motion + Behavior
profile and worker/runtime in both launches. Keep the session's recorded settings
with the footage. Changing to the historical larger ball is not a recommendation
for evaluating Small-division behavior.

## Evidence and reproduction

Local artifacts are retained in `logs/bhuman-runtime-diagnosis-20261019/`:

- `runtime.json`, `native-integration-tests.log`: pinned assets and nine passing
  protocol, cognition, team-message, pause, reset and failure-handling tests.
- `comparison.json`: all ten results, exact contact/trajectory hashes and limits.
- Per-case `summary.json`, `contacts.jsonl`, `trajectory.jsonl` and `run.log`.
- `native-runtime-probe`, `probe-binary.json`, `bhuman_runtime_probe.rs`,
  `probe-injection.patch` and build logs: frozen diagnostic instrumentation.
- Build and run wrappers, with explicit local checkout and runtime paths.

The diagnostic module was attached only while compiling the saved test binary;
the production source was restored immediately afterward. No physics or policy
parameters were tuned to these results. Reproducing elsewhere requires adjusting
the wrappers' local paths and using a fresh output directory.

The existing integration check was:

```sh
BHUMAN_WORKER=/path/to/bhuman-motion-worker \
BHUMAN_SOURCE=/path/to/BHumanCodeRelease \
cargo +1.98.0 test -p simulate --features bhuman --lib bhuman:: \
  -- --include-ignored --test-threads=1
```

Use the MuJoCo and ONNX Runtime environment described in the
[B-Human simulator guide](../../tools/simulate/bhuman/README.md).
