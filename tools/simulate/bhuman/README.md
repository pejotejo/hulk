# B-Human simulator players

Each B-Human opponent runs in its own process, with native K1 cognition and motion
module graphs from Code Release 2026. StrategyBehaviorControl and SkillBehaviorControl
choose motions, and the native MotionEngine and generators produce the joint commands.
The simulator supplies perfect pose, ball and player models. No image processing,
localization, camera capture, microphone capture or whistle detector runs.

## Build and select opponents

The native worker supports Linux on x86-64. Install Git, Python 3.12 or newer,
CMake 3.24 or newer, Ninja and Clang. The build uses mold when available and
otherwise uses the compiler's default linker. Start from an ordinary HULK
development environment, then launch:

```sh
./twix --simulator --bhuman /hulks/1
```

The launcher fetches missing simulator Git LFS meshes and ONNX models, builds the
pinned worker, enables Twix's `bhuman` Cargo feature and supplies its paths.
Cargo downloads MuJoCo 3.9.0 on its first build. The launcher configures its loader
path and uses the worker's pinned CPU ONNX Runtime for HULK motion inference,
including the preload workaround. An existing `ORT_DYLIB_PATH` takes precedence
and must name a shared library. Existing loader paths are retained.
Dependencies are cached under
`$XDG_CACHE_HOME/hulk-bhuman/<revision>`, using `~/.cache` as the default cache root.
Builds use all detected CPU threads by default; set `BHUMAN_JOBS` to limit them.
Subsequent launches rebuild the worker incrementally. Concurrent setup processes
wait on the same cache lock. Downloads require access to GitHub and the repository's
Git LFS remote; cached dependencies and hydrated assets work offline.
Missing system tools report installation commands; the launcher does not run sudo
or install system packages. See the [development environment](../../../docs/setup/development_environment.md)
for the regular Rust and native-library prerequisites.
The launchers check the workspace's minimum Rust version before setup. If the
active toolchain is older, install the reported version and select it for this
launch with `RUSTUP_TOOLCHAIN=1.98.0 ./twix --simulator --bhuman /hulks/1` (or the
same environment variable for `./simulator-mcp`). No global default is changed.

Inspect setup without downloads, cache writes or compilation:

```sh
./twix --simulator --bhuman --dry-run /hulks/1
python3 tools/simulate/bhuman/build.py --dry-run
python3 tools/simulate/tests/launcher_test.py
```

`./twix --help` describes launcher options; `./twix -- --help` forwards help to
Twix itself. Launchers can be invoked from any directory. Relative B-Human and
ONNX overrides resolve against the caller's directory.

In the simulator setup, set the opponent count above zero and select
**B-Human 2026**. The worker and source fields are filled from the launcher.
The setup, goal labels, scoreboard, referee controls and active-player tags use
B-Human's name, with B1–B5 for compact player badges.
To reuse a separately built worker, set both `BHUMAN_WORKER` and `BHUMAN_SOURCE`
before launching; this skips the native build.
`BHUMAN_SOURCE` alone builds the worker against that existing pinned checkout.
Set `MUJOCO_DYNAMIC_LINK_DIR` or `MUJOCO_STATIC_LINK_DIR` to use your own MuJoCo
3.9.0 build, or `MUJOCO_DOWNLOAD_DIR` to select its download cache. MuJoCo paths
must be absolute. Invalid paths fail before starting a build.
HULKs keep their selected estimator profile; B-Human uses perfect world models.
The MCP startup contract currently selects HULKs self-play.
`./twix --simulator` and `./simulator-mcp` use the same asset, MuJoCo and ONNX setup,
without building the native worker. If ONNX is unset they reuse the cached native
runtime when available, otherwise download only the pinned CPU runtime.
The legacy `./mujoco-simulator` still runs its Python application through `uv`;
it forwards arguments and reports how to install `uv` if missing.

For a separate worker build, run `python3 tools/simulate/bhuman/build.py`.
It prints the worker and source paths. Use `--cache PATH`, `--source PATH` to
reuse an existing release checkout, or `--jobs N` to set build concurrency.
`--runtime-only` installs ONNX Runtime without Git, CMake, Ninja or Clang;
`--print-cache` and `--print-ort` print resolved paths without setup. An incomplete
runtime installation is repaired from its checksum-verified archive on retry.
With Cargo directly, enable `--features bhuman`, pass
`--zenoh-namespace hulk_simulator`, and configure the worker paths, MuJoCo and
ONNX Runtime yourself.

The build pins B-Human commit `2e3374ba896da8288607f9a1881b876642fe18b8` and its
CompiledNN and SimRobot submodules, FlatBuffers 25.2.10 and ONNX Runtime 1.22.0.
Downloaded archives have SHA-256 checks. A CMake overlay adds the adapter and
replaces the native team socket implementation, without patching B-Human source.
The sparse checkout includes the native strategy and tactic configuration.
Ordinary HULK builds do not require the worker or FlatBuffers.

## Whistles, buttons and LEDs

The existing simulator whistle action broadcasts a detection to every robot.
For B-Human it sets `Whistle` with a logical timestamp, confidence 2 and two
listening channels. Native GameStateProvider decides whether to accept it,
including SET transitions and teammate whistles. Audio recognition stays excluded.

Body controls queue pressed/released states for native `KeyStates`. One edge is
consumed per cognition frame so a short tap between frames is preserved.
KeyStateEnhancer and the native game-state and behavior handlers measure holds
on simulation time. F1 enters Unstiff/Damping. Holding Stand for over two
simulated seconds activates the robot. Pressing Stand while active enters Prepare;
holding Walk for over two simulated seconds can leave Prepare when native checks
allow it. The progress bar freezes with simulation time. UI long-press event
labels do not override native timing. Players start active through B-Human's
own one-shot simulator startup request.

Native LEDHandler generates the RGB channel states. The simulator preserves
Off, On, Half and the normal/fast blinking variants, and renders them using the
same 400/80 ms clock divisions as BoosterProvider. The LED indicator freezes
while paused.

## Runtime and interfaces

`protocol.fbs` defines version 2 of the size-prefixed FlatBuffers protocol over a
private Unix socket. Initialization checks the pinned release and the ordered
22-joint list. Every request and reply carries a process epoch and sequence.
The host sends SI units; the adapter converts translations to native millimetres.
The serial mapping omits the T1 waist joint.

Motion executes every 2 ms; cognition executes every 20 ms. Both run synchronously
on the worker's IPC thread, with separate native blackboards and module graphs.
Their native representation packets carry cognition requests to motion and
motion feedback to cognition. RobotHealth carries the actual cognition heartbeat.
Physics snapshots and worker exchanges happen outside the shared physics mutex.

The host supplies field geometry through a temporary configuration layer,
including ball radius, so native tactics and the MuJoCo field use the same dimensions.
The pinned source checkout remains unchanged. World coordinates rotate with the
robot's GameController field side. Other robots retain team and player identities.

GameController v20 packets pass through native GameStateProvider, with penalties,
restart state and packet freshness intact. The host undoes the existing HULK
opponent identity translation before delivering packets to native team 5, and
sends return packets through the shared simulator network.

Native TeamMessageHandler retains its codec, clock synchronization and send-budget
policy. Its channel uses the worker connection instead of UDP. The host delivers
outgoing packets to other B-Human players on the same team after all robots finish
the current frame, and forwards packets for GameController budget accounting.
Private worker connections isolate simultaneous matches.

Actuation follows native calibration offsets, stiffness, motor gains, damping,
head-pitch feedforward and get-up torque control. Host torque is clamped to both
the returned limit and the model limit. Reset starts a fresh process and epoch;
pose resets clear native button and teammate queues. Exchanges have wall-time
deadlines, and failures stop the physics worker. Stop requires acknowledgment and
successful exit; drop kills and reaps the child. `Robot::step` is blocking.

A response must arrive within 30 wall-clock seconds, including both its size
prefix and body. Deadline errors report the operation, player, epoch, sequence,
simulation timestamp, elapsed time, byte progress and worker status before
cleanup. Older versions could report the same deadline as Linux's
`Resource temporarily unavailable (os error 11)`. A worker that is still running
at that point has not returned its response; this message alone does not explain
why. Preserve the full error and preceding worker stderr, and note the player
counts and whether it happened during setup, first Run, or later play.
The first Run also initializes native modules and their ONNX sessions.

## Verify

Fetch simulator Git LFS assets and configure MuJoCo as described in the
[simulator README](../README.md), then run:

```sh
export BHUMAN_WORKER=/path/to/bhuman-motion-worker
export BHUMAN_SOURCE=/path/to/BHumanCodeRelease
cargo test -p simulate --features bhuman --lib bhuman:: -- --include-ignored
cargo test -p simulate --features bhuman --lib robot_io::tests
cargo run -p simulate --features bhuman --bin bhuman-motion-smoke -- --soccer "$BHUMAN_WORKER" "$BHUMAN_SOURCE"
```

The tests exercise autonomous motion selection, whistle confidence, penalties,
button timing, native LED states, teammate encoding/decoding, mixed HULK/B-Human
physics, pause, independent workers, reset, shutdown, crashes and stale requests.
The physical soccer fixture starts in Initial, then switches to Playing with a
ball in MuJoCo. It requires movement without falling and reports displacement.
Omit `--soccer` for the explicit Prepare/Stand motion fixture. An optional final
argument sets the number of 2 ms frames.

After changing the schema, run the build script with `--generate-rust` to
regenerate checked-in Rust bindings. C++ bindings regenerate during native builds.

## Attribution

This product includes software developed by B-Human (http://www.b-human.de).
The fetched release retains its license and third-party notices. See its
`License.md` before redistribution or use in competition.
