# Kick and get-up requests

Behavior publishes `types::motion_command::MotionCommand` on
`behavior/motion_command`. Its `Kick` and `StandUp` variants carry policy choices
through the motion node to inference. Motion publishes `RobotCommand` on
`commands/robot_command`, containing the desired control mode and joint commands.

## Behavior inputs

`BodyMotion::Kick` and `MotionCommand::Kick` carry:

- `ball_position`: observed ball position in Ground coordinates, in metres.
- `ball_velocity`: current ball velocity in Ground coordinates, in m/s.
- `kick_direction`: desired kick orientation in Ground coordinates.
- `target_speed`: desired outgoing ball speed in m/s.
- `soft`: select the soft-kick policy.
- `quick`: enable the normal kick policy's quick flag.
- `strong`: enable the normal kick policy's strong flag, requesting maximum speed.

Behavior stores its selected target in `Blackboard::kick_target` for strength
selection. The kick command carries the resulting direction and inference
inputs. `last_kick_target` holds the Field target across ticks for remote and
penalty kicks.

The behavior `kick` action uses a coherent tracked position/velocity pair. A
recent, consistent visual percept may refine a close, slowly moving ball; it
does not replace the position of a fast tracked ball. Interception and walking
pursuit compute future points locally while retaining the observed position and
current velocity in the policy request. `Kick` selects a locomotion policy that
owns the final approach and alignment, including close balls beside or behind
the robot. A request with a large aim angle is not evidence of a physical mis-hit.

Initial choices come from `behavior_node.kicking.soft`, `.quick`, and
`.target_speed` (defaults: `false`, `false`, and `3.4`). Target speed is configured
independently of target distance. A long validated forward advance uses the
regular kick. Short advances, passes and inward cutbacks use separate soft-kick
speeds (1.2, 1.5 and 1.6 m/s). A continuing
kick retains its policy and speed until switching is allowed, while refreshing
position, velocity and a still-clear aim. Goalkeeper and penalty actions can
override the strong flag explicitly.

`StandUp` selects `GetUp`.
It uses position targets and motor-side PD gains configured under
`motion_inference.policies.GetUp`.

## Behavior selection parameters

These parameters belong to `behavior_node.kicking`. All distances are in metres.

| Parameter | Base default | Purpose |
| --- | --- | --- |
| `allow_strong_kicks` | `false` | Permit strong striker and penalty kicks; this does not limit target speed. |
| `strong_kick_min_target_distance` | `6.0` | Select strong striker kicks at or beyond this ball-to-target distance. Penalty selection only checks permission. |
| `kick_activation_distance` | `1.1` | Centre of the normal striker's walk-to-kick handoff band; outgoing corridor and body access must also be clear. |
| `kick_activation_hysteresis` | `0.2` | Total width of that band: enter below 1.0, leave above 1.2. |
| `approach_ball_standoff` | `0.3` | Minimum walking standoff; approach also preserves robot plus physical ball clearance and a 0.05 m margin. |
| `minimum_interception_forward_distance` | `0.3` | Minimum predicted interception distance along the outgoing direction for the striker, or along Ground x for the goalkeeper. |

`disable_strong_kick` clears the strong flag without selecting the soft policy
or lowering `target_speed`. The `kick_strength_subtree` retains settings during the current motion
commitment interval. Once switching is allowed, legacy keeper and set-play
actions use their current configured policy and speed; a prior soft pass must
not keep those actions on the soft policy indefinitely. Goalkeeper actions disable strong explicitly.

Ordinary handoff checks static obstacles, fresh teammate poses and rule areas.
Observed robot footprints still block outgoing ball corridors, but do not impose
walking clearance on a close opponent contest. Walking and Hold continue to
check every robot footprint. This preserves the existing policy's contest role;
it does not imply that opponent contact is eliminated.
An initial overlap with a coarse Robot proxy permits only outgoing rays that
never reduce separation from that proxy. Coincident centers, static collisions,
fresh teammate body access and rule restrictions remain excluded. These proxies
have no team identity and do not represent articulated foot geometry.

`kicking.approach.maximum_heading` and `.maximum_lateral_distance` bound the
additional incoming-ball handoff beyond the ordinary distance band and urgent
keeper eligibility. They do not require normal close-ball requests to be aligned.
`.maximum_ball_escape_speed` is currently unused. The locomotion policy receives
the coherent measured velocity for ordinary close rolling balls.

Strength selection and kick direction use the same target position. The separate
`substates.distance_for_kick` and `substates.distance_for_kick_hysteresis`
parameters control proximity to the set-play alignment position.

## Inference and simulation

Motion forwards the fields without applying policy limits. Inference caps ball
velocity and clamps target speed to the selected policy's limits. Strong normal
kicks use the maximum speed regardless of `target_speed`. Soft kicks ignore the
strong and quick flags. Base speed limits are `[0.5, 3.4]` m/s for normal kicks and
`[0.1, 1.7]` m/s for soft kicks.

Inference caps the ball-position observation at 2.0 m. Ordinary behavior handoff
leaves Kick above 1.2 m with the base configuration. The observation cap does not
establish the policy's physical reach.

The Bevy behavior simulator approximates the speed rules with configurable limits.
It does not model the learned quick-kick movement or recovery timing;
get-up clears the simulated fallen state immediately.

## Serialized commands

Injected kick commands use the `Kick` variant with all fields supplied explicitly.
Stand-up commands use `"StandUp"` without selector fields. Get-up inference
requests carry an empty `command: {}` inside the existing reset/deadline envelope.
Clients and parameter overrides must use the updated schema.
