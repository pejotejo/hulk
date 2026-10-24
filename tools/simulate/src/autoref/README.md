# Referee corrections and penalty shootouts

Click a recent decision in the referee desk to reverse that decision and every
newer decision. The last 16 decision groups remain available for two simulation
minutes. A group includes the controller actions and physical handling produced
by one operator call or physics observation; clicking any entry in a group
reverses the entire group. Reversed entries stay visible with a strikethrough.

Corrections restore scores, state, sides, cards, penalties, timers and pending
handling. Robots and the ball return to grounded, stationary placements. Play
stops for correction; use **Resume play** when ready. Simulation time continues,
restart observation is reconstructed, and earlier physics trajectories or heard
whistles are not replayed backwards. The engine retains small controller/handling
checkpoints, never complete Engines or MuJoCo worlds. The regular update reuses
its previous observation for the checkpoint rather than cloning it a second time.

The desk's **Penalty shootout** controls choose the +X or −X goal. Setup also
works before full time for practice, resets scores to zero, and sets the first
shot for HULKs. Each team needs a present robot that has not been sent off, and
the scene needs a ball. By default player 2 attacks when present and player 1
defends; use **Select for shot** on a player in Set to change that shot's striker
or goalkeeper. All other robots become substitutes and move beyond the carpet.
The striker approaches from behind the penalty spot, facing the goal; the
goalkeeper starts on the goal line facing the striker. Placements use the
selected field dimensions. Both teams shoot at the chosen physical goal.

**Whistle** starts the shot. Goals, exits, a stopped ball after a touch, and the
shot time limit use the controller's automatic rules when automatic calls are
enabled. **Finish shot** manually records a miss. **Set next shot** alternates the
teams until the controller decides the series is complete. Scores, shot counts
and the scoring mask reach GameController packets; red cards survive selection
and alternation. Normal kickoff automation is disabled during a shootout.

## Live commands

The existing `simulator_input` referee adapter accepts these additional calls.
Use a fresh outer `request_id` for each mutation, as described in the simulator
MCP guide. The command argument schemas are exposed through
`server_info.simulator_input_schemas.referee` by `autoref::command_schemas()`.

| `name` | `arguments` | Effect |
| --- | --- | --- |
| `shootout` | `{"goal":"positive_x"}` or `{"goal":"negative_x"}` | Start the fresh series and set its first shot. |
| `shot_player` | `{"team":"hulks","number":2}` | Select this team's striker or goalkeeper in Set; `opponents` is also accepted, numbers 1–5. |
| `whistle` | `{}` | Start the set shot. |
| `goal` | `{"team":"hulks"}` | Award a goal to the shooting team and finish its shot. |
| `finish_shot` | `{}` | Finish the current shot without scoring. |
| `next_shot` | `{}` | Set the next shot if the series can continue. |
| `reverse_decision` | `{"decision":42}` | Reverse the engine decision ID and every newer group. |
| `resume_play` | `{}` | Continue after a correction or stop. |

For example, the inner input object for setup is
`{"kind":"referee","name":"shootout","arguments":{"goal":"positive_x"}}`.
Decision IDs are carried on `Engine::events[*].decision`; the current MCP adapter
does not expose the decision log, so choose reversals from the UI or an in-process
caller that can inspect these IDs. The command schema rejects unknown properties
and requires every argument shown in the table.

The existing match strip and `N` shortcut follow shot setup, whistle and finish
steps without extra toolbar integration.

Run the engine transition tests with
`cargo test -p simulate --lib autoref::engine::tests` in the configured simulator
environment. They cover reversal of scores/cards/handling, bounded history,
both goal choices across all three HSL field sizes, red cards, shot outcomes,
completion, and refusal of incomplete scenes.
