# Tactical behavior evaluation — 2026-10-18

Experiments started on October 18 and continued on October 19.

This round extends `cd4d979836b987f5a39960c17bd6e3c1bb486bc0` on
`codex/hulks-behavior-improvements`, based on `alex/feat/bhuman-opponent-interface`.
The previous round fixed ownership and recovery but did not retain the proposed
kick/pass selector. Its one winning B-Human setup was insufficient evidence of
stronger general play. This round implements the tactical behavior itself.

The final comparison does not establish stronger general play or comfortable
wins. Native B-Human results are one win, one draw and two losses, with the same
aggregate 3–8 goals as the baseline. The branch retains independently tested
behavior corrections and records its adverse results below; it remains
experimental.


## Behavior changes

The kick/pass branch `pejotejo/ktsl-passing-motion-simulator` at
`8f25983997c20cae6555457cb33cc550c38171f5` is the starting point for obstacle-aware
shot openings, finite reception corridors, and pass preparation. These are adapted
to the existing HULKs tree and policy interface. No B-Human tactics are imported.

- Select and revalidate a visible shot, addressed pass, controlled advance,
  defensive clearance, inward cutback, or hold. An occupied shot corridor cannot
  fall back to an unchecked goal-center shot. Use physical goal posts once rather
  than treating padded navigation posts as ball obstacles.
- Treat travel range as a maximum. Intersect its target window with the field,
  shorten selected rays to the configured Euclidean limit and verify the result
  remains in bounds. The previous square-perimeter check incorrectly returned no
  six-metre clearance even on an empty Small field near midfield. Clear regions
  are clipped to forward progress before selecting their angular center, and
  ranked by usable opening angle times forward gain rather than width alone.
- After checking shots and acknowledged passes, a challenged striker in our half
  can release a long validated clearance instead of selecting a soft advance.
  The existing pressure, range and kick-speed settings are reused. Deep defensive
  emergency priority and indirect restart exclusions remain.
- In ordinary play, prefer a regular advance when a validated corridor gains
  more forward distance than the existing short-control range. Start this action
  with space; an already selected regular advance can continue under pressure.
  Keep soft advances for short openings, and retain shot and pass priority.
  This preserves the existing
  3.4 m/s kick for field progress rather than making every non-shot advance use
  the soft policy. Restarts requiring an inward release keep their own choices.
- Own corners, throw-ins and indirect free kicks enter the same tactical selector.
  Corners choose a receiver or put the ball back inside the field. They do not
  select a direct goal shot.
- A ball inside a noncoincident coarse Robot proxy may leave only along rays that
  never decrease its separation from any overlapping proxy. Physical post,
  person and unknown-obstacle collisions remain excluded. Robot proxies have no
  team identity; fresh teammate body access is checked separately. Hardware
  perception also uses coarse fixed-radius robot disks, which do not describe
  articulated feet. Requiring the ball center to remain outside the raw disk
  incorrectly rejected every action in a reproduced contested possession,
  despite an otherwise clear forward outward ray. Coincident centers and
  invalid geometry still fail closed.
- Walk behind the selected outgoing direction. Use a bounded, distance-faded
  pursuit prediction for rolling balls, and hand the observed position/velocity
  pair to the existing kick locomotion policy within its handoff range.
  Close, slow visual ball refinement requires a recent, consistent percept.
  The activation range is 1.1 m with 0.2 m hysteresis, replacing the old 1.3 m
  handoff. The policy owns final alignment, including close side/behind balls;
  a Kick request is not an instantaneous foot strike.
  A clear outgoing ball corridor is insufficient on its own: policy handoff also
  checks the body path against static obstacles, fresh teammates and rule areas.
  Dynamic robot footprints still restrict the outgoing ball corridor; the close
  policy can contest opponents rather than requiring walking clearance around
  them before takeover. This does not guarantee contact-free opponent contests.
  Planned
  approach endpoints are rechecked against original body, teammate, rule and ball
  footprints because planner destination projection can shrink other obstacles.
  Standoff and clearance use the same physical ball radius: 0.523 m in Small
  and 0.555 m in Middle with the current robot dimensions. This fixes a reproduced
  larger-ball refusal where a 0.500 m endpoint could never pass 0.505 m clearance.
  For forward rolling balls, the destination stays behind both the observed and
  predicted positions while preserving lateral pursuit. This fixes active Middle
  match refusals at 161 and 175 seconds where a clear route existed but prediction
  moved the normal endpoint into the observed ball's physical clearance circle.
- Urgent keeper clearances use that same coherent observation and target for
  eligibility and execution. They require feasible forward, heading and lateral
  alignment; otherwise the keeper continues through its existing interception,
  pursuit and defense branches.
- Resolve nearby ambiguous striker claims with an explicit deterministic winner;
  preserve the territorial Voronoi support map. Add fresh teammate poses as missing
  path obstacles. Support cells require footprint clearance and planned routes,
  and favor receiving lanes while leaving the shot lane clear.
  The support score retains a monotonic penalty beyond the preferred ball
  distance, so distant supporters still retreat toward useful play. A Gaussian
  reward alone became negligible over their whole region and left them upfield.
- When every outgoing corridor is blocked, the striker plans a position on the
  ball-to-own-goal lane instead of standing far from a contested ball. It faces
  the current observed ball, preserves clearance and restart restrictions, and
  stands when close or when no safe planned destination exists.
- Carry pass intent and receiver readiness in normal budgeted state messages.
  The receiver must face the ball, be upright and in place. The sender requires a
  fresh addressed acknowledgement after its announcement. Preparation has a
  timeout and cooldown; unusable message budgets choose another action.
  Under the existing pressure threshold, only a pass with a fresh
  matching acknowledgement stays eligible. The striker chooses another action
  instead of starting a receiver-preparation wait with a challenger close by.
  A locally observed launched pass remains reserved after the sender withdraws
  its offer. The receiver holds its position until the measured ball slows below
  0.5 m/s near reception, or has passed and needs recovery. Repeated completed
  offers cannot restart the same reservation.
- After releasing a restart, a field taker holds its release position; a keeper
  returns home. An ordinary support route can catch a slowing released ball, as
  the corner fixture reproduced, so field takers do not follow that route yet. The taker
  retains this restriction through missing ball observations until another robot
  is observed in physical contact range, or a new play resets it. Fresh teammate
  poses alone do not count as contact. This addresses the repeated-kicker foul
  reproduced in the initial corner fixtures.
  Contact is inferred from local ball/robot geometry rather than a simulator
  contact-event feed.
- On our kickoff, non-takers watch the ball from their legal starting poses until
  a fresh own observation shows movement over 0.3 m. Release is latched through a
  return to center; a new Ready/Set, different play or clock reset clears it.
  The taker and keeper retain their normal branches. Permission for our team to
  kick is not permission for supporters to cross halfway before ball movement.

Legacy keeper and set-play kick paths retain a prior tactical policy only during
the existing motion commitment interval. After expiry they restore current
configured soft/quick/speed settings and strength selection. V14 Middle snapshots
at 35/37/39 s exposed a keeper retaining a 1.5 m/s soft pass at elapsed
0.58/2.58/4.58 s despite a 0.5 s commitment. The fix checks actual elapsed time;
a stale cached diagnostic duration cannot extend that commitment indefinitely.

A valid observed non-upright state is announced through an unavailable-state
envelope on the ordinary message cadence. Teammates retain the fresh pose as a
body obstacle, but omit that robot from territorial/local ball ownership and
pass receiver selection. A subsequent available state restores eligibility;
old or duplicate receipts cannot undo a newer availability update. Missing or
expired fall information keeps the existing unknown-state behavior.
This addresses V14's defensive start: a fallen H2 continued sending fresh poses
and owned the ball cell while upright H3/H4 waited through most of 39–121 s.
The fix does not change get-up motion or assume recovery has succeeded.

Regular kicks remain 3.4 m/s, with strong kicks disabled. Controlled actions use
existing soft-kick policy speeds (short advance 1.2, pass 1.5, cutback 1.6 m/s).
Physics, ball contacts, learned motion models, native opponent and perception are
unchanged. The Bevy behavior harness mirrors receipt times and per-tick lifecycle
state for the new behavior.

## Communication and deployment

Deploy this protocol to the whole team. `State` and `CoordinatedState` retain
their previous bincode representations; `UnavailableState` is appended to the
message enum. All envelopes fit the 128-byte team packet limit. Older binaries
discard envelopes they do not understand, including the embedded pose/ball.
Setting `kicking.strategy.passing.maximum_distance` to zero disables passing,
but does not suppress availability updates, so it does not provide full mixed
team compatibility.

Receipt timestamps are local receive times, not claimed sender clocks. Old and
duplicate cached receipts cannot restore a cancelled intent; future receipts are
rejected. Existing GameController message-rate budgeting is retained. The pass
cadence must fit both the preparation timeout and acknowledgement freshness.

## Validation method

The frozen baseline is the previous round's final candidate, including 3.4 m/s
kicks. Each match runs 180 simulated seconds in `motion_behavior` with automatic
refereeing and exact active rosters: Small Foundation 4v4 or Middle Foundation
3v3. The matrix includes reflected/perturbed openings and an explicit opponent
possession start. B-Human is the unchanged pinned native worker. HULKs self-play
uses the selected executable for both teams, so it screens for regressions but is
not a match against a frozen old HULKs opponent.

Controlled fixtures hold irrelevant robots in Stand, settle actual teammate
messages, then release the relevant players. They sample every 0.2 seconds and
exercise corners, passing, crossing/receding balls and competing strikers.
A distant receding-ball probe exercises the walking approach before handoff.
Additional fixtures exercise blocked possession, distant defensive support and
the actual Middle 0.105 m ball with a 3v3 roster.
Scene-center distances are proximity measurements, not contact-event labels.

## Final controlled fixture results

The frozen baseline is `e142feb4733fec52d9f6da7e848425b8da3769cde34a8449f56bb4c159d65b47`;
the final V16 candidate is
`ca9d19ad14e1f2f4da110538efb3f88f7a0f5b2e59511d4d7627406dedc1f3b3`.
Each fixture is one 16-second run per version at 0.2-second sampling.

| Fixture | Baseline → final candidate | Score, baseline → candidate |
| --- | --- | --- |
| Corner | IFK 3.4 s → none; nonupright samples 41 → 0; ball movement over 0.2 m at 1.2 → 2.4 s. | 0–0 → 0–0 |
| Corner with receiver | IFK 3.6 s → none; movement at 1.4 → 3.8 s; minimum teammate center separation 0.502 → 0.796 m. | 0–0 → 1–0 |
| Passing lane | Non-positive ball–aim-dot request samples 11 → 0; simultaneous Kick samples 15 → 0; minimum separation 0.441 → 0.997 m. | 0–0 → 1–0 |
| Crossing ball | First Kick request 0.6 → 0.6 s; non-positive ball–aim-dot request samples 0 → 1. | 1–0 → 1–0 |
| Receding ball | First Kick request 0.6 → 0.6 s; non-positive ball–aim-dot request samples 3 → 1. | 1–0 → 0–0 |
| Distant receding ball | First Kick request 2.0 → 2.8 s; Stand samples 0 → 0. | 1–0 → 1–0 |
| Sideways alignment | First policy Kick request 0.6 → 0.6 s; first ball movement over 0.2 m at 2.0 → 2.6 s. | 1–0 → 0–0 |
| Ball behind robot | First policy Kick request 0.6 → 0.6 s; movement over 0.2 m at 3.6 → 3.4 s. | 1–0 → 1–0 |
| Competing strikers | Simultaneous Kick samples 5 → 0; non-positive ball–aim-dot requests 3 → 0; minimum separation 0.322 → 0.625 m. | 1–0 → 1–0 |
| Blocked possession | Held opponent limits this fixture to execution and goal-side cover; inspect snapshots rather than treating its score as strength. | 0–0 → 1–0 |
| Defensive support | Final supporter-to-ball center distance 5.7 → 1.4 m; held keeper prevents a live-opponent clearance assessment. | 0–0 → 0–0 |
| Actual Middle ball | First Kick request 0.8 → 1.0 s with the physical 0.105 m ball and 3v3 roster. | 1–0 → 1–0 |

Across the final fixtures, the candidate has 15 requests with a non-positive ball–aim dot product,
0 simultaneous Kick samples and 34 nonupright samples. Center separations
are proximity measurements, not proof that every physical contact was avoided.
Kick selects a locomotion policy that walks and aligns, so aim-angle and
ball–aim-dot request metrics do not identify physical mis-hits. Kick request time
is a behavior command, not measured foot-contact time.

Maximum fixture behavior receipt age is 98 ms on the simulation clock;
maximum source clock offset spread is 0 ms. `fixtures-comparison-v16.json`
records exact metrics, GameController timelines, hashes and resolved artifact
paths. Earlier V7/V8 results remain separate diagnostic evidence. Only
`base/behavior_node.json5` differs between the 48 frozen parameter files.

The passing fixture uses the real message handshake. Held opponents make these
controlled fixtures unsuitable as opponent-strength benchmarks. Slow restart
contact, any lost fixture goals and blocked execution remain adverse results;
see the score and timing rows above rather than assuming every change improved.

Both corners avoid the baseline retouch indirect free kicks, and the receiver
corner scores at 10.8 s. The corner receiver waits through the launched ball
flight and enters Kick at 5.2 s with measured speed 0.448 m/s. Passing-lane and
competing-striker simultaneous Kick samples fall from 15 and 5 to zero; minimum
center separation increases from 0.441/0.322 m to 0.997/0.625 m. These are useful
coordination results, with slower policy handoff or ball movement in several cases.

The passing-lane goal at 9.2 s is not a completed receiver finish: the receiver
holds through launch, resumes Walk at 4.4 s with speed 0.470 m/s, and never enters
Kick. The sender subsequently advances and shoots. All 34 nonupright fixture
samples occur after that goal, once the referee is Ready; they remain adverse
posture observations and do not demonstrate an in-Playing availability handover.

Rolling-ball precision is not established. Crossing scores at 7.6 s versus 6.4 s;
distant pursuit enters Kick 0.8 s later and scores at 6.8 s versus 5.6 s.
Receding and sideways cases lose baseline goals, ending 0–0 rather than 1–0;
the receding case produces an opponent goal kick at 8.6 s. The actual Middle
ball does score, but at 11.8 s versus 4.6 s. Behind-ball alignment scores at
11.0 s versus 11.8 s, while competing strikers score at 6.2 s versus 4.6 s.

The blocked-possession goal at 15.8 s follows the held opponent's LocalGameStuck
penalty at 10.2 s and is not evidence of beating a live defender. Defensive
support gets substantially closer to the ball but never clears it. These limits
remain alongside the successful legality and coordination checks.


## Match results

Each entry is a single unseeded 180-second run, scored HULKs–opponent.
Playing exposure uses any separately sampled own robot primary state Playing,
not the delayed raw GameController broadcast. Referee pauses reduce exposure.
Policy, posture and proximity statistics use the corresponding blackboard's
primary state; independently received topics can disagree at state boundaries.

| Setup | Baseline | Final candidate | Sampled Playing seconds, baseline → candidate |
| --- | --- | --- | --- |
| B-Human · Small opening | 0–3 | 0–3 | 106 → 116 |
| B-Human · Middle opening | 0–2 | 1–1 | 108 → 104 |
| B-Human · Small variation | 2–1 | 0–3 | 102 → 102 |
| B-Human · Small defense | 1–2 | 2–1 | 60 → 72 |
| HULKs · Small self-play | 1–0 | 0–1 | 142 → 142 |
| HULKs · Middle self-play | 0–1 | 1–0 | 128 → 136 |

Self-play changes both teams and is not a frozen-old-opponent benchmark. These
short runs do not estimate a win rate, and asynchronous scheduling prevents a
causal attribution of score differences to one particular behavior change.

The four native cases produce one win, one draw and two losses, compared with
one win and three losses for the baseline. Aggregate goals are unchanged at
3–8. Middle and defensive starts improve in these samples, but the baseline's
2–1 variation win becomes a 0–3 loss. Comfortable wins or stronger general play
have not been established. The retained branch is an experimental behavior
change with specific correctness fixes, not a validated competitive upgrade.

The final native runs have no sampled own penalty onsets. The defensive case
contains 13 nonupright Playing robot-samples; these are not distinct-fall counts.
Small self-play incurs one IllegalPosition onset and Middle self-play incurs
three. The two self-play scores reverse the baseline's outcomes while changing
both teams, so they cannot establish strength against the old HULKs behavior.

The new availability signal is exercised in the defensive match: H4 falls at
simulation time 11 s, peers report it unavailable by 13 s, and upright teammates
continue selecting and executing ball actions while H4 remains down. All nine
sampled legacy keeper Kick commands across the six matches use the regular
3.4 m/s policy. This supports the two lifecycle fixes; sparse snapshots and
unseeded scheduling do not assign the match-score changes to them.



Exploratory candidates did not establish a match-strength gain. V4's Small
B-Human opening ended 0–4, against the frozen baseline's 0–3. Its snapshots
exposed distant Hold and upfield defensive support; the revised behavior adds
goal-side cover and a monotonic long-range attraction to the ball.

V6's first three native setups were 0–3, 0–2 and 0–3. They exposed the larger-ball
standoff mismatch and a kickoff support penalty. The delayed raw Set broadcast
was current and intentional: the referee had already entered Playing, where
non-taker restrictions still applied. The fix observes ball release instead of
waiting for that delayed broadcast. A separate one-parameter V6 trial increased
the pressure-clearance zone from 1.8 to 3.0 m, ended 0–2 and produced no goals.
That single unseeded result was insufficient evidence of better general play;
the final value remains 1.8 m.

V7's first three native setups each ended 0–4. The fourth was deliberately
interrupted after an independent trace analysis found the forward-pursuit
approach refusal. The frozen V7 binary and completed/partial runs remain
separate diagnostic evidence. V8 changes only that destination calculation;
it preserves the previous collision checks and kick handoff gates. Its Small
opening ended 0–2, which is only one unseeded observation. V9 also
addresses a directly observed loss at 113 s in V7: a striker initiated an
unacknowledged pass despite an opponent foot gap of 0.733 m, inside the existing
0.8 m pressure threshold. It stood with the ball 0.22 m away; the opponent closed
and scored. V9 permits an already acknowledged pass under pressure, but avoids
starting that wait.

The three completed V9 native setups each ended 0–4. This regression prompted
an audit of the handoff assumption: our existing Kick/SoftKick models are
locomotion policies, and the reference HULKs passing branch explicitly leaves
final alignment to them. In the Small opening, V9 issued only 6 Kick requests
across 160 Playing robot-samples, versus the baseline's 55 across 212; near the
ball it spent substantially more samples walking. Requiring a pre-aligned pose
had duplicated work that the policy already performs. The revised handoff keeps
its ordinary distance band and current corridor/body/rule checks, while allowing
the policy to align close side/behind balls. Out-of-range interception remains
bounded. Large aim angles and non-positive ball–aim dot products in requests are
descriptive geometry, not physical mis-hit labels.

V10's corrected alignment handoff still ended 0–4 in its Small trial. V11
restored ordinary opponent contests at policy handoff while retaining teammate,
static and rule checks. It increased Playing Kick requests to 19/158 (V10 9/140,
baseline 55/212), but also ended 0–4 and incurred an IllegalPosition penalty after
an opponent throw-in. The sparse preceding path cleared the published rule
obstacle, so its intermediate breach was not established. These are adverse
results, not evidence of a competitive gain.

The V12 selector correction addresses independently reproduced maximum-range
clipping and own-half pressure-release opportunities. It retains regular 3.4 m/s
kicks and the existing controlled speeds rather than fitting new motion or
contact parameters to those losses.

V12's full native matrix ended 0–4, 0–3, 0–4 and 1–1, compared with the baseline's
0–3, 0–2, 2–1 and 1–2. Its self-play runs ended 2–1 and 0–0; both teams change in
those runs. The native results did not establish improved strength. Their full
evidence is retained in `results-v12.json` and `v12-archive-index.json`.

In that V12 Small opening, a striker issues Clear/Kick at 13 and 15 s while the
ball moves toward our goal; at 17 s the raw proxy overlap forces Hold and the
opponent scores at 19 s. On the following own kickoff, Dribble/Kick at 45 s is
followed by a return toward our goal at 47 s and a goal at 49 s. Two-second
snapshots establish commands and trajectories rather than exact foot contacts.
V12 has 16 Kick requests over 156 blackboard-Playing robot-samples, versus baseline 55/212.
That reduction is descriptive: baseline simultaneous requests are not necessarily
useful possession, and the Middle case did not show lower normalized policy use.

V12 then exposed the remaining raw-proxy-overlap rejection. At 17 s in its Small
opening, the observed ball lay 0.163 m from a Robot proxy center, inside its
0.2 m raw radius. This alone rejected every action before testing directions.
A forward target gaining 0.635 m increased separation from that proxy throughout
the ray and cleared the other supplied ball obstacles, teammate bodies and rule
areas. The retained correction permits only non-inward release from initial
Robot proxy overlap; it leaves other obstacle shadows and all ordinary body,
walking, teammate and rule checks in place. This is a geometry correction,
not evidence that the subsequent kick will physically succeed.

The full V13 native matrix ended 0–4, 0–4, 0–3 and 0–2; self-play ended 0–2 and
1–0. It did not establish a competitive gain. The native defensive case incurred
a LocalGameStuck penalty, and Middle self-play incurred MotionInStop. In Middle,
20 Kick samples included only four regular kicks, thirteen soft advances and
three soft passes, compared with eighteen regular baseline kicks. The following
selector revision restores regular kicks for validated long forward advances,
while retaining soft control for short space and passing. It requires forward
gain beyond the existing dribble range, so a long lateral target near the goal
does not trigger the ordinary advance rule. It changes action priority rather
than motion parameters. `results-v13.json` and `v13-archive-index.json` retain the
complete preceding evidence.

V14's complete native matrix finished 0–3, 0–3, 0–4 and 1–0; self-play finished
2–1 and 0–0. It still did not establish stronger general play. Its defensive win
included prolonged failed stand-up and unavailable-owner waiting, so the score
must not be treated as proof of effective defensive coordination. The subsequent
changes address the directly observed legacy kick-policy leak and unavailable
owner handoff, rather than changing motion/physics or searching kick speeds.
`results-v14.json`, `fixtures-comparison-v14.json` and `v14-archive-index.json`
retain the complete superseded evidence.

## Checks

The focused Rust suite passes 214 tests across the behavior node, team protocol,
message consumers, obstacle filter and Bevy behavior harness:

```sh
cargo +1.98.0 test -p behavior_node -p hsl_network_messages -p message_filter \
  -p player_states_receiver -p search_suggestor -p search_heatmap \
  -p obstacle_filter -p bevyhavior_simulator -j 2
```

The simulator executable also builds with the `bhuman` feature. Targeted semantic,
communication lifecycle and configuration reviews identified and fixed blocked
ownership overrides, stale receipt resurrection, acknowledgement cadence beyond
its freshness limit, premature receiver release and the restart taker's route
catching its own released ball.

## Reproduction

Artifacts live in `logs/behavior-tactics-20261018/`: frozen parameters and binaries,
per-run source runners, hashes, scene/blackboard snapshots, logs and summaries.
Runs use MuJoCo 3.9.0 and the existing ONNX Runtime library. The native B-Human
worker is pinned to `2e3374ba896da8288607f9a1881b876642fe18b8`.
The experiment-only native MCP adapter from the previous report is restored out
of the production source after building the saved binary.

The saved wrappers set the local runtime/library paths and preserve a runner copy,
arguments, parameter tree and binary hash for every run. They can produce a fresh
repeat without overwriting the retained evidence:

```sh
python3 logs/behavior-tactics-20261018/run_tactics_matrix.py candidate --run-prefix repeat-
python3 logs/behavior-tactics-20261018/run_tactics_fixtures.py candidate --run-prefix repeat-
```

The artifact root and checkout paths in these local scripts are explicit. Adjust
them when moving the evidence to another machine. `candidate-source.patch` and
`candidate-source-manifest.json` associate the frozen executable with its changed
source and base commit; the temporary native adapter is recorded separately.

These simulations use ideal pose, ball and obstacle inputs. Asynchronous physics
and node scheduling make repetitions nondeterministic, and referee pauses reduce
playing time. Hardware, vision and localization validation remain separate work.
The learned kick's actual ball direction can differ from the requested corridor;
an earlier crossing-ball diagnostic was about eight degrees straighter than its
selected opening. This work does not claim to repair or calibrate kick realism.
