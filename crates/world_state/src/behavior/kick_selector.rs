use std::time::{Duration, SystemTime};

use coordinate_systems::Field;
use geometry::{circle::Circle, line_segment::LineSegment};
use hsl_network_messages::{CoordinationIntent, PassIntent, PlayerNumber, Team};
use linear_algebra::{Point2, distance, point};
use serde::{Deserialize, Serialize};
use types::{
    field_dimensions::FieldDimensions, filtered_game_state::FilteredGameState,
    motion_command::KickPower, obstacles::ObstacleKind, parameters::KickingParameters,
};

use crate::behavior::node::{Blackboard, LastBall};

const FIELD_MARGIN: f32 = 0.3;
const PASS_RECEIVER_OBSTACLE_ECHO_RADIUS: f32 = 0.15;
const PASS_RECEIVER_REACH_MARGIN: f32 = 0.5;

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize)]
pub struct KickMemory {
    pub target: Option<Point2<Field>>,
    pub close_enough_to_kick: bool,
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize)]
pub struct CycleIntent {
    pub role: TacticalRole,
    pub kick: Option<KickIntent>,
    pub rule_constraints: RuleConstraints,
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
pub enum TacticalRole {
    #[default]
    Supporter,
    Striker,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct KickIntent {
    pub kind: KickKind,
    pub ball_position: Point2<Field>,
    pub target: Point2<Field>,
    pub power: KickPower,
    pub receiver: Option<PlayerNumber>,
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
pub enum KickKind {
    #[default]
    Shoot,
    Touch,
    Clear,
    Pass,
}

#[derive(Clone, Copy, Debug)]
struct KickCandidate {
    target: Point2<Field>,
    kind: KickKind,
    power: KickPower,
    score: f32,
    receiver: Option<PlayerNumber>,
    ignored_obstacle_point: Option<Point2<Field>>,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct RuleConstraints {
    pub may_kick_ball: bool,
    pub may_score_directly: bool,
    pub may_pass: bool,
}

impl Default for RuleConstraints {
    fn default() -> Self {
        Self {
            may_kick_ball: true,
            may_score_directly: true,
            may_pass: true,
        }
    }
}

pub fn rule_constraints(blackboard: &Blackboard) -> RuleConstraints {
    let Some(filtered_game_controller_state) =
        &blackboard.world_state.filtered_game_controller_state
    else {
        return RuleConstraints {
            may_kick_ball: true,
            may_score_directly: false,
            may_pass: true,
        };
    };

    match filtered_game_controller_state.sub_state {
        Some(sub_state) => match filtered_game_controller_state.kicking_team {
            Some(Team::Hulks) => RuleConstraints {
                may_kick_ball: true,
                may_score_directly: false,
                may_pass: sub_state != hsl_network_messages::SubState::PenaltyKick,
            },
            Some(Team::Opponent) | None => RuleConstraints {
                may_kick_ball: false,
                may_score_directly: false,
                may_pass: false,
            },
        },
        None => match filtered_game_controller_state.game_state {
            FilteredGameState::Playing {
                ball_is_free: true, ..
            } => RuleConstraints {
                may_kick_ball: true,
                may_score_directly: true,
                may_pass: true,
            },
            _ => RuleConstraints {
                may_kick_ball: false,
                may_score_directly: false,
                may_pass: false,
            },
        },
    }
}

pub fn select_cycle_intent(
    blackboard: &Blackboard,
    memory: KickMemory,
    selected_striker: Option<PlayerNumber>,
) -> CycleIntent {
    let rule_constraints = rule_constraints(blackboard);
    let role = if selected_striker == Some(blackboard.world_state.robot.player_number) {
        TacticalRole::Striker
    } else {
        TacticalRole::Supporter
    };
    let kick = if rule_constraints.may_kick_ball {
        blackboard.ball.as_ref().and_then(|ball| {
            let mut candidate_constraints = rule_constraints;
            if role != TacticalRole::Striker {
                candidate_constraints.may_pass = false;
            }
            select_best_candidate(blackboard, ball, memory, candidate_constraints)
                .map(|candidate| KickIntent {
                    kind: candidate.kind,
                    ball_position: ball.position,
                    target: candidate.target,
                    power: candidate.power,
                    receiver: candidate.receiver,
                })
                .or_else(|| {
                    if rule_constraints.may_score_directly {
                        fallback_goal_intent(
                            blackboard.field_dimensions,
                            ball,
                            &blackboard.parameters.kicking,
                        )
                    } else {
                        fallback_touch_intent(
                            blackboard.field_dimensions,
                            ball,
                            memory,
                            &blackboard.parameters.kicking,
                        )
                    }
                })
                .filter(|kick| {
                    !is_lane_blocked(
                        blackboard,
                        kick.ball_position,
                        kick.target,
                        (kick.kind == KickKind::Pass).then_some(kick.target),
                    )
                })
        })
    } else {
        None
    };

    CycleIntent {
        role,
        kick,
        rule_constraints,
    }
}

pub fn outgoing_coordination_intent(
    _now: SystemTime,
    cycle_intent: CycleIntent,
) -> Option<CoordinationIntent> {
    if cycle_intent.role != TacticalRole::Striker {
        return None;
    }

    let kick = cycle_intent.kick?;
    if kick.kind != KickKind::Pass {
        return None;
    }

    Some(CoordinationIntent::Pass(PassIntent {
        sequence: 0,
        receiver: kick.receiver?,
        receive_point: kick.target,
        age: Duration::ZERO,
    }))
}

fn select_best_candidate(
    blackboard: &Blackboard,
    ball: &LastBall,
    memory: KickMemory,
    rule_constraints: RuleConstraints,
) -> Option<KickCandidate> {
    let mut best_candidate: Option<KickCandidate> = None;
    let parameters = &blackboard.parameters.kicking;

    for (target, kind) in candidate_targets(
        blackboard.field_dimensions,
        ball.position,
        rule_constraints.may_score_directly,
        parameters,
    ) {
        let Some(candidate) = kick_candidate(ball.position, target, kind, memory, parameters)
        else {
            continue;
        };
        if is_lane_blocked(
            blackboard,
            ball.position,
            candidate.target,
            candidate.ignored_obstacle_point,
        ) {
            continue;
        }
        if match best_candidate {
            Some(best_candidate) => candidate.score > best_candidate.score,
            None => true,
        } {
            best_candidate = Some(candidate);
        }
    }

    if rule_constraints.may_pass {
        for candidate in pass_candidates(blackboard, ball.position, memory, parameters) {
            if is_lane_blocked(
                blackboard,
                ball.position,
                candidate.target,
                candidate.ignored_obstacle_point,
            ) {
                continue;
            }
            if match best_candidate {
                Some(best_candidate) => candidate.score > best_candidate.score,
                None => true,
            } {
                best_candidate = Some(candidate);
            }
        }
    }

    best_candidate
}

fn candidate_targets(
    field_dimensions: FieldDimensions,
    ball_position: Point2<Field>,
    may_score_directly: bool,
    parameters: &KickingParameters,
) -> Vec<(Point2<Field>, KickKind)> {
    const DIAGONAL: f32 = std::f32::consts::FRAC_1_SQRT_2;

    let mut targets = Vec::new();

    if may_score_directly {
        let goal_x = field_dimensions.length / 2.0;
        let goal_side_y = field_dimensions.goal_inner_width / 3.0;
        targets.push((point!(goal_x, 0.0), KickKind::Shoot));
        targets.push((point!(goal_x, goal_side_y), KickKind::Shoot));
        targets.push((point!(goal_x, -goal_side_y), KickKind::Shoot));
    }

    for (distance, directions, kind) in [
        (
            1.0,
            [(1.0, 0.0), (DIAGONAL, DIAGONAL), (DIAGONAL, -DIAGONAL)],
            KickKind::Touch,
        ),
        (
            long_kick_distance(parameters),
            [(1.0, 0.0), (DIAGONAL, DIAGONAL), (DIAGONAL, -DIAGONAL)],
            KickKind::Clear,
        ),
    ] {
        for (x, y) in directions {
            let target = point!(
                ball_position.x() + x * distance,
                ball_position.y() + y * distance
            );
            targets.push((clamp_to_field(field_dimensions, target, FIELD_MARGIN), kind));
        }
    }

    targets
}

fn long_kick_distance(parameters: &KickingParameters) -> f32 {
    if parameters.allow_schlong {
        parameters.schlong_max_distance
    } else {
        parameters.rumpelstilzchen_max_distance
    }
}

fn clamp_to_field(
    field_dimensions: FieldDimensions,
    target: Point2<Field>,
    margin: f32,
) -> Point2<Field> {
    point!(
        target.x().clamp(
            -field_dimensions.length / 2.0 + margin,
            field_dimensions.length / 2.0 - margin
        ),
        target.y().clamp(
            -field_dimensions.width / 2.0 + margin,
            field_dimensions.width / 2.0 - margin
        )
    )
}

fn kick_candidate(
    ball_position: Point2<Field>,
    target: Point2<Field>,
    kind: KickKind,
    memory: KickMemory,
    parameters: &KickingParameters,
) -> Option<KickCandidate> {
    let power = power_for_distance(distance(ball_position, target), parameters)?;
    let forward_progress = target.x() - ball_position.x();
    let mut score = match kind {
        KickKind::Shoot => 100.0,
        KickKind::Clear => 30.0 + forward_progress,
        KickKind::Touch => 10.0 + forward_progress,
        KickKind::Pass => return None,
    };

    if power == KickPower::Rumpelstilzchen {
        score += 2.0;
    }
    if memory
        .target
        .is_some_and(|previous_target| distance(previous_target, target) <= 0.25)
    {
        score += parameters.target_switch_score_margin;
    }

    Some(KickCandidate {
        target,
        kind,
        power,
        score,
        receiver: None,
        ignored_obstacle_point: None,
    })
}

fn pass_candidates(
    blackboard: &Blackboard,
    ball_position: Point2<Field>,
    memory: KickMemory,
    parameters: &KickingParameters,
) -> Vec<KickCandidate> {
    let mut candidates = Vec::new();

    for (player_number, player_state) in blackboard.world_state.player_states.iter() {
        if player_number == blackboard.world_state.robot.player_number
            || player_number == blackboard.parameters.goal_keeper_number
        {
            continue;
        }
        let Some(player_state) = player_state else {
            continue;
        };
        let age = blackboard
            .world_state
            .now
            .duration_since(player_state.last_seen)
            .unwrap_or(Duration::ZERO);
        if age > blackboard.pass_intent_timeout {
            continue;
        }

        let receiver_position = player_state.pose.position();
        let target = clamp_to_field(blackboard.field_dimensions, receiver_position, FIELD_MARGIN);
        let ignored_obstacle_point = Some(receiver_position);
        let Some(power) = power_for_distance(distance(ball_position, target), parameters) else {
            continue;
        };
        let forward_progress = target.x() - ball_position.x();
        if forward_progress <= 0.0 {
            continue;
        }
        if !receiver_reaches_before_obstacles(
            blackboard,
            receiver_position,
            target,
            ignored_obstacle_point,
        ) {
            continue;
        }

        let mut score = 60.0 + forward_progress;
        if power == KickPower::Rumpelstilzchen {
            score += 2.0;
        }
        if memory
            .target
            .is_some_and(|previous_target| distance(previous_target, target) <= 0.25)
        {
            score += parameters.target_switch_score_margin;
        }

        candidates.push(KickCandidate {
            target,
            kind: KickKind::Pass,
            power,
            score,
            receiver: Some(player_number),
            ignored_obstacle_point,
        });
    }

    candidates
}

fn receiver_reaches_before_obstacles(
    blackboard: &Blackboard,
    receiver_position: Point2<Field>,
    receive_point: Point2<Field>,
    ignored_obstacle_point: Option<Point2<Field>>,
) -> bool {
    let speed = blackboard.parameters.walk_speed.support.max(0.01);
    let receiver_eta = distance(receiver_position, receive_point) / speed;
    let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field else {
        return true;
    };
    let ignored_obstacle_index = ignored_obstacle_index(blackboard, ignored_obstacle_point);

    let nearest_obstacle_eta = blackboard
        .world_state
        .obstacles
        .iter()
        .enumerate()
        .filter(|(index, _)| Some(*index) != ignored_obstacle_index)
        .map(|(_, obstacle)| obstacle)
        .filter(|obstacle| {
            matches!(
                obstacle.kind,
                ObstacleKind::Robot | ObstacleKind::Person | ObstacleKind::Unknown
            )
        })
        .filter_map(|obstacle| {
            let center = ground_to_field * obstacle.position;
            Some(distance(center, receive_point) / speed)
        })
        .fold(None, |nearest: Option<f32>, obstacle_eta| {
            Some(nearest.map_or(obstacle_eta, |nearest| nearest.min(obstacle_eta)))
        });

    nearest_obstacle_eta
        .map(|obstacle_eta| receiver_eta + PASS_RECEIVER_REACH_MARGIN < obstacle_eta)
        .unwrap_or(true)
}

fn is_lane_blocked(
    blackboard: &Blackboard,
    ball_position: Point2<Field>,
    target: Point2<Field>,
    ignored_obstacle_point: Option<Point2<Field>>,
) -> bool {
    let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field else {
        return false;
    };
    let ignored_obstacle_index = ignored_obstacle_index(blackboard, ignored_obstacle_point);

    let line_segment = LineSegment::new(ball_position, target);
    blackboard
        .world_state
        .obstacles
        .iter()
        .enumerate()
        .filter(|(index, _)| Some(*index) != ignored_obstacle_index)
        .map(|(_, obstacle)| obstacle)
        .filter(|obstacle| obstacle.kind != ObstacleKind::Ball)
        .any(|obstacle| {
            let center = ground_to_field * obstacle.position;
            let radius = obstacle
                .radius_at_foot_height
                .max(obstacle.radius_at_hip_height)
                + blackboard.parameters.kicking.kick_corridor_radius;
            Circle::new(center, radius).intersects_line_segment(&line_segment)
        })
}

fn ignored_obstacle_index(
    blackboard: &Blackboard,
    ignored_obstacle_point: Option<Point2<Field>>,
) -> Option<usize> {
    let ignored_obstacle_point = ignored_obstacle_point?;
    let ground_to_field = blackboard.world_state.robot.ground_to_field?;

    blackboard
        .world_state
        .obstacles
        .iter()
        .enumerate()
        .filter(|(_, obstacle)| obstacle.kind == ObstacleKind::Robot)
        .filter_map(|(index, obstacle)| {
            let center = ground_to_field * obstacle.position;
            let distance_to_ignored_point = distance(center, ignored_obstacle_point);
            (distance_to_ignored_point < PASS_RECEIVER_OBSTACLE_ECHO_RADIUS)
                .then_some((index, distance_to_ignored_point))
        })
        .min_by(|(_, lhs_distance), (_, rhs_distance)| lhs_distance.total_cmp(rhs_distance))
        .map(|(index, _)| index)
}

pub fn fallback_goal_intent(
    field_dimensions: FieldDimensions,
    ball: &LastBall,
    parameters: &KickingParameters,
) -> Option<KickIntent> {
    let target = point!(field_dimensions.length / 2.0, 0.0);
    Some(KickIntent {
        kind: KickKind::Shoot,
        ball_position: ball.position,
        target,
        power: power_for_distance(distance(ball.position, target), parameters)?,
        receiver: None,
    })
}

pub fn fallback_touch_intent(
    field_dimensions: FieldDimensions,
    ball: &LastBall,
    memory: KickMemory,
    parameters: &KickingParameters,
) -> Option<KickIntent> {
    let side_sign = memory
        .target
        .filter(|target| {
            (-field_dimensions.length / 2.0..=field_dimensions.length / 2.0).contains(&target.x())
                && (-field_dimensions.width / 2.0..=field_dimensions.width / 2.0)
                    .contains(&target.y())
        })
        .and_then(|target| {
            let y_direction = target.y() - ball.position.y();
            (y_direction.abs() > f32::EPSILON).then_some(y_direction.signum())
        })
        .unwrap_or(if ball.position.y() <= 0.0 { 1.0 } else { -1.0 });
    let target_x = ball.position.x().clamp(
        -field_dimensions.length / 2.0 + 0.3,
        field_dimensions.length / 2.0 - 0.3,
    );
    let target_y = (ball.position.y() + side_sign * 1.0).clamp(
        -field_dimensions.width / 2.0 + 0.3,
        field_dimensions.width / 2.0 - 0.3,
    );

    let target = point!(target_x, target_y);

    Some(KickIntent {
        kind: KickKind::Touch,
        ball_position: ball.position,
        target,
        power: power_for_distance(distance(ball.position, target), parameters)?,
        receiver: None,
    })
}

pub fn power_for_distance(
    distance: f32,
    parameters: &types::parameters::KickingParameters,
) -> Option<KickPower> {
    if (parameters.rumpelstilzchen_min_distance..=parameters.rumpelstilzchen_max_distance)
        .contains(&distance)
    {
        Some(KickPower::Rumpelstilzchen)
    } else if parameters.allow_schlong
        && (parameters.schlong_min_distance..=parameters.schlong_max_distance).contains(&distance)
    {
        Some(KickPower::Schlong)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, SystemTime};

    use hsl_network_messages::{PlayerNumber, SubState, Team};
    use linear_algebra::{Isometry2, Pose2, distance, point, vector};
    use types::{
        field_dimensions::{FieldDimensions, Side},
        filtered_game_controller_state::FilteredGameControllerState,
        filtered_game_state::FilteredGameState,
        motion_command::MotionCommand,
        obstacles::{Obstacle, ObstacleKind},
        parameters::{BehaviorParameters, KickingParameters},
        players::Players,
        world_state::{PlayerState, WorldState},
    };

    use super::*;

    fn ball_at(x: f32, y: f32) -> LastBall {
        LastBall {
            position: point!(x, y),
            velocity: vector!(0.0, 0.0),
            age: SystemTime::UNIX_EPOCH,
            field_side: Side::Left,
        }
    }

    fn filtered_state(
        sub_state: Option<SubState>,
        kicking_team: Option<Team>,
        ball_is_free: bool,
        kick_off: bool,
    ) -> FilteredGameControllerState {
        FilteredGameControllerState {
            game_state: FilteredGameState::Playing {
                ball_is_free,
                kick_off,
            },
            sub_state,
            kicking_team,
            ..Default::default()
        }
    }

    fn blackboard(
        filtered_game_controller_state: Option<FilteredGameControllerState>,
        ball: LastBall,
    ) -> Blackboard {
        let mut world_state = WorldState::default();
        world_state.robot.player_number = PlayerNumber::Two;
        world_state.filtered_game_controller_state = filtered_game_controller_state;
        world_state.robot.ground_to_field = Some(Isometry2::identity());
        let mut parameters = BehaviorParameters::default();
        parameters.kicking = kicking_parameters(true);

        Blackboard {
            field_dimensions: FieldDimensions::SPL_2025,
            free_kick_obstacle_radius: 0.0,
            pass_intent_timeout: Duration::from_secs(1),
            parameters,
            world_state,

            path_obstacles_output: Vec::new(),
            time_since_last_switch: Duration::ZERO,
            direction_difference: 0.0,
            voronoi_inputs: Vec::new(),

            ball: Some(ball),
            last_ball: None,
            last_close_enough_to_kick: false,
            last_kick_target: None,
            cycle_intent: CycleIntent::default(),
            kick_memory: KickMemory::default(),
            last_motion_command: MotionCommand::default(),
            last_motion_switch_time: SystemTime::UNIX_EPOCH,
            last_motion_type: None,

            is_injected_motion_command: false,
            walk_position: None,
            body_motion: None,
            head_motion: None,
            voronoi_map: None,
        }
    }

    fn select_intent(
        filtered_game_controller_state: Option<FilteredGameControllerState>,
        memory: KickMemory,
    ) -> CycleIntent {
        select_intent_with_ball(filtered_game_controller_state, ball_at(0.0, 0.0), memory)
    }

    fn select_intent_with_ball(
        filtered_game_controller_state: Option<FilteredGameControllerState>,
        ball: LastBall,
        memory: KickMemory,
    ) -> CycleIntent {
        let blackboard = blackboard(filtered_game_controller_state, ball);
        select_cycle_intent(&blackboard, memory, Some(PlayerNumber::Two))
    }

    fn select_intent_with_parameters(
        filtered_game_controller_state: Option<FilteredGameControllerState>,
        ball: LastBall,
        memory: KickMemory,
        kicking_parameters: KickingParameters,
    ) -> CycleIntent {
        let mut blackboard = blackboard(filtered_game_controller_state, ball);
        blackboard.parameters.kicking = kicking_parameters;
        select_cycle_intent(&blackboard, memory, Some(PlayerNumber::Two))
    }

    fn kicking_parameters(allow_schlong: bool) -> KickingParameters {
        KickingParameters {
            allow_schlong,
            rumpelstilzchen_min_distance: 0.2,
            rumpelstilzchen_max_distance: 2.5,
            schlong_min_distance: 2.0,
            schlong_max_distance: 6.0,
            ..Default::default()
        }
    }

    fn player_state_at(x: f32, y: f32, last_seen: SystemTime) -> PlayerState {
        PlayerState {
            pose: Pose2::from(point!(x, y)),
            last_seen,
            ..Default::default()
        }
    }

    fn block_all_goal_shots(blackboard: &mut Blackboard) {
        let goal_x = blackboard.field_dimensions.length / 2.0;
        let goal_side_y = blackboard.field_dimensions.goal_inner_width / 3.0;
        blackboard.world_state.obstacles.extend([
            Obstacle::robot(point!(goal_x / 2.0, 0.0), 0.2, 0.2),
            Obstacle::robot(point!(goal_x / 2.0, goal_side_y / 2.0), 0.2, 0.2),
            Obstacle::robot(point!(goal_x / 2.0, -goal_side_y / 2.0), 0.2, 0.2),
        ]);
    }

    #[test]
    fn short_range_selects_rumpelstilzchen() {
        let parameters = kicking_parameters(true);

        assert_eq!(
            power_for_distance(1.0, &parameters),
            Some(KickPower::Rumpelstilzchen)
        );
    }

    #[test]
    fn overlapping_range_prefers_rumpelstilzchen() {
        let parameters = kicking_parameters(true);

        assert_eq!(
            power_for_distance(2.25, &parameters),
            Some(KickPower::Rumpelstilzchen)
        );
    }

    #[test]
    fn long_range_selects_schlong() {
        let parameters = kicking_parameters(true);

        assert_eq!(
            power_for_distance(4.0, &parameters),
            Some(KickPower::Schlong)
        );
    }

    #[test]
    fn long_range_returns_none_when_schlong_is_disallowed() {
        let parameters = kicking_parameters(false);

        assert_eq!(power_for_distance(4.0, &parameters), None);
    }

    #[test]
    fn out_of_range_returns_none() {
        let parameters = kicking_parameters(true);

        assert_eq!(power_for_distance(6.5, &parameters), None);
    }

    #[test]
    fn fallback_goal_intent_targets_opponent_goal_center_for_9m_field() {
        let ball = ball_at(0.0, 0.0);
        let parameters = kicking_parameters(true);

        let intent = fallback_goal_intent(FieldDimensions::SPL_2025, &ball, &parameters)
            .expect("goal fallback should have a matching power envelope");

        assert_eq!(intent.target, point!(4.5, 0.0));
        assert_eq!(intent.power, KickPower::Schlong);
    }

    #[test]
    fn outgoing_coordination_intent_serializes_selected_pass() {
        let cycle_intent = CycleIntent {
            role: TacticalRole::Striker,
            kick: Some(KickIntent {
                kind: KickKind::Pass,
                ball_position: point!(0.0, 0.0),
                target: point!(1.0, 0.5),
                power: KickPower::Rumpelstilzchen,
                receiver: Some(PlayerNumber::Four),
            }),
            ..Default::default()
        };

        let Some(CoordinationIntent::Pass(pass)) =
            outgoing_coordination_intent(SystemTime::UNIX_EPOCH, cycle_intent)
        else {
            panic!("selected pass should produce a coordination intent");
        };

        assert_eq!(pass.receiver, PlayerNumber::Four);
        assert_eq!(pass.receive_point, point!(1.0, 0.5));
    }

    #[test]
    fn no_filtered_game_state_suppresses_direct_scoring_but_allows_touch() {
        let intent = select_intent(None, KickMemory::default());

        assert!(intent.rule_constraints.may_kick_ball);
        assert!(!intent.rule_constraints.may_score_directly);
        let kick = intent.kick.expect("non-shot intent should be allowed");
        assert!(matches!(kick.kind, KickKind::Touch | KickKind::Clear));
    }

    #[test]
    fn own_restart_suppresses_shoot_candidates_and_selects_touch_or_clear() {
        let intent = select_intent(
            Some(filtered_state(
                Some(SubState::CornerKick),
                Some(Team::Hulks),
                true,
                false,
            )),
            KickMemory::default(),
        );

        assert!(intent.rule_constraints.may_kick_ball);
        assert!(!intent.rule_constraints.may_score_directly);
        let kick = intent
            .kick
            .expect("own restart should allow a non-shot kick");
        assert!(matches!(kick.kind, KickKind::Touch | KickKind::Clear));
    }

    #[test]
    fn opponent_substate_creates_no_kick_intent() {
        let intent = select_intent(
            Some(filtered_state(
                Some(SubState::CornerKick),
                Some(Team::Opponent),
                false,
                false,
            )),
            KickMemory::default(),
        );

        assert!(!intent.rule_constraints.may_kick_ball);
        assert!(!intent.rule_constraints.may_score_directly);
        assert!(intent.kick.is_none());
    }

    #[test]
    fn open_goal_selects_shoot() {
        let intent = select_intent(
            Some(filtered_state(None, None, true, false)),
            KickMemory::default(),
        );

        assert!(intent.rule_constraints.may_kick_ball);
        assert!(intent.rule_constraints.may_score_directly);
        let kick = intent.kick.expect("open play should allow shooting");
        assert_eq!(kick.kind, KickKind::Shoot);
        assert_eq!(kick.target, point!(4.5, 0.0));
        assert_eq!(kick.power, KickPower::Schlong);
    }

    #[test]
    fn blocked_center_goal_selects_non_center_target() {
        let ball = ball_at(0.0, 0.0);
        let mut blackboard = blackboard(Some(filtered_state(None, None, true, false)), ball);
        blackboard.world_state.robot.ground_to_field = Some(Isometry2::identity());
        blackboard.world_state.obstacles = vec![Obstacle {
            kind: ObstacleKind::Robot,
            position: point!(2.25, 0.0),
            radius_at_foot_height: 0.2,
            radius_at_hip_height: 0.2,
        }];

        let intent =
            select_cycle_intent(&blackboard, KickMemory::default(), Some(PlayerNumber::Two));

        let kick = intent
            .kick
            .expect("blocked center should still leave side shots");
        assert_eq!(kick.kind, KickKind::Shoot);
        assert_ne!(kick.target, point!(4.5, 0.0));
    }

    #[test]
    fn fresh_open_receiver_wins_when_direct_shot_is_blocked() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let ball = ball_at(0.0, 0.0);
        let mut blackboard = blackboard(Some(filtered_state(None, None, true, false)), ball);
        blackboard.world_state.now = now;
        blackboard.world_state.player_states = Players {
            four: Some(player_state_at(1.5, 1.5, now)),
            ..Players::new(None)
        };
        block_all_goal_shots(&mut blackboard);

        let intent =
            select_cycle_intent(&blackboard, KickMemory::default(), Some(PlayerNumber::Two));

        let kick = intent
            .kick
            .expect("blocked shots should leave an open pass");
        assert_eq!(kick.kind, KickKind::Pass);
        assert_eq!(kick.receiver, Some(PlayerNumber::Four));
        assert_eq!(kick.target, point!(1.5, 1.5));
    }

    #[test]
    fn receiver_obstacle_echo_does_not_block_pass() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let ball = ball_at(0.0, 0.0);
        let mut blackboard = blackboard(Some(filtered_state(None, None, true, false)), ball);
        blackboard.world_state.now = now;
        blackboard.world_state.player_states = Players {
            four: Some(player_state_at(1.5, 1.5, now)),
            ..Players::new(None)
        };
        block_all_goal_shots(&mut blackboard);
        blackboard
            .world_state
            .obstacles
            .push(Obstacle::robot(point!(1.5, 1.5), 0.2, 0.2));

        let intent =
            select_cycle_intent(&blackboard, KickMemory::default(), Some(PlayerNumber::Two));

        let kick = intent
            .kick
            .expect("blocked shots should leave an open pass");
        assert_eq!(kick.kind, KickKind::Pass);
        assert_eq!(kick.receiver, Some(PlayerNumber::Four));
    }

    #[test]
    fn lone_nearby_obstacle_is_not_treated_as_receiver_echo() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let ball = ball_at(0.0, 0.0);
        let mut blackboard = blackboard(Some(filtered_state(None, None, true, false)), ball);
        blackboard.world_state.now = now;
        blackboard.world_state.player_states = Players {
            four: Some(player_state_at(1.5, 1.5, now)),
            ..Players::new(None)
        };
        block_all_goal_shots(&mut blackboard);
        blackboard.world_state.obstacles.push(Obstacle {
            kind: ObstacleKind::Unknown,
            position: point!(1.5, 1.2),
            radius_at_foot_height: 0.2,
            radius_at_hip_height: 0.2,
        });

        let intent =
            select_cycle_intent(&blackboard, KickMemory::default(), Some(PlayerNumber::Two));

        let kick = intent
            .kick
            .expect("blocked pass should still leave clear or touch candidates");
        assert_ne!(kick.kind, KickKind::Pass);
    }

    #[test]
    fn lone_inside_radius_unknown_is_not_treated_as_receiver_echo() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let ball = ball_at(0.0, 0.0);
        let mut blackboard = blackboard(Some(filtered_state(None, None, true, false)), ball);
        blackboard.world_state.now = now;
        blackboard.world_state.player_states = Players {
            four: Some(player_state_at(1.5, 1.5, now)),
            ..Players::new(None)
        };
        block_all_goal_shots(&mut blackboard);
        blackboard.world_state.obstacles.push(Obstacle {
            kind: ObstacleKind::Unknown,
            position: point!(1.5, 1.42),
            radius_at_foot_height: 0.2,
            radius_at_hip_height: 0.2,
        });

        let intent =
            select_cycle_intent(&blackboard, KickMemory::default(), Some(PlayerNumber::Two));

        let kick = intent
            .kick
            .expect("blocked pass should still leave clear or touch candidates");
        assert_ne!(kick.kind, KickKind::Pass);
    }

    #[test]
    fn nearby_obstacle_behind_receiver_echo_blocks_pass() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let ball = ball_at(0.0, 0.0);
        let mut blackboard = blackboard(Some(filtered_state(None, None, true, false)), ball);
        blackboard.world_state.now = now;
        blackboard.world_state.player_states = Players {
            four: Some(player_state_at(1.5, 1.5, now)),
            ..Players::new(None)
        };
        block_all_goal_shots(&mut blackboard);
        blackboard.world_state.obstacles.extend([
            Obstacle::robot(point!(1.5, 1.5), 0.2, 0.2),
            Obstacle {
                kind: ObstacleKind::Unknown,
                position: point!(1.5, 1.2),
                radius_at_foot_height: 0.2,
                radius_at_hip_height: 0.2,
            },
        ]);

        let intent =
            select_cycle_intent(&blackboard, KickMemory::default(), Some(PlayerNumber::Two));

        let kick = intent
            .kick
            .expect("blocked pass should still leave clear or touch candidates");
        assert_ne!(kick.kind, KickKind::Pass);
    }

    #[test]
    fn stale_receiver_is_ignored() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let ball = ball_at(0.0, 0.0);
        let mut blackboard = blackboard(Some(filtered_state(None, None, true, false)), ball);
        blackboard.world_state.now = now;
        blackboard.world_state.player_states = Players {
            four: Some(player_state_at(1.5, 1.5, now - Duration::from_secs(2))),
            ..Players::new(None)
        };
        block_all_goal_shots(&mut blackboard);

        let intent =
            select_cycle_intent(&blackboard, KickMemory::default(), Some(PlayerNumber::Two));

        let kick = intent
            .kick
            .expect("blocked shots should still leave clear or touch candidates");
        assert_ne!(kick.kind, KickKind::Pass);
    }

    #[test]
    fn blocked_receiver_is_ignored() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let ball = ball_at(0.0, 0.0);
        let mut blackboard = blackboard(Some(filtered_state(None, None, true, false)), ball);
        blackboard.world_state.now = now;
        blackboard.world_state.player_states = Players {
            four: Some(player_state_at(1.5, 1.5, now)),
            ..Players::new(None)
        };
        block_all_goal_shots(&mut blackboard);
        blackboard
            .world_state
            .obstacles
            .push(Obstacle::robot(point!(0.75, 0.75), 0.2, 0.2));

        let intent =
            select_cycle_intent(&blackboard, KickMemory::default(), Some(PlayerNumber::Two));

        let kick = intent
            .kick
            .expect("blocked pass should still leave clear or touch candidates");
        assert_ne!(kick.kind, KickKind::Pass);
    }

    #[test]
    fn pass_is_not_selected_when_open_shot_exists() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let ball = ball_at(0.0, 0.0);
        let mut blackboard = blackboard(Some(filtered_state(None, None, true, false)), ball);
        blackboard.world_state.now = now;
        blackboard.world_state.player_states = Players {
            four: Some(player_state_at(1.5, 1.5, now)),
            ..Players::new(None)
        };

        let intent =
            select_cycle_intent(&blackboard, KickMemory::default(), Some(PlayerNumber::Two));

        let kick = intent.kick.expect("open play should allow shooting");
        assert_eq!(kick.kind, KickKind::Shoot);
        assert_eq!(kick.receiver, None);
    }

    #[test]
    fn selected_pass_produces_outgoing_coordination_intent() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let ball = ball_at(0.0, 0.0);
        let mut blackboard = blackboard(Some(filtered_state(None, None, true, false)), ball);
        blackboard.world_state.now = now;
        blackboard.world_state.player_states = Players {
            four: Some(player_state_at(1.5, 1.5, now)),
            ..Players::new(None)
        };
        block_all_goal_shots(&mut blackboard);

        let intent =
            select_cycle_intent(&blackboard, KickMemory::default(), Some(PlayerNumber::Two));

        let Some(CoordinationIntent::Pass(pass)) = outgoing_coordination_intent(now, intent) else {
            panic!("selected pass should produce a coordination intent");
        };
        assert_eq!(pass.receiver, PlayerNumber::Four);
        assert_eq!(pass.receive_point, point!(1.5, 1.5));
    }

    #[test]
    fn supporter_does_not_populate_or_emit_pass_intent() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let ball = ball_at(0.0, 0.0);
        let mut blackboard = blackboard(Some(filtered_state(None, None, true, false)), ball);
        blackboard.world_state.now = now;
        blackboard.world_state.player_states = Players {
            four: Some(player_state_at(1.5, 1.5, now)),
            ..Players::new(None)
        };
        block_all_goal_shots(&mut blackboard);

        let intent = select_cycle_intent(
            &blackboard,
            KickMemory::default(),
            Some(PlayerNumber::Three),
        );

        assert_eq!(intent.role, TacticalRole::Supporter);
        assert_ne!(intent.kick.map(|kick| kick.kind), Some(KickKind::Pass));
        assert!(outgoing_coordination_intent(now, intent).is_none());
    }

    #[test]
    fn penalty_restart_does_not_select_pass() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let ball = ball_at(0.0, 0.0);
        let mut blackboard = blackboard(
            Some(filtered_state(
                Some(SubState::PenaltyKick),
                Some(Team::Hulks),
                true,
                false,
            )),
            ball,
        );
        blackboard.world_state.now = now;
        blackboard.world_state.player_states = Players {
            four: Some(player_state_at(1.5, 1.5, now)),
            ..Players::new(None)
        };
        block_all_goal_shots(&mut blackboard);

        let intent =
            select_cycle_intent(&blackboard, KickMemory::default(), Some(PlayerNumber::Two));

        assert!(intent.rule_constraints.may_kick_ball);
        assert!(!intent.rule_constraints.may_pass);
        assert_ne!(intent.kick.map(|kick| kick.kind), Some(KickKind::Pass));
    }

    #[test]
    fn backward_receiver_is_ignored() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let ball = ball_at(0.0, 0.0);
        let mut blackboard = blackboard(Some(filtered_state(None, None, true, false)), ball);
        blackboard.world_state.now = now;
        blackboard.world_state.player_states = Players {
            four: Some(player_state_at(-1.0, 1.5, now)),
            ..Players::new(None)
        };
        block_all_goal_shots(&mut blackboard);

        let intent =
            select_cycle_intent(&blackboard, KickMemory::default(), Some(PlayerNumber::Two));

        let kick = intent
            .kick
            .expect("blocked shots should still leave clear or touch candidates");
        assert_ne!(kick.kind, KickKind::Pass);
    }

    #[test]
    fn candidates_outside_power_envelopes_are_rejected() {
        let intent = select_intent_with_parameters(
            Some(filtered_state(None, None, true, false)),
            ball_at(0.0, 0.0),
            KickMemory::default(),
            KickingParameters {
                allow_schlong: true,
                rumpelstilzchen_min_distance: 10.0,
                rumpelstilzchen_max_distance: 11.0,
                schlong_min_distance: 12.0,
                schlong_max_distance: 13.0,
                ..Default::default()
            },
        );

        assert!(intent.kick.is_none());
    }

    #[test]
    fn previous_target_gets_hysteresis_bonus_when_scores_are_close() {
        let clear_distance = 1.1;
        let previous_target = point!(
            std::f32::consts::FRAC_1_SQRT_2 * clear_distance,
            std::f32::consts::FRAC_1_SQRT_2 * clear_distance
        );

        let intent = select_intent_with_parameters(
            Some(filtered_state(
                Some(SubState::CornerKick),
                Some(Team::Hulks),
                true,
                false,
            )),
            ball_at(0.0, 0.0),
            KickMemory {
                target: Some(previous_target),
                ..Default::default()
            },
            KickingParameters {
                rumpelstilzchen_min_distance: 0.5,
                rumpelstilzchen_max_distance: clear_distance,
                target_switch_score_margin: 1.0,
                ..Default::default()
            },
        );

        let kick = intent
            .kick
            .expect("hysteresis should preserve a close previous clear target");
        assert_eq!(kick.kind, KickKind::Clear);
        assert!(distance(kick.target, previous_target) < 0.001);
    }

    #[test]
    fn no_substate_with_non_free_ball_creates_no_kick_intent() {
        let intent = select_intent(
            Some(filtered_state(None, None, false, true)),
            KickMemory::default(),
        );

        assert!(!intent.rule_constraints.may_kick_ball);
        assert!(!intent.rule_constraints.may_score_directly);
        assert!(intent.kick.is_none());
    }

    #[test]
    fn fallback_touch_keeps_previous_touch_side() {
        let parameters = kicking_parameters(true);
        let kick = fallback_touch_intent(
            FieldDimensions::SPL_2025,
            &ball_at(0.0, 0.01),
            KickMemory {
                target: Some(point!(0.0, 1.0)),
                ..Default::default()
            },
            &parameters,
        )
        .expect("touch fallback should fit the configured short envelope");

        assert_eq!(kick.kind, KickKind::Touch);
        assert_eq!(kick.target, point!(0.0, 1.01));
        assert_eq!(kick.power, KickPower::Rumpelstilzchen);
    }

    #[test]
    fn fallback_touch_target_is_clamped_near_field_edges() {
        let parameters = kicking_parameters(true);
        let kick = fallback_touch_intent(
            FieldDimensions::SPL_2025,
            &ball_at(4.45, 2.65),
            KickMemory {
                target: Some(point!(4.2, 2.7)),
                ..Default::default()
            },
            &parameters,
        )
        .expect("clamped touch fallback should fit the configured short envelope");

        assert_eq!(kick.kind, KickKind::Touch);
        assert_eq!(kick.target, point!(4.2, 2.7));
        assert_eq!(kick.power, KickPower::Rumpelstilzchen);
    }
}
