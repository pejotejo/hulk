use coordinate_systems::{Field, Ground};
use filtering::hysteresis::less_than_with_hysteresis;
use geometry::line_segment::LineSegment;
use linear_algebra::{Isometry2, Orientation2, Point2, Pose2, Vector2, point};
use types::{
    behavior_tree::Status,
    motion_command::{BodyMotion, HeadMotion, ImageRegion, MotionCommand, OrientationMode},
    motion_type::MotionType,
    obstacles::ObstacleKind,
    parameters::KickingParameters,
    path::traits::{EndPoints, Project},
};

use crate::{
    actions::stand,
    node::Blackboard,
    switch_motion_type::is_allowed_to_switch,
    voronoi::fresh_teammate_poses,
    walk::{has_rule_path_clearance, try_plan, walk_to_with_path},
};

use super::{
    ball::{kick_ball_state, projected_ball_position},
    passing::pass_is_ready,
    restarts::{arm_restart_release, should_yield_after_restart},
    strategy::{BallAction, resolve_action_target},
};

/// Resolve the selected action against the current ball and obstacle corridors.
pub fn execute_ball_action(blackboard: &mut Blackboard) -> Status {
    let Some(action) = blackboard.ball_action else {
        return stop_ball_action(blackboard);
    };
    if action == BallAction::Hold {
        return hold_ball_action(blackboard);
    }
    let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field else {
        return stop_ball_action(blackboard);
    };
    let Some((ball, velocity)) = kick_ball_state(blackboard, &ground_to_field) else {
        return stop_ball_action(blackboard);
    };
    let Some(target) = resolve_action_target(blackboard, action, ground_to_field * ball) else {
        // A previous opening may now be occupied. Let selection choose again;
        // a blocked corridor must not turn into a blind goal-center shot.
        blackboard.ball_action = None;
        return stop_ball_action(blackboard);
    };
    let target_in_ground = ground_to_field.inverse() * target;
    let Some(direction) = (target_in_ground - ball).try_normalize(f32::EPSILON) else {
        return stop_ball_action(blackboard);
    };
    blackboard.kick_target = Some(target_in_ground);
    blackboard.head_motion = Some(HeadMotion::LookAt {
        target: ball,
        height_above_ground: blackboard.field_dimensions.ball_radius,
        image_region_target: ImageRegion::Center,
    });

    let pass_ready = match action {
        BallAction::Pass { receiver } => pass_is_ready(blackboard, receiver, target),
        _ => true,
    };
    // A pass timeout can replace the action and start a cooldown. Do not send
    // the old pass after its announcement has been cancelled.
    if blackboard.ball_action != Some(action) {
        return stop_ball_action(blackboard);
    }

    let controlled = matches!(
        action,
        BallAction::Pass { .. } | BallAction::Dribble | BallAction::Cutback
    );
    let was_kicking = blackboard.last_motion_type == Some(MotionType::Kick);
    let can_switch = is_allowed_to_switch(blackboard);
    let ready = handoff_ready(
        ball,
        velocity,
        direction,
        blackboard.last_close_enough_to_kick && was_kicking,
        &blackboard.parameters.kicking,
        blackboard
            .parameters
            .ball
            .pursuit_lookahead
            .as_secs_f32()
            .min(0.6),
        !controlled,
    ) && has_body_path_clearance(
        blackboard,
        LineSegment(
            Point2::origin(),
            ball - direction * approach_standoff(blackboard),
        ),
        ground_to_field,
        BodyClearancePurpose::BallPolicyHandoff,
    );
    blackboard.last_close_enough_to_kick = ready;
    if ready && (was_kicking || can_switch) {
        if !pass_ready {
            return stand(blackboard);
        }
        let kicking = &blackboard.parameters.kicking;
        let mut target_speed = match action {
            BallAction::Pass { .. } => kicking.strategy.passing.target_speed,
            BallAction::Dribble => kicking.strategy.dribble_target_speed,
            BallAction::Cutback => kicking.strategy.cutback_target_speed,
            _ => kicking.target_speed,
        };
        let mut soft = controlled || kicking.soft;
        let mut quick = !controlled && kicking.quick;
        let mut strong = !controlled
            && kicking.allow_strong_kicks
            && (target_in_ground - ball).norm() >= kicking.strong_kick_min_target_distance;
        if was_kicking
            && !can_switch
            && let MotionCommand::Kick {
                soft: previous_soft,
                quick: previous_quick,
                strong: previous_strong,
                target_speed: previous_speed,
                ..
            } = blackboard.last_motion_command
        {
            // Soft and regular kicks run different policies. Keep the active
            // policy and its speed until the motion switch interval expires.
            if soft != previous_soft {
                return stand(blackboard);
            }
            soft = previous_soft;
            quick = previous_quick;
            strong = previous_strong && kicking.allow_strong_kicks;
            target_speed = previous_speed;
        }
        blackboard.body_motion = Some(BodyMotion::Kick {
            ball_position: ball,
            ball_velocity: velocity,
            kick_direction: Orientation2::from_vector(direction),
            target_speed,
            soft,
            quick,
            strong,
        });
        arm_restart_release(blackboard, ground_to_field * ball);
        return Status::Success;
    }

    if blackboard.last_motion_type != Some(MotionType::Walk) && !can_switch {
        return stand(blackboard);
    }
    walk_to_ball_action(blackboard, target)
}

fn stop_ball_action(blackboard: &mut Blackboard) -> Status {
    blackboard.kick_target = None;
    blackboard.last_close_enough_to_kick = false;
    stand(blackboard)
}

fn minimum_ball_clearance(blackboard: &Blackboard) -> f32 {
    let path = &blackboard.parameters.walking.path_planning;
    path.robot_radius
        + path
            .ball_obstacle_radius
            .max(blackboard.field_dimensions.ball_radius)
}

fn approach_standoff(blackboard: &Blackboard) -> f32 {
    blackboard
        .parameters
        .kicking
        .approach_ball_standoff
        .max(minimum_ball_clearance(blackboard) + 0.05)
}

#[derive(Clone, Copy)]
enum BodyClearancePurpose {
    Walking,
    BallPolicyHandoff,
}

/// Body access is independent of the outgoing ball corridor. Walking checks
/// original footprints, since planning may reduce them when projecting an
/// endpoint. The close-ball policy may contest observed robots; fresh teammate
/// poses, static obstacles and rule regions still constrain its handoff.
fn has_body_path_clearance(
    blackboard: &Blackboard,
    path: LineSegment<Ground>,
    ground_to_field: Isometry2<Ground, Field>,
    purpose: BodyClearancePurpose,
) -> bool {
    let own_radius = blackboard.parameters.walking.path_planning.robot_radius;
    let clearance = |position, radius: f32| {
        (path.project(position) - position).norm_squared() > (radius + own_radius).powi(2)
    };
    blackboard
        .world_state
        .obstacles
        .iter()
        // The policy handles foot contact with the ball. Its body launch pose,
        // rather than the ball center, is the end of this clearance check.
        .filter(|obstacle| match obstacle.kind {
            ObstacleKind::Ball => false,
            ObstacleKind::Robot => matches!(purpose, BodyClearancePurpose::Walking),
            _ => true,
        })
        .all(|obstacle| {
            clearance(
                obstacle.position,
                obstacle
                    .radius_at_foot_height
                    .max(obstacle.radius_at_hip_height),
            )
        })
        && fresh_teammate_poses(blackboard)
            .all(|(_, pose)| clearance(ground_to_field.inverse() * pose.position(), own_radius))
        && has_rule_path_clearance(blackboard, ground_to_field * path, own_radius)
}

/// Cover the ball-to-own-goal lane while no safe outgoing action is available.
/// Hold never hands possession to the kicking policy or pursues through a contest.
fn hold_ball_action(blackboard: &mut Blackboard) -> Status {
    blackboard.kick_target = None;
    blackboard.last_close_enough_to_kick = false;
    // The playing tree normally handles this first. Keep this action safe when
    // invoked directly too: positioning must not let a restart taker retouch.
    if should_yield_after_restart(blackboard) {
        return stand(blackboard);
    }
    let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field else {
        return stand(blackboard);
    };
    // Use the same current ball as the planner's obstacle. A cached position
    // without that obstacle is insufficient for moving around a contested ball.
    let Some(ball) = blackboard
        .world_state
        .ball
        .map(|ball| ball.ball_in_ground)
        .filter(|ball| ball.inner.coords.iter().all(|value| value.is_finite()))
    else {
        return stand(blackboard);
    };
    blackboard.head_motion = Some(HeadMotion::LookAt {
        target: ball,
        height_above_ground: blackboard.field_dimensions.ball_radius,
        image_region_target: ImageRegion::Center,
    });
    if blackboard.last_motion_type != Some(MotionType::Walk) && !is_allowed_to_switch(blackboard) {
        return stand(blackboard);
    }

    let path_parameters = &blackboard.parameters.walking.path_planning;
    let ball_clearance = minimum_ball_clearance(blackboard);
    let standoff = ball_clearance + 0.05;
    let walk = blackboard.parameters.walking.walk_and_stand;
    if ball.coords().norm() <= standoff + walk.target_reached_thresholds.x {
        return stand(blackboard);
    }
    let ball_in_field = ground_to_field * ball;
    let own_goal = point![-blackboard.field_dimensions.length / 2.0, 0.0];
    let Some(towards_own_goal) = (own_goal - ball_in_field).try_normalize(f32::EPSILON) else {
        return stand(blackboard);
    };
    let field_limit_x =
        (blackboard.field_dimensions.length / 2.0 - path_parameters.robot_radius).max(0.0);
    let field_limit_y =
        (blackboard.field_dimensions.width / 2.0 - path_parameters.robot_radius).max(0.0);
    let blocking = ball_in_field + towards_own_goal * standoff;
    let blocking = ground_to_field.inverse()
        * point![
            blocking.x().clamp(-field_limit_x, field_limit_x),
            blocking.y().clamp(-field_limit_y, field_limit_y)
        ];
    let Some(path) = try_plan(blackboard, blocking, ground_to_field) else {
        return stand(blackboard);
    };
    let Some(blocking) = path.segments.last().map(EndPoints::end_point) else {
        return stand(blackboard);
    };
    // The planner can project a destination out of an occupied region. Use its
    // actual endpoint, provided it still covers the goal side at a safe distance.
    let blocking_in_field = ground_to_field * blocking;
    if blocking_in_field.x().abs() > field_limit_x
        || blocking_in_field.y().abs() > field_limit_y
        || (blocking_in_field - ball_in_field).dot(&towards_own_goal) <= 0.0
        || (blocking - ball).norm() < ball_clearance
        || blocking.coords().norm() <= walk.target_reached_thresholds.x
        || !has_body_path_clearance(
            blackboard,
            LineSegment(blocking, blocking),
            ground_to_field,
            BodyClearancePurpose::Walking,
        )
    {
        return stand(blackboard);
    }
    walk_to_with_path(
        blackboard,
        Pose2::from_parts(blocking, Orientation2::from_vector(ball - blocking)),
        blackboard.parameters.walking.speed.blocking,
        OrientationMode::LookAt {
            target: ball,
            tolerance: walk.orientation_tolerance,
        },
        walk.normal_distance_to_be_aligned,
        walk.hysteresis,
        Some(path),
    )
}

/// Approach a predicted ball from behind the chosen outgoing target.
pub(crate) fn walk_to_ball_action(blackboard: &mut Blackboard, target: Point2<Field>) -> Status {
    let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field else {
        return Status::Failure;
    };
    let Some((ball, velocity)) = kick_ball_state(blackboard, &ground_to_field) else {
        return Status::Failure;
    };
    let projected = ground_to_field.inverse()
        * projected_ball_position(
            blackboard,
            ground_to_field * ball,
            ground_to_field * velocity,
        );
    let target = ground_to_field.inverse() * target;
    let Some(direction) = (target - projected).try_normalize(f32::EPSILON) else {
        return stand(blackboard);
    };
    // Keep the approach endpoint outside the obstacle used by the planner.
    // Otherwise projection out of that obstacle silently changes the standoff.
    let ball_clearance = minimum_ball_clearance(blackboard);
    let standoff = approach_standoff(blackboard);
    // Forward pursuit must not put the body pose inside the observed ball's
    // clearance circle. Stay behind both positions along the outgoing axis,
    // while preserving useful lateral or incoming-ball prediction.
    let forward_prediction = (projected - ball).dot(&direction).max(0.0);
    let approach = projected - direction * (standoff + forward_prediction);
    let orientation = Orientation2::from_vector(direction);
    let Some(path) = try_plan(blackboard, approach, ground_to_field) else {
        return stand(blackboard);
    };
    let Some(approach) = path.segments.last().map(EndPoints::end_point) else {
        return stand(blackboard);
    };
    if (approach - projected).norm() < ball_clearance
        || (approach - ball).norm() < ball_clearance
        || (projected - approach).dot(&direction) <= 0.0
        || !has_body_path_clearance(
            blackboard,
            LineSegment(approach, approach),
            ground_to_field,
            BodyClearancePurpose::Walking,
        )
    {
        return stand(blackboard);
    }
    let walk = blackboard.parameters.walking.walk_and_stand;
    walk_to_with_path(
        blackboard,
        Pose2::from_parts(approach, orientation),
        blackboard.parameters.walking.speed.kicking,
        // A clearance arc can point away from the ball for most of its length.
        // Keep facing the observed ball while moving around it; near the
        // endpoint, target_orientation supplies the outgoing kick alignment.
        OrientationMode::LookAt {
            target: ball,
            tolerance: walk.orientation_tolerance,
        },
        walk.normal_distance_to_be_aligned.max(standoff),
        walk.hysteresis,
        Some(path),
    )
}

fn handoff_ready(
    ball: Point2<Ground>,
    velocity: Vector2<Ground>,
    direction: Vector2<Ground>,
    was_ready: bool,
    parameters: &KickingParameters,
    interception_horizon: f32,
    allow_interception: bool,
) -> bool {
    // Kick selects a locomotion policy that owns final ball approach and
    // alignment. Its desired outgoing direction is not an instantaneous foot
    // strike, so nearby side/behind balls must not require a pre-aligned pose.
    if less_than_with_hysteresis(
        was_ready,
        ball.coords().norm(),
        parameters.kick_activation_distance,
        parameters.kick_activation_hysteresis,
    ) {
        return true;
    }

    // Outside the ordinary handoff range, only extend to an incoming ball
    // that will enter the forward kicking corridor within the pursuit horizon.
    if !allow_interception || velocity.norm_squared() <= f32::EPSILON {
        return false;
    }
    let approach = &parameters.approach;
    let heading = direction.y().atan2(direction.x()).abs();
    let heading_limit = approach.maximum_heading + if was_ready { 0.15 } else { 0.0 };
    let lateral_limit = approach.maximum_lateral_distance
        + if was_ready {
            parameters.kick_activation_hysteresis / 2.0
        } else {
            0.0
        };
    if heading > heading_limit || ball.x() <= 0.0 || ball.coords().dot(&direction) <= 0.0 {
        return false;
    }

    let incoming_speed = velocity.dot(&direction);
    if incoming_speed >= -f32::EPSILON {
        return false;
    }
    let forward_distance = parameters.minimum_interception_forward_distance.max(0.15);
    let time = (forward_distance - ball.coords().dot(&direction)) / incoming_speed;
    if time <= 0.0 || time > interception_horizon {
        return false;
    }
    let interception = ball + velocity * time;
    let interception_lateral = interception.x() * direction.y() - interception.y() * direction.x();
    interception.x() > 0.0
        && interception.coords().dot(&direction) > 0.0
        && interception.coords().norm()
            <= parameters.kick_activation_distance - parameters.kick_activation_hysteresis / 2.0
        && interception_lateral.abs() <= lateral_limit
}

#[cfg(test)]
mod tests {
    use linear_algebra::{Isometry2, point, vector};
    use types::{
        field_dimensions::FieldDimensions,
        obstacles::Obstacle,
        path::{PathSegment, traits::Project},
        primary_state::PrimaryState,
        world_state::{BallSource, BallState},
    };

    use super::*;

    fn parameters() -> KickingParameters {
        let mut parameters = KickingParameters::default();
        parameters.kick_activation_distance = 0.8;
        parameters.kick_activation_hysteresis = 0.2;
        parameters.minimum_interception_forward_distance = 0.15;
        parameters.approach.maximum_heading = 0.8;
        parameters.approach.maximum_lateral_distance = 0.4;
        parameters.approach.maximum_ball_escape_speed = 1.2;
        parameters
    }

    fn holding_blackboard(
        ball: Point2<Field>,
        robot_position: Point2<Field>,
        robot_heading: f32,
    ) -> Blackboard {
        let mut b = Blackboard::default();
        b.field_dimensions = FieldDimensions::SPL_2025;
        let transform = Isometry2::from_parts(robot_position.coords(), robot_heading);
        b.world_state.robot.ground_to_field = Some(transform);
        b.world_state.robot.primary_state = PrimaryState::Playing;
        b.world_state.robot.player_number = hsl_network_messages::PlayerNumber::Two;
        b.world_state.ball = Some(BallState {
            ball_in_field: ball,
            ball_in_ground: transform.inverse() * ball,
            source: BallSource::Own,
            ..Default::default()
        });
        b.ball_action = Some(BallAction::Hold);
        b.kick_target = Some(point![4.5, 0.0]);
        b.last_close_enough_to_kick = true;
        b.parameters.walking.path_planning.robot_radius = 0.28;
        b.parameters.walking.path_planning.ball_obstacle_radius = 0.05;
        b.parameters.walking.path_planning.rotation_penalty_factor = 0.4;
        let walk = &mut b.parameters.walking.walk_and_stand;
        walk.orientation_tolerance = 0.05;
        walk.normal_distance_to_be_aligned = 0.05;
        walk.target_reached_thresholds = nalgebra::Vector2::new(0.05, 0.1);
        b.parameters.walking.speed.blocking = 0.6;
        b.parameters.walking.speed.minimum_speed = 0.2;
        b.parameters.walking.speed.velocity_fade_distance = 3.0;
        b
    }

    fn small_and_middle_fields() -> [(&'static str, FieldDimensions); 2] {
        // Match location/hsl_{small,middle}/global.json5, including the different
        // physical balls. A smaller navigation-ball setting must not make the
        // normal approach endpoint fail its own physical clearance check.
        let small = FieldDimensions {
            length: 9.0,
            width: 6.0,
            ball_radius: 0.073,
            line_width: 0.05,
            penalty_marker_size: 0.1,
            goal_box_area_length: 1.0,
            goal_box_area_width: 3.0,
            penalty_area_length: 2.0,
            penalty_area_width: 4.0,
            penalty_marker_distance: 1.5,
            center_circle_diameter: 1.5,
            border_strip_width: 1.1,
            goal_inner_width: 1.8,
            goal_post_diameter: 0.1,
            goal_depth: 0.7,
            corner_arc_radius: 0.0,
        };
        let middle = FieldDimensions {
            length: 14.0,
            width: 9.0,
            ball_radius: 0.105,
            goal_box_area_width: 4.0,
            penalty_area_length: 3.0,
            penalty_area_width: 6.0,
            penalty_marker_distance: 2.0,
            center_circle_diameter: 3.0,
            goal_inner_width: 2.4,
            corner_arc_radius: 0.5,
            ..small
        };
        [("Small", small), ("Middle", middle)]
    }

    #[test]
    fn small_and_middle_approaches_clear_the_physical_ball_and_can_hand_over() {
        use std::time::Duration;

        for (name, field) in small_and_middle_fields() {
            let ball_in_field = point![1.5, 0.0];
            let mut b = holding_blackboard(ball_in_field, Point2::origin(), 0.0);
            b.field_dimensions = field;
            b.parameters.walking.path_planning.robot_radius = 0.4;
            b.parameters.walking.path_planning.ball_obstacle_radius = 0.05;
            b.parameters.kicking = parameters();
            b.parameters.kicking.kick_activation_distance = 1.1;
            b.parameters.kicking.approach_ball_standoff = 0.3;
            let goal = point![field.length / 2.0, 0.0];

            assert_eq!(walk_to_ball_action(&mut b, goal), Status::Success);
            let Some(BodyMotion::Walk { path, .. }) = &b.body_motion else {
                panic!("{name}: a clear route must approach the distant ball");
            };
            let destination = path.end_point();
            let observed_ball = b.world_state.ball.unwrap().ball_in_ground;
            assert!(destination.x() < observed_ball.x(), "{name}");
            assert!(destination.y().abs() < 1e-5, "{name}");
            let physical_margin = (destination - observed_ball).norm() - 0.4 - field.ball_radius;
            assert!((physical_margin - 0.05).abs() < 1e-5, "{name}");

            // Arriving at this valid body pose must leave the ball reachable by
            // the existing handoff gates, without changing their activation range.
            let destination_in_field = b.world_state.robot.ground_to_field.unwrap() * destination;
            let transform = Isometry2::from_parts(destination_in_field.coords(), 0.0);
            b.world_state.robot.ground_to_field = Some(transform);
            b.world_state.ball.as_mut().unwrap().ball_in_ground =
                transform.inverse() * ball_in_field;
            b.last_motion_type = Some(MotionType::Walk);
            b.world_state.now = b.last_motion_switch_time
                + b.parameters.control.allow_switch.walk
                + Duration::from_millis(1);
            b.ball_action = Some(BallAction::Shoot {
                opening: LineSegment(
                    point![goal.x(), -field.goal_inner_width / 3.0],
                    point![goal.x(), field.goal_inner_width / 3.0],
                ),
            });
            assert_eq!(execute_ball_action(&mut b), Status::Success);
            assert!(
                matches!(b.body_motion, Some(BodyMotion::Kick { .. })),
                "{name}: {:?}",
                b.body_motion
            );
        }
    }

    fn rolling_ball_approach_blackboard(
        field: FieldDimensions,
        velocity: Vector2<Ground>,
    ) -> Blackboard {
        use std::time::Duration;

        let mut b = holding_blackboard(point![2.0, 0.0], Point2::origin(), 0.0);
        b.field_dimensions = field;
        b.parameters.walking.path_planning.robot_radius = 0.4;
        b.parameters.walking.path_planning.ball_obstacle_radius = 0.05;
        b.parameters.kicking = parameters();
        b.parameters.kicking.kick_activation_distance = 1.1;
        b.parameters.kicking.approach_ball_standoff = 0.3;
        b.parameters.ball.pursuit_lookahead = Duration::from_millis(450);
        b.world_state.ball.as_mut().unwrap().ball_in_ground_velocity = velocity;
        b.ball_action = Some(BallAction::Shoot {
            opening: LineSegment(
                point![field.length / 2.0, -field.goal_inner_width / 3.0],
                point![field.length / 2.0, field.goal_inner_width / 3.0],
            ),
        });
        b
    }

    fn rolling_approach_destination(b: &mut Blackboard, name: &str) -> Point2<Ground> {
        assert_eq!(execute_ball_action(b), Status::Success);
        let Some(BodyMotion::Walk { path, .. }) = &b.body_motion else {
            panic!(
                "{name}: the distant rolling ball needs a walking approach: {:?}",
                b.body_motion
            );
        };
        let measured = b.world_state.ball.unwrap();
        let ground_to_field = b.world_state.robot.ground_to_field.unwrap();
        let projected = ground_to_field.inverse()
            * projected_ball_position(
                b,
                measured.ball_in_field,
                ground_to_field * measured.ball_in_ground_velocity,
            );
        let destination = path.end_point();
        let clearance = minimum_ball_clearance(b);
        assert!(
            (destination - measured.ball_in_ground).norm() >= clearance,
            "{name}: approach must clear the currently observed ball"
        );
        assert!(
            (destination - projected).norm() >= clearance,
            "{name}: approach must clear the predicted ball"
        );
        assert!(
            (path.project(measured.ball_in_ground) - measured.ball_in_ground).norm()
                >= clearance - 1e-4,
            "{name}: the walking route must clear the observed ball"
        );
        destination
    }

    #[test]
    fn forward_rolling_ball_is_approached_instead_of_waiting_for_it_to_stop() {
        for (name, field) in small_and_middle_fields() {
            let mut b = rolling_ball_approach_blackboard(field, vector![0.4, 0.0]);
            let destination = rolling_approach_destination(&mut b, name);
            // The former endpoint was only 0.343 m behind the measured ball on
            // Small, failing the 0.473 m physical-clearance check in open space.
            assert!(destination.x() > 1.0 && destination.x() < 1.5, "{name}");
            assert!(destination.y().abs() < 1e-5, "{name}");
        }
    }

    #[test]
    fn lateral_pursuit_still_leads_the_ball_without_crossing_its_current_position() {
        for (name, field) in small_and_middle_fields() {
            for sign in [-1.0, 1.0] {
                let mut b = rolling_ball_approach_blackboard(field, vector![0.0, 0.4 * sign]);
                let destination = rolling_approach_destination(&mut b, name);
                assert!(destination.x() > 1.0 && destination.x() < 1.5, "{name}");
                assert!(destination.y() * sign > 0.15, "{name}");
            }
        }
    }

    #[test]
    fn incoming_ball_keeps_its_predicted_approach_without_an_early_kick_handoff() {
        for (name, field) in small_and_middle_fields() {
            let mut b = rolling_ball_approach_blackboard(field, vector![-0.4, 0.0]);
            let destination = rolling_approach_destination(&mut b, name);
            assert!(destination.x() > 1.0 && destination.x() < 1.35, "{name}");
            assert!(destination.y().abs() < 1e-5, "{name}");
        }
    }

    #[test]
    fn middle_rolling_ball_trace_approaches_with_room_for_nearby_players() {
        // Middle native match at 161 s: the previous predicted endpoint was
        // 0.461 m from the observed ball, below the required 0.505 m, despite
        // there being room to approach behind it and above the nearby players.
        let field = small_and_middle_fields()[1].1;
        let mut b = rolling_ball_approach_blackboard(field, vector![0.46094358, 0.215333]);
        let transform = Isometry2::from_parts(
            vector![-3.218505, 2.5525792],
            (-0.35022575_f32).atan2(0.9366653),
        );
        b.world_state.robot.ground_to_field = Some(transform);
        let measured = b.world_state.ball.as_mut().unwrap();
        measured.ball_in_ground = point![1.404299, 0.2873733];
        measured.ball_in_field = transform * measured.ball_in_ground;
        b.world_state.obstacles = vec![
            Obstacle::robot(point![0.349715, -0.9696058], 0.2, 0.25),
            Obstacle::robot(point![0.7983835, -0.14753485], 0.2, 0.25),
        ];
        let target = transform * point![3.9687145, -0.8890081];

        assert_eq!(walk_to_ball_action(&mut b, target), Status::Success);
        let Some(BodyMotion::Walk { path, .. }) = &b.body_motion else {
            panic!("the observed free approach must not become Stand");
        };
        let destination = transform * path.end_point();
        assert!((destination - point![-2.045, 2.856]).norm() < 0.01);
        let measured = b.world_state.ball.unwrap();
        let clearance = minimum_ball_clearance(&b);
        assert!((path.end_point() - measured.ball_in_ground).norm() >= clearance);
        assert!(
            (path.project(measured.ball_in_ground) - measured.ball_in_ground).norm()
                >= clearance - 1e-4
        );
        for obstacle in &b.world_state.obstacles {
            assert!(
                (path.project(obstacle.position) - obstacle.position).norm()
                    >= obstacle.radius_at_hip_height
                        + b.parameters.walking.path_planning.robot_radius
                        - 1e-4
            );
        }
    }

    #[test]
    fn a_distant_held_ball_is_covered_from_its_goal_side_with_a_safe_route() {
        let mut b = holding_blackboard(point![1.3, 0.0], Point2::origin(), 0.0);
        b.world_state.obstacles = vec![
            Obstacle::robot(point![0.45, 0.1], 0.1, 0.1),
            Obstacle::robot(point![1.55, 0.0], 0.2, 0.2),
        ];
        let observed = b.world_state.ball.unwrap();

        assert_eq!(execute_ball_action(&mut b), Status::Success);
        let Some(BodyMotion::Walk {
            path,
            orientation_mode,
            target_orientation,
            ..
        }) = &b.body_motion
        else {
            panic!("a distant contest needs a planned blocking position");
        };
        assert!(
            path.segments
                .iter()
                .any(|s| matches!(s, PathSegment::Arc(_)))
        );
        let destination = path.end_point();
        assert!(destination.x() > 0.0 && destination.x() < observed.ball_in_ground.x());
        assert!(destination.y().abs() < 1e-5);
        assert!(target_orientation.angle().abs() < 1e-5);
        assert_eq!(
            *orientation_mode,
            OrientationMode::LookAt {
                target: observed.ball_in_ground,
                tolerance: 0.05,
            }
        );
        // Check the resulting route, rather than just the requested endpoint.
        for obstacle in &b.world_state.obstacles {
            let distance = (path.project(obstacle.position) - obstacle.position).norm();
            assert!(
                distance
                    >= obstacle.radius_at_hip_height
                        + b.parameters.walking.path_planning.robot_radius
                        - 1e-4,
                "route clearance {distance} for {obstacle:?}: {path:?}"
            );
        }
        assert!(
            (path.project(observed.ball_in_ground) - observed.ball_in_ground).norm()
                >= b.parameters.walking.path_planning.robot_radius
                    + b.parameters.walking.path_planning.ball_obstacle_radius
                    - 1e-4
        );
        assert!(b.kick_target.is_none());
        assert!(!b.last_close_enough_to_kick);
        assert_eq!(
            b.world_state.ball.unwrap().ball_in_ground,
            observed.ball_in_ground
        );
    }

    #[test]
    fn hold_stands_when_rule_regions_leave_no_route() {
        use geometry::circle::Circle;
        use types::rule_obstacles::RuleObstacle;

        let mut b = holding_blackboard(point![1.3, 0.0], Point2::origin(), 0.0);
        b.world_state.rule_obstacles = [-0.5, 0.5]
            .into_iter()
            .flat_map(|x| {
                [-0.5, 0.5]
                    .into_iter()
                    .map(move |y| RuleObstacle::Circle(Circle::new(point![x, y], 0.32)))
            })
            .collect();

        assert_eq!(execute_ball_action(&mut b), Status::Success);
        assert!(matches!(b.body_motion, Some(BodyMotion::Stand)));
        assert!(b.kick_target.is_none());
    }

    #[test]
    fn holding_does_not_override_a_restart_takers_release_barrier() {
        use hsl_network_messages::SubState;

        let mut b = holding_blackboard(point![1.3, 0.0], Point2::origin(), 0.0);
        b.restart_release = Some(super::super::restarts::RestartRelease {
            origin: point![1.0, 0.0],
            armed_at: b.world_state.now,
            released: false,
            restart: SubState::CornerKick,
            saw_open_play: true,
        });

        assert_eq!(execute_ball_action(&mut b), Status::Success);
        assert!(matches!(b.body_motion, Some(BodyMotion::Stand)));
        assert!(b.restart_release.as_ref().unwrap().released);
        assert!(b.kick_target.is_none());
    }

    #[test]
    fn holding_covers_the_own_goal_in_both_halves_and_mirrors_across_the_field() {
        for x in [-3.0, 2.0] {
            let mut destinations = Vec::new();
            for sign in [-1.0, 1.0] {
                let ball = point![x, sign];
                let mut b = holding_blackboard(ball, point![0.0, 2.3 * sign], 0.7 * sign);
                let transform = b.world_state.robot.ground_to_field.unwrap();

                assert_eq!(execute_ball_action(&mut b), Status::Success);
                let Some(BodyMotion::Walk { path, .. }) = &b.body_motion else {
                    panic!("the distant ball needs goal-side coverage");
                };
                let destination = transform * path.end_point();
                let own_goal = point![-b.field_dimensions.length / 2.0, 0.0];
                let to_goal = own_goal - ball;
                let to_destination = destination - ball;
                assert!(to_destination.dot(&to_goal) > 0.0);
                assert!(to_destination.norm() < to_goal.norm());
                assert!(
                    (to_goal.x() * to_destination.y() - to_goal.y() * to_destination.x()).abs()
                        < 1e-4
                );
                assert!(destination.x().abs() <= b.field_dimensions.length / 2.0);
                assert!(destination.y().abs() <= b.field_dimensions.width / 2.0);
                destinations.push(destination);
            }
            assert!((destinations[0].x() - destinations[1].x()).abs() < 1e-4);
            assert!((destinations[0].y() + destinations[1].y()).abs() < 1e-4);
        }
    }

    #[test]
    fn holding_stands_close_to_the_ball_instead_of_circling_through_it() {
        let mut b = holding_blackboard(point![0.3, 0.0], Point2::origin(), 1.0);
        assert_eq!(execute_ball_action(&mut b), Status::Success);
        assert!(matches!(b.body_motion, Some(BodyMotion::Stand)));
        assert!(b.kick_target.is_none());
    }

    #[test]
    fn holding_waits_for_the_existing_motion_switch_interval() {
        use std::time::Duration;

        let mut b = holding_blackboard(point![1.3, 0.0], Point2::origin(), 0.0);
        b.last_motion_type = Some(MotionType::Kick);
        b.parameters.control.allow_switch.kick = Duration::from_secs(1);
        b.world_state.now = b.world_state.now + Duration::from_millis(100);
        assert_eq!(execute_ball_action(&mut b), Status::Success);
        assert!(matches!(b.body_motion, Some(BodyMotion::Stand)));
        assert!(b.kick_target.is_none());
    }

    fn overlapping_projection_blackboard() -> Blackboard {
        let mut b = holding_blackboard(point![2.0, 0.0], Point2::origin(), 0.0);
        b.field_dimensions.ball_radius = 0.073;
        b.parameters.walking.path_planning.robot_radius = 0.4;
        b.world_state.obstacles = vec![
            Obstacle::robot(point![1.65, 0.1], 0.2, 0.2),
            Obstacle::robot(point![1.1, -0.55], 0.2, 0.2),
        ];
        b
    }

    #[test]
    fn hold_rejects_a_destination_projected_inside_a_second_robot() {
        let mut b = overlapping_projection_blackboard();
        // The requested [1.477, 0] is projected out of the first robot to about
        // [1.079, -0.230], inside the second robot's original 0.6 m clearance.
        assert_eq!(execute_ball_action(&mut b), Status::Success);
        assert!(matches!(b.body_motion, Some(BodyMotion::Stand)));
        assert!(b.kick_target.is_none());
    }

    #[test]
    fn hold_rechecks_fresh_teammates_and_rule_regions_after_projection() {
        use std::time::Duration;

        use geometry::circle::Circle;
        use types::{
            rule_obstacles::RuleObstacle, time_wrapper::TimeWrapper, world_state::PlayerState,
        };

        for use_teammate in [false, true] {
            let mut b = overlapping_projection_blackboard();
            b.world_state.obstacles.pop();
            if use_teammate {
                b.parameters.voronoi.teammate_pose_max_age = Duration::from_secs(1);
                b.timed_player_states.three = Some(TimeWrapper {
                    time: b.world_state.now,
                    inner: PlayerState {
                        pose: Pose2::new(point![1.1, -0.55], 0.0),
                        ..Default::default()
                    },
                });
            } else {
                b.world_state.rule_obstacles =
                    vec![RuleObstacle::Circle(Circle::new(point![1.1, -0.55], 0.2))];
            }
            assert_eq!(execute_ball_action(&mut b), Status::Success);
            assert!(
                matches!(b.body_motion, Some(BodyMotion::Stand)),
                "projected endpoint must respect teammate={use_teammate}: {:?}",
                b.body_motion
            );
        }
    }

    #[test]
    fn planned_ball_approach_also_rejects_an_occupied_projected_endpoint() {
        let mut b = overlapping_projection_blackboard();
        assert_eq!(
            walk_to_ball_action(&mut b, point![4.5, 0.0]),
            Status::Success
        );
        assert!(matches!(b.body_motion, Some(BodyMotion::Stand)));
    }

    #[test]
    fn projected_pursuit_clears_the_current_ball_and_a_nearby_body() {
        use std::time::Duration;

        let mut b = holding_blackboard(point![1.4, 0.0], Point2::origin(), 0.0);
        b.field_dimensions.ball_radius = 0.073;
        b.parameters.walking.path_planning.robot_radius = 0.4;
        b.parameters.ball.pursuit_lookahead = Duration::from_millis(450);
        b.world_state.ball.as_mut().unwrap().ball_in_ground_velocity = vector![1.0, 0.0];
        b.world_state.obstacles = vec![Obstacle::robot(point![1.35, 0.3], 0.1, 0.1)];
        assert_eq!(
            walk_to_ball_action(&mut b, point![4.5, 0.0]),
            Status::Success
        );
        let Some(BodyMotion::Walk { path, .. }) = &b.body_motion else {
            panic!("a safe approach behind both ball positions is available");
        };
        let measured = b.world_state.ball.unwrap();
        assert!(
            (path.project(measured.ball_in_ground) - measured.ball_in_ground).norm()
                >= minimum_ball_clearance(&b) - 1e-4
        );
        let obstacle = &b.world_state.obstacles[0];
        assert!(
            (path.project(obstacle.position) - obstacle.position).norm()
                >= obstacle.radius_at_hip_height + b.parameters.walking.path_planning.robot_radius
                    - 1e-4
        );
    }

    fn shooting_blackboard(ball: Point2<Field>) -> Blackboard {
        let mut b = holding_blackboard(ball, Point2::origin(), 0.0);
        b.parameters.kicking = parameters();
        b.parameters.kicking.kick_activation_distance = 1.1;
        b.parameters.walking.path_planning.robot_radius = 0.2;
        b.ball_action = Some(BallAction::Shoot {
            opening: LineSegment(point![4.5, -0.5], point![4.5, 0.5]),
        });
        b
    }

    #[test]
    fn a_clear_ball_corridor_keeps_static_body_barriers_at_policy_handoff() {
        for kind in [
            ObstacleKind::GoalPost,
            ObstacleKind::Person,
            ObstacleKind::Unknown,
        ] {
            for (obstacle_y, expect_walk) in [(0.22, true), (0.5, false)] {
                let mut b = shooting_blackboard(point![0.95, 0.0]);
                b.world_state.obstacles = vec![Obstacle {
                    kind,
                    ..Obstacle::robot(point![0.3, obstacle_y], 0.1, 0.1)
                }];
                let action = b.ball_action.unwrap();
                assert!(resolve_action_target(&b, action, point![0.95, 0.0]).is_some());
                assert_eq!(execute_ball_action(&mut b), Status::Success);
                if expect_walk {
                    assert!(matches!(b.body_motion, Some(BodyMotion::Walk { .. })));
                } else {
                    assert!(matches!(b.body_motion, Some(BodyMotion::Kick { .. })));
                }
            }
        }
    }

    #[test]
    fn a_nearby_opponent_body_does_not_block_a_clear_ball_policy_handoff() {
        for position in [point![0.1, 0.18], point![0.3, 0.22]] {
            let mut b = shooting_blackboard(point![0.95, 0.0]);
            b.world_state.obstacles = vec![Obstacle::robot(position, 0.1, 0.1)];
            assert!(resolve_action_target(&b, b.ball_action.unwrap(), point![0.95, 0.0]).is_some());
            let launch = LineSegment(Point2::origin(), point![0.65, 0.0]);
            assert!(!has_body_path_clearance(
                &b,
                launch,
                Isometry2::identity(),
                BodyClearancePurpose::Walking,
            ));
            assert!(has_body_path_clearance(
                &b,
                launch,
                Isometry2::identity(),
                BodyClearancePurpose::BallPolicyHandoff,
            ));
            assert_eq!(execute_ball_action(&mut b), Status::Success);
            assert!(matches!(b.body_motion, Some(BodyMotion::Kick { .. })));
        }
    }

    #[test]
    fn a_fresh_teammate_still_blocks_policy_handoff_even_when_observed_as_a_robot() {
        use std::time::Duration;
        use types::{time_wrapper::TimeWrapper, world_state::PlayerState};

        let mut b = shooting_blackboard(point![0.95, 0.0]);
        let position = point![0.3, 0.22];
        b.world_state.obstacles = vec![Obstacle::robot(position, 0.1, 0.1)];
        b.parameters.voronoi.teammate_pose_max_age = Duration::from_secs(1);
        b.timed_player_states.three = Some(TimeWrapper {
            time: b.world_state.now,
            inner: PlayerState {
                pose: Pose2::new(b.world_state.robot.ground_to_field.unwrap() * position, 0.0),
                ..Default::default()
            },
        });
        assert!(resolve_action_target(&b, b.ball_action.unwrap(), point![0.95, 0.0]).is_some());
        assert!(!has_body_path_clearance(
            &b,
            LineSegment(Point2::origin(), point![0.65, 0.0]),
            Isometry2::identity(),
            BodyClearancePurpose::BallPolicyHandoff,
        ));
        assert_eq!(execute_ball_action(&mut b), Status::Success);
        assert!(!matches!(b.body_motion, Some(BodyMotion::Kick { .. })));
    }

    #[test]
    fn a_walking_ball_approach_still_plans_around_observed_robots() {
        let mut b = shooting_blackboard(point![1.5, 0.0]);
        let obstacle = Obstacle::robot(point![0.45, 0.1], 0.1, 0.1);
        b.world_state.obstacles = vec![obstacle];
        assert_eq!(execute_ball_action(&mut b), Status::Success);
        let Some(BodyMotion::Walk { path, .. }) = &b.body_motion else {
            panic!("a distant ball still needs obstacle-aware walking");
        };
        assert!(
            path.segments
                .iter()
                .any(|segment| matches!(segment, PathSegment::Arc(_)))
        );
        assert!(
            (path.project(obstacle.position) - obstacle.position).norm()
                >= obstacle.radius_at_hip_height + b.parameters.walking.path_planning.robot_radius
                    - 1e-4
        );
    }

    #[test]
    fn coincident_robot_proxy_and_ball_select_hold_instead_of_a_policy_handoff() {
        let mut b = shooting_blackboard(point![0.95, 0.0]);
        b.world_state.obstacles = vec![Obstacle::robot(point![0.95, 0.0], 0.2, 0.25)];
        assert_eq!(
            super::super::strategy::select_ball_action(&mut b),
            Status::Success
        );
        assert_eq!(b.ball_action, Some(BallAction::Hold));
        assert_eq!(execute_ball_action(&mut b), Status::Success);
        assert!(!matches!(b.body_motion, Some(BodyMotion::Kick { .. })));
    }

    #[test]
    fn initial_robot_proxy_overlap_keeps_fresh_teammate_and_rule_handoff_guards() {
        use geometry::circle::Circle;
        use std::time::Duration;
        use types::{
            rule_obstacles::RuleObstacle, time_wrapper::TimeWrapper, world_state::PlayerState,
        };

        for guard in 0..3 {
            let mut b = shooting_blackboard(point![0.45, 0.0]);
            let proxy = point![0.3, 0.08];
            b.world_state.obstacles = vec![Obstacle::robot(proxy, 0.2, 0.25)];
            if guard == 1 {
                b.parameters.voronoi.teammate_pose_max_age = Duration::from_secs(1);
                b.timed_player_states.three = Some(TimeWrapper {
                    time: b.world_state.now,
                    inner: PlayerState {
                        pose: Pose2::new(b.world_state.robot.ground_to_field.unwrap() * proxy, 0.0),
                        ..Default::default()
                    },
                });
            } else if guard == 2 {
                b.world_state.rule_obstacles =
                    vec![RuleObstacle::Circle(Circle::new(point![0.1, 0.0], 0.025))];
            }
            assert!((b.world_state.ball.unwrap().ball_in_ground - proxy).norm() < 0.2);
            assert!(resolve_action_target(&b, b.ball_action.unwrap(), point![0.45, 0.0]).is_some());
            assert_eq!(execute_ball_action(&mut b), Status::Success);
            assert_eq!(
                matches!(b.body_motion, Some(BodyMotion::Kick { .. })),
                guard == 0,
                "proxy egress must preserve the teammate/rule handoff guard {guard}: {:?}",
                b.body_motion
            );
        }
    }

    #[test]
    fn handoff_allows_ordinary_foot_contact_with_the_ball() {
        let mut b = shooting_blackboard(point![0.3, 0.0]);
        b.parameters.walking.path_planning.robot_radius = 0.4;
        b.world_state.obstacles = vec![Obstacle::ball(point![0.3, 0.0], 0.073)];
        assert_eq!(execute_ball_action(&mut b), Status::Success);
        assert!(matches!(b.body_motion, Some(BodyMotion::Kick { .. })));
    }

    #[test]
    fn body_access_checks_rule_crossings_even_when_both_endpoints_are_clear() {
        use geometry::{circle::Circle, rectangle::Rectangle};
        use types::rule_obstacles::RuleObstacle;

        let mut b = shooting_blackboard(point![0.95, 0.0]);
        for obstacle in [
            RuleObstacle::Circle(Circle::new(point![0.3, 0.0], 0.05)),
            RuleObstacle::Rectangle(Rectangle {
                min: point![0.275, -0.05],
                max: point![0.325, 0.05],
            }),
        ] {
            b.world_state.rule_obstacles = vec![obstacle];
            assert!(has_rule_path_clearance(
                &b,
                LineSegment(point![0.0, 0.0], point![0.0, 0.0]),
                0.2,
            ));
            assert!(has_rule_path_clearance(
                &b,
                LineSegment(point![0.65, 0.0], point![0.65, 0.0]),
                0.2,
            ));
            assert!(!has_body_path_clearance(
                &b,
                LineSegment(point![0.0, 0.0], point![0.65, 0.0]),
                Isometry2::identity(),
                BodyClearancePurpose::BallPolicyHandoff,
            ));
            assert_eq!(execute_ball_action(&mut b), Status::Success);
            assert!(!matches!(b.body_motion, Some(BodyMotion::Kick { .. })));
        }
    }

    #[test]
    fn nearby_balls_can_enter_the_policy_before_final_alignment() {
        let parameters = parameters();
        assert!(handoff_ready(
            point![0.6, 0.0],
            Vector2::zeros(),
            vector![-1.0, 0.0],
            false,
            &parameters,
            0.45,
            true,
        ));
        assert!(handoff_ready(
            point![-0.3, 0.0],
            Vector2::zeros(),
            vector![1.0, 0.0],
            true,
            &parameters,
            0.45,
            true,
        ));
        assert!(handoff_ready(
            point![0.1, 0.6],
            Vector2::zeros(),
            vector![1.0, 0.0],
            false,
            &parameters,
            0.45,
            true,
        ));
    }

    #[test]
    fn nearby_side_back_and_rotated_states_reach_the_policy_with_observed_ball_inputs() {
        for (ball_in_field, heading, velocity) in [
            (point![-0.3, 0.0], 0.0, vector![0.0, 0.0]),
            (point![0.1, 0.6], 0.0, vector![0.0, 0.0]),
            (point![-0.6, 0.0], std::f32::consts::PI, vector![0.0, 0.0]),
            (point![0.6, 0.0], 0.0, vector![2.0, 0.0]),
        ] {
            let mut b = shooting_blackboard(ball_in_field);
            let transform = Isometry2::from_parts(vector![0.0, 0.0], heading);
            b.world_state.robot.ground_to_field = Some(transform);
            let ball = b.world_state.ball.as_mut().unwrap();
            ball.ball_in_ground = transform.inverse() * ball_in_field;
            ball.ball_in_ground_velocity = velocity;
            let observed = *ball;

            assert_eq!(execute_ball_action(&mut b), Status::Success);
            let Some(BodyMotion::Kick {
                ball_position,
                ball_velocity,
                ..
            }) = b.body_motion
            else {
                panic!("the close locomotion policy owns final alignment: {observed:?}");
            };
            assert_eq!(ball_position, observed.ball_in_ground);
            assert_eq!(ball_velocity, observed.ball_in_ground_velocity);
        }
    }

    #[test]
    fn a_planned_approach_from_beyond_the_ball_keeps_it_visible() {
        use linear_algebra::Isometry2;
        use types::{
            field_dimensions::FieldDimensions,
            path::PathSegment,
            world_state::{BallSource, BallState},
        };

        let mut b = Blackboard::default();
        b.field_dimensions = FieldDimensions::SPL_2025;
        b.world_state.robot.ground_to_field = Some(Isometry2::from_parts(vector![1.0, 0.0], 0.0));
        b.world_state.ball = Some(BallState {
            ball_in_field: point![0.5, 0.0],
            ball_in_ground: point![-0.5, 0.0],
            source: BallSource::Own,
            ..Default::default()
        });
        b.parameters.kicking.approach_ball_standoff = 0.3;
        let path = &mut b.parameters.walking.path_planning;
        path.robot_radius = 0.4;
        path.ball_obstacle_radius = 0.05;
        path.rotation_penalty_factor = 0.4;
        let walk = &mut b.parameters.walking.walk_and_stand;
        walk.orientation_tolerance = 0.05;
        walk.normal_distance_to_be_aligned = 0.05;
        walk.target_reached_thresholds = nalgebra::Vector2::new(0.05, 0.1);
        b.parameters.walking.speed.kicking = 1.0;
        b.parameters.walking.speed.minimum_speed = 0.2;
        b.parameters.walking.speed.velocity_fade_distance = 3.0;

        assert_eq!(
            walk_to_ball_action(&mut b, point![4.5, 0.0]),
            Status::Success
        );
        let Some(BodyMotion::Walk {
            path,
            orientation_mode,
            target_orientation,
            ..
        }) = b.body_motion
        else {
            panic!("this explicitly planned approach must keep its ball-clearance route");
        };
        assert!(
            path.segments
                .iter()
                .any(|segment| matches!(segment, PathSegment::Arc(_)))
        );
        assert_eq!(
            orientation_mode,
            OrientationMode::LookAt {
                target: point![-0.5, 0.0],
                tolerance: 0.05
            },
        );
        assert!(target_orientation.angle().abs() < f32::EPSILON);
    }

    #[test]
    fn planner_standoff_can_hand_over_without_walking_through_ball() {
        let parameters = parameters();
        assert!(handoff_ready(
            point![0.5, 0.0],
            Vector2::zeros(),
            vector![1.0, 0.0],
            false,
            &parameters,
            0.45,
            true,
        ));
        assert!(!handoff_ready(
            point![0.8, 0.0],
            Vector2::zeros(),
            vector![1.0, 0.0],
            false,
            &parameters,
            0.45,
            true,
        ));
        assert!(handoff_ready(
            point![0.8, 0.0],
            Vector2::zeros(),
            vector![1.0, 0.0],
            true,
            &parameters,
            0.45,
            true,
        ));
    }

    #[test]
    fn nearby_rolling_ball_uses_the_policy_while_distant_interception_is_bounded() {
        let parameters = parameters();
        assert!(handoff_ready(
            point![0.6, 0.0],
            vector![2.0, 0.0],
            vector![1.0, 0.0],
            false,
            &parameters,
            0.45,
            true,
        ));
        assert!(!handoff_ready(
            point![1.2, 0.0],
            vector![2.0, 0.0],
            vector![1.0, 0.0],
            false,
            &parameters,
            0.45,
            true,
        ));
        // The ball reaches the forward kicking plane at [0.15,0.39] in 0.35s.
        let ball = point![1.2, -0.1];
        let velocity = vector![-3.0, 1.4];
        assert!(handoff_ready(
            ball,
            velocity,
            vector![1.0, 0.0],
            false,
            &parameters,
            0.45,
            true
        ));
        assert!(!handoff_ready(
            ball,
            velocity,
            vector![1.0, 0.0],
            false,
            &parameters,
            0.2,
            true
        ));
        assert!(!handoff_ready(
            ball,
            velocity,
            vector![1.0, 0.0],
            false,
            &parameters,
            0.45,
            false
        ));
    }

    #[test]
    fn incoming_ball_missing_the_forward_corridor_is_not_a_kick() {
        let parameters = parameters();
        assert!(!handoff_ready(
            point![1.3, 1.0],
            vector![-3.0, 0.0],
            vector![1.0, 0.0],
            false,
            &parameters,
            0.5,
            true,
        ));
    }
}
