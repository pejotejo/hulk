use coordinate_systems::Field;
use filtering::hysteresis::{
    greater_than_with_relative_hysteresis, less_than_with_relative_hysteresis,
};
use geometry::{Distance, line_segment::LineSegment};
use linear_algebra::point;
use ros_z::Message;
use serde::{Deserialize, Serialize};
use types::{behavior_tree::Status, obstacles::ObstacleKind};

use crate::node::Blackboard;

use super::{geometry::goal_openings, kick_ball_position_in_ground};

/// Tactical intent only. Execution chooses the target, approach, and physical motion.
/// A future pass variant can identify a receiver and reception region.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Message)]
pub enum BallAction {
    Shoot { opening: LineSegment<Field> },
    Dribble,
    Clear,
}

/// Select ordinary striker intent. Clear `ball_action` when leaving that context.
/// Execution still needs to validate clear/dribble corridors and kick availability.
pub fn select_ball_action(blackboard: &mut Blackboard) -> Status {
    blackboard.ball_action = choose_ball_action(blackboard);
    if blackboard.ball_action.is_some() {
        Status::Success
    } else {
        Status::Failure
    }
}

fn choose_ball_action(blackboard: &Blackboard) -> Option<BallAction> {
    let world = &blackboard.world_state;
    let field = &blackboard.field_dimensions;
    let parameters = &blackboard.parameters.kicking.strategy;
    let previous = blackboard.ball_action;
    let ground_to_field = world.robot.ground_to_field?;
    let ball_in_ground = kick_ball_position_in_ground(blackboard, &ground_to_field)?;
    let ball = ground_to_field * ball_in_ground;
    let goal_x = field.length / 2.0;
    if ball.x().abs() >= goal_x || ball.y().abs() >= field.width / 2.0 {
        return None;
    }

    let was_clearing = matches!(previous, Some(BallAction::Clear));
    let own_goal = LineSegment(
        point![-goal_x, -field.goal_inner_width / 2.0],
        point![-goal_x, field.goal_inner_width / 2.0],
    );
    // Robot affiliation is unknown; nearby robots conservatively count as pressure.
    let under_pressure = world.obstacles.iter().any(|obstacle| {
        matches!(
            obstacle.kind,
            ObstacleKind::Robot | ObstacleKind::Person | ObstacleKind::Unknown
        ) && less_than_with_relative_hysteresis(
            was_clearing,
            (obstacle.position - ball_in_ground).norm() - obstacle.radius_at_foot_height,
            parameters.pressure_distance,
            0.0..=parameters.clear_distance_hysteresis,
        )
    });
    if under_pressure
        && less_than_with_relative_hysteresis(
            was_clearing,
            own_goal.distance_to(ball),
            parameters.clear_goal_distance,
            0.0..=parameters.clear_distance_hysteresis,
        )
    {
        return Some(BallAction::Clear);
    }

    let range = parameters.maximum_shot_distance;
    if range.is_finite() && range > goal_x - ball.x() {
        // Clip regions to reachable goal points before rating their angular width.
        let reachable_y = (range.powi(2) - (goal_x - ball.x()).powi(2)).sqrt();
        let mut best = None;
        let mut retained = None;
        for opening in goal_openings(
            ball,
            ground_to_field,
            &world.obstacles,
            field,
            parameters.obstacle_clearance,
        ) {
            let lower = opening.0.y().max(ball.y() - reachable_y);
            let upper = opening.1.y().min(ball.y() + reachable_y);
            if lower >= upper {
                continue;
            }
            let opening = LineSegment(point![goal_x, lower], point![goal_x, upper]);
            let angle = LineSegment(ball, opening.0).angle(LineSegment(ball, opening.1));
            if angle < parameters.minimum_shot_angle {
                continue;
            }
            if best.is_none_or(|(_, best_angle)| angle > best_angle) {
                best = Some((opening, angle));
            }
            if let Some(BallAction::Shoot { opening: previous }) = previous
                && previous.0.x() == goal_x
                && (lower..upper).contains(&previous.center().y())
            {
                retained = Some((opening, angle));
            }
        }
        if let Some((opening, angle)) = best
            && greater_than_with_relative_hysteresis(
                matches!(previous, Some(BallAction::Shoot { .. })),
                angle,
                parameters.minimum_shot_angle,
                0.0..=parameters.shot_angle_hysteresis,
            )
        {
            let opening = retained
                .filter(|(_, old_angle)| angle <= old_angle + parameters.shot_angle_hysteresis)
                .map_or(opening, |(opening, _)| opening);
            return Some(BallAction::Shoot { opening });
        }
    }

    Some(BallAction::Dribble)
}

#[cfg(test)]
mod tests {
    use types::{
        ball_position::BallPosition,
        field_dimensions::{FieldDimensions, Side},
        motion_command::BodyMotion,
        obstacles::Obstacle,
        parameters::KickStrategyParameters,
    };

    use crate::{kick::execution::execute_ball_action, node::LastBall, tree::striker_subtree};

    use super::*;

    fn blackboard() -> Blackboard {
        let mut blackboard = Blackboard {
            field_dimensions: FieldDimensions::SPL_2025,
            ball: Some(LastBall {
                position: point![2.0, 0.0],
                velocity: Default::default(),
                age: Default::default(),
                field_side: Side::Left,
                source: types::world_state::BallSource::Own,
            }),
            ..Default::default()
        };
        blackboard.world_state.robot.ground_to_field = Some(Default::default());
        blackboard.parameters.kicking.strategy = KickStrategyParameters {
            maximum_shot_distance: 3.0,
            obstacle_clearance: 0.05,
            minimum_shot_angle: 0.02,
            shot_angle_hysteresis: 0.05,
            pressure_distance: 0.8,
            clear_goal_distance: 1.5,
            clear_distance_hysteresis: 0.2,
        };
        blackboard
    }

    #[test]
    fn selects_intent_from_blackboard_and_clears_it_without_ball() {
        let mut blackboard = blackboard();
        assert_eq!(select_ball_action(&mut blackboard), Status::Success);
        assert!(matches!(
            blackboard.ball_action,
            Some(BallAction::Shoot { .. })
        ));
        blackboard.world_state.obstacles = vec![Obstacle::robot(point![3.0, 0.0], 0.5, 0.5)];
        select_ball_action(&mut blackboard);
        assert_eq!(blackboard.ball_action, Some(BallAction::Dribble));
        blackboard.ball.as_mut().unwrap().position = point![-3.8, 0.0];
        blackboard.world_state.obstacles = vec![Obstacle::robot(point![-3.2, 0.0], 0.2, 0.2)];
        select_ball_action(&mut blackboard);
        assert_eq!(blackboard.ball_action, Some(BallAction::Clear));
        blackboard.ball = None;
        assert_eq!(select_ball_action(&mut blackboard), Status::Failure);
        assert_eq!(blackboard.ball_action, None);
    }

    #[test]
    fn dribble_advances_toward_goal_through_a_clear_corridor() {
        let mut blackboard = blackboard();
        let ball = point![1.0, 1.0];
        blackboard.ball.as_mut().unwrap().position = ball;
        blackboard.parameters.kicking.kick_activation_distance = 10.0;
        select_ball_action(&mut blackboard);
        assert_eq!(blackboard.ball_action, Some(BallAction::Dribble));
        execute_ball_action(&mut blackboard);
        let goal = point![blackboard.field_dimensions.length / 2.0, 0.0];
        let target =
            blackboard.world_state.robot.ground_to_field.unwrap() * blackboard.kick_target.unwrap();
        assert!(target.x() > ball.x() && target.x() < goal.x());
        assert!(LineSegment(ball, target).angle(LineSegment(ball, goal)) < 1e-6);

        let obstacle = Obstacle::robot(point![2.0, 0.714], 0.2, 0.2);
        blackboard.world_state.obstacles = vec![obstacle];
        execute_ball_action(&mut blackboard);
        let target =
            blackboard.world_state.robot.ground_to_field.unwrap() * blackboard.kick_target.unwrap();
        assert!(
            LineSegment(ball, target).distance_to(
                blackboard.world_state.robot.ground_to_field.unwrap() * obstacle.position,
            ) > obstacle.radius_at_foot_height
                + blackboard.field_dimensions.ball_radius
                + blackboard.parameters.kicking.strategy.obstacle_clearance
        );

        // Strategy and execution must use the same observed ball, not disagree on range.
        blackboard.world_state.obstacles.clear();
        blackboard.visual_kick_ball_position = Some(BallPosition {
            position: point![2.0, 0.0],
            velocity: Default::default(),
            last_seen: Default::default(),
        });
        select_ball_action(&mut blackboard);
        assert!(matches!(
            blackboard.ball_action,
            Some(BallAction::Shoot { .. })
        ));
        execute_ball_action(&mut blackboard);
        assert_eq!(blackboard.kick_target.unwrap().x(), goal.x());
    }

    #[test]
    fn retains_shot_side_under_small_changes_but_rejects_blocked_opening() {
        let mut blackboard = blackboard();
        blackboard.world_state.obstacles = vec![Obstacle::robot(point![3.5, -0.02], 0.1, 0.1)];
        for y in [-0.02, 0.02] {
            blackboard.world_state.obstacles[0].position = point![3.5, y];
            select_ball_action(&mut blackboard);
            let Some(BallAction::Shoot { opening }) = blackboard.ball_action else {
                panic!("expected shot")
            };
            assert!(opening.center().y() > 0.0);
        }
        blackboard.world_state.obstacles[0].radius_at_foot_height = 1.0;
        select_ball_action(&mut blackboard);
        assert_eq!(blackboard.ball_action, Some(BallAction::Dribble));
    }

    #[test]
    fn execution_approaches_ball_revalidates_and_kicks_to_advance() {
        let mut blackboard = blackboard();
        blackboard.parameters.kicking.kick_activation_distance = 1.0;
        blackboard.parameters.kicking.approach_ball_standoff = 0.3;
        blackboard
            .parameters
            .walking
            .walk_and_stand
            .target_reached_thresholds = nalgebra::vector![0.05, 0.1];
        blackboard.parameters.walking.path_planning.robot_radius = 0.4;
        blackboard
            .parameters
            .walking
            .path_planning
            .ball_obstacle_radius = 0.05;
        let transform =
            linear_algebra::Isometry2::from_parts(linear_algebra::vector![0.2, -0.1], 0.3);
        blackboard.world_state.robot.ground_to_field = Some(transform);
        blackboard.world_state.obstacles = vec![Obstacle::robot(
            transform.inverse() * point![3.5, 0.0],
            0.1,
            0.1,
        )];
        let tree = striker_subtree();
        assert_eq!(tree.tick_with_trace(&mut blackboard).0, Status::Success);
        let target = transform * blackboard.kick_target.unwrap();
        let Some(BodyMotion::Walk {
            target_orientation, ..
        }) = blackboard.body_motion
        else {
            panic!("expected approach")
        };
        let ball_in_ground = transform.inverse() * point![2.0, 0.0];
        let facing_ball = linear_algebra::Orientation2::from_vector(ball_in_ground.coords());
        assert!((target_orientation.angle() - facing_ball.angle()).abs() < 1e-6);
        let expected = linear_algebra::Orientation2::from_vector(
            transform.inverse() * (target - point![2.0, 0.0]),
        );
        blackboard.parameters.kicking.kick_activation_distance = 3.0;
        tree.tick_with_trace(&mut blackboard);
        let Some(BodyMotion::Kick { kick_direction, .. }) = blackboard.body_motion else {
            panic!("expected shot")
        };
        assert_eq!(blackboard.kick_target, Some(transform.inverse() * target));
        assert!((kick_direction.angle() - expected.angle()).abs() < 1e-6);
        blackboard.world_state.obstacles[0].radius_at_foot_height = 1.0;
        execute_ball_action(&mut blackboard);
        assert_eq!(blackboard.body_motion, Some(BodyMotion::Stand));
        assert_eq!(blackboard.kick_target, None);

        blackboard.world_state.robot.ground_to_field = Some(Default::default());
        blackboard.ball.as_mut().unwrap().position = point![-3.8, 0.0];
        blackboard.world_state.obstacles = vec![Obstacle::robot(point![-3.8, 0.7], 0.2, 0.2)];
        blackboard.parameters.kicking.kick_activation_distance = 5.0;
        tree.tick_with_trace(&mut blackboard);
        assert_eq!(blackboard.ball_action, Some(BallAction::Clear));
        assert!(blackboard.kick_target.unwrap().x() > -3.8);
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick { .. })
        ));

        // The recorded stall had 6 cm lateral error; kick-policy alignment must handle it.
        blackboard.parameters.kicking.kick_activation_distance = 2.0;
        blackboard.ball.as_mut().unwrap().position = point![0.51, -0.06];
        blackboard.world_state.obstacles.clear();
        blackboard.parameters.kicking.allow_strong_kicks = true;
        blackboard
            .parameters
            .kicking
            .strong_kick_min_target_distance = 0.1;
        blackboard.last_motion_command = types::motion_command::MotionCommand::from_partial_motions(
            BodyMotion::Kick {
                ball_position: point![0.51, -0.06],
                ball_velocity: Default::default(),
                target_speed: 1.0,
                soft: false,
                quick: false,
                kick_direction: Default::default(),
                strong: true,
            },
            types::motion_command::HeadMotion::ZeroAngles,
        );
        tree.tick_with_trace(&mut blackboard);
        assert_eq!(blackboard.ball_action, Some(BallAction::Dribble));
        assert!(matches!(
            blackboard.body_motion,
            Some(BodyMotion::Kick {
                strong: false,
                soft: true,
                ..
            })
        ));
        assert!(blackboard.kick_target.unwrap().x() > 0.51);
        blackboard.world_state.obstacles = vec![Obstacle::robot(point![0.6, 0.0], 0.3, 0.3)];
        tree.tick_with_trace(&mut blackboard);
        assert_eq!(blackboard.body_motion, Some(BodyMotion::Stand));
    }
}
