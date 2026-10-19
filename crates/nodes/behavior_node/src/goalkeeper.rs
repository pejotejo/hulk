use coordinate_systems::Field;
use hsl_network_messages::SubState;
use linear_algebra::{Orientation2, Point2, Pose2, Vector2, point};
use types::{behavior_tree::Status, motion_command::OrientationMode, motion_type::MotionType};

use crate::{
    action,
    behavior_tree::Node,
    condition,
    conditions::{hulks_is_kicking_team, is_closest_to_ball},
    head::look_at_ball_subtree,
    kick::{
        apply_kick_target, ball::kick_ball_state, disable_strong_kick, intercept, kick,
        kick_alternatives_subtree,
    },
    negation,
    node::Blackboard,
    selection, sequence,
    substates::{is_in_sub_state, is_sub_state},
    subtree,
    switch_motion_type::switch_motion_type,
    tree::striker_subtree,
    voronoi::calculate_voronoi_grid,
    walk::{walk_alternatives_subtree, walk_to, walk_to_block_position},
};

pub fn goalkeeper_subtree() -> Node<Blackboard> {
    sequence!(
        subtree!(look_at_ball_subtree),
        selection!(
            sequence!(
                condition!(is_in_sub_state),
                subtree!(goalkeeper_sub_state_subtree)
            ),
            sequence!(
                condition!(is_goalkeeper_kick_away_needed),
                switch_motion_type(
                    MotionType::Kick,
                    sequence!(
                        action!(kick),
                        action!(select_goalkeeper_kick_away_target),
                        action!(disable_strong_kick),
                    ),
                    subtree!(kick_alternatives_subtree),
                )
            ),
            sequence!(
                condition!(is_goalkeeper_interception_candidate),
                switch_motion_type(
                    MotionType::Kick,
                    sequence!(
                        action!(kick),
                        action!(intercept),
                        action!(disable_strong_kick),
                    ),
                    subtree!(kick_alternatives_subtree),
                )
            ),
            sequence!(
                condition!(is_goalkeeper_striker_candidate),
                selection!(sequence!(
                    action!(calculate_voronoi_grid),
                    condition!(is_closest_to_ball),
                    subtree!(striker_subtree),
                ),),
            ),
            sequence!(
                condition!(is_ball_near_own_goal),
                subtree!(goalkeeper_active_defense_position_subtree),
            ),
            subtree!(goalkeeper_default_position_subtree),
        ),
    )
}

fn goalkeeper_sub_state_subtree() -> Node<Blackboard> {
    selection!(
        sequence!(
            condition!(is_sub_state, SubState::PenaltyKick),
            negation!(condition!(hulks_is_kicking_team)),
            switch_motion_type(
                MotionType::Walk,
                action!(walk_to_goalkeeper_penalty_position),
                subtree!(walk_alternatives_subtree),
            )
        ),
        sequence!(
            negation!(condition!(hulks_is_kicking_team)),
            selection!(
                condition!(is_sub_state, SubState::CornerKick),
                sequence!(
                    selection!(
                        condition!(is_sub_state, SubState::ThrowIn),
                        condition!(is_sub_state, SubState::IndirectFreeKick),
                        condition!(is_sub_state, SubState::DirectFreeKick),
                    ),
                    condition!(is_ball_near_own_goal)
                ),
            ),
            subtree!(goalkeeper_active_defense_position_subtree)
        ),
        sequence!(
            condition!(is_goalkeeper_striker_candidate),
            action!(calculate_voronoi_grid),
            condition!(is_closest_to_ball),
            subtree!(striker_subtree),
        ),
        subtree!(goalkeeper_default_position_subtree),
    )
}

fn goalkeeper_active_defense_position_subtree() -> Node<Blackboard> {
    switch_motion_type(
        MotionType::Walk,
        sequence!(
            action!(set_goalkeeper_active_defense_position),
            action!(walk_to_block_position)
        ),
        Node::Failure,
    )
}

fn goalkeeper_default_position_subtree() -> Node<Blackboard> {
    switch_motion_type(
        MotionType::Walk,
        action!(walk_to_goalkeeper_default_position),
        subtree!(walk_alternatives_subtree),
    )
}

fn is_goalkeeper_interception_candidate(blackboard: &mut Blackboard) -> bool {
    if !is_ball_near_own_goal(blackboard) {
        return false;
    }

    let Some(ball_velocity) = ball_interception_velocity_in_field(blackboard) else {
        return false;
    };

    if !is_interception_velocity_towards_own_half(blackboard, ball_velocity) {
        return false;
    }

    if let Some(ball) = &blackboard.ball {
        let field_dimensions = blackboard.field_dimensions;
        let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field else {
            return false;
        };

        let own_goal_x = -field_dimensions.length / 2.0;
        let interception_line_x = own_goal_x + blackboard.parameters.goalkeeper.x_offset;
        let time_to_interception_line =
            (interception_line_x - ball.position.x()) / ball_velocity.x();

        if time_to_interception_line <= 0.0 {
            return false;
        }

        let y_at_interception_line =
            ball.position.y() + ball_velocity.y() * time_to_interception_line;
        let goal_half_width = field_dimensions.goal_inner_width / 2.0
            + field_dimensions.goal_post_diameter / 2.0
            + field_dimensions.ball_radius;

        if y_at_interception_line.abs() >= goal_half_width {
            return false;
        }

        let ball_in_ground = ground_to_field.inverse() * ball.position;
        let velocity = ground_to_field.inverse() * ball.velocity;
        let time_to_closest_approach =
            -ball_in_ground.coords().dot(&velocity) / velocity.norm_squared();
        if time_to_closest_approach < 0.0 {
            return false;
        }

        let interception_point = ball_in_ground + velocity * time_to_closest_approach;
        interception_point.x()
            >= blackboard
                .parameters
                .kicking
                .minimum_interception_forward_distance
            && interception_point.coords().norm()
                <= blackboard
                    .parameters
                    .ball
                    .interception
                    .maximum_intercept_distance
    } else {
        false
    }
}

fn is_goalkeeper_kick_away_needed(blackboard: &mut Blackboard) -> bool {
    let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field else {
        return false;
    };
    let Some((ball, _)) = kick_ball_state(blackboard, &ground_to_field) else {
        return false;
    };
    let ball_in_field = ground_to_field * ball;
    let field = &blackboard.field_dimensions;
    let own_penalty_area_x = -field.length / 2.0 + field.penalty_area_length;
    if ball_in_field.x() >= own_penalty_area_x
        || ball_in_field.y().abs() >= field.penalty_area_width / 2.0
    {
        return false;
    }
    let target = ground_to_field.inverse() * goalkeeper_kick_away_target(blackboard, ball_in_field);
    let Some(direction) = (target - ball).try_normalize(f32::EPSILON) else {
        return false;
    };
    let approach = &blackboard.parameters.kicking.approach;
    // Urgency does not make a backwards or sideways policy entry feasible.
    // Use the same measured ball and target as the actual kick command.
    ball.x() > 0.0
        && ball.coords().norm()
            < blackboard
                .parameters
                .goalkeeper
                .kick_away_ball_maximum_robot_distance
        && ball.coords().dot(&direction) > 0.0
        && direction.y().atan2(direction.x()).abs() <= approach.maximum_heading
        && (ball.x() * direction.y() - ball.y() * direction.x()).abs()
            <= approach.maximum_lateral_distance
}

/// Only assign the goalkeeper balls that its behavior is willing to pursue.
pub fn is_goalkeeper_striker_candidate(blackboard: &mut Blackboard) -> bool {
    if is_in_sub_state(blackboard) {
        return hulks_is_kicking_team(blackboard) && is_sub_state(blackboard, SubState::GoalKick);
    }

    blackboard.ball.as_ref().is_some_and(|ball| {
        let own_goal_x = -blackboard.field_dimensions.length / 2.0;
        let maximum_ball_x = own_goal_x + blackboard.parameters.goalkeeper.striker_distance;

        ball.position.x() < maximum_ball_x
    })
}

fn is_ball_near_own_goal(blackboard: &mut Blackboard) -> bool {
    blackboard.ball.as_ref().is_some_and(|ball| {
        let own_goal_x = -blackboard.field_dimensions.length / 2.0;
        let maximum_ball_x = own_goal_x + blackboard.parameters.goalkeeper.passive_distance;

        ball.position.x() < maximum_ball_x
    })
}
pub fn set_goalkeeper_active_defense_position(blackboard: &mut Blackboard) -> Status {
    if let (Some(ball), Some(ground_to_field)) = (
        &blackboard.ball,
        blackboard.world_state.robot.ground_to_field,
    ) {
        let field_dimensions = blackboard.field_dimensions;
        let parameters = &blackboard.parameters.goalkeeper;

        let own_goal_line_x = -field_dimensions.length / 2.0;
        let own_goal_center = point!(own_goal_line_x, 0.0);
        let goal_to_ball = ball.position - own_goal_center;
        let defense_radius =
            (field_dimensions.goal_inner_width / 2.0 - parameters.distance_to_goalpost).max(0.0);
        let unclamped_defense_position_in_field = if goal_to_ball.norm() < f32::EPSILON {
            point!(own_goal_line_x + defense_radius, 0.0)
        } else {
            own_goal_center + goal_to_ball.normalize() * defense_radius
        };
        let minimum_defense_x = own_goal_line_x + parameters.distance_to_goalpost;
        let defense_position_in_field = point!(
            unclamped_defense_position_in_field
                .x()
                .max(minimum_defense_x),
            unclamped_defense_position_in_field.y()
        );
        let defense_position_in_ground = ground_to_field.inverse() * defense_position_in_field;

        if defense_position_in_ground.coords().norm()
            > parameters.active_defense_maximum_robot_distance
        {
            return Status::Failure;
        }

        blackboard.walk_position = Some(defense_position_in_ground);

        Status::Success
    } else {
        Status::Failure
    }
}

pub fn walk_to_goalkeeper_default_position(blackboard: &mut Blackboard) -> Status {
    if let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field {
        let field_dimensions = blackboard.field_dimensions;
        let own_goal_line_x = -field_dimensions.length / 2.0;
        let default_position_in_field = point!(
            own_goal_line_x + blackboard.parameters.goalkeeper.x_offset,
            0.0
        );
        let default_pose_in_field =
            Pose2::from_parts(default_position_in_field, Orientation2::new(0.0));
        let default_pose_in_ground = ground_to_field.inverse() * default_pose_in_field;
        let walk_and_stand = blackboard.parameters.walking.walk_and_stand;
        let blocking_speed = blackboard.parameters.walking.speed.blocking;
        let orientation_mode = if let Some(ball) = &blackboard.world_state.ball {
            OrientationMode::LookAt {
                target: ball.ball_in_ground,
                tolerance: walk_and_stand.orientation_tolerance,
            }
        } else if let Some(ball) = &blackboard.ball
            && ball.position.x() <= 0.0
        {
            OrientationMode::LookAt {
                target: ground_to_field.inverse() * ball.position,
                tolerance: walk_and_stand.orientation_tolerance,
            }
        } else {
            OrientationMode::AlignWithPath
        };

        walk_to(
            blackboard,
            default_pose_in_ground,
            blocking_speed,
            orientation_mode,
            walk_and_stand.normal_distance_to_be_aligned,
            walk_and_stand.goalkeeper_hysteresis,
        )
    } else {
        Status::Failure
    }
}
pub fn walk_to_goalkeeper_penalty_position(blackboard: &mut Blackboard) -> Status {
    if let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field {
        let own_goal_line_x = -blackboard.field_dimensions.length / 2.0;
        let penalty_position_in_field = point!(own_goal_line_x, 0.0);
        let penalty_pose_in_field =
            Pose2::from_parts(penalty_position_in_field, Orientation2::new(0.0));
        let penalty_pose_in_ground = ground_to_field.inverse() * penalty_pose_in_field;
        let walk_and_stand = blackboard.parameters.walking.walk_and_stand;
        let blocking_speed = blackboard.parameters.walking.speed.blocking;

        walk_to(
            blackboard,
            penalty_pose_in_ground,
            blocking_speed,
            OrientationMode::AlignWithPath,
            walk_and_stand.normal_distance_to_be_aligned,
            walk_and_stand.goalkeeper_hysteresis,
        )
    } else {
        Status::Failure
    }
}

fn select_goalkeeper_kick_away_target(blackboard: &mut Blackboard) -> Status {
    let Some(transform) = blackboard.world_state.robot.ground_to_field else {
        return Status::Failure;
    };
    let Some((ball, _)) = kick_ball_state(blackboard, &transform) else {
        return Status::Failure;
    };
    let target = goalkeeper_kick_away_target(blackboard, transform * ball);
    apply_kick_target(blackboard, target)
}

fn goalkeeper_kick_away_target(blackboard: &Blackboard, ball: Point2<Field>) -> Point2<Field> {
    let target_y = if ball.y() >= 0.0 {
        blackboard.field_dimensions.width / 4.0
    } else {
        -blackboard.field_dimensions.width / 4.0
    };
    point!(0.0, target_y)
}

fn ball_interception_velocity_in_field(blackboard: &Blackboard) -> Option<Vector2<Field>> {
    let ball = blackboard.ball.as_ref()?;
    Some(ball.velocity)
}

fn is_interception_velocity_towards_own_half(
    blackboard: &Blackboard,
    ball_velocity: Vector2<Field>,
) -> bool {
    let parameters = &blackboard.parameters.ball.interception;

    ball_velocity.norm() > parameters.minimum_ball_velocity
        && ball_velocity.x() < -parameters.minimum_ball_velocity_towards_own_half
}

#[cfg(test)]
mod tests {
    use coordinate_systems::Ground;
    use linear_algebra::{Isometry2, vector};
    use types::{
        ball_position::BallPosition,
        field_dimensions::FieldDimensions,
        motion_command::BodyMotion,
        world_state::{BallSource, BallState},
    };

    use super::*;

    fn kick_away_blackboard(
        ball: Point2<Ground>,
        ball_in_field: Point2<Field>,
        outgoing_direction: Vector2<Ground>,
    ) -> Blackboard {
        let mut b = Blackboard::default();
        b.field_dimensions = FieldDimensions::SPL_2025;
        b.field_dimensions.ball_radius = 0.073;
        b.parameters
            .goalkeeper
            .kick_away_ball_maximum_robot_distance = 1.5;
        b.parameters.kicking.approach.maximum_heading = 0.8;
        b.parameters.kicking.approach.maximum_lateral_distance = 0.4;
        let target = goalkeeper_kick_away_target(&b, ball_in_field);
        let heading = Orientation2::from_vector(target - ball_in_field).angle()
            - Orientation2::from_vector(outgoing_direction).angle();
        let rotation: Isometry2<Ground, Field> = Isometry2::from_parts(Vector2::zeros(), heading);
        let transform =
            Isometry2::from_parts(ball_in_field.coords() - (rotation * ball).coords(), heading);
        b.world_state.robot.ground_to_field = Some(transform);
        b.world_state.ball = Some(BallState {
            ball_in_ground: ball,
            ball_in_field,
            source: BallSource::Own,
            ..Default::default()
        });
        b
    }

    #[test]
    fn urgent_clearance_rejects_the_wrong_side_trace_and_its_reflection() {
        for sign in [-1.0, 1.0] {
            let ball = point![0.170, -0.302 * sign];
            let mut b =
                kick_away_blackboard(ball, point![-3.5, 0.8 * sign], vector![0.298, 0.954 * sign]);
            let transform = b.world_state.robot.ground_to_field.unwrap();
            let target = transform.inverse() * goalkeeper_kick_away_target(&b, transform * ball);
            let direction = (target - ball).normalize();
            assert!(ball.x() > 0.0 && ball.coords().norm() < 1.5);
            assert!(ball.coords().dot(&direction) < 0.0);
            assert!(direction.y().atan2(direction.x()).abs() > 1.2);
            assert!(!is_goalkeeper_kick_away_needed(&mut b));
        }
    }

    #[test]
    fn aligned_close_clearance_remains_eligible_with_the_same_executed_target() {
        let ball = point![0.3, 0.0];
        let mut b = kick_away_blackboard(ball, point![-3.5, 0.8], vector![1.0, 0.0]);
        assert!(is_goalkeeper_kick_away_needed(&mut b));
        assert_eq!(kick(&mut b), Status::Success);
        assert_eq!(select_goalkeeper_kick_away_target(&mut b), Status::Success);
        let Some(BodyMotion::Kick {
            ball_position,
            kick_direction,
            ..
        }) = b.body_motion
        else {
            panic!("expected an aligned urgent clearance")
        };
        assert_eq!(ball_position, ball);
        assert!(kick_direction.angle().abs() < 1e-5);
    }

    #[test]
    fn forward_clearance_still_needs_lateral_ball_alignment() {
        let mut b = kick_away_blackboard(point![0.2, 0.6], point![-3.5, 0.8], vector![1.0, 0.0]);
        assert!(!is_goalkeeper_kick_away_needed(&mut b));
    }

    #[test]
    fn eligibility_uses_the_same_visual_refinement_and_moving_ball_pair_as_kick() {
        let mut b = kick_away_blackboard(point![0.35, -0.36], point![-3.5, 0.8], vector![1.0, 0.0]);
        let visual = point![0.35, -0.44];
        b.visual_kick_ball_position = Some(BallPosition {
            position: visual,
            velocity: Vector2::zeros(),
            last_seen: b.world_state.now,
        });
        // A valid slow-ball refinement moves the ball outside the lateral band.
        assert!(!is_goalkeeper_kick_away_needed(&mut b));
        kick(&mut b);
        assert!(
            matches!(b.body_motion, Some(BodyMotion::Kick { ball_position, .. }) if ball_position == visual)
        );

        // Raw visual positions do not replace a moving filter's coherent pair.
        b.world_state.ball.as_mut().unwrap().ball_in_ground_velocity = vector![1.0, 0.0];
        assert!(is_goalkeeper_kick_away_needed(&mut b));
        kick(&mut b);
        assert!(
            matches!(b.body_motion, Some(BodyMotion::Kick { ball_position, ball_velocity, .. })
            if ball_position == point![0.35,-0.36] && ball_velocity == vector![1.0,0.0])
        );
    }
}
