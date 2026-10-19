use std::time::Duration;

use coordinate_systems::{Field, Ground};
use linear_algebra::{Isometry2, Point2, Vector2, point};
use ros_z::time::Time;
use types::ball_position::BallPosition;

use crate::node::{Blackboard, ball_timeout};

// A raw visual percept does not estimate velocity on the robot. Refine a slow
// filtered ball only; a moving ball needs the filter's position and velocity.
const VISUAL_REFINEMENT_MAXIMUM_SPEED: f32 = 0.3;
const VISUAL_REFINEMENT_MAXIMUM_AGE: Duration = Duration::from_millis(150);
const MAXIMUM_PURSUIT_LOOKAHEAD: f32 = 0.5;
const MAXIMUM_PURSUIT_DISPLACEMENT: f32 = 0.75;

/// Return an observed ball state for motion, without pursuit extrapolation.
pub(crate) fn kick_ball_state(
    blackboard: &Blackboard,
    ground_to_field: &Isometry2<Ground, Field>,
) -> Option<(Point2<Ground>, Vector2<Ground>)> {
    let field_to_ground = ground_to_field.inverse();
    let filtered = blackboard
        .world_state
        .ball
        .map(|ball| (ball.ball_in_ground, ball.ball_in_ground_velocity))
        .or_else(|| {
            blackboard.ball.as_ref().and_then(|ball| {
                (blackboard.world_state.now >= ball.age
                    && blackboard.world_state.now.duration_since(ball.age)
                        <= ball_timeout(&blackboard.parameters, ball.source))
                .then_some((
                    field_to_ground * ball.position,
                    field_to_ground * ball.velocity,
                ))
            })
        })
        .filter(|(position, velocity)| finite_state(*position, *velocity));

    refine_observed_ball(
        filtered,
        blackboard.visual_kick_ball_position,
        blackboard.world_state.now,
        blackboard.field_dimensions.ball_radius * 2.0,
    )
}

fn finite_state(position: Point2<Ground>, velocity: Vector2<Ground>) -> bool {
    position
        .inner
        .coords
        .iter()
        .chain(velocity.inner.iter())
        .all(|value| value.is_finite())
}

fn refine_observed_ball(
    filtered: Option<(Point2<Ground>, Vector2<Ground>)>,
    visual: Option<BallPosition<Ground>>,
    now: Time,
    maximum_position_difference: f32,
) -> Option<(Point2<Ground>, Vector2<Ground>)> {
    let visual = visual.filter(|ball| {
        ball.age_at(now)
            .is_some_and(|age| age <= VISUAL_REFINEMENT_MAXIMUM_AGE)
            && finite_state(ball.position, ball.velocity)
    });
    match (filtered, visual) {
        (Some((position, velocity)), Some(visual))
            if velocity.norm() <= VISUAL_REFINEMENT_MAXIMUM_SPEED
                && visual.velocity.norm() <= VISUAL_REFINEMENT_MAXIMUM_SPEED
                && (position - visual.position).norm() <= maximum_position_difference =>
        {
            Some((visual.position, velocity))
        }
        (Some(filtered), _) => Some(filtered),
        (None, Some(visual)) => Some((visual.position, visual.velocity)),
        (None, None) => None,
    }
}

/// A bounded pursuit point for walking and striker election, never a policy input.
pub(crate) fn projected_ball_position(
    blackboard: &Blackboard,
    position: Point2<Field>,
    velocity: Vector2<Field>,
) -> Point2<Field> {
    let distance = blackboard
        .world_state
        .robot
        .ground_to_field
        .map_or(f32::INFINITY, |transform| {
            (position - transform.as_pose().position()).norm()
        });
    project_ball(
        position,
        velocity,
        distance,
        blackboard.parameters.ball.pursuit_lookahead,
        &blackboard.field_dimensions,
    )
}

fn project_ball(
    position: Point2<Field>,
    velocity: Vector2<Field>,
    distance: f32,
    lookahead: Duration,
    field: &types::field_dimensions::FieldDimensions,
) -> Point2<Field> {
    if position
        .inner
        .coords
        .iter()
        .chain(velocity.inner.iter())
        .any(|coordinate| !coordinate.is_finite())
        || distance <= 0.75
        || position.x().abs() >= field.length / 2.0
        || position.y().abs() >= field.width / 2.0
    {
        return position;
    }

    // Blend out the prediction before handing the measured ball to the policy.
    let distance_weight = ((distance - 0.75) / 0.75).clamp(0.0, 1.0);
    let mut displacement =
        velocity * (lookahead.as_secs_f32().min(MAXIMUM_PURSUIT_LOOKAHEAD) * distance_weight);
    let displacement_length = displacement.norm();
    if displacement_length > MAXIMUM_PURSUIT_DISPLACEMENT {
        displacement *= MAXIMUM_PURSUIT_DISPLACEMENT / displacement_length;
    }
    let projected = position + displacement;
    let x_limit = field.length / 2.0 - field.ball_radius;
    let y_limit = field.width / 2.0 - field.ball_radius;
    point![
        projected.x().clamp(-x_limit, x_limit),
        projected.y().clamp(-y_limit, y_limit)
    ]
}

#[cfg(test)]
mod tests {
    use linear_algebra::vector;
    use types::field_dimensions::FieldDimensions;

    use super::*;

    fn visual(
        position: Point2<Ground>,
        velocity: Vector2<Ground>,
        now: Time,
    ) -> BallPosition<Ground> {
        BallPosition {
            position,
            velocity,
            last_seen: now,
        }
    }

    #[test]
    fn moving_ball_keeps_filtered_position_and_velocity() {
        let now = Time::from_nanos(1_000_000_000);
        let measured = (point![0.7, 0.2], vector![-1.0, 0.8]);
        let state = refine_observed_ball(
            Some(measured),
            Some(visual(point![0.65, 0.15], Vector2::zeros(), now)),
            now,
            0.15,
        );
        assert_eq!(state, Some(measured));
    }

    #[test]
    fn slow_visual_refinement_requires_matching_recent_percept() {
        let now = Time::from_nanos(1_000_000_000);
        let measured = (point![0.7, 0.2], vector![0.1, 0.0]);
        let matching = visual(point![0.65, 0.15], Vector2::zeros(), now);
        assert_eq!(
            refine_observed_ball(Some(measured), Some(matching), now, 0.15),
            Some((matching.position, measured.1)),
        );
        for percept in [
            visual(point![0.3, -0.5], Vector2::zeros(), now),
            visual(matching.position, Vector2::zeros(), Time::zero()),
            visual(
                matching.position,
                Vector2::zeros(),
                now + Duration::from_millis(1),
            ),
        ] {
            assert_eq!(
                refine_observed_ball(Some(measured), Some(percept), now, 0.15),
                Some(measured),
            );
        }
    }

    #[test]
    fn pursuit_leads_rolling_ball_but_never_changes_close_ball() {
        let field = FieldDimensions::SPL_2025;
        let position = point![0.0, 0.0];
        let velocity = vector![0.0, 2.0];
        let horizon = Duration::from_millis(400);
        assert_eq!(
            project_ball(position, velocity, 0.6, horizon, &field),
            position
        );
        assert_eq!(
            project_ball(position, velocity, 2.0, horizon, &field),
            point![0.0, MAXIMUM_PURSUIT_DISPLACEMENT],
        );
        assert_eq!(
            project_ball(position, velocity, 2.0, Duration::ZERO, &field),
            position,
        );
    }

    #[test]
    fn pursuit_stays_inside_touchline_and_preserves_restart_ball() {
        let field = FieldDimensions::SPL_2025;
        let horizon = Duration::from_secs(2);
        let projected = project_ball(point![4.4, 2.8], vector![3.0, 3.0], 3.0, horizon, &field);
        assert!(projected.x() <= field.length / 2.0 - field.ball_radius);
        assert!(projected.y() <= field.width / 2.0 - field.ball_radius);
        let restart = point![field.length / 2.0, field.width / 2.0];
        assert_eq!(
            project_ball(restart, vector![1.0, 1.0], 3.0, horizon, &field),
            restart
        );
    }
}
