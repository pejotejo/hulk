use coordinate_systems::{Field, Ground};
use geometry::{
    circle::Circle, line_segment::LineSegment, rectangle::Rectangle, visibility::visible_segments,
};
use linear_algebra::{Isometry2, Point2, point, vector};
use types::{
    field_dimensions::{FieldDimensions, Half, Side},
    obstacles::{Obstacle, ObstacleKind},
    parameters::PassingParameters,
};

/// Geometrically open opponent-goal segments for the ball center, in field coordinates.
/// `clearance` is an additional nonnegative physical margin in meters, not a risk score.
/// Boundaries may be tangent to inflated obstacles; execution must choose an interior target.
pub fn goal_openings(
    ball: Point2<Field>,
    ground_to_field: Isometry2<Ground, Field>,
    obstacles: &[Obstacle],
    field: &FieldDimensions,
    clearance: f32,
) -> Vec<LineSegment<Field>> {
    let margin = field.ball_radius + clearance;
    let goal_x = field.length / 2.0;
    let half_width = field.goal_inner_width / 2.0 - margin;
    if ball.x() >= goal_x || half_width <= 0.0 {
        return Vec::new();
    }
    let circles = obstacle_circles(ground_to_field, obstacles, field, clearance);
    // Follow each goal ray until the ball has passed the inflated posts completely.
    let checked_x = goal_x + field.goal_post_diameter - field.line_width / 2.0 + margin;
    let scale = (checked_x - ball.x()) / (goal_x - ball.x());
    let goal = LineSegment(point![goal_x, -half_width], point![goal_x, half_width]);
    visible_segments(
        ball,
        LineSegment(
            ball + (goal.0 - ball) * scale,
            ball + (goal.1 - ball) * scale,
        ),
        &circles,
    )
    .into_iter()
    .map(|opening| {
        LineSegment(
            point![goal_x, ball.y() + (opening.0.y() - ball.y()) / scale],
            point![goal_x, ball.y() + (opening.1.y() - ball.y()) / scale],
        )
    })
    .collect()
}

/// Safe in-field corridors covering at least `travel_distance` from the ball.
pub fn travel_openings(
    ball: Point2<Field>,
    ground_to_field: Isometry2<Ground, Field>,
    obstacles: &[Obstacle],
    field: &FieldDimensions,
    travel_distance: f32,
    clearance: f32,
) -> Vec<LineSegment<Field>> {
    let margin = field.ball_radius + clearance;
    let half_length = field.length / 2.0 - margin;
    let half_width = field.width / 2.0 - margin;
    if !travel_distance.is_finite()
        || travel_distance <= 0.0
        || half_length <= 0.0
        || half_width <= 0.0
    {
        return Vec::new();
    }
    let bounds = Rectangle {
        min: point![-half_length, -half_width],
        max: point![half_length, half_width],
    };
    let left = ball.x() - travel_distance;
    let right = ball.x() + travel_distance;
    let bottom = ball.y() - travel_distance;
    let top = ball.y() + travel_distance;
    let min_x = left.max(-half_length);
    let max_x = right.min(half_length);
    let min_y = bottom.max(-half_width);
    let max_y = top.min(half_width);
    let circles = obstacle_circles(ground_to_field, obstacles, field, clearance);
    // shortcut: square corridors check up to sqrt(2) times the bound; use arcs if too restrictive.
    [
        LineSegment(point![right, min_y], point![right, max_y]),
        LineSegment(point![min_x, top], point![max_x, top]),
        LineSegment(point![left, min_y], point![left, max_y]),
        LineSegment(point![min_x, bottom], point![max_x, bottom]),
    ]
    .into_iter()
    .filter(|edge| bounds.contains(edge.0) && bounds.contains(edge.1))
    .flat_map(|edge| visible_segments(ball, edge, &circles))
    .collect()
}

/// Visible reception regions on the ball-facing side of the receiver's current pose.
/// Keep every obstacle, including the receiver, in the finite corridor check.
pub fn pass_openings(
    ball: Point2<Field>,
    receiver: Point2<Field>,
    ground_to_field: Isometry2<Ground, Field>,
    obstacles: &[Obstacle],
    field: &FieldDimensions,
    parameters: &PassingParameters,
    clearance: f32,
) -> Vec<LineSegment<Field>> {
    let delta = receiver - ball;
    let distance = delta.norm();
    let travel = distance - parameters.receiver_standoff;
    if parameters.validate().is_err()
        || !distance.is_finite()
        || travel <= f32::EPSILON
        || travel >= parameters.maximum_distance
    {
        return Vec::new();
    }
    let direction = delta / distance;
    let center = ball + direction * travel;
    let lateral = vector![-direction.y(), direction.x()];
    let half_width = parameters
        .target_half_width
        .min((parameters.maximum_distance.powi(2) - travel.powi(2)).sqrt());
    let mut lower = -half_width;
    let mut upper = half_width;
    let margin = field.ball_radius + clearance;
    // Clip the region to field bounds; receiver eligibility owns the progress policy.
    for (coordinate, axis, minimum, maximum) in [
        (
            center.x(),
            lateral.x(),
            -field.length / 2.0 + margin,
            field.length / 2.0 - margin,
        ),
        (
            center.y(),
            lateral.y(),
            -field.width / 2.0 + margin,
            field.width / 2.0 - margin,
        ),
    ] {
        if minimum > maximum {
            return Vec::new();
        }
        if axis.abs() <= f32::EPSILON {
            if !(minimum..=maximum).contains(&coordinate) {
                return Vec::new();
            }
        } else {
            let first = (minimum - coordinate) / axis;
            let second = (maximum - coordinate) / axis;
            lower = lower.max(first.min(second));
            upper = upper.min(first.max(second));
        }
    }
    if lower >= upper {
        return Vec::new();
    }
    visible_segments(
        ball,
        LineSegment(center + lateral * lower, center + lateral * upper),
        &obstacle_circles(ground_to_field, obstacles, field, clearance),
    )
}

fn obstacle_circles(
    ground_to_field: Isometry2<Ground, Field>,
    obstacles: &[Obstacle],
    field: &FieldDimensions,
    clearance: f32,
) -> Vec<Circle<Field>> {
    let margin = field.ball_radius + clearance;
    let post_radius = field.goal_post_diameter / 2.0;
    obstacles
        .iter()
        .filter(|obstacle| obstacle.kind != ObstacleKind::Ball)
        .map(|obstacle| {
            Circle::new(
                ground_to_field * obstacle.position,
                obstacle.radius_at_foot_height + margin,
            )
        })
        .chain([Half::Own, Half::Opponent].into_iter().flat_map(|half| {
            [Side::Left, Side::Right].map(move |side| {
                let post = field.goal_post(half, side);
                Circle::new(
                    // Match the obstacle filter's placement behind the goal line.
                    post + vector![
                        post.x().signum() * (post_radius - field.line_width / 2.0),
                        0.0
                    ],
                    post_radius + margin,
                )
            })
        }))
        .collect()
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use geometry::Distance;
    use linear_algebra::IntoTransform;

    use super::*;

    #[test]
    fn pass_regions_respect_field_bounds_and_range_in_any_direction() {
        let field = FieldDimensions::SPL_2025;
        let parameters = PassingParameters {
            maximum_distance: 3.0,
            minimum_progress: 0.5,
            receiver_standoff: 0.5,
            target_half_width: 0.2,
            ..Default::default()
        };
        for (ball, receiver) in [
            (point![0.0, 2.89], point![2.0, 2.99]),
            (point![0.0, 0.0], point![3.499, 0.0]),
        ] {
            let openings = pass_openings(
                ball,
                receiver,
                Default::default(),
                &[],
                &field,
                &parameters,
                0.05,
            );
            assert_eq!(openings.len(), 1);
            assert!(openings[0].length() < 2.0 * parameters.target_half_width);
            for target in [openings[0].0, openings[0].center(), openings[0].1] {
                assert!((target - ball).norm() <= parameters.maximum_distance + 1e-6);
                assert!(target.x().abs() <= field.length / 2.0 - 0.1 + 1e-6);
                assert!(target.y().abs() <= field.width / 2.0 - 0.1 + 1e-6);
            }
        }
        for receiver in [point![-2.0, 0.0], point![0.0, -2.0]] {
            assert!(
                !pass_openings(
                    point![0.0, 0.0],
                    receiver,
                    Default::default(),
                    &[],
                    &field,
                    &parameters,
                    0.05
                )
                .is_empty()
            );
        }
    }

    #[test]
    fn goal_openings_transform_obstacles_and_include_ball_clearance() {
        let field = FieldDimensions::SPL_2025;
        let ball = point![0.0, 0.0];
        let transform: Isometry2<Ground, Field> =
            nalgebra::Isometry2::new(nalgebra::vector![1.0, -0.5], 0.7).framed_transform();
        let openings = goal_openings(ball, transform, &[], &field, 0.1);
        assert_eq!(openings.len(), 1);
        assert_relative_eq!(openings[0].0.y(), -openings[0].1.y());
        assert!(openings[0].1.y() < 0.6);
        assert_relative_eq!(
            LineSegment(ball, ball + (openings[0].1 - ball) * 2.0).distance_to(point![4.525, 0.8]),
            0.2,
            epsilon = 1e-6,
        );

        let obstacle = Obstacle::robot(transform.inverse() * point![3.0, 0.0], 0.2, 0.4);
        let openings = goal_openings(ball, transform, &[obstacle], &field, 0.1);
        assert_eq!(openings.len(), 2);
        let shadow_half_width = 4.5 * 0.35 / (9.0_f32 - 0.35_f32.powi(2)).sqrt();
        assert_relative_eq!(openings[0].1.y(), -shadow_half_width, epsilon = 1e-6);
        assert_relative_eq!(openings[1].0.y(), shadow_half_width, epsilon = 1e-6);
        let ball_obstacle = Obstacle::ball(transform.inverse() * ball, field.ball_radius);
        assert_eq!(
            goal_openings(ball, transform, &[ball_obstacle], &field, 0.1).len(),
            1
        );
        assert!(
            goal_openings(point![field.length / 2.0, 0.0], transform, &[], &field, 0.1).is_empty()
        );
        assert!(goal_openings(ball, transform, &[], &field, 1.0).is_empty());
    }

    #[test]
    fn shots_clear_posts_after_crossing_goal_line() {
        let field = FieldDimensions::SPL_2025;
        let ball = point![4.4, 0.5];
        let old_target = point![4.5, -0.59];
        let post = Circle::new(point![4.525, -0.8], 0.15);
        assert!(!post.intersects_line_segment(&LineSegment(ball, old_target)));
        assert!(post.intersects_line_segment(&LineSegment(ball, ball + (old_target - ball) * 2.0)));

        let openings = goal_openings(ball, Default::default(), &[], &field, 0.05);
        assert!(!openings.is_empty());
        for opening in openings {
            assert!(!(0.0..=1.0).contains(&opening.projection_factor(old_target)));
            for fraction in [0.01, 0.5, 0.99] {
                let target = opening.0 + (opening.1 - opening.0) * fraction;
                let beyond_goal = ball + (target - ball) * 2.0;
                for y in [-0.8, 0.8] {
                    assert!(
                        !Circle::new(point![4.525, y], 0.15)
                            .intersects_line_segment(&LineSegment(ball, beyond_goal))
                    );
                }
            }
        }
    }
}
