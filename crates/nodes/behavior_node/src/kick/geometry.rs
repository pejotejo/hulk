use coordinate_systems::{Field, Ground};
use geometry::{Distance, circle::Circle, line::Line2, line_segment::LineSegment};
use linear_algebra::{Isometry2, Point2, point, vector};
use types::{
    field_dimensions::{FieldDimensions, Half, Side},
    obstacles::{Obstacle, ObstacleKind},
    parameters::PassingParameters,
};

/// Geometrically open opponent-goal segments for the ball center, in field coordinates.
/// `clearance` is an additional nonnegative physical margin in meters, not a risk score.
/// Boundaries may be tangent to inflated obstacles; execution must choose an interior target.
/// Initial Robot proxy overlap permits non-inward ball separation; physical
/// post collisions remain excluded. Robot proxies have no team identity here.
/// Teammate/body access and walking clearance are checked separately by behavior.
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
        clearance,
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

/// Visible finite target regions inside the field and a square range window.
/// Callers also bound the selected target's Euclidean distance to this maximum.
pub fn travel_openings(
    ball: Point2<Field>,
    ground_to_field: Isometry2<Ground, Field>,
    obstacles: &[Obstacle],
    field: &FieldDimensions,
    maximum_distance: f32,
    clearance: f32,
) -> Vec<LineSegment<Field>> {
    let margin = field.ball_radius + clearance;
    let half_length = field.length / 2.0 - margin;
    let half_width = field.width / 2.0 - margin;
    if !maximum_distance.is_finite()
        || maximum_distance <= 0.0
        || !clearance.is_finite()
        || clearance < 0.0
        || !ball
            .inner
            .coords
            .iter()
            .all(|coordinate| coordinate.is_finite())
        || !half_length.is_finite()
        || !half_width.is_finite()
        || half_length <= 0.0
        || half_width <= 0.0
    {
        return Vec::new();
    }
    let min_x = (ball.x() - maximum_distance).max(-half_length);
    let max_x = (ball.x() + maximum_distance).min(half_length);
    let min_y = (ball.y() - maximum_distance).max(-half_width);
    let max_y = (ball.y() + maximum_distance).min(half_width);
    if min_x >= max_x || min_y >= max_y {
        return Vec::new();
    }
    let circles = obstacle_circles(ground_to_field, obstacles, field, clearance);
    // Intersect the range window with the field before taking its perimeter.
    // A maximum range beyond a field edge still has useful in-field targets.
    [
        LineSegment(point![max_x, min_y], point![max_x, max_y]),
        LineSegment(point![min_x, max_y], point![max_x, max_y]),
        LineSegment(point![min_x, min_y], point![min_x, max_y]),
        LineSegment(point![min_x, min_y], point![max_x, min_y]),
    ]
    .into_iter()
    .flat_map(|edge| visible_segments(ball, edge, &circles, clearance))
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
    pass_openings_with_progress(
        ball,
        receiver,
        ground_to_field,
        obstacles,
        field,
        parameters,
        clearance,
        parameters.minimum_progress,
    )
}

/// Restarts may deliberately use a backwards cutback while retaining the same
/// validated physical reception region and maximum pass range.
pub(super) fn pass_openings_with_progress(
    ball: Point2<Field>,
    receiver: Point2<Field>,
    ground_to_field: Isometry2<Ground, Field>,
    obstacles: &[Obstacle],
    field: &FieldDimensions,
    parameters: &PassingParameters,
    clearance: f32,
    minimum_progress: f32,
) -> Vec<LineSegment<Field>> {
    let delta = receiver - ball;
    let distance = delta.norm();
    let travel = distance - parameters.receiver_standoff;
    if parameters.validate().is_err()
        || !minimum_progress.is_finite()
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
    // Clip the region to field bounds and required forward progress, preserving its line.
    for (coordinate, axis, minimum, maximum) in [
        (
            center.x(),
            lateral.x(),
            (-field.length / 2.0 + margin).max(ball.x() + minimum_progress),
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
        clearance,
    )
}

#[derive(Clone, Copy, Debug)]
struct BallObstacle {
    circle: Circle<Field>,
    kind: ObstacleKind,
}

fn obstacle_circles(
    ground_to_field: Isometry2<Ground, Field>,
    obstacles: &[Obstacle],
    field: &FieldDimensions,
    clearance: f32,
) -> Vec<BallObstacle> {
    let margin = field.ball_radius + clearance;
    let post_radius = field.goal_post_diameter / 2.0;
    obstacles
        .iter()
        // GoalPost obstacles carry a navigation clearance radius. Use the physical
        // posts from the field model below, once, when checking ball trajectories.
        .filter(|obstacle| !matches!(obstacle.kind, ObstacleKind::Ball | ObstacleKind::GoalPost))
        .map(|obstacle| BallObstacle {
            circle: Circle::new(
                ground_to_field * obstacle.position,
                obstacle.radius_at_foot_height + margin,
            ),
            kind: obstacle.kind,
        })
        .chain([Half::Own, Half::Opponent].into_iter().flat_map(|half| {
            [Side::Left, Side::Right].map(move |side| {
                let post = field.goal_post(half, side);
                BallObstacle {
                    circle: Circle::new(
                        // Match the obstacle filter's placement behind the goal line.
                        post + vector![
                            post.x().signum() * (post_radius - field.line_width / 2.0),
                            0.0
                        ],
                        post_radius + margin,
                    ),
                    kind: ObstacleKind::GoalPost,
                }
            })
        }))
        .collect()
}

/// Clip a finite target region against circular shadows. Target-circle
/// intersections matter as well as tangents: an obstacle beyond a receiver must
/// not cast an infinite shadow over an otherwise clear pass.
/// An initially overlapping Robot proxy permits only rays that never decrease
/// separation. Other obstacle kinds retain their physical exclusion, including
/// initial overlap. This geometry alone does not identify teammates or contacts.
fn visible_segments(
    origin: Point2<Field>,
    target: LineSegment<Field>,
    obstacles: &[BallObstacle],
    clearance: f32,
) -> Vec<LineSegment<Field>> {
    let finite_point = |point: Point2<Field>| {
        point
            .inner
            .coords
            .iter()
            .all(|coordinate| coordinate.is_finite())
    };
    if !finite_point(origin)
        || !finite_point(target.0)
        || !finite_point(target.1)
        || !clearance.is_finite()
        || clearance < 0.0
        || obstacles.iter().any(|obstacle| {
            !finite_point(obstacle.circle.center)
                || !obstacle.circle.radius.is_finite()
                || obstacle.circle.radius < 0.0
        })
    {
        return Vec::new();
    }
    let line = Line2::from(target);
    if target.length_squared() <= f32::EPSILON || line.distance_to(origin) <= f32::EPSILON {
        return Vec::new();
    }
    let mut boundaries = vec![0.0, 1.0];
    for obstacle in obstacles {
        let circle = obstacle.circle;
        let away = origin - circle.center;
        if away.norm_squared() <= circle.radius.powi(2) {
            let coincident_robot =
                obstacle.kind == ObstacleKind::Robot && away.try_normalize(f32::EPSILON).is_none();
            let solid_overlap = obstacle.kind != ObstacleKind::Robot
                && away.norm_squared() <= (circle.radius - clearance).max(0.0).powi(2);
            if coincident_robot || solid_overlap {
                // Coincident proxy centers have no defined outward sector.
                // Static obstacle kinds remain physically solid.
                return Vec::new();
            }
            // A Robot disk is an occupancy proxy, not articulated foot geometry.
            // Permit an initial overlap only in the half-plane that cannot move
            // deeper into it; preserve every other obstacle's normal shadow.
            let target_direction = target.1 - target.0;
            boundaries.push((origin - target.0).dot(&away) / target_direction.dot(&away));
            continue;
        }
        let Some(tangents) = circle.tangents_with_point(origin) else {
            return Vec::new();
        };
        for tangent in [tangents.0, tangents.1] {
            boundaries.push(target.projection_factor(line.intersection(&Line2::from(tangent))));
        }
        let center = target.projection_factor(circle.center);
        let closest = line.closest_point(circle.center);
        let half_chord_squared = circle.radius.powi(2) - (closest - circle.center).norm_squared();
        if half_chord_squared >= 0.0 {
            let offset = (half_chord_squared / target.length_squared()).sqrt();
            boundaries.extend([center - offset, center + offset]);
        }
    }
    boundaries.retain(|value| (0.0..=1.0).contains(value));
    boundaries.sort_unstable_by(f32::total_cmp);
    boundaries.dedup();
    let point_at = |factor| target.0 + (target.1 - target.0) * factor;
    let is_clear = |point| {
        let path = LineSegment(origin, point);
        obstacles.iter().all(|obstacle| {
            let circle = obstacle.circle;
            let away = origin - circle.center;
            if away.norm_squared() <= circle.radius.powi(2) {
                let physical = Circle::new(circle.center, (circle.radius - clearance).max(0.0));
                (point - origin).dot(&away) >= 0.0
                    && (obstacle.kind == ObstacleKind::Robot
                        || !physical.intersects_line_segment(&path))
            } else {
                !circle.intersects_line_segment(&path)
            }
        })
    };
    let mut openings: Vec<LineSegment<Field>> = Vec::new();
    for pair in boundaries.windows(2) {
        if !is_clear(point_at((pair[0] + pair[1]) / 2.0)) {
            continue;
        }
        let start = point_at(pair[0]);
        let end = point_at(pair[1]);
        if let Some(previous) = openings.last_mut()
            && (previous.1 - start).norm_squared() == 0.0
            && is_clear(start)
        {
            previous.1 = end;
        } else {
            openings.push(LineSegment(start, end));
        }
    }
    openings
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use geometry::Distance;
    use linear_algebra::IntoTransform;

    use super::*;

    fn solid(circle: Circle<Field>) -> BallObstacle {
        BallObstacle {
            circle,
            kind: ObstacleKind::Unknown,
        }
    }

    fn coarse_robot(center: Point2<Field>) -> BallObstacle {
        BallObstacle {
            circle: Circle::new(center, 0.2 + 0.073 + 0.08),
            kind: ObstacleKind::Robot,
        }
    }

    #[test]
    fn travel_range_windows_keep_their_field_clipped_perimeters() {
        for field in [
            FieldDimensions {
                ball_radius: 0.073,
                ..FieldDimensions::SPL_2025
            },
            FieldDimensions {
                length: 14.0,
                width: 9.0,
                ball_radius: 0.105,
                ..FieldDimensions::SPL_2025
            },
        ] {
            for ball in [
                Point2::origin(),
                point![-field.length / 4.0, field.width / 4.0],
                point![field.length / 2.0, field.width / 2.0],
            ] {
                let clearance = 0.08;
                let openings =
                    travel_openings(ball, Default::default(), &[], &field, 6.0, clearance);
                assert!(
                    !openings.is_empty(),
                    "field-clipped targets must remain available at {ball:?}"
                );
                for opening in openings {
                    assert!(opening.length() > f32::EPSILON);
                    for target in [opening.0, opening.center(), opening.1] {
                        assert!(
                            target
                                .inner
                                .coords
                                .iter()
                                .all(|coordinate| coordinate.is_finite())
                        );
                        assert!(
                            target.x().abs()
                                <= field.length / 2.0 - field.ball_radius - clearance + 1e-6
                        );
                        assert!(
                            target.y().abs()
                                <= field.width / 2.0 - field.ball_radius - clearance + 1e-6
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn clipped_travel_windows_still_reject_blocked_or_degenerate_regions() {
        let field = FieldDimensions {
            ball_radius: 0.073,
            ..FieldDimensions::SPL_2025
        };
        let robot = Obstacle::robot(Point2::origin(), 0.2, 0.25);
        assert!(
            travel_openings(
                Point2::origin(),
                Default::default(),
                &[robot],
                &field,
                6.0,
                0.08
            )
            .is_empty()
        );
        for (ball, range, clearance) in [
            (Point2::origin(), 0.0, 0.08),
            (Point2::origin(), f32::NAN, 0.08),
            (point![f32::NAN, 0.0], 6.0, 0.08),
            (Point2::origin(), 6.0, f32::NAN),
            (
                point![field.length / 2.0, field.width / 2.0],
                field.ball_radius + 0.08,
                0.08,
            ),
        ] {
            assert!(
                travel_openings(ball, Default::default(), &[], &field, range, clearance).is_empty()
            );
        }

        let robot = Obstacle::robot(point![1.0, 0.0], 0.2, 0.25);
        let openings = travel_openings(
            Point2::origin(),
            Default::default(),
            &[robot],
            &field,
            6.0,
            0.08,
        );
        assert!(!openings.is_empty());
        for opening in openings {
            for fraction in [0.01, 0.5, 0.99] {
                let target = opening.0 + (opening.1 - opening.0) * fraction;
                assert!(
                    LineSegment(Point2::origin(), target)
                        .distance_to(point![robot.position.x(), robot.position.y()])
                        >= robot.radius_at_foot_height + field.ball_radius + 0.08 - 1e-5
                );
            }
        }
    }

    #[test]
    fn pass_regions_respect_field_bounds_range_and_progress() {
        let field = FieldDimensions::SPL_2025;
        let mut parameters = PassingParameters {
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
                assert!(target.x() - ball.x() >= parameters.minimum_progress);
                assert!((target - ball).norm() <= parameters.maximum_distance + 1e-6);
                assert!(target.x().abs() <= field.length / 2.0 - 0.1 + 1e-6);
                assert!(target.y().abs() <= field.width / 2.0 - 0.1 + 1e-6);
            }
        }
        parameters.minimum_progress = 5.0;
        assert!(
            pass_openings(
                point![0.0, 0.0],
                point![2.0, 0.0],
                Default::default(),
                &[],
                &field,
                &parameters,
                0.05
            )
            .is_empty()
        );
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

    #[test]
    fn padded_navigation_posts_do_not_close_physical_goal_openings() {
        let field = FieldDimensions::SPL_2025;
        let ball = point![2.5, 0.0];
        let expected = goal_openings(ball, Default::default(), &[], &field, 0.05);
        let padded_posts = [
            Obstacle::goal_post(point![4.525, -0.8], 0.6),
            Obstacle::goal_post(point![4.525, 0.8], 0.6),
        ];
        let actual = goal_openings(ball, Default::default(), &padded_posts, &field, 0.05);
        assert_eq!(actual, expected);
    }

    #[test]
    fn finite_corridors_match_segment_collision_checks() {
        let origin = point![0.0, 0.0];
        let target = LineSegment(point![4.0, -2.0], point![4.0, 2.0]);
        for obstacles in [
            vec![],
            vec![Circle::new(point![2.0, 0.0], 0.5)],
            vec![Circle::new(point![5.0, 0.0], 0.5)],
            vec![Circle::new(point![4.2, 0.0], 0.5)],
            vec![Circle::new(point![4.0, 0.0], 0.5)],
            vec![Circle::new(point![0.0, 0.0], 0.5)],
            vec![
                Circle::new(point![2.0, -0.3], 0.5),
                Circle::new(point![2.0, 0.3], 0.5),
            ],
        ] {
            let modeled: Vec<_> = obstacles.iter().copied().map(solid).collect();
            for target in [target, target.flip()] {
                let openings = visible_segments(origin, target, &modeled, 0.0);
                for step in 0..100 {
                    let point = target.0 + (target.1 - target.0) * ((step as f32 + 0.5) / 100.0);
                    let expected = obstacles.iter().all(|obstacle| {
                        !obstacle.intersects_line_segment(&LineSegment(origin, point))
                    });
                    let visible = openings
                        .iter()
                        .any(|opening| (0.0..1.0).contains(&opening.projection_factor(point)));
                    assert_eq!(visible, expected, "{point:?}, {obstacles:?}");
                }
            }
        }
    }

    #[test]
    fn margin_overlap_allows_only_monotonic_escape_outside_physical_obstacles() {
        let origin = point![0.0, 0.0];
        let clearance = 0.08;
        let margin_overlap = solid(Circle::new(point![-0.3, 0.0], 0.33));
        let forward = LineSegment(point![2.0, -1.0], point![2.0, 1.0]);
        assert_eq!(
            visible_segments(origin, forward, &[margin_overlap], clearance),
            vec![forward],
        );
        let backward = LineSegment(point![-2.0, -1.0], point![-2.0, 1.0]);
        assert!(visible_segments(origin, backward, &[margin_overlap], clearance).is_empty());
        let sideways = LineSegment(point![-1.0, 2.0], point![1.0, 2.0]);
        let openings = visible_segments(origin, sideways, &[margin_overlap], clearance);
        assert_eq!(openings.len(), 1);
        assert_relative_eq!(openings[0].0.x(), 0.0);
        assert_relative_eq!(openings[0].1.x(), 1.0);
        let physical = Circle::new(margin_overlap.circle.center, 0.25);
        for fraction in [0.01, 0.5, 0.99] {
            let point = openings[0].0 + (openings[0].1 - openings[0].0) * fraction;
            assert!((point - origin).dot(&(origin - physical.center)) >= 0.0);
            assert!(!physical.intersects_line_segment(&LineSegment(origin, point)));
        }
        for center_x in [-0.2, -0.25] {
            let physical_overlap = solid(Circle::new(point![center_x, 0.0], 0.33));
            assert!(visible_segments(origin, forward, &[physical_overlap], clearance).is_empty());
        }
    }

    #[test]
    fn a_robot_proxy_behind_the_ball_does_not_prevent_outward_shooting() {
        let field = FieldDimensions::SPL_2025;
        let ball = point![2.0, 0.0];
        let behind = Obstacle::robot(point![1.7, 0.0], 0.2, 0.2);
        let expected = goal_openings(ball, Default::default(), &[], &field, 0.08);
        let actual = goal_openings(ball, Default::default(), &[behind], &field, 0.08);
        assert!(!actual.is_empty());
        assert_eq!(actual, expected);
        let overlapping = Obstacle::robot(point![1.85, 0.0], 0.2, 0.2);
        assert_eq!(
            goal_openings(ball, Default::default(), &[overlapping], &field, 0.08),
            expected
        );
    }

    #[test]
    fn initial_robot_proxy_overlap_allows_rotated_and_reflected_non_inward_rays() {
        for angle in [0.0, 0.7, -1.8] {
            let transform: Isometry2<Field, Field> =
                Isometry2::from_parts(vector![1.2, -0.7], angle);
            for reflection in [-1.0, 1.0] {
                let transform_point =
                    |point: Point2<Field>| transform * point![point.x(), reflection * point.y()];
                let origin = transform_point(point![0.0, 0.0]);
                // Sweep through the raw 0.2 m proxy radius as well as its
                // ball/clearance inflation, without changing the allowed sector.
                for distance in [0.02, 0.1, 0.16263655, 0.199, 0.2, 0.272, 0.34] {
                    let robot = coarse_robot(transform_point(point![-distance, 0.0]));
                    let outward = LineSegment(
                        transform_point(point![2.0, -1.0]),
                        transform_point(point![2.0, 1.0]),
                    );
                    let openings = visible_segments(origin, outward, &[robot], 0.08);
                    assert_eq!(openings.len(), 1);
                    for fraction in [0.01, 0.5, 0.99] {
                        let target = openings[0].0 + (openings[0].1 - openings[0].0) * fraction;
                        let ray = target - origin;
                        let initial_distance = (origin - robot.circle.center).norm_squared();
                        assert!(ray.dot(&(origin - robot.circle.center)) > 0.0);
                        assert!(
                            (origin + ray * 0.01 - robot.circle.center).norm_squared()
                                > initial_distance
                        );
                        assert!((target - robot.circle.center).norm_squared() > initial_distance);
                    }
                    let inward = LineSegment(
                        transform_point(point![-2.0, -1.0]),
                        transform_point(point![-2.0, 1.0]),
                    );
                    assert!(visible_segments(origin, inward, &[robot], 0.08).is_empty());
                }
            }
        }
    }

    #[test]
    fn overlapping_robot_proxies_require_every_non_inward_sector() {
        let origin = point![0.0, 0.0];
        let target = LineSegment(point![2.0, -2.0], point![2.0, 2.0]);
        let robots = [
            coarse_robot(point![-0.15, 0.0]),
            coarse_robot(point![0.0, -0.1]),
        ];
        let openings = visible_segments(origin, target, &robots, 0.08);
        assert_eq!(openings.len(), 1);
        assert_relative_eq!(openings[0].0.y(), 0.0);
        assert_relative_eq!(openings[0].1.y(), 2.0);
        for fraction in [0.01, 0.5, 0.99] {
            let ray = openings[0].0 + (openings[0].1 - openings[0].0) * fraction - origin;
            assert!(
                robots
                    .iter()
                    .all(|robot| ray.dot(&(origin - robot.circle.center)) >= 0.0)
            );
        }
        let opposed = [
            coarse_robot(point![-0.1, 0.0]),
            coarse_robot(point![0.1, 0.0]),
        ];
        assert!(visible_segments(origin, target, &opposed, 0.08).is_empty());
    }

    #[test]
    fn coincident_and_nonfinite_robot_proxy_inputs_do_not_create_an_escape() {
        let origin = Point2::origin();
        let target = LineSegment(point![2.0, -1.0], point![2.0, 1.0]);
        for obstacle in [
            coarse_robot(origin),
            coarse_robot(point![f32::NAN, 0.0]),
            BallObstacle {
                circle: Circle::new(point![-0.1, 0.0], f32::NAN),
                kind: ObstacleKind::Robot,
            },
        ] {
            assert!(visible_segments(origin, target, &[obstacle], 0.08).is_empty());
        }
        let robot = coarse_robot(point![-0.1, 0.0]);
        for (origin, target) in [
            (origin, LineSegment(point![2.0, 1.0], point![2.0, 1.0])),
            (origin, LineSegment(point![-1.0, 0.0], point![1.0, 0.0])),
            (point![f32::NAN, 0.0], target),
            (origin, LineSegment(point![2.0, f32::NAN], target.1)),
        ] {
            assert!(visible_segments(origin, target, &[robot], 0.08).is_empty());
        }
    }

    #[test]
    fn small_match_proxy_overlap_retains_an_otherwise_clear_forward_outward_ray() {
        // Exact H4 local observations at 17 s in the V12 Small opening.
        // These are occupancy proxies, not evidence of physical foot contact.
        let field = FieldDimensions {
            ball_radius: 0.073,
            goal_inner_width: 1.8,
            ..FieldDimensions::SPL_2025
        };
        let transform = Isometry2::from_parts(
            vector![-2.977047, -0.9992117],
            (-0.99991053_f32).atan2(-0.013377192),
        );
        let ball = point![-2.9748948, -1.4415473];
        let positions = [
            point![-0.64630365, -0.9131341],
            point![0.68222, 1.828498],
            point![-0.98111784, 1.260079],
            point![-0.9796262, 6.756959],
            point![-1.7528381, 0.1879983],
            point![0.33525014, 0.1305356],
            point![-0.48612273, 4.048561],
        ];
        let obstacles: Vec<_> = positions
            .into_iter()
            .map(|position| Obstacle::robot(position, 0.2, 0.25))
            .collect();
        let contested_center = transform * positions[5];
        assert!((ball - contested_center).norm() < 0.2);
        let target = point![-2.34, -2.847];
        assert!((target - ball).dot(&(ball - contested_center)) > 0.0);
        let openings = travel_openings(ball, transform, &obstacles, &field, 6.0, 0.08);
        assert!(openings.iter().any(|opening| {
            (0.0..=1.0).contains(&opening.projection_factor(target))
                && opening.distance_to(target) < 1e-5
        }));
        let modeled = obstacle_circles(transform, &obstacles, &field, 0.08);
        for opening in openings {
            for fraction in [0.01, 0.5, 0.99] {
                let target = opening.0 + (opening.1 - opening.0) * fraction;
                let ray = target - ball;
                for obstacle in &modeled {
                    let away = ball - obstacle.circle.center;
                    if away.norm_squared() <= obstacle.circle.radius.powi(2) {
                        assert_eq!(obstacle.kind, ObstacleKind::Robot);
                        assert!(ray.dot(&away) >= -1e-5);
                    } else {
                        assert!(
                            LineSegment(ball, target).distance_to(obstacle.circle.center)
                                >= obstacle.circle.radius - 1e-5
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn robot_egress_does_not_relax_other_kinds_or_obstacles_ahead() {
        let origin = point![0.0, 0.0];
        let target = LineSegment(point![2.0, -0.5], point![2.0, 0.5]);
        for kind in [
            ObstacleKind::Person,
            ObstacleKind::Unknown,
            ObstacleKind::GoalPost,
        ] {
            for distance in [0.15, 0.272] {
                let obstacle = BallObstacle {
                    kind,
                    ..coarse_robot(point![-distance, 0.0])
                };
                assert!(visible_segments(origin, target, &[obstacle], 0.08).is_empty());
            }
        }
        for kind in [ObstacleKind::Unknown, ObstacleKind::Robot] {
            let blockers = [
                coarse_robot(point![-0.15, 0.0]),
                BallObstacle {
                    circle: Circle::new(point![1.0, 0.0], 0.5),
                    kind,
                },
            ];
            assert!(visible_segments(origin, target, &blockers, 0.08).is_empty());
        }
    }

    #[test]
    fn physical_posts_remain_solid_when_ball_is_already_touching() {
        let field = FieldDimensions {
            ball_radius: 0.073,
            ..FieldDimensions::SPL_2025
        };
        // The physical upper post is at (4.525,.8): choose a slightly closer ball
        // than post radius + ball radius, while still outside the post itself.
        let ball = point![4.405, 0.8];
        let obstacles = obstacle_circles(Default::default(), &[], &field, 0.08);
        assert!(
            visible_segments(
                ball,
                LineSegment(point![3.0, 0.5], point![3.0, 1.0]),
                &obstacles,
                0.08
            )
            .is_empty()
        );
    }

    #[test]
    fn goal_corridors_use_initial_robot_proxy_egress() {
        let field = FieldDimensions {
            ball_radius: 0.073,
            ..FieldDimensions::SPL_2025
        };
        let ball = point![2.0, 0.0];
        let overlapping_proxy = Obstacle::robot(point![1.728, 0.0], 0.2, 0.2);
        let expected = goal_openings(ball, Default::default(), &[], &field, 0.08);
        let actual = goal_openings(ball, Default::default(), &[overlapping_proxy], &field, 0.08);
        assert!(!actual.is_empty());
        assert_eq!(actual, expected);
    }
}
