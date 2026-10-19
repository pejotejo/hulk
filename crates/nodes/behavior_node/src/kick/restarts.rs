use coordinate_systems::Field;
use hsl_network_messages::{SubState, Team};
use linear_algebra::{Orientation2, Point2, Pose2, point};
use ros_z::{Message, time::Time};
use serde::{Deserialize, Serialize};
use types::{
    behavior_tree::Status, filtered_game_state::FilteredGameState, motion_command::OrientationMode,
    motion_type::MotionType, obstacles::ObstacleKind, primary_state::PrimaryState,
    world_state::BallSource,
};

use crate::{
    actions::stand,
    node::Blackboard,
    switch_motion_type::is_allowed_to_switch,
    walk::{try_plan, walk_to_with_path},
};

const RELEASE_DISTANCE: f32 = 0.1;
const TOUCH_UNCERTAINTY: f32 = 0.05;
// Match the game-state filter's observed kickoff movement criterion.
const KICKOFF_SUPPORT_RELEASE_DISTANCE: f32 = 0.3;

/// Own kickoff permission applies to the taker before the ball moves. Keep
/// supporters at their legal starting poses until an observed first touch.
#[derive(Clone, Debug, Serialize, Deserialize, Message)]
pub struct KickoffSupport {
    pub started_at: Time,
    pub updated_at: Time,
    pub released: bool,
}

/// Update even while the playing subtree is inactive so a new Ready/Set or a
/// clock reset cannot reuse the preceding kickoff's release.
pub fn update_kickoff_support(blackboard: &mut Blackboard) {
    let own_kickoff = blackboard.world_state.robot.primary_state == PrimaryState::Playing
        && blackboard
            .world_state
            .filtered_game_controller_state
            .as_ref()
            .is_some_and(|game| {
                game.kicking_team == Some(Team::Hulks)
                    && game.sub_state.is_none()
                    && matches!(
                        game.game_state,
                        FilteredGameState::Playing { kick_off: true, .. }
                    )
            });
    if !own_kickoff {
        blackboard.kickoff_support = None;
        return;
    }
    let now = blackboard.world_state.now;
    if blackboard
        .kickoff_support
        .as_ref()
        .is_none_or(|state| now < state.updated_at)
    {
        blackboard.kickoff_support = Some(KickoffSupport {
            started_at: now,
            updated_at: now,
            released: false,
        });
    }
    let state = blackboard.kickoff_support.as_mut().unwrap();
    state.updated_at = now;
    if let Some(ball) = blackboard.world_state.ball {
        let last_seen = Time::from_wallclock(ball.last_seen_ball);
        state.released |= ball.source == BallSource::Own
            && finite_position(ball.ball_in_field)
            && state.started_at <= last_seen
            && last_seen <= now
            && now.duration_since(last_seen)
                <= crate::node::ball_timeout(&blackboard.parameters, ball.source)
            && ball.ball_in_field.coords().norm() > KICKOFF_SUPPORT_RELEASE_DISTANCE;
    }
}

pub fn is_waiting_for_kickoff_movement(blackboard: &mut Blackboard) -> bool {
    update_kickoff_support(blackboard);
    blackboard
        .kickoff_support
        .as_ref()
        .is_some_and(|state| !state.released)
}

/// A restart taker must not touch the ball again before another player does.
/// There is no time-based permission to retouch: an unseen ball or missing peer
/// must not silently erase the restriction.
#[derive(Clone, Debug, Serialize, Deserialize, Message)]
pub struct RestartRelease {
    pub origin: Point2<Field>,
    pub armed_at: Time,
    pub released: bool,
    pub restart: SubState,
    pub saw_open_play: bool,
}

/// Called only after execution has issued an actual kick command. Repeated
/// pre-contact commands retain the original ball position.
pub(crate) fn arm_restart_release(blackboard: &mut Blackboard, ball: Point2<Field>) {
    reset_restart_release_if_new_play(blackboard);
    if blackboard.restart_release.is_some() || !finite_position(ball) {
        return;
    }
    let Some(game) = blackboard
        .world_state
        .filtered_game_controller_state
        .as_ref()
    else {
        return;
    };
    let Some(restart) = game.sub_state else {
        return;
    };
    if game.kicking_team != Some(Team::Hulks) || restart == SubState::PenaltyKick {
        return;
    }
    blackboard.restart_release = Some(RestartRelease {
        origin: ball,
        armed_at: blackboard.world_state.now,
        released: false,
        restart,
        saw_open_play: false,
    });
}

/// Also called by the runtime outside Playing, so Ready/Set resets cannot be
/// missed while the normal playing subtree is inactive.
pub fn reset_restart_release_if_new_play(blackboard: &mut Blackboard) {
    let Some(release) = blackboard.restart_release.as_mut() else {
        return;
    };
    let primary_reset = matches!(
        blackboard.world_state.robot.primary_state,
        PrimaryState::Initial | PrimaryState::Ready | PrimaryState::Set | PrimaryState::Finished
    );
    let game_reset = blackboard
        .world_state
        .filtered_game_controller_state
        .as_ref()
        .is_some_and(|game| {
            if let Some(restart) = game.sub_state {
                restart != release.restart
                    || release.saw_open_play
                    || game.kicking_team == Some(Team::Opponent)
            } else {
                release.saw_open_play = true;
                matches!(
                    game.game_state,
                    FilteredGameState::Playing { kick_off: true, .. }
                )
            }
        });
    if primary_reset || game_reset || blackboard.world_state.now < release.armed_at {
        blackboard.restart_release = None;
    }
}

pub fn has_current_restart_ball(blackboard: &mut Blackboard) -> bool {
    current_ball(blackboard).is_some()
}

/// Continue the first kick before contact. Once it has released the ball, keep
/// this robot out of possession until another player can have made the touch.
pub fn should_yield_after_restart(blackboard: &mut Blackboard) -> bool {
    reset_restart_release_if_new_play(blackboard);
    let Some(release) = blackboard.restart_release.as_ref() else {
        return false;
    };
    let Some(ball) = current_ball(blackboard) else {
        // The tree searches without kicking while preserving the barrier.
        return true;
    };
    let released = release.released || (ball - release.origin).norm() > RELEASE_DISTANCE;
    if !released {
        return false;
    }
    // Network poses describe robot centers, not a foot reaching the ball. In
    // particular, the navigation radius is not evidence of another touch.
    let observed_touch = blackboard
        .world_state
        .robot
        .ground_to_field
        .is_some_and(|transform| {
            blackboard.world_state.obstacles.iter().any(|obstacle| {
                obstacle.kind == ObstacleKind::Robot
                    && (transform * obstacle.position - ball).norm()
                        <= obstacle.radius_at_foot_height
                            + blackboard.field_dimensions.ball_radius
                            + TOUCH_UNCERTAINTY
            })
        });
    if observed_touch {
        blackboard.restart_release = None;
        return false;
    }
    blackboard.restart_release.as_mut().unwrap().released = true;
    true
}

/// Yield possession without legacy walk fallbacks that can continue an old kick. The keeper moves home; field takers hold their release pose.
pub fn yield_after_restart(blackboard: &mut Blackboard) -> Status {
    if blackboard.last_motion_type != Some(MotionType::Walk) && !is_allowed_to_switch(blackboard) {
        return stand(blackboard);
    }
    if blackboard.world_state.robot.player_number != blackboard.parameters.goalkeeper.player_number
    {
        // A support route can catch a slowing outgoing ball even while its
        // current position is avoided. Hold the release position until another
        // player takes over; keep looking at the ball to observe that handoff.
        return stand(blackboard);
    }
    let Some(transform) = blackboard.world_state.robot.ground_to_field else {
        return stand(blackboard);
    };
    let home = transform.inverse()
        * point![
            -blackboard.field_dimensions.length / 2.0 + blackboard.parameters.goalkeeper.x_offset,
            0.0
        ];
    let Some(ball) = current_ball(blackboard) else {
        return stand(blackboard);
    };
    let ball_in_ground = transform.inverse() * ball;
    let Some(path) = try_plan(blackboard, home, transform) else {
        return stand(blackboard);
    };
    let walk = blackboard.parameters.walking.walk_and_stand;
    walk_to_with_path(
        blackboard,
        Pose2::from_parts(home, Orientation2::from_vector(ball_in_ground - home)),
        blackboard.parameters.walking.speed.blocking,
        OrientationMode::LookAt {
            target: ball_in_ground,
            tolerance: walk.orientation_tolerance,
        },
        walk.normal_distance_to_be_aligned,
        walk.goalkeeper_hysteresis,
        Some(path),
    )
}

fn current_ball(blackboard: &Blackboard) -> Option<Point2<Field>> {
    blackboard.world_state.ball.and_then(|ball| {
        (ball.source == BallSource::Own && finite_position(ball.ball_in_field))
            .then_some(ball.ball_in_field)
    })
}

fn finite_position(position: Point2<Field>) -> bool {
    position
        .inner
        .iter()
        .all(|coordinate| coordinate.is_finite())
}

#[cfg(test)]
mod tests {
    use super::*;
    use hsl_network_messages::PlayerNumber;
    use std::time::Duration;
    use types::{
        field_dimensions::FieldDimensions,
        filtered_game_controller_state::FilteredGameControllerState,
        obstacles::Obstacle,
        time_wrapper::TimeWrapper,
        world_state::{BallState, PlayerState},
    };

    fn blackboard() -> Blackboard {
        let mut b = Blackboard::default();
        b.field_dimensions = FieldDimensions::SPL_2025;
        b.world_state.robot.primary_state = PrimaryState::Playing;
        b.world_state.robot.player_number = PlayerNumber::Two;
        b.world_state.robot.ground_to_field = Some(Default::default());
        b.parameters.walking.path_planning.robot_radius = 0.28;
        b.parameters.voronoi.teammate_pose_max_age = Duration::from_secs(1);
        b.world_state.filtered_game_controller_state = Some(FilteredGameControllerState {
            sub_state: Some(SubState::CornerKick),
            kicking_team: Some(Team::Hulks),
            ..Default::default()
        });
        set_ball(&mut b, point![4.5, 3.0]);
        arm_restart_release(&mut b, point![4.5, 3.0]);
        b
    }

    fn set_ball(b: &mut Blackboard, position: Point2<Field>) {
        b.world_state.ball = Some(BallState {
            ball_in_field: position,
            ball_in_ground: b.world_state.robot.ground_to_field.unwrap().inverse() * position,
            source: BallSource::Own,
            ..Default::default()
        });
    }

    fn own_kickoff() -> Blackboard {
        let mut b = blackboard();
        b.restart_release = None;
        b.world_state.now = Time::from_nanos(10_000_000_000);
        b.parameters.ball.last_ball_timeout = Duration::from_secs(1);
        let game = b
            .world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap();
        game.sub_state = None;
        game.game_state = FilteredGameState::Playing {
            ball_is_free: true,
            kick_off: true,
        };
        observe_kickoff_ball(&mut b, Point2::origin());
        b
    }

    fn observe_kickoff_ball(b: &mut Blackboard, position: Point2<Field>) {
        set_ball(b, position);
        b.world_state.ball.as_mut().unwrap().last_seen_ball = b.world_state.now.to_wallclock();
    }

    #[test]
    fn own_kickoff_support_waits_for_observed_movement_and_stays_released() {
        let mut b = own_kickoff();
        assert!(is_waiting_for_kickoff_movement(&mut b));
        observe_kickoff_ball(&mut b, point![0.29, 0.0]);
        assert!(is_waiting_for_kickoff_movement(&mut b));
        observe_kickoff_ball(&mut b, point![0.31, 0.0]);
        assert!(!is_waiting_for_kickoff_movement(&mut b));
        observe_kickoff_ball(&mut b, Point2::origin());
        assert!(!is_waiting_for_kickoff_movement(&mut b));
        b.world_state.ball = None;
        assert!(!is_waiting_for_kickoff_movement(&mut b));
    }

    #[test]
    fn missing_team_stale_future_and_nonfinite_balls_do_not_release_kickoff_support() {
        let mut b = own_kickoff();
        assert!(is_waiting_for_kickoff_movement(&mut b));
        b.world_state.ball = None;
        assert!(is_waiting_for_kickoff_movement(&mut b));

        observe_kickoff_ball(&mut b, point![1.0, 0.0]);
        b.world_state.ball.as_mut().unwrap().source = BallSource::Team;
        assert!(is_waiting_for_kickoff_movement(&mut b));
        b.world_state.ball.as_mut().unwrap().source = BallSource::Own;
        b.world_state.now = b.world_state.now + Duration::from_secs(2);
        assert!(is_waiting_for_kickoff_movement(&mut b));

        observe_kickoff_ball(&mut b, point![1.0, 0.0]);
        b.world_state.ball.as_mut().unwrap().last_seen_ball =
            (b.world_state.now + Duration::from_millis(20)).to_wallclock();
        assert!(is_waiting_for_kickoff_movement(&mut b));
        observe_kickoff_ball(&mut b, point![f32::NAN, 0.0]);
        assert!(is_waiting_for_kickoff_movement(&mut b));
    }

    #[test]
    fn ready_set_and_clock_reset_require_a_new_kickoff_observation() {
        for reset in [PrimaryState::Ready, PrimaryState::Set] {
            let mut b = own_kickoff();
            observe_kickoff_ball(&mut b, point![0.4, 0.0]);
            assert!(!is_waiting_for_kickoff_movement(&mut b));
            b.world_state.robot.primary_state = reset;
            update_kickoff_support(&mut b);
            assert!(b.kickoff_support.is_none());
            b.world_state.robot.primary_state = PrimaryState::Playing;
            b.world_state.now = b.world_state.now + Duration::from_millis(20);
            // The preceding kickoff's observed ball cannot release this one.
            assert!(is_waiting_for_kickoff_movement(&mut b));
            observe_kickoff_ball(&mut b, Point2::origin());
            assert!(is_waiting_for_kickoff_movement(&mut b));
        }

        let mut b = own_kickoff();
        observe_kickoff_ball(&mut b, point![0.4, 0.0]);
        assert!(!is_waiting_for_kickoff_movement(&mut b));
        b.world_state.now = b.world_state.now - Duration::from_secs(1);
        assert!(is_waiting_for_kickoff_movement(&mut b));
        observe_kickoff_ball(&mut b, point![0.4, 0.0]);
        assert!(!is_waiting_for_kickoff_movement(&mut b));
    }

    #[test]
    fn opponent_kickoffs_and_open_play_do_not_hold_our_supporters() {
        let mut b = own_kickoff();
        assert!(is_waiting_for_kickoff_movement(&mut b));
        b.world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .kicking_team = Some(Team::Opponent);
        assert!(!is_waiting_for_kickoff_movement(&mut b));
        assert!(b.kickoff_support.is_none());
        let game = b
            .world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap();
        game.kicking_team = Some(Team::Hulks);
        game.game_state = FilteredGameState::Playing {
            ball_is_free: true,
            kick_off: false,
        };
        assert!(!is_waiting_for_kickoff_movement(&mut b));
        b.world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .game_state = FilteredGameState::Playing {
            ball_is_free: true,
            kick_off: true,
        };
        assert!(is_waiting_for_kickoff_movement(&mut b));
    }

    #[test]
    fn first_release_yields_but_precontact_kick_continues() {
        let mut b = blackboard();
        assert!(!should_yield_after_restart(&mut b));
        set_ball(&mut b, point![4.47, 2.97]);
        assert!(!should_yield_after_restart(&mut b));
        arm_restart_release(&mut b, point![4.47, 2.97]);
        assert_eq!(b.restart_release.as_ref().unwrap().origin, point![4.5, 3.0]);
        set_ball(&mut b, point![4.1, 2.6]);
        assert!(should_yield_after_restart(&mut b));
        assert!(b.restart_release.as_ref().unwrap().released);
    }

    #[test]
    fn a_weak_first_touch_still_yields_after_visible_movement() {
        let mut b = blackboard();
        set_ball(&mut b, point![4.35, 3.0]);
        assert!(should_yield_after_restart(&mut b));
        assert!(b.restart_release.as_ref().unwrap().released);
    }

    #[test]
    fn a_fresh_teammate_pose_alone_cannot_establish_a_second_touch() {
        let mut b = blackboard();
        set_ball(&mut b, point![4.0, 2.5]);
        b.world_state.now = b.world_state.now + Duration::from_millis(100);
        b.timed_player_states.three = Some(TimeWrapper {
            time: b.world_state.now,
            inner: PlayerState {
                pose: Pose2::new(point![3.8, 2.5], 0.0),
                ..Default::default()
            },
        });
        assert!(should_yield_after_restart(&mut b));
        assert!(b.restart_release.is_some());
    }

    #[test]
    fn a_locally_observed_other_robot_can_supply_the_second_touch() {
        let mut b = blackboard();
        set_ball(&mut b, point![4.0, 2.5]);
        b.world_state.obstacles = vec![Obstacle::robot(point![3.8, 2.5], 0.2, 0.2)];
        assert!(!should_yield_after_restart(&mut b));
        assert!(b.restart_release.is_none());
    }

    #[test]
    fn stale_or_pre_kick_peer_positions_do_not_release_the_taker() {
        for (received, now) in [
            (Time::zero(), Time::from_nanos(200_000_000)),
            (
                Time::from_nanos(100_000_000),
                Time::from_nanos(2_000_000_000),
            ),
        ] {
            let mut b = blackboard();
            set_ball(&mut b, point![4.0, 2.5]);
            b.world_state.now = now;
            b.timed_player_states.three = Some(TimeWrapper {
                time: received,
                inner: PlayerState {
                    pose: Pose2::new(point![3.8, 2.5], 0.0),
                    ..Default::default()
                },
            });
            assert!(should_yield_after_restart(&mut b));
            assert!(b.restart_release.is_some());
        }
    }

    #[test]
    fn field_taker_holds_release_position_instead_of_chasing_the_ball_to_support() {
        let mut b = blackboard();
        b.parameters.goalkeeper.player_number = PlayerNumber::One;
        set_ball(&mut b, point![4.0, 2.5]);
        assert!(should_yield_after_restart(&mut b));
        assert_eq!(yield_after_restart(&mut b), Status::Success);
        assert!(matches!(
            b.body_motion,
            Some(types::motion_command::BodyMotion::Stand)
        ));
    }

    #[test]
    fn losing_the_ball_keeps_the_barrier_without_a_timeout() {
        let mut b = blackboard();
        set_ball(&mut b, point![4.0, 2.5]);
        assert!(should_yield_after_restart(&mut b));
        b.world_state.ball = None;
        b.world_state.now = b.world_state.now + Duration::from_secs(30);
        assert!(should_yield_after_restart(&mut b));
        assert!(!has_current_restart_ball(&mut b));
        assert!(b.restart_release.is_some());
    }

    #[test]
    fn new_ready_set_restart_and_kickoff_reset_the_barrier() {
        for primary in [PrimaryState::Ready, PrimaryState::Set] {
            let mut b = blackboard();
            b.world_state.robot.primary_state = primary;
            reset_restart_release_if_new_play(&mut b);
            assert!(b.restart_release.is_none());
        }
        let mut b = blackboard();
        b.world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap()
            .sub_state = Some(SubState::ThrowIn);
        reset_restart_release_if_new_play(&mut b);
        assert!(b.restart_release.is_none());
        let mut b = blackboard();
        let game = b
            .world_state
            .filtered_game_controller_state
            .as_mut()
            .unwrap();
        game.sub_state = None;
        game.game_state = FilteredGameState::Playing {
            ball_is_free: false,
            kick_off: true,
        };
        reset_restart_release_if_new_play(&mut b);
        assert!(b.restart_release.is_none());
    }
}
