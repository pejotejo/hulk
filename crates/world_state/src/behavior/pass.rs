use std::time::Duration;

use hsl_network_messages::{CoordinationIntent, PassIntent};
use linear_algebra::{Orientation2, Pose2, vector};
use types::{behavior_tree::Status, motion_command::OrientationMode};

use crate::behavior::{node::Blackboard, walk};

pub fn active_pass_to_me(blackboard: &Blackboard) -> Option<PassIntent> {
    blackboard
        .world_state
        .player_states
        .iter()
        .filter_map(|(_, player_state)| player_state.as_ref())
        .filter_map(|player_state| player_state.coordination)
        .find_map(|coordination| match coordination.intent {
            CoordinationIntent::Pass(pass) => {
                if pass.receiver != blackboard.world_state.robot.player_number {
                    return None;
                }
                let age = blackboard
                    .world_state
                    .now
                    .duration_since(coordination.received_at)
                    .unwrap_or(Duration::ZERO)
                    .saturating_add(pass.age);
                (age <= blackboard.pass_intent_timeout).then_some(pass)
            }
        })
}

pub fn walk_to_pass_receive_position(blackboard: &mut Blackboard) -> Status {
    let (Some(pass), Some(ground_to_field)) = (
        active_pass_to_me(blackboard),
        blackboard.world_state.robot.ground_to_field,
    ) else {
        return Status::Failure;
    };

    let field_to_ground = ground_to_field.inverse();
    let receive_point = field_to_ground * pass.receive_point;
    let tolerance = blackboard.parameters.walk_and_stand.orientation_tolerance;
    let (orientation, orientation_mode) = if let Some(ball) = &blackboard.ball {
        let ball_position = field_to_ground * ball.position;
        let direction = (ball_position - receive_point)
            .try_normalize(f32::EPSILON)
            .unwrap_or_else(|| vector!(1.0, 0.0));
        let orientation = Orientation2::from_vector(direction);
        (
            orientation,
            OrientationMode::LookAt {
                target: ball_position,
                tolerance,
            },
        )
    } else {
        let direction = receive_point
            .coords()
            .try_normalize(f32::EPSILON)
            .unwrap_or_else(|| vector!(1.0, 0.0));
        let orientation = Orientation2::from_vector(direction);
        (
            orientation,
            OrientationMode::LookTowards {
                direction: orientation,
                tolerance,
            },
        )
    };

    walk::walk_to(
        blackboard,
        Pose2::from_parts(receive_point, orientation),
        blackboard.parameters.walk_speed.support,
        orientation_mode,
        blackboard
            .parameters
            .walk_and_stand
            .normal_distance_to_be_aligned,
        blackboard.parameters.walk_and_stand.hysteresis,
    )
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, SystemTime};

    use coordinate_systems::Field;
    use hsl_network_messages::{PassIntent, PlayerNumber};
    use linear_algebra::{Point2, point};
    use types::{
        field_dimensions::FieldDimensions,
        motion_command::MotionCommand,
        parameters::BehaviorParameters,
        players::Players,
        world_state::{PlayerCoordinationState, PlayerState, WorldState},
    };

    use crate::behavior::kick_selector::{CycleIntent, KickMemory};

    use super::*;

    fn blackboard(
        now: SystemTime,
        received_at: SystemTime,
        timeout: Duration,
        receive_point: Point2<Field>,
        pass_age: Duration,
    ) -> Blackboard {
        let mut world_state = WorldState::default();
        world_state.now = now;
        world_state.robot.player_number = PlayerNumber::Three;
        world_state.player_states = Players {
            two: Some(PlayerState {
                coordination: Some(PlayerCoordinationState {
                    intent: CoordinationIntent::Pass(PassIntent {
                        sequence: 7,
                        receiver: PlayerNumber::Three,
                        receive_point,
                        age: pass_age,
                    }),
                    received_at,
                }),
                ..Default::default()
            }),
            ..Players::new(None)
        };

        Blackboard {
            field_dimensions: FieldDimensions::SPL_2025,
            free_kick_obstacle_radius: 0.0,
            pass_intent_timeout: timeout,
            parameters: BehaviorParameters::default(),
            world_state,

            path_obstacles_output: Vec::new(),
            time_since_last_switch: Duration::ZERO,
            direction_difference: 0.0,
            voronoi_inputs: Vec::new(),

            ball: None,
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

    #[test]
    fn active_pass_to_self_is_accepted_when_fresh() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let receive_point = point!(1.0, 0.5);
        let blackboard = blackboard(
            now,
            now - Duration::from_millis(500),
            Duration::from_secs(1),
            receive_point,
            Duration::ZERO,
        );

        let pass = active_pass_to_me(&blackboard).expect("fresh pass to self should be active");

        assert_eq!(pass.sequence, 7);
        assert_eq!(pass.receiver, PlayerNumber::Three);
        assert_eq!(pass.receive_point, receive_point);
    }

    #[test]
    fn active_pass_to_self_is_rejected_when_stale() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let blackboard = blackboard(
            now,
            now - Duration::from_secs(2),
            Duration::from_secs(1),
            point!(1.0, 0.5),
            Duration::ZERO,
        );

        assert!(active_pass_to_me(&blackboard).is_none());
    }

    #[test]
    fn active_pass_to_self_is_rejected_when_sender_age_is_stale() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let blackboard = blackboard(
            now,
            now,
            Duration::from_secs(1),
            point!(1.0, 0.5),
            Duration::from_secs(2),
        );

        assert!(active_pass_to_me(&blackboard).is_none());
    }

    #[test]
    fn active_pass_to_self_rejects_max_sender_age_without_overflow() {
        let now = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        let blackboard = blackboard(
            now,
            now,
            Duration::from_secs(1),
            point!(1.0, 0.5),
            Duration::MAX,
        );

        assert!(active_pass_to_me(&blackboard).is_none());
    }
}
