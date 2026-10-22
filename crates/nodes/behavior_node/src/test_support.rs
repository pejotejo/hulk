use std::time::Duration;

use hsl_network_messages::{GamePhase, Half, PlayerNumber};
use ros_z::time::Time;
use types::{
    field_dimensions::{FieldDimensions, GlobalFieldSide},
    filtered_game_controller_state::FilteredGameControllerState,
    filtered_game_state::FilteredGameState,
    parameters::{
        BehaviorParameters, GoalkeeperParameters, WalkSpeedParameters, WalkingBehaviorParameters,
    },
};

use crate::node::Blackboard;

pub(crate) fn game_controller_state() -> FilteredGameControllerState {
    FilteredGameControllerState {
        game_state: FilteredGameState::Initial,
        opponent_game_state: FilteredGameState::Initial,
        half: Half::First,
        remaining_time_in_half: Duration::ZERO,
        game_phase: GamePhase::Normal,
        kicking_team: None,
        penalties: Default::default(),
        remaining_number_of_messages: 0,
        sub_state: None,
        global_field_side: GlobalFieldSide::Away,
        new_own_penalties_last_cycle: Default::default(),
        new_opponent_penalties_last_cycle: Default::default(),
    }
}

#[cfg(test)]
impl Default for Blackboard {
    fn default() -> Self {
        Self {
            field_dimensions: FieldDimensions::SPL_2025,
            player_number: PlayerNumber::Three,
            parameters: BehaviorParameters {
                control: Default::default(),
                ball: Default::default(),
                walking: WalkingBehaviorParameters {
                    path_planning: Default::default(),
                    walk_and_stand: Default::default(),
                    speed: WalkSpeedParameters {
                        kicking: 1.0,
                        search: 1.0,
                        blocking: 1.0,
                        minimum_speed: 0.2,
                        velocity_fade_distance: 1.0,
                        walk_to_kickoff: 0.5,
                    },
                },
                kicking: Default::default(),
                goalkeeper: GoalkeeperParameters {
                    player_number: PlayerNumber::Three,
                    x_offset: 0.0,
                    passive_distance: 0.0,
                    striker_distance: 0.0,
                    kick_away_ball_maximum_robot_distance: 0.0,
                    active_defense_maximum_robot_distance: 0.0,
                    distance_to_goalpost: 0.0,
                },
                substates: Default::default(),
                kickoff: Default::default(),
                voronoi: Default::default(),
                network: Default::default(),
            },
            walking_velocity_limits: Default::default(),
            world_state: Default::default(),
            controller_input: None,
            remote_control_enabled: false,
            path_obstacles_output: Vec::new(),
            time_since_last_switch: Duration::ZERO,
            direction_difference: 0.0,
            voronoi_inputs: Vec::new(),
            ball: None,
            visual_kick_ball_position: None,
            last_ball: None,
            last_close_enough_to_kick: false,
            kick_target: None,
            ball_action: None,
            last_kick_target: None,
            last_motion_command: None,
            last_motion_switch_time: Time::zero(),
            last_motion_type: None,
            last_sent_game_controller_return_message_time: None,
            last_sent_hsl_message_time: None,
            last_announced_pass: None,
            replacement_goalkeeper_coordinator: Default::default(),
            last_sent_replacement_goalkeeper_claim: false,
            kickoff_receiver_wait_started: None,
            restart_pass_wait_started: None,
            kickoff_prepared: false,
            kickoff_started: None,
            last_closest_to_ball: false,
            closest_to_ball_entered_area_since: None,
            closest_to_ball_left_area_since: None,
            deflecting_ball: false,
            is_injected_motion_command: false,
            walk_position: None,
            body_motion: None,
            head_motion: None,
            voronoi_map: None,
        }
    }
}
