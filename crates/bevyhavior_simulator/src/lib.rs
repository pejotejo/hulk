#![allow(unexpected_cfgs)]

use interfake::FakeDataInterface;

pub mod autoref;
pub mod ball;
pub mod fake_data;
pub mod field_dimensions;
pub mod game_controller;
pub mod interfake;
mod recorder;
pub mod robot;
pub mod scenario;
mod server;
pub mod simulator;
pub mod soft_error;
pub mod time;
pub mod whistle;

include!(concat!(env!("OUT_DIR"), "/generated_code.rs"));

pub trait HardwareInterface:
    hardware::NetworkInterface
    + hardware::RecordingInterface
    + hardware::TimeInterface
    + FakeDataInterface
{
}

#[cfg(test)]
mod tests {
    #[test]
    fn selected_scenarios_are_declared_as_bins() {
        let manifest = include_str!("../Cargo.toml");

        for scenario in [
            "golden_goal",
            "golden_goal_opponent_kickoff",
            "flickering_ball",
            "penalty_shootout_attacking",
            "goal_kicks",
            "ingame_penalty_kick_opponent",
            "ingame_penalty_kick_opponent_with_kick",
            "ingame_penalty_kick",
            "ball_search",
            "defender_positioning",
            "standing_searcher",
            "quantum_ball",
            "hulks_vs_ghosts",
            "walk_around_ball",
            "oscillating_obstacle",
            "mpc_step_planning_optimizer",
            "kicking_team_filtering",
            "kick_in",
            "ingame_penalty_kick_striker_penalized",
            "demonstration",
            "intercept_ball",
            "striker_dies",
            "golden_goal_striker_penalized",
            "step_planning_test",
        ] {
            assert!(
                manifest.contains(&format!("name = \"{scenario}\"")),
                "{scenario} should be declared as a runnable scenario bin",
            );
        }
    }
}
