use std::env::current_dir;

use bevy::{
    app::{App, AppExit, First, Plugin, TaskPoolPlugin, Update},
    diagnostic::FrameCountPlugin,
    ecs::{message::Messages, schedule::IntoScheduleConfigs},
    time::Time,
};
use color_eyre::{
    Result,
    eyre::{Context, ContextCompat, bail},
};

use hula_types::hardware::Ids;
use repository::Repository;

use crate::{
    autoref::autoref_plugin,
    ball::{BallResource, move_ball},
    field_dimensions::SimulatorFieldDimensions,
    game_controller::{GameController, game_controller_plugin},
    recorder::{Recording, recording_plugin},
    robot::{self, cycle_robots, move_robots},
    soft_error::{SoftErrorResource, soft_error_plugin},
    structs::Parameters,
    time::{Ticks, update_time},
    whistle::WhistleResource,
};

#[derive(Default, Debug, Copy, Clone, Eq, PartialEq)]
pub enum Mode {
    Serve,
    #[default]
    Run,
}

#[derive(Copy, Clone)]
pub struct SimulatorPlugin {
    mode: Mode,
}

impl SimulatorPlugin {
    pub fn new(mode: Mode) -> Self {
        Self { mode }
    }

    pub fn run() -> Self {
        Self::new(Mode::Run)
    }

    pub fn serve() -> Self {
        Self::new(Mode::Serve)
    }

    pub fn mode(&self) -> Mode {
        self.mode
    }
}

impl Default for SimulatorPlugin {
    fn default() -> Self {
        Self::run()
    }
}

impl Plugin for SimulatorPlugin {
    fn build(&self, app: &mut App) {
        let parameters = load_parameters().expect("failed to load parameters");

        app.add_plugins((TaskPoolPlugin::default(), FrameCountPlugin))
            .add_plugins(game_controller_plugin)
            .add_plugins(autoref_plugin)
            .add_plugins(soft_error_plugin)
            .insert_resource(SimulatorFieldDimensions::from(parameters.field_dimensions))
            .insert_resource(GameController::default())
            .insert_resource(BallResource::default())
            .insert_resource(WhistleResource::default())
            .insert_resource(robot::Messages::default())
            .insert_resource(Time::<()>::default())
            .insert_resource(Time::<Ticks>::default())
            .add_systems(First, update_time)
            .add_systems(
                Update,
                (
                    move_robots,
                    move_ball.after(move_robots),
                    cycle_robots.before(move_robots),
                ),
            );

        if self.mode == Mode::Serve {
            app.add_plugins(recording_plugin);
        }
    }
}

pub trait AppExt {
    fn run_to_completion(&mut self) -> Result<()>;
}

impl AppExt for App {
    fn run_to_completion(&mut self) -> Result<()> {
        let mut event_reader = self
            .world_mut()
            .resource_mut::<Messages<AppExit>>()
            .get_cursor();

        let exit = loop {
            self.update();

            let events = self.world().resource::<Messages<AppExit>>();
            if let Some(exit_message) = event_reader.read(events).last() {
                break exit_message.clone();
            }
        };

        if let AppExit::Error(code) = exit {
            bail!("scenario exited with error code {code}")
        }

        let soft_errors = self
            .world_mut()
            .get_resource_mut::<SoftErrorResource>()
            .expect("soft error storage should exist");

        if !soft_errors.errors.is_empty() {
            bail!("{} soft error(s) found", soft_errors.errors.len());
        }

        if let Some(recording) = self.world_mut().remove_resource::<Recording>() {
            recording.join()?;
        }

        Ok(())
    }
}

fn load_parameters() -> Result<Parameters> {
    let ids = Ids {
        robot_id: "behavior_simulator".to_string(),
    };
    let current_directory = current_dir().wrap_err("failed to get current directory")?;
    let repository =
        Repository::find_root(current_directory).wrap_err("failed to get repository root")?;
    let parameters_path = repository.root.join("etc/parameters");

    parameters::directory::deserialize(parameters_path, &ids, true)
        .wrap_err("failed to parse initial parameters")
}

#[cfg(test)]
mod tests {
    use bevy::app::App;

    use crate::{autoref::AutorefState, recorder::Recording};

    use super::*;

    #[test]
    fn run_mode_does_not_install_recording() {
        let mut app = App::new();

        app.add_plugins(SimulatorPlugin::run());

        assert!(!app.world().contains_resource::<Recording>());
    }

    #[test]
    fn default_plugin_is_headless_for_tests() {
        assert_eq!(SimulatorPlugin::default().mode(), Mode::Run);
    }

    #[test]
    fn simulator_installs_autoref() {
        let mut app = App::new();

        app.add_plugins(SimulatorPlugin::run());

        assert!(app.world().contains_resource::<AutorefState>());
    }
}
