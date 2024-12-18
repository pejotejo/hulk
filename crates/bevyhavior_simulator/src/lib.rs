#![allow(unexpected_cfgs)]

use hardware::{NetworkInterface, RecordingInterface, SpeakerInterface, TimeInterface};
use interfake::FakeDataInterface;

pub mod autoref;
pub mod ball;
pub mod fake_data;
pub mod field_dimensions;
pub mod game_controller;
pub mod interfake;
pub mod recorder;
pub mod robot;
pub mod scenario;
pub mod server;
pub mod simulator;
pub mod time;
pub mod whistle;

include!(concat!(env!("OUT_DIR"), "/generated_code.rs"));

pub trait HardwareInterface:
    TimeInterface + NetworkInterface + RecordingInterface + FakeDataInterface + SpeakerInterface
{
}