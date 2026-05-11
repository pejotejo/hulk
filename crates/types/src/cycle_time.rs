use std::time::{Duration, SystemTime, UNIX_EPOCH};

use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use ros_z::Message;
use serde::{Deserialize, Serialize};

#[derive(
    Clone,
    Copy,
    Debug,
    Deserialize,
    Serialize,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
    PartialEq,
    Message,
)]
pub struct CycleTime {
    pub start_time: SystemTime,
    pub last_cycle_duration: Duration,
}

impl Default for CycleTime {
    fn default() -> Self {
        Self {
            start_time: UNIX_EPOCH,
            last_cycle_duration: Default::default(),
        }
    }
}
