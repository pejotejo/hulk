use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};

#[derive(
    Clone,
    Copy,
    Debug,
    Deserialize,
    Eq,
    Hash,
    PartialEq,
    Serialize,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
)]
pub enum RampDirection{
    Left,
    Right,
}


#[derive(
    Clone,
    Copy,
    Debug,
    Default,
    Deserialize,
    Eq,
    Hash,
    PartialEq,
    Serialize,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
)]
pub enum PrimaryState {
    #[default]
    Unstiff,
    Animation {
        stiff: bool,
    },
    Initial,
    Ready,
    Set,
    Playing,
    Penalized,
    Finished,
    Calibration,
    Standby,
    KickingRollingBall{
        ramp_direction: RampDirection,
    },
}
