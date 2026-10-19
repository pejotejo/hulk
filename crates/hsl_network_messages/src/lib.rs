mod bindings;
mod game_controller_return_message;
mod game_controller_state_message;

use std::{
    fmt::{self, Display, Formatter},
    time::Duration,
};

use coordinate_systems::Field;
use linear_algebra::{Point2, Pose2};
use ros_z::Message;
use serde::{Deserialize, Serialize};

pub use game_controller_return_message::GameControllerReturnMessage;
pub use game_controller_state_message::{
    GameControllerStateMessage, GamePhase, GameState, Half, Penalty, PenaltyShoot, Player,
    SubState, Team, TeamColor, TeamState,
};

#[derive(Clone, Copy, Debug, Deserialize, Serialize, Message)]
pub enum HulkMessage {
    State(StateMessage),
    /// Optional coordination envelope; the legacy State wire layout is unchanged.
    CoordinatedState(CoordinatedStateMessage),
    /// Keep a teammate's pose visible while it cannot take part in play.
    UnavailableState(StateMessage),
}

impl HulkMessage {
    pub fn state(&self) -> &StateMessage {
        match self {
            Self::State(state) | Self::UnavailableState(state) => state,
            Self::CoordinatedState(message) => &message.state,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Message)]
pub struct PassIntent {
    pub receiver: PlayerNumber,
    pub target: Point2<Field>,
}

impl PassIntent {
    pub const TIMEOUT: Duration = Duration::from_secs(6);
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Message)]
pub struct PassReady {
    pub sender: PlayerNumber,
    pub target: Point2<Field>,
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, Message)]
pub struct CoordinatedStateMessage {
    pub state: StateMessage,
    pub pass: Option<PassIntent>,
    pub pass_ready: Option<PassReady>,
}

impl Default for HulkMessage {
    fn default() -> Self {
        HulkMessage::State(StateMessage::default())
    }
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, Message)]
pub struct StrikerMessage {
    pub player_number: PlayerNumber,
    pub pose: Pose2<Field>,
    pub ball_position: BallPosition<Field>,
    pub time_to_reach_kick_position: Duration,
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, Message)]
pub struct StateMessage {
    pub player_number: PlayerNumber,
    pub pose: Pose2<Field>,
    pub head_yaw: f32,
    pub ball_position: Option<BallPosition<Field>>,
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, Message)]
pub struct BallPosition<Frame> {
    pub position: Point2<Frame>,
    pub age: Duration,
}

pub const HULKS_TEAM_NUMBER: u8 = 24;
pub const NONE_TEAM_NUMBER: u8 = 255;

#[derive(
    Clone,
    Copy,
    Debug,
    Default,
    Deserialize,
    Eq,
    Hash,
    Ord,
    PartialEq,
    PartialOrd,
    Serialize,
    ros_z::Message,
)]
pub enum PlayerNumber {
    One,
    Two,
    #[default]
    Three,
    Four,
    Five,
}

impl Display for PlayerNumber {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        let number = match self {
            PlayerNumber::One => "1",
            PlayerNumber::Two => "2",
            PlayerNumber::Three => "3",
            PlayerNumber::Four => "4",
            PlayerNumber::Five => "5",
        };

        write!(formatter, "{number}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn hulk_striker_message_size() {
        let test_message = HulkMessage::State(StateMessage {
            player_number: PlayerNumber::Five,
            pose: Pose2::default(),
            head_yaw: 0.0,
            ball_position: Some(BallPosition {
                position: Point2::origin(),
                age: Duration::MAX,
            }),
        });
        assert!(bincode::serialize(&test_message).unwrap().len() <= 128)
    }
    #[test]
    fn legacy_state_keeps_its_original_bincode_envelope() {
        #[derive(Serialize)]
        enum LegacyHulkMessage {
            State(StateMessage),
        }
        let state = StateMessage {
            player_number: PlayerNumber::Five,
            pose: Pose2::default(),
            head_yaw: 1.2,
            ball_position: Some(BallPosition {
                position: Point2::origin(),
                age: Duration::MAX,
            }),
        };
        let legacy = bincode::serialize(&LegacyHulkMessage::State(state)).unwrap();
        let current = bincode::serialize(&HulkMessage::State(state)).unwrap();
        assert_eq!(current, legacy);
    }

    #[test]
    fn coordinated_state_round_trips_within_team_packet_budget() {
        let intent = PassIntent {
            receiver: PlayerNumber::Three,
            target: Point2::origin(),
        };
        let message = HulkMessage::CoordinatedState(CoordinatedStateMessage {
            state: StateMessage {
                ball_position: Some(BallPosition {
                    age: Duration::MAX,
                    ..Default::default()
                }),
                ..Default::default()
            },
            pass: Some(intent),
            pass_ready: Some(PassReady {
                sender: PlayerNumber::Two,
                target: intent.target,
            }),
        });
        let bytes = bincode::serialize(&message).unwrap();
        assert!(bytes.len() <= 128, "packet size {}", bytes.len());
        let HulkMessage::CoordinatedState(decoded) = bincode::deserialize(&bytes).unwrap() else {
            panic!("coordination envelope lost")
        };
        assert_eq!(decoded.pass, Some(intent));
        assert_eq!(decoded.pass_ready.unwrap().sender, PlayerNumber::Two);
        assert!(decoded.state.ball_position.is_some());
    }

    #[test]
    fn coordinated_state_keeps_its_previous_bincode_envelope() {
        #[derive(Serialize)]
        enum PreviousHulkMessage {
            State(StateMessage),
            CoordinatedState(CoordinatedStateMessage),
        }
        let coordinated = CoordinatedStateMessage {
            state: StateMessage {
                player_number: PlayerNumber::Two,
                ..Default::default()
            },
            pass: Some(PassIntent {
                receiver: PlayerNumber::Three,
                target: Point2::origin(),
            }),
            pass_ready: Some(PassReady {
                sender: PlayerNumber::Four,
                target: Point2::origin(),
            }),
        };
        for (previous, current) in [
            (
                PreviousHulkMessage::State(coordinated.state),
                HulkMessage::State(coordinated.state),
            ),
            (
                PreviousHulkMessage::CoordinatedState(coordinated),
                HulkMessage::CoordinatedState(coordinated),
            ),
        ] {
            assert_eq!(
                bincode::serialize(&previous).unwrap(),
                bincode::serialize(&current).unwrap()
            );
        }
    }

    #[test]
    fn unavailable_state_round_trips_within_team_packet_budget_and_exposes_the_pose() {
        let state = StateMessage {
            player_number: PlayerNumber::Five,
            pose: Pose2::new(Point2::origin(), 1.2),
            head_yaw: 0.3,
            ball_position: Some(BallPosition {
                position: Point2::origin(),
                age: Duration::MAX,
            }),
        };
        let message = HulkMessage::UnavailableState(state);
        let bytes = bincode::serialize(&message).unwrap();
        assert!(bytes.len() <= 128, "packet size {}", bytes.len());
        let decoded: HulkMessage = bincode::deserialize(&bytes).unwrap();
        assert!(matches!(decoded, HulkMessage::UnavailableState(_)));
        assert_eq!(decoded.state().player_number, state.player_number);
        assert_eq!(decoded.state().pose, state.pose);
        assert_eq!(decoded.state().head_yaw, state.head_yaw);
        assert!(decoded.state().ball_position.is_some());
    }
}
