use serde::{Deserialize, Serialize};

/// Target scope for config reads, overlays, and runtime writes.
///
/// Precedence is always `Default < Location < Robot`.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize, Default,
)]
#[repr(u8)]
pub enum ConfigScope {
    #[default]
    Default = 0,
    Location = 1,
    Robot = 2,
}
