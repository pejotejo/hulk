pub mod bringup_manager;
pub mod config;
pub mod msgs;
pub mod namespacing;
pub mod nodes;
pub mod stack;
pub mod topics;

pub fn into_eyre<T, E>(result: std::result::Result<T, E>) -> color_eyre::Result<T>
where
    E: std::fmt::Display,
{
    result.map_err(|error| color_eyre::eyre::eyre!(error.to_string()))
}
