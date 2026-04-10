pub mod bringup_manager;
pub mod config;
pub mod msgs;
pub mod namespacing;
pub mod nodes;
pub mod stack;
pub mod topics;

pub trait IntoEyreResultExt<T> {
    fn into_eyre(self) -> color_eyre::Result<T>;
}

impl<T, E> IntoEyreResultExt<T> for std::result::Result<T, E>
where
    E: std::fmt::Display,
{
    fn into_eyre(self) -> color_eyre::Result<T> {
        self.map_err(|error| color_eyre::eyre::eyre!(error.to_string()))
    }
}
