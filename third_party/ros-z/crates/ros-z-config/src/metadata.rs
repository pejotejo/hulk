/// Metadata describing one addressable config field.
#[derive(Debug, Clone, PartialEq)]
pub struct ConfigFieldMetadata {
    pub path: String,
    pub type_name: String,
    pub description: String,
    pub writable: bool,
    pub min: Option<f64>,
    pub max: Option<f64>,
}

impl ConfigFieldMetadata {
    /// Prefix the field path with a parent object path.
    pub fn prefixed(mut self, prefix: &str) -> Self {
        if prefix.is_empty() {
            return self;
        }
        self.path = format!("{prefix}.{}", self.path);
        self
    }
}

/// Optional metadata provider for config types.
///
/// This trait is only needed when using metadata-enabled bindings through
/// `bind_config_with_metadata_as::<T>(...)`. Ordinary config bindings do not require
/// metadata support.
pub trait ConfigMetadata {
    /// Return metadata for all addressable fields in the config type.
    fn config_metadata() -> Vec<ConfigFieldMetadata>;

    /// Return metadata with every field path prefixed by `prefix`.
    fn config_metadata_prefixed(prefix: &str) -> Vec<ConfigFieldMetadata> {
        Self::config_metadata()
            .into_iter()
            .map(|field| field.prefixed(prefix))
            .collect()
    }
}
