use std::{fs, io::Write, path::Path};

use serde_json::Value;

use crate::{ConfigError, Result};

pub fn write_pretty_json(path: &Path, value: &Value) -> Result<()> {
    let Some(parent) = path.parent() else {
        return Err(ConfigError::PersistenceError {
            path: path.to_path_buf(),
            message: "target path has no parent directory".to_string(),
        });
    };

    fs::create_dir_all(parent).map_err(|err| ConfigError::PersistenceError {
        path: path.to_path_buf(),
        message: err.to_string(),
    })?;

    let data =
        serde_json::to_string_pretty(value).map_err(|err| ConfigError::PersistenceError {
            path: path.to_path_buf(),
            message: err.to_string(),
        })?;

    let file_name = path
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("config.json");
    let temp_path = parent.join(format!(".{file_name}.tmp"));

    let mut temp_file =
        fs::File::create(&temp_path).map_err(|err| ConfigError::PersistenceError {
            path: temp_path.clone(),
            message: err.to_string(),
        })?;
    temp_file
        .write_all(data.as_bytes())
        .map_err(|err| ConfigError::PersistenceError {
            path: temp_path.clone(),
            message: err.to_string(),
        })?;
    temp_file
        .sync_all()
        .map_err(|err| ConfigError::PersistenceError {
            path: temp_path.clone(),
            message: err.to_string(),
        })?;
    drop(temp_file);

    fs::rename(&temp_path, path).map_err(|err| ConfigError::PersistenceError {
        path: path.to_path_buf(),
        message: err.to_string(),
    })?;

    Ok(())
}
