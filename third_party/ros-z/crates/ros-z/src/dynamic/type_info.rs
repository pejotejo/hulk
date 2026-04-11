use tracing::warn;

use crate::dynamic::{MessageSchema, MessageSchemaTypeDescription};
use crate::entity::{TypeHash, TypeInfo, TYPE_HASH_NOT_SUPPORTED};
use crate::ros_msg::dds_type_name_to_canonical;

pub(crate) fn ros_type_name_from_dds(dds_name: &str) -> String {
    dds_type_name_to_canonical(dds_name)
}

pub(crate) fn schema_hash(schema: &MessageSchema) -> Option<TypeHash> {
    match schema.compute_type_hash() {
        Ok(hash) => Some(hash),
        Err(error) => {
            warn!(
                "[NOD] Failed to compute type hash for {}: {}",
                schema.type_name, error
            );
            None
        }
    }
}

pub(crate) fn schema_type_info(schema: &MessageSchema) -> TypeInfo {
    TypeInfo {
        name: schema.type_name.clone(),
        hash: schema_hash(schema),
    }
}

pub(crate) fn schema_type_info_with_hash(
    schema: &MessageSchema,
    discovered_hash: &str,
) -> TypeInfo {
    TypeInfo {
        name: schema.type_name.clone(),
        hash: if discovered_hash == TYPE_HASH_NOT_SUPPORTED {
            None
        } else {
            match TypeHash::from_rihs_string(discovered_hash) {
                Ok(hash) => Some(hash),
                Err(error) => {
                    warn!(
                        "[NOD] Failed to parse discovered type hash for {}: {} ({})",
                        schema.type_name, discovered_hash, error
                    );
                    None
                }
            }
        },
    }
}
