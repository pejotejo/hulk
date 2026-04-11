use tracing::warn;

use crate::dynamic::{MessageSchema, MessageSchemaTypeDescription};
use crate::entity::{TypeHash, TypeInfo};
use crate::ros_msg::dds_type_name_to_canonical;

pub(crate) fn ros_type_name_from_dds(dds_name: &str) -> String {
    dds_type_name_to_canonical(dds_name)
}

pub(crate) fn schema_hash(schema: &MessageSchema) -> TypeHash {
    match schema.compute_type_hash() {
        Ok(hash) => {
            let rihs_string = hash.to_rihs_string();
            TypeHash::from_rihs_string(&rihs_string).unwrap_or_else(TypeHash::zero)
        }
        Err(error) => {
            warn!(
                "[NOD] Failed to compute type hash for {}: {}",
                schema.type_name, error
            );
            TypeHash::zero()
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
        hash: TypeHash::from_rihs_string(discovered_hash).unwrap_or_else(TypeHash::zero),
    }
}
