use std::{sync::Arc, time::Duration};

use crate::{
    Builder, dynamic::DynamicError, node::ZNode, topic_name::qualify_remote_private_service_name,
};

use crate::dynamic::{MessageSchema, discovery::TopicSchemaCandidate};
use crate::extended_type_description_service::{
    GetExtendedTypeDescription, GetExtendedTypeDescriptionRequest,
    GetExtendedTypeDescriptionResponse,
};

use crate::extended_schema::schema_from_extension_json;

/// Query the extended type-description service for a single current topic candidate.
pub(crate) async fn query_extended_type_description(
    node: &ZNode,
    candidate: &TopicSchemaCandidate,
    timeout: Duration,
) -> Result<(Arc<MessageSchema>, String), DynamicError> {
    let service_name = qualify_remote_private_service_name(
        "get_extended_type_description",
        &candidate.namespace,
        &candidate.node_name,
    )
    .map_err(|e| DynamicError::SerializationError(e.to_string()))?;
    let node_fqn =
        qualify_remote_private_service_name("", &candidate.namespace, &candidate.node_name)
            .map_err(|e| DynamicError::SerializationError(e.to_string()))?;

    let client = node
        .create_client::<GetExtendedTypeDescription>(&service_name)
        .build()
        .map_err(|e| DynamicError::SerializationError(e.to_string()))?;
    let request = GetExtendedTypeDescriptionRequest {
        type_name: candidate.type_name.clone(),
        type_hash: candidate.type_hash.clone(),
    };

    let response = client
        .call_or_timeout(&request, timeout)
        .await
        .map_err(|_| DynamicError::ServiceTimeout {
            node: node_fqn,
            service: service_name,
        })?;

    let schema = schema_from_extended_type_description_response(&response)?;
    Ok((schema, response.type_hash))
}

pub fn schema_from_extended_type_description_response(
    response: &GetExtendedTypeDescriptionResponse,
) -> Result<Arc<MessageSchema>, DynamicError> {
    if !response.successful {
        return Err(DynamicError::SerializationError(format!(
            "Response indicates failure: {}",
            response.failure_reason
        )));
    }

    schema_from_extension_json(&response.schema_json)
}
