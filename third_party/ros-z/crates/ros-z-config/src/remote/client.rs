use std::{num::NonZeroUsize, sync::Arc};

use ros_z::{
    Builder,
    ServiceTypeInfo,
    msg::{ZMessage, ZService},
    node::ZNode,
    pubsub::ZSub,
    qos::{QosDurability, QosHistory, QosProfile, QosReliability},
    service::ZClient,
};

use crate::{
    ConfigError, ConfigScope,
    remote::types::{
        GetNodeConfigMetadataRequest, GetNodeConfigMetadataResponse,
        GetNodeConfigMetadataSrv, GetNodeConfigSnapshotRequest, GetNodeConfigSnapshotResponse,
        GetNodeConfigSnapshotSrv, GetNodeConfigValueRequest, GetNodeConfigValueResponse,
        GetNodeConfigValueSrv, ListNodeConfigPathsRequest, ListNodeConfigPathsResponse,
        ListNodeConfigPathsSrv, NodeConfigEvent, NodeConfigWriteJson, ReloadNodeConfigRequest,
        ReloadNodeConfigResponse, ReloadNodeConfigSrv, ResetNodeConfigRequest,
        ResetNodeConfigResponse, ResetNodeConfigSrv, SetNodeConfigAtomicallyRequest,
        SetNodeConfigAtomicallyResponse, SetNodeConfigAtomicallySrv, SetNodeConfigRequest,
        SetNodeConfigResponse, SetNodeConfigSrv,
    },
};

const CONFIG_EVENT_HISTORY_DEPTH: usize = 64;

/// Reusable client for the remote node-local config services of one target node.
#[derive(Debug, Clone)]
pub struct RemoteConfigClient {
    node: Arc<ZNode>,
    target_node_fqn: String,
}

impl RemoteConfigClient {
    /// Create a remote config client for `target_node_fqn` using `node` as the
    /// local caller identity.
    pub fn new(node: Arc<ZNode>, target_node_fqn: impl Into<String>) -> crate::Result<Self> {
        let target_node_fqn = target_node_fqn.into();
        if !target_node_fqn.starts_with('/') {
            return Err(ConfigError::RemoteError {
                message: format!(
                    "remote config target must be an absolute node FQN, got '{target_node_fqn}'"
                ),
            });
        }

        Ok(Self {
            node,
            target_node_fqn,
        })
    }

    /// Return the absolute fully qualified target node name.
    pub fn target_node_fqn(&self) -> &str {
        &self.target_node_fqn
    }

    /// Return the absolute service name for `suffix` on the target config API.
    pub fn service_name(&self, suffix: &str) -> String {
        format!("{}/config/{suffix}", self.target_node_fqn)
    }

    /// Return the absolute events topic for the target config API.
    pub fn events_topic(&self) -> String {
        format!("{}/config/events", self.target_node_fqn)
    }

    /// Fetch the full effective config snapshot.
    pub async fn get_snapshot(&self) -> crate::Result<GetNodeConfigSnapshotResponse> {
        self.call_service::<GetNodeConfigSnapshotSrv>(
            &self.service_name("get_snapshot"),
            &GetNodeConfigSnapshotRequest {},
        )
        .await
    }

    /// Fetch one effective config value by path.
    pub async fn get_value(
        &self,
        path: impl Into<String>,
    ) -> crate::Result<GetNodeConfigValueResponse> {
        self.call_service::<GetNodeConfigValueSrv>(
            &self.service_name("get_value"),
            &GetNodeConfigValueRequest { path: path.into() },
        )
        .await
    }

    /// Set one JSON value at `path` in `target_scope`.
    pub async fn set_json(
        &self,
        path: impl Into<String>,
        value: &serde_json::Value,
        target_scope: ConfigScope,
        expected_revision: Option<u64>,
    ) -> crate::Result<SetNodeConfigResponse> {
        self.call_service::<SetNodeConfigSrv>(
            &self.service_name("set"),
            &SetNodeConfigRequest {
                path: path.into(),
                value_json: serialize_json(value)?,
                target_scope,
                expected_revision,
            },
        )
        .await
    }

    /// Apply several JSON writes atomically.
    pub async fn set_json_atomically(
        &self,
        writes: Vec<NodeConfigWriteJson>,
        expected_revision: Option<u64>,
    ) -> crate::Result<SetNodeConfigAtomicallyResponse> {
        self.call_service::<SetNodeConfigAtomicallySrv>(
            &self.service_name("set_atomic"),
            &SetNodeConfigAtomicallyRequest {
                writes,
                expected_revision,
            },
        )
        .await
    }

    /// Reset one scope-local override.
    pub async fn reset(
        &self,
        path: impl Into<String>,
        target_scope: ConfigScope,
        expected_revision: Option<u64>,
    ) -> crate::Result<ResetNodeConfigResponse> {
        self.call_service::<ResetNodeConfigSrv>(
            &self.service_name("reset"),
            &ResetNodeConfigRequest {
                path: path.into(),
                target_scope,
                expected_revision,
            },
        )
        .await
    }

    /// Reload overlays from disk.
    pub async fn reload(&self) -> crate::Result<ReloadNodeConfigResponse> {
        self.call_service::<ReloadNodeConfigSrv>(
            &self.service_name("reload"),
            &ReloadNodeConfigRequest {},
        )
        .await
    }

    /// List metadata-backed field paths.
    pub async fn list_paths(
        &self,
        prefixes: Vec<String>,
        depth: u64,
        writable_only: bool,
    ) -> crate::Result<ListNodeConfigPathsResponse> {
        self.call_service::<ListNodeConfigPathsSrv>(
            &self.service_name("list_paths"),
            &ListNodeConfigPathsRequest {
                prefixes,
                depth,
                writable_only,
            },
        )
        .await
    }

    /// Fetch field metadata for the selected paths, or all paths when empty.
    pub async fn get_metadata(
        &self,
        paths: Vec<String>,
    ) -> crate::Result<GetNodeConfigMetadataResponse> {
        self.call_service::<GetNodeConfigMetadataSrv>(
            &self.service_name("get_metadata"),
            &GetNodeConfigMetadataRequest { paths },
        )
        .await
    }

    /// Subscribe to remote config change events.
    pub fn subscribe_events(&self) -> crate::Result<ZSub<NodeConfigEvent>> {
        self.node
            .create_sub::<NodeConfigEvent>(&self.events_topic())
            .with_qos(QosProfile {
                reliability: QosReliability::Reliable,
                durability: QosDurability::TransientLocal,
                history: QosHistory::KeepLast(
                    NonZeroUsize::new(CONFIG_EVENT_HISTORY_DEPTH).expect("non-zero"),
                ),
                ..Default::default()
            })
            .build()
            .map_err(map_remote_err)
    }

    async fn call_service<S>(
        &self,
        service_name: &str,
        request: &S::Request,
    ) -> crate::Result<S::Response>
    where
        S: ZService + ServiceTypeInfo,
        S::Request: ZMessage,
        S::Response: ZMessage,
        for<'a> <S::Response as ZMessage>::Serdes:
            ros_z::msg::ZDeserializer<Output = S::Response, Input<'a> = &'a [u8]>,
    {
        let client = self.build_client::<S>(service_name)?;
        client.call(request).await.map_err(map_remote_err)
    }

    fn build_client<S>(&self, service_name: &str) -> crate::Result<ZClient<S>>
    where
        S: ZService + ServiceTypeInfo,
    {
        self.node
            .create_client::<S>(service_name)
            .build()
            .map_err(map_remote_err)
    }
}

fn serialize_json(value: &serde_json::Value) -> crate::Result<String> {
    serde_json::to_string(value).map_err(|err| ConfigError::RemoteError {
        message: format!("failed to serialize JSON payload: {err}"),
    })
}

fn map_remote_err<E: std::fmt::Display>(err: E) -> ConfigError {
    ConfigError::RemoteError {
        message: err.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use ros_z::{Builder, context::ZContextBuilder};

    use super::RemoteConfigClient;

    #[test]
    fn rejects_non_absolute_target_fqn() {
        let ctx = ZContextBuilder::default().build().expect("build ctx");
        let node = Arc::new(ctx.create_node("tester").build().expect("build node"));
        let err = RemoteConfigClient::new(node, "vision/ball_detector")
            .expect_err("must reject relative target");
        assert!(err.to_string().contains("absolute node FQN"));
    }

    #[test]
    fn builds_absolute_service_and_event_names() {
        let ctx = ZContextBuilder::default().build().expect("build ctx");
        let node = Arc::new(ctx.create_node("tester").build().expect("build node"));
        let client =
            RemoteConfigClient::new(node, "/vision/ball_detector").expect("build client");

        assert_eq!(
            client.service_name("get_snapshot"),
            "/vision/ball_detector/config/get_snapshot"
        );
        assert_eq!(client.service_name("set"), "/vision/ball_detector/config/set");
        assert_eq!(client.events_topic(), "/vision/ball_detector/config/events");
    }
}
