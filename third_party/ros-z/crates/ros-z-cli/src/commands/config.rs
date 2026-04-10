use std::time::Duration;

use color_eyre::eyre::{Result, bail, eyre};
use ros_z_config::{
    GetNodeConfigMetadataResponse, GetNodeConfigSnapshotResponse, GetNodeConfigValueResponse,
    ListNodeConfigPathsResponse, ReloadNodeConfigResponse, ResetNodeConfigResponse,
    SetNodeConfigResponse,
};

use crate::{
    app::AppContext,
    cli::ConfigCommand,
    model::config::{
        ConfigMetadataView, ConfigMutationView, ConfigPathsView, ConfigSnapshotView,
        ConfigValueView, ConfigWatchEventView,
    },
    render::{OutputMode, json, text},
    support::config::{
        can_resolve_config_node_fqn, parse_config_json, resolve_config_node_fqn,
        verify_config_capability, verify_config_metadata_capability,
    },
};

const WATCH_MATCH_TIMEOUT: Duration = Duration::from_secs(5);

pub async fn run(app: &AppContext, output_mode: OutputMode, command: ConfigCommand) -> Result<()> {
    match command {
        ConfigCommand::Snapshot { node } => render_snapshot(app, output_mode, &node).await,
        ConfigCommand::Get { path, node } => render_get(app, output_mode, &node, &path).await,
        ConfigCommand::Set {
            path,
            value,
            node,
            layer,
            expected_revision,
        } => {
            render_set(
                app,
                output_mode,
                &node,
                &path,
                &value,
                &layer,
                expected_revision,
            )
            .await
        }
        ConfigCommand::Reset {
            path,
            node,
            layer,
            expected_revision,
        } => render_reset(app, output_mode, &node, &path, &layer, expected_revision).await,
        ConfigCommand::Reload { node } => render_reload(app, output_mode, &node).await,
        ConfigCommand::Paths {
            node,
            prefix,
            depth,
            writable_only,
        } => {
            render_paths(
                app,
                output_mode,
                &node,
                prefix,
                depth.unwrap_or(0),
                writable_only,
            )
            .await
        }
        ConfigCommand::Metadata { node, paths } => {
            render_metadata(app, output_mode, &node, paths).await
        }
        ConfigCommand::Watch { node } => render_watch(app, output_mode, &node).await,
    }
}

async fn render_snapshot(app: &AppContext, output_mode: OutputMode, selector: &str) -> Result<()> {
    let (node_fqn, client) = resolve_client(app, selector).await?;
    let response = client.get_snapshot().await?;
    ensure_success(&node_fqn, "get config snapshot", &response)?;
    let view = ConfigSnapshotView::from_response(response)?;

    match output_mode {
        OutputMode::Json => json::print_pretty(&view),
        OutputMode::Text => {
            text::print_config_snapshot(&view)?;
            Ok(())
        }
    }
}

async fn render_get(
    app: &AppContext,
    output_mode: OutputMode,
    selector: &str,
    path: &str,
) -> Result<()> {
    let (node_fqn, client) = resolve_client(app, selector).await?;
    let response = client.get_value(path).await?;
    ensure_success(&node_fqn, &format!("get config value at {path}"), &response)?;
    let view = ConfigValueView::from_response(node_fqn, response)?;

    match output_mode {
        OutputMode::Json => json::print_pretty(&view),
        OutputMode::Text => {
            text::print_config_value(&view)?;
            Ok(())
        }
    }
}

async fn render_set(
    app: &AppContext,
    output_mode: OutputMode,
    selector: &str,
    path: &str,
    value: &str,
    layer: &str,
    expected_revision: Option<u64>,
) -> Result<()> {
    let (node_fqn, client) = resolve_client(app, selector).await?;
    let parsed = parse_config_json(value)?;
    let response = client
        .set_json(path, &parsed, layer.to_string(), expected_revision)
        .await?;
    ensure_success(&node_fqn, &format!("set config value at {path}"), &response)?;
    let view = ConfigMutationView::new(
        node_fqn,
        "set",
        Some(path.to_string()),
        Some(layer.to_string()),
        response.committed_revision,
        response.changed_paths,
        true,
    );

    match output_mode {
        OutputMode::Json => json::print_pretty(&view),
        OutputMode::Text => {
            text::print_config_mutation(&view);
            Ok(())
        }
    }
}

async fn render_reset(
    app: &AppContext,
    output_mode: OutputMode,
    selector: &str,
    path: &str,
    layer: &str,
    expected_revision: Option<u64>,
) -> Result<()> {
    let (node_fqn, client) = resolve_client(app, selector).await?;
    let response = client
        .reset(path, layer.to_string(), expected_revision)
        .await?;
    ensure_success(
        &node_fqn,
        &format!("reset config value at {path}"),
        &response,
    )?;
    let view = ConfigMutationView::new(
        node_fqn,
        "reset",
        Some(path.to_string()),
        Some(layer.to_string()),
        response.committed_revision,
        response.changed_paths,
        true,
    );

    match output_mode {
        OutputMode::Json => json::print_pretty(&view),
        OutputMode::Text => {
            text::print_config_mutation(&view);
            Ok(())
        }
    }
}

async fn render_reload(app: &AppContext, output_mode: OutputMode, selector: &str) -> Result<()> {
    let (node_fqn, client) = resolve_client(app, selector).await?;
    let response = client.reload().await?;
    ensure_success(&node_fqn, "reload config overlays", &response)?;
    let view = ConfigMutationView::new(
        node_fqn,
        "reload",
        None,
        None,
        response.committed_revision,
        response.changed_paths,
        true,
    );

    match output_mode {
        OutputMode::Json => json::print_pretty(&view),
        OutputMode::Text => {
            text::print_config_mutation(&view);
            Ok(())
        }
    }
}

async fn render_paths(
    app: &AppContext,
    output_mode: OutputMode,
    selector: &str,
    prefixes: Vec<String>,
    depth: u64,
    writable_only: bool,
) -> Result<()> {
    let (node_fqn, client) = resolve_metadata_client(app, selector).await?;
    let response = client
        .list_paths(prefixes.clone(), depth, writable_only)
        .await?;
    ensure_success(&node_fqn, "list config paths", &response)?;
    let view = ConfigPathsView::new(
        node_fqn,
        response.revision,
        prefixes,
        depth,
        writable_only,
        response.paths,
    );

    match output_mode {
        OutputMode::Json => json::print_pretty(&view),
        OutputMode::Text => {
            text::print_config_paths(&view);
            Ok(())
        }
    }
}

async fn render_metadata(
    app: &AppContext,
    output_mode: OutputMode,
    selector: &str,
    paths: Vec<String>,
) -> Result<()> {
    let (node_fqn, client) = resolve_metadata_client(app, selector).await?;
    let response = client.get_metadata(paths.clone()).await?;
    ensure_success(&node_fqn, "get config metadata", &response)?;
    let view = ConfigMetadataView::from_response(node_fqn, paths, response);

    match output_mode {
        OutputMode::Json => json::print_pretty(&view),
        OutputMode::Text => {
            text::print_config_metadata(&view);
            Ok(())
        }
    }
}

async fn render_watch(app: &AppContext, output_mode: OutputMode, selector: &str) -> Result<()> {
    let (_node_fqn, client) = resolve_client(app, selector).await?;
    let subscriber = client.subscribe_events()?;
    let _ = subscriber.wait_for_publisher(1, WATCH_MATCH_TIMEOUT).await;

    loop {
        let event = subscriber
            .async_recv()
            .await
            .map_err(|error| eyre!(error.to_string()))?;
        let view = ConfigWatchEventView::from_event(event)?;
        match output_mode {
            OutputMode::Json => json::print_line(&view)?,
            OutputMode::Text => text::print_config_watch_event(&view),
        }
    }
}

async fn resolve_client(
    app: &AppContext,
    selector: &str,
) -> Result<(String, ros_z_config::RemoteConfigClient)> {
    app.wait_for_graph_settle().await;
    app.wait_for_graph_condition(|graph| can_resolve_config_node_fqn(graph, selector))
        .await;
    let node_fqn = resolve_config_node_fqn(app.graph(), selector)?;
    verify_config_capability(app.graph(), &node_fqn)?;
    let client = app.config_client(&node_fqn)?;
    Ok((node_fqn, client))
}

async fn resolve_metadata_client(
    app: &AppContext,
    selector: &str,
) -> Result<(String, ros_z_config::RemoteConfigClient)> {
    let (node_fqn, client) = resolve_client(app, selector).await?;
    verify_config_metadata_capability(app.graph(), &node_fqn)?;
    Ok((node_fqn, client))
}

fn ensure_success<T>(node_fqn: &str, action: &str, response: &T) -> Result<()>
where
    T: ConfigServiceResponse,
{
    if response.success() {
        return Ok(());
    }

    bail!("{action} failed for {node_fqn}: {}", response.message())
}

trait ConfigServiceResponse {
    fn success(&self) -> bool;
    fn message(&self) -> &str;
}

impl ConfigServiceResponse for GetNodeConfigSnapshotResponse {
    fn success(&self) -> bool {
        self.success
    }

    fn message(&self) -> &str {
        &self.message
    }
}

impl ConfigServiceResponse for GetNodeConfigValueResponse {
    fn success(&self) -> bool {
        self.success
    }

    fn message(&self) -> &str {
        &self.message
    }
}

impl ConfigServiceResponse for SetNodeConfigResponse {
    fn success(&self) -> bool {
        self.success
    }

    fn message(&self) -> &str {
        &self.message
    }
}

impl ConfigServiceResponse for ResetNodeConfigResponse {
    fn success(&self) -> bool {
        self.success
    }

    fn message(&self) -> &str {
        &self.message
    }
}

impl ConfigServiceResponse for ReloadNodeConfigResponse {
    fn success(&self) -> bool {
        self.success
    }

    fn message(&self) -> &str {
        &self.message
    }
}

impl ConfigServiceResponse for ListNodeConfigPathsResponse {
    fn success(&self) -> bool {
        self.success
    }

    fn message(&self) -> &str {
        &self.message
    }
}

impl ConfigServiceResponse for GetNodeConfigMetadataResponse {
    fn success(&self) -> bool {
        self.success
    }

    fn message(&self) -> &str {
        &self.message
    }
}
