use ros_z::graph::GraphSnapshot;

use crate::{
    model::{
        config::{
            ConfigMetadataFieldView, ConfigMetadataView, ConfigMutationView, ConfigPathsView,
            ConfigSnapshotView, ConfigValueView, ConfigWatchEventView,
        },
        echo::EchoHeader,
        graph::{NodeSummary, ServiceSummary, TopicSummary},
        info::{EndpointSummary, NamedType, NodeInfo, ServiceInfo, TopicInfo},
        param::{ParameterGetView, ParameterListView, ParameterSetView},
        watch::WatchEvent,
    },
    support::nodes::fully_qualified_node_name,
};

use color_eyre::eyre::Result;
use ros_z_record::InspectionReport;

pub fn print_topic_summaries(topics: &[TopicSummary]) {
    let name_width = column_width(topics.iter().map(|topic| topic.name.as_str()));
    let type_width = column_width(topics.iter().map(|topic| topic.type_name.as_str()));

    for topic in topics {
        println!(
            "{:<name_width$}  {:<type_width$}  pubs={} subs={}",
            topic.name, topic.type_name, topic.publishers, topic.subscribers,
        );
    }
}

pub fn print_node_summaries(nodes: &[NodeSummary]) {
    for node in nodes {
        println!("{}", node.fqn);
    }
}

pub fn print_service_summaries(services: &[ServiceSummary]) {
    let name_width = column_width(services.iter().map(|service| service.name.as_str()));
    let type_width = column_width(services.iter().map(|service| service.type_name.as_str()));

    for service in services {
        println!(
            "{:<name_width$}  {:<type_width$}  servers={} clients={}",
            service.name, service.type_name, service.servers, service.clients,
        );
    }
}

pub fn print_inspection_report(report: &InspectionReport) {
    println!("File: {}", report.input.display());
    println!("Profile: {}", report.profile.as_deref().unwrap_or("none"));
    println!("Library: {}", report.library.as_deref().unwrap_or("none"));
    println!(
        "Summary: {}",
        if report.summary_present {
            "present"
        } else {
            "absent"
        }
    );
    println!("Schemas: {}", report.schema_count);
    println!("Channels: {}", report.channel_count);
    println!("Attachments: {}", report.attachment_count);
    println!("Metadata: {}", report.metadata_count);
    println!("Messages: {}", report.message_count);
    println!(
        "Time Range: {} -> {}",
        format_optional_u64(report.message_start_time),
        format_optional_u64(report.message_end_time),
    );

    if let Some(session) = &report.ros_z.session {
        println!();
        println!("ros-z Session ({})", session.len());
        for (key, value) in session {
            println!("{key}: {value}");
        }
    }

    if let Some(requested_topics) = &report.ros_z.requested_topics {
        println!();
        println!("Requested Topics ({})", requested_topics.len());
        for topic in requested_topics {
            println!("{topic}");
        }
    }

    println!();
    println!("Topics ({})", report.topics.len());
    for topic in &report.topics {
        println!(
            "{}  messages={} bytes={} channels={}",
            topic.topic, topic.message_count, topic.byte_count, topic.channel_count,
        );
        if let Some(type_name) = &topic.type_name {
            println!("  type={type_name}");
        }
        if let Some(type_hash) = &topic.type_hash {
            println!("  hash={type_hash}");
        }
        if !topic.source_ids.is_empty() {
            println!("  sources={}", topic.source_ids.join(", "));
        }
    }

    if !report.warnings.is_empty() {
        println!();
        println!("Warnings ({})", report.warnings.len());
        for warning in &report.warnings {
            println!("{warning}");
        }
    }
}

pub fn print_graph_snapshot(snapshot: &GraphSnapshot) {
    println!("Domain {}", snapshot.domain_id);
    println!();

    println!("Topics ({})", snapshot.topics.len());
    let topics: Vec<_> = snapshot
        .topics
        .clone()
        .into_iter()
        .map(TopicSummary::from)
        .collect();
    print_topic_summaries(&topics);
    println!();

    println!("Nodes ({})", snapshot.nodes.len());
    let mut nodes: Vec<_> = snapshot
        .nodes
        .iter()
        .map(|node| NodeSummary::new(node.name.clone(), node.namespace.clone()))
        .collect();
    nodes.sort_by(|left, right| left.fqn.cmp(&right.fqn));
    print_node_summaries(&nodes);
    println!();

    println!("Services ({})", snapshot.services.len());
    let mut services: Vec<_> = snapshot
        .services
        .iter()
        .map(|service| ServiceSummary::new(service.name.clone(), service.type_name.clone(), 0, 0))
        .collect();
    services.sort_by(|left, right| left.name.cmp(&right.name));
    let name_width = column_width(services.iter().map(|service| service.name.as_str()));
    let type_width = column_width(services.iter().map(|service| service.type_name.as_str()));
    for service in services {
        println!(
            "{:<name_width$}  {:<type_width$}",
            service.name, service.type_name,
        );
    }
}

pub fn print_topic_info(info: &TopicInfo) {
    println!("Topic {}", info.name);
    println!("Type: {}", info.type_name);
    println!();
    print_endpoint_section("Publishers", &info.publishers);
    println!();
    print_endpoint_section("Subscribers", &info.subscribers);
}

pub fn print_service_info(info: &ServiceInfo) {
    println!("Service {}", info.name);
    println!("Type: {}", info.type_name);
    println!();
    print_endpoint_section("Servers", &info.servers);
    println!();
    print_endpoint_section("Clients", &info.clients);
}

pub fn print_node_info(info: &NodeInfo) {
    println!("Node {}", info.fqn);
    println!();
    print_named_type_section("Publishers", &info.publishers);
    println!();
    print_named_type_section("Subscribers", &info.subscribers);
    println!();
    print_named_type_section("Services", &info.services);
    println!();
    print_named_type_section("Clients", &info.clients);
    println!();
    print_named_type_section("Action servers", &info.action_servers);
    println!();
    print_named_type_section("Action clients", &info.action_clients);
}

pub fn print_parameter_list(view: &ParameterListView) {
    println!("Node: {}", view.node);
    if view.names.is_empty() {
        println!("Parameters: 0");
        return;
    }

    println!("Parameters ({})", view.names.len());
    for name in &view.names {
        println!("{name}");
    }

    if !view.prefixes.is_empty() {
        println!();
        println!("Prefixes ({})", view.prefixes.len());
        for prefix in &view.prefixes {
            println!("{prefix}");
        }
    }
}

pub fn print_parameter_get(view: &ParameterGetView) {
    println!("Node: {}", view.node);
    println!("Name: {}", view.name);
    println!("Type: {}", view.parameter.value_type);
    println!("Value: {}", view.parameter.display);
}

pub fn print_parameter_set(view: &ParameterSetView) {
    println!("Node: {}", view.node);
    println!("Name: {}", view.name);
    println!("Type: {}", view.parameter.value_type);
    println!("Value: {}", view.parameter.display);
    println!("Atomic: {}", view.atomic);
    println!("Updated: {}", view.successful);
}

pub fn print_config_snapshot(view: &ConfigSnapshotView) -> Result<()> {
    println!("Node: {}", view.node);
    println!("Config Key: {}", view.config_key);
    println!("Revision: {}", view.revision);
    println!(
        "Committed At: {}.{:09}",
        view.committed_at.sec, view.committed_at.nanosec
    );
    println!("Effective:");
    println!("{}", serde_json::to_string_pretty(&view.effective)?);
    for (layer, overlay) in view.layers.iter().zip(&view.layer_overlays) {
        print_overlay_summary(&format!("Layer Overlay [{layer}]"), overlay)?;
    }
    Ok(())
}

pub fn print_config_value(view: &ConfigValueView) -> Result<()> {
    println!("Node: {}", view.node);
    println!("Path: {}", view.path);
    println!("Revision: {}", view.revision);
    println!("Source Layer: {}", view.effective_source_layer);
    println!("Value: {}", serde_json::to_string(&view.value)?);
    Ok(())
}

pub fn print_config_mutation(view: &ConfigMutationView) {
    println!("Node: {}", view.node);
    println!("Operation: {}", view.operation);
    if let Some(path) = &view.path {
        println!("Path: {path}");
    }
    if let Some(layer) = &view.target_layer {
        println!("Target Layer: {layer}");
    }
    println!("Committed Revision: {}", view.committed_revision);
    println!("Successful: {}", view.successful);
    if view.changed_paths.is_empty() {
        println!("Changed Paths: 0");
    } else {
        println!("Changed Paths ({})", view.changed_paths.len());
        for path in &view.changed_paths {
            println!("{path}");
        }
    }
}

pub fn print_config_paths(view: &ConfigPathsView) {
    println!("Node: {}", view.node);
    println!("Revision: {}", view.revision);
    println!("Depth: {}", view.depth);
    println!("Writable Only: {}", view.writable_only);
    if !view.prefixes.is_empty() {
        println!("Prefixes: {}", view.prefixes.join(", "));
    }
    if view.paths.is_empty() {
        println!("Paths: 0");
        return;
    }

    println!("Paths ({})", view.paths.len());
    for path in &view.paths {
        println!("{path}");
    }
}

pub fn print_config_metadata(view: &ConfigMetadataView) {
    println!("Node: {}", view.node);
    println!("Revision: {}", view.revision);
    if !view.requested_paths.is_empty() {
        println!("Requested Paths: {}", view.requested_paths.join(", "));
    }
    if view.metadata.is_empty() {
        println!("Metadata: 0");
        return;
    }

    println!("Metadata ({})", view.metadata.len());
    for field in &view.metadata {
        print_config_metadata_field(field);
    }
}

pub fn print_config_watch_event(view: &ConfigWatchEventView) {
    println!(
        "config {} ({}) rev {} -> {} source={} paths={}",
        view.node,
        view.config_key,
        view.previous_revision,
        view.revision,
        view.source,
        view.changed_paths.join(", ")
    );
}

pub fn print_echo_header(header: &EchoHeader) {
    println!("Topic: {}", header.topic);
    println!("Type: {}", header.type_name);
    println!("Hash: {}", header.type_hash);
    println!();
}

pub fn print_echo_message(message: &str, count: Option<usize>, seen: usize) {
    print!("{message}");
    if count.is_none_or(|limit| seen < limit) {
        println!();
    }
}

pub fn print_watch_event(event: &WatchEvent) {
    match event {
        WatchEvent::InitialState { snapshot } => print_graph_snapshot(snapshot),
        WatchEvent::TopicDiscovered { name, type_name } => {
            println!("topic + {name} ({type_name})");
        }
        WatchEvent::TopicRemoved { name } => {
            println!("topic - {name}");
        }
        WatchEvent::NodeDiscovered { namespace, name } => {
            println!("node + {}", fully_qualified_node_name(namespace, name));
        }
        WatchEvent::NodeRemoved { namespace, name } => {
            println!("node - {}", fully_qualified_node_name(namespace, name));
        }
        WatchEvent::ServiceDiscovered { name, type_name } => {
            println!("service + {name} ({type_name})");
        }
        WatchEvent::ServiceRemoved { name } => {
            println!("service - {name}");
        }
    }
}

fn print_endpoint_section(label: &str, endpoints: &[EndpointSummary]) {
    println!("{label} ({})", endpoints.len());
    if endpoints.is_empty() {
        println!("none");
        return;
    }

    for endpoint in endpoints {
        match (&endpoint.node, &endpoint.type_hash) {
            (Some(node), Some(type_hash)) => println!("{node} [{type_hash}]"),
            (Some(node), None) => println!("{node}"),
            (None, Some(type_hash)) => println!("unknown [{type_hash}]"),
            (None, None) => println!("unknown"),
        }
    }
}

fn print_named_type_section(label: &str, entries: &[NamedType]) {
    println!("{label} ({})", entries.len());
    if entries.is_empty() {
        println!("none");
        return;
    }

    let name_width = column_width(entries.iter().map(|entry| entry.name.as_str()));
    for entry in entries {
        println!("{:<name_width$}  {}", entry.name, entry.type_name);
    }
}

fn format_optional_u64(value: Option<u64>) -> String {
    value
        .map(|value| value.to_string())
        .unwrap_or_else(|| "none".to_string())
}

fn column_width<'a>(values: impl Iterator<Item = &'a str>) -> usize {
    values.map(str::len).max().unwrap_or(0)
}

fn print_config_metadata_field(field: &ConfigMetadataFieldView) {
    println!(
        "{}  type={} writable={} source={}",
        field.path, field.type_name, field.writable, field.effective_source_layer
    );
    if !field.description.is_empty() {
        println!("  doc: {}", field.description);
    }
    if field.min.is_some() || field.max.is_some() {
        println!(
            "  range: {}..{}",
            field
                .min
                .map(|value| value.to_string())
                .unwrap_or_else(|| "-inf".to_string()),
            field
                .max
                .map(|value| value.to_string())
                .unwrap_or_else(|| "+inf".to_string())
        );
    }
}

fn print_overlay_summary(label: &str, value: &serde_json::Value) -> Result<()> {
    let pretty = serde_json::to_string_pretty(value)?;
    let line_count = pretty.lines().count();
    if pretty.len() <= 400 && line_count <= 12 {
        println!("{label}:");
        println!("{pretty}");
    } else {
        println!(
            "{label}: large JSON overlay ({} bytes, {} lines, use --json for full content)",
            pretty.len(),
            line_count
        );
    }
    Ok(())
}
