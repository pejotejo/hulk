use clap::{Parser, Subcommand, ValueEnum};

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq, Eq, Default)]
pub enum Backend {
    #[default]
    RmwZenoh,
    #[value(name = "ros2dds")]
    Ros2Dds,
}

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq, Eq)]
pub enum ListTarget {
    Topics,
    Nodes,
    Services,
}

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq, Eq)]
pub enum InfoTarget {
    Topic,
    Service,
    Node,
}

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq, Eq)]
pub enum ParameterValueTypeArg {
    Bool,
    Integer,
    Double,
    String,
    ByteArray,
    BoolArray,
    IntegerArray,
    DoubleArray,
    StringArray,
    NotSet,
}

#[derive(Debug, Parser)]
#[command(name = "rosz")]
#[command(about = "Scriptable command-line companion to ros-z")]
pub struct Cli {
    /// Zenoh router address
    #[arg(long, default_value = "tcp/127.0.0.1:7447", global = true)]
    pub router: String,

    /// ROS domain ID
    #[arg(long, default_value_t = 0, global = true)]
    pub domain: usize,

    /// Backend selection (rmw-zenoh or ros2dds)
    #[arg(long, value_enum, default_value = "rmw-zenoh", global = true)]
    pub backend: Backend,

    /// Emit JSON output when supported
    #[arg(long, global = true)]
    pub json: bool,

    #[command(subcommand)]
    pub command: Command,
}

#[derive(Debug, Subcommand)]
pub enum Command {
    /// List graph entities
    List {
        #[arg(value_enum)]
        target: ListTarget,
    },
    /// Watch graph changes continuously
    Watch,
    /// Show the full graph snapshot
    Graph,
    /// Dynamically inspect a topic's messages
    Echo {
        topic: String,
        #[arg(long)]
        count: Option<usize>,
        #[arg(long)]
        timeout: Option<f64>,
    },
    /// Show metadata for a topic, service, or node
    Info {
        #[arg(value_enum)]
        target: InfoTarget,
        name: String,
    },
    /// Remote parameter operations
    Param {
        #[command(subcommand)]
        command: ParamCommand,
    },
    /// Remote config operations
    Config {
        #[command(subcommand)]
        command: ConfigCommand,
    },
}

#[derive(Debug, Subcommand)]
pub enum ParamCommand {
    /// List parameters on a node
    List {
        #[arg(long)]
        node: String,
        #[arg(long)]
        prefix: Vec<String>,
        #[arg(long)]
        depth: Option<u64>,
    },
    /// Get a parameter from a node
    Get {
        name: String,
        #[arg(long)]
        node: String,
    },
    /// Set a parameter on a node
    Set {
        name: String,
        value: String,
        #[arg(long)]
        node: String,
        #[arg(long = "type", value_enum)]
        value_type: Option<ParameterValueTypeArg>,
        #[arg(long)]
        atomic: bool,
    },
}

#[derive(Debug, Subcommand)]
pub enum ConfigCommand {
    /// Fetch the full effective config snapshot for a node
    Snapshot {
        #[arg(long)]
        node: String,
    },
    /// Fetch one effective config value by path
    Get {
        path: String,
        #[arg(long)]
        node: String,
    },
    /// Set one JSON value at a config path
    Set {
        path: String,
        value: String,
        #[arg(long)]
        node: String,
        #[arg(long)]
        layer: String,
        #[arg(long)]
        expected_revision: Option<u64>,
    },
    /// Reset one layer-local override
    Reset {
        path: String,
        #[arg(long)]
        node: String,
        #[arg(long)]
        layer: String,
        #[arg(long)]
        expected_revision: Option<u64>,
    },
    /// Reload config overlays from disk
    Reload {
        #[arg(long)]
        node: String,
    },
    /// List metadata-backed config paths
    Paths {
        #[arg(long)]
        node: String,
        #[arg(long)]
        prefix: Vec<String>,
        #[arg(long)]
        depth: Option<u64>,
        #[arg(long)]
        writable_only: bool,
    },
    /// Fetch config metadata for one or more paths
    Metadata {
        #[arg(long)]
        node: String,
        paths: Vec<String>,
    },
    /// Watch config change events for a node
    Watch {
        #[arg(long)]
        node: String,
    },
}

#[cfg(test)]
mod tests {
    use clap::Parser;

    use super::{
        Backend, Cli, Command, ConfigCommand, ListTarget, ParamCommand, ParameterValueTypeArg,
    };

    #[test]
    fn parses_echo_command_with_defaults() {
        let cli = Cli::parse_from(["rosz", "echo", "/chatter", "--count", "1"]);

        assert_eq!(cli.router, "tcp/127.0.0.1:7447");
        assert_eq!(cli.domain, 0);
        assert_eq!(cli.backend, Backend::RmwZenoh);
        assert!(!cli.json);

        match cli.command {
            Command::Echo {
                topic,
                count,
                timeout,
            } => {
                assert_eq!(topic, "/chatter");
                assert_eq!(count, Some(1));
                assert_eq!(timeout, None);
            }
            other => panic!("unexpected command: {other:?}"),
        }
    }

    #[test]
    fn parses_global_flags_after_subcommand() {
        let cli = Cli::parse_from([
            "rosz",
            "list",
            "topics",
            "--router",
            "tcp/192.168.1.10:7447",
            "--domain",
            "7",
            "--backend",
            "ros2dds",
            "--json",
        ]);

        assert_eq!(cli.router, "tcp/192.168.1.10:7447");
        assert_eq!(cli.domain, 7);
        assert_eq!(cli.backend, Backend::Ros2Dds);
        assert!(cli.json);

        match cli.command {
            Command::List { target } => assert_eq!(target, ListTarget::Topics),
            other => panic!("unexpected command: {other:?}"),
        }
    }

    #[test]
    fn parses_param_set_with_type_override() {
        let cli = Cli::parse_from([
            "rosz",
            "param",
            "set",
            "max_speed",
            "42",
            "--node",
            "talker",
            "--type",
            "integer",
            "--atomic",
        ]);

        match cli.command {
            Command::Param { command } => match command {
                ParamCommand::Set {
                    name,
                    value,
                    node,
                    value_type,
                    atomic,
                } => {
                    assert_eq!(name, "max_speed");
                    assert_eq!(value, "42");
                    assert_eq!(node, "talker");
                    assert_eq!(value_type, Some(ParameterValueTypeArg::Integer));
                    assert!(atomic);
                }
                other => panic!("unexpected param command: {other:?}"),
            },
            other => panic!("unexpected command: {other:?}"),
        }
    }

    #[test]
    fn parses_config_set_with_layer_and_revision() {
        let cli = Cli::parse_from([
            "rosz",
            "config",
            "set",
            "threshold",
            "0.72",
            "--node",
            "/vision/ball_detector",
            "--layer",
            "/tmp/config/location",
            "--expected-revision",
            "4",
        ]);

        match cli.command {
            Command::Config { command } => match command {
                ConfigCommand::Set {
                    path,
                    value,
                    node,
                    layer,
                    expected_revision,
                } => {
                    assert_eq!(path, "threshold");
                    assert_eq!(value, "0.72");
                    assert_eq!(node, "/vision/ball_detector");
                    assert_eq!(layer, "/tmp/config/location");
                    assert_eq!(expected_revision, Some(4));
                }
                other => panic!("unexpected config command: {other:?}"),
            },
            other => panic!("unexpected command: {other:?}"),
        }
    }

    #[test]
    fn parses_config_paths_with_repeated_prefixes() {
        let cli = Cli::parse_from([
            "rosz",
            "config",
            "paths",
            "--node",
            "/motion/walk_publisher",
            "--prefix",
            "linear",
            "--prefix",
            "publish",
            "--depth",
            "1",
            "--writable-only",
        ]);

        match cli.command {
            Command::Config { command } => match command {
                ConfigCommand::Paths {
                    node,
                    prefix,
                    depth,
                    writable_only,
                } => {
                    assert_eq!(node, "/motion/walk_publisher");
                    assert_eq!(prefix, vec!["linear", "publish"]);
                    assert_eq!(depth, Some(1));
                    assert!(writable_only);
                }
                other => panic!("unexpected config command: {other:?}"),
            },
            other => panic!("unexpected command: {other:?}"),
        }
    }

    #[test]
    fn parses_config_metadata_with_variadic_paths() {
        let cli = Cli::parse_from([
            "rosz",
            "config",
            "metadata",
            "--node",
            "/motion/walk_publisher",
            "linear_x",
            "publish_hz",
        ]);

        match cli.command {
            Command::Config { command } => match command {
                ConfigCommand::Metadata { node, paths } => {
                    assert_eq!(node, "/motion/walk_publisher");
                    assert_eq!(paths, vec!["linear_x", "publish_hz"]);
                }
                other => panic!("unexpected config command: {other:?}"),
            },
            other => panic!("unexpected command: {other:?}"),
        }
    }
}
