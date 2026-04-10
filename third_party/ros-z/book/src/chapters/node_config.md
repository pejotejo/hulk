# Node Configuration

`ros-z-config` provides typed, layered node-local configuration for native Rust applications built on top of `ros-z`.

Use it when you want:

- strongly typed config structs in Rust
- runtime reloads and writes
- layered overlays such as `base`, `scene`, `task`, or `robot`
- metadata for UI or remote tooling

This is separate from:

- Zenoh transport configuration in the [Networking](./networking.md) chapters
- ROS 2-style parameters described in [Parameters](./parameters.md)

## Core Model

`ros-z-config` separates two concerns:

1. which config layers are active for this runtime
2. which logical config a node wants

At runtime, the context or node selects an ordered list of directories:

```rust,no_run
# use ros_z::{Builder, context::ZContextBuilder};
# fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
let ctx = ZContextBuilder::default()
    .with_config_layers([
        "./config/base",
        "./config/scenes/match",
        "./config/tasks/striker",
        "./config/robot/robot-01",
    ])
    .build()?;
# Ok(())
# }
```

Each node then binds exactly one explicit config key:

```rust,no_run
# use ros_z::{Builder, context::ZContextBuilder};
# use ros_z_config::prelude::*;
# use serde::{Deserialize, Serialize};
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct BehaviorConfig {
    enabled: bool,
    walk_speed: f64,
}

# fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
# let ctx = ZContextBuilder::default()
#     .with_config_layers(["./config/base"])
#     .build()?;
let node = ctx.create_node("behavior").with_namespace("motion").build()?;
let config = node.bind_config_as::<BehaviorConfig>("behavior")?;
# let _ = config;
# Ok(())
# }
```

For `config_key = "behavior"`, `ros-z-config` reads:

```text
./config/base/behavior.json5
./config/scenes/match/behavior.json5
./config/tasks/striker/behavior.json5
./config/robot/robot-01/behavior.json5
```

Later layers override earlier layers.

There is no second lookup path based on the node FQN.

## Reading Typed Config

The effective merged config is deserialized into your Rust type:

```rust,no_run
# use ros_z::{Builder, context::ZContextBuilder};
# use ros_z_config::prelude::*;
# use serde::{Deserialize, Serialize};
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct VisionConfig {
    enabled: bool,
    threshold: f64,
}

# fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
# let ctx = ZContextBuilder::default()
#     .with_config_layers(["./config/base", "./config/robot/robot-01"])
#     .build()?;
let node = ctx.create_node("ball_detector").with_namespace("vision").build()?;
let config = node.bind_config_as::<VisionConfig>("ball_detector")?;

let snapshot = config.snapshot();
let typed = snapshot.typed();
println!("enabled={} threshold={}", typed.enabled, typed.threshold);
# Ok(())
# }
```

Missing files are treated as empty objects.

## Merge Semantics

Merge rules are simple:

1. every matching file must deserialize to a JSON object
2. later layers override earlier layers recursively
3. object fields merge recursively
4. non-object values replace earlier values at the same path
5. the final merged object is deserialized into your config type

Example:

```json5
// ./config/base/behavior.json5
{
  enabled: true,
  walk: {
    forward: 0.2,
    turn: 0.1,
  }
}
```

```json5
// ./config/robot/robot-01/behavior.json5
{
  walk: {
    forward: 0.3,
  }
}
```

Effective result:

```json5
{
  enabled: true,
  walk: {
    forward: 0.3,
    turn: 0.1,
  }
}
```

## Writing And Resetting Values

Writes target one concrete active layer.

```rust,no_run
# use ros_z::{Builder, context::ZContextBuilder};
# use ros_z_config::prelude::*;
# use serde::{Deserialize, Serialize};
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct WalkConfig {
    linear_x: f64,
}

# fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
# let ctx = ZContextBuilder::default()
#     .with_config_layers(["./config/base", "./config/robot/robot-01"])
#     .build()?;
let node = ctx.create_node("walk_publisher").with_namespace("motion").build()?;
let config = node.bind_config_as::<WalkConfig>("walk_publisher")?;

config.set_json("linear_x", serde_json::json!(0.25), "./config/robot/robot-01")?;
config.reset("linear_x", "./config/robot/robot-01")?;
# Ok(())
# }
```

These operations write to:

```text
./config/robot/robot-01/walk_publisher.json5
```

If a write targets a layer that is not active for the node, the operation fails.

## Watching For Updates

Local consumers can subscribe to the latest committed snapshot:

```rust,no_run
# use ros_z::{Builder, context::ZContextBuilder};
# use ros_z_config::prelude::*;
# use serde::{Deserialize, Serialize};
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct VisionConfig {
    threshold: f64,
}

# async fn demo() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
# let ctx = ZContextBuilder::default()
#     .with_config_layers(["./config/base"])
#     .build()?;
let node = ctx.create_node("ball_detector").with_namespace("vision").build()?;
let config = node.bind_config_as::<VisionConfig>("ball_detector")?;

let mut updates = config.subscribe();
updates.changed().await?;
let snapshot = updates.borrow().clone();
println!("threshold={}", snapshot.typed().threshold);
# Ok(())
# }
```

## Metadata And Introspection

If you need field metadata for UIs or remote tooling, derive `ConfigMetadata` and bind with metadata enabled:

```rust,no_run
# use ros_z::{Builder, context::ZContextBuilder};
# use ros_z_config::prelude::*;
# use serde::{Deserialize, Serialize};
#[derive(Debug, Clone, Serialize, Deserialize, ros_z_config::ConfigMetadata)]
#[serde(deny_unknown_fields)]
struct WalkConfig {
    #[config(doc = "Forward speed", min = -1.0, max = 1.0)]
    linear_x: f64,
}

# fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
# let ctx = ZContextBuilder::default()
#     .with_config_layers(["./config/base"])
#     .build()?;
let node = ctx.create_node("walk_publisher").with_namespace("motion").build()?;
let config = node.bind_config_with_metadata_as::<WalkConfig>("walk_publisher")?;

let paths = config.list_paths()?;
let meta = config.get_metadata("linear_x")?;
assert!(paths.contains(&"linear_x".to_string()));
assert_eq!(meta.min, Some(-1.0));
# Ok(())
# }
```

See also:

- `crates/ros-z-config/examples/metadata.rs`
- `crates/ros-z-config/examples/two_nodes.rs`

## Provenance And `effective_source_layer`

Snapshots and remote responses report `effective_source_layer` for a queried config path.

This means:

- the layer that provided the effective value for that specific path after merging

For leaf paths this is precise.

For object-valued paths this is only a summary of the winning layer for that queried path. Different child fields may still come from different layers.

Example:

- `effective_source_layer("walk.forward")` may be `./config/robot/robot-01`
- `effective_source_layer("walk.turn")` may be `./config/base`

## CLI Workflow

With a running config-enabled node, `ros-z-cli` can inspect and modify config remotely:

```bash
rosz config snapshot --node /motion/walk_publisher
rosz config get linear_x --node /motion/walk_publisher
rosz config metadata --node /motion/walk_publisher linear_x
rosz config set linear_x 0.33 --node /motion/walk_publisher --layer ./config/robot/robot-01
rosz config reset linear_x --node /motion/walk_publisher --layer ./config/robot/robot-01
rosz config reload --node /motion/walk_publisher
```

## Recommended Project Layout

`ros-z-config` does not assign semantic meaning to layer directories. Those conventions belong to your application.

A common robotics layout is:

```text
config/
  base/
    behavior.json5
    walk_publisher.json5
  scenes/
    match/
      behavior.json5
  tasks/
    striker/
      behavior.json5
  robot/
    robot-01/
      behavior.json5
      walk_publisher.json5
```

HULKs-specific concepts such as location, scene, task, and robot should be resolved into ordered layer paths before constructing the `ZContext`.

## Choosing Between Parameters And `ros-z-config`

Use ROS parameters when you want ROS 2-compatible parameter server behavior.

Use `ros-z-config` when you want:

- typed Rust config structs
- recursive layered overlays
- config-key-based lookup instead of node-FQN-based lookup
- richer metadata and operator tooling
