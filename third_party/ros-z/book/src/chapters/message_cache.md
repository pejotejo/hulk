# Message Cache

**`ZCache<T>` is a timestamp-indexed, capacity-bounded sliding-window cache of received messages.** It mirrors the ROS 2 `message_filters::Cache<T>` API, letting you query a time-ordered history of messages without a background thread or executor.

```admonish note
`ZCache<T>` wraps a Zenoh subscriber and retains the N most recent messages in a `BTreeMap` keyed by timestamp. Queries run in O(log n) using Rust's range API.
```

## Quick Start

```rust,ignore
use ros_z::prelude::*;
use ros_z_msgs::sensor_msgs::Imu;
use std::time::Duration;

let ctx = ZContextBuilder::default().build()?;
let node = ctx.create_node("cache_node").build()?;

// Retain the 200 most recent messages, indexed by Zenoh transport timestamp.
let cache = node.create_cache::<Imu>("/imu/data", 200).build()?;

// Query the last 100 ms.
let now = ZTime::from_wallclock(std::time::SystemTime::now());
let window = cache.get_interval(now - Duration::from_millis(100), now);
println!("Messages in window: {}", window.len());
```

```admonish tip
`use ros_z::Builder;` must be in scope to call `.build()`. It is re-exported by `ros_z::prelude::*`.
```

## Stamp Strategies

Two indexing strategies are available and selected at **compile time** via the type-state builder:

| Strategy | Type | When to use |
|----------|------|-------------|
| **ZenohStamp** (default) | `ZenohStamp` | Zero-config; works for any message type. Indexed by the Zenoh transport timestamp. |
| **ExtractorStamp** | `ExtractorStamp<T, F>` | Sensor fusion: align by logical time (e.g. `header.stamp`). |

### ZenohStamp (default)

No configuration needed. The cache reads the `uhlc::Timestamp` that the Zenoh transport attaches to every published sample and converts it to `ZTime`. If timestamping is disabled on the peer, the cache falls back to the current wallclock time at receive time and logs a one-time warning.

```rust,ignore
use ros_z::prelude::*;
use ros_z_msgs::sensor_msgs::LaserScan;

let cache = node.create_cache::<LaserScan>("/scan", 100).build()?;
```

### ExtractorStamp (application-level)

Supply a closure that extracts a `ZTime` from the deserialized message. Use this when you need messages aligned by their logical capture time rather than network arrival time.

```rust,ignore
use ros_z::prelude::*;
use ros_z_msgs::sensor_msgs::Imu;

let cache = node
    .create_cache::<Imu>("/imu/data", 200)
    .with_stamp(|msg: &Imu| {
        // Read header.stamp from the message.
        let nanos = (msg.header.stamp.sec as i64 * 1_000_000_000)
            + i64::from(msg.header.stamp.nanosec);
        ZTime::from_nanos(nanos)
    })
    .build()?;
```

## Query API

All query methods take and return `ZTime` values and return clones of the stored messages. They are safe to call from multiple threads.

### `get_interval(t_start, t_end) -> Vec<T>`

Returns all messages with timestamp in `[t_start, t_end]`, inclusive, ordered by timestamp ascending. If `t_start > t_end` the result is empty (no panic).

```rust,ignore
let msgs = cache.get_interval(
    ZTime::from_wallclock(std::time::SystemTime::now()) - Duration::from_millis(500),
    ZTime::from_wallclock(std::time::SystemTime::now()),
);
```

### `get_before(t) -> Option<T>`

The most recent message with timestamp ≤ `t`. Returns `None` if the cache is empty or all messages are strictly after `t`.

```rust,ignore
let latest = cache.get_before(ZTime::from_wallclock(std::time::SystemTime::now()));
```

### `get_after(t) -> Option<T>`

The earliest message with timestamp ≥ `t`. Returns `None` if the cache is empty or all messages are strictly before `t`.

```rust,ignore
let next = cache.get_after(camera_stamp);
```

### `get_nearest(t) -> Option<T>`

The message whose timestamp is nearest to `t` (either side). When two messages are equidistant, the one with the earlier timestamp is returned. Returns `None` if the cache is empty.

```rust,ignore
// Align an IMU sample to a camera frame timestamp.
let imu = imu_cache.get_nearest(camera_stamp);
```

### Introspection

```rust,ignore
cache.oldest_stamp()   // Option<ZTime> — timestamp of the oldest entry
cache.newest_stamp()   // Option<ZTime> — timestamp of the newest entry
cache.len()            // usize — number of messages currently stored
cache.is_empty()       // bool
cache.clear()          // remove all messages
```

## Capacity and Eviction

The cache holds at most `capacity` messages. When a new message arrives and the cache is full, the oldest message (smallest timestamp) is evicted first, regardless of insertion order. Capacity can be changed at build time:

```rust,ignore
let cache = node
    .create_cache::<RosString>("/topic", 10)  // initial capacity
    .with_capacity(50)                         // override
    .build()?;
```

## Comparison with `message_filters::Cache<T>`

`ZCache<T>` covers the core `message_filters::Cache<T>` API with some differences:

| Feature | `message_filters::Cache<T>` | `ZCache<T>` |
|---------|----------------------------|-------------|
| Storage | `std::deque` (insertion-sorted) | `BTreeMap` (always sorted, O(log n) range) |
| Stamp strategies | Runtime function pointer (`hasHeader` / receive time) | **Compile-time** type-state (`ZenohStamp` / `ExtractorStamp`) |
| Headerless messages | `allow_headerless` flag, falls back to receive time | `ZenohStamp` handles any type — no header needed |
| `getInterval` | ✅ | ✅ `get_interval` |
| `getElemBeforeTime` | Strict `< t` (exclusive) | `get_before`: inclusive `≤ t` |
| `getElemAfterTime` | Strict `> t` (exclusive) | `get_after`: inclusive `≥ t` |
| `getSurroundingInterval` | ✅ | Not implemented |
| `get_nearest` | Not present | ✅ nearest-neighbor lookup |
| `clear` / `len` / `is_empty` | Not present | ✅ |
| Filter chaining (`signalMessage`) | ✅ (passes through to downstream) | Not applicable (ros-z has no filter chain) |
| Subscriber lifecycle | External, via `connectInput` | Owned by `ZCache`, dropped on `ZCache` drop |

```admonish note
`get_before` and `get_after` use **inclusive** bounds (`≤` / `≥`), unlike the C++ `getElemBeforeTime` / `getElemAfterTime` which are **exclusive** (`<` / `>`). This matches the query semantics of `get_interval`.
```

## Running the Examples

Three focused examples demonstrate the cache. Run them in separate terminals:

```bash
# Terminal 1 — publisher (sends msg-0, msg-1, … every 100 ms)
cargo run --example z_cache_talker

# Terminal 2 — cache consumer with Zenoh transport timestamp (default)
cargo run --example z_cache_zenoh_stamp

# Terminal 2 — cache consumer with application-level timestamp
cargo run --example z_cache_app_stamp
```

## Resources

- **[Pub/Sub](./pubsub.md)** — Publisher and subscriber patterns that `ZCache` builds on
- **[Services](./services.md)** — Request/reply communication
- **API docs** — `cargo doc -p ros-z --open`, then navigate to `ros_z::cache`
