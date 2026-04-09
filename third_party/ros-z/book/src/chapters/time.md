# Time Model

**ros-z distinguishes wallclock time from logical time, and uses receiver-side terms `transport time` and `source time` when talking about message metadata.**

## Core Terms

- `wallclock time`: time tied to the real host clock.
- `logical time`: time on the modeled timeline for a robot, simulation, or replay.
- `transport time`: the Zenoh transport timestamp observed on the received sample.
- `source time`: the timestamp attached by the publisher from its local `ZClock`.

`transport time` and `source time` are receiver-side names for two timestamps that may be present on the same message.

## Important Relationship

Logical time and wallclock time can be numerically identical on a live robot. They diverge during simulation, replay, or any workflow where the modeled timeline does not advance with the host clock.

That is why ros-z keeps the concepts distinct even when the values happen to match.

## API Mapping

- `ZTime`: a generic instant on a clock timeline.
- `ZClock::wallclock()`: a clock backed by the host wallclock.
- `ZClock::logical(start)`: a manually controlled logical clock.
- `ZContextBuilder::with_clock(...)`: injects the clock used by the context.

`ZTime` is intentionally generic. Convert to or from host time explicitly with `ZTime::from_wallclock(...)` and `ZTime::to_wallclock()` when you need wallclock interop.

## Caution

`ZTime` values from different clock domains are type-compatible but can still be semantically incompatible.

Examples of silent mistakes:

- comparing `ZClock::wallclock().now()` with a `source_time` produced by a logical clock
- querying a transport-time cache with a logical `source_time`

These operations compile, but they can produce meaningless answers if the values come from different timelines.

## Pub-Sub Semantics

Typed and dynamic subscriber metadata APIs return `Received<T>`.

Use:

- `recv_with_metadata()`
- `recv_timeout_with_metadata()`
- `async_recv_with_metadata()`
- `try_recv_with_metadata()` for dynamic subscribers

That wrapper preserves:

- `message`
- `transport_time`
- `source_time`
- `sequence_number`
- `source_gid`

Use `transport_time` when you want receiver-observed ordering, arrival timing, or a wallclock-shaped timeline.

Use `source_time` when you want the publisher's clock timeline, such as simulation-aware processing or aligning streams against producer-side time.

`source_time` is not the same as ROS 2 `header.stamp`.

- `source_time`: infrastructure metadata set by ros-z from the publisher's `ZClock::now()` at publish time
- `header.stamp`: application data carried inside the message and set by user code

## Migration

- `recv()` / `async_recv()` still return the decoded message for source compatibility.
- Use the `*_with_metadata()` variants when you need transport and source timestamps.
- Legacy aliases remain available for the renamed time helpers: `from_system_time`, `to_system_time`, `from_unix_nanos`, `as_unix_nanos`, `ZClock::system()`, and `ZClock::simulated(...)`.
- Cache query methods and `with_stamp(...)` accept both `SystemTime` and `ZTime` through `Into<ZTime>`.
- `ClockKind` and `with_clock_kind(...)` were intentionally removed. Migrate to `with_clock(ZClock::wallclock())` or `with_clock(ZClock::logical(start))`.

## Cache Semantics

`ZCache` now uses `ZTime` for all query and introspection APIs.

- `ZenohStamp`: indexes by transport time.
- `ExtractorStamp`: indexes by logical time you extract from the message.

This keeps cache queries in the same time vocabulary as the rest of ros-z.
