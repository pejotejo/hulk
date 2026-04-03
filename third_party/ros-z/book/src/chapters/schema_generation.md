# Schema Generation And Discovery

ros-z has one internal schema model and two discovery transports.

- The internal model is `MessageSchema`, `FieldSchema`, and `FieldType`.
- The standard transport is ROS 2 `GetTypeDescription`.
- The extended transport is ros-z `GetExtendedTypeDescription`.

This chapter explains how static Rust types become runtime schemas, how basic and extended schemas interact, and how dynamic discovery drives tools such as `rosz echo`.

## The Core Model

Every runtime schema in ros-z uses the same in-memory representation:

- `MessageSchema`: the full message type
- `FieldSchema`: one named field
- `FieldType`: the field shape

`FieldType` is the key boundary between standard-compatible and ros-z-only schemas.

Standard-compatible field kinds:

- primitives
- `Message`
- `Array`
- `Sequence`
- `BoundedSequence`
- `BoundedString`

Extended-only field kinds:

- `Optional`
- `Enum`

That split drives everything else.

- If a `MessageSchema` contains only standard-compatible field kinds, ros-z can expose it through the standard ROS 2 type description service.
- If a `MessageSchema` contains `Optional` or `Enum`, ros-z must expose it through the parallel extended type description service.

## Trait Responsibilities

Schema generation is split across three traits.

### `MessageTypeInfo`

`MessageTypeInfo` is the top-level metadata trait.

It is responsible for:

- DDS type name
- advertised type hash
- an optional standard-compatible `message_schema()`
- a default nested field shape via `field_type()`
- optional registration hooks via `register_type_extensions()`

This trait is used by typed publishers when they advertise a topic and decide whether they can register a schema with the standard type description service.

### `FieldTypeInfo`

`FieldTypeInfo` is the nested field-shape trait.

It is responsible for exactly one thing:

- mapping a Rust field type to a runtime `FieldType`

Most ordinary message types get this automatically through `MessageTypeInfo`.

The main reason `FieldTypeInfo` exists is to support Rust field types that are not standalone ROS messages but still have a well-defined wire shape. The new nalgebra integration uses this hook directly:

- `Point2<f32>` maps to `float32[2]`
- `Point3<f64>` maps to `float64[3]`
- `Isometry3<f32>` maps to a nested message with `rotation` and `translation`

### `ExtendedMessageTypeInfo`

`ExtendedMessageTypeInfo` is the richer schema authoring trait.

It is responsible for:

- producing a full schema even when it contains `Optional` and `Enum`
- computing extended hashes when needed
- registering extended-only schemas with `~get_extended_type_description`

It is not the only schema trait. It sits on top of the same `MessageSchema` model and only becomes observable at runtime when the produced schema actually uses extended field kinds.

## Derive Behavior

ros-z provides two derives.

### `#[derive(MessageTypeInfo)]`

This derive supports standard ROS 2 schema-compatible structs.

It accepts:

- named structs
- primitives
- `String`
- `Vec<T>`
- `[T; N]`
- nested field types that implement `FieldTypeInfo`

It rejects:

- enums
- `Option<T>`
- maps
- tuples
- other non-ROS shapes

For unknown nested fields the derive now calls:

```rust,ignore
<#ty as ::ros_z::FieldTypeInfo>::field_type()
```

This is what allows a standard-compatible message to contain nalgebra field types directly.

### `#[derive(ExtendedMessageTypeInfo)]`

This derive supports:

- named structs
- enums
- `Option<T>`
- nested field types that implement `FieldTypeInfo`

The important rule is that the derive name does not force the discovery path.

Instead, the generated `MessageTypeInfo` impl does this:

- build the full extended schema
- check `schema.uses_extended_types()`
- if false, expose it through normal `message_schema()`
- if true, return `None` from `message_schema()` and register it through the extended service

This means an `ExtendedMessageTypeInfo` type can still use the standard ROS 2 discovery path if the resulting schema is fully standard-compatible.

## Publisher Registration Flow

When you create a typed publisher with:

```rust,ignore
let publisher = node.create_pub::<T>("topic").build()?;
```

ros-z runs the following flow:

1. Read `T::type_info()` for the advertised DDS type name and hash.
2. Call `T::message_schema()`.
3. If a standard-compatible schema exists:
   - register it with the node's `TypeDescriptionService`
   - attach the schema to the publisher builder for dynamic serialization support
4. Call `T::register_type_extensions(node)`.
5. If the type is extended-only, register the full schema with the node's `ExtendedTypeDescriptionService`.

The two services are independent.

- Standard registration uses `~get_type_description`.
- Extended registration uses `~get_extended_type_description`.

## Standard And Extended Discovery

Dynamic subscriber auto-discovery always produces the same result:

- a `MessageSchema`
- a discovered type hash

The only difference is how the schema is fetched.

### Standard Discovery

`create_dyn_sub_auto()` first tries standard discovery.

Flow:

1. Qualify the topic name.
2. Look up current publishers in the graph.
3. Collect candidate publisher nodes plus their advertised type names and hashes.
4. Query each candidate's `~get_type_description` service.
5. Convert the ROS 2 `TypeDescription` wire payload into a `MessageSchema`.
6. Build a dynamic subscriber with that schema.

This path can only succeed when the discovered schema contains no `Optional` or `Enum` fields.

### Extended Discovery

If standard discovery fails, ros-z falls back to the extended service.

Flow:

1. Query each candidate's `~get_extended_type_description` service.
2. Receive a ros-z JSON schema payload.
3. Convert that JSON payload back into the same `MessageSchema` model.
4. Build a dynamic subscriber with that schema.

The runtime subscriber does not care which transport succeeded. After discovery, both paths use the same schema representation.

## Dynamic Runtime Values

Once a schema is known, dynamic decoding uses:

- `DynamicMessage`
- `DynamicValue`

`DynamicMessage` stores:

- the discovered `MessageSchema`
- one `DynamicValue` per field

`DynamicValue` supports:

- primitives
- nested messages
- arrays
- `Optional`
- enums

`rosz echo` uses `create_dyn_sub_auto()` and then renders `DynamicMessage` as JSON. That is why fixing schema generation automatically improves the CLI.

## Mixing Basic And Extended Fields

The key composition rule is now:

- top-level message identity comes from `MessageTypeInfo`
- nested field shape comes from `FieldTypeInfo`

That means an extended message can contain nested fields that only need a basic schema shape.

Examples:

- generated ROS messages can be nested inside extended Rust-native messages
- nalgebra field types can appear inside both standard and extended messages
- only the presence of `Optional` or enums pushes the whole message onto the extended discovery path

This keeps the model simple:

- standard-compatible nested fields stay standard-compatible
- extended-only nested fields remain explicit
- the full message decides which discovery transport is required

## Nalgebra Support

ros-z now includes first-class `FieldTypeInfo` impls for a focused set of nalgebra aliases.

Supported families:

- `Point2`, `Point3`
- `Vector2`, `Vector3`
- `Translation2`, `Translation3`
- `Rotation2`, `Rotation3`
- `UnitComplex`, `UnitQuaternion`
- `Isometry2`, `Isometry3`

for both `f32` and `f64`.

The contract is simple:

- ros-z mirrors serde's wire shape
- not a hand-designed ROS-friendly shape

Examples:

- `Point3<f64>` becomes `float64[3]`
- `UnitComplex<f64>` becomes `float64[2]`
- `Isometry3<f32>` becomes a nested message with:
  - `rotation`
  - `translation`

This is exactly what makes nalgebra-backed transport messages work with dynamic discovery and with `rosz echo`.

## The Hulk Example

The `hulk` binary is now a concrete end-to-end example of the model.

Pipeline:

- `ball_detection` publishes `BallObservation`
- `ball_filter` consumes `BallObservation` and publishes `BallTrack`
- `motion` consumes `BallTrack` and publishes `WalkCommand`

The pipeline intentionally exercises both discovery paths:

- `BallTrack` is standard-compatible and uses normal ROS 2 type description discovery
- `BallObservation` and `WalkCommand` use `Option` or enums and therefore use extended discovery

All three messages use nalgebra field types directly.

That makes `hulk` a good reference for:

- nalgebra-backed transport messages
- mixed standard and extended schemas
- dynamic subscriber auto-discovery
- CLI inspection with `rosz echo`

## Practical Rules

When authoring new message types, use these rules:

1. Use `MessageTypeInfo` when the message only contains standard-compatible field kinds.
2. Use `ExtendedMessageTypeInfo` when the message needs `Option`, enums, or other ros-z-only schema extensions.
3. Add `FieldTypeInfo` impls for reusable field families that are not ordinary top-level ROS messages.
4. Enable `with_type_description_service()` on nodes that publish standard-compatible schemas you want to expose dynamically.
5. Enable `with_extended_type_description_service()` on nodes that publish extended-only schemas.

That is enough for `create_dyn_sub_auto()` and `rosz echo` to discover and decode the topic automatically.
