use ros_z::ExtendedMessageTypeInfo;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, ExtendedMessageTypeInfo)]
#[ros_msg(type_name = "custom_msgs/msg/GenericEnum")]
enum GenericEnum<T> {
    Value(T),
}

fn main() {}
