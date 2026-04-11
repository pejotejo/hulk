use ros_z::MessageTypeInfo;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, MessageTypeInfo)]
#[ros_msg(type_name = "custom_msgs/msg/ConstGeneric")]
struct ConstGeneric<const N: usize> {
    value: u32,
}

fn main() {}
