mod common;

use std::sync::{
    Arc,
    atomic::{AtomicUsize, Ordering},
};
use std::time::Duration;

use common::*;
use ros_z::{Builder, MessageTypeInfo};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, MessageTypeInfo)]
#[ros_msg(type_name = "test_msgs/msg/DebugValue")]
struct DebugValue {
    value: u32,
}

impl ros_z::msg::ZMessage for DebugValue {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}

#[test]
fn publisher_and_subscriber_expose_match_counts() {
    let router = TestRouter::new();
    let pub_ctx = create_ros_z_context_with_router(&router).expect("pub ctx");
    let sub_ctx = create_ros_z_context_with_router(&router).expect("sub ctx");

    let pub_node = pub_ctx.create_node("pub_node").build().expect("pub node");
    let sub_node = sub_ctx.create_node("sub_node").build().expect("sub node");

    let publisher = pub_node
        .create_pub::<DebugValue>("/debug/matching")
        .build()
        .expect("publisher");
    let subscriber = sub_node
        .create_sub::<DebugValue>("/debug/matching")
        .build()
        .expect("subscriber");

    let rt = tokio::runtime::Runtime::new().expect("runtime");
    rt.block_on(async {
        assert!(publisher.wait_for_subscription(1, Duration::from_secs(5)).await);
        assert!(subscriber.wait_for_publisher(1, Duration::from_secs(5)).await);
    });

    assert!(publisher.has_subscribers());
    assert!(subscriber.has_publishers());
    assert_eq!(publisher.subscriber_count(), 1);
    assert_eq!(subscriber.publisher_count(), 1);
}

#[test]
fn publish_if_subscribed_only_builds_when_needed() {
    let router = TestRouter::new();
    let pub_ctx = create_ros_z_context_with_router(&router).expect("pub ctx");
    let sub_ctx = create_ros_z_context_with_router(&router).expect("sub ctx");

    let pub_node = pub_ctx.create_node("pub_node").build().expect("pub node");
    let publisher = pub_node
        .create_pub::<DebugValue>("/debug/lazy")
        .build()
        .expect("publisher");

    let calls = Arc::new(AtomicUsize::new(0));
    let result = publisher
        .publish_if_subscribed({
            let calls = calls.clone();
            move || {
                calls.fetch_add(1, Ordering::SeqCst);
                DebugValue { value: 7 }
            }
        })
        .expect("publish_if_subscribed without subscribers");
    assert!(!result);
    assert_eq!(calls.load(Ordering::SeqCst), 0);

    let sub_node = sub_ctx.create_node("sub_node").build().expect("sub node");
    let subscriber = sub_node
        .create_sub::<DebugValue>("/debug/lazy")
        .build()
        .expect("subscriber");

    let rt = tokio::runtime::Runtime::new().expect("runtime");
    rt.block_on(async {
        assert!(publisher.wait_for_subscription(1, Duration::from_secs(5)).await);
        assert!(subscriber.wait_for_publisher(1, Duration::from_secs(5)).await);
    });

    let result = publisher
        .publish_if_subscribed({
            let calls = calls.clone();
            move || {
                calls.fetch_add(1, Ordering::SeqCst);
                DebugValue { value: 42 }
            }
        })
        .expect("publish_if_subscribed with subscribers");
    assert!(result);
    assert_eq!(calls.load(Ordering::SeqCst), 1);

    let received = subscriber
        .recv_timeout(Duration::from_secs(5))
        .expect("receive lazy message");
    assert_eq!(received, DebugValue { value: 42 });
}
