use std::{
    fs,
    path::{Path, PathBuf},
    time::Duration,
    sync::atomic::{AtomicUsize, Ordering},
};

use ros_z::{Builder, context::ZContextBuilder};
use ros_z_config::{
    ConfigMetadata, ConfigScope, GetNodeConfigMetadataSrv, GetNodeConfigSnapshotSrv,
    GetNodeConfigValueSrv, ListNodeConfigPathsSrv, NodeConfigExt, NodeConfigEvent,
    RemoteConfigClient, SetNodeConfigSrv,
};
use serde::{Deserialize, Serialize};

static NEXT_ID: AtomicUsize = AtomicUsize::new(1);
type TestResult<T = ()> = std::result::Result<T, Box<dyn std::error::Error + Send + Sync>>;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct VisionConfig {
    enabled: bool,
    threshold: f64,
    nested: NestedConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct NestedConfig {
    count: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize, ConfigMetadata)]
#[serde(deny_unknown_fields)]
struct MetadataConfig {
    #[config(doc = "Enable publishing", writable = true)]
    enabled: bool,
    #[config(doc = "Forward speed", min = -1.0, max = 1.0)]
    linear_x: f64,
}

fn temp_config_root() -> PathBuf {
    let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
    let root = std::env::temp_dir().join(format!("ros_z_config_test_{id}"));
    let _ = fs::remove_dir_all(&root);
    fs::create_dir_all(&root).expect("create temp config root");
    root
}

fn write_file(root: &Path, relative: &str, contents: &str) {
    let path = root.join(relative);
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).expect("create parent dirs");
    }
    fs::write(path, contents).expect("write config file");
}

fn build_ctx(root: &Path, suffix: &str) -> ros_z::Result<ros_z::context::ZContext> {
    ZContextBuilder::default()
        .with_domain_id(10_000 + NEXT_ID.fetch_add(1, Ordering::Relaxed))
        .with_mode("peer")
        .disable_multicast_scouting()
        .with_config_root(root)
        .with_location(format!("lab-{suffix}"))
        .with_robot(format!("robot-{suffix}"))
        .build()
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn bind_merge_set_and_subscribe_work() -> TestResult {
    let root = temp_config_root();
    write_file(
        &root,
        "default/vision/ball_detector.json5",
        r#"{
            enabled: true,
            threshold: 0.5,
            nested: { count: 1 }
        }"#,
    );
    write_file(
        &root,
        "location/lab-a/vision/ball_detector.json5",
        r#"{ threshold: 0.8 }"#,
    );

    let ctx = build_ctx(&root, "a")?;
    let node = ctx
        .create_node("ball_detector")
        .with_namespace("vision")
        .build()?;

    let config = node.bind_config::<VisionConfig>()?;
    let snapshot = config.snapshot();
    assert!(snapshot.typed().enabled);
    assert_eq!(snapshot.typed().threshold, 0.8);

    let mut rx = config.subscribe();
    config.set_json("nested.count", serde_json::json!(7), ConfigScope::Robot)?;
    rx.changed().await.expect("watch update");
    let updated = rx.borrow().clone();
    assert_eq!(updated.typed().nested.count, 7);

    let robot_file = fs::read_to_string(root.join("robot/robot-a/vision/ball_detector.json5"))?;
    let reparsed: serde_json::Value = json5::from_str(&robot_file)?;
    assert_eq!(reparsed["nested"]["count"], 7);

    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn second_bind_fails() -> TestResult {
    let root = temp_config_root();
    write_file(
        &root,
        "default/vision/ball_detector.json5",
        r#"{ enabled: true, threshold: 0.5, nested: { count: 1 } }"#,
    );

    let ctx = build_ctx(&root, "b")?;
    let node = ctx
        .create_node("ball_detector")
        .with_namespace("vision")
        .build()?;

    let _config = node.bind_config::<VisionConfig>()?;
    let err = node
        .bind_config::<VisionConfig>()
        .expect_err("second bind must fail");
    assert!(err.to_string().contains("already bound"));
    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn late_validation_hook_validates_current_snapshot() -> TestResult {
    let root = temp_config_root();
    write_file(
        &root,
        "default/vision/ball_detector.json5",
        r#"{ enabled: true, threshold: 2.5, nested: { count: 1 } }"#,
    );

    let ctx = build_ctx(&root, "c")?;
    let node = ctx
        .create_node("ball_detector")
        .with_namespace("vision")
        .build()?;

    let config = node.bind_config::<VisionConfig>()?;
    let err = config
        .add_validation_hook(std::sync::Arc::new(|cfg: &VisionConfig| {
            if cfg.threshold > 1.0 {
                Err("threshold too high".into())
            } else {
                Ok(())
            }
        }))
        .expect_err("late hook must validate current snapshot");
    assert!(err.to_string().contains("threshold too high"));
    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn remote_v1_services_work() -> TestResult {
    let root = temp_config_root();
    write_file(
        &root,
        "default/motion/walk_publisher.json5",
        r#"{ enabled: true, threshold: 0.5, nested: { count: 1 } }"#,
    );

    let ctx = build_ctx(&root, "d")?;
    let server_node = ctx
        .create_node("walk_publisher")
        .with_namespace("motion")
        .build()?;
    let _config = server_node.bind_config::<VisionConfig>()?;

    let client_node = ctx.create_node("tester").with_namespace("tools").build()?;

    let snapshot_client = client_node
        .create_client::<GetNodeConfigSnapshotSrv>("/motion/walk_publisher/config/get_snapshot")
        .build()?;
    let snapshot = snapshot_client.call(&Default::default()).await?;
    assert!(snapshot.success);
    assert!(snapshot.value_json.contains("threshold"));

    let set_client = client_node
        .create_client::<SetNodeConfigSrv>("/motion/walk_publisher/config/set")
        .build()?;
    let set_response = set_client
        .call(&ros_z_config::SetNodeConfigRequest {
            path: "threshold".into(),
            value_json: "0.9".into(),
            target_scope: ConfigScope::Robot,
            expected_revision: None,
        })
        .await?;
    assert!(set_response.success);

    let value_client = client_node
        .create_client::<GetNodeConfigValueSrv>("/motion/walk_publisher/config/get_value")
        .build()?;
    let value_response = value_client
        .call(&ros_z_config::GetNodeConfigValueRequest {
            path: "threshold".into(),
        })
        .await?;
    assert!(value_response.success);
    assert_eq!(value_response.value_json, "0.9");

    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn metadata_local_and_remote_work_when_enabled() -> TestResult {
    let root = temp_config_root();
    write_file(
        &root,
        "default/motion/walk_publisher.json5",
        r#"{ enabled: true, linear_x: 0.2 }"#,
    );

    let ctx = build_ctx(&root, "e")?;
    let server_node = ctx
        .create_node("walk_publisher")
        .with_namespace("motion")
        .build()?;
    let config = server_node.bind_config_with_metadata::<MetadataConfig>()?;

    let paths = config.list_paths()?;
    assert!(paths.contains(&"enabled".to_string()));
    assert!(paths.contains(&"linear_x".to_string()));
    let meta = config.get_metadata("linear_x")?;
    assert_eq!(meta.min, Some(-1.0));
    assert_eq!(meta.max, Some(1.0));

    let client_node = ctx.create_node("tester").with_namespace("tools").build()?;
    let paths_client = client_node
        .create_client::<ListNodeConfigPathsSrv>("/motion/walk_publisher/config/list_paths")
        .build()?;
    let path_response = paths_client.call(&Default::default()).await?;
    assert!(path_response.success);
    assert!(path_response.paths.contains(&"linear_x".to_string()));

    let meta_client = client_node
        .create_client::<GetNodeConfigMetadataSrv>("/motion/walk_publisher/config/get_metadata")
        .build()?;
    let meta_response = meta_client
        .call(&ros_z_config::GetNodeConfigMetadataRequest {
            paths: vec!["linear_x".into()],
        })
        .await?;
    assert!(meta_response.success);
    assert_eq!(meta_response.metadata.len(), 1);
    assert_eq!(meta_response.metadata[0].path, "linear_x");

    let remote_client_node = std::sync::Arc::new(
        ctx.create_node("tester_remote_client")
            .with_namespace("tools")
            .build()?,
    );
    let remote_client = RemoteConfigClient::new(remote_client_node, "/motion/walk_publisher")?;
    let remote_paths = remote_client.list_paths(Vec::new(), 0, false).await?;
    assert!(remote_paths.success);
    assert!(remote_paths.paths.contains(&"linear_x".to_string()));

    let remote_metadata = remote_client.get_metadata(Vec::new()).await?;
    assert!(remote_metadata.success);
    assert!(remote_metadata.metadata.iter().any(|entry| entry.path == "linear_x"));

    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn remote_client_round_trips_and_receives_events() -> TestResult {
    let root = temp_config_root();
    write_file(
        &root,
        "default/motion/walk_publisher.json5",
        r#"{ enabled: true, threshold: 0.5, nested: { count: 1 } }"#,
    );

    let ctx = build_ctx(&root, "remote-client")?;
    let server_node = ctx
        .create_node("walk_publisher")
        .with_namespace("motion")
        .build()?;
    let _config = server_node.bind_config::<VisionConfig>()?;

    let client_node = std::sync::Arc::new(
        ctx.create_node("tester")
            .with_namespace("tools")
            .build()?,
    );
    let client = RemoteConfigClient::new(client_node, "/motion/walk_publisher")?;

    let snapshot = client.get_snapshot().await?;
    assert!(snapshot.success);
    assert!(snapshot.value_json.contains("threshold"));

    let value = client.get_value("threshold").await?;
    assert!(value.success);
    assert_eq!(value.value_json, "0.5");

    let events = client.subscribe_events()?;
    assert!(events.wait_for_publisher(1, Duration::from_secs(5)).await);

    let set = client
        .set_json(
            "threshold",
            &serde_json::json!(0.9),
            ConfigScope::Robot,
            None,
        )
        .await?;
    assert!(set.success);
    assert!(set.changed_paths.contains(&"threshold".to_string()));

    let event: NodeConfigEvent = events.async_recv().await?;
    assert_eq!(event.node_fqn, "/motion/walk_publisher");
    assert!(event.changed_paths.contains(&"threshold".to_string()));

    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn reset_exposes_lower_scope_and_noop_succeeds() -> TestResult {
    let root = temp_config_root();
    write_file(
        &root,
        "default/vision/ball_detector.json5",
        r#"{ enabled: true, threshold: 0.5, nested: { count: 1 } }"#,
    );
    write_file(
        &root,
        "robot/robot-f/vision/ball_detector.json5",
        r#"{ threshold: 0.9 }"#,
    );

    let ctx = build_ctx(&root, "f")?;
    let node = ctx
        .create_node("ball_detector")
        .with_namespace("vision")
        .build()?;
    let config = node.bind_config::<VisionConfig>()?;
    assert_eq!(config.snapshot().typed().threshold, 0.9);

    config.reset("threshold", ConfigScope::Robot)?;
    assert_eq!(config.snapshot().typed().threshold, 0.5);

    config.reset("threshold", ConfigScope::Robot)?;
    assert_eq!(config.snapshot().typed().threshold, 0.5);
    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn reset_rejects_invalid_config() -> TestResult {
    #[derive(Debug, Clone, Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct RequiredOnlyConfig {
        threshold: f64,
    }

    let root = temp_config_root();
    write_file(
        &root,
        "robot/robot-g/vision/ball_detector.json5",
        r#"{ threshold: 0.9 }"#,
    );

    let ctx = build_ctx(&root, "g")?;
    let node = ctx
        .create_node("ball_detector")
        .with_namespace("vision")
        .build()?;
    let config = node.bind_config::<RequiredOnlyConfig>()?;

    let err = config
        .reset("threshold", ConfigScope::Robot)
        .expect_err("reset should fail when it removes required config");
    assert!(
        err.to_string().contains("deserialization") || err.to_string().contains("missing field")
    );
    assert_eq!(config.snapshot().typed().threshold, 0.9);
    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn reload_updates_snapshot_and_preserves_last_good_on_failure() -> TestResult {
    let root = temp_config_root();
    let path = root.join("default/vision/ball_detector.json5");
    write_file(
        &root,
        "default/vision/ball_detector.json5",
        r#"{ enabled: true, threshold: 0.5, nested: { count: 1 } }"#,
    );

    let ctx = build_ctx(&root, "h")?;
    let node = ctx
        .create_node("ball_detector")
        .with_namespace("vision")
        .build()?;
    let config = node.bind_config::<VisionConfig>()?;

    fs::write(
        &path,
        r#"{ enabled: true, threshold: 0.8, nested: { count: 3 } }"#,
    )?;
    config.reload()?;
    assert_eq!(config.snapshot().typed().threshold, 0.8);
    assert_eq!(config.snapshot().typed().nested.count, 3);

    fs::write(&path, r#"{ threshold: "bad" }"#)?;
    let err = config
        .reload()
        .expect_err("reload must reject invalid config");
    assert!(err.to_string().contains("deserialization"));
    assert_eq!(config.snapshot().typed().threshold, 0.8);
    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn revision_mismatch_rejects_local_atomic_write() -> TestResult {
    let root = temp_config_root();
    write_file(
        &root,
        "default/vision/ball_detector.json5",
        r#"{ enabled: true, threshold: 0.5, nested: { count: 1 } }"#,
    );

    let ctx = build_ctx(&root, "i")?;
    let node = ctx
        .create_node("ball_detector")
        .with_namespace("vision")
        .build()?;
    let config = node.bind_config::<VisionConfig>()?;

    let err = config
        .set_json_atomically(
            vec![ros_z_config::ConfigJsonWrite {
                path: "threshold".into(),
                value: serde_json::json!(0.9),
                target_scope: ConfigScope::Robot,
            }],
            Some(999),
        )
        .expect_err("revision mismatch must fail");
    assert!(err.to_string().contains("revision mismatch"));
    assert_eq!(config.snapshot().typed().threshold, 0.5);
    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn local_atomic_write_updates_multiple_paths() -> TestResult {
    let root = temp_config_root();
    write_file(
        &root,
        "default/vision/ball_detector.json5",
        r#"{ enabled: true, threshold: 0.5, nested: { count: 1 } }"#,
    );

    let ctx = build_ctx(&root, "j")?;
    let node = ctx
        .create_node("ball_detector")
        .with_namespace("vision")
        .build()?;
    let config = node.bind_config::<VisionConfig>()?;
    let revision = config.snapshot().revision;

    config.set_json_atomically(
        vec![
            ros_z_config::ConfigJsonWrite {
                path: "threshold".into(),
                value: serde_json::json!(0.9),
                target_scope: ConfigScope::Robot,
            },
            ros_z_config::ConfigJsonWrite {
                path: "nested.count".into(),
                value: serde_json::json!(42),
                target_scope: ConfigScope::Robot,
            },
        ],
        Some(revision),
    )?;

    let snapshot = config.snapshot();
    assert_eq!(snapshot.typed().threshold, 0.9);
    assert_eq!(snapshot.typed().nested.count, 42);
    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn concurrent_readers_and_writers_do_not_lose_updates() -> TestResult {
    let root = temp_config_root();
    write_file(
        &root,
        "default/vision/ball_detector.json5",
        r#"{ enabled: true, threshold: 0.5, nested: { count: 0 } }"#,
    );

    let ctx = build_ctx(&root, "k")?;
    let node = ctx
        .create_node("ball_detector")
        .with_namespace("vision")
        .build()?;
    let config = node.bind_config::<VisionConfig>()?;

    let writer_a = {
        let config = config.clone();
        tokio::spawn(async move {
            for value in 1..=10u32 {
                config.set_json("nested.count", serde_json::json!(value), ConfigScope::Robot)?;
            }
            Ok::<_, ros_z_config::ConfigError>(())
        })
    };

    let writer_b = {
        let config = config.clone();
        tokio::spawn(async move {
            for value in 1..=10u32 {
                config.set_json(
                    "threshold",
                    serde_json::json!(0.5 + (value as f64 / 10.0)),
                    ConfigScope::Robot,
                )?;
            }
            Ok::<_, ros_z_config::ConfigError>(())
        })
    };

    let reader = {
        let config = config.clone();
        tokio::spawn(async move {
            for _ in 0..100 {
                let snapshot = config.snapshot();
                let _ = snapshot.typed().threshold;
                let _ = snapshot.typed().nested.count;
                tokio::task::yield_now().await;
            }
        })
    };

    writer_a.await??;
    writer_b.await??;
    reader.await?;

    let snapshot = config.snapshot();
    assert_eq!(snapshot.typed().nested.count, 10);
    assert_eq!(snapshot.typed().threshold, 1.5);
    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn persistence_round_trip_pretty_json_is_valid_json5() -> TestResult {
    let root = temp_config_root();
    write_file(
        &root,
        "default/vision/ball_detector.json5",
        r#"{ enabled: true, threshold: 0.5, nested: { count: 1 } }"#,
    );

    let ctx = build_ctx(&root, "l")?;
    let node = ctx
        .create_node("ball_detector")
        .with_namespace("vision")
        .build()?;
    let config = node.bind_config::<VisionConfig>()?;

    config.set_json("threshold", serde_json::json!(0.75), ConfigScope::Robot)?;
    let written = fs::read_to_string(root.join("robot/robot-l/vision/ball_detector.json5"))?;
    let reparsed: serde_json::Value = json5::from_str(&written)?;
    assert_eq!(reparsed["threshold"], 0.75);
    Ok(())
}
