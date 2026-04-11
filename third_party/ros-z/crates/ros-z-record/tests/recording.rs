use std::{
    collections::{BTreeMap, HashSet},
    fs,
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    },
    thread,
    time::Duration,
};

use anyhow::{Result, anyhow};
use mcap::{Message, MessageStream, Summary};
use ros_z::{
    Builder,
    context::{ZContext, ZContextBuilder},
    encoding::Encoding,
};
use ros_z_msgs::std_msgs::String as RosString;
use ros_z_record::{RecorderOptions, RecordingHandle, RecordingPlan, RecordingReport};
use tokio::{task::JoinHandle, time::MissedTickBehavior};
use tokio_util::sync::CancellationToken;
use zenoh::{Wait, config::WhatAmI};

static UNIQUE_ID: AtomicUsize = AtomicUsize::new(0);

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[serial_test::serial]
async fn records_single_topic_to_mcap() -> Result<()> {
    let router = TestRouter::new();
    let publisher_ctx = create_context(router.endpoint())?;
    let recorder_ctx = create_context(router.endpoint())?;
    let topic = unique_topic("/record_one");

    let publisher_node = create_node(&publisher_ctx, "publisher_one", true)?;
    let publisher = boxed(publisher_node.create_pub::<RosString>(&topic).build())?;
    let publisher_task = spawn_string_publisher(publisher, "hello");
    tokio::time::sleep(Duration::from_millis(500)).await;

    let recorder_node = create_node(&recorder_ctx, "recorder_one", false)?;
    let tempdir = tempfile::tempdir()?;
    let output = tempdir.path().join("single.mcap");
    let recording = start_recorder(
        Arc::clone(&recorder_node),
        output.clone(),
        vec![topic.clone()],
    )
    .await?;

    tokio::time::sleep(Duration::from_millis(600)).await;
    stop_publisher(publisher_task).await?;
    let report = stop_recording(recording).await?;

    assert!(report.total_messages >= 2);
    assert!(report.silent_topics.is_empty());

    let bytes = fs::read(&output)?;
    let summary = Summary::read(&bytes)?.ok_or_else(|| anyhow!("missing mcap summary"))?;
    let messages = collect_messages(&bytes)?;

    assert_eq!(summary.schemas.len(), 1);
    assert_eq!(summary.channels.len(), 1);
    assert!(!summary.chunk_indexes.is_empty());
    assert!(summary.stats.is_some());
    assert_eq!(
        summary.stats.as_ref().map(|stats| stats.message_count),
        Some(report.total_messages)
    );
    assert_eq!(messages.len() as u64, report.total_messages);
    assert_eq!(messages[0].channel.topic, topic);
    assert_eq!(messages[0].channel.message_encoding, "cdr");
    assert_eq!(
        messages[0]
            .channel
            .schema
            .as_ref()
            .map(|schema| schema.encoding.clone()),
        Some("ros-z/schema+json;v=1".to_string())
    );
    assert!(
        messages[0]
            .channel
            .metadata
            .get("source_id")
            .is_some_and(|source_id| source_id.starts_with("gid:"))
    );
    assert!(
        messages
            .windows(2)
            .all(|pair| pair[1].sequence > pair[0].sequence)
    );
    assert!(messages.iter().all(|message| message.publish_time > 0));
    assert!(messages.iter().all(|message| message.log_time > 0));

    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[serial_test::serial]
async fn prepare_fails_when_schema_is_missing() -> Result<()> {
    let router = TestRouter::new();
    let recorder_ctx = create_context(router.endpoint())?;
    let recorder_node = create_node(&recorder_ctx, "recorder_missing_schema", false)?;
    let tempdir = tempfile::tempdir()?;
    let output = tempdir.path().join("missing_schema.mcap");

    let prepared = RecordingPlan::build(
        recorder_node,
        RecorderOptions {
            output,
            topics: vec![unique_topic("/missing_schema")],
            discovery_timeout: Duration::from_millis(250),
            duration_limit: None,
            stats_interval: Duration::from_secs(1),
            session_metadata: BTreeMap::new(),
        },
    )
    .await;

    assert!(prepared.is_err());
    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[serial_test::serial]
async fn records_two_publishers_as_two_channels() -> Result<()> {
    let router = TestRouter::new();
    let publisher_ctx_one = create_context(router.endpoint())?;
    let publisher_ctx_two = create_context(router.endpoint())?;
    let recorder_ctx = create_context(router.endpoint())?;
    let topic = unique_topic("/record_two_publishers");

    let publisher_node_one = create_node(&publisher_ctx_one, "publisher_one", true)?;
    let publisher_node_two = create_node(&publisher_ctx_two, "publisher_two", true)?;
    let publisher_one = boxed(publisher_node_one.create_pub::<RosString>(&topic).build())?;
    let publisher_two = boxed(publisher_node_two.create_pub::<RosString>(&topic).build())?;
    let publisher_task_one = spawn_string_publisher(publisher_one, "one");
    let publisher_task_two = spawn_string_publisher(publisher_two, "two");
    tokio::time::sleep(Duration::from_millis(500)).await;

    let recorder_node = create_node(&recorder_ctx, "recorder_two_publishers", false)?;
    let tempdir = tempfile::tempdir()?;
    let output = tempdir.path().join("two_publishers.mcap");
    let recording = start_recorder(
        Arc::clone(&recorder_node),
        output.clone(),
        vec![topic.clone()],
    )
    .await?;

    tokio::time::sleep(Duration::from_millis(600)).await;
    stop_publisher(publisher_task_one).await?;
    stop_publisher(publisher_task_two).await?;
    let report = stop_recording(recording).await?;
    assert!(report.total_messages >= 2);

    let bytes = fs::read(&output)?;
    let summary = Summary::read(&bytes)?.ok_or_else(|| anyhow!("missing mcap summary"))?;
    let source_ids: HashSet<_> = summary
        .channels
        .values()
        .map(|channel| channel.metadata.get("source_id").cloned())
        .collect::<Option<Vec<_>>>()
        .ok_or_else(|| anyhow!("missing source_id metadata"))?
        .into_iter()
        .collect();

    assert_eq!(summary.channels.len(), 2);
    assert_eq!(source_ids.len(), 2);

    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[serial_test::serial]
async fn rejects_non_cdr_encoding_during_recording() -> Result<()> {
    let router = TestRouter::new();
    let publisher_ctx = create_context(router.endpoint())?;
    let recorder_ctx = create_context(router.endpoint())?;
    let topic = unique_topic("/record_non_cdr");

    let publisher_node = create_node(&publisher_ctx, "publisher_non_cdr", true)?;
    let publisher = boxed(
        publisher_node
            .create_pub::<RosString>(&topic)
            .with_encoding(Encoding::protobuf().with_schema("std_msgs/msg/String"))
            .build(),
    )?;
    let publisher_task = spawn_string_publisher(publisher, "bad");
    tokio::time::sleep(Duration::from_millis(500)).await;

    let recorder_node = create_node(&recorder_ctx, "recorder_non_cdr", false)?;
    let tempdir = tempfile::tempdir()?;
    let output = tempdir.path().join("non_cdr.mcap");
    let (_shutdown, handle) =
        start_recorder(Arc::clone(&recorder_node), output, vec![topic.clone()]).await?;

    let error = tokio::time::timeout(Duration::from_secs(5), handle.wait())
        .await
        .map_err(|_| anyhow!("recorder did not stop after non-CDR sample"))?
        .expect_err("recording should fail for non-CDR encoding");

    stop_publisher(publisher_task).await?;
    assert!(error.to_string().contains("non-CDR encoding"));
    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[serial_test::serial]
async fn cancellation_does_not_hang_on_silent_topics() -> Result<()> {
    let router = TestRouter::new();
    let publisher_ctx = create_context(router.endpoint())?;
    let recorder_ctx = create_context(router.endpoint())?;
    let topic = unique_topic("/record_silent");

    let publisher_node = create_node(&publisher_ctx, "publisher_silent", true)?;
    let publisher = boxed(publisher_node.create_pub::<RosString>(&topic).build())?;
    let publisher_task = spawn_string_publisher(publisher, "warmup");
    tokio::time::sleep(Duration::from_millis(500)).await;

    let recorder_node = create_node(&recorder_ctx, "recorder_silent", false)?;
    let tempdir = tempfile::tempdir()?;
    let prepared = prepare_recorder(
        Arc::clone(&recorder_node),
        tempdir.path().join("silent.mcap"),
        vec![topic.clone()],
    )
    .await?;
    stop_publisher(publisher_task).await?;

    let shutdown = CancellationToken::new();
    let handle = prepared.spawn(shutdown.clone()).await?;
    let report = stop_recording((shutdown, handle)).await?;

    assert_eq!(report.total_messages, 0);
    assert_eq!(report.silent_topics, vec![topic]);
    Ok(())
}

fn create_node(
    ctx: &ZContext,
    prefix: &str,
    with_type_description_service: bool,
) -> Result<Arc<ros_z::node::ZNode>> {
    let name = unique_name(prefix);
    let builder = ctx.create_node(&name);
    let builder = if with_type_description_service {
        builder.with_type_description_service()
    } else {
        builder
    };

    Ok(Arc::new(boxed(builder.build())?))
}

fn create_context(endpoint: &str) -> Result<ZContext> {
    boxed(
        ZContextBuilder::default()
            .disable_multicast_scouting()
            .with_connect_endpoints([endpoint])
            .with_logging_enabled()
            .build(),
    )
}

fn spawn_string_publisher(
    publisher: ros_z::pubsub::ZPub<RosString>,
    label: &'static str,
) -> (CancellationToken, JoinHandle<Result<()>>) {
    let shutdown = CancellationToken::new();
    let task_shutdown = shutdown.clone();
    let handle = tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(100));
        interval.set_missed_tick_behavior(MissedTickBehavior::Skip);
        let mut index = 0u64;

        loop {
            tokio::select! {
                _ = task_shutdown.cancelled() => break,
                _ = interval.tick() => {
                    boxed(publisher.publish(&RosString {
                        data: format!("{label}-{index}"),
                    }))?;
                    index += 1;
                }
            }
        }

        Ok(())
    });

    (shutdown, handle)
}

async fn stop_publisher(
    (shutdown, handle): (CancellationToken, JoinHandle<Result<()>>),
) -> Result<()> {
    shutdown.cancel();
    handle.await.map_err(|error| anyhow!(error.to_string()))??;
    Ok(())
}

async fn prepare_recorder(
    recorder_node: Arc<ros_z::node::ZNode>,
    output: PathBuf,
    topics: Vec<String>,
) -> Result<RecordingPlan> {
    RecordingPlan::build(
        recorder_node,
        RecorderOptions {
            output,
            topics,
            discovery_timeout: Duration::from_secs(15),
            duration_limit: None,
            stats_interval: Duration::from_secs(1),
            session_metadata: BTreeMap::new(),
        },
    )
    .await
}

async fn start_recorder(
    recorder_node: Arc<ros_z::node::ZNode>,
    output: PathBuf,
    topics: Vec<String>,
) -> Result<(CancellationToken, RecordingHandle)> {
    let prepared = prepare_recorder(recorder_node, output, topics).await?;
    let shutdown = CancellationToken::new();
    let handle = prepared.spawn(shutdown.clone()).await?;
    Ok((shutdown, handle))
}

async fn stop_recording(
    (shutdown, handle): (CancellationToken, RecordingHandle),
) -> Result<RecordingReport> {
    shutdown.cancel();
    tokio::time::timeout(Duration::from_secs(3), handle.wait())
        .await
        .map_err(|_| anyhow!("recorder shutdown timed out"))?
}

fn collect_messages(bytes: &[u8]) -> Result<Vec<Message<'static>>> {
    Ok(MessageStream::new(bytes)?.collect::<std::result::Result<Vec<_>, _>>()?)
}

fn unique_topic(prefix: &str) -> String {
    format!("{}-{}", prefix, UNIQUE_ID.fetch_add(1, Ordering::Relaxed))
}

fn unique_name(prefix: &str) -> String {
    format!("{}_{}", prefix, UNIQUE_ID.fetch_add(1, Ordering::Relaxed))
}

fn boxed<T>(result: std::result::Result<T, Box<dyn std::error::Error + Send + Sync>>) -> Result<T> {
    result.map_err(|error| anyhow!(error.to_string()))
}

struct TestRouter {
    endpoint: String,
    _session: zenoh::Session,
}

impl TestRouter {
    fn new() -> Self {
        for _attempt in 0..5u32 {
            let port = {
                let listener = std::net::TcpListener::bind("127.0.0.1:0")
                    .expect("failed to bind ephemeral port");
                listener.local_addr().expect("listener addr").port()
            };
            let endpoint = format!("tcp/127.0.0.1:{port}");

            let mut config = zenoh::Config::default();
            config.set_mode(Some(WhatAmI::Router)).unwrap();
            config
                .insert_json5("listen/endpoints", &format!("[\"{endpoint}\"]"))
                .unwrap();
            config
                .insert_json5("scouting/multicast/enabled", "false")
                .unwrap();

            if let Ok(session) = zenoh::open(config).wait() {
                thread::sleep(Duration::from_millis(500));
                return Self {
                    endpoint,
                    _session: session,
                };
            }
        }

        panic!("failed to open test router after retries");
    }

    fn endpoint(&self) -> &str {
        &self.endpoint
    }
}
