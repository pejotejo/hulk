use std::{
    collections::{BTreeMap, HashMap, HashSet},
    fs::File,
    io::{BufWriter, Seek, Write},
    num::NonZeroUsize,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use anyhow::{Context, Result, anyhow};
use flume::{Receiver, RecvTimeoutError};
use mcap::{
    Compression, WriteOptions, Writer,
    records::{MessageHeader, system_time_to_nanos},
    write::Metadata,
};
use ros_z::{
    Builder,
    attachment::Attachment,
    dynamic::DynSub,
    encoding::Encoding,
    qos::{QosHistory, QosProfile},
};
use tokio::{sync::watch, task::JoinSet};
use tokio_util::sync::CancellationToken;
use tracing::warn;
use zenoh::sample::Sample;

use crate::api::{RecordingReport, RecordingStartup, StatsSnapshot, TopicStats};
use crate::{PreparedRecording, RecordingHandle, TopicPlan};

const WRITER_QUEUE_CAPACITY: usize = 65_536;
const INTERNAL_STATS_TICK: Duration = Duration::from_millis(100);
const SCHEMA_ENCODING: &str = "ros-z/schema+json;v=1";
const MESSAGE_ENCODING: &str = "cdr";

#[derive(Debug, Clone, Hash, PartialEq, Eq)]
enum SourceKey {
    NoAttachment,
    Gid([u8; 16]),
}

#[derive(Debug)]
struct SampleEnvelope {
    topic_index: usize,
    payload: Vec<u8>,
    transport_time_ns: u64,
    source_time_ns: Option<u64>,
    sequence: u32,
    source: SourceKey,
    zenoh_encoding: String,
}

#[derive(Debug, Clone, Hash, PartialEq, Eq)]
struct ChannelKey {
    topic_index: usize,
    source: SourceKey,
    zenoh_encoding: String,
}

struct TopicState {
    requested_topic: String,
    qualified_topic: String,
    type_name: String,
    type_hash: String,
    startup_publisher_count: usize,
    schema_id: u16,
    messages: u64,
    bytes: u64,
}

struct McapRecorder<W: Write + Seek> {
    writer: Writer<W>,
    started_at: SystemTime,
    topics: Vec<TopicState>,
    channels: HashMap<ChannelKey, u16>,
    total_messages: u64,
    total_bytes: u64,
}

pub async fn spawn(
    prepared: PreparedRecording,
    shutdown: CancellationToken,
) -> Result<RecordingHandle> {
    let output_path = prepared.options.output.clone();

    match spawn_inner(prepared, shutdown).await {
        Ok(handle) => Ok(handle),
        Err(error) => {
            cleanup_partial_output(&output_path).await;
            Err(error)
        }
    }
}

async fn spawn_inner(
    prepared: PreparedRecording,
    shutdown: CancellationToken,
) -> Result<RecordingHandle> {
    let subscribers = create_subscribers(&prepared)?;
    let output_file = create_output_file(&prepared)?;
    let recorder = McapRecorder::new(
        BufWriter::new(output_file),
        &prepared.startup,
        &prepared.topics,
        &prepared.options.session_metadata,
        prepared.options.discovery_timeout,
        prepared.options.duration_limit,
    )?;

    let (sender, receiver) = flume::bounded(WRITER_QUEUE_CAPACITY);
    let (stats_tx, stats_rx) = watch::channel(initial_stats(&prepared.topics));
    let writer_shutdown = shutdown.clone();
    let writer_task = tokio::task::spawn_blocking(move || {
        let result = recorder.run(receiver, stats_tx);
        if result.is_err() {
            writer_shutdown.cancel();
        }
        result
    });

    let mut receive_tasks = JoinSet::new();
    for (topic_index, (topic, subscriber)) in
        prepared.topics.into_iter().zip(subscribers).enumerate()
    {
        let sender = sender.clone();
        let shutdown = shutdown.clone();
        receive_tasks.spawn(async move {
            receive_topic(
                topic_index,
                topic.startup.qualified_topic,
                subscriber,
                sender,
                shutdown,
            )
            .await
        });
    }
    drop(sender);

    let join_handle = tokio::spawn(async move {
        let mut first_error = None;

        while let Some(result) = receive_tasks.join_next().await {
            match result {
                Ok(Ok(())) => {}
                Ok(Err(error)) => {
                    shutdown.cancel();
                    if first_error.is_none() {
                        first_error = Some(error);
                    }
                }
                Err(error) => {
                    shutdown.cancel();
                    if first_error.is_none() {
                        first_error = Some(anyhow!("receive task panicked: {error}"));
                    }
                }
            }
        }

        let report = writer_task.await.context("writer task panicked")??;

        if let Some(error) = first_error {
            Err(error)
        } else {
            Ok(report)
        }
    });

    Ok(RecordingHandle::new(stats_rx, join_handle))
}

fn create_subscribers(prepared: &PreparedRecording) -> Result<Vec<DynSub>> {
    prepared
        .topics
        .iter()
        .map(|topic| {
            prepared
                .node
                .create_dyn_sub(&topic.startup.qualified_topic, topic.schema.clone())
                .with_qos(recorder_qos())
                .build()
                .map_err(|error| anyhow!(error.to_string()))
                .with_context(|| {
                    format!(
                        "failed to create recorder subscriber for {}",
                        topic.startup.qualified_topic
                    )
                })
        })
        .collect()
}

fn create_output_file(prepared: &PreparedRecording) -> Result<File> {
    File::options()
        .create_new(true)
        .write(true)
        .open(&prepared.options.output)
        .with_context(|| {
            format!(
                "failed to create output file {}",
                prepared.options.output.display()
            )
        })
}

async fn receive_topic(
    topic_index: usize,
    topic_name: String,
    subscriber: DynSub,
    sender: flume::Sender<SampleEnvelope>,
    shutdown: CancellationToken,
) -> Result<()> {
    let mut warned_missing_transport_timestamp = false;
    let mut warned_invalid_source_timestamp = false;
    let mut warned_bad_sequence = HashSet::new();

    loop {
        let sample = tokio::select! {
            biased;
            _ = shutdown.cancelled() => break,
            result = subscriber.async_recv_serialized() => {
                result.map_err(|error| anyhow!("subscriber receive failed for {topic_name}: {error}"))?
            }
        };

        let envelope = sample_to_envelope(
            topic_index,
            &topic_name,
            sample,
            &mut warned_missing_transport_timestamp,
            &mut warned_invalid_source_timestamp,
            &mut warned_bad_sequence,
        )?;

        let send_result = tokio::select! {
            biased;
            _ = shutdown.cancelled() => break,
            result = sender.send_async(envelope) => result,
        };

        if send_result.is_err() {
            break;
        }
    }

    Ok(())
}

fn sample_to_envelope(
    topic_index: usize,
    topic_name: &str,
    sample: Sample,
    warned_missing_transport_timestamp: &mut bool,
    warned_invalid_source_timestamp: &mut bool,
    warned_bad_sequence: &mut HashSet<SourceKey>,
) -> Result<SampleEnvelope> {
    let payload = sample.payload().to_bytes().to_vec();
    let zenoh_encoding = sample.encoding().to_string();

    match parse_recording_encoding(&zenoh_encoding) {
        Ok(()) => {}
        Err(Some(other)) => {
            return Err(anyhow!(
                "received non-CDR encoding on {topic_name}: {other} ({zenoh_encoding})"
            ));
        }
        Err(None) => {
            return Err(anyhow!(
                "received unsupported encoding on {topic_name}: {zenoh_encoding}"
            ));
        }
    }

    let transport_time_ns = sample
        .timestamp()
        .map(timestamp_to_unix_nanos)
        .unwrap_or_else(|| wallclock_timestamp(topic_name, warned_missing_transport_timestamp));

    let attachment = decode_attachment(topic_name, sample.attachment());
    let source = attachment
        .as_ref()
        .map(|attachment| SourceKey::Gid(attachment.source_gid))
        .unwrap_or(SourceKey::NoAttachment);

    Ok(SampleEnvelope {
        topic_index,
        payload,
        transport_time_ns,
        source_time_ns: source_timestamp(
            topic_name,
            attachment.as_ref(),
            warned_invalid_source_timestamp,
        ),
        sequence: normalize_sequence(
            attachment
                .as_ref()
                .map(|attachment| attachment.sequence_number),
            topic_name,
            source.clone(),
            warned_bad_sequence,
        ),
        source,
        zenoh_encoding,
    })
}

fn timestamp_to_unix_nanos(timestamp: &zenoh::time::Timestamp) -> u64 {
    timestamp
        .get_time()
        .to_system_time()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

fn wallclock_timestamp(topic_name: &str, warned: &mut bool) -> u64 {
    if !*warned {
        warn!("missing transport timestamp for topic {topic_name}; using wallclock time");
        *warned = true;
    }

    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

fn decode_attachment(
    topic_name: &str,
    attachment: Option<&zenoh::bytes::ZBytes>,
) -> Option<Attachment> {
    match attachment {
        Some(raw) => match Attachment::try_from(raw) {
            Ok(attachment) => Some(attachment),
            Err(error) => {
                warn!("failed to decode attachment metadata for {topic_name}: {error}");
                None
            }
        },
        None => None,
    }
}

fn source_timestamp(
    topic_name: &str,
    attachment: Option<&Attachment>,
    warned: &mut bool,
) -> Option<u64> {
    attachment.and_then(|attachment| match u64::try_from(attachment.source_timestamp) {
        Ok(timestamp) => Some(timestamp),
        Err(_) => {
            if !*warned {
                warn!(
                    "invalid source timestamp for topic {topic_name}; omitting publish_time override"
                );
                *warned = true;
            }
            None
        }
    })
}

fn recorder_qos() -> QosProfile {
    QosProfile {
        history: QosHistory::KeepLast(NonZeroUsize::new(1000).unwrap()),
        ..Default::default()
    }
}

fn parse_recording_encoding(zenoh_encoding: &str) -> std::result::Result<(), Option<Encoding>> {
    match Encoding::from_zenoh_encoding(zenoh_encoding) {
        Some(Encoding::Cdr) => Ok(()),
        Some(other) => Err(Some(other)),
        None if zenoh_encoding.is_empty() || zenoh_encoding == "zenoh/bytes" => Ok(()),
        None => Err(None),
    }
}

fn normalize_sequence(
    sequence_number: Option<i64>,
    topic_name: &str,
    source: SourceKey,
    warned_bad_sequence: &mut HashSet<SourceKey>,
) -> u32 {
    match sequence_number {
        Some(sequence) if (0..=u32::MAX as i64).contains(&sequence) => sequence as u32,
        Some(sequence) => {
            if warned_bad_sequence.insert(source) {
                warn!("invalid attachment sequence number on {topic_name}: {sequence}; writing 0");
            }
            0
        }
        None => 0,
    }
}

async fn cleanup_partial_output(path: &std::path::Path) {
    if let Err(error) = tokio::fs::remove_file(path).await {
        if error.kind() == std::io::ErrorKind::NotFound {
            return;
        }

        warn!(
            "failed to remove partial recording file {}: {}",
            path.display(),
            error
        );
    }
}

impl<W: Write + Seek> McapRecorder<W> {
    fn new(
        writer: W,
        startup: &RecordingStartup,
        topics: &[TopicPlan],
        session_metadata: &BTreeMap<String, String>,
        discovery_timeout: Duration,
        duration_limit: Option<Duration>,
    ) -> Result<Self> {
        let started_at = SystemTime::now();
        let mut writer = WriteOptions::new()
            .use_chunks(true)
            .compression(Some(Compression::Zstd))
            .chunk_size(Some(8 * 1024 * 1024))
            .emit_summary_records(true)
            .emit_message_indexes(true)
            .emit_chunk_indexes(true)
            .emit_metadata_indexes(true)
            .emit_statistics(true)
            .profile("ros-z")
            .library(format!("ros-z-record/{}", env!("CARGO_PKG_VERSION")))
            .create(writer)
            .context("failed to create mcap writer")?;

        let mut schema_ids = HashMap::new();
        let mut topic_states = Vec::with_capacity(topics.len());

        for topic in topics {
            let schema_id = schema_id(&mut writer, &mut schema_ids, topic)?;
            topic_states.push(TopicState {
                requested_topic: topic.startup.requested_topic.clone(),
                qualified_topic: topic.startup.qualified_topic.clone(),
                type_name: topic.startup.type_name.clone(),
                type_hash: topic.startup.type_hash.clone(),
                startup_publisher_count: topic.startup.publishers.len(),
                schema_id,
                messages: 0,
                bytes: 0,
            });
        }

        write_startup_metadata(
            &mut writer,
            startup,
            session_metadata,
            started_at,
            discovery_timeout,
            duration_limit,
        )?;

        Ok(Self {
            writer,
            started_at,
            topics: topic_states,
            channels: HashMap::new(),
            total_messages: 0,
            total_bytes: 0,
        })
    }

    fn run(
        mut self,
        receiver: Receiver<SampleEnvelope>,
        stats_tx: watch::Sender<StatsSnapshot>,
    ) -> Result<RecordingReport> {
        let mut last_stats_tick = Instant::now();

        loop {
            match receiver.recv_timeout(INTERNAL_STATS_TICK) {
                Ok(envelope) => self.write_sample(envelope)?,
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => break,
            }

            if last_stats_tick.elapsed() >= INTERNAL_STATS_TICK {
                let _ = stats_tx.send(self.snapshot());
                last_stats_tick = Instant::now();
            }
        }

        let _ = stats_tx.send(self.snapshot());
        self.finish()
    }

    fn write_sample(&mut self, envelope: SampleEnvelope) -> Result<()> {
        let channel_id = self.ensure_channel(
            envelope.topic_index,
            &envelope.source,
            &envelope.zenoh_encoding,
        )?;
        let payload_len = u64::try_from(envelope.payload.len()).unwrap_or(u64::MAX);

        self.writer
            .write_to_known_channel(
                &MessageHeader {
                    channel_id,
                    sequence: envelope.sequence,
                    log_time: envelope.transport_time_ns,
                    publish_time: envelope
                        .source_time_ns
                        .unwrap_or(envelope.transport_time_ns),
                },
                &envelope.payload,
            )
            .context("failed to write MCAP message")?;

        self.total_messages += 1;
        self.total_bytes += payload_len;

        let topic = &mut self.topics[envelope.topic_index];
        topic.messages += 1;
        topic.bytes += payload_len;

        Ok(())
    }

    fn ensure_channel(
        &mut self,
        topic_index: usize,
        source: &SourceKey,
        zenoh_encoding: &str,
    ) -> Result<u16> {
        let key = ChannelKey {
            topic_index,
            source: source.clone(),
            zenoh_encoding: zenoh_encoding.to_string(),
        };

        if let Some(channel_id) = self.channels.get(&key).copied() {
            return Ok(channel_id);
        }

        let topic = &self.topics[topic_index];
        let channel_id = self
            .writer
            .add_channel(
                topic.schema_id,
                &topic.qualified_topic,
                MESSAGE_ENCODING,
                &channel_metadata(topic, source, zenoh_encoding),
            )
            .with_context(|| format!("failed to add channel for {}", topic.qualified_topic))?;

        self.channels.insert(key, channel_id);
        Ok(channel_id)
    }

    fn snapshot(&self) -> StatsSnapshot {
        StatsSnapshot {
            total_messages: self.total_messages,
            total_bytes: self.total_bytes,
            topic_stats: self
                .topics
                .iter()
                .map(|topic| TopicStats {
                    topic: topic.qualified_topic.clone(),
                    messages: topic.messages,
                    bytes: topic.bytes,
                })
                .collect(),
        }
    }

    fn finish(mut self) -> Result<RecordingReport> {
        self.writer
            .finish()
            .context("failed to finalize mcap file")?;

        Ok(RecordingReport {
            started_at: self.started_at,
            finished_at: SystemTime::now(),
            total_messages: self.total_messages,
            total_bytes: self.total_bytes,
            topic_stats: self.snapshot().topic_stats,
            silent_topics: self
                .topics
                .iter()
                .filter(|topic| topic.messages == 0)
                .map(|topic| topic.qualified_topic.clone())
                .collect(),
        })
    }
}

fn schema_id<W: Write + Seek>(
    writer: &mut Writer<W>,
    schema_ids: &mut HashMap<(String, String), u16>,
    topic: &TopicPlan,
) -> Result<u16> {
    let schema_key = (
        topic.startup.type_name.clone(),
        topic.startup.schema_json.clone(),
    );

    if let Some(schema_id) = schema_ids.get(&schema_key).copied() {
        return Ok(schema_id);
    }

    let schema_id = writer
        .add_schema(
            &topic.startup.type_name,
            SCHEMA_ENCODING,
            topic.startup.schema_json.as_bytes(),
        )
        .with_context(|| format!("failed to add schema for {}", topic.startup.qualified_topic))?;
    schema_ids.insert(schema_key, schema_id);
    Ok(schema_id)
}

fn channel_metadata(
    topic: &TopicState,
    source: &SourceKey,
    zenoh_encoding: &str,
) -> BTreeMap<String, String> {
    BTreeMap::from([
        ("requested_topic".to_string(), topic.requested_topic.clone()),
        ("qualified_topic".to_string(), topic.qualified_topic.clone()),
        ("type_name".to_string(), topic.type_name.clone()),
        ("type_hash".to_string(), topic.type_hash.clone()),
        ("schema_encoding".to_string(), SCHEMA_ENCODING.to_string()),
        ("zenoh_encoding".to_string(), zenoh_encoding.to_string()),
        ("source_id".to_string(), source_label(source)),
        (
            "startup_publisher_count".to_string(),
            topic.startup_publisher_count.to_string(),
        ),
    ])
}

fn source_label(source: &SourceKey) -> String {
    match source {
        SourceKey::NoAttachment => "no_attachment".to_string(),
        SourceKey::Gid(gid) => format!("gid:{}", hex::encode(gid)),
    }
}

fn write_startup_metadata<W: Write + Seek>(
    writer: &mut Writer<W>,
    startup: &RecordingStartup,
    session_metadata: &BTreeMap<String, String>,
    started_at: SystemTime,
    discovery_timeout: Duration,
    duration_limit: Option<Duration>,
) -> Result<()> {
    let mut session = session_metadata.clone();
    session.insert(
        "output_path".to_string(),
        startup.output.display().to_string(),
    );
    session.insert(
        "started_at_unix_ns".to_string(),
        system_time_to_nanos(&started_at).to_string(),
    );
    session.insert(
        "discovery_timeout_ms".to_string(),
        discovery_timeout.as_millis().to_string(),
    );
    session.insert(
        "duration_limit_ms".to_string(),
        duration_limit
            .map(|duration| duration.as_millis().to_string())
            .unwrap_or_else(|| "none".to_string()),
    );
    session.insert(
        "recorder_version".to_string(),
        env!("CARGO_PKG_VERSION").to_string(),
    );

    write_metadata(writer, "ros-z.session", session)?;
    write_metadata(
        writer,
        "ros-z.request",
        BTreeMap::from([(
            "topics".to_string(),
            serde_json::to_string(&startup.requested_topics)
                .context("failed to serialize requested topics metadata")?,
        )]),
    )?;
    write_metadata(
        writer,
        "ros-z.resolved_topics",
        BTreeMap::from([(
            "topics".to_string(),
            serde_json::to_string(&startup.resolved_topics)
                .context("failed to serialize resolved topic metadata")?,
        )]),
    )?;

    Ok(())
}

fn write_metadata<W: Write + Seek>(
    writer: &mut Writer<W>,
    name: &str,
    metadata: BTreeMap<String, String>,
) -> Result<()> {
    writer
        .write_metadata(&Metadata {
            name: name.to_string(),
            metadata,
        })
        .with_context(|| format!("failed to write {name} metadata"))
}

fn initial_stats(topics: &[TopicPlan]) -> StatsSnapshot {
    StatsSnapshot {
        total_messages: 0,
        total_bytes: 0,
        topic_stats: topics
            .iter()
            .map(|topic| TopicStats {
                topic: topic.startup.qualified_topic.clone(),
                messages: 0,
                bytes: 0,
            })
            .collect(),
    }
}

#[cfg(test)]
mod tests {
    use std::{collections::BTreeMap, fs, io::BufWriter, sync::Arc, time::Duration};

    use mcap::{MessageStream, Summary};
    use tempfile::tempdir;
    use tokio::sync::watch;

    use crate::{
        TopicPlan,
        api::{RecordingStartup, ResolvedPublisher, ResolvedTopic, StatsSnapshot},
    };

    use super::{
        McapRecorder, SampleEnvelope, SourceKey, normalize_sequence, parse_recording_encoding,
    };
    use ros_z::encoding::Encoding;

    #[test]
    fn accepts_default_zenoh_bytes_encoding() {
        assert_eq!(parse_recording_encoding("application/octet-stream"), Ok(()));
        assert_eq!(parse_recording_encoding("application/cdr"), Ok(()));
        assert_eq!(parse_recording_encoding("zenoh/bytes"), Ok(()));
        assert_eq!(parse_recording_encoding(""), Ok(()));
        assert_eq!(
            parse_recording_encoding("application/protobuf"),
            Err(Some(Encoding::protobuf()))
        );
    }

    #[test]
    fn invalid_sequence_numbers_fall_back_to_zero_once() {
        let mut warned = std::collections::HashSet::new();
        let key = SourceKey::NoAttachment;

        assert_eq!(
            normalize_sequence(Some(7), "/topic", key.clone(), &mut warned),
            7
        );
        assert_eq!(
            normalize_sequence(Some(-1), "/topic", key.clone(), &mut warned),
            0
        );
        assert_eq!(warned.len(), 1);
        assert_eq!(
            normalize_sequence(Some(i64::from(u32::MAX) + 1), "/topic", key, &mut warned),
            0
        );
        assert_eq!(warned.len(), 1);
    }

    #[test]
    fn writer_splits_channels_by_source_id_and_preserves_headers() {
        let tempdir = tempdir().expect("tempdir");
        let output = tempdir.path().join("writer_test.mcap");
        let file = std::fs::File::create(&output).expect("output file");
        let topics = vec![topic_plan("/record", "/record")];
        let startup = RecordingStartup {
            output: output.clone(),
            requested_topics: vec!["/record".to_string()],
            resolved_topics: vec![topics[0].startup.clone()],
        };
        let recorder = McapRecorder::new(
            BufWriter::new(file),
            &startup,
            &topics,
            &BTreeMap::new(),
            Duration::from_secs(1),
            None,
        )
        .expect("writer");
        let (sender, receiver) = flume::bounded(16);
        let (stats_tx, _stats_rx) = watch::channel(StatsSnapshot {
            total_messages: 0,
            total_bytes: 0,
            topic_stats: vec![],
        });

        sender
            .send(SampleEnvelope {
                topic_index: 0,
                payload: vec![1, 2, 3],
                transport_time_ns: 10,
                source_time_ns: Some(5),
                sequence: 7,
                source: SourceKey::Gid([1; 16]),
                zenoh_encoding: "zenoh/bytes".to_string(),
            })
            .expect("send first envelope");
        sender
            .send(SampleEnvelope {
                topic_index: 0,
                payload: vec![4, 5, 6],
                transport_time_ns: 20,
                source_time_ns: Some(8),
                sequence: 8,
                source: SourceKey::Gid([2; 16]),
                zenoh_encoding: "zenoh/bytes".to_string(),
            })
            .expect("send second envelope");
        drop(sender);

        let report = recorder.run(receiver, stats_tx).expect("run writer");
        assert_eq!(report.total_messages, 2);

        let bytes = fs::read(output).expect("read mcap");
        let summary = Summary::read(&bytes)
            .expect("summary parse")
            .expect("summary section");
        let messages = MessageStream::new(&bytes)
            .expect("message stream")
            .collect::<std::result::Result<Vec<_>, _>>()
            .expect("collect messages");

        assert_eq!(summary.channels.len(), 2);
        assert_eq!(messages.len(), 2);
        assert_eq!(messages[0].sequence, 7);
        assert_eq!(messages[0].publish_time, 5);
        assert_eq!(messages[1].sequence, 8);
        assert_eq!(messages[1].publish_time, 8);
    }

    #[test]
    fn writer_reports_silent_topics() {
        let tempdir = tempdir().expect("tempdir");
        let output = tempdir.path().join("silent_writer_test.mcap");
        let file = std::fs::File::create(&output).expect("output file");
        let topics = vec![topic_plan("/silent", "/silent")];
        let startup = RecordingStartup {
            output,
            requested_topics: vec!["/silent".to_string()],
            resolved_topics: vec![topics[0].startup.clone()],
        };
        let recorder = McapRecorder::new(
            BufWriter::new(file),
            &startup,
            &topics,
            &BTreeMap::new(),
            Duration::from_secs(1),
            None,
        )
        .expect("writer");
        let (sender, receiver) = flume::bounded(1);
        let (stats_tx, _stats_rx) = watch::channel(StatsSnapshot {
            total_messages: 0,
            total_bytes: 0,
            topic_stats: vec![],
        });
        drop(sender);

        let report = recorder.run(receiver, stats_tx).expect("run writer");

        assert_eq!(report.total_messages, 0);
        assert_eq!(report.silent_topics, vec!["/silent".to_string()]);
    }

    fn topic_plan(requested_topic: &str, qualified_topic: &str) -> TopicPlan {
        let schema = ros_z::dynamic::MessageSchema::builder("std_msgs/msg/String")
            .field("data", ros_z::dynamic::FieldType::String)
            .build()
            .expect("schema");
        TopicPlan {
            startup: ResolvedTopic {
                requested_topic: requested_topic.to_string(),
                qualified_topic: qualified_topic.to_string(),
                type_name: "std_msgs/msg/String".to_string(),
                type_hash:
                    "RIHS01_0000000000000000000000000000000000000000000000000000000000000000"
                        .to_string(),
                schema_json: "{\"type_name\":\"std_msgs/msg/String\"}".to_string(),
                publishers: vec![ResolvedPublisher {
                    node_fqn: Some("/talker".to_string()),
                    type_hash: None,
                    qos: "::,10:,:,,:,,:".to_string(),
                }],
            },
            schema: Arc::clone(&schema),
        }
    }
}
