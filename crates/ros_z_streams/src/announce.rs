use std::fmt::Debug;

use ros_z::{
    Builder, MessageTypeInfo, msg::ZMessage, node::ZNode, pubsub::ZPub, time::ZTime,
};
use serde::{Deserialize, Serialize};
use zenoh::Result as ZResult;

#[derive(Debug, Serialize, Deserialize, MessageTypeInfo)]
#[ros_msg(type_name = "ros_z_streams/msg/Announcement")]
pub struct Announcement {
    pub(crate) time: ZTime,
    pub(crate) id: u64,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct WithId<T> {
    pub(crate) id: u64,
    pub(crate) value: T,
}

impl<T> ZMessage for WithId<T>
where
    for<'de> T: Deserialize<'de>,
    T: ZMessage + Serialize,
{
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}

impl<T: MessageTypeInfo> MessageTypeInfo for WithId<T> {
    fn type_name() -> &'static str {
        T::type_name()
    }

    fn type_hash() -> ros_z::TypeHash {
        T::type_hash()
    }
}

impl ZMessage for Announcement {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}

/// A wrapper around a [`ZSub`] that automatically publishes an announcement.
pub struct AnnouncingPublisher<T>
where
    T: ZMessage + Serialize,
    for<'de> T: Deserialize<'de>,
{
    next_id: u64,
    data_publisher: ZPub<WithId<T>>,
    announcement_publisher: ZPub<Announcement>,
}

impl<T> AnnouncingPublisher<T>
where
    T: ZMessage + Serialize,
    for<'de> T: Deserialize<'de>,
{
    pub async fn announce(&mut self, time: ZTime) -> ZResult<PendingAnnouncement<'_, T>> {
        let id = self.next_id;
        // TODO(oleflb): think about wraparounds?
        self.next_id += 1;
        self.announcement_publisher
            .async_publish(&Announcement { time, id })
            .await?;
        Ok(PendingAnnouncement {
            id,
            data_publisher: &mut self.data_publisher,
        })
    }
}

pub trait CreateAnnouncingPublisher<T>
where
    T: ZMessage + Serialize,
    for<'de> T: Deserialize<'de>,
{
    fn create_fut_pub(&self, topic: &str) -> ZResult<AnnouncingPublisher<T>>;
}

impl<T> CreateAnnouncingPublisher<T> for ZNode
where
    T: Send + Sync + Serialize + ZMessage,
    for<'de> T: Deserialize<'de>,
{
    fn create_fut_pub(&self, topic: &str) -> ZResult<AnnouncingPublisher<T>> {
        let data_publisher = self.create_pub(topic).build().unwrap();
        let announcement_publisher = self.create_pub(&format!("{topic}/announce")).build()?;

        Ok(AnnouncingPublisher {
            next_id: 0,
            data_publisher,
            announcement_publisher,
        })
    }
}

#[must_use = "data must be published before the announcement is dropped"]
pub struct PendingAnnouncement<'a, T>
where
    T: ZMessage + Serialize,
    for<'de> T: Deserialize<'de>,
{
    id: u64,
    data_publisher: &'a mut ZPub<WithId<T>>,
}

impl<'a, T> PendingAnnouncement<'a, T>
where
    T: ZMessage + Serialize,
    for<'de> T: Deserialize<'de>,
{
    pub async fn async_publish(self, value: T) -> ZResult<()> {
        self.data_publisher
            .async_publish(&WithId { id: self.id, value })
            .await
    }
}
