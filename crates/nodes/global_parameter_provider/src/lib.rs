use std::{boxed::Box, future::Future, pin::Pin, sync::Arc};

use color_eyre::Result;
use hsl_network_messages::PlayerNumber;
use serde::{Deserialize, Serialize};

use ros_z::{prelude::*, qos::QosDurability};
use types::{
    field_dimensions::FieldDimensions,
    walking_velocity_limits::{WALKING_VELOCITY_LIMITS_TOPIC, WalkingVelocityLimits},
};

#[derive(Debug, Clone, Serialize, Deserialize, Message)]
#[serde(deny_unknown_fields)]
pub struct Parameters {
    pub player_number: PlayerNumber,
    pub field_dimensions: FieldDimensions,
    pub walking_velocity_limits: WalkingVelocityLimits,
}

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx))
}

async fn run(ctx: Arc<Context>) -> Result<()> {
    let node = ctx.create_node("global_parameter_provider").build().await?;

    let node_parameters = node.bind_parameter_as::<Parameters>("global")?;

    node_parameters
        .add_validation_hook(|parameters| parameters.walking_velocity_limits.validate())?;

    let player_number_pub = node
        .publisher::<PlayerNumber>("player_number")
        .qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        })
        .build()
        .await?;

    let field_dimensions_pub = node
        .publisher::<FieldDimensions>("field_dimensions")
        .qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        })
        .build()
        .await?;

    let walking_velocity_limits_pub = node
        .publisher::<WalkingVelocityLimits>(WALKING_VELOCITY_LIMITS_TOPIC)
        .qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        })
        .build()
        .await?;

    let mut parameters_receiver = node_parameters.subscribe();
    loop {
        let parameters = parameters_receiver.borrow_and_update().clone();
        let parameters = parameters.typed();

        player_number_pub.publish(&parameters.player_number).await?;
        field_dimensions_pub
            .publish(&parameters.field_dimensions)
            .await?;
        walking_velocity_limits_pub
            .publish(&parameters.walking_velocity_limits)
            .await?;

        parameters_receiver.changed().await?;
    }
}
