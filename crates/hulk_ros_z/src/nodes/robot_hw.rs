use std::{
    net::{IpAddr, Ipv4Addr, SocketAddr},
    sync::Arc,
};

use booster::{ButtonEventMsg, FallDownState, LowState, Odometer, RemoteControllerState};
use booster_sdk::{
    client::{BoosterClient, light_control::LightControlClient},
    types::RobotMode,
};
use color_eyre::{
    Result,
    eyre::{Context, eyre},
};
use ros_z::{
    Builder, ExtendedMessageTypeInfo, MessageTypeInfo,
    context::ZContext,
    msg::{SerdeCdrSerdes, ZMessage},
    node::ZNode,
    pubsub::ZPub,
};
use ros2::sensor_msgs::image::Image;
use serde::{Deserialize, Serialize};

use crate::{IntoEyreResultExt, x5_receiver::X5Receiver};

const X5_ADDRESS: SocketAddr = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(192, 168, 127, 10)), 7654);
const ZENOH_LOCALHOST_ENDPOINT: &str = "tcp/127.0.0.1:7447";

const LOW_STATE_ZENOH_TOPIC: &str = "rt/low_state";
const ODOMETER_STATE_ZENOH_TOPIC: &str = "rt/odometer_state";
const FALL_DOWN_ZENOH_TOPIC: &str = "rt/fall_down";
const BUTTON_EVENT_ZENOH_TOPIC: &str = "rt/button_event";
const REMOTE_CONTROLLER_STATE_ZENOH_TOPIC: &str = "rt/remote_controller_state";

const LOW_STATE_ROSZ_TOPIC: &str = "robot_hw/low_state";
const ODOMETER_STATE_ROSZ_TOPIC: &str = "robot_hw/odometer_state";
const FALL_DOWN_ROSZ_TOPIC: &str = "robot_hw/fall_down";
const BUTTON_EVENT_ROSZ_TOPIC: &str = "robot_hw/button_event";
const REMOTE_CONTROLLER_STATE_ROSZ_TOPIC: &str = "robot_hw/remote_controller_state";

#[derive(Serialize, Deserialize, ExtendedMessageTypeInfo)]
#[ros_msg(type_name = "hulk_ros_z/msg/LedCommand")]
pub enum LedCommand {
    SetParam { r: u8, g: u8, b: u8 },
    Stop,
}

impl ros_z::msg::ZMessage for LedCommand {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}

#[derive(Serialize, Deserialize, ExtendedMessageTypeInfo)]
#[ros_msg(type_name = "hulk_ros_z/msg/HighLevelCommand")]
pub enum HighLevelCommand {
    ChangeMode { mode: i32 },
    MoveRobot { forward: f32, left: f32, turn: f32 },
    RotateHead { pitch: f32, yaw: f32 },
    RotateHeadWithDirection { pitch: i32, yaw: i32 },
    LieDown,
    GetUp,
    GetUpWithMode { mode: i32 },
    EnterWbcGait,
    ExitWbcGait,
    VisualKick { start: bool },
    ResetOdometer,
}

impl ros_z::msg::ZMessage for HighLevelCommand {
    type Serdes = ros_z::msg::SerdeCdrSerdes<Self>;
}

pub async fn run(ctx: Arc<ZContext>) -> Result<()> {
    let node = Arc::new(
        ctx.create_node("robot_hw")
            .with_type_description_service()
            .with_extended_type_description_service()
            .build()
            .into_eyre()?,
    );
    // let _config = node
    //     .bind_config_with_metadata_as::<RobotHwConfig>("robot_hw")
    //     .into_eyre()?;

    // TODO: camera info service
    let left_image_pub = node
        .create_pub::<Image>("robot_hw/left_image")
        .build()
        .into_eyre()?;
    let right_image_pub = node
        .create_pub::<Image>("robot_hw/right_image")
        .build()
        .into_eyre()?;
    tokio::spawn(image_publisher_task(left_image_pub, right_image_pub));

    let zenoh_session = Arc::new(
        zenoh::open(localhost_zenoh_config()?)
            .await
            .map_err(|error| eyre!("failed to create Zenoh session: {error}"))?,
    );

    spawn_zenoh_rosz_bridges(&node, zenoh_session);

    // TODO: get robot state service
    let led_command_sub = node
        .create_sub::<LedCommand>("robot_hw/led_command")
        .build()
        .into_eyre()?;
    let high_level_command_sub = node
        .create_sub::<HighLevelCommand>("robot_hw/high_level_command")
        .build()
        .into_eyre()?;

    let high_level_interface_client = Arc::new(BoosterClient::new()?);
    let light_control_client = Arc::new(LightControlClient::new()?);

    loop {
        tokio::select! {
            led_command = led_command_sub.async_recv() => {
                let led_command = led_command.into_eyre()?;

                tokio::spawn({
                    let light_control_client = light_control_client.clone();

                    handle_led_command(light_control_client, led_command)
                });
            },
            high_level_command = high_level_command_sub.async_recv() => {
                let high_level_command = high_level_command.into_eyre()?;

                tokio::spawn({
                    let high_level_interface_client = high_level_interface_client.clone();

                    handle_high_level_command(high_level_interface_client, high_level_command)
                });
            }
        }
    }
}

// TODO: deduplicate with zenoh bridge, speak ROS directly
fn spawn_zenoh_rosz_bridges(node: &Arc<ZNode>, zenoh_session: Arc<zenoh::Session>) {
    tokio::spawn(zenoh_rosz_bridge::<LowState>(
        zenoh_session.clone(),
        node.clone(),
        LOW_STATE_ZENOH_TOPIC,
        LOW_STATE_ROSZ_TOPIC,
    ));
    tokio::spawn(zenoh_rosz_bridge::<Odometer>(
        zenoh_session.clone(),
        node.clone(),
        ODOMETER_STATE_ZENOH_TOPIC,
        ODOMETER_STATE_ROSZ_TOPIC,
    ));
    tokio::spawn(zenoh_rosz_bridge::<FallDownState>(
        zenoh_session.clone(),
        node.clone(),
        FALL_DOWN_ZENOH_TOPIC,
        FALL_DOWN_ROSZ_TOPIC,
    ));
    tokio::spawn(zenoh_rosz_bridge::<ButtonEventMsg>(
        zenoh_session.clone(),
        node.clone(),
        BUTTON_EVENT_ZENOH_TOPIC,
        BUTTON_EVENT_ROSZ_TOPIC,
    ));
    tokio::spawn(zenoh_rosz_bridge::<RemoteControllerState>(
        zenoh_session.clone(),
        node.clone(),
        REMOTE_CONTROLLER_STATE_ZENOH_TOPIC,
        REMOTE_CONTROLLER_STATE_ROSZ_TOPIC,
    ));
}

async fn handle_led_command(
    light_control_client: Arc<LightControlClient>,
    led_command: LedCommand,
) -> Result<()> {
    match led_command {
        LedCommand::SetParam { r, g, b } => {
            if let Err(err) = light_control_client.set_led_light_color(r, g, b).await {
                log::error!("failed to set leds: {err}");
            }
        }
        LedCommand::Stop => {
            if let Err(err) = light_control_client.stop_led_light_control().await {
                log::error!("failed to stop led control: {err}");
            }
        }
    };

    Ok(())
}

async fn handle_high_level_command(
    high_level_interface_client: Arc<BoosterClient>,
    high_level_command: HighLevelCommand,
) -> Result<()> {
    match high_level_command {
        HighLevelCommand::ChangeMode { mode } => high_level_interface_client
            .change_mode(RobotMode::try_from(mode).into_eyre()?)
            .await
            .into_eyre(),
        HighLevelCommand::MoveRobot {
            forward,
            left,
            turn,
        } => high_level_interface_client
            .move_robot(forward, left, turn)
            .await
            .into_eyre(),
        HighLevelCommand::RotateHead { pitch, yaw } => high_level_interface_client
            .rotate_head(pitch, yaw)
            .await
            .into_eyre(),
        HighLevelCommand::RotateHeadWithDirection { pitch, yaw } => high_level_interface_client
            .rotate_head_with_direction(pitch, yaw)
            .await
            .into_eyre(),
        HighLevelCommand::LieDown => high_level_interface_client.lie_down().await.into_eyre(),
        HighLevelCommand::GetUp => high_level_interface_client.get_up().await.into_eyre(),
        HighLevelCommand::GetUpWithMode { mode } => high_level_interface_client
            .get_up_with_mode(mode.try_into().into_eyre()?)
            .await
            .into_eyre(),
        HighLevelCommand::EnterWbcGait => high_level_interface_client
            .enter_wbc_gait()
            .await
            .into_eyre(),
        HighLevelCommand::ExitWbcGait => high_level_interface_client
            .exit_wbc_gait()
            .await
            .into_eyre(),
        HighLevelCommand::VisualKick { start } => high_level_interface_client
            .visual_kick(start)
            .await
            .into_eyre(),
        HighLevelCommand::ResetOdometer => high_level_interface_client
            .reset_odometry()
            .await
            .into_eyre(),
    }
}

async fn image_publisher_task(
    left_image_pub: ZPub<Image, SerdeCdrSerdes<Image>>,
    right_image_pub: ZPub<Image, SerdeCdrSerdes<Image>>,
) -> Result<()> {
    let x5_receiver = X5Receiver::new(X5_ADDRESS);

    loop {
        tokio::select! {
            left_frame = x5_receiver.next_left_frame() => {
                left_image_pub.async_publish(&left_frame.into()).await.into_eyre()?;
            }
            right_frame = x5_receiver.next_right_frame() => {
                right_image_pub.async_publish(&right_frame.into()).await.into_eyre()?;
            }
        }
    }
}

fn localhost_zenoh_config() -> Result<zenoh::Config> {
    let mut config = zenoh::Config::default();
    config
        .insert_json5("mode", r#""client""#)
        .map_err(|error| eyre!("failed to set Zenoh mode: {error}"))?;
    config
        .insert_json5(
            "connect/endpoints",
            &format!(r#"["{ZENOH_LOCALHOST_ENDPOINT}"]"#),
        )
        .map_err(|error| eyre!("failed to set Zenoh connect endpoint: {error}"))?;
    Ok(config)
}

async fn zenoh_rosz_bridge<'de, T: MessageTypeInfo + Serialize + Deserialize<'de> + ZMessage>(
    zenoh_session: Arc<zenoh::Session>,
    rosz_node: Arc<ZNode>,
    zenoh_topic: &str,
    rosz_topic: &str,
) -> Result<()> {
    let zenoh_subscriber = zenoh_session
        .declare_subscriber(zenoh_topic)
        .await
        .into_eyre()?;

    let rosz_publisher = rosz_node.create_pub::<T>(rosz_topic).build().into_eyre()?;

    loop {
        let zenoh_sample = zenoh_subscriber.recv_async().await.into_eyre()?;
        let deserialized_sample = cdr::deserialize(&zenoh_sample.payload().to_bytes())
            .wrap_err("deserialization failed")?;
        rosz_publisher
            .async_publish(&deserialized_sample)
            .await
            .into_eyre()?;
    }
}
