use std::sync::Arc;

use booster_sdk::{
    client::{BoosterClient, light_control::LightControlClient},
    types::RobotMode,
};
use color_eyre::Result;
use ros_z::{Builder, ExtendedMessageTypeInfo, context::ZContext};
use serde::{Deserialize, Serialize};

use crate::IntoEyreResultExt;

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
    let node = ctx
        .create_node("robot_hw")
        .with_type_description_service()
        .with_extended_type_description_service()
        .build()
        .into_eyre()?;
    // let _config = node
    //     .bind_config_with_metadata_as::<RobotHwConfig>("robot_hw")
    //     .into_eyre()?;

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
