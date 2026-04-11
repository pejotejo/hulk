use std::{f32::consts::PI, sync::Arc, time::Duration};

use color_eyre::Result;
use linear_algebra::{Rotation2, Vector2};
use ros_z::{Builder, context::ZContext};
use ros_z_config::prelude::*;

use crate::{
    IntoEyreResultExt, config::BehaviorConfig, msgs::maybe_ball_position::MaybeBallPosition,
};
use types::motion_command::{HeadMotion, ImageRegion, MotionCommand};

pub async fn run(ctx: Arc<ZContext>) -> Result<()> {
    let node = ctx
        .create_node("behavior")
        .with_type_description_service()
        .with_extended_type_description_service()
        .build()
        .into_eyre()?;
    let config = node
        .bind_config_with_metadata_as::<BehaviorConfig>("behavior")
        .into_eyre()?;
    config
        .add_validation_hook(|cfg: &BehaviorConfig| {
            for (name, value, min, max) in [
                ("behavior.walk.forward", cfg.walk.forward, 0.0, 5.0),
                ("behavior.walk.angular_scale", cfg.walk.angular_scale, 0.0, 5.0),
                ("behavior.walk.angular_max", cfg.walk.angular_max, 0.0, 5.0),
            ] {
                if !value.is_finite() {
                    return Err(format!("{name} must be finite"));
                }
                if value < min || value > max {
                    return Err(format!("{name} must be between {min} and {max}"));
                }
            }
            Ok(())
        })
        .into_eyre()?;

    let maybe_ball_position_sub = node
        .create_sub::<MaybeBallPosition>("ball_filter/ball_position")
        .build()
        .into_eyre()?;
    let motion_command_pub = node
        .create_pub::<MotionCommand>("behavior/motion_command")
        .build()
        .into_eyre()?;

    let mut maybe_ball_position = MaybeBallPosition { position: None };
    let mut timer = node.clock().timer(Duration::from_secs_f64(1.0 / 30.0));

    loop {
        let cfg = config.snapshot().typed().clone();

        tokio::select! {
            msg = maybe_ball_position_sub.async_recv() => {
                    maybe_ball_position = msg.into_eyre()?;
               }
            _ = timer.tick() => {

                let mut motion_command = MotionCommand::Stand{ head: HeadMotion::LookAround};

                if let Some(ball_position) = maybe_ball_position.position  {
                    let ball_coordinates_in_ground = ball_position.position.coords();

                    let normalized_angle_to_ball =
                    Rotation2::rotation_between(Vector2::x_axis(), ball_coordinates_in_ground).angle()
                        / (0.5 * PI);

                    motion_command = MotionCommand::WalkWithVelocity {
                        head: HeadMotion::LookAt { target: ball_position.position, image_region_target: ImageRegion::Bottom },
                        velocity: ball_coordinates_in_ground.normalize() * cfg.walk.forward,
                        angular_velocity: (normalized_angle_to_ball * cfg.walk.angular_scale).clamp(-cfg.walk.angular_max, cfg.walk.angular_max),
                    };
                }
                motion_command_pub.async_publish(&motion_command).await.into_eyre()?;

            }
        }
    }
}
