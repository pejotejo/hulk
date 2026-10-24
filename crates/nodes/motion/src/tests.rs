use super::*;
use ros_z::prelude::*;
use std::path::PathBuf;

/// Exercise real service delivery and the final motor-command acceptance path.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn transport_allowance_preserves_logical_deadline_and_emergency_stop() {
    let clock = Clock::logical(Time::zero());
    let context = ContextBuilder::default()
        .with_clock(clock.clone())
        .disable_multicast_scouting()
        .with_connect_endpoints(std::iter::empty::<&str>())
        .with_listen_endpoints(["tcp/127.0.0.1:0"])
        .with_parameter_layers([
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../etc/parameters/base")
        ])
        .build()
        .await
        .unwrap();
    let node = context.create_node("transport_test").build().await.unwrap();
    let mut parameters = node
        .bind_parameter_as::<Parameters>("motion")
        .unwrap()
        .snapshot()
        .typed()
        .clone();
    let mut walk = node
        .service_server::<WalkInferenceService>(WALK_INFERENCE_SERVICE)
        .build()
        .await
        .unwrap();
    let mut head = node
        .service_server::<HeadMotionService>(HEAD_MOTION_SERVICE_TOPIC)
        .build()
        .await
        .unwrap();
    let emergency = node
        .subscriber::<()>("motion/emergency_stop")
        .build()
        .await
        .unwrap();
    let mut state = MotionState {
        head_motion_client: node
            .service_client(HEAD_MOTION_SERVICE_TOPIC)
            .build()
            .await
            .unwrap(),
        walk_inference_client: node
            .service_client(WALK_INFERENCE_SERVICE)
            .build()
            .await
            .unwrap(),
        kick_inference_client: node
            .service_client(KICK_INFERENCE_SERVICE)
            .build()
            .await
            .unwrap(),
        get_up_inference_client: node
            .service_client(GETUP_INFERENCE_SERVICE)
            .build()
            .await
            .unwrap(),
        motion_emergency_stop_pub: node
            .publisher("motion/emergency_stop")
            .build()
            .await
            .unwrap(),
        reset_inference: true,
        get_up_failed: false,
        last_joints_command: Joints::fill(MotorCommand::damping()),
        last_timestamp: clock.now(),
    };
    let server_clock = clock.clone();
    let walk_task = tokio::spawn(async move {
        for expire in [false, true] {
            let received = walk.take_request_async().await.unwrap();
            assert_eq!(
                received
                    .message()
                    .deadline
                    .duration_since(server_clock.now()),
                Duration::from_millis(40)
            );
            tokio::time::sleep(Duration::from_millis(80)).await;
            if expire {
                server_clock.advance(Duration::from_millis(40)).unwrap();
            }
            received
                .reply(&Ok(LowerBodyJoints::fill(MotorCommand::damping())))
                .unwrap();
        }
    });
    let head_task = tokio::spawn(async move {
        for _ in 0..2 {
            head.take_request_async()
                .await
                .unwrap()
                .reply(&Ok(HeadJoints::fill(MotorCommand::damping())))
                .unwrap();
        }
    });
    assert!(
        parameters.inference_transport_timeout.is_none(),
        "hardware keeps its existing transport budget"
    );
    parameters.inference_transport_timeout = Some(Duration::from_millis(500));
    parameters.head_motion_timeout = Duration::from_millis(500);
    let limits = JointLimits {
        position: Joints::fill([-4.0, 4.0]),
        maximum_torque: Joints::fill(100.0),
    };
    for expired in [false, true] {
        let command = state
            .infer(
                MotionPlan::Walk {
                    head_motion: HeadMotion::LookAround,
                    command: WalkCommand::stand(),
                },
                &clock,
                &parameters,
                &limits,
                UpperBodyJoints::default(),
                true,
            )
            .await
            .unwrap();
        if expired {
            assert!(matches!(command, RobotCommand::Damping));
            tokio::time::timeout(Duration::from_secs(1), emergency.recv())
                .await
                .unwrap()
                .unwrap();
        } else {
            assert!(
                matches!(command, RobotCommand::Custom { .. }),
                "wall delay with frozen logical time must remain valid"
            );
            assert_eq!(clock.now(), Time::zero());
            assert!(
                tokio::time::timeout(Duration::from_millis(10), emergency.recv())
                    .await
                    .is_err()
            );
        }
    }
    walk_task.await.unwrap();
    head_task.await.unwrap();
}
