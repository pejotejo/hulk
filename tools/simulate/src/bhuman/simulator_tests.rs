use std::{
    sync::{Arc, atomic::AtomicBool},
    thread,
    time::{Duration, Instant},
};

use bevy::prelude::*;

use super::*;
use crate::{
    bevy_mujoco::{MjcfObject, MujocoWorldPlugin, SharedPhysics, SimulationMode},
    simulation::{PhysicsWorker, Speed},
    team::Team,
};

#[test]
#[ignore = "Requires native B-Human worker, source, simulator assets and HULK motion models"]
fn mixed_match_steps_native_players_and_preserves_pause() {
    let runtime = tokio::runtime::Runtime::new().unwrap();
    let router = runtime
        .block_on(
            ContextBuilder::default()
                .with_mode("router")
                .disable_multicast_scouting()
                .with_connect_endpoints(std::iter::empty::<&str>())
                .with_listen_endpoints(["tcp/127.0.0.1:0"])
                .build(),
        )
        .unwrap();
    let endpoint =
        runtime.block_on(async { router.session().info().locators().await[0].to_string() });
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let parameter_root = root.join("../../etc/parameters");
    let field = crate::FieldConfiguration::load(&parameter_root, "hsl_middle").unwrap();
    let dimensions = field.dimensions;
    let configuration = crate::Configuration {
        referee: crate::RefereeMode::Automatic,
        competition: crate::Competition::default(),
        parameter_root,
        model_directory: root.join("../../etc/neural_networks"),
        router: Some(endpoint),
        namespace: crate::robot_namespace(crate::RobotId::FIRST),
        location: Some("hsl_middle".into()),
        robot_count: 5,
        opponent_count: 5,
        opponent: crate::Opponent::BHuman {
            executable: std::env::var_os("BHUMAN_WORKER")
                .expect("Set BHUMAN_WORKER")
                .into(),
            source: std::env::var_os("BHUMAN_SOURCE")
                .expect("Set BHUMAN_SOURCE")
                .into(),
        },
        field_configuration: Some(field),
        profile: crate::Profile::MotionBehavior,
        mcap_enabled: Some(false),
    };
    let team = runtime
        .block_on(Team::new_with_controller(
            runtime.handle().clone(),
            configuration,
            Arc::new(AtomicBool::new(false)),
            None,
            false,
        ))
        .unwrap();
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, MujocoWorldPlugin));
    let parameters: crate::parameters::SimulatorParameters =
        json5::from_str(include_str!("../../parameters/simulator.json5")).unwrap();
    app.world_mut().spawn((
        MjcfObject::from_factory(
            move || {
                crate::scene::ball::ball_spec(f64::from(dimensions.ball_radius), &parameters.ball)
            },
            "ball",
        )
        .with_free_joint("ball_free_joint")
        .grounded(),
        Transform::default(),
    ));
    for side in [-1.0, 1.0] {
        app.world_mut().spawn((
            MjcfObject::from_factory(
                move || crate::scene::goal::goal_spec((&dimensions).into()),
                "goal",
            )
            .with_mocap_body("goal"),
            Transform::from_xyz(side * dimensions.length / 2.0, 0.0, 0.0).with_rotation(
                Quat::from_rotation_y(if side > 0.0 {
                    std::f32::consts::PI
                } else {
                    0.0
                }),
            ),
        ));
    }
    for member in team.members() {
        let entity = app
            .world_mut()
            .spawn((
                MjcfObject::new(
                    concat!(env!("CARGO_MANIFEST_DIR"), "/assets/k1_robot.xml"),
                    "Trunk",
                )
                .with_free_joint("world_joint")
                .grounded(),
                member.pose,
            ))
            .id();
        team.bind(member.id, entity);
    }
    app.update();
    let physics = app.world().resource::<SharedPhysics>().clone();
    physics.lock().mode = SimulationMode::Paused;
    let mut worker = PhysicsWorker::start_team(physics.clone(), team.clone());
    worker.set_speed(Speed::Quadruple);
    let wait = |worker: &mut PhysicsWorker, condition: &dyn Fn() -> bool| {
        let deadline = Instant::now() + Duration::from_secs(30);
        while !condition() {
            if let Some(error) = worker.poll() {
                panic!("{error}");
            }
            assert!(Instant::now() < deadline, "mixed match condition timed out");
            thread::sleep(Duration::from_millis(10));
        }
    };
    wait(&mut worker, &|| {
        team.members().iter().all(|member| member.entity.is_some())
    });
    let native: Vec<_> = team
        .members()
        .into_iter()
        .filter(|member| member.io.is_bhuman())
        .collect();
    assert_eq!(native.len(), 5);
    assert_ne!(
        native[0]
            .io
            .bhuman()
            .unwrap()
            .state
            .lock()
            .unwrap()
            .robot
            .child
            .id(),
        native[1]
            .io
            .bhuman()
            .unwrap()
            .state
            .lock()
            .unwrap()
            .robot
            .child
            .id()
    );
    {
        let mut world = physics.lock();
        world.remaining_steps = Some(1000);
        world.mode = SimulationMode::Running;
    }
    wait(&mut worker, &|| physics.lock().remaining_steps == Some(0));
    for member in &native {
        let state = member.io.bhuman().unwrap().state.lock().unwrap();
        assert_eq!(state.time_ns, 1000 * STEP_NS);
        assert_eq!(state.output.as_ref().unwrap().player_state, "active");
        eprintln!(
            "{} initialized: {} / {}",
            member.id,
            state.output.as_ref().unwrap().game_state,
            state.output.as_ref().unwrap().motion,
        );
        assert!(
            state.packet.is_some(),
            "native player never received GameController data"
        );
        assert!(member.io.primary.get_latest().is_some());
    }
    let paused = team.now();
    thread::sleep(Duration::from_millis(100));
    assert_eq!(team.now(), paused);
    for member in &native {
        assert_eq!(
            member.io.bhuman().unwrap().state.lock().unwrap().time_ns,
            1000 * STEP_NS
        );
    }
    // The automatic referee already enters Ready when it observes the new ball.
    for command in ["set", "whistle", "ball_free"] {
        team.referee()
            .unwrap()
            .command(command, &serde_json::json!({}))
            .unwrap();
        {
            let mut world = physics.lock();
            world.remaining_steps = Some(500);
            world.mode = SimulationMode::Running;
        }
        wait(&mut worker, &|| physics.lock().remaining_steps == Some(0));
    }
    for member in &native {
        let state = member.io.bhuman().unwrap().state.lock().unwrap();
        assert_eq!(state.time_ns, 2500 * STEP_NS);
    }
    // Exercise a full side beyond initial model loading and the first match transition.
    let playing_started = Instant::now();
    let mut saw_playing = false;
    for chunk in 0..30 {
        {
            let mut world = physics.lock();
            world.remaining_steps = Some(1000);
            world.mode = SimulationMode::Running;
        }
        wait(&mut worker, &|| physics.lock().remaining_steps == Some(0));
        for member in &native {
            let state = member.io.bhuman().unwrap().state.lock().unwrap();
            assert_eq!(state.time_ns, (3500 + chunk * 1000) * STEP_NS);
            saw_playing |= state.output.as_ref().unwrap().game_state == "playing";
        }
        eprintln!(
            "5v5 active play: {} simulation seconds, {:.2}s wall",
            (chunk + 1) * 2,
            playing_started.elapsed().as_secs_f64()
        );
        assert!(
            playing_started.elapsed() < Duration::from_secs(180),
            "5v5 play exceeded wall budget"
        );
    }
    assert!(saw_playing, "native players never observed active play");
    // Both edges arrive while paused. The bridge must preserve the tap until
    // separate cognition frames can consume it.
    native[0]
        .io
        .button_event(0, ButtonEventType::PressDown)
        .unwrap();
    native[0]
        .io
        .button_event(0, ButtonEventType::PressUp)
        .unwrap();
    {
        let mut world = physics.lock();
        world.remaining_steps = Some(50);
        world.mode = SimulationMode::Running;
    }
    wait(&mut worker, &|| physics.lock().remaining_steps == Some(0));
    {
        let state = native[0].io.bhuman().unwrap().state.lock().unwrap();
        let output = state.output.as_ref().unwrap();
        assert_eq!(output.player_state, "unstiff");
        assert_eq!(
            output.leds,
            [LedState::Off, LedState::Off, LedState::Blinking]
        );
        assert!(output.joints.iter().all(|joint| joint.kp == 0.0));
    }
    let color = native[0].io.led_color();
    thread::sleep(Duration::from_millis(100));
    assert_eq!(
        native[0].io.led_color(),
        color,
        "native LED blink advanced during pause"
    );
    drop(worker);
    drop(native);
    drop(team);
    drop(app);
    assert!(crate::finish_recordings().is_empty());
    router.shutdown().unwrap();
}
