use super::*;

fn fixture(location: &str) -> (Engine, Snapshot) {
    let field = FieldConfiguration::load(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../etc/parameters"),
        location,
    )
    .unwrap();
    let mut engine = Engine::new(Competition::Middle, field, crate::Opponent::Hulks).unwrap();
    engine.enabled = false;
    let mut snapshot = Snapshot {
        ball: Some(Ball {
            entity: 1,
            position: [0.0, 0.0, 0.11],
            epoch: 0,
        }),
        ..Default::default()
    };
    for team in TeamId::ALL {
        for number in 1..=3 {
            snapshot.robots.push(Robot {
                id: RobotId { team, number },
                position: [
                    if team == TeamId::Hulks { -2.0 } else { 2.0 },
                    f64::from(number),
                    0.65,
                ],
                yaw: 0.0,
                speed: 0.0,
                fallen: false,
                penalized: false,
            });
        }
    }
    engine.update(&snapshot, Duration::ZERO);
    (engine, snapshot)
}

fn playing(engine: &mut Engine) {
    for call in [Call::Kickoff, Call::Set, Call::Whistle, Call::BallFree] {
        engine.call(call).unwrap();
    }
}

fn apply(snapshot: &mut Snapshot, effects: Vec<Effect>) {
    for effect in effects {
        match effect {
            Effect::Ball([x, y]) => {
                let ball = snapshot.ball.as_mut().unwrap();
                ball.position[0] = x;
                ball.position[1] = y;
                ball.epoch += 1;
            }
            Effect::Robot(id, [x, y], yaw) => {
                let robot = snapshot
                    .robots
                    .iter_mut()
                    .find(|robot| robot.id == id)
                    .unwrap();
                robot.position[0] = x;
                robot.position[1] = y;
                robot.yaw = yaw;
            }
            Effect::Whistle => {}
        }
    }
}

fn flush(engine: &mut Engine, snapshot: &mut Snapshot) {
    advance(engine, snapshot, Duration::ZERO);
    engine.update(snapshot, Duration::ZERO);
}

fn advance(engine: &mut Engine, snapshot: &mut Snapshot, dt: Duration) {
    let effects = engine.update(snapshot, dt);
    apply(snapshot, effects);
}

#[test]
fn reverse_goal_restores_both_packet_views_and_placements_without_rewinding_time() {
    let (mut engine, mut snapshot) = fixture("hsl_small");
    playing(&mut engine);
    flush(&mut engine, &mut snapshot);
    engine.call(Call::Goal(TeamId::Hulks)).unwrap();
    let decision = engine.events.back().unwrap().decision.unwrap();
    engine.core.seek(Duration::from_secs(1));
    let now = engine.core.get_time();
    engine.reverse(decision).unwrap();
    for delayed in [false, true] {
        let game = engine.core.get_game(delayed);
        assert_eq!(game.teams[Side::Home].score, 0);
        assert_eq!(game.state, State::Playing);
        assert_eq!(game.set_play, SetPlay::NoSetPlay);
        assert!(game.stopped);
    }
    assert_eq!(engine.core.get_time(), now);
    let effects = engine.update(&snapshot, Duration::from_millis(10));
    assert!(effects.contains(&Effect::Ball([0.0, 0.0])));
    assert_eq!(
        effects
            .iter()
            .filter(|e| matches!(e, Effect::Robot(..)))
            .count(),
        6
    );
    assert!(!effects.contains(&Effect::Whistle));
    assert!(!engine.can_reverse(decision));
    assert!(
        engine
            .events
            .iter()
            .filter(|event| event.decision == Some(decision))
            .all(|event| event.reversed)
    );
}

#[test]
fn automatic_goal_reversal_restores_the_ball_before_its_crossing() {
    let (mut engine, mut snapshot) = fixture("hsl_small");
    playing(&mut engine);
    flush(&mut engine, &mut snapshot);
    engine.enabled = true;
    snapshot.ball.as_mut().unwrap().position[0] = 5.0;
    engine.update(&snapshot, Duration::from_millis(10));
    assert_eq!(engine.core.get_game(false).teams[Side::Home].score, 1);
    let event = engine
        .events
        .iter()
        .rev()
        .find(|event| event.text.starts_with("Goal"))
        .unwrap();
    assert_eq!(event.origin, Origin::Automatic);
    engine.reverse(event.decision.unwrap()).unwrap();
    flush(&mut engine, &mut snapshot);
    assert_eq!(snapshot.ball.as_ref().unwrap().position[0], 0.0);
    assert_eq!(engine.core.get_game(false).teams[Side::Home].score, 0);
    assert!(engine.core.get_game(false).stopped);
}

#[test]
fn reversing_penalty_cancels_handling_and_later_calls_including_cards() {
    let (mut engine, mut snapshot) = fixture("hsl_small");
    playing(&mut engine);
    flush(&mut engine, &mut snapshot);
    let id = snapshot.robots[0].id;
    let before = engine.core.get_game(false).teams[Side::Home].clone();
    engine
        .call(Call::Penalize(id, PenaltyCall::Caution))
        .unwrap();
    let decision = engine.events.back().unwrap().decision.unwrap();
    snapshot.robots[0].penalized = true;
    advance(&mut engine, &mut snapshot, Duration::from_secs(2));
    engine
        .call(Call::Penalize(id, PenaltyCall::SendOff))
        .unwrap();
    engine.reverse(decision).unwrap();
    assert_eq!(engine.core.get_game(false).teams[Side::Home], before);
    assert!(engine.handlers.is_empty());
    advance(&mut engine, &mut snapshot, Duration::ZERO);
    assert_eq!(snapshot.robots[0].position[..2], [-2.0, 1.0]);
    engine.update(&snapshot, Duration::from_secs(2));
    assert_eq!(engine.penalty(id), Penalty::NoPenalty);
    assert!(engine.core.get_game(false).stopped);
}

#[test]
fn reversing_restart_cancels_delayed_ball_placement_and_resume() {
    let (mut engine, mut snapshot) = fixture("hsl_small");
    playing(&mut engine);
    flush(&mut engine, &mut snapshot);
    engine
        .call(Call::Restart(TeamId::Hulks, SetPlay::CornerKick))
        .unwrap();
    let decision = engine.events.back().unwrap().decision.unwrap();
    assert!(engine.placement.is_some());
    engine.reverse(decision).unwrap();
    flush(&mut engine, &mut snapshot);
    let effects = engine.update(&snapshot, Duration::from_secs(2));
    assert!(
        !effects
            .iter()
            .any(|effect| matches!(effect, Effect::Ball(_)))
    );
    assert_eq!(engine.core.get_game(false).set_play, SetPlay::NoSetPlay);
    assert!(engine.core.get_game(false).stopped);
    assert!(engine.placement.is_none() && engine.resume_at.is_none());
}

#[test]
fn manual_ball_edit_invalidates_prior_and_current_update_corrections() {
    let (mut engine, mut snapshot) = fixture("hsl_small");
    for call in [Call::Kickoff, Call::Set, Call::Whistle] {
        engine.call(call).unwrap();
    }
    flush(&mut engine, &mut snapshot);
    assert_eq!(engine.core.get_game(false).set_play, SetPlay::KickOff);
    let previous: Vec<_> = engine
        .events
        .iter()
        .filter_map(|event| event.decision)
        .collect();
    assert!(!previous.is_empty());
    assert!(previous.iter().all(|id| engine.can_reverse(*id)));

    let first_edit_event = engine.next_event;
    let ball = snapshot.ball.as_mut().unwrap();
    ball.position = [1.25, -0.75, 0.11];
    ball.epoch += 1;
    engine.update(&snapshot, Duration::ZERO);
    // The edit ends the active kickoff in this update, producing an accepted action.
    assert_eq!(engine.core.get_game(false).set_play, SetPlay::NoSetPlay);
    assert!(
        engine
            .events
            .iter()
            .any(|event| event.id >= first_edit_event && event.accepted)
    );
    assert!(previous.iter().all(|id| !engine.can_reverse(*id)));
    assert!(
        engine.decisions.is_empty(),
        "the edit update must not retain its pre-edit checkpoint"
    );
    assert!(
        engine
            .events
            .iter()
            .filter(|event| event.id >= first_edit_event)
            .all(|event| event.decision.is_none())
    );
    assert!(engine.reverse(previous[0]).is_err());

    engine.call(Call::Goal(TeamId::Hulks)).unwrap();
    let current = engine.events.back().unwrap().decision.unwrap();
    assert!(engine.can_reverse(current));
    engine.reverse(current).unwrap();
    flush(&mut engine, &mut snapshot);
    assert_eq!(snapshot.ball.as_ref().unwrap().position[..2], [1.25, -0.75]);
    assert_eq!(engine.core.get_game(false).teams[Side::Home].score, 0);
}

#[test]
fn commanded_ball_placement_preserves_prior_corrections() {
    let (mut engine, mut snapshot) = fixture("hsl_small");
    playing(&mut engine);
    flush(&mut engine, &mut snapshot);
    engine
        .call(Call::Restart(TeamId::Hulks, SetPlay::CornerKick))
        .unwrap();
    let decision = engine.events.back().unwrap().decision.unwrap();
    let previous: Vec<_> = engine
        .events
        .iter()
        .filter_map(|event| event.decision)
        .collect();
    advance(&mut engine, &mut snapshot, Duration::from_secs(2));
    assert_ne!(snapshot.ball.as_ref().unwrap().position[..2], [0.0, 0.0]);
    assert!(engine.commanded_ball.is_some());
    engine.update(&snapshot, Duration::ZERO);
    assert!(engine.commanded_ball.is_none());
    assert!(previous.iter().all(|id| engine.can_reverse(*id)));

    engine.reverse(decision).unwrap();
    flush(&mut engine, &mut snapshot);
    assert_eq!(snapshot.ball.as_ref().unwrap().position[..2], [0.0, 0.0]);
}

#[test]
fn decision_history_is_bounded_by_count_and_simulation_age() {
    let (mut engine, snapshot) = fixture("hsl_small");
    for _ in 0..10 {
        engine.call(Call::StopPlay).unwrap();
        engine.call(Call::ResumePlay).unwrap();
    }
    assert_eq!(engine.decisions.len(), 16);
    let newest = engine.events.back().unwrap().decision.unwrap();
    assert!(engine.can_reverse(newest));
    engine.update(&snapshot, Duration::from_secs(121));
    assert!(!engine.can_reverse(newest));
    assert!(engine.reverse(newest).is_err());
}

#[test]
fn shootout_places_two_participants_and_keeps_the_goal_across_turns_on_every_field() {
    for location in ["hsl_small", "hsl_middle", "hsl_large"] {
        for sides in [
            SideMapping::HomeDefendsLeftGoal,
            SideMapping::HomeDefendsRightGoal,
        ] {
            let (mut engine, mut snapshot) = fixture(location);
            engine.call(Call::Shootout(sides)).unwrap();
            flush(&mut engine, &mut snapshot);
            let goal = if sides == SideMapping::HomeDefendsLeftGoal {
                1.0
            } else {
                -1.0
            } * f64::from(engine.field.dimensions.length)
                / 2.0;
            for kicking in [Side::Home, Side::Away] {
                let game = engine.core.get_game(false);
                assert_eq!(game.phase, Phase::PenaltyShootout);
                assert_eq!(game.state, State::Set);
                assert_eq!(game.set_play, SetPlay::NoSetPlay);
                assert_eq!(game.kicking_side, Some(kicking));
                let mark = snapshot.ball.as_ref().unwrap().position[0];
                assert!(
                    (mark
                        - (goal
                            - goal.signum()
                                * f64::from(engine.field.dimensions.penalty_marker_distance)))
                    .abs()
                        < 1e-5
                );
                let mut active = 0;
                for robot in &snapshot.robots {
                    if engine.penalty(robot.id) == Penalty::NoPenalty {
                        active += 1;
                        assert_eq!(robot.position[1], 0.0);
                        if side(robot.id.team) == kicking {
                            assert!(goal.signum() * (mark - robot.position[0]) >= 0.74);
                            assert!((robot.yaw.cos() - goal.signum()).abs() < 1e-5);
                        } else {
                            assert!((robot.position[0] - goal).abs() < 1e-5);
                            assert!((robot.yaw.cos() + goal.signum()).abs() < 1e-5);
                        }
                    } else {
                        assert!(
                            robot.position[1]
                                > f64::from(engine.field.dimensions.width) / 2.0
                                    + f64::from(engine.field.dimensions.border_strip_width)
                        );
                    }
                }
                assert_eq!(active, 2);
                engine.call(Call::Whistle).unwrap();
                engine.call(Call::FinishShot).unwrap();
                if kicking == Side::Home {
                    engine.call(Call::NextShot).unwrap();
                    flush(&mut engine, &mut snapshot);
                }
            }
        }
    }
}

#[test]
fn shootout_goal_miss_timeout_and_completion_use_controller_shot_state() {
    let (mut engine, mut snapshot) = fixture("hsl_small");
    engine.enabled = true;
    engine
        .call(Call::Shootout(SideMapping::HomeDefendsLeftGoal))
        .unwrap();
    flush(&mut engine, &mut snapshot);
    engine.call(Call::Whistle).unwrap();
    snapshot.ball.as_mut().unwrap().position[0] = 5.0;
    engine.update(&snapshot, Duration::from_millis(10));
    let game = engine.core.get_game(false);
    assert_eq!(game.state, State::Finished);
    assert_eq!(game.teams[Side::Home].score, 1);
    assert_eq!(game.teams[Side::Home].penalty_shot_mask, 1);
    engine.call(Call::NextShot).unwrap();
    flush(&mut engine, &mut snapshot);
    engine.call(Call::Whistle).unwrap();
    snapshot.ball.as_mut().unwrap().position[1] = 4.0;
    engine.update(&snapshot, Duration::from_millis(10));
    assert_eq!(engine.core.get_game(false).state, State::Finished);
    assert_eq!(engine.core.get_game(false).teams[Side::Away].score, 0);
    engine.call(Call::NextShot).unwrap();
    flush(&mut engine, &mut snapshot);
    engine.call(Call::Whistle).unwrap();
    engine.update(&snapshot, Duration::from_secs(61));
    assert_eq!(engine.core.get_game(false).state, State::Finished);
    // A decisive lead ends the series even though both teams still have eligible robots.
    engine.core.get_context(false).game.teams[Side::Home].score = 3;
    engine.core.get_context(false).game.teams[Side::Home].penalty_shot = 3;
    engine.core.get_context(false).game.teams[Side::Away].penalty_shot = 2;
    assert!(!engine.allowed(Call::NextShot));
}

#[test]
fn reversing_shootout_goal_restores_its_attempt_and_scoring_mask() {
    let (mut engine, mut snapshot) = fixture("hsl_small");
    engine.enabled = true;
    engine
        .call(Call::Shootout(SideMapping::HomeDefendsLeftGoal))
        .unwrap();
    flush(&mut engine, &mut snapshot);
    engine.call(Call::Whistle).unwrap();
    snapshot.ball.as_mut().unwrap().position[0] = 5.0;
    engine.update(&snapshot, Duration::from_millis(10));
    let decision = engine.events.back().unwrap().decision.unwrap();
    engine.reverse(decision).unwrap();
    let game = engine.core.get_game(false);
    assert_eq!(game.phase, Phase::PenaltyShootout);
    assert_eq!(game.state, State::Playing);
    assert_eq!(game.kicking_side, Some(Side::Home));
    assert_eq!(game.teams[Side::Home].penalty_shot, 1);
    assert_eq!(game.teams[Side::Home].penalty_shot_mask, 0);
    assert_eq!(game.teams[Side::Home].score, 0);
    flush(&mut engine, &mut snapshot);
    assert_eq!(snapshot.ball.as_ref().unwrap().position[0], 3.0);
}

#[test]
fn shot_selection_preserves_red_cards_and_setup_can_be_reversed() {
    let (mut engine, mut snapshot) = fixture("hsl_small");
    playing(&mut engine);
    flush(&mut engine, &mut snapshot);
    let sent_off = snapshot.robots[2].id;
    engine
        .call(Call::Penalize(sent_off, PenaltyCall::SendOff))
        .unwrap();
    engine.call(Call::Goal(TeamId::Hulks)).unwrap();
    let before = engine.core.get_game(false).clone();
    engine
        .call(Call::Shootout(SideMapping::HomeDefendsRightGoal))
        .unwrap();
    let decision = engine.events.back().unwrap().decision.unwrap();
    assert_eq!(engine.core.get_game(false).teams[Side::Home].score, 0);
    assert_eq!(engine.penalty(sent_off), Penalty::SentOff);
    assert!(!engine.allowed(Call::ShotPlayer(sent_off)));
    engine.call(Call::Whistle).unwrap();
    engine.call(Call::FinishShot).unwrap();
    engine.call(Call::NextShot).unwrap();
    assert_eq!(engine.penalty(sent_off), Penalty::SentOff);
    engine.reverse(decision).unwrap();
    let mut expected = before;
    expected.stopped = true;
    assert_eq!(engine.core.get_game(false), &expected);
}

#[test]
fn shootout_red_card_still_removes_the_active_player() {
    let (mut engine, mut snapshot) = fixture("hsl_small");
    engine
        .call(Call::Shootout(SideMapping::HomeDefendsLeftGoal))
        .unwrap();
    flush(&mut engine, &mut snapshot);
    let striker = snapshot
        .robots
        .iter()
        .find(|robot| {
            robot.id.team == TeamId::Hulks && engine.penalty(robot.id) == Penalty::NoPenalty
        })
        .unwrap()
        .id;
    engine
        .call(Call::Penalize(striker, PenaltyCall::SendOff))
        .unwrap();
    let effects = engine.update(&snapshot, Duration::from_secs(2));
    assert!(effects.iter().any(|effect| matches!(effect, Effect::Robot(id, [_, y], _) if *id == striker && y.abs() > f64::from(engine.field.dimensions.width) / 2.0 + f64::from(engine.field.dimensions.border_strip_width))));
    assert_eq!(engine.penalty(striker), Penalty::SentOff);
}

#[test]
fn shootout_refuses_incomplete_scene_without_changing_match_state() {
    let (mut engine, mut snapshot) = fixture("hsl_small");
    let before = engine.core.get_game(false).clone();
    snapshot
        .robots
        .retain(|robot| robot.id.team == TeamId::Hulks);
    engine.update(&snapshot, Duration::ZERO);
    assert!(
        engine
            .call(Call::Shootout(SideMapping::HomeDefendsLeftGoal))
            .is_err()
    );
    assert_eq!(engine.core.get_game(false), &before);
}
