//! Bounded referee corrections, without retaining physics frames or cloning an Engine.
use super::*;

const LIMIT: usize = 16;
const MAX_AGE: Duration = Duration::from_secs(120);

pub(super) struct Checkpoint {
    id: u64,
    time: Duration,
    game: Game,
    pub(super) snapshot: Snapshot,
    handlers: BTreeMap<RobotId, Handler>,
    pending: Vec<Effect>,
    placement: Option<([f64; 2], f64)>,
    resume_at: Option<f64>,
    commanded_ball: Option<[f64; 2]>,
    manual_ball_outside: bool,
    next_half_at: f64,
    whistles: VecDeque<f64>,
    last_contact: Option<RobotId>,
}

impl Engine {
    pub(super) fn checkpoint(&self) -> Checkpoint {
        self.checkpoint_with_snapshot(self.snapshot.clone())
    }

    pub(super) fn checkpoint_with_snapshot(&self, snapshot: Snapshot) -> Checkpoint {
        Checkpoint {
            id: self.next_event,
            time: self.core.get_time(),
            game: self.core.get_game(false).clone(),
            snapshot,
            handlers: self.handlers.clone(),
            pending: self.pending.clone(),
            placement: self.placement,
            resume_at: self.resume_at,
            commanded_ball: self.commanded_ball,
            manual_ball_outside: self.manual_ball_outside,
            next_half_at: self.next_half_at,
            whistles: self.whistles.clone(),
            last_contact: self.last_contact,
        }
    }

    pub(super) fn remember(&mut self, checkpoint: Checkpoint) {
        let mut changed = false;
        for event in &mut self.events {
            if event.id >= checkpoint.id && event.accepted {
                event.decision = Some(checkpoint.id);
                changed = true;
            }
        }
        if changed {
            self.decisions.push_back(checkpoint);
        }
        while self.decisions.len() > LIMIT
            || self
                .decisions
                .front()
                .is_some_and(|entry| self.core.get_time().saturating_sub(entry.time) > MAX_AGE)
        {
            self.decisions.pop_front();
        }
    }

    pub fn can_reverse(&self, decision: u64) -> bool {
        self.decisions.iter().any(|entry| {
            entry.id == decision && self.core.get_time().saturating_sub(entry.time) <= MAX_AGE
        })
    }

    /// Reverse this decision and all decisions after it. Stop play so the operator can correct
    /// the scene before resuming; physics and the transport clock remain monotonic.
    pub fn reverse(&mut self, decision: u64) -> Result<(), String> {
        if !self.can_reverse(decision) {
            return Err(
                "Only the last 16 decisions within two simulation minutes can be reversed".into(),
            );
        }
        let index = self
            .decisions
            .iter()
            .position(|entry| entry.id == decision)
            .unwrap();
        let mut removed = self.decisions.split_off(index);
        let checkpoint = removed.pop_front().unwrap();
        let text = self
            .events
            .iter()
            .find(|event| event.decision == Some(decision))
            .map_or("decision".to_owned(), |event| event.text.clone());
        for event in &mut self.events {
            if event.decision.is_some_and(|id| id >= decision) {
                event.reversed = true;
            }
        }

        let offset = self.now() - checkpoint.time.as_secs_f64();
        let mut game = checkpoint.game;
        game.stopped = true;
        // Synchronize both packet views; an old delayed Playing/Set state must not survive.
        *self.core.get_context(true).game = game.clone();
        *self.core.get_context(false).game = game;
        self.mapping = self.core.get_game(false).sides;
        self.snapshot = checkpoint.snapshot;
        self.snapshot.contacts.clear();
        self.handlers = checkpoint.handlers;
        for handler in self.handlers.values_mut() {
            match &mut handler.phase {
                Handling::Waiting(time) | Handling::Placed(time) => *time += offset,
                _ => {}
            }
        }
        self.placement = checkpoint.placement.map(|(p, time)| (p, time + offset));
        self.resume_at = checkpoint.resume_at.map(|time| time + offset);
        self.next_half_at = checkpoint.next_half_at + offset;
        self.whistles = checkpoint
            .whistles
            .into_iter()
            .map(|time| time + offset)
            .collect();
        self.commanded_ball = checkpoint.commanded_ball;
        self.manual_ball_outside = checkpoint.manual_ball_outside;
        self.last_contact = checkpoint.last_contact;
        self.pending.clear();
        // Corrections restore grounded, stationary poses, rather than replaying dynamics.
        if let Some(ball) = &self.snapshot.ball {
            let p = [ball.position[0], ball.position[1]];
            self.pending.push(Effect::Ball(p));
            self.commanded_ball = Some(p);
        }
        for robot in &self.snapshot.robots {
            self.pending.push(Effect::Robot(
                robot.id,
                [robot.position[0], robot.position[1]],
                robot.yaw,
            ));
        }
        self.pending.extend(checkpoint.pending);
        self.reseed_referee();
        self.restoring = true;
        self.reviewing = true;
        self.log(
            self.core.get_time(),
            Origin::Operator,
            true,
            format!("Reversed: {text}; play stopped"),
        );
        self.core.log_now(LogEntry::GameState(Box::new(
            self.core.get_game(false).clone(),
        )));
        Ok(())
    }

    fn reseed_referee(&mut self) {
        self.referee = Autoreferee::new(Self::config(&self.field));
        self.stuck = super::super::stuck::Stuck::default();
        let restored = self.core.get_game(false).clone();
        let world = self.world();
        self.core.get_context(false).game.state = State::Initial;
        self.core.get_context(false).game.set_play = SetPlay::NoSetPlay;
        self.referee.update(&mut self.core, &world);
        // Reconstruct restart restrictions without emitting another placement or whistle.
        *self.core.get_context(false).game = restored.clone();
        if restored.phase == Phase::PenaltyShootout && restored.state == State::Playing {
            self.core.get_context(false).game.state = State::Set;
            self.referee.update(&mut self.core, &world);
            *self.core.get_context(false).game = restored;
        }
        self.referee.update(&mut self.core, &world);
        if let Some(id) = self.last_contact {
            self.referee
                .ball_contact(&mut self.core, side(id.team), PlayerNumber::new(id.number));
        }
    }
}
