//! Shootout practice uses the controller's shot counters, alternation and completion rules.
use super::*;
use game_controller_core::{log::LoggedAction, timer::Timer};

impl Engine {
    fn shot_candidate(&self, side: Side, goalkeeper: bool) -> Option<RobotId> {
        self.snapshot
            .robots
            .iter()
            .filter(|robot| {
                super::super::side(robot.id.team) == side
                    && self.penalty(robot.id) != Penalty::SentOff
            })
            .min_by_key(|robot| {
                (
                    if goalkeeper {
                        robot.id.number != 1
                    } else {
                        robot.id.number == 1
                    },
                    robot.id.number,
                )
            })
            .map(|robot| robot.id)
    }

    pub(super) fn shootout_allowed(&mut self, call: Call) -> Option<bool> {
        let game = self.core.get_game(false);
        match call {
            Call::Shootout(_) => Some(
                game.phase != Phase::PenaltyShootout
                    && self.snapshot.ball.is_some()
                    && [Side::Home, Side::Away]
                        .into_iter()
                        .all(|side| self.shot_candidate(side, false).is_some()),
            ),
            Call::NextShot => Some(
                self.snapshot.ball.is_some()
                    && [Side::Home, Side::Away]
                        .into_iter()
                        .all(|side| self.shot_candidate(side, false).is_some())
                    && call.action(game).is_legal(&self.core.get_context(false)),
            ),
            Call::ShotPlayer(id) => Some(
                game.phase == Phase::PenaltyShootout
                    && game.state == State::Set
                    && self.snapshot.robots.iter().any(|robot| robot.id == id)
                    && self.penalty(id) != Penalty::SentOff,
            ),
            _ => None,
        }
    }

    // Upstream selection resets every other player's penalty, including sent-off players.
    // Keep red cards across shot selection and alternation.
    fn apply_shot_action(&mut self, action: VAction) {
        let mut sent_off = Vec::new();
        for side in [Side::Home, Side::Away] {
            for player in PlayerNumber::all() {
                if self.core.get_game(false).teams[side][player].penalty == Penalty::SentOff {
                    sent_off.push((side, player));
                }
            }
        }
        self.core.apply(action, ActionSource::Network);
        for (side, player) in sent_off {
            self.core.get_context(false).game.teams[side][player].penalty = Penalty::SentOff;
        }
    }

    pub(super) fn call_shootout(&mut self, call: Call) -> bool {
        if !matches!(
            call,
            Call::Shootout(_) | Call::NextShot | Call::ShotPlayer(_)
        ) {
            return false;
        }
        self.pending.clear();
        self.handlers.clear();
        self.placement = None;
        self.resume_at = None;
        self.whistles.clear();
        self.last_contact = None;
        self.manual_ball_outside = false;
        self.reviewing = false;
        self.observe(false);
        if let Call::Shootout(_) = call {
            let action = call.action(self.core.get_game(false));
            // An explicit simulator setup is allowed before full time, unlike match progression.
            action.execute(&mut self.core.get_context(false));
            self.core.log_now(LogEntry::Action(LoggedAction {
                source: ActionSource::Network,
                action,
            }));
            let game = self.core.get_context(false).game;
            game.stopped = false;
            game.switch_half_timer = Timer::Stopped;
            game.timeout_rewind_timer = Timer::Stopped;
            for team in game.teams.values_mut() {
                team.score = 0;
            }
            *self.core.get_context(true).game = self.core.get_game(false).clone();
            self.observe(false);
        }
        if matches!(call, Call::Shootout(_) | Call::NextShot) {
            self.apply_shot_action(VAction::WaitForPenaltyShot(WaitForPenaltyShot));
            self.observe(false);
            let kicking = self.core.get_game(false).kicking_side.unwrap();
            for side in [Side::Home, Side::Away] {
                if let Some(id) = self.shot_candidate(side, side != kicking) {
                    self.apply_shot_action(Call::ShotPlayer(id).action(self.core.get_game(false)));
                }
            }
        } else {
            self.apply_shot_action(call.action(self.core.get_game(false)));
        }
        self.observe(false);
        self.mapping = self.core.get_game(false).sides;
        self.place_shootout();
        self.restoring = true;
        self.core.log_now(LogEntry::GameState(Box::new(
            self.core.get_game(false).clone(),
        )));
        true
    }

    fn place_shootout(&mut self) {
        let game = self.core.get_game(false);
        let kicking = game.kicking_side.unwrap();
        let direction = if self.away(team(kicking)) { -1.0 } else { 1.0 };
        let field = self.field.dimensions;
        let goal = direction * f64::from(field.length) / 2.0;
        let mark = goal - direction * f64::from(field.penalty_marker_distance);
        let approach = f64::from(field.penalty_area_length)
            .max(f64::from(field.penalty_marker_distance) + 0.75)
            .min(f64::from(field.length) / 2.0);
        let yaw = if direction > 0.0 {
            0.0
        } else {
            std::f64::consts::PI
        };
        self.pending
            .retain(|effect| !matches!(effect, Effect::Ball(_) | Effect::Robot(..)));
        self.pending.push(Effect::Ball([mark, 0.0]));
        self.commanded_ball = Some([mark, 0.0]);
        for robot in &self.snapshot.robots {
            let active = game.teams[side(robot.id.team)][PlayerNumber::new(robot.id.number)]
                .penalty
                == Penalty::NoPenalty;
            let (position, facing) = if active && side(robot.id.team) == kicking {
                ([goal - direction * approach, 0.0], yaw)
            } else if active {
                ([goal, 0.0], yaw + std::f64::consts::PI)
            } else {
                let own = if self.away(robot.id.team) { 1.0 } else { -1.0 };
                (
                    [
                        own * (0.5 + 0.7 * f64::from(robot.id.number - 1)),
                        f64::from(field.width) / 2.0 + f64::from(field.border_strip_width) + 0.6,
                    ],
                    -std::f64::consts::FRAC_PI_2,
                )
            };
            self.pending.push(Effect::Robot(robot.id, position, facing));
        }
    }
}
