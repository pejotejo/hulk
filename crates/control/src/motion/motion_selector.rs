use color_eyre::Result;
use context_attribute::context;
use framework::{AdditionalOutput, MainOutput};
use serde::{Deserialize, Serialize};
use types::{
    fall_state::Kind,
    motion_command::{JumpDirection, MotionCommand},
    motion_selection::{MotionSafeExits, MotionSelection, MotionType},
};

#[derive(Deserialize, Serialize)]
pub struct MotionSelector {
    last_motion: MotionType,
    stand_up_count: u32,
}

#[context]
pub struct CreationContext {}

#[context]
pub struct CycleContext {
    motion_command: Input<MotionCommand, "motion_command">,
    has_ground_contact: Input<bool, "has_ground_contact">,

    motion_safe_exits: CyclerState<MotionSafeExits, "motion_safe_exits">,
    stand_up_count: AdditionalOutput<u32, "stand_up_count">,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    pub motion_selection: MainOutput<MotionSelection>,
}

impl MotionSelector {
    pub fn new(_context: CreationContext) -> Result<Self> {
        Ok(Self {
            last_motion: MotionType::Unstiff,
            stand_up_count: 0,
        })
    }

    pub fn cycle(&mut self, mut context: CycleContext) -> Result<MainOutputs> {
        let motion_safe_to_exit = context.motion_safe_exits[self.last_motion];
        let requested_motion = motion_type_from_command(context.motion_command);

        let current_motion = transition_motion(
            self.last_motion,
            requested_motion,
            motion_safe_to_exit,
            *context.has_ground_contact,
        );

        self.stand_up_count =
            stand_up_counting(self.last_motion, current_motion, self.stand_up_count);

        context
            .stand_up_count
            .fill_if_subscribed(|| self.stand_up_count);

        let dispatching_motion = if current_motion == MotionType::Dispatching {
            if requested_motion == MotionType::Unstiff {
                Some(MotionType::SitDown)
            } else {
                Some(requested_motion)
            }
        } else {
            None
        };

        self.last_motion = current_motion;
        Ok(MainOutputs {
            motion_selection: MotionSelection {
                current_motion,
                dispatching_motion,
            }
            .into(),
        })
    }
}

fn motion_type_from_command(command: &MotionCommand) -> MotionType {
    match command {
        MotionCommand::ArmsUpSquat => MotionType::ArmsUpSquat,
        MotionCommand::ArmsUpStand { .. } => MotionType::ArmsUpStand,
        MotionCommand::FallProtection { .. } => MotionType::FallProtection,
        MotionCommand::Initial { .. } => MotionType::Initial,
        MotionCommand::Jump { direction } => match direction {
            JumpDirection::Left => MotionType::JumpLeft,
            JumpDirection::Right => MotionType::JumpRight,
            JumpDirection::Center => MotionType::CenterJump,
        },
        MotionCommand::Penalized => MotionType::Penalized,
        MotionCommand::SitDown { .. } => MotionType::SitDown,
        MotionCommand::Stand { .. } => MotionType::Stand,
        MotionCommand::StandUp { kind } => match kind {
            Kind::FacingDown => MotionType::StandUpFront,
            Kind::FacingUp => MotionType::StandUpBack,
            Kind::Sitting => MotionType::StandUpSitting,
        },
        MotionCommand::KeeperMotion { direction } => match direction {
            JumpDirection::Left => MotionType::KeeperJumpLeft,
            JumpDirection::Right => MotionType::KeeperJumpRight,
            JumpDirection::Center => MotionType::WideStance,
        },

        MotionCommand::Unstiff => MotionType::Unstiff,
        MotionCommand::Animation { stiff: false } => MotionType::Animation,
        MotionCommand::Animation { stiff: true } => MotionType::AnimationStiff,
        MotionCommand::Walk { .. } => MotionType::Walk,
        MotionCommand::InWalkKick { .. } => MotionType::Walk,
    }
}

fn transition_motion(
    from: MotionType,
    to: MotionType,
    motion_safe_to_exit: bool,
    has_ground_contact: bool,
) -> MotionType {
    match (from, motion_safe_to_exit, to, has_ground_contact) {
        (MotionType::SitDown, true, MotionType::Unstiff, _) => MotionType::Unstiff,
        (_, _, MotionType::Unstiff, false) => MotionType::Unstiff,
        (MotionType::Dispatching, true, MotionType::Unstiff, true) => MotionType::SitDown,
        (MotionType::StandUpFront, _, MotionType::FallProtection, _) => MotionType::StandUpFront,
        (MotionType::StandUpBack, _, MotionType::FallProtection, _) => MotionType::StandUpBack,
        (MotionType::WideStance, _, MotionType::FallProtection, _) => MotionType::WideStance,
        (MotionType::JumpLeft, _, MotionType::FallProtection, _) => MotionType::JumpLeft,
        (MotionType::JumpRight, _, MotionType::FallProtection, _) => MotionType::JumpRight,
        (MotionType::CenterJump, _, MotionType::FallProtection, _) => MotionType::CenterJump,
        (MotionType::StandUpSitting, _, MotionType::FallProtection, _) => {
            MotionType::StandUpSitting
        }
        (MotionType::ArmsUpStand, _, MotionType::FallProtection, _) => MotionType::ArmsUpStand,
        (MotionType::StandUpFront, true, MotionType::StandUpFront, _) => MotionType::Dispatching,
        (MotionType::StandUpBack, true, MotionType::StandUpBack, _) => MotionType::Dispatching,
        (MotionType::StandUpSitting, true, MotionType::StandUpSitting, _) => {
            MotionType::Dispatching
        }
        (_, _, MotionType::FallProtection, _) => MotionType::FallProtection,
        (MotionType::Walk, _, MotionType::WideStance, _) => MotionType::WideStance,
        (MotionType::Walk, _, MotionType::KeeperJumpRight, _) => MotionType::KeeperJumpRight,
        (MotionType::Walk, _, MotionType::KeeperJumpLeft, _) => MotionType::KeeperJumpLeft,

        (MotionType::WideStance, true, MotionType::WideStance, _) => MotionType::WideStance,
        (MotionType::KeeperJumpRight, true, MotionType::KeeperJumpRight, _) => {
            MotionType::KeeperJumpRight
        }
        (MotionType::KeeperJumpLeft, true, MotionType::KeeperJumpLeft, _) => {
            MotionType::KeeperJumpLeft
        }
        (_, true, MotionType::WideStance, _) => MotionType::WideStance,
        (_, true, MotionType::KeeperJumpRight, _) => MotionType::KeeperJumpRight,
        (_, true, MotionType::KeeperJumpLeft, _) => MotionType::KeeperJumpLeft,
        (_, _, MotionType::CenterJump, _) => MotionType::CenterJump,
        (MotionType::ArmsUpSquat, _, MotionType::JumpRight, _) => MotionType::JumpRight,
        (MotionType::ArmsUpSquat, _, MotionType::JumpLeft, _) => MotionType::JumpLeft,
        (MotionType::ArmsUpStand, _, _, false) => MotionType::ArmsUpStand,
        (MotionType::Dispatching, true, _, _) => to,
        (MotionType::Stand, _, MotionType::Walk, _) => MotionType::Walk,
        (MotionType::Walk, _, MotionType::Stand, _) => MotionType::Stand,
        (MotionType::Unstiff | MotionType::AnimationStiff, true, MotionType::Animation, _) => {
            MotionType::Animation
        }
        (MotionType::Animation, true, MotionType::AnimationStiff, _) => MotionType::AnimationStiff,
        (from, true, to, _) if from != to => MotionType::Dispatching,
        _ => from,
    }
}

fn stand_up_counting(
    last_motion: MotionType,
    current_motion: MotionType,
    stand_up_count: u32,
) -> u32 {
    if !last_motion.is_standup_motion() && current_motion.is_standup_motion() {
        return stand_up_count + 1;
    }

    if current_motion.is_stable() {
        return 0;
    }

    stand_up_count
}
