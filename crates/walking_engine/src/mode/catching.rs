use super::{walking::Walking, Mode, WalkTransition};
use coordinate_systems::Walk;
use geometry::polygon::is_inside_convex_hull;
use linear_algebra::{vector, IntoFramed, Orientation2, Point2, Pose2};
use nalgebra::SimdPartialOrd;
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};
use types::{
    joints::body::BodyJoints,
    motion_command::KickVariant,
    motor_commands::MotorCommands,
    robot_dimensions::{transform_left_sole_outline, transform_right_sole_outline},
    step::Step,
    support_foot::Side,
};

use crate::{
    anatomic_constraints::clamp_feet_to_anatomic_constraints, feet::Feet, step_plan::StepPlan,
    step_state::StepState, stiffness::Stiffness as _, Context,
};

#[derive(
    Clone, Copy, Debug, Serialize, Deserialize, PathSerialize, PathDeserialize, PathIntrospect,
)]
pub struct Catching {
    pub step: StepState,
}

impl Catching {
    pub fn new(context: &Context, last_step_state: StepState, support_side: Side) -> Self {
        let Some(robot_to_ground) = context.robot_to_ground else {
            return Self {
                step: last_step_state,
            };
        };

        let parameters = context.parameters;
        let robot_to_walk = context.robot_to_walk;
        let ground_to_robot = robot_to_ground.inverse();

        let mut target =
            (robot_to_walk * ground_to_robot * context.zero_moment_point.extend(0.0)).xy();

        let current_feet =
            Feet::from_joints(robot_to_walk, &context.last_actuated_joints, support_side);
        let support_outline: Vec<_> = if support_side == Side::Left {
            transform_left_sole_outline(current_feet.support_sole.as_transform())
                .map(|point| point.xy())
                .collect()
        } else {
            transform_right_sole_outline(current_feet.support_sole.as_transform())
                .map(|point| point.xy())
                .collect()
        };
        if target.x().abs() < context.parameters.catching_steps.balance_region_x
            && ((support_side == Side::Left
                && support_outline.iter().all(|point| point.y() < target.y()))
                || (support_side == Side::Right
                    && support_outline.iter().all(|point| point.y() > target.y())))
        {
            target.inner.y = -target.y();
        }

        let clamped_target = target
            .inner
            .coords
            .simd_clamp(
                -context.parameters.catching_steps.max_target_distance,
                context.parameters.catching_steps.max_target_distance,
            )
            .framed()
            .as_point();

        let target_projection_into_foot_support = context
            .parameters
            .foot_support
            .project_point_into_rect(clamped_target);
        let displacement = clamped_target - target_projection_into_foot_support;

        let (support_base_offset, swing_base_offset) = match support_side {
            Side::Left => (
                parameters.base.foot_offset_left,
                parameters.base.foot_offset_right,
            ),
            Side::Right => (
                parameters.base.foot_offset_right,
                parameters.base.foot_offset_left,
            ),
        };

        let desired_end_feet = Feet {
            support_sole: Pose2::from_parts(
                support_base_offset.xy().as_point()
                    - displacement.component_mul(&parameters.catching_steps.over_estimation_factor)
                        * 0.5,
                Orientation2::default(),
            ),
            swing_sole: Pose2::from_parts(
                swing_base_offset.xy().as_point()
                    + displacement.component_mul(&parameters.catching_steps.over_estimation_factor),
                Orientation2::default(),
            ),
        };

        let clamped_feet =
            clamp_feet_to_anatomic_constraints(desired_end_feet, support_side, context.parameters);

        let start_feet = last_step_state.plan.start_feet;
        let plan = StepPlan::new_with_start_and_end_feet(
            context,
            support_side,
            start_feet,
            clamped_feet.at_ground(),
        );

        Self {
            step: StepState {
                plan,
                ..last_step_state
            },
        }
    }

    pub fn new_from_catching(
        self,
        context: &Context,
        last_step_state: StepState,
        support_side: Side,
    ) -> Self {
        let new_catching = Catching::new(context, last_step_state, support_side);

        let old_norm = self
            .step
            .plan
            .end_feet
            .swing_sole
            .position()
            .xy()
            .coords()
            .norm();
        let new_norm = new_catching
            .step
            .plan
            .end_feet
            .swing_sole
            .position()
            .xy()
            .coords()
            .norm();

        if new_norm > old_norm {
            new_catching
        } else {
            self
        }
    }
}

impl WalkTransition for Catching {
    fn stand(self, context: &Context) -> Mode {
        let current_step = self.step;

        if current_step.is_support_switched(context) {
            return Mode::Walking(Walking::new(
                context,
                Step::ZERO,
                current_step.plan.support_side.opposite(),
                Step::ZERO,
            ));
        }

        if should_catch(
            context,
            current_step.plan.end_feet,
            current_step.plan.support_side,
        ) {
            return Mode::Catching(Catching::new_from_catching(
                self,
                context,
                self.step,
                self.step.plan.support_side,
            ));
        }

        Mode::Catching(self)
    }

    fn walk(self, context: &Context, _requested_step: Step) -> Mode {
        let current_step = self.step;
        let should_catch_now = should_catch(
            context,
            current_step.plan.end_feet,
            current_step.plan.support_side,
        );

        if current_step.is_support_switched(context) {
            let executed_step = self
                .step
                .plan
                .end_feet
                .to_step(context.parameters, self.step.plan.support_side);

            return Mode::Walking(Walking::new(
                context,
                Step::ZERO,
                self.step.plan.support_side.opposite(),
                executed_step,
            ));
        }

        if should_catch_now {
            return Mode::Catching(Catching::new_from_catching(
                self,
                context,
                self.step,
                self.step.plan.support_side,
            ));
        }

        Mode::Catching(self)
    }

    fn kick(
        self,
        context: &Context,
        _variant: KickVariant,
        _kicking_side: Side,
        _strength: f32,
    ) -> Mode {
        let current_step = self.step;

        if current_step.is_support_switched(context) {
            return Mode::Walking(Walking::new(
                context,
                Step::ZERO,
                current_step.plan.support_side.opposite(),
                Step::ZERO,
            ));
        }

        if should_catch(
            context,
            current_step.plan.end_feet,
            current_step.plan.support_side,
        ) {
            return Mode::Catching(Catching::new_from_catching(
                self,
                context,
                self.step,
                self.step.plan.support_side,
            ));
        }

        Mode::Catching(self)
    }
}

impl Catching {
    pub fn compute_commands(&mut self, context: &Context) -> MotorCommands<BodyJoints> {
        let feet = self.step.compute_feet(context);
        self.step.compute_joints(context, feet).apply_stiffness(
            context.parameters.stiffnesses.leg_stiffness_walk,
            context.parameters.stiffnesses.arm_stiffness,
        )
    }

    pub fn tick(&mut self, context: &Context) {
        self.step.tick(context);
    }
}

pub fn should_catch(context: &Context, end_feet: Feet, support_side: Side) -> bool {
    let catching_steps = &context.parameters.catching_steps;
    if !catching_steps.enabled {
        return false;
    }
    let Some(robot_to_ground) = context.robot_to_ground else {
        return false;
    };

    let ground_to_robot = robot_to_ground.inverse();
    let robot_to_walk = context.robot_to_walk;

    let current_feet =
        Feet::from_joints(robot_to_walk, &context.last_actuated_joints, support_side);

    let zmp = context.zero_moment_point;
    let target_scaling_x = if zmp.coords().x() < 0.0 {
        catching_steps.target_x_scale_backward
    } else {
        catching_steps.target_x_scale_forward
    };
    let target_scaling = vector![target_scaling_x, catching_steps.target_y_scale];

    let target = (robot_to_walk * ground_to_robot * zmp.extend(0.0))
        .xy()
        .coords()
        .component_mul(&target_scaling)
        .as_point();

    is_outside_support_polygon(end_feet, support_side, target, current_feet)
}

fn is_outside_support_polygon(
    end_feet: Feet,
    support_side: Side,
    target: Point2<Walk>,
    current_feet: Feet,
) -> bool {
    // the red swing foot
    let target_swing_sole = end_feet.swing_sole;

    let feet_outlines: Vec<_> = if support_side == Side::Left {
        transform_left_sole_outline(current_feet.support_sole.as_transform())
            .chain(transform_right_sole_outline(
                current_feet.swing_sole.as_transform(),
            ))
            .chain(transform_right_sole_outline(
                target_swing_sole.as_transform(),
            ))
            .map(|point| point.xy())
            .collect()
    } else {
        transform_right_sole_outline(current_feet.support_sole.as_transform())
            .chain(transform_left_sole_outline(
                current_feet.swing_sole.as_transform(),
            ))
            .chain(transform_left_sole_outline(
                target_swing_sole.as_transform(),
            ))
            .map(|point| point.xy())
            .collect()
    };

    !is_inside_convex_hull(&feet_outlines, &target)
}
