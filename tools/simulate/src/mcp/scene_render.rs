//! Renderer-owned model: advertising and lighting never mutate the live physics model.
use color_eyre::{Result, eyre::ensure};
use mujoco_rs::prelude::{MjData, MjModel, MjtGeom, MjtObj};

use crate::{match_effects, scene::arena};

pub(super) struct VisualSnapshot {
    pub data: MjData<Box<MjModel>>,
    boards: Vec<(usize, usize)>,
    materials: [i32; arena::BRAND_COUNT],
}

impl VisualSnapshot {
    pub fn new(model: &MjModel, width: u32, height: u32) -> Result<Self> {
        let mut model = model.try_clone()?;
        model.vis_mut().headlight.ambient = [0.4; 3];
        model.vis_mut().headlight.diffuse = [0.35; 3];
        model.vis_mut().global.offwidth = width as i32;
        model.vis_mut().global.offheight = height as i32;
        // MuJoCo scales clipping by scene extent; the distant harbor must not clip close shots.
        model.vis_mut().map.znear = (0.03 / model.stat().extent.max(0.01)) as f32;
        if let Some(ground) = model.name_to_id(MjtObj::mjOBJ_GEOM, "ground") {
            // The physical infinite plane would cover the decorative lowered waterfront.
            let colors = model.geom_rgba_mut();
            colors[ground][3] = 0.0;
        }
        let mut materials = [-1; arena::BRAND_COUNT];
        let mut boards = Vec::with_capacity(arena::BRAND_COUNT);
        for (initial, material) in materials.iter_mut().enumerate() {
            let name = format!("brand_{initial}");
            *material = (0..model.nmat() as usize)
                .find(|id| {
                    model
                        .id_to_name(MjtObj::mjOBJ_MATERIAL, *id)
                        .is_some_and(|value| value.ends_with(&name))
                })
                .map_or(-1, |id| id as i32);
            let name = format!("arena_brand_{initial}");
            if let Some(id) = (0..model.ngeom() as usize).find(|id| {
                model
                    .id_to_name(MjtObj::mjOBJ_GEOM, *id)
                    .is_some_and(|value| value.ends_with(&name))
            }) {
                boards.push((id, initial));
            }
        }
        ensure!(
            materials.iter().all(|id| *id >= 0) && boards.len() == arena::BRAND_COUNT,
            "Arena render materials are incomplete"
        );
        Ok(Self {
            data: MjData::new(Box::new(model)),
            boards,
            materials,
        })
    }

    pub fn animate(&mut self, time: f64) {
        // SAFETY: only material indices in an independently owned visual model change.
        // Array dimensions/signature remain unchanged; this data is never stepped.
        let model = unsafe { self.data.model_mut() };
        let ids = unsafe { model.geom_matid_mut() };
        for &(geom, initial) in &self.boards {
            ids[geom] = self.materials[arena::brand_index(initial, time)];
        }
    }
}

pub(super) fn confetti(
    renderer: &mut mujoco_rs::renderer::MjRenderer,
    effects: &match_effects::MatchEffects,
    time: f64,
) {
    let scene = renderer.user_scene_mut();
    scene.clear_geom();
    if let Some(burst) = effects.burst(time) {
        for (pose, color) in match_effects::particles(burst, time) {
            let p = pose.translation;
            let basis = bevy::prelude::Mat3::from_quat(pose.rotation);
            let to_field =
                |v: bevy::prelude::Vec3| [f64::from(v.x), -f64::from(v.z), f64::from(v.y)];
            let x = to_field(basis.x_axis);
            let y = to_field(basis.y_axis);
            let z = to_field(basis.z_axis);
            let geom = scene.create_geom(
                MjtGeom::mjGEOM_BOX,
                Some([0.0375, 0.0175, 0.0005]),
                Some(to_field(p)),
                Some([x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]]),
                Some(match_effects::CONFETTI_COLORS[color]),
            );
            geom.emission = 0.6;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn render_material_and_light_changes_leave_live_model_and_state_untouched() {
        let mut spec =
            arena::arena_spec(types::field_dimensions::FieldDimensions::SPL_2025).unwrap();
        let body = spec.world_body_mut().add_body().with_pos([0.0, 0.0, 1.0]);
        body.add_joint()
            .with_type(mujoco_rs::prelude::MjtJoint::mjJNT_FREE);
        body.add_geom()
            .with_type(MjtGeom::mjGEOM_SPHERE)
            .with_size([0.1, 0.0, 0.0]);
        let model = spec.compile().unwrap();
        let mut live = MjData::new(Box::new(model));
        live.forward();
        live.qvel_mut()[0] = 0.25;
        let materials = live.model().geom_matid().to_vec();
        let contacts = live.model().geom_contype().to_vec();
        let lights = live.model_vis().headlight.diffuse;
        let qpos = live.qpos().to_vec();
        let qvel = live.qvel().to_vec();
        let mut visual = VisualSnapshot::new(live.model(), 800, 600).unwrap();
        live.copy_visual_to(&mut visual.data).unwrap();
        visual.animate(12.0);
        assert_ne!(visual.data.model().geom_matid(), materials);
        assert_eq!(live.model().geom_matid(), materials);
        assert_eq!(live.model().geom_contype(), contacts);
        assert_eq!(live.model_vis().headlight.diffuse, lights);
        assert_eq!(live.qpos(), qpos);
        assert_eq!(live.qvel(), qvel);
        assert_eq!(live.time(), 0.0);
        assert_eq!(live.model().signature(), visual.data.model().signature());
        assert!(
            (f64::from(visual.data.model_vis().map.znear) * visual.data.model().stat().extent
                - 0.03)
                .abs()
                < 1e-6
        );
    }
}
