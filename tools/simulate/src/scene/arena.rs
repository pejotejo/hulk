//! Static arena decoration shared by the Bevy viewport and headless MuJoCo renderer.
use std::{f32::consts::PI, fmt::Write};

use bevy::{
    asset::RenderAssetUsages,
    image::{CompressedImageFormats, ImageSampler, ImageType},
    prelude::*,
};
use mujoco_rs::prelude::MjSpec;
use types::field_dimensions::FieldDimensions;

use crate::{parameters::CurrentSimulatorParameters, scene::SceneParameterUpdateSet};

const ASPHALT: usize = 0;
const CONCRETE: usize = 1;
const NAVY: usize = 2;
const TEAL: usize = 3;
const WHITE: usize = 4;
const STEEL: usize = 5;
const TURF: usize = 6;
const TURF_LIGHT: usize = 7;
const LAMP: usize = 8;
const COLORS: [[f32; 3]; 9] = [
    [0.055, 0.075, 0.095],
    [0.32, 0.37, 0.40],
    [0.035, 0.075, 0.14],
    [0.025, 0.47, 0.53],
    [0.88, 0.91, 0.92],
    [0.16, 0.21, 0.25],
    [0.09, 0.32, 0.17],
    [0.115, 0.365, 0.195],
    [0.93, 0.97, 1.0],
];
const BRANDS: [(&str, &[u8]); 8] = [
    (
        "hulks_panel.png",
        include_bytes!("../../assets/branding/hulks_panel.png"),
    ),
    (
        "kdab_panel.png",
        include_bytes!("../../assets/branding/kdab_panel.png"),
    ),
    (
        "cc_panel.png",
        include_bytes!("../../assets/branding/cc_panel.png"),
    ),
    (
        "draeger_panel.png",
        include_bytes!("../../assets/branding/draeger_panel.png"),
    ),
    (
        "gea_panel.png",
        include_bytes!("../../assets/branding/gea_panel.png"),
    ),
    (
        "haspa_panel.png",
        include_bytes!("../../assets/branding/haspa_panel.png"),
    ),
    (
        "tk_panel.png",
        include_bytes!("../../assets/branding/tk_panel.png"),
    ),
    (
        "gegenstrom_panel.png",
        include_bytes!("../../assets/branding/gegenstrom_panel.png"),
    ),
];
const PANEL_HEIGHT: f32 = 0.48;

struct Block {
    position: Vec3,
    size: Vec3,
    finish: usize,
}

struct Panel {
    position: Vec3,
    width: f32,
    yaw: f32,
    brand: usize,
}

#[derive(Default)]
struct Layout {
    blocks: Vec<Block>,
    panels: Vec<Panel>,
}

impl Layout {
    fn block(&mut self, position: Vec3, size: Vec3, finish: usize) {
        self.blocks.push(Block {
            position,
            size,
            finish,
        });
    }
}

fn layout(field: &FieldDimensions) -> Layout {
    let mut arena = Layout::default();
    // Keep all furniture beyond both the carpet runoff and the backs of the goals.
    let end = field.length / 2.0 + field.border_strip_width.max(field.goal_depth) + 0.55;
    let side = field.width / 2.0 + field.border_strip_width + 0.55;
    arena.block(
        Vec3::new(0.0, -0.10, 0.0),
        Vec3::new(2.0 * end + 2.4, 0.16, 2.0 * side + 5.0),
        ASPHALT,
    );
    arena.block(
        Vec3::new(0.0, -0.025, 0.0),
        Vec3::new(2.0 * end + 0.6, 0.04, 2.0 * side + 0.6),
        CONCRETE,
    );
    arena.block(
        Vec3::new(0.0, 0.00155, 0.0),
        Vec3::new(
            field.length + 2.0 * field.border_strip_width,
            0.0001,
            field.width + 2.0 * field.border_strip_width,
        ),
        TURF,
    );
    for index in (0..12).step_by(2) {
        arena.block(
            Vec3::new(
                -field.length / 2.0 + (index as f32 + 0.5) * field.length / 12.0,
                0.0017,
                0.0,
            ),
            Vec3::new(field.length / 12.0, 0.0001, field.width),
            TURF_LIGHT,
        );
    }

    for sign in [-1.0, 1.0] {
        arena.block(
            Vec3::new(0.0, 0.31, sign * side),
            Vec3::new(end * 2.0, 0.62, 0.12),
            NAVY,
        );
        arena.block(
            Vec3::new(0.0, 0.64, sign * side),
            Vec3::new(end * 2.0 + 0.12, 0.045, 0.17),
            TEAL,
        );
        arena.block(
            Vec3::new(sign * end, 0.31, 0.0),
            Vec3::new(0.12, 0.62, side * 2.0),
            NAVY,
        );
        arena.block(
            Vec3::new(sign * end, 0.64, 0.0),
            Vec3::new(0.17, 0.045, side * 2.0),
            TEAL,
        );

        let long_count = ((2.0 * end / 1.7).round() as usize).max(8);
        for index in 0..long_count {
            let spacing = 2.0 * end / long_count as f32;
            arena.panels.push(Panel {
                position: Vec3::new(
                    -end + (index as f32 + 0.5) * spacing,
                    0.33,
                    sign * (side - 0.066),
                ),
                width: (spacing - 0.07).min(PANEL_HEIGHT * 3.0),
                yaw: if sign < 0.0 { 0.0 } else { PI },
                brand: index % BRANDS.len(),
            });
        }
        let end_count = ((2.0 * side / 1.7).round() as usize).max(6);
        for index in 0..end_count {
            let spacing = 2.0 * side / end_count as f32;
            arena.panels.push(Panel {
                position: Vec3::new(
                    sign * (end - 0.066),
                    0.33,
                    -side + (index as f32 + 0.5) * spacing,
                ),
                width: (spacing - 0.07).min(PANEL_HEIGHT * 3.0),
                yaw: -sign * PI / 2.0,
                brand: (index + 3) % BRANDS.len(),
            });
        }

        // Three low rows leave the match visible from the default elevated camera.
        for row in 0..3 {
            let z = sign * (side + 0.75 + row as f32 * 0.52);
            let height = 0.18 + row as f32 * 0.20;
            arena.block(
                Vec3::new(0.0, height / 2.0, z),
                Vec3::new(end * 1.72, height, 0.52),
                CONCRETE,
            );
            let seats = ((end * 1.72 / 0.48).floor() as usize).max(4);
            for seat in 0..seats {
                if seat == seats / 2 || seat == seats / 2 - 1 {
                    continue;
                }
                let x = -end * 0.86 + (seat as f32 + 0.5) * end * 1.72 / seats as f32;
                let finish = match (seat / 4 + row) % 5 {
                    0 => WHITE,
                    1 | 2 => TEAL,
                    _ => NAVY,
                };
                arena.block(
                    Vec3::new(x, height + 0.05, z),
                    Vec3::new(0.36, 0.09, 0.34),
                    finish,
                );
                arena.block(
                    Vec3::new(x, height + 0.22, z + sign * 0.14),
                    Vec3::new(0.36, 0.30, 0.06),
                    finish,
                );
            }
        }
        for x_sign in [-1.0, 1.0] {
            let x = x_sign * (end + 0.40);
            let z = sign * (side + 0.15);
            arena.block(Vec3::new(x, 1.6, z), Vec3::new(0.07, 3.2, 0.07), STEEL);
            arena.block(Vec3::new(x, 3.2, z), Vec3::new(0.64, 0.12, 0.28), NAVY);
            arena.block(Vec3::new(x, 3.13, z), Vec3::new(0.56, 0.025, 0.22), LAMP);
        }
    }
    arena
}

#[derive(Resource)]
struct ArenaAssets {
    cube: Handle<Mesh>,
    panel: Handle<Mesh>,
    surfaces: Vec<Handle<StandardMaterial>>,
    brands: Vec<Handle<StandardMaterial>>,
}

impl FromWorld for ArenaAssets {
    fn from_world(world: &mut World) -> Self {
        let cube = world.resource_mut::<Assets<Mesh>>().add(Cuboid::default());
        let panel = world
            .resource_mut::<Assets<Mesh>>()
            .add(Rectangle::default());
        let surfaces = COLORS
            .iter()
            .enumerate()
            .map(|(index, color)| {
                world
                    .resource_mut::<Assets<StandardMaterial>>()
                    .add(StandardMaterial {
                        base_color: Color::srgb(color[0], color[1], color[2]),
                        perceptual_roughness: 0.9,
                        unlit: index == LAMP,
                        ..default()
                    })
            })
            .collect();
        let brands = BRANDS
            .iter()
            .map(|(_, bytes)| {
                let image = Image::from_buffer(
                    bytes,
                    ImageType::Extension("png"),
                    CompressedImageFormats::NONE,
                    true,
                    ImageSampler::default(),
                    RenderAssetUsages::default(),
                )
                .expect("decode bundled arena branding");
                let texture = world.resource_mut::<Assets<Image>>().add(image);
                world
                    .resource_mut::<Assets<StandardMaterial>>()
                    .add(StandardMaterial {
                        base_color_texture: Some(texture),
                        perceptual_roughness: 0.9,
                        ..default()
                    })
            })
            .collect();
        Self {
            cube,
            panel,
            surfaces,
            brands,
        }
    }
}

#[derive(Component)]
struct Arena;

#[derive(Default, Resource)]
struct ArenaDimensions(Option<[f32; 4]>);

pub(super) struct ArenaPlugin;

impl Plugin for ArenaPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ArenaAssets>()
            .init_resource::<ArenaDimensions>()
            .insert_resource(ClearColor(Color::srgb(0.055, 0.075, 0.105)))
            .add_systems(PreUpdate, update_arena.in_set(SceneParameterUpdateSet));
    }
}

fn update_arena(
    mut commands: Commands,
    parameters: Res<CurrentSimulatorParameters>,
    assets: Res<ArenaAssets>,
    mut dimensions: ResMut<ArenaDimensions>,
    roots: Query<Entity, With<Arena>>,
    mut lights: Query<&mut DirectionalLight>,
) {
    let field = &parameters.field_dimensions;
    let current = [
        field.length,
        field.width,
        field.border_strip_width,
        field.goal_depth,
    ];
    if dimensions.0 == Some(current) {
        return;
    }
    dimensions.0 = Some(current);
    for mut light in &mut lights {
        light.illuminance = 15_000.0;
        light.color = Color::srgb(1.0, 0.96, 0.90);
    }
    for root in &roots {
        commands.entity(root).despawn();
    }
    let arena = layout(field);
    commands
        .spawn((Arena, Transform::default(), Visibility::default()))
        .with_children(|parent| {
            for block in arena.blocks {
                parent.spawn((
                    Mesh3d(assets.cube.clone()),
                    MeshMaterial3d(assets.surfaces[block.finish].clone()),
                    Transform::from_translation(block.position).with_scale(block.size),
                ));
            }
            for panel in arena.panels {
                parent.spawn((
                    Mesh3d(assets.panel.clone()),
                    MeshMaterial3d(assets.brands[panel.brand].clone()),
                    Transform::from_translation(panel.position)
                        .with_rotation(Quat::from_rotation_y(panel.yaw))
                        .with_scale(Vec3::new(panel.width, panel.width / 3.0, 1.0)),
                ));
            }
        });
}

/// Non-colliding decoration, including turf; attach the returned `arena` root body.
pub(crate) fn arena_spec(field: FieldDimensions) -> Result<MjSpec, String> {
    MjSpec::from_xml_string(&arena_xml(&field)).map_err(|error| error.to_string())
}

fn arena_xml(field: &FieldDimensions) -> String {
    let arena = layout(field);
    let mut meshes: Vec<ArenaMesh> = (0..COLORS.len() + BRANDS.len())
        .map(|_| ArenaMesh::default())
        .collect();
    for block in arena.blocks {
        meshes[block.finish].add_box(block.position, block.size, 0.0);
    }
    for panel in arena.panels {
        meshes[COLORS.len() + panel.brand].add_box(
            panel.position,
            Vec3::new(panel.width, panel.width / 3.0, 0.004),
            panel.yaw,
        );
    }
    let mut xml = String::from("<mujoco><asset>");
    let directory = concat!(env!("CARGO_MANIFEST_DIR"), "/assets/branding/");
    for (index, (file, _)) in BRANDS.iter().enumerate() {
        let path = format!("{directory}{file}")
            .replace('&', "&amp;")
            .replace('"', "&quot;");
        write!(xml, "<texture name=\"brand_{index}\" type=\"2d\" file=\"{path}\"/><material name=\"brand_{index}\" texture=\"brand_{index}\" texuniform=\"false\" specular=\"0\" shininess=\"0\"/>").unwrap();
    }
    // Merge static boxes by material so the headless scene needs only 17 arena geoms.
    for (index, mesh) in meshes.iter().enumerate() {
        write!(xml, "<mesh name=\"part_{index}\" vertex=\"").unwrap();
        for [x, y, z] in &mesh.vertices {
            write!(xml, "{x} {y} {z} ").unwrap();
        }
        xml.push_str("\" face=\"");
        for [a, b, c] in &mesh.faces {
            write!(xml, "{a} {b} {c} ").unwrap();
        }
        xml.push_str("\" texcoord=\"");
        for _ in 0..mesh.vertices.len() / 8 {
            xml.push_str("0 1 1 1 1 0 0 0 1 1 0 1 0 0 1 0 ");
        }
        xml.push_str("\"/>");
    }
    xml.push_str("</asset><worldbody><body name=\"arena\">");
    for index in 0..meshes.len() {
        let finish = if index < COLORS.len() {
            let [r, g, b] = COLORS[index];
            format!("rgba=\"{r} {g} {b} 1\"")
        } else {
            format!("material=\"brand_{}\"", index - COLORS.len())
        };
        write!(
            xml,
            "<geom type=\"mesh\" mesh=\"part_{index}\" {finish} contype=\"0\" conaffinity=\"0\"/>"
        )
        .unwrap();
    }
    xml.push_str("</body></worldbody></mujoco>");
    xml
}

#[derive(Default)]
struct ArenaMesh {
    vertices: Vec<[f32; 3]>,
    faces: Vec<[usize; 3]>,
}

impl ArenaMesh {
    fn add_box(&mut self, position: Vec3, size: Vec3, yaw: f32) {
        let base = self.vertices.len();
        let (sin, cos) = yaw.sin_cos();
        for [x, y, z] in [
            [-0.5, -0.5, -0.5],
            [0.5, -0.5, -0.5],
            [0.5, -0.5, 0.5],
            [-0.5, -0.5, 0.5],
            [-0.5, 0.5, -0.5],
            [0.5, 0.5, -0.5],
            [0.5, 0.5, 0.5],
            [-0.5, 0.5, 0.5],
        ] {
            // Bevy is Y-up; MuJoCo uses field coordinates (X, -Z, Y).
            let (x, y, z) = (x * size.x, y * size.z, z * size.y);
            self.vertices.push([
                position.x + cos * x - sin * y,
                -position.z + sin * x + cos * y,
                position.y + z,
            ]);
        }
        for face in [
            [0, 1, 2],
            [0, 2, 3],
            [5, 4, 7],
            [5, 7, 6],
            [4, 0, 3],
            [4, 3, 7],
            [1, 5, 6],
            [1, 6, 2],
            [3, 2, 6],
            [3, 6, 7],
            [4, 5, 1],
            [4, 1, 0],
        ] {
            self.faces.push(face.map(|index| base + index));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn furniture_stays_outside_runoff_and_goal_depth() {
        for length in [9.0, 14.0, 22.0] {
            let field = FieldDimensions {
                length,
                width: length * 2.0 / 3.0,
                ..FieldDimensions::SPL_2025
            };
            let arena = layout(&field);
            let x = field.length / 2.0 + field.border_strip_width.max(field.goal_depth);
            let z = field.width / 2.0 + field.border_strip_width;
            for block in arena.blocks.iter().filter(|block| block.position.y > 0.01) {
                assert!(
                    block.position.x.abs() - block.size.x / 2.0 > x
                        || block.position.z.abs() - block.size.z / 2.0 > z
                );
            }
            assert!(arena.panels.iter().all(|panel| panel.width > 0.0));
            for panel in &arena.panels {
                let normal = Quat::from_rotation_y(panel.yaw) * Vec3::Z;
                assert!(normal.dot(-panel.position) > 0.0);
            }
            for brand in 0..BRANDS.len() {
                assert!(arena.panels.iter().any(|panel| panel.brand == brand));
            }
        }
    }

    #[test]
    fn headless_arena_compiles_without_contacts_or_dofs() {
        let model = arena_spec(FieldDimensions::SPL_2025)
            .unwrap()
            .compile()
            .unwrap();
        assert!(model.geom_contype().iter().all(|value| *value == 0));
        assert!(model.geom_conaffinity().iter().all(|value| *value == 0));
        assert_eq!(model.nv(), 0);
        assert_eq!(model.ngeom() as usize, COLORS.len() + BRANDS.len());
    }
}
